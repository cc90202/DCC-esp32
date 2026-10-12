//! Central fault and e-stop state manager.
//!
//! The pure `policy` module drives all transitions. The firmware fault-manager
//! task applies GPIO and state-watch changes immediately, while a separate
//! relay task forwards scheduler and presentation effects without
//! backpressuring the safety loop.
//!
//! The fault manager is a state machine with three states. In `Normal` the
//! track is powered, motion commands are accepted and the track driver is
//! enabled. `EstopLatched` is reached when the operator presses the stop
//! button, and a resume press leaves it. `FaultLatched` is reached on a
//! hardware fault such as a track short or a CV error, and only a long resume
//! press or a service action clears it. In both latched states the track driver
//! is disabled.
//!
//! A fault can be latched from either of the other two states, and it replaces
//! a pending e-stop rather than queueing behind it. A short resume press while
//! a fault is latched is deliberately ignored, so an operator cannot clear a
//! hardware fault by reflex.
//!
//! The runtime state is exposed as [`crate::system_status::FaultManagerState`]
//! for observers. State transitions and their required effects remain private
//! to the pure policy so callers cannot bypass the fault manager's application
//! rules.

mod policy;
#[cfg(any(test, target_arch = "riscv32"))]
mod power_fence;
#[cfg(any(test, target_arch = "riscv32"))]
mod relay;

#[cfg(target_arch = "riscv32")]
pub(crate) use relay::{FaultEffectsSignal, FaultEffectsTaskContext, fault_effects_task};

#[cfg(target_arch = "riscv32")]
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal, watch};
#[cfg(target_arch = "riscv32")]
use embassy_time::{Duration, Instant, with_timeout};

#[cfg(target_arch = "riscv32")]
use crate::dcc::PowerGeneration;
#[cfg(target_arch = "riscv32")]
use crate::runtime_channels::{
    BootReadySender, FaultEventReceiver, RuntimeReceiver, RuntimeSender, announce_ready,
};
#[cfg(target_arch = "riscv32")]
use crate::system_status::{BootReadyEvent, FaultEvent, FaultManagerState};
#[cfg(target_arch = "riscv32")]
use crate::track_output::TrackOutput;
#[cfg(target_arch = "riscv32")]
use policy::FaultPolicy;
#[cfg(target_arch = "riscv32")]
use relay::FaultEffectsSnapshot;

#[cfg(target_arch = "riscv32")]
pub type FaultStateWatch =
    watch::Watch<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, FaultManagerState, 1>;

#[cfg(target_arch = "riscv32")]
pub(crate) struct FaultManagerTaskContext {
    pub(crate) receiver: FaultEventReceiver,
    pub(crate) power_quiesce_sender: RuntimeSender<PowerGeneration, 1>,
    pub(crate) power_fence_ack_receiver: RuntimeReceiver<PowerGeneration, 1>,
    pub(crate) track_output: TrackOutput,
    pub(crate) state_sender: watch::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        FaultManagerState,
        1,
    >,
    pub(crate) effects_signal: &'static FaultEffectsSignal,
    pub(crate) ready_sender: BootReadySender,
}

/// Longest wait for the engine to drain the frames queued before a power
/// transition.
#[cfg(target_arch = "riscv32")]
const FENCE_TIMEOUT: Duration = Duration::from_millis(500);

/// Longest wait for an event before the loop reports a heartbeat.
#[cfg(target_arch = "riscv32")]
const HEARTBEAT_PERIOD: Duration = Duration::from_millis(500);

/// Power-on requests refused while the update interlock is engaged, before
/// any policy or scheduler effect. Arming in a latched fault state still
/// completes, without powering on.
#[cfg(target_arch = "riscv32")]
fn refused_while_inhibited(event: FaultEvent, state: FaultManagerState) -> bool {
    crate::track_output::inhibited()
        && (matches!(
            event,
            FaultEvent::ResumeShortPressed
                | FaultEvent::ResumeLongPressed
                | FaultEvent::NetworkResume(_)
                | FaultEvent::FaultClearedByService
        ) || (event == FaultEvent::TrackPowerArmed && state == FaultManagerState::Normal))
}

/// Signalled once a stop requested while the track interlock is engaged has
/// taken effect and every frame queued before it has left the engine.
#[cfg(target_arch = "riscv32")]
pub(crate) static INTERLOCK_STOP_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signalled after every `TrackPowerArmed` the fault manager applied.
///
/// Like [`INTERLOCK_STOP_DONE`], a `Signal` wakes one waiter only. One wait
/// at a time is guaranteed by the update lifecycle: a confirmation waits
/// only during a trial, a boot arming only before the track is armed, an
/// update or setup only while it holds the update lock.
#[cfg(target_arch = "riscv32")]
pub(crate) static TRACK_POWER_ARMED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// A stop while the interlock is engaged: whoever engaged it waits for the
/// stop to be fenced, so it needs the fence and [`INTERLOCK_STOP_DONE`].
#[cfg(target_arch = "riscv32")]
fn is_interlock_stop(event: FaultEvent) -> bool {
    event == FaultEvent::StopPressed && crate::track_output::inhibited()
}

/// Applies the decided track power after a completed fence; a network
/// resume also needs its lease to be current. Returns whether it applied.
#[cfg(target_arch = "riscv32")]
fn apply_track_power(event: FaultEvent, enabled: bool, track_output: &mut TrackOutput) -> bool {
    let FaultEvent::NetworkResume(permit) = event else {
        return track_output.set_track_enabled(enabled);
    };
    crate::track_authority::apply_if_current(permit, Instant::now().as_millis(), || {
        track_output.set_track_enabled(enabled)
    }) == Some(true)
}

/// Tells whoever waits that a stop or an arming took effect. Returns true
/// for an interlock stop, whose effects were already published.
#[cfg(target_arch = "riscv32")]
fn signal_completion(event: FaultEvent, interlock_stop: bool) -> bool {
    if interlock_stop {
        INTERLOCK_STOP_DONE.signal(());
    } else if event == FaultEvent::TrackPowerArmed {
        TRACK_POWER_ARMED.signal(());
    }
    interlock_stop
}

/// Waits until every frame queued before `generation` has crossed the
/// engine fence. `Err(Some(event))` returns a stop or fault that arrived
/// meanwhile; `Err(None)` is a timeout.
#[cfg(target_arch = "riscv32")]
async fn wait_for_fence(
    generation: PowerGeneration,
    receiver: &FaultEventReceiver,
    quiesce_sender: &RuntimeSender<PowerGeneration, 1>,
    ack_receiver: &RuntimeReceiver<PowerGeneration, 1>,
) -> Result<(), Option<FaultEvent>> {
    match with_timeout(
        FENCE_TIMEOUT,
        power_fence::request_and_wait(generation, receiver, quiesce_sender, ack_receiver),
    )
    .await
    {
        Ok(Ok(())) => Ok(()),
        Ok(Err(interrupting_event)) => Err(Some(interrupting_event)),
        Err(_) => Err(None),
    }
}

/// Applies a decision to the effects snapshot and hands it to the relay.
#[cfg(target_arch = "riscv32")]
fn publish_effects(
    effects: &mut FaultEffectsSnapshot,
    decision: policy::FaultDecision,
    signal: &FaultEffectsSignal,
) {
    effects.apply(decision);
    signal.signal(*effects);
}

#[cfg(target_arch = "riscv32")]
fn log_decision(decision: &policy::FaultDecision) {
    if decision.state != decision.previous_state {
        defmt::warn!(
            "fault_manager: state {:?} -> {:?}, track_enabled={}, motion_enabled={}",
            decision.previous_state,
            decision.state,
            decision.track_enabled,
            decision.motion_enabled
        );
    } else {
        defmt::info!(
            "fault_manager: state unchanged {:?}, track_enabled={}, motion_enabled={}",
            decision.state,
            decision.track_enabled,
            decision.motion_enabled
        );
    }
}

/// Applies pure fault-policy decisions to GPIO and non-blocking state outputs.
#[cfg(target_arch = "riscv32")]
#[embassy_executor::task]
pub(crate) async fn fault_manager_task(context: FaultManagerTaskContext) -> ! {
    let FaultManagerTaskContext {
        receiver,
        power_quiesce_sender,
        power_fence_ack_receiver,
        mut track_output,
        state_sender,
        effects_signal,
        ready_sender,
    } = context;

    let mut policy = FaultPolicy::new();
    let mut effects = FaultEffectsSnapshot::new();
    let mut next_power_generation = 1u32;
    if !track_output.set_track_enabled(policy.track_enabled()) {
        defmt::info!("fault_manager: initial track power held off by the update interlock");
    }
    state_sender.send(policy.state());
    effects_signal.signal(effects);
    announce_ready(ready_sender, BootReadyEvent::FaultManager).await;

    let mut pending_event = None;
    loop {
        let event = match pending_event.take() {
            Some(event) => event,
            // Wake up periodically so the heartbeat shows the loop is alive.
            None => match with_timeout(HEARTBEAT_PERIOD, receiver.receive()).await {
                Ok(event) => event,
                Err(_) => {
                    crate::liveness::beat(crate::liveness::Task::Fault);
                    continue;
                }
            },
        };
        crate::liveness::beat(crate::liveness::Task::Fault);
        if refused_while_inhibited(event, policy.state()) {
            defmt::info!(
                "fault_manager: {:?} refused while the track is inhibited",
                event
            );
            continue;
        }
        defmt::info!(
            "fault_manager: event={:?} state_before={:?}",
            event,
            policy.state()
        );
        let mut candidate_policy = policy;
        let decision = candidate_policy.handle(event);
        let interlock_stop = is_interlock_stop(event);
        // The stop policy takes effect even if the drain below is interrupted
        // by a higher-priority fault or times out; the bridge goes off at the
        // start of the fence. Only the interlock acknowledgement needs the fence.
        if interlock_stop {
            policy = candidate_policy;
            state_sender.send(decision.state);
            publish_effects(&mut effects, decision, effects_signal);
        }
        let requires_fence =
            interlock_stop || (decision.track_enabled && !track_output.is_track_enabled());
        let fence_complete = if requires_fence {
            // Keep the physical bridge low until all frames queued before this
            // transition have crossed the engine fence.
            track_output.disable();
            let generation = PowerGeneration::new(next_power_generation);
            next_power_generation = next_power_generation.wrapping_add(1).max(1);
            let fenced = wait_for_fence(
                generation,
                &receiver,
                &power_quiesce_sender,
                &power_fence_ack_receiver,
            )
            .await;
            // A concurrent stop or fault takes precedence over a pending
            // power-on: keep the bridge low and process it next.
            if let Err(Some(interrupting_event)) = fenced {
                pending_event = Some(interrupting_event);
            }
            fenced.is_ok()
        } else {
            true
        };
        let applied =
            fence_complete && apply_track_power(event, decision.track_enabled, &mut track_output);
        if !applied {
            defmt::warn!("fault_manager: power transition interrupted, timed out or unauthorized");
            continue;
        }
        policy = candidate_policy;
        if signal_completion(event, interlock_stop) {
            continue;
        }

        if decision.state != decision.previous_state {
            state_sender.send(decision.state);
        }
        log_decision(&decision);
        publish_effects(&mut effects, decision, effects_signal);
    }
}
