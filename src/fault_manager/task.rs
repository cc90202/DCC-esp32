//! Firmware fault-manager task: performs the side effects that
//! [`FaultLoop::step`] decides, in a fixed order, for each fault event.

use embassy_time::{Duration, Instant, with_timeout};

use super::policy::FaultDecision;
use super::power_fence;
use super::relay::{FaultEffectsSignal, FaultEffectsSnapshot};
use super::step::FaultLoop;
use super::{FaultStateSender, INTERLOCK_STOP_DONE, TRACK_POWER_ARMED};
use crate::dcc::PowerGeneration;
use crate::dcc::channels::{PowerFenceAckReceiver, PowerQuiesceSender};
use crate::system_status::channels::{BootReadySender, FaultEventReceiver};
use crate::system_status::{BootReadyEvent, FaultEvent, FaultManagerState, announce_ready};
use crate::track_output::TrackOutput;

pub(crate) struct FaultManagerTaskContext {
    pub(crate) receiver: FaultEventReceiver,
    pub(crate) power_quiesce_sender: PowerQuiesceSender,
    pub(crate) power_fence_ack_receiver: PowerFenceAckReceiver,
    pub(crate) track_output: TrackOutput,
    pub(crate) state_sender: FaultStateSender,
    pub(crate) effects_signal: &'static FaultEffectsSignal,
    pub(crate) ready_sender: BootReadySender,
}

/// Longest wait for the engine to drain the frames queued before a power
/// transition.
const FENCE_TIMEOUT: Duration = Duration::from_millis(500);

/// Longest wait for an event before the loop reports a heartbeat.
const HEARTBEAT_PERIOD: Duration = Duration::from_millis(500);

/// Power-on requests refused while the update interlock is engaged, before
/// any policy or scheduler effect. Arming in a latched fault state still
/// completes, without powering on.
fn refused_while_inhibited(event: FaultEvent, state: FaultManagerState) -> bool {
    crate::track_output::inhibited()
        && (matches!(
            event,
            FaultEvent::ResumeShortPressed
                | FaultEvent::ResumeLongPressed
                | FaultEvent::NetworkResume(_)
        ) || (event == FaultEvent::TrackPowerArmed && state == FaultManagerState::Normal))
}

/// Applies the decided track power after a completed fence; a network
/// resume also needs its lease to be current. Returns whether it applied.
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
async fn wait_for_fence(
    generation: PowerGeneration,
    receiver: &FaultEventReceiver,
    quiesce_sender: &PowerQuiesceSender,
    ack_receiver: &PowerFenceAckReceiver,
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
fn publish_effects(
    effects: &mut FaultEffectsSnapshot,
    decision: FaultDecision,
    signal: &FaultEffectsSignal,
) {
    effects.apply(decision);
    signal.signal(*effects);
}

fn log_decision(decision: &FaultDecision) {
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

    let mut loop_state = FaultLoop::new();
    let mut effects = FaultEffectsSnapshot::new();
    if !track_output.set_track_enabled(loop_state.track_enabled()) {
        defmt::info!("fault_manager: initial track power held off by the update interlock");
    }
    state_sender.send(loop_state.state());
    effects_signal.signal(effects);
    announce_ready(ready_sender, BootReadyEvent::FaultManager).await;

    loop {
        let event = match loop_state.take_pending() {
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
        if refused_while_inhibited(event, loop_state.state()) {
            defmt::info!(
                "fault_manager: {:?} refused while the track is inhibited",
                event
            );
            continue;
        }
        defmt::info!(
            "fault_manager: event={:?} state_before={:?}",
            event,
            loop_state.state()
        );
        let plan = loop_state.step(
            event,
            track_output.is_track_enabled(),
            crate::track_output::inhibited(),
        );
        // The stop policy takes effect even if the drain below is interrupted
        // by a higher-priority fault or times out; the bridge goes off at the
        // start of the fence. Only the interlock acknowledgement needs the fence.
        if plan.interlock_stop {
            state_sender.send(plan.decision.state);
            publish_effects(&mut effects, plan.decision, effects_signal);
        }
        let fence_complete = match plan.fence {
            Some(generation) => {
                // Keep the physical bridge low until all frames queued before
                // this transition have crossed the engine fence.
                track_output.disable();
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
                    loop_state.defer(interrupting_event);
                }
                fenced.is_ok()
            }
            None => true,
        };
        let applied = fence_complete
            && apply_track_power(event, plan.decision.track_enabled, &mut track_output);
        if !applied {
            defmt::warn!("fault_manager: power transition interrupted, timed out or unauthorized");
            continue;
        }
        loop_state.commit(&plan);
        if signal_completion(event, plan.interlock_stop) {
            continue;
        }

        if plan.decision.state != plan.decision.previous_state {
            state_sender.send(plan.decision.state);
        }
        log_decision(&plan.decision);
        publish_effects(&mut effects, plan.decision, effects_signal);
    }
}
