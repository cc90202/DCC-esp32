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
//! The runtime state is exposed as [`FaultManagerState`] for observers. State
//! transitions and their required effects remain private to the pure policy so
//! callers cannot bypass the fault manager's application rules.

mod policy;
#[cfg(any(test, target_arch = "riscv32"))]
mod relay;

pub use policy::FaultManagerState;
#[cfg(target_arch = "riscv32")]
pub(crate) use relay::{FaultEffectsSignal, FaultEffectsTaskContext, fault_effects_task};

#[cfg(target_arch = "riscv32")]
use embassy_futures::select::{Either, select};
#[cfg(target_arch = "riscv32")]
use embassy_sync::watch;
#[cfg(target_arch = "riscv32")]
use embassy_time::{Duration, Instant, with_timeout};

#[cfg(target_arch = "riscv32")]
use crate::dcc::PowerGeneration;
#[cfg(target_arch = "riscv32")]
use crate::runtime_channels::{
    BootReadySender, FaultEventReceiver, RuntimeReceiver, RuntimeSender, announce_ready,
};
#[cfg(target_arch = "riscv32")]
use crate::system_status::{BootReadyEvent, FaultEvent};
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
    track_output.set_track_enabled(policy.track_enabled());
    state_sender.send(policy.state());
    effects_signal.signal(effects);
    announce_ready(ready_sender, BootReadyEvent::FaultManager).await;

    let mut pending_event = None;
    loop {
        let event = match pending_event.take() {
            Some(event) => event,
            None => receiver.receive().await,
        };
        defmt::info!(
            "fault_manager: event={:?} state_before={:?}",
            event,
            policy.state()
        );
        let mut candidate_policy = policy;
        let decision = candidate_policy.handle(event);
        let requires_fence = decision.track_enabled && !track_output.is_track_enabled();
        let fence_complete = if requires_fence {
            // Keep the physical bridge low until all frames queued before this
            // transition have crossed the engine fence.
            track_output.set_track_enabled(false);
            let generation = PowerGeneration::new(next_power_generation);
            next_power_generation = next_power_generation.wrapping_add(1).max(1);
            power_quiesce_sender.send(generation).await;
            match with_timeout(Duration::from_millis(500), async {
                select(receiver.receive(), power_fence_ack_receiver.receive()).await
            })
            .await
            {
                Ok(Either::Second(ack)) if ack == generation => true,
                Ok(Either::Second(stale_ack)) => {
                    defmt::warn!(
                        "fault_manager: ignored stale power fence ack={} expected={}",
                        stale_ack.get(),
                        generation.get()
                    );
                    false
                }
                Ok(Either::First(interrupting_event)) => {
                    // A concurrent stop or fault takes precedence over a
                    // pending power-on. Keep the bridge low and process it
                    // before accepting another transition.
                    pending_event = Some(interrupting_event);
                    false
                }
                Err(_) => false,
            }
        } else {
            true
        };
        let enabled = match event {
            FaultEvent::NetworkResume(permit) => {
                fence_complete
                    && crate::track_authority::apply_if_current(
                        permit,
                        Instant::now().as_millis(),
                        || track_output.set_track_enabled(decision.track_enabled),
                    )
                    .is_some()
            }
            _ if fence_complete => {
                track_output.set_track_enabled(decision.track_enabled);
                true
            }
            _ => false,
        };
        if !enabled {
            defmt::warn!("fault_manager: rejected stale network power-on request");
            continue;
        }
        policy = candidate_policy;

        if decision.state != decision.previous_state {
            state_sender.send(decision.state);
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

        effects.apply(decision);
        effects_signal.signal(effects);
    }
}
