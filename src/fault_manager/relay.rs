//! Backpressure-isolated delivery of fault-policy side effects.

#[cfg(target_arch = "riscv32")]
use embassy_sync::channel::Sender;
#[cfg(target_arch = "riscv32")]
use embassy_sync::signal::Signal;

#[cfg(target_arch = "riscv32")]
use crate::{
    dcc::SchedulerCommand,
    system_status::{DisplayEvent, FaultCause, SystemStatusEvent},
};

use super::FaultManagerState;
use super::policy::{FaultDecision, SchedulerEffect, StatusEffect};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SchedulerMode {
    Paused,
    Running,
}

/// Latest fault effects required by runtime adapters.
///
/// Generations preserve one-shot effects while the runtime signal coalesces intermediate
/// snapshots. The final scheduler mode remains authoritative when opposite
/// transitions occur while an adapter is backpressured.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct FaultEffectsSnapshot {
    state: FaultManagerState,
    scheduler_mode: SchedulerMode,
    scheduler_sync_generation: u32,
    emergency_stop_generation: u32,
    restart_discovery_generation: u32,
}

impl FaultEffectsSnapshot {
    pub(super) const fn new() -> Self {
        Self {
            state: FaultManagerState::Normal,
            scheduler_mode: SchedulerMode::Paused,
            scheduler_sync_generation: 0,
            emergency_stop_generation: 0,
            restart_discovery_generation: 0,
        }
    }

    pub(super) fn apply(&mut self, decision: FaultDecision) {
        self.state = decision.state;
        for effect in decision.scheduler_effects.into_iter().flatten() {
            match effect {
                SchedulerEffect::EmergencyStopAll => {
                    self.emergency_stop_generation = self.emergency_stop_generation.wrapping_add(1);
                }
                SchedulerEffect::Pause => {
                    self.scheduler_mode = SchedulerMode::Paused;
                    self.scheduler_sync_generation = self.scheduler_sync_generation.wrapping_add(1);
                }
                SchedulerEffect::Resume => {
                    self.scheduler_mode = SchedulerMode::Running;
                    self.scheduler_sync_generation = self.scheduler_sync_generation.wrapping_add(1);
                }
                SchedulerEffect::RestartRailcomDiscovery => {
                    self.restart_discovery_generation =
                        self.restart_discovery_generation.wrapping_add(1);
                }
            }
        }
    }
}

#[cfg(target_arch = "riscv32")]
pub(crate) type FaultEffectsSignal =
    Signal<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, FaultEffectsSnapshot>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct FaultRelayPlan {
    scheduler_effects: [Option<SchedulerEffect>; 3],
    status_effects: [Option<StatusEffect>; 2],
}

fn plan_fault_relay(
    previous: FaultEffectsSnapshot,
    current: FaultEffectsSnapshot,
) -> FaultRelayPlan {
    let mut scheduler_effects = [None; 3];
    let mut scheduler_index = 0;

    if current.emergency_stop_generation != previous.emergency_stop_generation {
        scheduler_effects[scheduler_index] = Some(SchedulerEffect::EmergencyStopAll);
        scheduler_index += 1;
    }

    if current.scheduler_sync_generation != previous.scheduler_sync_generation {
        scheduler_effects[scheduler_index] = Some(match current.scheduler_mode {
            SchedulerMode::Paused => SchedulerEffect::Pause,
            SchedulerMode::Running => SchedulerEffect::Resume,
        });
        scheduler_index += 1;
    }

    if current.scheduler_mode == SchedulerMode::Running
        && current.restart_discovery_generation != previous.restart_discovery_generation
    {
        scheduler_effects[scheduler_index] = Some(SchedulerEffect::RestartRailcomDiscovery);
    }

    let status_effects = match (previous.state, current.state) {
        (previous, current) if previous == current => [None, None],
        (_, FaultManagerState::FaultLatched(cause)) => {
            [Some(StatusEffect::FaultLatched(cause)), None]
        }
        (FaultManagerState::FaultLatched(_), FaultManagerState::EstopLatched) => [
            Some(StatusEffect::FaultCleared),
            Some(StatusEffect::EstopActive),
        ],
        (FaultManagerState::Normal, FaultManagerState::EstopLatched) => {
            [Some(StatusEffect::EstopActive), None]
        }
        (FaultManagerState::EstopLatched, FaultManagerState::Normal) => {
            [Some(StatusEffect::EstopCleared), None]
        }
        (FaultManagerState::FaultLatched(_), FaultManagerState::Normal) => {
            [Some(StatusEffect::FaultCleared), None]
        }
        (FaultManagerState::Normal, FaultManagerState::Normal)
        | (FaultManagerState::EstopLatched, FaultManagerState::EstopLatched) => [None, None],
    };

    FaultRelayPlan {
        scheduler_effects,
        status_effects,
    }
}

#[cfg(target_arch = "riscv32")]
pub(crate) struct FaultEffectsTaskContext {
    pub(crate) effects_signal: &'static FaultEffectsSignal,
    pub(crate) scheduler_sender: Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        SchedulerCommand,
        32,
    >,
    pub(crate) status_sender: Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        SystemStatusEvent,
        16,
    >,
    pub(crate) net_status_sender: Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        SystemStatusEvent,
        8,
    >,
    pub(crate) display_sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        DisplayEvent,
        8,
    >,
}

/// Relays coalescible fault effects without applying backpressure to the
/// safety-state loop.
#[cfg(target_arch = "riscv32")]
#[embassy_executor::task]
pub(crate) async fn fault_effects_task(context: FaultEffectsTaskContext) -> ! {
    let FaultEffectsTaskContext {
        effects_signal,
        scheduler_sender,
        status_sender,
        net_status_sender,
        display_sender,
    } = context;
    let mut delivered = FaultEffectsSnapshot::new();

    loop {
        let current = effects_signal.wait().await;
        let plan = plan_fault_relay(delivered, current);

        for effect in plan.scheduler_effects.into_iter().flatten() {
            let command = match effect {
                SchedulerEffect::EmergencyStopAll => SchedulerCommand::EmergencyStopAll,
                SchedulerEffect::Pause => SchedulerCommand::Pause,
                SchedulerEffect::Resume => SchedulerCommand::Resume,
                SchedulerEffect::RestartRailcomDiscovery => {
                    SchedulerCommand::RestartRailcomDiscovery
                }
            };
            scheduler_sender.send(command).await;
        }

        for effect in plan.status_effects.into_iter().flatten() {
            let (event, display_event) = match effect {
                StatusEffect::EstopActive => (
                    SystemStatusEvent::EstopActive,
                    DisplayEvent::Fault(FaultCause::Estop),
                ),
                StatusEffect::EstopCleared => {
                    (SystemStatusEvent::EstopCleared, DisplayEvent::FaultCleared)
                }
                StatusEffect::FaultLatched(cause) => (
                    SystemStatusEvent::FaultLatched(cause),
                    DisplayEvent::Fault(cause),
                ),
                StatusEffect::FaultCleared => {
                    (SystemStatusEvent::FaultCleared, DisplayEvent::FaultCleared)
                }
            };
            status_sender.send(event).await;
            net_status_sender.send(event).await;
            if display_sender.try_send(display_event).is_err() {
                defmt::warn!("fault_manager: display event dropped");
            }
        }

        delivered = current;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::system_status::{FaultCause, FaultEvent};

    use super::super::policy::FaultPolicy;

    fn armed_snapshot() -> (FaultPolicy, FaultEffectsSnapshot) {
        let mut policy = FaultPolicy::new();
        let mut snapshot = FaultEffectsSnapshot::new();
        snapshot.apply(policy.handle(FaultEvent::TrackPowerArmed));
        (policy, snapshot)
    }

    #[test]
    fn blocked_relay_preserves_estop_and_final_resume() {
        let (mut policy, delivered) = armed_snapshot();
        let mut latest = delivered;

        latest.apply(policy.handle(FaultEvent::StopPressed));
        latest.apply(policy.handle(FaultEvent::ResumeShortPressed));

        assert_eq!(
            plan_fault_relay(delivered, latest).scheduler_effects,
            [
                Some(SchedulerEffect::EmergencyStopAll),
                Some(SchedulerEffect::Resume),
                Some(SchedulerEffect::RestartRailcomDiscovery),
            ]
        );
    }

    #[test]
    fn blocked_relay_uses_final_paused_mode() {
        let (mut policy, delivered) = armed_snapshot();
        let mut latest = delivered;

        latest.apply(policy.handle(FaultEvent::ResumeShortPressed));
        latest.apply(policy.handle(FaultEvent::FaultLatched(FaultCause::Internal)));

        let plan = plan_fault_relay(delivered, latest);
        assert_eq!(
            plan.scheduler_effects,
            [Some(SchedulerEffect::Pause), None, None]
        );
        assert_eq!(
            plan.status_effects,
            [Some(StatusEffect::FaultLatched(FaultCause::Internal)), None]
        );
    }

    #[test]
    fn estop_to_fault_publishes_only_authoritative_fault() {
        let (mut policy, mut delivered) = armed_snapshot();
        delivered.apply(policy.handle(FaultEvent::StopPressed));
        let mut latest = delivered;

        latest.apply(policy.handle(FaultEvent::FaultLatched(FaultCause::TrackShort)));

        assert_eq!(
            plan_fault_relay(delivered, latest).status_effects,
            [
                Some(StatusEffect::FaultLatched(FaultCause::TrackShort)),
                None,
            ]
        );
    }

    #[test]
    fn unchanged_snapshot_has_no_relay_work() {
        let (_, snapshot) = armed_snapshot();

        assert_eq!(
            plan_fault_relay(snapshot, snapshot),
            FaultRelayPlan {
                scheduler_effects: [None; 3],
                status_effects: [None; 2],
            }
        );
    }
}
