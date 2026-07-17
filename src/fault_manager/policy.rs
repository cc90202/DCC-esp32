//! Pure fault-management policy.
//!
//! This module owns the safety state machine and describes its required effects
//! without depending on Embassy channels, the scheduler actor, or GPIO drivers.

#[cfg(any(test, target_arch = "riscv32"))]
use crate::system_status::FaultCause;
#[cfg(any(test, target_arch = "riscv32"))]
use crate::system_status::FaultEvent;
pub use crate::system_status::FaultManagerState;

#[cfg(any(test, target_arch = "riscv32"))]
const MAX_SCHEDULER_EFFECTS: usize = 2;
#[cfg(any(test, target_arch = "riscv32"))]
const MAX_STATUS_EFFECTS: usize = 2;

/// Scheduler-facing effect requested by the fault policy.
#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum SchedulerEffect {
    EmergencyStopAll,
    Pause,
    Resume,
    RestartRailcomDiscovery,
}

/// Status-facing effect requested by the fault policy.
#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum StatusEffect {
    EstopActive,
    EstopCleared,
    FaultLatched(FaultCause),
    FaultCleared,
}

/// Complete, framework-independent result of handling one fault event.
#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct FaultDecision {
    pub(super) previous_state: FaultManagerState,
    pub(super) state: FaultManagerState,
    pub(super) track_enabled: bool,
    pub(super) motion_enabled: bool,
    pub(super) scheduler_effects: [Option<SchedulerEffect>; MAX_SCHEDULER_EFFECTS],
    pub(super) status_effects: [Option<StatusEffect>; MAX_STATUS_EFFECTS],
}

/// Stateful application policy for track faults and emergency stop.
///
/// Track output remains disabled until
/// [`FaultEvent::TrackPowerArmed`](crate::system_status::FaultEvent::TrackPowerArmed)
/// is received, even though the logical initial state is [`FaultManagerState::Normal`].
#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct FaultPolicy {
    state: FaultManagerState,
    armed: bool,
}

#[cfg(any(test, target_arch = "riscv32"))]
impl FaultPolicy {
    #[must_use]
    pub(super) const fn new() -> Self {
        Self {
            state: FaultManagerState::Normal,
            armed: false,
        }
    }

    #[must_use]
    pub(super) const fn state(self) -> FaultManagerState {
        self.state
    }

    #[must_use]
    pub(super) const fn track_enabled(self) -> bool {
        self.armed && matches!(self.state, FaultManagerState::Normal)
    }

    /// Apply one event and return the effects that outer adapters must perform.
    pub(super) fn handle(&mut self, event: FaultEvent) -> FaultDecision {
        if event == FaultEvent::TrackPowerArmed {
            self.armed = true;
        }

        let previous_state = self.state;
        let mut scheduler_effects = [None; MAX_SCHEDULER_EFFECTS];
        let mut status_effects = [None; MAX_STATUS_EFFECTS];

        match (self.state, event) {
            (FaultManagerState::Normal, FaultEvent::TrackPowerArmed) => {
                scheduler_effects[0] = Some(SchedulerEffect::Resume);
            }
            (FaultManagerState::EstopLatched, FaultEvent::TrackPowerArmed)
            | (FaultManagerState::FaultLatched(_), FaultEvent::TrackPowerArmed) => {}
            (FaultManagerState::Normal, FaultEvent::StopPressed) => {
                self.state = FaultManagerState::EstopLatched;
                scheduler_effects[0] = Some(SchedulerEffect::EmergencyStopAll);
                scheduler_effects[1] = Some(SchedulerEffect::Pause);
                status_effects[0] = Some(StatusEffect::EstopActive);
            }
            (FaultManagerState::EstopLatched, FaultEvent::ResumeShortPressed)
            | (FaultManagerState::EstopLatched, FaultEvent::ResumeLongPressed) => {
                self.state = FaultManagerState::Normal;
                scheduler_effects[0] = Some(SchedulerEffect::Resume);
                status_effects[0] = Some(StatusEffect::EstopCleared);
            }
            (FaultManagerState::FaultLatched(_), FaultEvent::ResumeShortPressed) => {}
            (FaultManagerState::FaultLatched(_), FaultEvent::ResumeLongPressed) => {
                self.state = FaultManagerState::Normal;
                scheduler_effects[0] = Some(SchedulerEffect::Resume);
                status_effects[0] = Some(StatusEffect::FaultCleared);
            }
            (FaultManagerState::Normal, FaultEvent::FaultLatched(cause)) => {
                self.state = FaultManagerState::FaultLatched(cause);
                scheduler_effects[0] = Some(SchedulerEffect::Pause);
                status_effects[0] = Some(StatusEffect::FaultLatched(cause));
            }
            (FaultManagerState::EstopLatched, FaultEvent::FaultLatched(cause)) => {
                self.state = FaultManagerState::FaultLatched(cause);
                status_effects[0] = Some(StatusEffect::EstopCleared);
                status_effects[1] = Some(StatusEffect::FaultLatched(cause));
            }
            (FaultManagerState::FaultLatched(_), FaultEvent::FaultLatched(cause)) => {
                self.state = FaultManagerState::FaultLatched(cause);
                status_effects[0] = Some(StatusEffect::FaultLatched(cause));
            }
            (FaultManagerState::FaultLatched(_), FaultEvent::FaultClearedByService) => {
                self.state = FaultManagerState::Normal;
                scheduler_effects[0] = Some(SchedulerEffect::Resume);
                status_effects[0] = Some(StatusEffect::FaultCleared);
            }
            (FaultManagerState::Normal, FaultEvent::ResumeShortPressed)
            | (FaultManagerState::Normal, FaultEvent::ResumeLongPressed) => {
                scheduler_effects[0] = Some(SchedulerEffect::Resume);
            }
            (FaultManagerState::Normal, FaultEvent::FaultClearedByService)
            | (FaultManagerState::EstopLatched, FaultEvent::StopPressed)
            | (FaultManagerState::EstopLatched, FaultEvent::FaultClearedByService)
            | (FaultManagerState::FaultLatched(_), FaultEvent::StopPressed) => {}
        }

        if scheduler_effects[0] == Some(SchedulerEffect::Resume) && self.track_enabled() {
            scheduler_effects[1] = Some(SchedulerEffect::RestartRailcomDiscovery);
        }

        FaultDecision {
            previous_state,
            state: self.state,
            track_enabled: self.track_enabled(),
            motion_enabled: matches!(self.state, FaultManagerState::Normal),
            scheduler_effects,
            status_effects,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn track_stays_disabled_until_armed() {
        let policy = FaultPolicy::new();

        assert_eq!(policy.state(), FaultManagerState::Normal);
        assert!(!policy.track_enabled());
    }

    #[test]
    fn track_power_armed_in_normal_resumes_scheduler_and_discovery() {
        let mut policy = FaultPolicy::new();
        let decision = policy.handle(FaultEvent::TrackPowerArmed);

        assert_eq!(decision.state, FaultManagerState::Normal);
        assert!(decision.track_enabled);
        assert_eq!(
            decision.scheduler_effects,
            [
                Some(SchedulerEffect::Resume),
                Some(SchedulerEffect::RestartRailcomDiscovery),
            ]
        );
    }

    #[test]
    fn stop_latches_estop_and_disables_track() {
        let mut policy = FaultPolicy::new();
        policy.handle(FaultEvent::TrackPowerArmed);
        let decision = policy.handle(FaultEvent::StopPressed);

        assert_eq!(decision.state, FaultManagerState::EstopLatched);
        assert!(!decision.track_enabled);
        assert!(!decision.motion_enabled);
        assert_eq!(
            decision.scheduler_effects,
            [
                Some(SchedulerEffect::EmergencyStopAll),
                Some(SchedulerEffect::Pause),
            ]
        );
        assert_eq!(
            decision.status_effects,
            [Some(StatusEffect::EstopActive), None]
        );
    }

    #[test]
    fn resume_short_from_estop_clears_estop_and_restarts_discovery() {
        let mut policy = FaultPolicy::new();
        policy.handle(FaultEvent::TrackPowerArmed);
        policy.handle(FaultEvent::StopPressed);
        let decision = policy.handle(FaultEvent::ResumeShortPressed);

        assert_eq!(decision.state, FaultManagerState::Normal);
        assert!(decision.track_enabled);
        assert_eq!(
            decision.scheduler_effects,
            [
                Some(SchedulerEffect::Resume),
                Some(SchedulerEffect::RestartRailcomDiscovery),
            ]
        );
        assert_eq!(
            decision.status_effects,
            [Some(StatusEffect::EstopCleared), None]
        );
    }

    #[test]
    fn resume_in_normal_retriggers_scheduler_resume() {
        let mut policy = FaultPolicy::new();
        let decision = policy.handle(FaultEvent::ResumeShortPressed);

        assert_eq!(decision.state, FaultManagerState::Normal);
        assert_eq!(
            decision.scheduler_effects,
            [Some(SchedulerEffect::Resume), None]
        );
    }

    #[test]
    fn resume_short_is_ignored_while_fault_latched() {
        let mut policy = FaultPolicy::new();
        policy.handle(FaultEvent::FaultLatched(FaultCause::CvService));
        let decision = policy.handle(FaultEvent::ResumeShortPressed);

        assert_eq!(
            decision.state,
            FaultManagerState::FaultLatched(FaultCause::CvService)
        );
        assert_eq!(decision.scheduler_effects, [None, None]);
    }

    #[test]
    fn resume_long_clears_fault_latched() {
        let mut policy = FaultPolicy::new();
        policy.handle(FaultEvent::TrackPowerArmed);
        policy.handle(FaultEvent::FaultLatched(FaultCause::TrackShort));
        let decision = policy.handle(FaultEvent::ResumeLongPressed);

        assert_eq!(decision.state, FaultManagerState::Normal);
        assert!(decision.track_enabled);
        assert_eq!(
            decision.status_effects,
            [Some(StatusEffect::FaultCleared), None]
        );
    }

    #[test]
    fn fault_cleared_by_service_returns_to_normal() {
        let mut policy = FaultPolicy::new();
        policy.handle(FaultEvent::FaultLatched(FaultCause::TrackShort));
        let decision = policy.handle(FaultEvent::FaultClearedByService);

        assert_eq!(decision.state, FaultManagerState::Normal);
        assert_eq!(
            decision.status_effects,
            [Some(StatusEffect::FaultCleared), None]
        );
    }

    #[test]
    fn fault_from_normal_pauses_scheduler() {
        let mut policy = FaultPolicy::new();
        let decision = policy.handle(FaultEvent::FaultLatched(FaultCause::TrackShort));

        assert_eq!(
            decision.state,
            FaultManagerState::FaultLatched(FaultCause::TrackShort)
        );
        assert_eq!(
            decision.scheduler_effects,
            [Some(SchedulerEffect::Pause), None]
        );
    }

    #[test]
    fn fault_replaces_estop_state() {
        let mut policy = FaultPolicy::new();
        policy.handle(FaultEvent::StopPressed);
        let decision = policy.handle(FaultEvent::FaultLatched(FaultCause::TrackShort));

        assert_eq!(
            decision.state,
            FaultManagerState::FaultLatched(FaultCause::TrackShort)
        );
        assert_eq!(
            decision.status_effects,
            [
                Some(StatusEffect::EstopCleared),
                Some(StatusEffect::FaultLatched(FaultCause::TrackShort)),
            ]
        );
    }
}
