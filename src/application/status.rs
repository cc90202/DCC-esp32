//! Framework-independent runtime status reducer.

use crate::station_status::{FaultCause, FaultManagerState, LedState, SystemStatusEvent};

/// State reducer that maps runtime events to a single LED state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusModel {
    boot_completed: bool,
    wifi_connecting: bool,
    safety_state: FaultManagerState,
}

impl Default for StatusModel {
    fn default() -> Self {
        Self::new()
    }
}

impl StatusModel {
    /// Build a new model in boot state.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            boot_completed: false,
            wifi_connecting: false,
            safety_state: FaultManagerState::Normal,
        }
    }

    /// Apply one event to the model.
    pub fn apply(&mut self, event: SystemStatusEvent) {
        match event {
            SystemStatusEvent::BootStarted => {
                self.boot_completed = false;
            }
            SystemStatusEvent::BootCompleted => {
                self.boot_completed = true;
            }
            SystemStatusEvent::WifiConnecting => {
                self.wifi_connecting = true;
            }
            SystemStatusEvent::WifiConnected => {
                self.wifi_connecting = false;
            }
            SystemStatusEvent::WifiDisconnected => {
                self.wifi_connecting = true;
            }
            SystemStatusEvent::EstopActive => {
                if !matches!(self.safety_state, FaultManagerState::FaultLatched(_)) {
                    self.safety_state = FaultManagerState::EstopLatched;
                }
            }
            SystemStatusEvent::EstopCleared => {
                if matches!(self.safety_state, FaultManagerState::EstopLatched) {
                    self.safety_state = FaultManagerState::Normal;
                }
            }
            SystemStatusEvent::FaultLatched(cause) => {
                self.safety_state = FaultManagerState::FaultLatched(cause);
            }
            SystemStatusEvent::FaultCleared => {
                if matches!(self.safety_state, FaultManagerState::FaultLatched(_)) {
                    self.safety_state = FaultManagerState::Normal;
                }
            }
        }
    }

    /// Replaces the safety state with the one the fault manager holds now.
    ///
    /// Used when status events were dropped and the model can no longer be
    /// rebuilt from them.
    pub fn sync_safety_state(&mut self, state: FaultManagerState) {
        self.safety_state = state;
    }

    /// Returns whether track power should be considered enabled for operator/protocol status.
    #[must_use]
    pub const fn track_power_on(self) -> bool {
        matches!(self.safety_state, FaultManagerState::Normal)
    }

    /// Returns whether the system is currently latched in emergency stop.
    #[must_use]
    pub const fn estop_active(self) -> bool {
        matches!(self.safety_state, FaultManagerState::EstopLatched)
    }

    /// Returns whether the current fault is a track short circuit (Z21 CentralState bit2).
    #[must_use]
    pub const fn short_circuit(self) -> bool {
        matches!(
            self.safety_state,
            FaultManagerState::FaultLatched(FaultCause::TrackShort)
        )
    }

    /// Returns the currently latched fault cause, if any.
    #[must_use]
    pub const fn fault_cause(self) -> Option<FaultCause> {
        match self.safety_state {
            FaultManagerState::FaultLatched(cause) => Some(cause),
            FaultManagerState::Normal | FaultManagerState::EstopLatched => None,
        }
    }

    /// Returns whether programming on main may be accepted.
    ///
    /// This is intentionally conservative: main-track programming is rejected
    /// whenever track power is logically off, including e-stop and any fault.
    #[must_use]
    pub const fn pom_allowed(self) -> bool {
        self.track_power_on()
    }

    /// Compute the current LED state with fixed priority.
    #[must_use]
    pub const fn led_state(self) -> LedState {
        match self.safety_state {
            FaultManagerState::FaultLatched(_) => LedState::FaultLatched,
            FaultManagerState::EstopLatched => LedState::EstopActive,
            FaultManagerState::Normal if self.boot_completed && self.wifi_connecting => {
                LedState::WifiConnecting
            }
            FaultManagerState::Normal if self.boot_completed => LedState::Running,
            FaultManagerState::Normal => LedState::Booting,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_boot_event_keeps_running_state() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        assert_eq!(model.led_state(), LedState::Running);

        model.apply(SystemStatusEvent::BootCompleted);
        assert_eq!(model.led_state(), LedState::Running);
    }

    #[test]
    fn test_wifi_connecting_before_boot_stays_booting() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::WifiConnecting);
        assert_eq!(model.led_state(), LedState::Booting);
    }

    #[test]
    fn test_wifi_reconnect_sequence_returns_to_running() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        model.apply(SystemStatusEvent::WifiConnecting);
        model.apply(SystemStatusEvent::WifiConnected);
        model.apply(SystemStatusEvent::WifiDisconnected);
        assert_eq!(model.led_state(), LedState::WifiConnecting);
        model.apply(SystemStatusEvent::WifiConnecting);
        model.apply(SystemStatusEvent::WifiConnected);
        assert_eq!(model.led_state(), LedState::Running);
    }

    #[test]
    fn test_estop_latched_until_cleared() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);

        model.apply(SystemStatusEvent::EstopActive);
        assert_eq!(model.led_state(), LedState::EstopActive);

        model.apply(SystemStatusEvent::EstopCleared);
        assert_eq!(model.led_state(), LedState::Running);
    }

    #[test]
    fn test_fault_has_top_priority() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        model.apply(SystemStatusEvent::EstopActive);
        model.apply(SystemStatusEvent::FaultLatched(FaultCause::Internal));
        assert_eq!(model.led_state(), LedState::FaultLatched);
    }

    #[test]
    fn stale_clear_event_cannot_clear_a_different_safety_state() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::FaultLatched(FaultCause::Internal));
        model.apply(SystemStatusEvent::EstopCleared);
        assert_eq!(model.fault_cause(), Some(FaultCause::Internal));

        model.apply(SystemStatusEvent::FaultCleared);
        model.apply(SystemStatusEvent::EstopActive);
        model.apply(SystemStatusEvent::FaultCleared);
        assert!(model.estop_active());
    }

    #[test]
    fn test_track_power_on_in_normal_state() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        assert!(model.track_power_on());
        assert!(!model.estop_active());
    }

    #[test]
    fn test_track_power_off_during_estop() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        model.apply(SystemStatusEvent::EstopActive);
        assert!(!model.track_power_on());
        assert!(model.estop_active());
    }

    #[test]
    fn test_track_power_off_during_fault() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        model.apply(SystemStatusEvent::FaultLatched(FaultCause::TrackShort));
        assert!(!model.track_power_on());
        assert!(!model.estop_active());
    }

    #[test]
    fn test_fault_has_priority_over_wifi_reconnect() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        model.apply(SystemStatusEvent::WifiDisconnected);
        model.apply(SystemStatusEvent::FaultLatched(FaultCause::Internal));
        assert_eq!(model.led_state(), LedState::FaultLatched);
    }

    #[test]
    fn test_estop_has_priority_over_wifi_reconnect() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        model.apply(SystemStatusEvent::WifiDisconnected);
        model.apply(SystemStatusEvent::EstopActive);
        assert_eq!(model.led_state(), LedState::EstopActive);
    }

    #[test]
    fn test_short_circuit_flag_only_on_track_short() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);

        model.apply(SystemStatusEvent::FaultLatched(FaultCause::TrackShort));
        assert!(model.short_circuit());
        assert!(!model.track_power_on());

        model.apply(SystemStatusEvent::FaultCleared);
        assert!(!model.short_circuit());
        assert!(model.track_power_on());
    }

    #[test]
    fn test_non_short_fault_does_not_set_short_circuit() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);

        model.apply(SystemStatusEvent::FaultLatched(FaultCause::Internal));
        assert!(!model.short_circuit());
        assert!(!model.track_power_on());

        model.apply(SystemStatusEvent::FaultLatched(FaultCause::Internal));
        assert!(!model.short_circuit());
    }

    #[test]
    fn test_fault_cause_tracks_latched_fault() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        assert_eq!(model.fault_cause(), None);

        model.apply(SystemStatusEvent::FaultLatched(FaultCause::Internal));
        assert_eq!(model.fault_cause(), Some(FaultCause::Internal));
    }

    #[test]
    fn test_pom_allowed_only_in_normal_running_state() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        assert!(model.pom_allowed());

        model.apply(SystemStatusEvent::EstopActive);
        assert!(!model.pom_allowed());

        model.apply(SystemStatusEvent::EstopCleared);
        assert!(model.pom_allowed());

        model.apply(SystemStatusEvent::FaultLatched(FaultCause::Internal));
        assert!(!model.pom_allowed());
    }
}
