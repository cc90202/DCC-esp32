//! Framework-independent runtime status reducer.

use crate::system_status::{FaultCause, FaultManagerState, LedState, SystemStatusEvent};

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

    /// Returns whether any fault is currently latched.
    #[must_use]
    pub const fn fault_active(self) -> bool {
        matches!(self.safety_state, FaultManagerState::FaultLatched(_))
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
        self.track_power_on() && !self.fault_active()
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
