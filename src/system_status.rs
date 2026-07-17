//! System runtime status model for LEDs and operator controls.
//!
//! # Overview
//!
//! Tracks system state (booting, running, e-stop, fault) via events emitted by various
//! tasks. [`crate::application::StatusModel`] applies events to compute the current
//! `LedState`, which the status LED task uses to drive GPIO14 (green) and GPIO15 (red).
//!
//! # LED States and Meanings
//!
//! - **Booting** (green blink 125ms) - System starting up
//! - **WifiConnecting** (red blink 500ms) - WiFi connection in progress
//! - **Running** (solid green) - Ready, no faults
//! - **EstopActive** (solid red) - Operator pressed stop button
//! - **FaultLatched** (solid red) - Hardware fault (track short, CV error, etc.)
//!
//! # Examples
//!
//! **Apply events to the model:**
//!
//! ```
//! use dcc_esp32::application::StatusModel;
//! use dcc_esp32::system_status::{SystemStatusEvent, LedState};
//!
//! let mut model = StatusModel::new();
//! assert_eq!(model.led_state(), LedState::Booting);
//!
//! // Boot completes
//! model.apply(SystemStatusEvent::BootCompleted);
//! assert_eq!(model.led_state(), LedState::Running);
//!
//! // WiFi starts connecting
//! model.apply(SystemStatusEvent::WifiConnecting);
//! assert_eq!(model.led_state(), LedState::WifiConnecting);
//!
//! // WiFi connects
//! model.apply(SystemStatusEvent::WifiConnected);
//! assert_eq!(model.led_state(), LedState::Running);
//! ```
//!
//! **Fault handling:**
//!
//! ```
//! use dcc_esp32::application::StatusModel;
//! use dcc_esp32::system_status::{SystemStatusEvent, FaultCause, LedState};
//!
//! let mut model = StatusModel::new();
//! model.apply(SystemStatusEvent::BootCompleted);
//!
//! // Track short detected
//! model.apply(SystemStatusEvent::FaultLatched(FaultCause::TrackShort));
//! assert_eq!(model.led_state(), LedState::FaultLatched);
//!
//! // Service clears the fault
//! model.apply(SystemStatusEvent::FaultCleared);
//! assert_eq!(model.led_state(), LedState::Running);
//! ```

/// Fault origin used for observability and future policy routing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum FaultCause {
    CvService,
    TrackShort,
    Estop,
    Internal,
}

/// Mutually exclusive safety state governing track power and motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum FaultManagerState {
    /// Track powered, motion commands accepted.
    Normal,
    /// Operator-initiated emergency stop; resume button clears.
    EstopLatched,
    /// Hardware/service fault; requires long-press or service clear.
    FaultLatched(FaultCause),
}

/// Runtime status events emitted by tasks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum SystemStatusEvent {
    BootStarted,
    BootCompleted,
    WifiConnecting,
    WifiConnected,
    WifiDisconnected,
    EstopActive,
    EstopCleared,
    FaultLatched(FaultCause),
    FaultCleared,
}

/// LED representation of the current system state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum LedState {
    Booting,
    WifiConnecting,
    Running,
    EstopActive,
    FaultLatched,
}

pub use crate::application::StatusModel;

// --- Boot lifecycle events ---
//
// Events emitted by tasks during boot to acknowledge readiness (or report a
// degraded optional peripheral) to the composition root (`boot.rs`). These
// types live here rather than in `boot.rs` because they are imported by
// several independent task modules (net, display, fault manager, control
// buttons, short detector); keeping them in the composition root would force
// every one of those modules to depend on it.

/// Reason an optional (non-critical) peripheral failed to initialise.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum OptionalPeripheralInit {
    DisplayI2c,
    DisplayInit,
    DisplayUnavailable,
}

/// Readiness acknowledgement sent by a task once it has finished its own
/// startup sequence and is ready to participate in normal runtime operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum BootReadyEvent {
    DisplayReady,
    DisplayDegraded(OptionalPeripheralInit),
    DccEngine,
    StatusLed,
    Scheduler,
    Net,
    FaultManager,
    StopButton,
    ResumeButton,
    ShortDetector,
}

// --- Fault manager events ---
//
// `FaultEvent` lives here (rather than in `fault_manager.rs`) because it is
// imported by task modules across the crate (dcc engine, boot, short
// detector, control buttons, net) that must not depend on `fault_manager`
// itself, which also depends on `dcc::SchedulerCommand`. Keeping the event
// type in this pure, dependency-free hub breaks that cycle.

/// External events consumed by the fault state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum FaultEvent {
    /// Boot sequence has finished; track power may be enabled if state permits.
    TrackPowerArmed,
    /// Physical stop button pressed (GPIO22).
    StopPressed,
    /// Resume button short-press (<2 s) - clears e-stop, ignored during fault.
    ResumeShortPressed,
    /// Resume button long-press (≥2 s) - force-clears any latched state.
    ResumeLongPressed,
    /// Hardware or service fault detected (e.g. track short, CV error).
    FaultLatched(FaultCause),
    /// Fault condition resolved by automated service logic.
    FaultClearedByService,
}

// --- Display events ---
//
// `BootStep` and `DisplayEvent` live here (rather than in `display.rs`)
// because they are imported by task modules (dcc scheduler, fault manager,
// boot, net) that must not depend on `display`, which pulls in the
// embedded-graphics/ssd1306 rendering stack. `display.rs` continues to own
// the async rendering task and consumes these types read-only.

/// Boot sequence milestones shown on the display during initialisation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum BootStep {
    PeripheralsInit,
    WifiConnecting,
    WifiConnected,
    DccEngineReady,
    SystemRunning,
}

/// Events that update the OLED display content.
#[derive(Debug, Clone)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum DisplayEvent {
    BootProgress(BootStep),
    IpAssigned([u8; 4]),
    ProvisioningMode {
        ssid: heapless::String<14>,
        setup_url: &'static str,
    },
    SystemState(LedState),
    ActiveLocoCount(u8),
    Fault(FaultCause),
    FaultCleared,
    Message(heapless::String<21>),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_boot_to_running() {
        let mut model = StatusModel::new();
        assert_eq!(model.led_state(), LedState::Booting);
        model.apply(SystemStatusEvent::BootCompleted);
        assert_eq!(model.led_state(), LedState::Running);
    }

    #[test]
    fn test_boot_event_keeps_running_state() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        assert_eq!(model.led_state(), LedState::Running);

        model.apply(SystemStatusEvent::BootCompleted);
        assert_eq!(model.led_state(), LedState::Running);
    }

    #[test]
    fn test_wifi_connecting_blinks_red_after_boot() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        model.apply(SystemStatusEvent::WifiConnecting);
        assert_eq!(model.led_state(), LedState::WifiConnecting);
    }

    #[test]
    fn test_wifi_connected_returns_to_running() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        model.apply(SystemStatusEvent::WifiConnecting);
        model.apply(SystemStatusEvent::WifiConnected);
        assert_eq!(model.led_state(), LedState::Running);
    }

    #[test]
    fn test_wifi_disconnected_returns_to_connecting_state() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        model.apply(SystemStatusEvent::WifiConnected);
        model.apply(SystemStatusEvent::WifiDisconnected);
        assert_eq!(model.led_state(), LedState::WifiConnecting);
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
    fn test_fault_clear_returns_to_running() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        model.apply(SystemStatusEvent::FaultLatched(FaultCause::TrackShort));
        assert_eq!(model.led_state(), LedState::FaultLatched);

        model.apply(SystemStatusEvent::FaultCleared);
        assert_eq!(model.led_state(), LedState::Running);
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

        model.apply(SystemStatusEvent::FaultLatched(FaultCause::CvService));
        assert!(!model.short_circuit());
        assert!(!model.track_power_on());

        model.apply(SystemStatusEvent::FaultLatched(FaultCause::Internal));
        assert!(!model.short_circuit());
    }

    #[test]
    fn test_fault_active_and_cause_track_latched_fault() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);
        assert!(!model.fault_active());
        assert_eq!(model.fault_cause(), None);

        model.apply(SystemStatusEvent::FaultLatched(FaultCause::CvService));
        assert!(model.fault_active());
        assert_eq!(model.fault_cause(), Some(FaultCause::CvService));
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
