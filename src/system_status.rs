//! System runtime status model for LEDs and operator controls.
//!
//! Tracks system state (booting, running, e-stop, fault) via events emitted by various
//! tasks. [`crate::application::StatusModel`] applies events to compute the current
//! `LedState`, which the status LED task uses to drive GPIO14 (green) and GPIO15 (red).
//!
//! # LED states
//!
//! - `Booting`: green blinking at 125ms, the system is starting up
//! - `WifiConnecting`: red blinking at 500ms, the WiFi connection is in progress
//! - `Running`: solid green, ready and no faults
//! - `EstopActive`: solid red, the operator pressed the stop button
//! - `FaultLatched`: solid red, hardware fault such as a track short or CV error
//!
//! # Examples
//!
//! Apply events to the model:
//!
//! ```ignore
//! use crate::application::StatusModel;
//! use crate::system_status::{LedState, SystemStatusEvent};
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
//! Fault handling:
//!
//! ```ignore
//! use crate::application::StatusModel;
//! use crate::system_status::{FaultCause, LedState, SystemStatusEvent};
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

impl core::fmt::Display for OptionalPeripheralInit {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::DisplayI2c => "display I2C initialization failed",
            Self::DisplayInit => "display controller initialization failed",
            Self::DisplayUnavailable => "display is unavailable",
        };
        formatter.write_str(message)
    }
}

impl core::error::Error for OptionalPeripheralInit {}

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
    LeaseWatchdog,
}

/// Defines [`CriticalTask`] from one `Variant => "task_name"` row per task,
/// so the spawn-failure message is built from the same literal as the name.
macro_rules! critical_tasks {
    ($( $(#[$meta:meta])* $variant:ident => $name:literal, )+) => {
        /// Runtime task whose spawn operation failed.
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
        pub enum CriticalTask {
            $( $(#[$meta])* $variant, )+
        }

        impl CriticalTask {
            /// Every critical task, in declaration order.
            pub const ALL: [Self; [$(Self::$variant),+].len()] = [$(Self::$variant),+];

            /// Name of the Embassy task function spawned for this task.
            #[must_use]
            pub const fn name(self) -> &'static str {
                match self {
                    $( Self::$variant => $name, )+
                }
            }

            /// Diagnostic reported when spawning this task fails.
            #[must_use]
            pub const fn spawn_failure_message(self) -> &'static str {
                match self {
                    $( Self::$variant => concat!("failed to spawn ", $name), )+
                }
            }
        }
    };
}

critical_tasks! {
    /// Display rendering task.
    Display => "display_task",
    /// DCC waveform engine task.
    DccEngine => "dcc_engine_task",
    /// Status LED task.
    StatusLed => "status_led_task",
    /// DCC packet scheduler task.
    Scheduler => "scheduler_task",
    /// RailCom diagnostics task.
    RailcomDiag => "railcom_diag_task",
    /// Programming-on-main actor task.
    PomActor => "pom_actor_task",
    /// Programming-on-main cutout monitor task.
    PomCutoutMonitor => "pom_cutout_monitor_task",
    /// RailCom ISR capture task.
    RailcomIsrCapture => "railcom_isr_capture_task",
    /// RailCom UART dispatch task.
    RailcomUartDispatch => "railcom_uart_runtime_dispatch_task",
    /// Network task.
    Net => "net_task",
    /// Fault side-effects task.
    FaultEffects => "fault_effects_task",
    /// Fault manager task.
    FaultManager => "fault_manager_task",
    /// Runtime provisioning request task.
    ProvisioningRequest => "provisioning_request_task",
    /// Provisioning status LED task.
    ProvisioningLed => "provisioning_led_task",
    /// Emergency-stop button task.
    StopButton => "stop_button_task",
    /// Resume button task.
    ResumeButton => "resume_button_task",
    /// Track short-circuit detector task.
    ShortDetector => "short_detector_task",
    /// Controller lease watchdog task.
    LeaseWatchdog => "client_watchdog_task",
}

impl core::fmt::Display for CriticalTask {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str(self.spawn_failure_message())
    }
}

impl core::error::Error for CriticalTask {}

// --- Fault manager events ---
//
// `FaultEvent` lives here (rather than in `fault_manager.rs`) because it is
// imported by task modules across the crate (dcc engine, boot, short
// detector, control buttons, net) that must not depend on `fault_manager`
// itself, which also depends on `dcc::SchedulerCommand`. Keeping the event
// type in a hub that depends on nothing they do breaks that cycle.
//
// The hub is no longer dependency-free in the strict sense: the `channels`
// submodule below names Embassy types and `announce_ready` logs. Both are
// gated to the target, and neither points back at a module that imports this
// one, so the direction that matters is unchanged.

use crate::authority::LeasePermit;

/// External events consumed by the fault state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum FaultEvent {
    /// Boot sequence has finished; track power may be enabled if state permits.
    TrackPowerArmed,
    /// Physical stop button pressed (GPIO22).
    StopPressed,
    /// Resume button short-press (<2 s): clears e-stop, ignored during fault.
    ResumeShortPressed,
    /// Resume button long-press (≥2 s): force-clears any latched state.
    ResumeLongPressed,
    /// Network power-on request carrying the lease that must still be current
    /// when the physical output is enabled.
    NetworkResume(LeasePermit),
    /// Hardware or service fault detected (e.g. track short, CV error).
    FaultLatched(FaultCause),
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
    /// Terminal recovery state; later boot progress must not hide it.
    UsbRecovery,
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
    Message(crate::display::RowText),
    /// Removes the last [`Self::Message`] from the footer.
    MessageCleared,
}

/// Embassy transport aliases for system status events.
///
/// Gated as a whole rather than item by item: this module is not target-only
/// but `runtime_channels` is, so without the gate the host build would look for
/// a module that does not exist there.
#[cfg(target_arch = "riscv32")]
pub(crate) mod channels {
    use super::{BootReadyEvent, DisplayEvent, FaultEvent, SystemStatusEvent};
    use crate::runtime_channels::{RuntimeChannel, RuntimeReceiver, RuntimeSender};

    /// Status events fanned out to the display, the LED and the network task.
    const STATUS_DEPTH: usize = 16;
    /// Dedicated fan-out to the net task, which drains more slowly.
    const NET_STATUS_DEPTH: usize = 8;
    /// Readiness acknowledgements at boot. Ten tasks announce into nine slots,
    /// which works because the barrier drains continuously while they announce:
    /// see `boot::readiness::wait_for_runtime_ready`, which owns the reasoning.
    const BOOT_READY_DEPTH: usize = 9;
    /// Fault reports, which may arrive in bursts while the track is cut.
    const FAULT_DEPTH: usize = 16;
    /// Display updates, coalesced by the display task.
    const DISPLAY_DEPTH: usize = 8;

    pub(crate) type SystemStatusChannel = RuntimeChannel<SystemStatusEvent, STATUS_DEPTH>;
    pub(crate) type SystemStatusSender = RuntimeSender<SystemStatusEvent, STATUS_DEPTH>;
    pub(crate) type SystemStatusReceiver = RuntimeReceiver<SystemStatusEvent, STATUS_DEPTH>;

    pub(crate) type NetStatusChannel = RuntimeChannel<SystemStatusEvent, NET_STATUS_DEPTH>;
    pub(crate) type NetStatusSender = RuntimeSender<SystemStatusEvent, NET_STATUS_DEPTH>;
    pub(crate) type NetStatusReceiver = RuntimeReceiver<SystemStatusEvent, NET_STATUS_DEPTH>;

    pub(crate) type BootReadyChannel = RuntimeChannel<BootReadyEvent, BOOT_READY_DEPTH>;
    pub(crate) type BootReadySender = RuntimeSender<BootReadyEvent, BOOT_READY_DEPTH>;
    pub(crate) type BootReadyReceiver = RuntimeReceiver<BootReadyEvent, BOOT_READY_DEPTH>;

    pub(crate) type FaultEventChannel = RuntimeChannel<FaultEvent, FAULT_DEPTH>;
    pub(crate) type FaultEventSender = RuntimeSender<FaultEvent, FAULT_DEPTH>;
    pub(crate) type FaultEventReceiver = RuntimeReceiver<FaultEvent, FAULT_DEPTH>;

    pub(crate) type DisplayChannel = RuntimeChannel<DisplayEvent, DISPLAY_DEPTH>;
    pub(crate) type DisplaySender = RuntimeSender<DisplayEvent, DISPLAY_DEPTH>;
    pub(crate) type DisplayReceiver = RuntimeReceiver<DisplayEvent, DISPLAY_DEPTH>;
}

/// Publishes one readiness acknowledgement and records it consistently.
///
/// This lives with the event it publishes rather than in the boot adapter:
/// `boot` already depends on every module that calls it, so putting it there
/// would close a cycle with each of them.
#[cfg(target_arch = "riscv32")]
pub(crate) async fn announce_ready(sender: channels::BootReadySender, event: BootReadyEvent) {
    sender.send(event).await;
    defmt::info!("boot: ready ack from {:?}", event);
}

#[cfg(test)]
mod tests {
    /// The first module-level example, as a test that actually runs.
    #[test]
    fn the_model_walks_boot_to_running_through_wifi() {
        let mut model = StatusModel::new();
        assert_eq!(model.led_state(), LedState::Booting);

        model.apply(SystemStatusEvent::BootCompleted);
        assert_eq!(model.led_state(), LedState::Running);

        model.apply(SystemStatusEvent::WifiConnecting);
        assert_eq!(model.led_state(), LedState::WifiConnecting);

        model.apply(SystemStatusEvent::WifiConnected);
        assert_eq!(model.led_state(), LedState::Running);
    }

    /// The second module-level example, as a test that actually runs.
    #[test]
    fn a_latched_fault_holds_the_led_until_it_is_cleared() {
        let mut model = StatusModel::new();
        model.apply(SystemStatusEvent::BootCompleted);

        model.apply(SystemStatusEvent::FaultLatched(FaultCause::TrackShort));
        assert_eq!(model.led_state(), LedState::FaultLatched);

        model.apply(SystemStatusEvent::FaultCleared);
        assert_eq!(model.led_state(), LedState::Running);
    }

    use super::*;
    use crate::application::StatusModel;

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

        model.apply(SystemStatusEvent::FaultLatched(FaultCause::Internal));
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

        model.apply(SystemStatusEvent::FaultLatched(FaultCause::Internal));
        assert!(model.fault_active());
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

    #[test]
    fn every_critical_task_has_a_distinct_name() {
        for (index, task) in CriticalTask::ALL.iter().enumerate() {
            for other in &CriticalTask::ALL[index + 1..] {
                assert_ne!(task.name(), other.name(), "{task:?} and {other:?}");
            }
            assert!(task.spawn_failure_message().ends_with(task.name()));
        }
    }
}
