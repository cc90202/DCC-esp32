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
//! use crate::station_status::{LedState, SystemStatusEvent};
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
//! use crate::station_status::{FaultCause, LedState, SystemStatusEvent};
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

/// Runtime task whose spawn operation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CriticalTask {
    /// Display rendering task.
    Display,

    /// DCC waveform engine task.
    DccEngine,

    /// Status LED task.
    StatusLed,

    /// DCC packet scheduler task.
    Scheduler,

    /// RailCom diagnostics task.
    RailcomDiag,

    /// Programming-on-main actor task.
    PomActor,

    /// Programming-on-main cutout monitor task.
    PomCutoutMonitor,

    /// RailCom ISR capture task.
    RailcomIsrCapture,

    /// RailCom UART dispatch task.
    RailcomUartDispatch,

    /// Network task.
    Net,

    /// Fault side-effects task.
    FaultEffects,

    /// Fault manager task.
    FaultManager,

    /// Runtime provisioning request task.
    ProvisioningRequest,

    /// Provisioning status LED task.
    ProvisioningLed,

    /// Emergency-stop button task.
    StopButton,

    /// Resume button task.
    ResumeButton,

    /// Track short-circuit detector task.
    ShortDetector,

    /// Controller lease watchdog task.
    LeaseWatchdog,
}

impl CriticalTask {
    /// Diagnostic reported when spawning this task fails, naming the Embassy
    /// task function.
    #[must_use]
    pub const fn spawn_failure_message(self) -> &'static str {
        match self {
            Self::Display => "failed to spawn display_task",
            Self::DccEngine => "failed to spawn dcc_engine_task",
            Self::StatusLed => "failed to spawn status_led_task",
            Self::Scheduler => "failed to spawn scheduler_task",
            Self::RailcomDiag => "failed to spawn railcom_diag_task",
            Self::PomActor => "failed to spawn pom_actor_task",
            Self::PomCutoutMonitor => "failed to spawn pom_cutout_monitor_task",
            Self::RailcomIsrCapture => "failed to spawn railcom_isr_capture_task",
            Self::RailcomUartDispatch => "failed to spawn railcom_uart_runtime_dispatch_task",
            Self::Net => "failed to spawn net_task",
            Self::FaultEffects => "failed to spawn fault_effects_task",
            Self::FaultManager => "failed to spawn fault_manager_task",
            Self::ProvisioningRequest => "failed to spawn provisioning_request_task",
            Self::ProvisioningLed => "failed to spawn provisioning_led_task",
            Self::StopButton => "failed to spawn stop_button_task",
            Self::ResumeButton => "failed to spawn resume_button_task",
            Self::ShortDetector => "failed to spawn short_detector_task",
            Self::LeaseWatchdog => "failed to spawn client_watchdog_task",
        }
    }
}

impl core::fmt::Display for CriticalTask {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str(self.spawn_failure_message())
    }
}

impl core::error::Error for CriticalTask {}

// --- Fault manager events ---
//
// `FaultEvent` and the status enums live in `station_status` (rather than in
// `fault_manager.rs`) because they are imported by task modules across the
// crate (dcc engine, boot, short detector, control buttons, net) that must
// not depend on `fault_manager` itself, and by the application policies that
// must not depend on this hub. They are re-exported here for the tasks.
//
// This hub is not dependency-free in the strict sense: the `channels`
// submodule below names Embassy types and `announce_ready` logs. Both are
// gated to the target, and neither points back at a module that imports this
// one, so the direction that matters is unchanged.

use crate::station_status::{FaultCause, LedState};

/// Name of the provisioning access point: a fixed prefix and four hex digits
/// of the MAC address; `net::provisioning::ap` checks the prefix length.
pub const PROVISIONING_SSID_LEN: usize = 14;

/// Bounded text of the provisioning access point name.
pub type ProvisioningSsid = heapless::String<PROVISIONING_SSID_LEN>;

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
        ssid: ProvisioningSsid,
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
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch};

    use super::{BootReadyEvent, DisplayEvent};
    use crate::runtime_channels::{RuntimeChannel, RuntimeReceiver, RuntimeSender};
    use crate::station_status::{FaultEvent, FaultManagerState, SystemStatusEvent};

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

    /// Receivers allowed on the fault-state watch; the short detector is the only one.
    const FAULT_STATE_RECEIVERS: usize = 1;

    /// Latest fault-manager state, published by the fault manager.
    pub(crate) type FaultStateWatch =
        watch::Watch<CriticalSectionRawMutex, FaultManagerState, FAULT_STATE_RECEIVERS>;
    pub(crate) type FaultStateSender =
        watch::Sender<'static, CriticalSectionRawMutex, FaultManagerState, FAULT_STATE_RECEIVERS>;
    pub(crate) type FaultStateReceiver =
        watch::Receiver<'static, CriticalSectionRawMutex, FaultManagerState, FAULT_STATE_RECEIVERS>;
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
    use crate::application::StatusModel;
    use crate::station_status::{FaultCause, LedState, SystemStatusEvent};

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
}
