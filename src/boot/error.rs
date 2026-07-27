//! Boot failures and their externally visible recovery policy.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use crate::net::provisioning::ProvisioningApError;
use crate::net::udp_control::NetInitError;
use crate::net::wifi_config::{EspFlashStoreError, StoreError};
use crate::rmt_dcc::InitError as RmtInitError;
use crate::system_status::OptionalPeripheralInit;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
/// Recovery policy selected for a boot failure.
pub enum BootAction {
    /// Restart because a critical invariant or service is unavailable.
    Reset,
    /// Continue with the affected optional capability disabled.
    DegradedMode,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
/// Failure detected while assembling or starting the firmware runtime.
pub enum BootError {
    /// An optional peripheral could not be initialized.
    OptionalPeripheralInit(OptionalPeripheralInit),
    /// The host-independent DCC startup self-check failed.
    DccSelfCheck(DccSelfCheckError),
    /// Hardware required for safe operation could not be initialized.
    CriticalHardwareInit(CriticalHardwareInit),
    /// A required runtime task could not be spawned.
    CriticalTaskSpawn(CriticalTask),
    /// A spawned task reported that its initialization failed.
    CriticalTaskInit(CriticalTaskInit),
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
/// Invariant that failed during the DCC startup self-check.
pub enum DccSelfCheckError {
    /// The standard idle packet could not be encoded.
    IdlePacketEncoding,
    /// The encoded idle packet could not be converted to an RMT waveform.
    IdleWaveformBuild,
    /// The standard reset packet could not be encoded.
    ResetPacketEncoding,
    /// The known-valid short-address fixture was rejected.
    ShortAddress3Invalid,
    /// The 28-step speed fixture could not be encoded.
    Speed28PacketEncoding,
    /// The known-valid long-address fixture was rejected.
    LongAddress1000Invalid,
    /// The 128-step speed fixture could not be encoded.
    Speed128PacketEncoding,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
/// Critical hardware resource that failed during boot.
pub enum CriticalHardwareInit {
    /// The ESP RMT peripheral could not be initialized.
    Rmt,
    /// RMT channel zero could not be configured.
    RmtChannel0,
    /// The DCC RMT driver rejected its configuration.
    RmtDriver(RmtInitError),
    /// The RailCom receive UART could not be initialized.
    RailcomUart,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
/// Runtime task whose spawn operation failed.
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
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
/// Initialization failure reported by a running critical task.
pub enum CriticalTaskInit {
    /// Network initialization failed.
    Net(NetInitError),
    /// The provisioning access point could not be started.
    ProvisioningAp(ProvisioningApError),
    /// Persistent Wi-Fi configuration could not be initialized.
    WifiConfig(WifiConfigInitError),
    /// The single fault-state receiver had already been acquired.
    FaultStateReceiverUnavailable,
    /// Not all critical tasks acknowledged readiness before the deadline.
    ReadinessTimeout,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
/// Failure while opening or reading persistent Wi-Fi configuration.
pub enum WifiConfigInitError {
    /// The flash partition backing the store could not be opened.
    Partition(EspFlashStoreError),
    /// The configuration store could not be read.
    Store(StoreError),
}

/// Channel used by critical tasks to report asynchronous initialization failures.
pub type BootFailureChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, CriticalTaskInit, 4>;

impl BootError {
    /// Select the recovery action associated with this failure.
    pub const fn action(self) -> BootAction {
        match self {
            Self::OptionalPeripheralInit(_) => BootAction::DegradedMode,
            Self::DccSelfCheck(_)
            | Self::CriticalHardwareInit(_)
            | Self::CriticalTaskSpawn(_)
            | Self::CriticalTaskInit(_) => BootAction::Reset,
        }
    }

    /// Return the stable, user-facing diagnostic for this failure.
    pub const fn message(self) -> &'static str {
        match self {
            Self::OptionalPeripheralInit(OptionalPeripheralInit::DisplayI2c) => {
                "display I2C init failed"
            }
            Self::OptionalPeripheralInit(OptionalPeripheralInit::DisplayInit) => {
                "display SSD1306 init failed"
            }
            Self::OptionalPeripheralInit(OptionalPeripheralInit::DisplayUnavailable) => {
                "display disabled"
            }
            Self::DccSelfCheck(DccSelfCheckError::IdlePacketEncoding) => {
                "idle packet encoding failed"
            }
            Self::DccSelfCheck(DccSelfCheckError::IdleWaveformBuild) => {
                "idle waveform build failed"
            }
            Self::DccSelfCheck(DccSelfCheckError::ResetPacketEncoding) => {
                "reset packet encoding failed"
            }
            Self::DccSelfCheck(DccSelfCheckError::ShortAddress3Invalid) => {
                "short address 3 must be valid"
            }
            Self::DccSelfCheck(DccSelfCheckError::Speed28PacketEncoding) => {
                "speed28 packet encoding failed"
            }
            Self::DccSelfCheck(DccSelfCheckError::LongAddress1000Invalid) => {
                "long address 1000 must be valid"
            }
            Self::DccSelfCheck(DccSelfCheckError::Speed128PacketEncoding) => {
                "speed128 packet encoding failed"
            }
            Self::CriticalHardwareInit(CriticalHardwareInit::Rmt) => "RMT init failed",
            Self::CriticalHardwareInit(CriticalHardwareInit::RmtChannel0) => {
                "RMT channel0 configure failed"
            }
            Self::CriticalHardwareInit(CriticalHardwareInit::RmtDriver(_)) => {
                "RMT ISR driver init failed"
            }
            Self::CriticalHardwareInit(CriticalHardwareInit::RailcomUart) => {
                "RailCom UART RX init failed"
            }
            Self::CriticalTaskSpawn(CriticalTask::Display) => "failed to spawn display_task",
            Self::CriticalTaskSpawn(CriticalTask::DccEngine) => "failed to spawn dcc_engine_task",
            Self::CriticalTaskSpawn(CriticalTask::StatusLed) => "failed to spawn status_led_task",
            Self::CriticalTaskSpawn(CriticalTask::Scheduler) => "failed to spawn scheduler_task",
            Self::CriticalTaskSpawn(CriticalTask::RailcomDiag) => {
                "failed to spawn railcom_diag_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::PomActor) => "failed to spawn pom_actor_task",
            Self::CriticalTaskSpawn(CriticalTask::PomCutoutMonitor) => {
                "failed to spawn pom_cutout_monitor_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::RailcomIsrCapture) => {
                "failed to spawn railcom_isr_capture_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::RailcomUartDispatch) => {
                "failed to spawn railcom_uart_runtime_dispatch_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::Net) => "failed to spawn net_task",
            Self::CriticalTaskSpawn(CriticalTask::FaultEffects) => {
                "failed to spawn fault_effects_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::FaultManager) => {
                "failed to spawn fault_manager_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::ProvisioningRequest) => {
                "failed to spawn provisioning_request_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::ProvisioningLed) => {
                "failed to spawn provisioning_led_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::StopButton) => "failed to spawn stop_button_task",
            Self::CriticalTaskSpawn(CriticalTask::ResumeButton) => {
                "failed to spawn resume_button_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::ShortDetector) => {
                "failed to spawn short_detector_task"
            }
            Self::CriticalTaskInit(CriticalTaskInit::FaultStateReceiverUnavailable) => {
                "fault-state watch receiver already taken"
            }
            Self::CriticalTaskInit(CriticalTaskInit::Net(_)) => "network initialization failed",
            Self::CriticalTaskInit(CriticalTaskInit::ProvisioningAp(_)) => {
                "provisioning access point initialization failed"
            }
            Self::CriticalTaskInit(CriticalTaskInit::WifiConfig(_)) => {
                "WiFi configuration initialization failed"
            }
            Self::CriticalTaskInit(CriticalTaskInit::ReadinessTimeout) => {
                "critical task readiness timeout"
            }
        }
    }
}

pub(super) fn log_degraded_boot_error(error: BootError) {
    defmt::warn!("boot: degraded mode: {}", error.message());
}
