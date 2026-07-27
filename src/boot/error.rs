//! Boot failures and their externally visible recovery policy.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use crate::net::provisioning::ProvisioningApError;
use crate::net::udp_control::NetInitError;
use crate::net::wifi_config::{EspFlashStoreError, StoreError};
use crate::system_status::OptionalPeripheralInit;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum BootAction {
    Reset,
    DegradedMode,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum BootError {
    OptionalPeripheralInit(OptionalPeripheralInit),
    DccSelfCheck(DccSelfCheckError),
    CriticalHardwareInit(CriticalHardwareInit),
    CriticalTaskSpawn(CriticalTask),
    CriticalTaskInit(CriticalTaskInit),
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum DccSelfCheckError {
    IdlePacketEncoding,
    IdleWaveformBuild,
    ResetPacketEncoding,
    ShortAddress3Invalid,
    Speed28PacketEncoding,
    LongAddress1000Invalid,
    Speed128PacketEncoding,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CriticalHardwareInit {
    Rmt,
    RmtChannel0,
    RmtDriver,
    RailcomUart,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CriticalTask {
    Display,
    DccEngine,
    StatusLed,
    Scheduler,
    RailcomDiag,
    PomActor,
    PomCutoutMonitor,
    RailcomIsrCapture,
    RailcomUartDispatch,
    Net,
    FaultEffects,
    FaultManager,
    ProvisioningRequest,
    ProvisioningLed,
    StopButton,
    ResumeButton,
    ShortDetector,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CriticalTaskInit {
    Net(NetInitError),
    ProvisioningAp(ProvisioningApError),
    WifiConfig(WifiConfigInitError),
    FaultStateReceiverUnavailable,
    ReadinessTimeout,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum WifiConfigInitError {
    Partition(EspFlashStoreError),
    Store(StoreError),
}

impl WifiConfigInitError {
    const fn as_str(self) -> &'static str {
        match self {
            Self::Partition(_) => "WiFi config partition unavailable",
            Self::Store(_) => "WiFi config store operation failed",
        }
    }
}

pub type BootFailureChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, CriticalTaskInit, 4>;

impl BootError {
    pub const fn action(self) -> BootAction {
        match self {
            Self::OptionalPeripheralInit(_) => BootAction::DegradedMode,
            Self::DccSelfCheck(_)
            | Self::CriticalHardwareInit(_)
            | Self::CriticalTaskSpawn(_)
            | Self::CriticalTaskInit(_) => BootAction::Reset,
        }
    }

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
            Self::CriticalHardwareInit(CriticalHardwareInit::RmtDriver) => {
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
            Self::CriticalTaskInit(CriticalTaskInit::Net(error)) => error.as_str(),
            Self::CriticalTaskInit(CriticalTaskInit::ProvisioningAp(error)) => error.as_str(),
            Self::CriticalTaskInit(CriticalTaskInit::WifiConfig(error)) => error.as_str(),
            Self::CriticalTaskInit(CriticalTaskInit::ReadinessTimeout) => {
                "critical task readiness timeout"
            }
        }
    }
}

pub(super) fn log_degraded_boot_error(error: BootError) {
    defmt::warn!("boot: degraded mode: {}", error.message());
}
