//! Boot failures and their externally visible recovery policy.

use core::fmt;

use crate::net::NetInitError;
use crate::net::provisioning::ProvisioningApError;
use crate::net::wifi_config::{EspFlashStoreError, EspWifiConfigStoreError};
use crate::rmt_dcc::InitError as RmtInitError;
use crate::system_status::{CriticalTask, OptionalPeripheralInit};

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
    /// The encoded idle packet could not be converted to an RMT waveform.
    IdleWaveformBuild,
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
    Store(EspWifiConfigStoreError),
    /// The shared flash handle was not open; a boot-order bug.
    FlashUnavailable,
}

/// One slot per critical task that can report an asynchronous failure.
const BOOT_FAILURE_DEPTH: usize = 4;

/// Channel used by critical tasks to report asynchronous initialization failures.
pub(crate) type BootFailureChannel =
    crate::runtime_channels::RuntimeChannel<CriticalTaskInit, BOOT_FAILURE_DEPTH>;
pub(crate) type CriticalTaskInitSender =
    crate::runtime_channels::RuntimeSender<CriticalTaskInit, BOOT_FAILURE_DEPTH>;
pub(crate) type CriticalTaskInitReceiver =
    crate::runtime_channels::RuntimeReceiver<CriticalTaskInit, BOOT_FAILURE_DEPTH>;

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
            Self::OptionalPeripheralInit(peripheral) => degraded_peripheral_message(peripheral),
            Self::DccSelfCheck(error) => error.message(),
            Self::CriticalHardwareInit(error) => error.message(),
            Self::CriticalTaskSpawn(task) => task.spawn_failure_message(),
            Self::CriticalTaskInit(error) => error.message(),
        }
    }
}

/// Boot diagnostic for an optional peripheral that left the system degraded.
const fn degraded_peripheral_message(peripheral: OptionalPeripheralInit) -> &'static str {
    match peripheral {
        OptionalPeripheralInit::DisplayI2c => "display I2C init failed",
        OptionalPeripheralInit::DisplayInit => "display SSD1306 init failed",
        OptionalPeripheralInit::DisplayUnavailable => "display disabled",
    }
}

impl DccSelfCheckError {
    /// Return the stable diagnostic for this self-check failure.
    pub const fn message(self) -> &'static str {
        match self {
            Self::IdleWaveformBuild => "idle waveform build failed",
            Self::ShortAddress3Invalid => "short address 3 must be valid",
            Self::Speed28PacketEncoding => "speed28 packet encoding failed",
            Self::LongAddress1000Invalid => "long address 1000 must be valid",
            Self::Speed128PacketEncoding => "speed128 packet encoding failed",
        }
    }
}

impl CriticalHardwareInit {
    /// Return the stable diagnostic for this hardware failure, without its source.
    pub const fn message(self) -> &'static str {
        match self {
            Self::Rmt => "RMT init failed",
            Self::RmtChannel0 => "RMT channel0 configure failed",
            Self::RmtDriver(_) => "RMT ISR driver init failed",
            Self::RailcomUart => "RailCom UART RX init failed",
        }
    }
}

impl CriticalTaskInit {
    /// Return the stable diagnostic for this task failure, without its source.
    pub const fn message(self) -> &'static str {
        match self {
            Self::Net(_) => "network initialization failed",
            Self::ProvisioningAp(_) => "provisioning access point initialization failed",
            Self::WifiConfig(_) => "WiFi configuration initialization failed",
            Self::FaultStateReceiverUnavailable => "fault-state watch receiver already taken",
            Self::ReadinessTimeout => "critical task readiness timeout",
        }
    }
}

impl From<OptionalPeripheralInit> for BootError {
    fn from(peripheral: OptionalPeripheralInit) -> Self {
        Self::OptionalPeripheralInit(peripheral)
    }
}

impl From<DccSelfCheckError> for BootError {
    fn from(error: DccSelfCheckError) -> Self {
        Self::DccSelfCheck(error)
    }
}

impl From<CriticalHardwareInit> for BootError {
    fn from(error: CriticalHardwareInit) -> Self {
        Self::CriticalHardwareInit(error)
    }
}

impl From<CriticalTaskInit> for BootError {
    fn from(error: CriticalTaskInit) -> Self {
        Self::CriticalTaskInit(error)
    }
}

impl From<WifiConfigInitError> for CriticalTaskInit {
    fn from(error: WifiConfigInitError) -> Self {
        Self::WifiConfig(error)
    }
}

impl From<WifiConfigInitError> for BootError {
    fn from(error: WifiConfigInitError) -> Self {
        Self::CriticalTaskInit(error.into())
    }
}

impl fmt::Display for BootError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.message())
    }
}

impl core::error::Error for BootError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::OptionalPeripheralInit(error) => Some(error),
            Self::DccSelfCheck(error) => Some(error),
            Self::CriticalHardwareInit(error) => Some(error),
            Self::CriticalTaskSpawn(error) => Some(error),
            Self::CriticalTaskInit(error) => Some(error),
        }
    }
}

impl fmt::Display for DccSelfCheckError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.message())
    }
}

impl core::error::Error for DccSelfCheckError {}

impl fmt::Display for CriticalHardwareInit {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::RmtDriver(error) => write!(formatter, "{}: {error}", self.message()),
            Self::Rmt | Self::RmtChannel0 | Self::RailcomUart => {
                formatter.write_str(self.message())
            }
        }
    }
}

impl core::error::Error for CriticalHardwareInit {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::RmtDriver(error) => Some(error),
            Self::Rmt | Self::RmtChannel0 | Self::RailcomUart => None,
        }
    }
}

impl fmt::Display for CriticalTaskInit {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Net(error) => write!(formatter, "{}: {error}", self.message()),
            Self::ProvisioningAp(error) => write!(formatter, "{}: {error}", self.message()),
            Self::WifiConfig(error) => write!(formatter, "{}: {error}", self.message()),
            Self::FaultStateReceiverUnavailable | Self::ReadinessTimeout => {
                formatter.write_str(self.message())
            }
        }
    }
}

impl core::error::Error for CriticalTaskInit {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Net(error) => Some(error),
            Self::ProvisioningAp(error) => Some(error),
            Self::WifiConfig(error) => Some(error),
            Self::FaultStateReceiverUnavailable | Self::ReadinessTimeout => None,
        }
    }
}

impl WifiConfigInitError {
    /// Return the stable diagnostic for this configuration failure, without its source.
    pub const fn message(self) -> &'static str {
        match self {
            Self::Partition(_) => "WiFi flash partition failed",
            Self::Store(_) => "WiFi configuration store failed",
            Self::FlashUnavailable => "flash not open before WiFi setup",
        }
    }
}

impl fmt::Display for WifiConfigInitError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Partition(error) => write!(formatter, "{}: {error}", self.message()),
            Self::Store(error) => write!(formatter, "{}: {error}", self.message()),
            Self::FlashUnavailable => formatter.write_str(self.message()),
        }
    }
}

impl core::error::Error for WifiConfigInitError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Partition(error) => Some(error),
            Self::Store(error) => Some(error),
            Self::FlashUnavailable => None,
        }
    }
}

pub(super) fn log_degraded_boot_error(error: BootError) {
    defmt::warn!("boot: degraded mode: {}", error.message());
}
