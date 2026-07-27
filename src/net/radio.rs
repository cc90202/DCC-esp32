//! Shared esp-radio controller ownership and WiFi bring-up.

use core::fmt;

use esp_radio::wifi::{Interfaces, ModeConfig, WifiController};
use static_cell::StaticCell;

// Controller must be 'static so WifiController and WifiDevice are 'static too.
// `init_controller` is called at most once per boot: station mode and
// provisioning AP mode are mutually exclusive and separated by a reboot.
static RADIO_CONTROLLER: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RadioInitError {
    General(i32),
    WifiDriver,
    WrongClockConfig,
    InterruptsDisabled,
    SchedulerNotInitialized,
    Unknown,
}

impl From<esp_radio::InitializationError> for RadioInitError {
    fn from(error: esp_radio::InitializationError) -> Self {
        match error {
            esp_radio::InitializationError::General(code) => Self::General(code),
            esp_radio::InitializationError::WifiError(_) => Self::WifiDriver,
            esp_radio::InitializationError::WrongClockConfig => Self::WrongClockConfig,
            esp_radio::InitializationError::InterruptsDisabled => Self::InterruptsDisabled,
            esp_radio::InitializationError::SchedulerNotInitialized => {
                Self::SchedulerNotInitialized
            }
            _ => Self::Unknown,
        }
    }
}

impl fmt::Display for RadioInitError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::General(code) => write!(formatter, "general esp-radio error {code}"),
            Self::WifiDriver => formatter.write_str("esp-radio WiFi driver initialization failed"),
            Self::WrongClockConfig => {
                formatter.write_str("esp-radio clock configuration is invalid")
            }
            Self::InterruptsDisabled => formatter.write_str("esp-radio interrupts are disabled"),
            Self::SchedulerNotInitialized => {
                formatter.write_str("esp-radio scheduler is not initialized")
            }
            Self::Unknown => formatter.write_str("unknown esp-radio initialization error"),
        }
    }
}

/// Error for the WiFi bring-up sequence shared by station and AP mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum WifiBringupError {
    EspRadioInit(RadioInitError),
    WifiInit,
    WifiSetConfig,
    WifiStart,
}

impl fmt::Display for WifiBringupError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EspRadioInit(error) => write!(formatter, "esp-radio init failed: {error}"),
            Self::WifiInit => formatter.write_str("WiFi init failed"),
            Self::WifiSetConfig => formatter.write_str("WiFi set_config failed"),
            Self::WifiStart => formatter.write_str("WiFi start failed"),
        }
    }
}

fn init_controller() -> Result<&'static mut esp_radio::Controller<'static>, RadioInitError> {
    let controller = esp_radio::init().map_err(RadioInitError::from)?;
    Ok(RADIO_CONTROLLER.init(controller))
}

/// Initialize esp-radio and start WiFi with the given mode configuration.
///
/// The returned controller must be kept alive for as long as WiFi is used;
/// dropping it stops the radio.
pub(super) async fn start_wifi(
    wifi: esp_hal::peripherals::WIFI<'static>,
    mode_config: &ModeConfig,
) -> Result<(WifiController<'static>, Interfaces<'static>), WifiBringupError> {
    let controller = init_controller().map_err(|error| {
        defmt::error!("esp-radio initialization failed: {:?}", error);
        WifiBringupError::EspRadioInit(error)
    })?;
    let (mut wifi_ctrl, interfaces) =
        esp_radio::wifi::new(controller, wifi, esp_radio::wifi::Config::default()).map_err(
            |error| {
                defmt::error!(
                    "WiFi driver initialization failed: {:?}",
                    defmt::Debug2Format(&error)
                );
                WifiBringupError::WifiInit
            },
        )?;

    wifi_ctrl.set_config(mode_config).map_err(|error| {
        defmt::error!(
            "WiFi configuration failed: {:?}",
            defmt::Debug2Format(&error)
        );
        WifiBringupError::WifiSetConfig
    })?;
    wifi_ctrl.start_async().await.map_err(|error| {
        defmt::error!("WiFi start failed: {:?}", defmt::Debug2Format(&error));
        WifiBringupError::WifiStart
    })?;

    Ok((wifi_ctrl, interfaces))
}
