//! Shared esp-radio controller ownership and WiFi bring-up.

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

/// Error for the WiFi bring-up sequence shared by station and AP mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum WifiBringupError {
    EspRadioInit(RadioInitError),
    WifiInit,
    WifiSetConfig,
    WifiStart,
}

impl WifiBringupError {
    pub(crate) const fn as_str(self) -> &'static str {
        match self {
            Self::EspRadioInit(_) => "esp-radio init failed",
            Self::WifiInit => "WiFi init failed",
            Self::WifiSetConfig => "WiFi set_config failed",
            Self::WifiStart => "WiFi start failed",
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
    let controller = init_controller().map_err(WifiBringupError::EspRadioInit)?;
    let (mut wifi_ctrl, interfaces) =
        esp_radio::wifi::new(controller, wifi, esp_radio::wifi::Config::default())
            .map_err(|_| WifiBringupError::WifiInit)?;

    wifi_ctrl
        .set_config(mode_config)
        .map_err(|_| WifiBringupError::WifiSetConfig)?;
    wifi_ctrl
        .start_async()
        .await
        .map_err(|_| WifiBringupError::WifiStart)?;

    Ok((wifi_ctrl, interfaces))
}
