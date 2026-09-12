//! Shared esp-radio controller ownership and WiFi bring-up.

use core::fmt;

use esp_radio::wifi::{Interfaces, ModeConfig, WifiController};
use static_cell::StaticCell;

// Controller must be 'static so WifiController and WifiDevice are 'static too.
// `init_controller` is called at most once per boot: station mode and
// provisioning AP mode are mutually exclusive and separated by a reboot.
static RADIO_CONTROLLER: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();

#[derive(Debug, Clone, Copy)]
pub struct RadioInitError(esp_radio::InitializationError);

#[cfg(target_arch = "riscv32")]
impl defmt::Format for RadioInitError {
    fn format(&self, formatter: defmt::Formatter) {
        defmt::write!(
            formatter,
            "esp-radio initialization failed: {:?}",
            defmt::Debug2Format(&self.0)
        );
    }
}

impl From<esp_radio::InitializationError> for RadioInitError {
    fn from(error: esp_radio::InitializationError) -> Self {
        Self(error)
    }
}

impl fmt::Display for RadioInitError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "esp-radio initialization failed: {}", self.0)
    }
}

impl core::error::Error for RadioInitError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        Some(&self.0)
    }
}

/// Error for the WiFi bring-up sequence shared by station and AP mode.
#[derive(Debug, Clone, Copy)]
pub enum WifiBringupError {
    EspRadioInit(RadioInitError),
    WifiInit(esp_radio::wifi::WifiError),
    WifiSetConfig(esp_radio::wifi::WifiError),
    WifiStart(esp_radio::wifi::WifiError),
}

#[cfg(target_arch = "riscv32")]
impl defmt::Format for WifiBringupError {
    fn format(&self, formatter: defmt::Formatter) {
        match self {
            Self::EspRadioInit(error) => defmt::write!(formatter, "{:?}", error),
            Self::WifiInit(error) => defmt::write!(
                formatter,
                "WiFi init failed: {:?}",
                defmt::Debug2Format(error)
            ),
            Self::WifiSetConfig(error) => defmt::write!(
                formatter,
                "WiFi set_config failed: {:?}",
                defmt::Debug2Format(error)
            ),
            Self::WifiStart(error) => defmt::write!(
                formatter,
                "WiFi start failed: {:?}",
                defmt::Debug2Format(error)
            ),
        }
    }
}

impl fmt::Display for WifiBringupError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EspRadioInit(error) => write!(formatter, "esp-radio init failed: {error}"),
            Self::WifiInit(error) => write!(formatter, "WiFi init failed: {error}"),
            Self::WifiSetConfig(error) => write!(formatter, "WiFi set_config failed: {error}"),
            Self::WifiStart(error) => write!(formatter, "WiFi start failed: {error}"),
        }
    }
}

impl core::error::Error for WifiBringupError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::EspRadioInit(error) => Some(error),
            Self::WifiInit(error) | Self::WifiSetConfig(error) | Self::WifiStart(error) => {
                Some(error)
            }
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
                WifiBringupError::WifiInit(error)
            },
        )?;

    wifi_ctrl.set_config(mode_config).map_err(|error| {
        defmt::error!(
            "WiFi configuration failed: {:?}",
            defmt::Debug2Format(&error)
        );
        WifiBringupError::WifiSetConfig(error)
    })?;
    wifi_ctrl.start_async().await.map_err(|error| {
        defmt::error!("WiFi start failed: {:?}", defmt::Debug2Format(&error));
        WifiBringupError::WifiStart(error)
    })?;

    Ok((wifi_ctrl, interfaces))
}
