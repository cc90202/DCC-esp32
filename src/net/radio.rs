//! Shared esp-radio controller ownership.

use static_cell::StaticCell;

// Controller must be 'static so WifiController and WifiDevice are 'static too.
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

pub(super) fn init_controller()
-> Result<&'static mut esp_radio::Controller<'static>, RadioInitError> {
    let controller = esp_radio::init().map_err(RadioInitError::from)?;
    Ok(RADIO_CONTROLLER.init(controller))
}
