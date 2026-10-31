//! Persistent WiFi provisioning state and the safe reboot request task.

use defmt::warn;
use esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN;
use esp_storage::FlashStorage;

use crate::control_buttons::{ProvisioningRequest, ProvisioningRequestReceiver};
use crate::flash::FlashGuard;
use crate::net::wifi_config::{EspWifiConfigStore, wifi_config_store_from_partition};
use dcc_provisioning::ProvisioningFlagStore;

use super::{BootError, WifiConfigInitError};

/// Locks the shared flash for the WiFi configuration store.
pub(super) async fn lock_flash() -> Result<FlashGuard, BootError> {
    crate::flash::lock()
        .await
        .ok_or_else(|| WifiConfigInitError::FlashUnavailable.into())
}

/// Open the `dcc_cfg` flash partition as the WiFi configuration store.
pub(super) fn open_wifi_config_store<'a, 'd>(
    flash: &'a mut FlashStorage<'d>,
    partition_table_buffer: &'a mut [u8; PARTITION_TABLE_MAX_LEN],
) -> Result<EspWifiConfigStore<'a, 'd>, BootError> {
    wifi_config_store_from_partition(flash, partition_table_buffer)
        .map_err(|error| WifiConfigInitError::Partition(error).into())
}

/// E-stop the track, persist the next-boot flag, then reboot into setup mode.
#[embassy_executor::task]
pub(super) async fn provisioning_request_task(
    partition_table_buffer: &'static mut [u8; PARTITION_TABLE_MAX_LEN],
    receiver: ProvisioningRequestReceiver,
) -> ! {
    loop {
        match receiver.receive().await {
            ProvisioningRequest::Requested => {
                // The update lock stops the track and keeps updates out
                // while the flag is written.
                let _interlock =
                    match crate::ota::acquire(dcc_ota::admission::Operation::Provisioning).await {
                        Ok(guard) => guard,
                        Err(error) => {
                            warn!("WiFi setup refused: {}", error.code());
                            continue;
                        }
                    };
                let Some(mut flash) = crate::flash::lock().await else {
                    defmt::error!("WiFi setup refused: flash not open");
                    continue;
                };
                warn!("WiFi provisioning requested; saving next-boot flag");
                match set_force_provisioning_on_next_boot(&mut flash, partition_table_buffer) {
                    Ok(()) => {
                        warn!("WiFi provisioning flag saved; rebooting");
                        crate::restart::restart();
                    }
                    Err(error) => {
                        defmt::error!("WiFi provisioning flag save failed: {}", error);
                    }
                }
            }
        }
    }
}

fn set_force_provisioning_on_next_boot(
    flash: &mut FlashStorage<'static>,
    partition_table_buffer: &mut [u8; PARTITION_TABLE_MAX_LEN],
) -> Result<(), WifiConfigInitError> {
    let mut store = wifi_config_store_from_partition(flash, partition_table_buffer)
        .map_err(WifiConfigInitError::Partition)?;
    store
        .set_force_on_next_boot()
        .map_err(WifiConfigInitError::Store)
}
