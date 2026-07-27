//! Persistent WiFi provisioning state and the safe reboot request task.

use defmt::warn;
use embassy_time::{Duration, Timer};
use esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN;
use esp_storage::FlashStorage;

use crate::control_buttons::ProvisioningRequest;
use crate::net::wifi_config::{
    EspWifiConfigStore, ProvisioningFlagStore, wifi_config_store_from_partition,
};
use crate::runtime_channels::{FaultEventSender, RuntimeReceiver};
use crate::system_status::FaultEvent;

use super::{BootError, CriticalTaskInit, WifiConfigInitError};

/// Open the `dcc_cfg` flash partition as the WiFi configuration store.
pub(super) fn open_wifi_config_store<'a, 'd>(
    flash: &'a mut FlashStorage<'d>,
    partition_table_buffer: &'a mut [u8; PARTITION_TABLE_MAX_LEN],
) -> Result<EspWifiConfigStore<'a, 'd>, BootError> {
    wifi_config_store_from_partition(flash, partition_table_buffer).map_err(|error| {
        BootError::CriticalTaskInit(CriticalTaskInit::WifiConfig(
            WifiConfigInitError::Partition(error),
        ))
    })
}

const PROVISIONING_TRACK_DISABLE_GRACE: Duration = Duration::from_millis(100);

/// E-stop the track, persist the next-boot flag, then reboot into setup mode.
#[embassy_executor::task]
pub(super) async fn provisioning_request_task(
    mut flash: FlashStorage<'static>,
    partition_table_buffer: &'static mut [u8; PARTITION_TABLE_MAX_LEN],
    fault_sender: FaultEventSender,
    receiver: RuntimeReceiver<ProvisioningRequest, 1>,
) -> ! {
    loop {
        match receiver.receive().await {
            ProvisioningRequest::Requested => {
                crate::track_safety::disable_track_intentionally();
                fault_sender.send(FaultEvent::StopPressed).await;
                Timer::after(PROVISIONING_TRACK_DISABLE_GRACE).await;
                warn!("WiFi provisioning requested; saving next-boot flag");
                match set_force_provisioning_on_next_boot(&mut flash, partition_table_buffer) {
                    Ok(()) => {
                        warn!("WiFi provisioning flag saved; rebooting");
                        esp_hal::system::software_reset();
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
