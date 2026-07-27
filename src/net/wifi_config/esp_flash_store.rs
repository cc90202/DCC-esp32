//! ESP32-C6 flash adapter for the WiFi configuration store.

use core::fmt;

use esp_bootloader_esp_idf::partitions::{
    FlashRegion, PARTITION_TABLE_MAX_LEN, read_partition_table,
};
use esp_storage::FlashStorage;

use super::store::WifiConfigStore;

pub(crate) const DCC_CFG_PARTITION_LABEL: &str = "dcc_cfg";
// 3 x 4 KiB sectors: slot A + slot B used by `WifiConfigStore`, plus one
// spare sector reserved for future config records (see partitions.csv).
const DCC_CFG_PARTITION_SIZE: usize = 0x3000;

pub type EspWifiConfigStore<'a, 'd> = WifiConfigStore<FlashRegion<'a, FlashStorage<'d>>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum EspFlashStoreError {
    PartitionTable,
    MissingPartition,
    InvalidPartitionSize,
}

impl fmt::Display for EspFlashStoreError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PartitionTable => formatter.write_str("failed to read the partition table"),
            Self::MissingPartition => formatter.write_str("WiFi configuration partition missing"),
            Self::InvalidPartitionSize => {
                formatter.write_str("WiFi configuration partition has an invalid size")
            }
        }
    }
}

pub fn wifi_config_store_from_partition<'a, 'd>(
    flash: &'a mut FlashStorage<'d>,
    partition_table_buffer: &'a mut [u8; PARTITION_TABLE_MAX_LEN],
) -> Result<EspWifiConfigStore<'a, 'd>, EspFlashStoreError> {
    let partition_table = read_partition_table(flash, partition_table_buffer)
        .map_err(|_| EspFlashStoreError::PartitionTable)?;
    let partition = partition_table
        .iter()
        .find(|entry| entry.label_as_str() == DCC_CFG_PARTITION_LABEL)
        .ok_or(EspFlashStoreError::MissingPartition)?;

    if partition.len() as usize != DCC_CFG_PARTITION_SIZE {
        return Err(EspFlashStoreError::InvalidPartitionSize);
    }

    Ok(WifiConfigStore::new(partition.as_embedded_storage(flash)))
}
