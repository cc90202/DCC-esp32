//! Wi-Fi configuration storage on the ESP32 flash.
//!
//! The record format, the boot decision and the validation live in
//! `dcc_provisioning`; this module only opens the partition that holds them.

mod esp_flash_store;

pub use esp_flash_store::{
    EspFlashStoreError, EspWifiConfigStore, EspWifiConfigStoreError,
    wifi_config_store_from_partition,
};
