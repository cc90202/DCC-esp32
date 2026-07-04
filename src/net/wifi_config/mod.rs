//! Host-testable WiFi provisioning policy.
//!
//! This module owns validation and boot decisions only. It deliberately does
//! not depend on `esp-radio`, flash/NVS, GPIO, HTTP, or display code.

mod credentials;
#[cfg(target_arch = "riscv32")]
mod esp_flash_store;
mod provisioning_policy;
mod store;

pub use credentials::{CredentialError, WifiCredentials};
#[cfg(target_arch = "riscv32")]
pub use esp_flash_store::{
    EspFlashStoreError, EspWifiConfigStore, wifi_config_store_from_partition,
};
pub use provisioning_policy::{
    ProvisioningDecision, ProvisioningReason, StoredCredentialsState, decide_provisioning,
};
pub use store::{ProvisioningFlagStore, StoreError, WifiConfigStore, WifiCredentialsStore};
