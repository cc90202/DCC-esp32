//! Wi-Fi provisioning for the DCC command station, without a HAL.
//!
//! When the station has no usable credentials it opens its own access point
//! and serves a setup page; this crate holds every decision of that flow, and
//! the firmware only adds the radio, the sockets and the flash partition:
//!
//! - [`decide_provisioning`] and [`load_wifi_credentials_and_decision`] choose
//!   at boot between joining the stored network and opening the access point;
//! - [`WifiConfigStore`] keeps the credentials and the setup flag in two flash
//!   sectors through [`embedded_storage::nor_flash::NorFlash`];
//! - [`http_parser`] reads the requests of the setup page, whose content is in
//!   [`html`];
//! - [`dhcp_codec`] answers the DHCP requests of the phone that joins the
//!   access point, with the addresses in [`net_config`].
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod boot_decision;
mod credentials;
pub mod dhcp_codec;
pub mod html;
pub mod http_parser;
pub mod net_config;
mod provisioning_policy;
mod store;

pub use boot_decision::{SetupFlag, load_wifi_credentials_and_decision};
pub use credentials::{CredentialError, WifiCredentials};
pub use provisioning_policy::{
    ProvisioningDecision, ProvisioningReason, StoredCredentialsState, decide_provisioning,
};
pub use store::{
    ProvisioningFlagStore, STORE_SPAN, StoreError, WifiConfigStore, WifiCredentialsStore,
};
