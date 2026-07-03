//! Host-testable WiFi provisioning policy.
//!
//! This module owns validation and boot decisions only. It deliberately does
//! not depend on `esp-radio`, flash/NVS, GPIO, HTTP, or display code.

mod credentials;
mod provisioning_policy;

pub use credentials::{CredentialError, WifiCredentials};
pub use provisioning_policy::{
    ProvisioningDecision, ProvisioningReason, StoredCredentialsState, decide_provisioning,
};
