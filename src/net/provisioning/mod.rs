//! Runtime WiFi provisioning support.
//!
//! Host-testable pieces (`http_parser`, `dhcp_codec`, `net_config`) are
//! compiled for tests too; the SoftAP, HTTP and DHCP server tasks are
//! target-only.

#[cfg(target_arch = "riscv32")]
mod ap;
#[cfg(any(test, target_arch = "riscv32"))]
mod dhcp_codec;
#[cfg(target_arch = "riscv32")]
mod dhcp_server;
#[cfg(target_arch = "riscv32")]
mod html;
#[cfg(any(test, target_arch = "riscv32"))]
mod http_parser;
#[cfg(target_arch = "riscv32")]
mod http_server;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod net_config;

#[cfg(target_arch = "riscv32")]
pub(crate) use ap::{ProvisioningApError, run_provisioning_ap};
