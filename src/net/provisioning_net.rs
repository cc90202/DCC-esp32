//! Shared provisioning network constants.

pub(crate) const SERVER_IP_OCTETS: [u8; 4] = [192, 168, 4, 1];
pub(crate) const SETUP_URL: &str = "http://192.168.4.1";
pub(crate) const CLIENT_IP_OCTETS: [u8; 4] = [192, 168, 4, 2];
pub(crate) const SUBNET_MASK_OCTETS: [u8; 4] = [255, 255, 255, 0];
#[cfg(target_arch = "riscv32")]
pub(crate) const PREFIX_LEN: u8 = 24;
