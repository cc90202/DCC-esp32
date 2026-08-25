//! Shared provisioning network constants.

use smoltcp::wire::Ipv4Address;

pub(crate) const SERVER_IP: Ipv4Address = Ipv4Address::new(192, 168, 4, 1);
pub(crate) const SETUP_URL: &str = "http://192.168.4.1";
pub(crate) const CLIENT_IP: Ipv4Address = Ipv4Address::new(192, 168, 4, 2);
pub(crate) const SUBNET_MASK: Ipv4Address = Ipv4Address::new(255, 255, 255, 0);
#[cfg(target_arch = "riscv32")]
pub(crate) const PREFIX_LEN: u8 = 24;
