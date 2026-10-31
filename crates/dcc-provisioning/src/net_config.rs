//! Shared provisioning network constants.

use smoltcp::wire::Ipv4Address;

/// Address of the station on its own access point.
pub const SERVER_IP: Ipv4Address = Ipv4Address::new(192, 168, 4, 1);
/// Address of the setup page, shown on the display; points at [`SERVER_IP`].
pub const SETUP_URL: &str = "http://192.168.4.1";
/// The only address leased: one phone configures the station at a time.
pub const CLIENT_IP: Ipv4Address = Ipv4Address::new(192, 168, 4, 2);
/// Netmask of the access point network.
pub const SUBNET_MASK: Ipv4Address = Ipv4Address::new(255, 255, 255, 0);
/// [`SUBNET_MASK`] as a prefix length, for the interface configuration.
pub const PREFIX_LEN: u8 = 24;
