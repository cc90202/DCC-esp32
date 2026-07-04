//! Provisioning-only DHCPv4 server task.

use defmt::{info, warn};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, IpEndpoint, Stack};
use embassy_time::{Duration, Timer};

use super::dhcp_codec::{
    BROADCAST_IP, DHCP_CLIENT_PORT, DHCP_PACKET_BUFFER_SIZE, DHCP_SERVER_PORT,
    DhcpReplyDestination, build_reply,
};

#[embassy_executor::task]
pub(crate) async fn run_dhcp_server(stack: Stack<'static>) -> ! {
    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buffer = [0; DHCP_PACKET_BUFFER_SIZE];
    let mut tx_buffer = [0; DHCP_PACKET_BUFFER_SIZE];
    let mut request = [0; DHCP_PACKET_BUFFER_SIZE];
    let mut reply = [0; DHCP_PACKET_BUFFER_SIZE];
    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );

    while socket.bind(DHCP_SERVER_PORT).is_err() {
        warn!("DHCP server bind failed; retrying");
        Timer::after(Duration::from_secs(1)).await;
    }
    info!("DHCP server ready: lease 192.168.4.2");

    loop {
        let size = match socket.recv_from(&mut request).await {
            Ok((size, _remote)) => size,
            Err(_) => {
                warn!("DHCP receive failed");
                continue;
            }
        };

        let built_reply = match build_reply(&request[..size], &mut reply) {
            Ok(Some(built_reply)) => built_reply,
            Ok(None) => continue,
            Err(error) => {
                warn!("DHCP reply skipped: {:?}", error);
                continue;
            }
        };

        let client = match built_reply.destination {
            DhcpReplyDestination::Broadcast => BROADCAST_IP,
            DhcpReplyDestination::Unicast(address) => address,
        };
        let client = IpEndpoint::new(IpAddress::Ipv4(client), DHCP_CLIENT_PORT);
        if socket
            .send_to(&reply[..built_reply.len], client)
            .await
            .is_err()
        {
            warn!("DHCP send failed");
        }
    }
}
