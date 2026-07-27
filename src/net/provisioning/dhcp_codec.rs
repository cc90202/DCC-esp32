//! Host-testable DHCPv4 reply builder for provisioning SoftAP mode.

#[cfg(target_arch = "riscv32")]
use smoltcp::wire::{
    DHCP_CLIENT_PORT as SMOLTCP_DHCP_CLIENT_PORT, DHCP_SERVER_PORT as SMOLTCP_DHCP_SERVER_PORT,
};
use smoltcp::wire::{DhcpMessageType, DhcpPacket, DhcpRepr, Ipv4Address};

use super::net_config::{CLIENT_IP, SERVER_IP, SUBNET_MASK};
#[cfg(target_arch = "riscv32")]
pub(crate) const BROADCAST_IP: Ipv4Address = Ipv4Address::BROADCAST;
pub(crate) const UNSPECIFIED_IP: Ipv4Address = Ipv4Address::UNSPECIFIED;
#[cfg(target_arch = "riscv32")]
pub(crate) const DHCP_SERVER_PORT: u16 = SMOLTCP_DHCP_SERVER_PORT;
#[cfg(target_arch = "riscv32")]
pub(crate) const DHCP_CLIENT_PORT: u16 = SMOLTCP_DHCP_CLIENT_PORT;
pub(crate) const DHCP_PACKET_BUFFER_SIZE: usize = 576;

const LEASE_SECONDS: u32 = 3600;
const RENEW_SECONDS: u32 = LEASE_SECONDS / 2;
const REBIND_SECONDS: u32 = LEASE_SECONDS * 7 / 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) enum DhcpReplyError {
    InvalidPacket,
    BufferTooSmall,
    Emit,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DhcpReplyDestination {
    Broadcast,
    Unicast(Ipv4Address),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct DhcpReply {
    pub(crate) len: usize,
    pub(crate) destination: DhcpReplyDestination,
}

pub(crate) fn build_reply(
    request: &[u8],
    reply: &mut [u8],
) -> Result<Option<DhcpReply>, DhcpReplyError> {
    let packet = DhcpPacket::new_checked(request).map_err(|_| DhcpReplyError::InvalidPacket)?;
    let request = DhcpRepr::parse(&packet).map_err(|_| DhcpReplyError::InvalidPacket)?;

    let reply_type = match request.message_type {
        DhcpMessageType::Discover => DhcpMessageType::Offer,
        DhcpMessageType::Request if request_targets_this_server(&request) => DhcpMessageType::Ack,
        _ => return Ok(None),
    };
    let destination = reply_destination(&request);

    let response = DhcpRepr {
        message_type: reply_type,
        transaction_id: request.transaction_id,
        secs: request.secs,
        client_hardware_address: request.client_hardware_address,
        client_ip: UNSPECIFIED_IP,
        your_ip: CLIENT_IP,
        server_ip: SERVER_IP,
        router: Some(SERVER_IP),
        subnet_mask: Some(SUBNET_MASK),
        relay_agent_ip: UNSPECIFIED_IP,
        broadcast: matches!(destination, DhcpReplyDestination::Broadcast),
        requested_ip: None,
        client_identifier: request.client_identifier,
        server_identifier: Some(SERVER_IP),
        parameter_request_list: None,
        dns_servers: None,
        max_size: None,
        lease_duration: Some(LEASE_SECONDS),
        renew_duration: Some(RENEW_SECONDS),
        rebind_duration: Some(REBIND_SECONDS),
        additional_options: &[],
    };

    let len = response.buffer_len();
    if reply.len() < len {
        return Err(DhcpReplyError::BufferTooSmall);
    }

    let mut packet = DhcpPacket::new_unchecked(&mut reply[..len]);
    response
        .emit(&mut packet)
        .map_err(|_| DhcpReplyError::Emit)?;
    Ok(Some(DhcpReply { len, destination }))
}

fn request_targets_this_server(request: &DhcpRepr<'_>) -> bool {
    match (request.server_identifier, request.requested_ip) {
        (Some(server), Some(requested_ip)) => server == SERVER_IP && requested_ip == CLIENT_IP,
        (None, Some(requested_ip)) => {
            request.client_ip == UNSPECIFIED_IP && requested_ip == CLIENT_IP
        }
        (None, None) => request.client_ip == CLIENT_IP,
        (Some(_), None) => false,
    }
}

fn reply_destination(request: &DhcpRepr<'_>) -> DhcpReplyDestination {
    if request.client_ip == CLIENT_IP && !request.broadcast {
        DhcpReplyDestination::Unicast(CLIENT_IP)
    } else {
        DhcpReplyDestination::Broadcast
    }
}

#[cfg(test)]
mod tests {
    use smoltcp::wire::{DhcpMessageType, EthernetAddress};

    use super::*;

    const CLIENT_MAC: EthernetAddress = EthernetAddress([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]);
    const XID: u32 = 0x1234_5678;

    #[test]
    fn discover_returns_offer_for_single_lease() {
        let request = request_packet(DhcpMessageType::Discover, UNSPECIFIED_IP, None, None);
        let reply = reply_to(&request).expect("discover should produce offer");

        with_parsed_packet(&reply, |reply| {
            assert_eq!(reply.message_type, DhcpMessageType::Offer);
            assert!(reply.broadcast);
            assert_eq!(reply.transaction_id, XID);
            assert_eq!(reply.client_hardware_address, CLIENT_MAC);
            assert_eq!(reply.your_ip, CLIENT_IP);
            assert_eq!(reply.server_ip, SERVER_IP);
            assert_eq!(reply.router, Some(SERVER_IP));
            assert_eq!(reply.subnet_mask, Some(SUBNET_MASK));
            assert_eq!(reply.server_identifier, Some(SERVER_IP));
            assert_eq!(reply.lease_duration, Some(LEASE_SECONDS));
        })
        .expect("reply should parse");
    }

    #[test]
    fn request_for_server_returns_ack() {
        let request = request_packet(
            DhcpMessageType::Request,
            UNSPECIFIED_IP,
            Some(CLIENT_IP),
            Some(SERVER_IP),
        );
        let reply = reply_to(&request).expect("request should produce ack");

        with_parsed_packet(&reply, |reply| {
            assert_eq!(reply.message_type, DhcpMessageType::Ack);
            assert!(reply.broadcast);
            assert_eq!(reply.your_ip, CLIENT_IP);
        })
        .expect("reply should parse");
    }

    #[test]
    fn init_reboot_request_for_client_address_returns_ack() {
        let request = request_packet(
            DhcpMessageType::Request,
            UNSPECIFIED_IP,
            Some(CLIENT_IP),
            None,
        );
        let reply = reply_to(&request).expect("init-reboot request should produce ack");

        with_parsed_packet(&reply, |reply| {
            assert_eq!(reply.message_type, DhcpMessageType::Ack);
            assert!(reply.broadcast);
            assert_eq!(reply.your_ip, CLIENT_IP);
        })
        .expect("reply should parse");
    }

    #[test]
    fn renewal_request_from_client_address_returns_ack() {
        let request = request_packet_with_broadcast_flag(
            DhcpMessageType::Request,
            CLIENT_IP,
            None,
            None,
            false,
        );
        let reply = reply_to(&request).expect("renewal request should produce ack");

        assert_eq!(reply.destination, DhcpReplyDestination::Unicast(CLIENT_IP));

        with_parsed_packet(&reply, |reply| {
            assert_eq!(reply.message_type, DhcpMessageType::Ack);
            assert!(!reply.broadcast);
            assert_eq!(reply.your_ip, CLIENT_IP);
        })
        .expect("reply should parse");
    }

    #[test]
    fn renewal_request_with_broadcast_flag_returns_broadcast_ack() {
        let request = request_packet_with_broadcast_flag(
            DhcpMessageType::Request,
            CLIENT_IP,
            None,
            None,
            true,
        );
        let reply = reply_to(&request).expect("renewal request should produce ack");

        assert_eq!(reply.destination, DhcpReplyDestination::Broadcast);

        with_parsed_packet(&reply, |reply| {
            assert_eq!(reply.message_type, DhcpMessageType::Ack);
            assert!(reply.broadcast);
        })
        .expect("reply should parse");
    }

    #[test]
    fn request_without_identifiers_or_client_address_is_ignored() {
        let request = request_packet(DhcpMessageType::Request, UNSPECIFIED_IP, None, None);

        assert!(reply_to(&request).is_none());
    }

    #[test]
    fn renewal_request_from_other_client_address_is_ignored() {
        let request = request_packet(
            DhcpMessageType::Request,
            Ipv4Address::new(192, 168, 4, 99),
            None,
            None,
        );

        assert!(reply_to(&request).is_none());
    }

    #[test]
    fn request_for_other_server_is_ignored() {
        let request = request_packet(
            DhcpMessageType::Request,
            UNSPECIFIED_IP,
            Some(CLIENT_IP),
            Some(Ipv4Address::new(192, 168, 4, 254)),
        );

        assert!(reply_to(&request).is_none());
    }

    #[test]
    fn request_for_other_address_is_ignored() {
        let request = request_packet(
            DhcpMessageType::Request,
            UNSPECIFIED_IP,
            Some(Ipv4Address::new(192, 168, 4, 99)),
            Some(SERVER_IP),
        );

        assert!(reply_to(&request).is_none());
    }

    #[test]
    fn invalid_packet_returns_error() {
        let mut reply = [0; DHCP_PACKET_BUFFER_SIZE];

        assert_eq!(
            build_reply(&[0; 8], &mut reply),
            Err(DhcpReplyError::InvalidPacket)
        );
    }

    #[test]
    fn undersized_reply_buffer_returns_error() {
        let request = request_packet(DhcpMessageType::Discover, UNSPECIFIED_IP, None, None);
        let mut reply = [0; 16];

        assert_eq!(
            build_reply(&request, &mut reply),
            Err(DhcpReplyError::BufferTooSmall)
        );
    }

    struct TestReply {
        bytes: heapless::Vec<u8, DHCP_PACKET_BUFFER_SIZE>,
        destination: DhcpReplyDestination,
    }

    impl core::ops::Deref for TestReply {
        type Target = [u8];

        fn deref(&self) -> &Self::Target {
            &self.bytes
        }
    }

    fn reply_to(request: &[u8]) -> Option<TestReply> {
        let mut buffer = [0; DHCP_PACKET_BUFFER_SIZE];
        let built = build_reply(request, &mut buffer).expect("reply builder should not fail")?;
        let mut reply = heapless::Vec::new();
        reply
            .extend_from_slice(&buffer[..built.len])
            .expect("reply should fit fixed test buffer");
        Some(TestReply {
            bytes: reply,
            destination: built.destination,
        })
    }

    fn request_packet(
        message_type: DhcpMessageType,
        client_ip: Ipv4Address,
        requested_ip: Option<Ipv4Address>,
        server_identifier: Option<Ipv4Address>,
    ) -> heapless::Vec<u8, DHCP_PACKET_BUFFER_SIZE> {
        request_packet_with_broadcast_flag(
            message_type,
            client_ip,
            requested_ip,
            server_identifier,
            true,
        )
    }

    fn request_packet_with_broadcast_flag(
        message_type: DhcpMessageType,
        client_ip: Ipv4Address,
        requested_ip: Option<Ipv4Address>,
        server_identifier: Option<Ipv4Address>,
        broadcast: bool,
    ) -> heapless::Vec<u8, DHCP_PACKET_BUFFER_SIZE> {
        let repr = DhcpRepr {
            message_type,
            transaction_id: XID,
            secs: 0,
            client_hardware_address: CLIENT_MAC,
            client_ip,
            your_ip: UNSPECIFIED_IP,
            server_ip: UNSPECIFIED_IP,
            router: None,
            subnet_mask: None,
            relay_agent_ip: UNSPECIFIED_IP,
            broadcast,
            requested_ip,
            client_identifier: None,
            server_identifier,
            parameter_request_list: None,
            dns_servers: None,
            max_size: None,
            lease_duration: None,
            renew_duration: None,
            rebind_duration: None,
            additional_options: &[],
        };

        let mut buffer = heapless::Vec::<u8, DHCP_PACKET_BUFFER_SIZE>::new();
        buffer
            .resize(repr.buffer_len(), 0)
            .expect("test DHCP packet should fit fixed buffer");
        let mut packet = DhcpPacket::new_unchecked(buffer.as_mut_slice());
        repr.emit(&mut packet)
            .expect("test DHCP packet should encode");
        buffer
    }

    fn with_parsed_packet<T>(packet: &[u8], f: impl FnOnce(DhcpRepr<'_>) -> T) -> Option<T> {
        let packet = DhcpPacket::new_checked(packet).ok()?;
        let repr = DhcpRepr::parse(&packet).ok()?;
        Some(f(repr))
    }
}
