//! Purely diagnostic packet checks performed before track power can be armed.

use defmt::info;

use crate::dcc::{DccAddress, DccPacket, Direction};

use super::{BootError, DccSelfCheckError};

fn log_packet_bytes(label: &str, packet: DccPacket) {
    let bytes = packet.to_bytes();
    info!("{}: {:?}", label, bytes.as_slice());
}

/// Encodes and logs a few reference packets, failing if any cannot be built.
///
/// The idle waveform buffer is not covered here: it is built and checked in
/// [`super::hardware::initialize_dcc_rmt`], which fails boot if that build fails.
pub(super) fn verify_boot_packet_encoding() -> Result<(), BootError> {
    info!("boot: running DCC packet self-check");

    log_packet_bytes("Idle packet", DccPacket::Idle);
    log_packet_bytes("Reset packet", DccPacket::Reset);

    let short_addr_3 = DccAddress::new_short(3).ok_or(DccSelfCheckError::ShortAddress3Invalid)?;
    let speed28 = DccPacket::speed_28step(short_addr_3, 14, Direction::Forward)
        .ok_or(DccSelfCheckError::Speed28PacketEncoding)?;
    log_packet_bytes("Speed28 packet (addr=3, fwd, spd=14)", speed28);

    let long_addr_1000 =
        DccAddress::new_long(1000).ok_or(DccSelfCheckError::LongAddress1000Invalid)?;
    let speed128 = DccPacket::speed_128step(long_addr_1000, 64, Direction::Reverse)
        .ok_or(DccSelfCheckError::Speed128PacketEncoding)?;
    log_packet_bytes("Speed128 packet (addr=1000, rev, spd=64)", speed128);

    info!("boot: DCC packet self-check complete");
    Ok(())
}
