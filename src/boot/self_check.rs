//! Purely diagnostic packet checks performed before track power can be armed.

use defmt::info;

use crate::dcc::{DccAddress, DccPacket, Direction};
use crate::dcc_runtime::build_idle_rmt_buffer;

use super::{BootError, DccSelfCheckError};

fn log_packet_bytes(label: &str, packet: DccPacket, error: BootError) -> Result<(), BootError> {
    let bytes = packet.to_bytes().map_err(|_| error)?;
    info!("{}: {:?}", label, bytes.as_slice());
    Ok(())
}

pub(super) fn verify_boot_packet_encoding() -> Result<(), BootError> {
    info!("boot: running DCC packet self-check");

    log_packet_bytes(
        "Idle packet",
        DccPacket::Idle,
        BootError::DccSelfCheck(DccSelfCheckError::IdlePacketEncoding),
    )?;
    build_idle_rmt_buffer()
        .map_err(|_| BootError::DccSelfCheck(DccSelfCheckError::IdleWaveformBuild))?;
    log_packet_bytes(
        "Reset packet",
        DccPacket::Reset,
        BootError::DccSelfCheck(DccSelfCheckError::ResetPacketEncoding),
    )?;

    let short_addr_3 = DccAddress::new_short(3).ok_or(BootError::DccSelfCheck(
        DccSelfCheckError::ShortAddress3Invalid,
    ))?;
    let speed28 = DccPacket::speed_28step(short_addr_3, 14, Direction::Forward).ok_or(
        BootError::DccSelfCheck(DccSelfCheckError::Speed28PacketEncoding),
    )?;
    log_packet_bytes(
        "Speed28 packet (addr=3, fwd, spd=14)",
        speed28,
        BootError::DccSelfCheck(DccSelfCheckError::Speed28PacketEncoding),
    )?;

    let long_addr_1000 = DccAddress::new_long(1000).ok_or(BootError::DccSelfCheck(
        DccSelfCheckError::LongAddress1000Invalid,
    ))?;
    let speed128 = DccPacket::speed_128step(long_addr_1000, 64, Direction::Reverse).ok_or(
        BootError::DccSelfCheck(DccSelfCheckError::Speed128PacketEncoding),
    )?;
    log_packet_bytes(
        "Speed128 packet (addr=1000, rev, spd=64)",
        speed128,
        BootError::DccSelfCheck(DccSelfCheckError::Speed128PacketEncoding),
    )?;

    info!("boot: DCC packet self-check complete");
    Ok(())
}
