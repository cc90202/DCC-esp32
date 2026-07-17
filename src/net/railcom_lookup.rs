//! RailCom sighting lookups for `LAN_RAILCOM_GETDATA` polling.
//!
//! ESP32-C6 only, gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

use crate::dcc::DccAddress;
use crate::railcom::loco_tracker::RailcomLocoSighting;
use crate::railcom::pipeline::PacketSequence;

pub(super) fn railcom_sighting_for_address(address: DccAddress) -> Option<RailcomLocoSighting> {
    let stats = crate::railcom::loco_tracker::railcom_loco_tracker_stats();
    stats
        .recent_sightings
        .iter()
        .flatten()
        .find(|sighting| sighting.address == address)
        .copied()
}

pub(super) fn latest_railcom_sighting() -> Option<RailcomLocoSighting> {
    let stats = crate::railcom::loco_tracker::railcom_loco_tracker_stats();
    let current = PacketSequence::new(crate::railcom::stats().packet_boundary_count);
    stats
        .recent_sightings
        .iter()
        .flatten()
        .min_by_key(|sighting| current.age_since(sighting.packet_sequence))
        .copied()
}
