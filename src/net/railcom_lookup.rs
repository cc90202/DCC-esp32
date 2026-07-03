//! RailCom sighting lookups for `LAN_RAILCOM_GETDATA` polling.
//!
//! ESP32-C6 only - gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

use crate::dcc::DccAddress;
use crate::railcom::loco_tracker::RailcomLocoSighting;

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
    stats
        .recent_sightings
        .iter()
        .flatten()
        .max_by_key(|sighting| sighting.packet_sequence)
        .copied()
}

pub(super) fn railcom_request_address(raw_address: u16) -> Option<DccAddress> {
    DccAddress::from_magnitude(raw_address)
}
