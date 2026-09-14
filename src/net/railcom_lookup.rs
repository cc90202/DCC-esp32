//! RailCom sighting lookups for `LAN_RAILCOM_GETDATA` polling.
//!
//! ESP32-C6 only, gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

use crate::dcc::DccAddress;
use crate::railcom::PacketSequence;
use crate::railcom::loco_tracker::RailcomLocoSighting;

/// The packet boundary the track has reached.
///
/// Must be the very counter that stamps a receive window, not another one that
/// merely advances at the same rate: ages are compared in absolute terms now,
/// and two counters that agree only by habit would drift apart the first time
/// one of them is restarted, leaving every sighting born already stale.
///
/// Read here rather than behind a helper in `railcom`, which would be the
/// natural home: the driver already notifies `railcom` on every boundary, so a
/// call the other way would close a dependency cycle. That cycle is a
/// consequence of the notification, not of the structure — it goes away with
/// the second boundary counter, which means touching the ISR, and this branch
/// leaves the driver untouched on purpose.
fn current_packet_boundary() -> PacketSequence {
    PacketSequence::new(crate::rmt_dcc::isr_heartbeat())
}

pub(super) fn railcom_sighting_for_address(address: DccAddress) -> Option<RailcomLocoSighting> {
    crate::railcom::loco_tracker::railcom_loco_tracker_stats()
        .current_sighting_for(address, current_packet_boundary())
}

pub(super) fn latest_railcom_sighting() -> Option<RailcomLocoSighting> {
    crate::railcom::loco_tracker::railcom_loco_tracker_stats()
        .latest_current_sighting(current_packet_boundary())
}
