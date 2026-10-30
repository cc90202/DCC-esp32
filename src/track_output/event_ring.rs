//! Task-side readers of the cutout event ring and packet metadata slots.
//!
//! The interrupt in the parent module is the only writer of the slots; these
//! functions read them and count the events that were overwritten before
//! they could be drained.

use core::sync::atomic::Ordering;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use heapless::Vec;

#[cfg(any(test, feature = "bench-diag"))]
use super::TrackOutputStats;
use super::{
    CUTOUT, CUTOUT_EVENT_NOTIFY, CUTOUT_EVENT_RING_CAPACITY, CUTOUT_EVENT_SLOTS,
    CUTOUT_EVENT_WRITE_COUNT, RAILCOM_PACKET_META_CAPACITY, RAILCOM_PACKET_META_SLOTS,
};
use crate::cutout::PacketSequence;
use crate::cutout_metadata::{
    CutoutRuntimeEvent, PackedRailcomPacketMetadata, RailcomPacketMetadata,
};

/// Returns a copy of the current cutout statistics.
#[must_use]
#[cfg(any(test, feature = "bench-diag"))]
pub(crate) fn stats() -> TrackOutputStats {
    CUTOUT.snapshot()
}

/// Returns the metadata recorded for the cutout after `packet_sequence`.
///
/// Returns `None` when the slot was never written, is being written, was
/// read torn, or has already been reused by a later packet.
#[must_use]
pub(crate) fn railcom_packet_metadata(
    packet_sequence: PacketSequence,
) -> Option<RailcomPacketMetadata> {
    let packet_sequence = packet_sequence.value();
    let slot = packet_sequence as usize % RAILCOM_PACKET_META_CAPACITY;
    let [recorded_sequence, metadata_raw, pom_request_id] =
        RAILCOM_PACKET_META_SLOTS[slot].snapshot()?;
    let metadata = PackedRailcomPacketMetadata::from_raw(metadata_raw);
    (recorded_sequence == packet_sequence).then(|| metadata.public_metadata(pom_request_id))
}

/// Drain cutout runtime events observed since `next_read`.
///
/// If the producer outruns the consumer beyond ring capacity, the oldest
/// events are dropped and counted in diagnostics.
///
/// The counters use wrapping arithmetic because overflowing `u32` is expected
/// during long-running operation. Per-slot generations reject torn snapshots.
pub(crate) fn drain_cutout_runtime_events(
    next_read: &mut u32,
    out: &mut Vec<CutoutRuntimeEvent, CUTOUT_EVENT_RING_CAPACITY>,
) {
    let write = CUTOUT_EVENT_WRITE_COUNT.load(Ordering::Acquire);
    let available = write.wrapping_sub(*next_read);
    if available > CUTOUT_EVENT_RING_CAPACITY as u32 {
        let dropped = available - CUTOUT_EVENT_RING_CAPACITY as u32;
        *next_read = write.wrapping_sub(CUTOUT_EVENT_RING_CAPACITY as u32);
        CUTOUT
            .cutout_event_dropped_count
            .fetch_add(dropped, Ordering::Relaxed);
    }

    let mut remaining = write
        .wrapping_sub(*next_read)
        .min(CUTOUT_EVENT_RING_CAPACITY as u32);
    while remaining > 0 && out.len() < CUTOUT_EVENT_RING_CAPACITY {
        let slot = *next_read as usize % CUTOUT_EVENT_RING_CAPACITY;
        let Some([sequence, metadata_raw, pom_request_id]) = CUTOUT_EVENT_SLOTS[slot].snapshot()
        else {
            CUTOUT
                .cutout_event_dropped_count
                .fetch_add(1, Ordering::Relaxed);
            *next_read = next_read.wrapping_add(1);
            remaining -= 1;
            continue;
        };
        let metadata = PackedRailcomPacketMetadata::from_raw(metadata_raw);
        let _ = out.push(metadata.runtime_event(PacketSequence::new(sequence), pom_request_id));
        *next_read = next_read.wrapping_add(1);
        remaining -= 1;
    }
}

/// Returns a receiver that yields one wake-up token whenever the interrupt
/// pushes a cutout event; the events themselves are read with
/// [`drain_cutout_runtime_events`].
#[must_use]
pub(crate) fn cutout_event_notify_receiver() -> Receiver<'static, CriticalSectionRawMutex, u8, 1> {
    CUTOUT_EVENT_NOTIFY.receiver()
}
