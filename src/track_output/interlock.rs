//! Update interlock that keeps the track unpowered while an update is pending.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use critical_section::with as critical_section_with;

static TRACK_INHIBITED: AtomicBool = AtomicBool::new(true);
static INHIBIT_EPOCH: AtomicU32 = AtomicU32::new(0);

/// Whether the update interlock refuses to power the track; set from power-on
/// until boot policy agrees.
pub(crate) fn inhibited() -> bool {
    TRACK_INHIBITED.load(Ordering::Acquire)
}

/// Counts how many times the interlock was engaged, so an operator action
/// that started before an engagement can be recognised as stale.
pub(crate) fn inhibit_epoch() -> u32 {
    INHIBIT_EPOCH.load(Ordering::Acquire)
}

/// Engages or lifts the interlock, serialized with the enable check in
/// [`TrackOutput::set_track_enabled`](super::TrackOutput::set_track_enabled).
pub(crate) fn set_inhibited(inhibit: bool) {
    critical_section_with(|_| {
        if inhibit {
            INHIBIT_EPOCH.fetch_add(1, Ordering::AcqRel);
        }
        TRACK_INHIBITED.store(inhibit, Ordering::Release);
    });
}
