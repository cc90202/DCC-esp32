#[cfg(any(test, target_arch = "riscv32"))]
use core::sync::atomic::Ordering;

use crate::diagnostics::diagnostic_counters;

/// Minimum number of transmitted packets between two RailCom cutouts on
/// non-Programming traffic. Caps telemetry cutouts at roughly 1 every 45 ms
/// (6 x ~7.5 ms/packet) so the detector has time to settle between windows
/// and refresh/idle packets still make measurable forward progress.
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) const RAILCOM_MIN_PACKET_GAP: u8 = 6;

#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PacketClass {
    Safety,
    Programming,
    Telemetry,
    Command,
    Refresh,
    Idle,
}

diagnostic_counters! {
    #[cfg(any(test, target_arch = "riscv32"))]
    static RAILCOM_SCHEDULER: RailcomSchedulerCounters;

    /// Snapshot of scheduler-side RailCom cutout decisions.
    pub snapshot RailcomSchedulerStats;

    @test reset;

    cutout_granted_count,
    cutout_granted_pom_count,
    cutout_skipped_budget_count,
    cutout_skipped_priority_count,
    track_logon_sent_count,
    track_search_sent_count,
    track_search_throttled_count,
}

#[cfg(any(test, target_arch = "riscv32"))]
pub(super) struct RailcomCutoutBudget {
    packets_since_cutout: u8,
}

#[cfg(any(test, target_arch = "riscv32"))]
impl RailcomCutoutBudget {
    pub(super) const fn new() -> Self {
        Self {
            packets_since_cutout: u8::MAX,
        }
    }

    pub(super) fn allow_cutout_for(&mut self, class: PacketClass) -> bool {
        let immediate = matches!(class, PacketClass::Programming | PacketClass::Telemetry);
        let budgeted = matches!(class, PacketClass::Refresh | PacketClass::Idle);
        let allowed =
            immediate || (budgeted && self.packets_since_cutout >= RAILCOM_MIN_PACKET_GAP);

        if allowed {
            self.packets_since_cutout = 0;
            RAILCOM_SCHEDULER
                .cutout_granted_count
                .fetch_add(1, Ordering::Relaxed);
            if matches!(class, PacketClass::Programming) {
                RAILCOM_SCHEDULER
                    .cutout_granted_pom_count
                    .fetch_add(1, Ordering::Relaxed);
            }
        } else {
            self.packets_since_cutout = self.packets_since_cutout.saturating_add(1);
            if budgeted {
                RAILCOM_SCHEDULER
                    .cutout_skipped_budget_count
                    .fetch_add(1, Ordering::Relaxed);
            } else {
                RAILCOM_SCHEDULER
                    .cutout_skipped_priority_count
                    .fetch_add(1, Ordering::Relaxed);
            }
        }

        allowed
    }

    pub(super) fn note_packet_without_cutout(&mut self) {
        self.packets_since_cutout = self.packets_since_cutout.saturating_add(1);
    }
}

#[cfg(target_arch = "riscv32")]
pub(super) fn record_track_logon_sent() {
    RAILCOM_SCHEDULER
        .track_logon_sent_count
        .fetch_add(1, Ordering::Relaxed);
}

#[cfg(target_arch = "riscv32")]
pub(super) fn record_track_search_sent() {
    RAILCOM_SCHEDULER
        .track_search_sent_count
        .fetch_add(1, Ordering::Relaxed);
}

#[cfg(target_arch = "riscv32")]
pub(super) fn record_track_search_throttled() {
    RAILCOM_SCHEDULER
        .track_search_throttled_count
        .fetch_add(1, Ordering::Relaxed);
}

/// Returns a copy of the scheduler-side RailCom decision counters.
#[must_use]
#[cfg(any(test, target_arch = "riscv32"))]
pub fn railcom_scheduler_stats() -> RailcomSchedulerStats {
    RAILCOM_SCHEDULER.snapshot()
}

#[cfg(test)]
pub(super) fn reset_railcom_scheduler_stats() {
    RAILCOM_SCHEDULER.reset();
}
