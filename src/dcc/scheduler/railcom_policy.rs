use core::sync::atomic::{AtomicU32, Ordering};

/// Minimum number of transmitted packets between two RailCom cutouts on
/// non-Programming traffic. Caps telemetry cutouts at roughly 1 every 45 ms
/// (6 x ~7.5 ms/packet) so the detector has time to settle between windows
/// and refresh/idle packets still make measurable forward progress.
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) const RAILCOM_MIN_PACKET_GAP: u8 = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PacketClass {
    Safety,
    Programming,
    Telemetry,
    Command,
    Refresh,
    #[cfg(any(test, target_arch = "riscv32"))]
    Idle,
}

/// Snapshot of scheduler-side RailCom cutout decisions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomSchedulerStats {
    pub cutout_granted_count: u32,
    pub cutout_granted_pom_count: u32,
    pub cutout_skipped_budget_count: u32,
    pub cutout_skipped_priority_count: u32,
    pub track_logon_sent_count: u32,
    pub track_search_sent_count: u32,
    pub track_search_throttled_count: u32,
}

static CUTOUT_GRANTED_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_GRANTED_POM_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_SKIPPED_BUDGET_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_SKIPPED_PRIORITY_COUNT: AtomicU32 = AtomicU32::new(0);
static TRACK_LOGON_SENT_COUNT: AtomicU32 = AtomicU32::new(0);
static TRACK_SEARCH_SENT_COUNT: AtomicU32 = AtomicU32::new(0);
static TRACK_SEARCH_THROTTLED_COUNT: AtomicU32 = AtomicU32::new(0);

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
            CUTOUT_GRANTED_COUNT.fetch_add(1, Ordering::Relaxed);
            if matches!(class, PacketClass::Programming) {
                CUTOUT_GRANTED_POM_COUNT.fetch_add(1, Ordering::Relaxed);
            }
        } else {
            self.packets_since_cutout = self.packets_since_cutout.saturating_add(1);
            if budgeted {
                CUTOUT_SKIPPED_BUDGET_COUNT.fetch_add(1, Ordering::Relaxed);
            } else {
                CUTOUT_SKIPPED_PRIORITY_COUNT.fetch_add(1, Ordering::Relaxed);
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
    TRACK_LOGON_SENT_COUNT.fetch_add(1, Ordering::Relaxed);
}

#[cfg(target_arch = "riscv32")]
pub(super) fn record_track_search_sent() {
    TRACK_SEARCH_SENT_COUNT.fetch_add(1, Ordering::Relaxed);
}

#[cfg(target_arch = "riscv32")]
pub(super) fn record_track_search_throttled() {
    TRACK_SEARCH_THROTTLED_COUNT.fetch_add(1, Ordering::Relaxed);
}

/// Returns a copy of the scheduler-side RailCom decision counters.
#[must_use]
pub fn railcom_scheduler_stats() -> RailcomSchedulerStats {
    RailcomSchedulerStats {
        cutout_granted_count: CUTOUT_GRANTED_COUNT.load(Ordering::Acquire),
        cutout_granted_pom_count: CUTOUT_GRANTED_POM_COUNT.load(Ordering::Acquire),
        cutout_skipped_budget_count: CUTOUT_SKIPPED_BUDGET_COUNT.load(Ordering::Acquire),
        cutout_skipped_priority_count: CUTOUT_SKIPPED_PRIORITY_COUNT.load(Ordering::Acquire),
        track_logon_sent_count: TRACK_LOGON_SENT_COUNT.load(Ordering::Acquire),
        track_search_sent_count: TRACK_SEARCH_SENT_COUNT.load(Ordering::Acquire),
        track_search_throttled_count: TRACK_SEARCH_THROTTLED_COUNT.load(Ordering::Acquire),
    }
}

#[cfg(test)]
pub(super) fn reset_railcom_scheduler_stats() {
    CUTOUT_GRANTED_COUNT.store(0, Ordering::Release);
    CUTOUT_GRANTED_POM_COUNT.store(0, Ordering::Release);
    CUTOUT_SKIPPED_BUDGET_COUNT.store(0, Ordering::Release);
    CUTOUT_SKIPPED_PRIORITY_COUNT.store(0, Ordering::Release);
    TRACK_LOGON_SENT_COUNT.store(0, Ordering::Release);
    TRACK_SEARCH_SENT_COUNT.store(0, Ordering::Release);
    TRACK_SEARCH_THROTTLED_COUNT.store(0, Ordering::Release);
}
