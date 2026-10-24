//! Pure short-signal qualification: the edge-suppression window and the
//! integrating LOW-sample counter, kept free of GPIO and timer access.

/// Keep the full monotonic clock width: an expired 32-bit circular deadline
/// would look like a future deadline again after half a counter revolution.
#[derive(Clone, Copy)]
pub(super) struct ShortSuppression {
    pub(super) until_ms: u64,
}

impl ShortSuppression {
    pub(super) fn starting_at(now_ms: u64, duration_ms: u64) -> Self {
        Self {
            until_ms: now_ms.saturating_add(duration_ms),
        }
    }

    pub(super) fn is_active_at(self, now_ms: u64) -> bool {
        now_ms < self.until_ms
    }
}

/// Total pin samples taken after a falling edge before declaring the window
/// inconclusive. With 1 ms sampling this gives a 100 ms qualification window,
/// the same tolerance DCC-EX gives its fault pin when no overcurrent is
/// measured. A decoder with a large keep-alive capacitor set on a live track
/// holds FAULT low through the DRV8874's retry cycles for tens of
/// milliseconds while the capacitor charges; a 20 ms window latched that as a
/// short.
pub(super) const SHORT_CONFIRM_SAMPLES: u16 = 100;

/// Minimum LOW samples within the window required to latch a short (75%).
/// Motor inrush, capacitor charging and wheel micro-shorts on turnouts end
/// before accumulating this many LOW samples; a dead short keeps the line LOW
/// and confirms after 75 ms.
pub(super) const SHORT_CONFIRM_MIN_LOW_SAMPLES: u16 = 75;

/// Integrating qualifier for the short signal: counts LOW samples after a
/// falling edge and decides as early as possible.
///
/// `record` returns `Some(true)` once enough LOW samples accumulated to latch
/// a short, `Some(false)` once so many HIGH samples were seen that the LOW
/// threshold can no longer be reached, and `None` while still undecided.
pub(super) struct ShortQualifier {
    low: u16,
    high: u16,
}

impl ShortQualifier {
    pub(super) const fn new() -> Self {
        Self { low: 0, high: 0 }
    }

    pub(super) fn record(&mut self, is_low: bool) -> Option<bool> {
        if is_low {
            self.low += 1;
        } else {
            self.high += 1;
        }
        if self.low >= SHORT_CONFIRM_MIN_LOW_SAMPLES {
            Some(true)
        } else if self.high > SHORT_CONFIRM_SAMPLES - SHORT_CONFIRM_MIN_LOW_SAMPLES {
            Some(false)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        SHORT_CONFIRM_MIN_LOW_SAMPLES, SHORT_CONFIRM_SAMPLES, ShortQualifier, ShortSuppression,
    };

    #[test]
    fn suppression_window_is_active_before_deadline() {
        let window = ShortSuppression::starting_at(100, 250);
        assert!(window.is_active_at(100));
        assert!(!window.is_active_at(350));
        assert!(!window.is_active_at(351));
    }

    #[test]
    fn suppression_window_crosses_u32_boundary_without_truncation() {
        let start = u64::from(u32::MAX) - 10;
        let window = ShortSuppression::starting_at(start, 250);
        assert!(window.is_active_at(start + 100));
        assert!(!window.is_active_at(start + 250));
    }

    #[test]
    fn expired_suppression_never_reactivates_after_25_or_50_days() {
        let window = ShortSuppression::starting_at(100, 250);
        for elapsed in [1, (1u64 << 31) + 1, (1u64 << 32) + 1, 10 * (1u64 << 32)] {
            assert!(!window.is_active_at(350 + elapsed));
        }
        assert!(!(ShortSuppression { until_ms: 0 }).is_active_at((1u64 << 31) + 1));
    }

    #[test]
    fn solid_low_confirms_as_soon_as_threshold_reached() {
        let mut q = ShortQualifier::new();
        for _ in 0..SHORT_CONFIRM_MIN_LOW_SAMPLES - 1 {
            assert_eq!(q.record(true), None);
        }
        assert_eq!(q.record(true), Some(true));
    }

    #[test]
    fn brief_glitch_is_rejected_early() {
        let mut q = ShortQualifier::new();
        // A few LOW samples from the inrush pulse, then the line recovers HIGH.
        assert_eq!(q.record(true), None);
        assert_eq!(q.record(true), None);
        let max_highs = SHORT_CONFIRM_SAMPLES - SHORT_CONFIRM_MIN_LOW_SAMPLES;
        for _ in 0..max_highs {
            assert_eq!(q.record(false), None);
        }
        // One more HIGH makes reaching the LOW threshold impossible.
        assert_eq!(q.record(false), Some(false));
    }

    #[test]
    fn keep_alive_capacitor_charge_is_rejected() {
        // A Roco decoder set on a live track held FAULT low for tens of
        // milliseconds while its capacitor charged, then released it.
        let mut q = ShortQualifier::new();
        for _ in 0..50 {
            assert_eq!(q.record(true), None);
        }
        let mut verdict = None;
        while verdict.is_none() {
            verdict = q.record(false);
        }
        assert_eq!(verdict, Some(false));
    }

    #[test]
    fn intermittent_short_above_duty_threshold_confirms() {
        // A bouncing dead short (3 LOW, 1 HIGH) must still confirm once
        // enough LOW samples accumulate within the window.
        let mut q = ShortQualifier::new();
        let mut verdict = None;
        let mut n = 0u16;
        while verdict.is_none() && n < 2 * SHORT_CONFIRM_SAMPLES {
            let is_low = n % 4 != 3;
            verdict = q.record(is_low);
            n += 1;
        }
        assert_eq!(verdict, Some(true));
    }

    #[test]
    fn undecided_window_returns_none() {
        let mut q = ShortQualifier::new();
        assert_eq!(q.record(true), None);
        assert_eq!(q.record(false), None);
    }
}
