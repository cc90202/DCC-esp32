//! Pure helpers for button debounce and press classification.
//!
//! Keeping this logic free of HAL/Embassy types allows fast host-side tests for
//! safety-relevant operator input behavior.

/// Debounce verdict for a candidate active-low button press.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DebouncedPress {
    Confirmed,
    IgnoredBounce,
}

/// Debounce verdict for a candidate active-low button release.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DebouncedRelease {
    Confirmed,
    IgnoredBounce,
}

/// Classify a RESUME press duration against the configured long-press threshold.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResumePressKind {
    Short,
    Long,
}

/// Return whether a low level remained stable long enough to count as a press.
#[must_use]
pub const fn debounce_active_low_press(
    level_low_before_delay: bool,
    level_low_after_delay: bool,
) -> DebouncedPress {
    if level_low_before_delay && level_low_after_delay {
        DebouncedPress::Confirmed
    } else {
        DebouncedPress::IgnoredBounce
    }
}

/// Return whether a high level remained stable long enough to count as a release.
#[must_use]
pub const fn debounce_active_low_release(
    level_high_before_delay: bool,
    level_high_after_delay: bool,
) -> DebouncedRelease {
    if level_high_before_delay && level_high_after_delay {
        DebouncedRelease::Confirmed
    } else {
        DebouncedRelease::IgnoredBounce
    }
}

/// Classify a measured press duration.
#[must_use]
pub const fn classify_resume_press(held_ms: u64, long_press_threshold_ms: u64) -> ResumePressKind {
    if held_ms >= long_press_threshold_ms {
        ResumePressKind::Long
    } else {
        ResumePressKind::Short
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_debounce_press_accepts_stable_low() {
        assert_eq!(
            debounce_active_low_press(true, true),
            DebouncedPress::Confirmed
        );
    }

    #[test]
    fn test_debounce_press_rejects_bounce() {
        assert_eq!(
            debounce_active_low_press(true, false),
            DebouncedPress::IgnoredBounce
        );
    }

    #[test]
    fn test_debounce_release_accepts_stable_high() {
        assert_eq!(
            debounce_active_low_release(true, true),
            DebouncedRelease::Confirmed
        );
    }

    #[test]
    fn test_debounce_release_rejects_bounce() {
        assert_eq!(
            debounce_active_low_release(true, false),
            DebouncedRelease::IgnoredBounce
        );
    }

    #[test]
    fn test_classify_resume_press_short_below_threshold() {
        assert_eq!(classify_resume_press(1_999, 2_000), ResumePressKind::Short);
    }

    #[test]
    fn test_classify_resume_press_long_at_threshold() {
        assert_eq!(classify_resume_press(2_000, 2_000), ResumePressKind::Long);
    }
}
