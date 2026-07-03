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

/// Classified duration for the blue Resume button.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResumePress {
    Short,
    Long,
    Provisioning,
}

/// Domain action produced by a classified Resume button press.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResumeButtonAction {
    ResumeShortFault,
    ResumeLongFault,
    RequestWifiProvisioning,
}

/// Existing long-resume threshold for the blue Resume button.
pub const RESUME_LONG_PRESS_MS: u64 = 2_000;

/// Runtime WiFi provisioning threshold for the blue Resume button.
pub const RESUME_PROVISIONING_PRESS_MS: u64 = 10_000;

/// Classify a confirmed Resume button press by duration.
///
/// The 10-second provisioning threshold takes priority over the existing
/// 2-second long-resume action. Runtime code should classify once, then emit a
/// single event.
#[must_use]
pub const fn classify_resume_press(duration_ms: u64) -> ResumePress {
    if duration_ms >= RESUME_PROVISIONING_PRESS_MS {
        ResumePress::Provisioning
    } else if duration_ms >= RESUME_LONG_PRESS_MS {
        ResumePress::Long
    } else {
        ResumePress::Short
    }
}

/// Convert a classified Resume press into the single action it is allowed to emit.
#[must_use]
pub const fn resume_action_for_press(press: ResumePress) -> ResumeButtonAction {
    match press {
        ResumePress::Short => ResumeButtonAction::ResumeShortFault,
        ResumePress::Long => ResumeButtonAction::ResumeLongFault,
        ResumePress::Provisioning => ResumeButtonAction::RequestWifiProvisioning,
    }
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
    fn test_resume_press_short_below_two_seconds() {
        assert_eq!(classify_resume_press(1_999), ResumePress::Short);
    }

    #[test]
    fn test_resume_press_long_at_two_seconds() {
        assert_eq!(classify_resume_press(2_000), ResumePress::Long);
    }

    #[test]
    fn test_resume_press_long_before_ten_seconds() {
        assert_eq!(classify_resume_press(9_999), ResumePress::Long);
    }

    #[test]
    fn test_resume_press_provisioning_at_ten_seconds() {
        assert_eq!(classify_resume_press(10_000), ResumePress::Provisioning);
    }

    #[test]
    fn test_resume_press_provisioning_after_ten_seconds() {
        assert_eq!(classify_resume_press(10_001), ResumePress::Provisioning);
    }

    #[test]
    fn test_resume_provisioning_action_does_not_emit_long_resume_fault() {
        assert_eq!(
            resume_action_for_press(ResumePress::Provisioning),
            ResumeButtonAction::RequestWifiProvisioning
        );
    }

    #[test]
    fn test_resume_long_action_still_emits_long_fault() {
        assert_eq!(
            resume_action_for_press(ResumePress::Long),
            ResumeButtonAction::ResumeLongFault
        );
    }
}
