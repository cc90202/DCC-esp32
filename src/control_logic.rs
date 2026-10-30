//! Pure helpers for button press classification.
//!
//! Keeping this logic free of HAL/Embassy types allows fast host-side tests for
//! safety-relevant operator input behavior.

/// Classified duration for the blue Resume button.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ResumePress {
    Short,
    Long,
    Provisioning,
}

/// Existing long-resume threshold for the blue Resume button.
pub(crate) const RESUME_LONG_PRESS_MS: u64 = 2_000;

/// Runtime WiFi provisioning threshold for the blue Resume button.
pub(crate) const RESUME_PROVISIONING_PRESS_MS: u64 = 10_000;
const RESUME_SHORT_MAX_MS: u64 = RESUME_LONG_PRESS_MS - 1;
const RESUME_LONG_MAX_MS: u64 = RESUME_PROVISIONING_PRESS_MS - 1;

/// Classify a confirmed Resume button press by duration.
///
/// The 10-second provisioning threshold takes priority over the existing
/// 2-second long-resume action. Runtime code should classify once, then emit a
/// single event.
#[must_use]
pub(crate) const fn classify_resume_press(duration_ms: u64) -> ResumePress {
    match duration_ms {
        0..=RESUME_SHORT_MAX_MS => ResumePress::Short,
        RESUME_LONG_PRESS_MS..=RESUME_LONG_MAX_MS => ResumePress::Long,
        RESUME_PROVISIONING_PRESS_MS.. => ResumePress::Provisioning,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
