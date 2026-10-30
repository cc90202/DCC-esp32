//! Lines the OLED footer shows while an update runs.
//!
//! The footer is drawn from column 0 in a 6-pixel font on a 128-pixel panel,
//! so 21 characters fit. Every text is checked against that width: fixed ones
//! at compile time, the ones carrying a version by falling back to a shorter
//! wording when the version would push them past the edge.

use core::{fmt::Write as _, time::Duration};

use dcc_ota_format::Version;

use crate::{admission::Lifecycle, journal::Record};

/// Columns the notices are written for: the footer row of the display.
pub const NOTICE_COLUMNS: usize = 21;

/// One notice line, never wider than [`NOTICE_COLUMNS`].
pub type NoticeText = heapless::String<NOTICE_COLUMNS>;

/// Transfer progress in tenths, 0 to 10.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tenths(u8);

impl Tenths {
    /// No progress yet.
    pub const ZERO: Self = Self(0);

    /// How many tenths of `total` bytes `received` covers.
    #[must_use]
    pub fn of(received: u32, total: u32) -> Self {
        let tenths = (u64::from(received) * 10 / u64::from(total.max(1))).min(10);
        Self(tenths as u8)
    }

    /// The progress as a percentage.
    #[must_use]
    pub const fn percent(self) -> u8 {
        self.0 * 10
    }
}

/// A step of the update the operator should see.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Notice {
    /// The upload has reached this progress.
    Receiving(Tenths),
    /// The package passed the pre-upload check and the upload may follow.
    /// It does not invite to power the track: a powered track refuses the
    /// upload. The notice stays until the track is powered again.
    Checked,
    /// The image is in flash and being verified.
    Verifying,
    /// The chip restarts into the new image.
    Restarting,
    /// The new image failed its trial; the chip restarts into the old one.
    RestartingBack,
    /// The new image runs and is being watched before confirmation.
    Trial {
        /// Version on trial.
        version: Version,
        /// What is left of the healthy minute; `None` until the system is
        /// healthy.
        left: Option<Duration>,
    },
    /// The new image is confirmed; the track waits for the operator.
    Confirmed(Version),
    /// The update stopped part way; the track waits for the operator.
    Failed,
    /// The package was refused before any flash write; the track waits.
    Rejected,
    /// After a restart the track stays off until the operator powers it.
    TrackOff,
}

const CHECKED: &str = "Valid: ready, upload";
const VERIFYING: &str = "OTA: verifying";
const RESTARTING: &str = "OTA: restarting...";
const RESTARTING_BACK: &str = "OTA FAIL: restarting";
const TRIAL_FALLBACK: &str = "OTA: testing new";
const CONFIRMED_FALLBACK: &str = "OTA OK: enable Z21";
const FAILED: &str = "Error: enable Z21";
const REJECTED: &str = "Rejected: enable Z21";
const TRACK_OFF: &str = "Track off: enable Z21";
const RECEIVING_FALLBACK: &str = "OTA: receiving";

const _: () = {
    let fixed = [
        CHECKED,
        VERIFYING,
        RESTARTING,
        RESTARTING_BACK,
        TRIAL_FALLBACK,
        CONFIRMED_FALLBACK,
        FAILED,
        REJECTED,
        TRACK_OFF,
        RECEIVING_FALLBACK,
    ];
    let mut i = 0;
    while i < fixed.len() {
        assert!(
            fixed[i].len() <= NOTICE_COLUMNS,
            "footer text wider than the display"
        );
        i += 1;
    }
};

impl Notice {
    /// Returns the footer line, shortened instead of cut when too wide.
    #[must_use]
    pub fn text(self) -> NoticeText {
        match self {
            Self::Receiving(progress) => formatted(
                format_args!("OTA: receiving {}%", progress.percent()),
                RECEIVING_FALLBACK,
            ),
            Self::Checked => fixed(CHECKED),
            Self::Verifying => fixed(VERIFYING),
            Self::Restarting => fixed(RESTARTING),
            Self::RestartingBack => fixed(RESTARTING_BACK),
            Self::Trial {
                version,
                left: None,
            } => formatted(format_args!("OTA: testing {version}"), TRIAL_FALLBACK),
            Self::Trial {
                version,
                left: Some(left),
            } => formatted(
                format_args!("OTA test {version}: {}s", left.as_secs()),
                TRIAL_FALLBACK,
            ),
            Self::Confirmed(version) => {
                formatted(format_args!("OK {version}: enable Z21"), CONFIRMED_FALLBACK)
            }
            Self::Failed => fixed(FAILED),
            Self::Rejected => fixed(REJECTED),
            Self::TrackOff => fixed(TRACK_OFF),
        }
    }

    /// The line to show once boot has reconciled the journal: the candidate
    /// on trial, or a reminder that the track waits for Z21 when this boot
    /// holds it off. The running version is always in the display title.
    #[must_use]
    pub fn after_boot(lifecycle: Lifecycle, record: &Record, track_held: bool) -> Option<Self> {
        match lifecycle {
            Lifecycle::Trial => record.candidate().map(|c| Self::Trial {
                version: c.version(),
                left: None,
            }),
            lifecycle if lifecycle.is_running() && track_held => Some(Self::TrackOff),
            Lifecycle::Starting | Lifecycle::Running | Lifecycle::Armed | Lifecycle::Recovery => {
                None
            }
        }
    }

    /// Whether the line asks the operator to power the track again.
    #[must_use]
    pub const fn awaits_track(self) -> bool {
        matches!(
            self,
            Self::Checked | Self::Confirmed(_) | Self::Failed | Self::Rejected | Self::TrackOff
        )
    }
}

/// Every fixed text is asserted to fit [`NOTICE_COLUMNS`] at compile time
/// above, so the empty fallback is never taken.
fn fixed(text: &str) -> NoticeText {
    NoticeText::try_from(text).unwrap_or_default()
}

fn formatted(args: core::fmt::Arguments<'_>, short: &str) -> NoticeText {
    let mut text = NoticeText::new();
    if text.write_fmt(args).is_ok() {
        text
    } else {
        fixed(short)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WIDEST_VERSION: Version = Version::new(u16::MAX, u16::MAX, u16::MAX);
    const USUAL_VERSION: Version = Version::new(0, 1, 1);

    fn every_notice(version: Version) -> [Notice; 11] {
        [
            Notice::Receiving(Tenths(10)),
            Notice::Checked,
            Notice::Verifying,
            Notice::Restarting,
            Notice::RestartingBack,
            Notice::Trial {
                version,
                left: None,
            },
            Notice::Trial {
                version,
                left: Some(Duration::from_secs(60)),
            },
            Notice::Confirmed(version),
            Notice::Failed,
            Notice::Rejected,
            Notice::TrackOff,
        ]
    }

    #[test]
    fn every_line_fits_and_is_never_empty() {
        for version in [USUAL_VERSION, WIDEST_VERSION] {
            for notice in every_notice(version) {
                let text = notice.text();
                assert!(!text.is_empty(), "{notice:?} rendered empty");
                assert!(text.len() <= NOTICE_COLUMNS, "{notice:?} too wide");
            }
        }
    }

    #[test]
    fn a_usual_version_is_shown_and_a_too_wide_one_falls_back() {
        assert_eq!(
            Notice::Confirmed(USUAL_VERSION).text(),
            "OK 0.1.1: enable Z21"
        );
        assert_eq!(Notice::Confirmed(WIDEST_VERSION).text(), CONFIRMED_FALLBACK);
        let trial = Notice::Trial {
            version: USUAL_VERSION,
            left: Some(Duration::from_millis(42_900)),
        };
        assert_eq!(trial.text(), "OTA test 0.1.1: 42s");
        assert_eq!(
            Notice::Receiving(Tenths::of(5, 4)).text(),
            "OTA: receiving 100%"
        );
        assert_eq!(Tenths::of(399, 1000), Tenths(3));
    }

    #[test]
    fn after_boot_names_the_trial_or_asks_for_the_track() {
        use crate::{journal::tests::image, layout::Slot};
        let installed = Record::installed(image(Slot::A, 0));
        let running = Lifecycle::Running;
        assert_eq!(Notice::after_boot(running, &installed, false), None);
        let update = installed.begin(image(Slot::B, 1)).unwrap();
        assert_eq!(
            Notice::after_boot(running, &update, true),
            Some(Notice::TrackOff)
        );
        assert_eq!(Notice::after_boot(running, &update, false), None);
        assert_eq!(
            Notice::after_boot(Lifecycle::Trial, &update, true),
            Some(Notice::Trial {
                version: image(Slot::B, 1).version(),
                left: None,
            })
        );
        assert_eq!(Notice::after_boot(Lifecycle::Recovery, &update, true), None);
    }
}
