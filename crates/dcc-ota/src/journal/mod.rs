//! The update journal: which image runs, which one is being installed or
//! tried, and how the last update ended.
//!
//! A [`Record`] changes only through the transitions below, each of which
//! checks the step it starts from, and [`Update`] carries the candidate in
//! every step that has one, so an inconsistent record cannot be built
//! outside this module. The `store` submodule persists it so that a power cut at any
//! byte leaves either the old or the new record.

mod boot;
mod codec;
pub(crate) mod store;

pub use boot::{Blame, BootAction, CurrentImage, ResetCause};

use dcc_ota_format::Version;

use crate::{StateError, image::ImageId, layout::Slot};

/// Where the update stands, with the candidate it concerns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Update {
    /// No update since the record was created.
    None,
    /// The candidate is being written to the inactive slot.
    Receiving(ImageId),
    /// The candidate is written, verified and selected for the next boot.
    Ready(ImageId),
    /// The candidate booted and is on trial.
    Trying(ImageId),
    /// The last update ended; the current image runs.
    Ended(ImageId, Ending),
}

impl Update {
    /// The image of the running or last update.
    #[must_use]
    pub const fn candidate(self) -> Option<ImageId> {
        match self {
            Self::None => None,
            Self::Receiving(image)
            | Self::Ready(image)
            | Self::Trying(image)
            | Self::Ended(image, _) => Some(image),
        }
    }

    /// How the last update ended; `None` while one runs or before the first.
    #[must_use]
    pub const fn ending(self) -> Option<Ending> {
        match self {
            Self::Ended(_, ending) => Some(ending),
            _ => None,
        }
    }

    /// Whether no update is in flight.
    #[must_use]
    pub const fn is_settled(self) -> bool {
        matches!(self, Self::None | Self::Ended(..))
    }
}

/// How an update ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ending {
    /// The candidate was confirmed and is now the current image; `previous`
    /// is the image it replaced, which the bootloader falls back to if the
    /// new one no longer passes its image check.
    Confirmed {
        /// The image confirmed before.
        previous: ImageId,
    },
    /// The transfer stopped before the candidate proved anything; the
    /// release may be sent again.
    Aborted,
    /// The candidate could not be made bootable, or stopped booting.
    NotBootable,
    /// The candidate failed its trial boot and is rejected.
    RolledBack,
}

/// One journal entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Record {
    generation: u64,
    current: ImageId,
    update: Update,
    rejected: Option<Version>,
    hold_track_off: bool,
}

impl Record {
    /// The first record of a device: `booted` runs, confirmed.
    #[must_use]
    pub(crate) const fn installed(booted: ImageId) -> Self {
        Self {
            generation: 0,
            current: booted,
            update: Update::None,
            rejected: None,
            hold_track_off: false,
        }
    }

    /// Save counter; the higher of the two stored records wins.
    #[cfg(test)]
    pub(crate) const fn generation(&self) -> u64 {
        self.generation
    }

    /// The confirmed image, or the one restored after a failure.
    #[must_use]
    pub const fn current(&self) -> ImageId {
        self.current
    }

    /// Where the update stands.
    #[must_use]
    pub const fn update(&self) -> Update {
        self.update
    }

    /// The image of the running or last update, if any.
    #[must_use]
    pub const fn candidate(&self) -> Option<ImageId> {
        self.update.candidate()
    }

    /// Highest release that failed a trial boot; nothing at or below it is
    /// accepted again. A failed transfer never raises it.
    #[must_use]
    pub const fn rejected(&self) -> Option<Version> {
        self.rejected
    }

    /// Whether the next boot keeps the track off until the operator powers
    /// it again. Every update step sets it; the first ordinary boot after
    /// the update honours it and clears it, so the boots that follow start
    /// as before the update.
    #[must_use]
    pub const fn holds_track_off(&self) -> bool {
        self.hold_track_off
    }

    /// Lets the boots that follow power the track at start-up again.
    const fn release_track(mut self) -> Self {
        self.hold_track_off = false;
        self
    }

    /// Starts a transfer of `target` into the inactive slot.
    ///
    /// # Errors
    /// Returns [`StateError::WrongPhase`] while another update is unfinished
    /// or when `target` is the running slot.
    pub(crate) fn begin(mut self, target: ImageId) -> Result<Self, StateError> {
        if !self.update.is_settled() || target.slot() == self.current.slot() {
            return Err(StateError::WrongPhase);
        }
        self.update = Update::Receiving(target);
        self.hold_track_off = true;
        Ok(self)
    }

    /// Marks the fully written `target` as selected for the next boot.
    ///
    /// # Errors
    /// Returns [`StateError::Corrupt`] unless `target` is the candidate
    /// being received.
    pub(crate) fn ready(mut self, target: ImageId) -> Result<Self, StateError> {
        if self.update != Update::Receiving(target) {
            return Err(StateError::Corrupt);
        }
        self.update = Update::Ready(target);
        Ok(self)
    }

    /// Ends a transfer that was never booted with `ending`; the release
    /// stays eligible.
    ///
    /// # Errors
    /// Returns [`StateError::WrongPhase`] outside a transfer.
    pub(crate) fn end_transfer(mut self, ending: Ending) -> Result<Self, StateError> {
        let (Update::Receiving(candidate) | Update::Ready(candidate)) = self.update else {
            return Err(StateError::WrongPhase);
        };
        self.update = Update::Ended(candidate, ending);
        self.hold_track_off = true;
        Ok(self)
    }

    /// The candidate on trial, if it runs from `booted`.
    fn trial_in(&self, booted: Slot) -> Option<ImageId> {
        match self.update {
            Update::Trying(candidate) if candidate.slot() == booted => Some(candidate),
            _ => None,
        }
    }

    /// Confirms the candidate running from `booted`.
    ///
    /// # Errors
    /// Returns [`StateError::Corrupt`] unless a trial of the image in
    /// `booted` is running.
    pub(crate) fn confirm(mut self, booted: Slot) -> Result<Self, StateError> {
        let candidate = self.trial_in(booted).ok_or(StateError::Corrupt)?;
        self.update = Update::Ended(
            candidate,
            Ending::Confirmed {
                previous: self.current,
            },
        );
        self.current = candidate;
        Ok(self)
    }

    /// Rejects the candidate on trial and returns to the current image.
    ///
    /// # Errors
    /// Returns [`StateError::WrongPhase`] in any other step: rejecting a
    /// release that never booted would block it for good.
    pub(crate) fn roll_back(mut self) -> Result<Self, StateError> {
        let Update::Trying(candidate) = self.update else {
            return Err(StateError::WrongPhase);
        };
        self.reject(candidate);
        self.update = Update::Ended(candidate, Ending::RolledBack);
        Ok(self)
    }

    /// Keeps the track off after the restart that follows.
    #[must_use]
    pub(crate) const fn keep_track_off(mut self) -> Self {
        self.hold_track_off = true;
        self
    }

    /// Raises the failed-boot floor to `image`, never lowering it.
    fn reject(&mut self, image: ImageId) {
        let version = image.version();
        self.rejected = Some(self.rejected.map_or(version, |floor| floor.max(version)));
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use dcc_ota_format::{Digest, ImageLen};

    pub(crate) fn image(slot: Slot, minor: u16) -> ImageId {
        ImageId::new(
            slot,
            Version::new(1, minor, 0),
            ImageLen::new(8192).unwrap(),
            Digest::new([minor as u8; 32]),
        )
    }

    /// A record with `update` in flight from slot A to slot B.
    pub(crate) fn in_flight(update: fn(ImageId) -> Update) -> Record {
        Record {
            update: update(image(Slot::B, 1)),
            hold_track_off: true,
            ..Record::installed(image(Slot::A, 0))
        }
    }

    #[test]
    fn a_failed_transfer_stays_retryable_and_a_failed_trial_does_not() {
        let installed = Record::installed(image(Slot::A, 0));
        let receiving = installed.begin(image(Slot::B, 3)).unwrap();
        let aborted = receiving.end_transfer(Ending::NotBootable).unwrap();
        assert_eq!(aborted.update().ending(), Some(Ending::NotBootable));
        assert_eq!(aborted.rejected(), None);
        let retry = aborted.begin(image(Slot::B, 3)).unwrap();
        assert_eq!(
            retry
                .end_transfer(Ending::Aborted)
                .unwrap()
                .update()
                .ending(),
            Some(Ending::Aborted)
        );
        let trying = Record {
            update: Update::Trying(image(Slot::B, 3)),
            ..aborted
        };
        let rolled_back = trying.roll_back().unwrap();
        assert_eq!(rolled_back.rejected(), Some(Version::new(1, 3, 0)));
        // A later, lower failure never lowers the floor.
        let lower = Record {
            update: Update::Trying(image(Slot::B, 2)),
            ..rolled_back
        };
        assert_eq!(
            lower.roll_back().unwrap().rejected(),
            Some(Version::new(1, 3, 0))
        );
    }

    #[test]
    fn transitions_refuse_the_wrong_step_or_slot() {
        let installed = Record::installed(image(Slot::A, 0));
        assert_eq!(
            installed.begin(image(Slot::A, 1)),
            Err(StateError::WrongPhase)
        );
        assert_eq!(installed.ready(image(Slot::B, 1)), Err(StateError::Corrupt));
        assert!(installed.end_transfer(Ending::Aborted).is_err());
        let receiving = installed.begin(image(Slot::B, 1)).unwrap();
        assert_eq!(
            receiving.begin(image(Slot::B, 2)),
            Err(StateError::WrongPhase)
        );
        assert_eq!(receiving.ready(image(Slot::B, 2)), Err(StateError::Corrupt));
        assert_eq!(
            receiving.roll_back(),
            Err(StateError::WrongPhase),
            "never booted"
        );
        assert_eq!(installed.roll_back(), Err(StateError::WrongPhase));
        let ready = receiving.ready(image(Slot::B, 1)).unwrap();
        assert_eq!(
            ready.confirm(Slot::B),
            Err(StateError::Corrupt),
            "not booted yet"
        );
        let trying = in_flight(Update::Trying);
        assert_eq!(trying.confirm(Slot::A), Err(StateError::Corrupt));
        let confirmed = trying.confirm(Slot::B).unwrap();
        assert_eq!(confirmed.current(), image(Slot::B, 1));
        assert_eq!(
            confirmed.update(),
            Update::Ended(
                image(Slot::B, 1),
                Ending::Confirmed {
                    previous: image(Slot::A, 0)
                }
            )
        );
    }
}
