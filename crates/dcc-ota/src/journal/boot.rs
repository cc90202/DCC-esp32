//! What a boot does with the journal, given the image that actually started.

use super::{Ending, Record, Update};
use crate::{StateError, image::ImageId, layout::Slot};

/// Whether the recorded current image is intact in flash.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurrentImage {
    /// Its hash matches the journal.
    Intact,
    /// It cannot be booted back into.
    Damaged,
}

/// Why the chip last restarted, as far as a trial cares.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetCause {
    /// Power was lost or sagged, or a host on USB or JTAG reset the chip:
    /// not the image's fault.
    External,
    /// Software restart, crash or watchdog: the running image is to blame.
    Other,
}

/// Who a failed boot check of a candidate is blamed on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Blame {
    /// The candidate itself: it is rejected for good.
    Candidate,
    /// The device, for example a flash read that failed: the candidate
    /// stays eligible for a new upload.
    Device,
}

/// Decision taken at boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootAction {
    /// Run the booted image; the record is unchanged.
    Run(Record),
    /// Run the booted image with the track off until the operator powers
    /// it: the first ordinary boot after an update. Save the record, which
    /// no longer holds the track for the boots that follow.
    RunStopped(Record),
    /// Start the trial of the booted candidate; save the record first.
    Trial(Record),
    /// Save the record and boot its current image.
    Restore(Record),
}

impl Record {
    /// The candidate selected for this boot has started from `booted`: it
    /// is on trial from this moment, before any check of it runs, so a
    /// candidate that hangs in those checks is rolled back by the restart
    /// that follows instead of being started again.
    ///
    /// Returns `None` unless a ready candidate lives in `booted`.
    #[must_use]
    pub(crate) fn start_trial(mut self, booted: Slot) -> Option<Self> {
        let Update::Ready(candidate) = self.update else {
            return None;
        };
        if candidate.slot() != booted {
            return None;
        }
        self.update = Update::Trying(candidate);
        self.hold_track_off = true;
        Some(self)
    }

    /// Decides what to do now that `booted` has started.
    ///
    /// `booted` is the hashed identity of the running image; `current` says
    /// whether the recorded current image is intact in flash; `cause` is why
    /// the chip restarted. A trial cut short by a power loss or an external reset goes back to
    /// the current image without rejecting the candidate, which can then be
    /// uploaded again; any other interrupted trial rejects it.
    ///
    /// # Errors
    /// Returns [`StateError::Corrupt`] when no trusted image can be named.
    pub(crate) fn on_boot(
        self,
        booted: ImageId,
        current: CurrentImage,
        cause: ResetCause,
    ) -> Result<BootAction, StateError> {
        if self.update.is_settled() {
            self.on_settled_boot(booted, current)
        } else {
            self.on_update_boot(booted, current, cause)
        }
    }

    /// A boot with no update in flight: run the current image, or go back
    /// to it.
    fn on_settled_boot(
        mut self,
        booted: ImageId,
        current: CurrentImage,
    ) -> Result<BootAction, StateError> {
        if self.current == booted {
            return Ok(if self.hold_track_off {
                BootAction::RunStopped(self.release_track())
            } else {
                BootAction::Run(self)
            });
        }
        if let Update::Ended(_, Ending::Confirmed { previous }) = self.update
            && previous == booted
        {
            // The bootloader fell back to the image confirmed before: the
            // current one no longer boots.
            let failed = self.current;
            self.reject(failed);
            self.current = booted;
            self.update = Update::Ended(failed, Ending::NotBootable);
            self.hold_track_off = true;
            return Ok(BootAction::Restore(self));
        }
        match current {
            CurrentImage::Intact => Ok(BootAction::Restore(self)),
            CurrentImage::Damaged => Err(StateError::Corrupt),
        }
    }

    /// A boot during an update: start the trial of the ready candidate, or
    /// go back to the current image.
    fn on_update_boot(
        mut self,
        booted: ImageId,
        current: CurrentImage,
        cause: ResetCause,
    ) -> Result<BootAction, StateError> {
        if current == CurrentImage::Damaged {
            return Err(StateError::Corrupt);
        }
        self.hold_track_off = true;
        let restored = match self.update {
            Update::Ready(candidate) if candidate == booted => {
                self.update = Update::Trying(candidate);
                return Ok(BootAction::Trial(self));
            }
            Update::Trying(_) if cause == ResetCause::External => self.withdraw_trial(),
            Update::Trying(_) => self.roll_back(),
            _ => self.end_transfer(Ending::Aborted),
        };
        restored.map(BootAction::Restore)
    }

    /// Ends a trial that was cut short through no fault of the candidate, a
    /// power loss, an external reset or the operator entering Wi-Fi setup: back to the current
    /// image, the candidate stays eligible for a new upload.
    ///
    /// # Errors
    /// Returns [`StateError::WrongPhase`] outside a trial.
    pub(crate) fn withdraw_trial(mut self) -> Result<Self, StateError> {
        let Update::Trying(candidate) = self.update else {
            return Err(StateError::WrongPhase);
        };
        self.update = Update::Ended(candidate, Ending::Aborted);
        self.hold_track_off = true;
        Ok(self)
    }

    /// Ends the trial of a candidate that started but failed its own boot
    /// checks, rejecting it unless `blame` falls on the device.
    ///
    /// Returns `None` when `booted` is not a candidate the journal expected,
    /// so there is nothing safe to restore.
    #[must_use]
    pub(crate) fn on_failed_candidate_boot(self, booted: Slot, blame: Blame) -> Option<Self> {
        // A selected candidate that started has booted, even if it failed
        // before recording its trial.
        let trying = match self.update {
            Update::Trying(candidate) if candidate.slot() == booted => self,
            _ => self.start_trial(booted)?,
        };
        match blame {
            Blame::Candidate => trying.roll_back().ok(),
            Blame::Device => trying.withdraw_trial().ok(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::journal::tests::{image, in_flight};
    use dcc_ota_format::{Digest, Version};

    type Step = fn(ImageId) -> Update;
    const STEPS: [Step; 3] = [Update::Receiving, Update::Ready, Update::Trying];

    #[test]
    fn only_the_ready_candidate_starts_one_trial() {
        let old = image(Slot::A, 0);
        let new = image(Slot::B, 1);
        let unknown = image(Slot::B, 2);
        for step in STEPS {
            let record = in_flight(step);
            for booted in [old, new, unknown] {
                let action = record
                    .on_boot(booted, CurrentImage::Intact, ResetCause::Other)
                    .unwrap();
                if record.update() == Update::Ready(new) && booted == new {
                    let BootAction::Trial(trial) = action else {
                        panic!("missing trial")
                    };
                    assert_eq!(trial.update(), Update::Trying(new));
                    // A second boot of the same trial is a failed trial.
                    let Ok(BootAction::Restore(again)) =
                        trial.on_boot(new, CurrentImage::Intact, ResetCause::Other)
                    else {
                        panic!("a trial must not restart")
                    };
                    assert_eq!(again.update(), Update::Ended(new, Ending::RolledBack));
                } else {
                    let BootAction::Restore(restored) = action else {
                        panic!("untrusted boot")
                    };
                    assert_eq!(restored.current(), old);
                    assert!(restored.holds_track_off());
                    assert_eq!(
                        restored.rejected(),
                        (record.update() == Update::Trying(new)).then_some(new.version())
                    );
                }
            }
            assert_eq!(
                record.on_boot(new, CurrentImage::Damaged, ResetCause::Other),
                Err(StateError::Corrupt)
            );
        }
    }

    #[test]
    fn fallback_needs_the_full_previous_identity() {
        let confirmed = in_flight(Update::Trying).confirm(Slot::B).unwrap();
        let Ok(BootAction::Restore(fallback)) =
            confirmed.on_boot(image(Slot::A, 0), CurrentImage::Damaged, ResetCause::Other)
        else {
            panic!("missing fallback")
        };
        assert_eq!(fallback.current(), image(Slot::A, 0));
        assert_eq!(fallback.rejected(), Some(image(Slot::B, 1).version()));
        let original = image(Slot::A, 0);
        let changed = ImageId::new(
            Slot::A,
            original.version(),
            original.image_len(),
            Digest::new([9; 32]),
        );
        assert_eq!(
            confirmed.on_boot(changed, CurrentImage::Damaged, ResetCause::Other),
            Err(StateError::Corrupt)
        );
        // The first boot of the confirmed image waits for the operator;
        // the ones after it do not.
        let Ok(BootAction::RunStopped(released)) =
            confirmed.on_boot(image(Slot::B, 1), CurrentImage::Intact, ResetCause::Other)
        else {
            panic!("the boot after an update must hold the track")
        };
        assert!(confirmed.holds_track_off());
        assert!(!released.holds_track_off());
        assert_eq!(released.current(), confirmed.current());
        assert_eq!(released.update(), confirmed.update());
        assert_eq!(
            released.on_boot(image(Slot::B, 1), CurrentImage::Intact, ResetCause::Other),
            Ok(BootAction::Run(released))
        );
        assert!(matches!(
            confirmed.on_boot(changed, CurrentImage::Intact, ResetCause::Other),
            Ok(BootAction::Restore(_))
        ));
    }

    #[test]
    fn an_interrupted_transfer_keeps_the_earlier_rejection() {
        let Ok(BootAction::Restore(rejected)) = in_flight(Update::Trying).on_boot(
            image(Slot::A, 0),
            CurrentImage::Intact,
            ResetCause::Other,
        ) else {
            panic!("missing rollback")
        };
        let receiving = rejected.begin(image(Slot::B, 2)).unwrap();
        let Ok(BootAction::Restore(aborted)) =
            receiving.on_boot(image(Slot::A, 0), CurrentImage::Intact, ResetCause::Other)
        else {
            panic!("missing abort")
        };
        assert_eq!(
            aborted.update(),
            Update::Ended(image(Slot::B, 2), Ending::Aborted)
        );
        assert_eq!(aborted.rejected(), Some(Version::new(1, 1, 0)));
    }

    #[test]
    fn a_candidate_failing_its_own_checks_is_rolled_back_unless_the_device_failed() {
        for step in [Update::Ready as Step, Update::Trying] {
            let failed = in_flight(step)
                .on_failed_candidate_boot(Slot::B, Blame::Candidate)
                .unwrap();
            assert_eq!(failed.update().ending(), Some(Ending::RolledBack));
            assert_eq!(failed.rejected(), Some(image(Slot::B, 1).version()));
            assert!(failed.holds_track_off());
            let withdrawn = in_flight(step)
                .on_failed_candidate_boot(Slot::B, Blame::Device)
                .unwrap();
            assert_eq!(withdrawn.update().ending(), Some(Ending::Aborted));
            assert_eq!(withdrawn.rejected(), None);
        }
        let failed = |record: Record, slot| record.on_failed_candidate_boot(slot, Blame::Candidate);
        assert_eq!(failed(in_flight(Update::Ready), Slot::A), None);
        assert_eq!(failed(in_flight(Update::Receiving), Slot::B), None);
        assert_eq!(failed(Record::installed(image(Slot::A, 0)), Slot::A), None);
    }

    #[test]
    fn only_a_ready_candidate_in_the_booted_slot_starts_its_trial() {
        let trying = in_flight(Update::Ready).start_trial(Slot::B).unwrap();
        assert_eq!(trying.update(), Update::Trying(image(Slot::B, 1)));
        assert_eq!(in_flight(Update::Ready).start_trial(Slot::A), None);
        assert_eq!(in_flight(Update::Receiving).start_trial(Slot::B), None);
        assert_eq!(in_flight(Update::Trying).start_trial(Slot::B), None);
    }

    #[test]
    fn a_trial_ended_by_power_loss_or_setup_keeps_the_candidate_eligible() {
        let trying = in_flight(Update::Trying);
        let new = image(Slot::B, 1);
        let Ok(BootAction::Restore(interrupted)) =
            trying.on_boot(new, CurrentImage::Intact, ResetCause::External)
        else {
            panic!("a trial cut by a power loss must go back to the current image")
        };
        let withdrawn = trying.withdraw_trial().unwrap();
        for ended in [interrupted, withdrawn] {
            assert_eq!(ended.current(), image(Slot::A, 0));
            assert_eq!(ended.rejected(), None);
            assert_eq!(ended.update(), Update::Ended(new, Ending::Aborted));
            assert!(ended.holds_track_off());
            // The same release can be uploaded again.
            assert!(ended.begin(new).is_ok());
        }
        let Ok(BootAction::Restore(failed)) =
            trying.on_boot(new, CurrentImage::Intact, ResetCause::Other)
        else {
            panic!("a crashed trial must go back too")
        };
        assert_eq!(failed.rejected(), Some(new.version()));
        assert_eq!(
            in_flight(Update::Ready).withdraw_trial(),
            Err(StateError::WrongPhase)
        );
    }
}
