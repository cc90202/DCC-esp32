//! The trial of a new image, one monitor tick at a time, and the arming of
//! the track once boot is complete.

use core::time::Duration;

use super::{Device, Event, Restart, Runtime, state::Lock};
use crate::{
    Error, StateError,
    admission::{Lifecycle, Step},
    journal::{Ending, Record, store},
    notice::Notice,
    sequence,
    status::Stage,
    trial::{Decision, Window},
};

/// The trial's health history, kept by the monitor across ticks. It starts
/// when the trial does, not when the monitor does: the boot check before it
/// may take a while.
#[derive(Debug, Default)]
pub struct Trial(Option<Window>);

impl<D: Device> Runtime<'_, D> {
    /// One monitor tick: asks for the restart an abandoned transfer needs,
    /// retries a boot arming that failed and runs the trial, confirming or
    /// rolling back when its window decides. A failure the journal cannot
    /// explain ends in recovery.
    pub async fn tick(self, trial: &mut Trial) -> Option<Restart> {
        let (lifecycle, arming_pending, lock) = self
            .device
            .state(|s| (s.lifecycle(), s.arming_pending, s.lock));
        if lock == Lock::Abandoned {
            return Some(Restart);
        }
        if arming_pending && lifecycle.is_running() {
            self.arm_after_boot().await;
        }
        if lifecycle != Lifecycle::Trial {
            return None;
        }
        let window = trial
            .0
            .get_or_insert_with(|| Window::new(self.device.now()));
        let now = self.device.now();
        let decided = match window.observe(now, self.device.health()) {
            Decision::Wait => {
                self.count_down(window.left(now));
                Ok(None)
            }
            Decision::Rollback => self.roll_back().await,
            Decision::Confirm => self.confirm(window).await,
        };
        decided.unwrap_or_else(|error| {
            self.enter_recovery(error);
            None
        })
    }

    /// Publishes what is left of the healthy minute, and shows it whenever
    /// the whole seconds change.
    fn count_down(self, left: Option<Duration>) {
        let seconds = |left: Option<Duration>| left.map(|left| left.as_secs());
        let (changed, version) = self.device.state(|s| {
            let changed = seconds(s.trial_left) != seconds(left);
            s.trial_left = left;
            (
                changed,
                s.record.and_then(|r| r.candidate()).map(|c| c.version()),
            )
        });
        if changed && let Some(version) = version {
            self.device.show(Notice::Trial { version, left });
        }
    }

    /// Arms the track once every critical task is ready, unless a trial or
    /// recovery keeps it off. A handshake that fails, for example because a
    /// fault arrived meanwhile, leaves the track inhibited and the next
    /// tick retries: the journal and flash are fine, so this is no reason
    /// for recovery.
    pub async fn arm_after_boot(self) {
        if !self.device.state(|s| s.lifecycle().is_running()) {
            return;
        }
        let result = if self.device.state(|s| s.holds_track_off()) {
            self.arm_stopped().await
        } else {
            self.arm().await
        };
        if let Err(error) = result {
            self.device.report(Event::ArmingRetried(error));
        }
        self.device.state(|s| s.arming_pending = result.is_err());
    }

    /// Leaves the trial without blaming the candidate, which stays eligible
    /// for a new upload, and goes back to the confirmed image: the operator
    /// chose Wi-Fi setup, where a trial cannot prove anything.
    ///
    /// # Errors
    /// Returns [`Error::Forbidden`] outside a trial, or the restore error.
    pub async fn withdraw_trial(self) -> Result<Option<Restart>, Error> {
        self.end_trial(Record::withdraw_trial).await
    }

    /// Rejects the candidate on trial and goes back to the confirmed image.
    async fn roll_back(self) -> Result<Option<Restart>, Error> {
        self.end_trial(Record::roll_back).await
    }

    /// Ends the trial through `end` and goes back to the confirmed image.
    async fn end_trial(
        self,
        end: fn(Record) -> Result<Record, StateError>,
    ) -> Result<Option<Restart>, Error> {
        let (record, _) = self.record()?;
        self.restore(end(record)?).await
    }

    /// Confirms the candidate if the trial still allows it once the track is
    /// stopped.
    ///
    /// Before the journal commit every failure only postpones the decision;
    /// the trial deadline still rolls back. After it the image is confirmed,
    /// so a failure only needs a restart to finish.
    async fn confirm(self, window: &Window) -> Result<Option<Restart>, Error> {
        if let Err(error) = self.device.stop().await {
            self.device.report(Event::ConfirmPostponed(error));
            return Ok(None);
        }
        let (trying, booted) = self.record()?;
        let committed = self
            .with_flash(|flash| {
                // Health again right before the durable commit.
                if !window.eligible(self.device.now(), self.device.health()) {
                    return Ok(None);
                }
                sequence::confirm(flash, trying, booted).map(Some)
            })
            .await;
        let saved = match committed {
            Ok(Some(saved)) => saved,
            Ok(None) => return Ok(None),
            Err(error) => {
                let landed = self.with_flash(store::load).await;
                // The confirmation landed and only the selection failed: the
                // next boot repairs it. Otherwise the journal still holds the
                // trial and the next tick tries again; the trial deadline
                // rolls back if the flash never recovers.
                if let Ok(Some(landed)) = landed
                    && matches!(landed.update().ending(), Some(Ending::Confirmed { .. }))
                {
                    self.device.report(Event::ConfirmedRestarting(error));
                    return Ok(Some(Restart));
                }
                self.device.report(Event::ConfirmRetried(error));
                return Ok(None);
            }
        };
        self.device.state(|s| {
            s.record = Some(saved);
            s.advance(Step::Confirm);
            s.progress.stage = Stage::Idle;
        });
        if let Err(error) = self.arm().await {
            self.device.report(Event::ConfirmedRestarting(error));
            return Ok(Some(Restart));
        }
        self.device.inhibit(false);
        self.device
            .show(Notice::Confirmed(saved.current().version()));
        Ok(None)
    }
}
