//! The update runtime: what the device does at boot, during a trial, during
//! a transfer and on the way back to a trusted image, step by step.
//!
//! Every effect on the device, flash, clock, track, display and watchdog,
//! goes through [`Device`] and the smaller traits it builds on, so the
//! whole flow runs on the host against a simulated device. The firmware
//! implements them and keeps only the tasks, the network and the restart
//! itself: a step that ends in a restart returns [`Restart`] and the caller
//! restarts.
//!
//! - [`Runtime::start`] identifies the running image and reconciles the
//!   journal;
//! - [`Runtime::tick`] retries the boot arming and runs the trial;
//! - [`Runtime::acquire`] admits one operation and returns the
//!   [`UpdateGuard`] that drives a transfer;
//! - [`Runtime::withdraw_trial`] leaves a trial for Wi-Fi setup.

mod boot;
mod recovery;
mod state;
mod transfer;
mod trial;

#[cfg(test)]
pub(crate) mod sim;
#[cfg(test)]
mod tests;

use core::time::Duration;

use dcc_ota_format::{Digest, ImageLen, Version, image::BuildKind};
use ed25519_dalek::VerifyingKey;
use embedded_storage::nor_flash::NorFlash;

pub use state::State;
pub use transfer::{Admitted, UpdateGuard, Written};
pub use trial::Trial;

use crate::{
    Error, Refusal,
    admission::{Lifecycle, Step},
    image::{Hasher, ImageId},
    journal::Ending,
    journal::{Record, store},
    layout::{SECTOR_SIZE, Slot},
    notice::Notice,
    status::{JSON_CAPACITY, Snapshot},
    trial::Health,
    uptime::Uptime,
};

/// Pause between two hashed or verified sectors, so other tasks run.
const SECTOR_YIELD: Duration = Duration::from_millis(1);

/// Time since boot, and waiting.
#[expect(async_fn_in_trait, reason = "single-threaded executor; no Send bound")]
pub trait Clock {
    /// Time since boot.
    fn now(&self) -> Uptime;

    /// Lets other tasks run for `duration`.
    async fn pause(&self, duration: Duration);
}

/// The track power, as the fault manager controls it.
///
/// # Contract
/// [`Self::is_on`] and [`Self::inhibit`] are called inside
/// [`Device::state`], so the admission decision and the lock are taken
/// together; they must not call [`Device::state`] themselves.
#[expect(async_fn_in_trait, reason = "single-threaded executor; no Send bound")]
pub trait Track {
    /// Whether the track is powered.
    fn is_on(&self) -> bool;

    /// Keeps the track off whatever the operator asks, or lets the operator
    /// power it again.
    fn inhibit(&self, inhibited: bool);

    /// Stops the track and waits until every frame queued before the stop
    /// has left the engine.
    ///
    /// # Errors
    /// Returns [`Error::Timeout`] if the stop is not confirmed in time.
    async fn stop(&self) -> Result<(), Error>;

    /// Arms track power and waits until it is applied.
    ///
    /// # Errors
    /// Returns [`Error::Timeout`] if the arming is not confirmed in time.
    async fn arm(&self) -> Result<(), Error>;
}

/// The flash holding the slots, the bootloader selection and the journal.
#[expect(async_fn_in_trait, reason = "single-threaded executor; no Send bound")]
pub trait Storage {
    /// The flash driver.
    type Flash: NorFlash;

    /// Runs `step` with the flash locked; `None` before the flash is open.
    async fn flash<R>(&self, step: impl FnOnce(&mut Self::Flash) -> R) -> Option<R>;
}

/// What the operator and the log learn about the update.
pub trait Feedback {
    /// Shows an update step on the display.
    fn show(&self, notice: Notice);

    /// Hands over something worth logging.
    fn report(&self, event: Event);
}

/// What the update runtime needs from the device besides time, track,
/// flash and feedback.
#[expect(async_fn_in_trait, reason = "single-threaded executor; no Send bound")]
pub trait Device: Clock + Track + Storage + Feedback {
    /// Version of the running build.
    fn version(&self) -> Version;

    /// Signing policy of the running build; updates must be images it
    /// [accepts](BuildKind::accepts).
    fn build_kind(&self) -> BuildKind;

    /// Reads or changes the shared state, atomically; see the contract of
    /// [`Track`].
    fn state<R>(&self, change: impl FnOnce(&mut State) -> R) -> R;

    /// Health of the tasks and services a trial watches.
    fn health(&self) -> Health;

    /// The key this build trusts for its next update.
    ///
    /// # Errors
    /// Returns the error to report when the embedded key is unusable.
    fn trusted_key(&self) -> Result<VerifyingKey, Error>;

    /// Waits until the watchdog guards the operation that holds the lock.
    ///
    /// # Errors
    /// Returns [`Error::Timeout`] if the watchdog does not start in time.
    async fn guarded(&self) -> Result<(), Error>;

    /// Recovery was just recorded in the state and the track inhibited:
    /// stop every other path to the track and ask for a USB flash.
    fn recovery_entered(&self, error: Error);
}

/// The chip must restart now; the step that returns it has finished.
#[must_use = "the chip must restart"]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Restart;

/// Something the runtime did that the device may log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// The new image failed its own boot check and is rolled back.
    CandidateFailedBootCheck(Error),
    /// Track arming after boot was not confirmed; the next tick retries.
    ArmingRetried(Error),
    /// The track could not be stopped for the confirmation; the trial goes
    /// on.
    ConfirmPostponed(Error),
    /// The confirmation was not saved; the next tick retries.
    ConfirmRetried(Error),
    /// The candidate is confirmed but a later step failed; the restart that
    /// follows finishes the job.
    ConfirmedRestarting(Error),
    /// A transfer was dropped without an abort; the lock stays held and the
    /// next tick restarts, which cleans up from the journal.
    TransferAbandoned,
}

/// The update runtime on one device.
pub struct Runtime<'d, D> {
    device: &'d D,
}

impl<D> Clone for Runtime<'_, D> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<D> Copy for Runtime<'_, D> {}

impl<D> core::fmt::Debug for Runtime<'_, D> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Runtime").finish_non_exhaustive()
    }
}

impl<'d, D: Device> Runtime<'d, D> {
    /// The runtime acting on `device`.
    #[must_use]
    pub const fn new(device: &'d D) -> Self {
        Self { device }
    }

    /// Renders `GET /update/status`.
    ///
    /// # Errors
    /// Returns [`Error::Internal`] if the document overflows its buffer.
    pub fn status(self) -> Result<heapless::String<JSON_CAPACITY>, Error> {
        let track_enabled = self.device.is_on();
        let uptime = self.device.now();
        self.device
            .state(|s| {
                // A rollback is final only once the restart that follows it
                // has booted the confirmed image; until then the trial runs.
                let rolling_back = s.lifecycle() == Lifecycle::Trial
                    && s.record.and_then(|r| r.update().ending()) == Some(Ending::RolledBack);
                let reason = s
                    .progress
                    .reason
                    .or_else(|| rolling_back.then_some(Error::Refused(Refusal::PendingVerify)));
                Snapshot {
                    record: s.record,
                    boot_id: s.boot_id,
                    uptime,
                    version: self.device.version(),
                    slot: s.slot,
                    lifecycle: s.lifecycle(),
                    busy: s.busy(),
                    track_enabled,
                    transfer: s.progress.transfer,
                    stage: s.progress.stage,
                    reason,
                    trial_left: s.trial_left.filter(|_| s.lifecycle() == Lifecycle::Trial),
                }
            })
            .json()
    }

    /// Stops every update path for good: records `error`, keeps the track
    /// off, and lets the device do the rest.
    pub fn enter_recovery(self, error: Error) {
        self.device.state(|s| {
            s.enter_recovery(error);
            self.device.inhibit(true);
        });
        self.device.recovery_entered(error);
    }

    /// Runs `step` with the flash locked, once per sector, pausing between
    /// sectors so other tasks keep running, until it returns a value.
    async fn step_until<R, E: Into<Error>>(
        self,
        mut step: impl FnMut(&mut D::Flash) -> Result<Option<R>, E>,
    ) -> Result<R, Error> {
        loop {
            if let Some(done) = self.with_flash(&mut step).await? {
                return Ok(done);
            }
            self.device.pause(SECTOR_YIELD).await;
        }
    }

    /// Runs `step` with the flash locked.
    async fn with_flash<R, E: Into<Error>>(
        self,
        step: impl FnOnce(&mut D::Flash) -> Result<R, E>,
    ) -> Result<R, Error> {
        self.device
            .flash(step)
            .await
            .ok_or(Error::Unavailable)?
            .map_err(Into::into)
    }

    /// The shared record and the running slot.
    fn record(self) -> Result<(Record, Slot), Error> {
        let (record, slot) = self.device.state(|s| (s.record, s.slot));
        Ok((record.ok_or(Error::Unavailable)?, slot))
    }

    /// Runs a write sequence on the running slot and keeps what it saved
    /// as the shared record.
    async fn apply<E: Into<Error>>(
        self,
        sequence: impl FnOnce(&mut D::Flash, Slot) -> Result<Record, E>,
    ) -> Result<Record, Error> {
        let booted = self.device.state(|s| s.slot);
        let saved = self.with_flash(|flash| sequence(flash, booted)).await?;
        self.device.state(|s| s.record = Some(saved));
        Ok(saved)
    }

    /// Saves `record` and keeps the stored copy as the shared record.
    async fn save(self, record: Record) -> Result<Record, Error> {
        self.apply(|flash, _| store::save(flash, record)).await
    }

    /// Hashes the first `len` bytes of `slot`, one sector per lock of the
    /// flash, pausing between sectors so other tasks keep running.
    async fn hash(self, slot: Slot, len: ImageLen) -> Result<Digest, Error> {
        let mut hasher = Hasher::new(slot, len);
        let mut buffer = [0; SECTOR_SIZE];
        self.step_until(|flash| hasher.step(flash, &mut buffer))
            .await
    }

    /// Whether the image `id` names is intact in its slot.
    async fn image_intact(self, id: ImageId) -> Result<bool, Error> {
        Ok(self.hash(id.slot(), id.image_len()).await? == id.digest())
    }

    /// Arms the track and records it; the inhibit stays until the arming
    /// is applied, so an older resume request queued before it is refused.
    async fn arm(self) -> Result<(), Error> {
        self.device.arm().await?;
        self.device.state(|s| s.advance(Step::Arming));
        Ok(())
    }

    /// Arms with the track stopped, then lifts the inhibit.
    async fn arm_stopped(self) -> Result<(), Error> {
        self.device.stop().await?;
        self.arm().await?;
        self.device.inhibit(false);
        Ok(())
    }
}
