//! The update lock and the publication order of one transfer.
//!
//! [`Runtime::acquire`] admits one operation at a time and stops the track;
//! the returned [`UpdateGuard`] drives the transfer and, when dropped, lifts
//! the lock. Each step hands the next one a proof only the step before can
//! make: [`UpdateGuard::verify`] an [`Admitted`] package for
//! [`UpdateGuard::begin`], [`UpdateGuard::finish`] a [`Written`] image for
//! [`UpdateGuard::activate`], which consumes the guard and ends in a restart
//! or, if cleaning up failed, in recovery.

use core::time::Duration;

use dcc_ota_format::{
    ImageLen,
    package::{HEADER_LEN, Manifest, PackageError},
};

use super::{Device, Event, Restart, Runtime, state::Lock};
use crate::{
    Error,
    admission::{Admission, Lifecycle, Operation},
    image::ImageId,
    journal::{Ending, Record, Update, store},
    notice::{Notice, Tenths},
    sequence,
    status::{Stage, Transfer},
    uptime::Uptime,
    writer::{Buffers, Writer},
};

/// Minimum spacing between two update starts.
const UPDATE_SPACING: Duration = Duration::from_secs(1);
/// Pause before another operation after a failed transfer.
const FAILURE_BACKOFF: Duration = Duration::from_secs(30);
/// Time for the reply to leave the radio and the display to draw before
/// the restart; the same as the provisioning reboot.
const RESTART_DELAY: Duration = Duration::from_millis(500);

/// How a transfer that failed with `error` ends. A broken connection or
/// package may simply be sent again; anything else means the image could
/// not be made bootable. Every kind is listed, so a new one forces a
/// decision here.
const fn ending_after(error: Error) -> Ending {
    match error {
        Error::Timeout
        | Error::Network
        | Error::Request(_)
        | Error::Package(PackageError::Malformed) => Ending::Aborted,
        Error::Flash(_)
        | Error::Corrupt
        | Error::Forbidden
        | Error::Unavailable
        | Error::Internal
        | Error::Package(_)
        | Error::Image(_)
        | Error::Ineligible(_)
        | Error::Refused(_) => Ending::NotBootable,
    }
}

/// A package this device accepts: authenticated with the trusted key and
/// eligible against the journal. Only [`UpdateGuard::verify`] makes one.
#[derive(Debug, Clone, Copy)]
pub struct Admitted(Manifest);

impl Admitted {
    /// Length of the image that follows the header.
    #[must_use]
    pub const fn image_len(&self) -> ImageLen {
        self.0.fields().image_len()
    }
}

/// An image written, read back and made bootable in the inactive slot.
/// Only [`UpdateGuard::finish`] makes one.
#[derive(Debug, Clone, Copy)]
pub struct Written(ImageId);

/// How far the guarded operation got.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// The lock is held; nothing reached the inactive slot.
    Admitted,
    /// Bytes may have reached the inactive slot, so cleanup must run.
    Receiving,
    /// The image is selected: the lock stays held until the restart.
    Selected,
}

impl<'d, D: Device> Runtime<'d, D> {
    /// Takes the lock for `operation` and stops the track.
    ///
    /// # Errors
    /// Returns the admission refusal, or [`Error::Timeout`] when the
    /// watchdog or the track does not answer in time.
    pub async fn acquire(self, operation: Operation) -> Result<UpdateGuard<'d, D>, Error> {
        let now = self.device.now();
        // Decide and take the lock in one critical section, track included.
        self.device.state(|s| {
            Admission {
                lifecycle: s.lifecycle(),
                busy: s.lock != Lock::Free,
                track_on: self.device.is_on(),
                backoff_until: s.limits.backoff_until,
                next_update: s.limits.next_update,
            }
            .check(now, operation)?;
            s.lock = Lock::Held;
            self.device.inhibit(true);
            if operation == Operation::Update {
                s.limits.next_update = now + UPDATE_SPACING;
            }
            Ok::<(), Error>(())
        })?;
        let guard = UpdateGuard {
            runtime: self,
            phase: Phase::Admitted,
            shown: Tenths::ZERO,
        };
        // No signature or flash work until the watchdog guards this
        // operation.
        self.device.guarded().await?;
        self.device.stop().await?;
        Ok(guard)
    }
}

/// Holds the update lock; dropping it releases the lock. Holding one also
/// proves the track was stopped when the lock was taken.
pub struct UpdateGuard<'d, D: Device> {
    runtime: Runtime<'d, D>,
    phase: Phase,
    /// Progress last shown on the display.
    shown: Tenths,
}

impl<D: Device> Drop for UpdateGuard<'_, D> {
    fn drop(&mut self) {
        let device = self.runtime.device;
        match self.phase {
            Phase::Selected => return,
            Phase::Receiving => {
                // Dropped mid-transfer without a finished abort: the journal
                // still holds the transfer. Keep the lock and the inhibit,
                // and let the restart that follows clean up from the journal.
                device.report(Event::TransferAbandoned);
                device.state(|s| s.lock = Lock::Abandoned);
                return;
            }
            Phase::Admitted => {}
        }
        device.state(|s| {
            s.lock = Lock::Free;
            if s.lifecycle().is_running() {
                device.inhibit(false);
            }
        });
    }
}

impl<D: Device> core::fmt::Debug for UpdateGuard<'_, D> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("UpdateGuard")
            .field("phase", &self.phase)
            .finish_non_exhaustive()
    }
}

impl<D: Device> UpdateGuard<'_, D> {
    /// Authenticates a package header and checks it against the version
    /// policy. Holding the guard proves the track is stopped.
    ///
    /// # Errors
    /// Returns the authentication or eligibility error.
    pub fn verify(&self, header: &[u8; HEADER_LEN]) -> Result<Admitted, Error> {
        let (record, _) = self.runtime.record()?;
        let key = self.runtime.device.trusted_key()?;
        let manifest = Manifest::authenticate(header, &key)?;
        manifest.check_eligibility(record.current().version(), record.rejected())?;
        Ok(Admitted(manifest))
    }

    /// Journals the transfer and opens the writer on the inactive slot.
    ///
    /// # Errors
    /// Returns [`Error::Forbidden`] when the journal refuses a new transfer,
    /// or a flash error. A journal that cannot be written enters recovery.
    pub async fn begin<'b>(
        &mut self,
        Admitted(manifest): Admitted,
        buffers: &'b mut Buffers,
    ) -> Result<Writer<'b>, Error> {
        let runtime = self.runtime;
        let (record, booted) = runtime.record()?;
        let target = ImageId::of(&manifest, booted.other());
        record.begin(target)?;
        if let Err(error) = runtime
            .apply(|flash, _| sequence::begin(flash, record, target))
            .await
        {
            return Err(self.after_failed_begin(record, error).await);
        }
        self.phase = Phase::Receiving;
        let writer = runtime
            .with_flash(|flash| {
                Writer::new(
                    manifest,
                    booted,
                    runtime.device.build_kind(),
                    buffers,
                    flash,
                )
            })
            .await?;
        runtime.device.state(|s| s.progress.reason = None);
        self.progress(&writer, Stage::Receiving);
        self.shown = Tenths::ZERO;
        runtime.device.show(Notice::Receiving(Tenths::ZERO));
        Ok(writer)
    }

    /// After [`sequence::begin`] failed: if the journal already records the
    /// transfer, the slot may hold bytes and the normal cleanup applies; if
    /// it still holds `before`, nothing happened; a journal that cannot be
    /// read means recovery.
    async fn after_failed_begin(&mut self, before: Record, error: Error) -> Error {
        let runtime = self.runtime;
        match runtime.with_flash(store::load).await {
            Ok(Some(landed)) if matches!(landed.update(), Update::Receiving(_)) => {
                runtime.device.state(|s| s.record = Some(landed));
                self.phase = Phase::Receiving;
            }
            // Nothing landed: the slot was never touched, so the transfer
            // simply failed and the lock can go.
            Ok(Some(landed)) if landed == before => {}
            _ => runtime.enter_recovery(error),
        }
        error
    }

    fn progress(&self, writer: &Writer<'_>, stage: Stage) {
        let transfer = Transfer {
            received: writer.received(),
            written: writer.written(),
            total: writer.target().image_len().get(),
        };
        self.runtime.device.state(|s| {
            s.progress.transfer = transfer;
            s.progress.stage = stage;
        });
    }

    /// Writes one bounded chunk; the flash is never held across network I/O.
    ///
    /// # Errors
    /// Returns the writer's error.
    pub async fn push(&mut self, writer: &mut Writer<'_>, bytes: &[u8]) -> Result<(), Error> {
        self.runtime
            .with_flash(|flash| writer.push(flash, bytes))
            .await?;
        self.progress(writer, Stage::Receiving);
        let tenths = Tenths::of(writer.received(), writer.target().image_len().get());
        if tenths != self.shown {
            self.shown = tenths;
            self.runtime.device.show(Notice::Receiving(tenths));
        }
        Ok(())
    }

    /// Seals, reads back and makes the image bootable, before `deadline`.
    ///
    /// # Errors
    /// Returns [`Error::Timeout`] past the deadline, or the writer's error.
    pub async fn finish(
        &self,
        writer: &mut Writer<'_>,
        deadline: Uptime,
    ) -> Result<Written, Error> {
        let runtime = self.runtime;
        runtime.with_flash(|flash| writer.seal(flash)).await?;
        self.progress(writer, Stage::Verifying);
        runtime.device.show(Notice::Verifying);
        runtime
            .step_until(|flash| {
                if runtime.device.now() >= deadline {
                    return Err(Error::Timeout);
                }
                Ok(writer.verify_step(flash)?.then_some(()))
            })
            .await?;
        if runtime.device.now() >= deadline {
            return Err(Error::Timeout);
        }
        runtime
            .with_flash(|flash| writer.commit_first(flash))
            .await?;
        self.progress(writer, Stage::Ready);
        Ok(Written(writer.target()))
    }

    /// Selects `image` for the next boot, lets `reply` answer the client,
    /// then asks for the restart, still holding the lock.
    ///
    /// On failure the transfer is cleaned up, `reply` gets the error and the
    /// error is returned with the lock released: the running image never
    /// stopped, so there is nothing to restart. If the cleanup itself fails
    /// the device stays in recovery for good, holding the lock, and this
    /// never returns.
    ///
    /// # Errors
    /// Returns the publication error once the transfer is cleaned up.
    pub async fn activate(
        mut self,
        Written(image): Written,
        reply: impl AsyncFnOnce(Result<(), Error>),
    ) -> Result<Restart, Error> {
        if let Err(error) = self.publish(image).await {
            self.abort(error).await;
            reply(Err(error)).await;
            if self.runtime.device.state(|s| s.lifecycle()) == Lifecycle::Recovery {
                // Recovery is final: the lock and the inhibit stay held.
                let _held = self;
                match core::future::pending::<core::convert::Infallible>().await {}
            }
            return Err(error);
        }
        reply(Ok(())).await;
        self.runtime.device.show(Notice::Restarting);
        self.runtime.device.pause(RESTART_DELAY).await;
        // The lock and the inhibit stay held until the restart: releasing
        // them would let the track power up under the selected image.
        self.phase = Phase::Selected;
        Ok(Restart)
    }

    async fn publish(&self, image: ImageId) -> Result<(), Error> {
        let (record, _) = self.runtime.record()?;
        self.runtime
            .apply(|flash, _| sequence::publish(flash, record, image))
            .await
            .map(|_| ())
    }

    /// Records a failed transfer and undoes what reached flash; a cleanup
    /// that fails enters recovery.
    pub async fn abort(&mut self, error: Error) {
        let runtime = self.runtime;
        runtime.device.show(Notice::Failed);
        let retry_at = runtime.device.now() + FAILURE_BACKOFF;
        let in_recovery = runtime.device.state(|s| {
            s.progress.reason = Some(error);
            s.limits.backoff_until = retry_at;
            s.lifecycle() == Lifecycle::Recovery
        });
        if self.phase != Phase::Receiving {
            return;
        }
        if in_recovery {
            // Recovery already keeps the track off for good.
            self.phase = Phase::Admitted;
            return;
        }
        let cleaned = async {
            let (record, _) = runtime.record()?;
            runtime
                .apply(|flash, booted| sequence::abort(flash, record, booted, ending_after(error)))
                .await
        }
        .await;
        // Only now is the transfer over: cancelled before this point, the
        // guard still holds the lock and the restart cleans up.
        self.phase = Phase::Admitted;
        match cleaned {
            Ok(_) => runtime.device.state(|s| s.progress.stage = Stage::Idle),
            Err(cleanup_error) => runtime.enter_recovery(cleanup_error),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dcc_ota_format::image::ImageError;

    #[test]
    fn only_a_broken_connection_or_package_leaves_the_transfer_retryable() {
        for error in [
            Error::Timeout,
            Error::Network,
            Error::Package(PackageError::Malformed),
        ] {
            assert_eq!(ending_after(error), Ending::Aborted, "{error:?}");
        }
        for error in [
            Error::Image(ImageError::DigestMismatch),
            Error::Package(PackageError::BadSignature),
            Error::Corrupt,
        ] {
            assert_eq!(ending_after(error), Ending::NotBootable, "{error:?}");
        }
    }
}
