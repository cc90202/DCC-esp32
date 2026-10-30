//! Start-up: identify the running image, reconcile the journal and decide
//! whether the track may be powered.

use dcc_ota_format::{
    Digest, ImageLen, Version,
    package::{KeyId, Manifest, PackageError, unsigned_header},
};
use embedded_storage::nor_flash::ReadNorFlash;

use super::{Device, Event, Restart, Runtime};
use crate::{
    Error, FlashError,
    admission::{Lifecycle, Step},
    image::{self, ImageId},
    journal::{Blame, BootAction, CurrentImage, ResetCause, store},
    layout::Slot,
    notice::Notice,
    otadata, sequence,
    status::Stage,
};

impl<D: Device> Runtime<'_, D> {
    /// Reconciles the journal before anything else touches flash or the
    /// track. `booted_slot` reads the slot the bootloader started, after
    /// checking the partition table; `cause` is why the chip restarted.
    ///
    /// A failure the journal cannot explain ends in recovery; a candidate
    /// that fails its own checks is rolled back.
    pub async fn start(
        self,
        booted_slot: impl FnOnce(&mut D::Flash) -> Result<Slot, Error>,
        cause: ResetCause,
    ) -> Option<Restart> {
        let checked = match self.boot_check(booted_slot, cause).await {
            Ok(checked) => checked,
            Err(error) => self.recover_from_failed_boot(error).await,
        };
        if checked.is_some() {
            return checked;
        }
        let (lifecycle, record, held) = self.device.state(|s| {
            if s.lifecycle() == Lifecycle::Starting {
                s.advance(Step::SettledBoot);
                s.progress.stage = Stage::Idle;
            }
            (s.lifecycle(), s.record, s.holds_track_off())
        });
        if lifecycle.is_running() && !held {
            self.device.inhibit(false);
        }
        if let Some(notice) = record.and_then(|r| Notice::after_boot(lifecycle, &r, held)) {
            self.device.show(notice);
        }
        None
    }

    async fn boot_check(
        self,
        booted_slot: impl FnOnce(&mut D::Flash) -> Result<Slot, Error>,
        cause: ResetCause,
    ) -> Result<Option<Restart>, Error> {
        let (slot, record) = self
            .with_flash(|flash| Ok::<_, Error>((booted_slot(flash)?, store::load(flash)?)))
            .await?;
        // Published before any check of the running image can fail, so a
        // candidate that fails one can still be rolled back.
        self.device.state(|s| {
            s.slot = slot;
            s.record = record;
        });
        // A started candidate is on trial before its checks run: if it hangs
        // in them, the watchdog restart rolls it back.
        if let Some(trying) = record.and_then(|r| r.start_trial(slot)) {
            self.save(trying).await?;
        }
        let booted = self.identify(slot).await?;
        let Some(record) = record else {
            self.apply(|flash, _| sequence::install(flash, booted))
                .await?;
            return Ok(None);
        };
        let current = if record.current() == booted || self.image_intact(record.current()).await? {
            CurrentImage::Intact
        } else {
            CurrentImage::Damaged
        };
        self.apply_boot_action(record.on_boot(booted, current, cause)?, slot)
            .await
    }

    /// Hashes the image running from `slot` and proves this build can do
    /// an update; returns the image's identity.
    async fn identify(self, slot: Slot) -> Result<ImageId, Error> {
        let len = self
            .with_flash(|flash| image::encoded_length(flash, slot))
            .await?;
        let booted = ImageId::new(
            slot,
            self.device.version(),
            len,
            self.hash(slot, len).await?,
        );
        self.self_test(slot).await?;
        Ok(booted)
    }

    /// Carries out what the journal decided for this boot.
    async fn apply_boot_action(
        self,
        action: BootAction,
        slot: Slot,
    ) -> Result<Option<Restart>, Error> {
        match action {
            BootAction::Run(record) => {
                self.with_flash(|flash| sequence::settle(flash, record, slot))
                    .await?;
                Ok(None)
            }
            BootAction::RunStopped(record) => {
                self.device.state(|s| s.track_held = true);
                self.apply(|flash, booted| sequence::release(flash, record, booted))
                    .await?;
                Ok(None)
            }
            // Saved by `start_trial` before the checks.
            BootAction::Trial(_) => {
                self.device.state(|s| {
                    s.advance(Step::TrialBoot);
                    s.progress.stage = Stage::Health;
                });
                Ok(None)
            }
            // `on_boot` restores only an intact current image.
            BootAction::Restore(record) => self.restore_verified(record).await,
        }
    }

    /// A candidate that fails its own boot checks goes back to the
    /// confirmed image, rejected unless the flash failed under it; anything
    /// else needs USB.
    async fn recover_from_failed_boot(self, error: Error) -> Option<Restart> {
        let (record, booted) = self.device.state(|s| (s.record, s.slot));
        let blame = match error {
            Error::Flash(FlashError::Driver(_) | FlashError::ReadBack) => Blame::Device,
            Error::Flash(FlashError::Unsuitable)
            | Error::Corrupt
            | Error::Forbidden
            | Error::Unavailable
            | Error::Internal
            | Error::Network
            | Error::Timeout
            | Error::Request(_)
            | Error::Package(_)
            | Error::Image(_)
            | Error::Ineligible(_)
            | Error::Refused(_) => Blame::Candidate,
        };
        let Some(rolled_back) = record.and_then(|r| r.on_failed_candidate_boot(booted, blame))
        else {
            self.enter_recovery(error);
            return None;
        };
        self.device.report(Event::CandidateFailedBootCheck(error));
        self.restore(rolled_back).await.unwrap_or_else(|error| {
            self.enter_recovery(error);
            None
        })
    }

    /// Proves, without writing, that this build can verify a package and
    /// reach the inactive slot and otadata: a build that cannot must not be
    /// confirmed.
    async fn self_test(self, booted: Slot) -> Result<(), Error> {
        let key = self.device.trusted_key()?;
        let fixture = unsigned_header(
            Version::new(1, 0, 0),
            ImageLen::MIN,
            Digest::default(),
            KeyId::of(&key),
        );
        // An unsigned header must fail on its signature, not earlier.
        if Manifest::authenticate(&fixture, &key) != Err(PackageError::BadSignature) {
            return Err(Error::Corrupt);
        }
        self.with_flash(|flash| {
            let mut word = [0; 4];
            flash
                .read(booted.other().offset(), &mut word)
                .map_err(FlashError::driver)?;
            otadata::read(flash).map(|_| ())
        })
        .await
    }
}
