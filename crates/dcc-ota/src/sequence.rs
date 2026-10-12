//! The flash writes of each update transition, in the order that keeps
//! every power cut recoverable: whatever byte a cut hits, the journal and
//! the bootloader selection still lead the next boot to a trusted image.
//!
//! Each function runs under one lock of the flash; none of them waits on
//! the network.

use embedded_storage::nor_flash::NorFlash;

use crate::{
    StateError,
    image::ImageId,
    journal::{Ending, Record, store},
    layout::Slot,
    otadata::{self, EntryState},
    storage,
};

/// Selects `slot` in `state`, unless the bootloader already does.
fn select<F: NorFlash>(flash: &mut F, slot: Slot, state: EntryState) -> Result<(), StateError> {
    if otadata::is_selected(flash, slot, state)? {
        return Ok(());
    }
    otadata::select(flash, slot, state)
}

/// Makes a stale candidate unbootable: erases the first sector of a
/// candidate that is neither running nor the current image, unless it is
/// already blank, so a normal boot does not wear the flash.
///
/// # Errors
/// Returns a [`FlashError`](crate::FlashError) when the read or erase fails.
pub(crate) fn erase_stale_candidate<F: NorFlash>(
    flash: &mut F,
    record: Record,
    booted: Slot,
) -> Result<(), crate::FlashError> {
    let Some(candidate) = record
        .candidate()
        .filter(|c| c.slot() != booted && *c != record.current())
    else {
        return Ok(());
    };
    let base = candidate.slot().offset();
    let mut magic = [0; 4];
    storage::read(flash, base, &mut magic)?;
    if magic == [storage::ERASED; 4] {
        return Ok(());
    }
    storage::erase_sector(flash, base)
}

/// First boot of a device: journals the running image as confirmed, then
/// selects it.
///
/// # Errors
/// Returns the journal's or the selection's error.
pub(crate) fn install<F: NorFlash>(flash: &mut F, booted: ImageId) -> Result<Record, StateError> {
    let record = store::save(flash, Record::installed(booted))?;
    select(flash, booted.slot(), EntryState::Valid)?;
    Ok(record)
}

/// A normal boot: makes sure the bootloader keeps the current image and
/// that no stale candidate stays bootable.
///
/// # Errors
/// Returns the selection's error, or a flash error.
pub(crate) fn settle<F: NorFlash>(
    flash: &mut F,
    record: Record,
    booted: Slot,
) -> Result<(), StateError> {
    select(flash, record.current().slot(), EntryState::Valid)?;
    Ok(erase_stale_candidate(flash, record, booted)?)
}

/// The first ordinary boot after an update: saves the record that no longer
/// holds the track for the boots that follow, then settles as a normal
/// boot. Returns the saved record.
///
/// # Errors
/// Returns the journal's or the selection's error.
pub(crate) fn release<F: NorFlash>(
    flash: &mut F,
    record: Record,
    booted: Slot,
) -> Result<Record, StateError> {
    let saved = store::save(flash, record)?;
    settle(flash, saved, booted)?;
    Ok(saved)
}

/// Goes back to the current image: journal first, so a cut leaves a record
/// that already names the image to boot, then the selection, then the
/// candidate is made unbootable. Returns the saved record.
///
/// # Errors
/// Returns the journal's or the selection's error.
pub(crate) fn restore<F: NorFlash>(
    flash: &mut F,
    record: Record,
    booted: Slot,
) -> Result<Record, StateError> {
    let saved = store::save(flash, record.keep_track_off())?;
    settle(flash, saved, booted)?;
    Ok(saved)
}

/// Starts a transfer of `target`: the journal records it first, then
/// every bootloader entry that still points at the target slot is erased,
/// before any byte of the new image is written there. Returns the saved
/// record.
///
/// # Errors
/// Returns [`StateError::WrongPhase`] when the journal refuses a new
/// transfer, or the journal's or the selection's error.
pub(crate) fn begin<F: NorFlash>(
    flash: &mut F,
    record: Record,
    target: ImageId,
) -> Result<Record, StateError> {
    let saved = store::save(flash, record.begin(target)?)?;
    otadata::remove_slot(flash, target.slot())?;
    Ok(saved)
}

/// Confirms the candidate on trial in `booted`: the journal first, then the
/// bootloader selection becomes valid. Returns the saved record.
///
/// A cut after the journal leaves a confirmed record whose next boot
/// repairs the selection.
///
/// # Errors
/// Returns [`StateError::Corrupt`] unless a trial of the image in `booted`
/// is running, or the journal's or the selection's error.
pub(crate) fn confirm<F: NorFlash>(
    flash: &mut F,
    record: Record,
    booted: Slot,
) -> Result<Record, StateError> {
    let saved = store::save(flash, record.confirm(booted)?)?;
    select(flash, booted, EntryState::Valid)?;
    Ok(saved)
}

/// Publishes a written and verified image: journal first, then the
/// bootloader selection for one trial boot. Returns the saved record.
///
/// # Errors
/// Returns [`StateError::Corrupt`] unless `image` is the candidate being
/// received, or the journal's or the selection's error.
pub(crate) fn publish<F: NorFlash>(
    flash: &mut F,
    record: Record,
    image: ImageId,
) -> Result<Record, StateError> {
    let saved = store::save(flash, record.ready(image)?)?;
    select(flash, image.slot(), EntryState::New)?;
    Ok(saved)
}

/// Undoes a failed transfer: the selection goes back to the current image
/// first, in case publishing got that far, then the candidate is made
/// unbootable, then the journal records the outcome. Returns the saved
/// record.
///
/// # Errors
/// Returns [`StateError::WrongPhase`] outside a transfer, or the journal's
/// or the selection's error.
pub(crate) fn abort<F: NorFlash>(
    flash: &mut F,
    record: Record,
    booted: Slot,
    ending: Ending,
) -> Result<Record, StateError> {
    let aborted = record.end_transfer(ending)?;
    select(flash, record.current().slot(), EntryState::Valid)?;
    erase_stale_candidate(flash, record, booted)?;
    store::save(flash, aborted)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        journal::{BootAction, CurrentImage, tests::image},
        layout::{JOURNAL_OFFSET, SECTOR_SIZE},
        test_flash::SimFlash,
    };

    /// Flash up to the end of the journal, with a blank candidate slot.
    fn device() -> SimFlash {
        SimFlash::new(0, JOURNAL_OFFSET as usize + 2 * SECTOR_SIZE)
    }

    /// The image the bootloader starts, by the selection and the slot's
    /// first byte; then what the journal does with it.
    fn next_boot(flash: &mut SimFlash, record: &Record) -> Result<BootAction, StateError> {
        let selected = otadata::active(otadata::read(flash).unwrap()).map(|(_, e)| e.slot());
        let bootable = |slot: Slot| flash.bytes[slot.offset() as usize] == 0xE9;
        let booted = match selected {
            Some(slot) if bootable(slot) => slot,
            _ => record.current().slot(),
        };
        let image = [Some(record.current()), record.candidate()]
            .into_iter()
            .flatten()
            .find(|id| id.slot() == booted)
            .unwrap();
        let loaded = store::load(flash)?.unwrap();
        loaded.on_boot(
            image,
            CurrentImage::Intact,
            crate::journal::ResetCause::Other,
        )
    }

    /// A device running `current` from slot A with a verified candidate in
    /// slot B, published for its trial boot.
    fn published() -> (SimFlash, Record) {
        let mut flash = device();
        let current = image(Slot::A, 0);
        let candidate = image(Slot::B, 1);
        let record = install(&mut flash, current).unwrap();
        flash.bytes[Slot::B.offset() as usize..][..64]
            .copy_from_slice(&dcc_ota_format::testing::fixture("1.1.0", 4096, false)[..64]);
        let receiving = store::save(&mut flash, record.begin(candidate).unwrap()).unwrap();
        let ready = publish(&mut flash, receiving, candidate).unwrap();
        (flash, ready)
    }

    /// The published device after the candidate booted into its trial.
    fn on_trial() -> (SimFlash, Record) {
        let (mut flash, ready) = published();
        let action = ready.on_boot(
            image(Slot::B, 1),
            CurrentImage::Intact,
            crate::journal::ResetCause::Other,
        );
        let Ok(BootAction::Trial(trying)) = action else {
            panic!("{action:?}")
        };
        let trying = store::save(&mut flash, trying).unwrap();
        (flash, trying)
    }

    #[test]
    fn a_cut_while_undoing_a_publication_still_boots_a_trusted_image() {
        let (mut base, ready) = published();
        base.start_log();
        let mut uncut = base.clone();
        let aborted = abort(&mut uncut, ready, Slot::A, Ending::Aborted).unwrap();
        let cuts: std::vec::Vec<_> = uncut.coarse_cuts().collect();
        assert!(matches!(
            next_boot(&mut uncut, &aborted),
            Ok(BootAction::Restore(_) | BootAction::RunStopped(_))
        ));
        for (call, prefix) in cuts {
            let mut flash = base.clone();
            flash.cut = Some((call, prefix));
            // Whether it reports success or not, the next boot must be safe.
            let _ = abort(&mut flash, ready, Slot::A, Ending::Aborted);
            flash.cut = None;
            let action = next_boot(&mut flash, &ready);
            assert!(action.is_ok(), "call {call}, prefix {prefix}: {action:?}");
        }
    }

    #[test]
    fn a_cut_while_restoring_still_boots_a_trusted_image() {
        let (mut base, trying) = on_trial();
        base.start_log();
        let rolled_back = trying.roll_back().unwrap();
        let mut done = base.clone();
        let saved = restore(&mut done, rolled_back, Slot::B).unwrap();
        assert!(matches!(
            next_boot(&mut done, &saved),
            Ok(BootAction::RunStopped(_))
        ));
        every_cut_boots(&base, &trying, |flash| {
            let _ = restore(flash, rolled_back, Slot::B);
        });
    }

    /// Runs `step` on a copy of `base` cut at every point it writes, and
    /// checks that each cut still leads the next boot to a trusted image.
    fn every_cut_boots(
        base: &SimFlash,
        journal_before: &Record,
        mut step: impl FnMut(&mut SimFlash),
    ) {
        let mut uncut = base.clone();
        step(&mut uncut);
        let cuts: std::vec::Vec<_> = uncut.coarse_cuts().collect();
        for (call, prefix) in cuts {
            let mut flash = base.clone();
            flash.cut = Some((call, prefix));
            step(&mut flash);
            flash.cut = None;
            let action = next_boot(&mut flash, journal_before);
            assert!(action.is_ok(), "call {call}, prefix {prefix}: {action:?}");
        }
    }

    #[test]
    fn a_cut_while_starting_a_transfer_still_boots_the_current_image() {
        let mut base = device();
        let installed = install(&mut base, image(Slot::A, 0)).unwrap();
        base.start_log();
        every_cut_boots(&base, &installed, |flash| {
            let _ = begin(flash, installed, image(Slot::B, 1));
        });
    }

    #[test]
    fn a_cut_while_confirming_still_boots_a_trusted_image() {
        let (mut base, trying) = on_trial();
        base.start_log();
        every_cut_boots(&base, &trying, |flash| {
            let _ = confirm(flash, trying, Slot::B);
        });
        let mut done = base.clone();
        let confirmed = confirm(&mut done, trying, Slot::B).unwrap();
        assert!(matches!(
            next_boot(&mut done, &confirmed),
            Ok(BootAction::RunStopped(_))
        ));
    }

    #[test]
    fn a_stale_candidate_is_erased_once() {
        let (mut flash, ready) = published();
        let aborted = abort(&mut flash, ready, Slot::A, Ending::Aborted).unwrap();
        assert_eq!(flash.bytes[Slot::B.offset() as usize], 0xFF);
        flash.start_log();
        settle(&mut flash, aborted, Slot::A).unwrap();
        assert!(
            flash.erased.iter().all(|&at| at != Slot::B.offset()),
            "already blank"
        );
    }
}
