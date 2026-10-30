//! Two-sector journal storage: every save erases the older sector, writes
//! the body, verifies it and only then writes the commit word, so a power cut
//! at any byte keeps the last committed record readable.

use embedded_storage::nor_flash::NorFlash;

use super::{
    Record,
    codec::{BODY_LEN, RECORD_SIZE, decode, encode},
};
use crate::{
    FlashError, StateError,
    layout::{JOURNAL_OFFSET, SECTOR_SIZE},
    storage::{self, check_geometry, erase_sector, sector, verify, verify_erased},
};

const SECTORS: usize = 2;

/// Returns the newest committed record and its sector.
fn scan<F: NorFlash>(flash: &mut F) -> Result<Option<(usize, Record)>, StateError> {
    check_geometry(flash, JOURNAL_OFFSET as usize + SECTORS * SECTOR_SIZE)?;
    let mut latest: Option<(usize, Record)> = None;
    for index in 0..SECTORS {
        let mut bytes = [0; RECORD_SIZE];
        storage::read(flash, sector(JOURNAL_OFFSET, index), &mut bytes)?;
        let Ok(record) = decode(&bytes) else {
            continue;
        };
        if let Some((_, newest)) = latest
            && newest.generation == record.generation
            && newest != record
        {
            return Err(StateError::Corrupt);
        }
        if latest.is_none_or(|(_, newest)| record.generation > newest.generation) {
            latest = Some((index, record));
        }
    }
    if latest.is_none() && !never_committed(flash)? {
        return Err(StateError::Corrupt);
    }
    Ok(latest)
}

/// Whether the journal never held a committed record: both sectors blank,
/// or the very first save cut short. That save always goes to sector 0 and
/// writes its commit word last, so it leaves sector 1 blank, the rest of
/// sector 0 erased and the commit word not fully written. A record that was
/// ever committed has a complete commit word, and a journal that once held
/// one keeps it whatever a later save does, so anything else is corruption.
fn never_committed<F: NorFlash>(flash: &mut F) -> Result<bool, FlashError> {
    let first = sector(JOURNAL_OFFSET, 0);
    let mut commit = [0; RECORD_SIZE - BODY_LEN];
    storage::read(flash, first + BODY_LEN as u32, &mut commit)?;
    Ok(commit != [0; RECORD_SIZE - BODY_LEN]
        && erased(flash, sector(JOURNAL_OFFSET, 1), SECTOR_SIZE)?
        && erased(flash, first + RECORD_SIZE as u32, SECTOR_SIZE - RECORD_SIZE)?)
}

/// Whether `len` bytes at `offset` are erased.
fn erased<F: NorFlash>(flash: &mut F, offset: u32, len: usize) -> Result<bool, FlashError> {
    match verify_erased(flash, offset, len) {
        Ok(()) => Ok(true),
        Err(FlashError::ReadBack) => Ok(false),
        Err(error) => Err(error),
    }
}

/// Loads the newest committed record; `None` for a blank journal.
///
/// # Errors
/// Returns [`StateError::Corrupt`] for a non-blank journal without a valid
/// record, and [`StateError::Flash`] for an unsuitable flash or a failed
/// read.
pub(crate) fn load<F: NorFlash>(flash: &mut F) -> Result<Option<Record>, StateError> {
    Ok(scan(flash)?.map(|(_, record)| record))
}

/// Saves `record` as the next generation and returns it as stored.
///
/// Once the commit word is written the record may be durable even if a
/// later step fails, so the journal is read again before reporting a
/// failure: success means [`load`] returns the new record.
///
/// # Errors
/// Returns [`StateError::Flash`] when the record did not land; [`load`]
/// then still returns the previous one. Returns [`StateError::Corrupt`] for
/// a corrupt journal or an exhausted generation counter.
pub(crate) fn save<F: NorFlash>(flash: &mut F, mut record: Record) -> Result<Record, StateError> {
    let newest = scan(flash)?;
    record.generation = newest.map_or(Ok(1), |(_, r)| {
        r.generation.checked_add(1).ok_or(StateError::Corrupt)
    })?;
    let bytes = encode(&record)?;
    let base = sector(JOURNAL_OFFSET, newest.map_or(0, |(index, _)| 1 - index));
    erase_sector(flash, base)?;
    storage::write(flash, base, &bytes[..BODY_LEN])?;
    verify(flash, base, &bytes[..BODY_LEN])?;
    verify_erased(flash, base + BODY_LEN as u32, RECORD_SIZE - BODY_LEN)?;
    let committed = storage::write(flash, base + BODY_LEN as u32, &bytes[BODY_LEN..])
        .and_then(|()| verify(flash, base, &bytes));
    let Err(error) = committed else {
        return Ok(record);
    };
    // The commit word may have landed before the step that failed.
    if load(flash) == Ok(Some(record)) {
        Ok(record)
    } else {
        Err(error.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        journal::{Update, tests::image},
        layout::Slot,
        test_flash::SimFlash,
    };
    use dcc_ota_format::Version;

    fn journal() -> SimFlash {
        SimFlash::new(JOURNAL_OFFSET, SECTORS * SECTOR_SIZE)
    }

    #[test]
    fn a_cut_during_the_very_first_save_leaves_a_blank_journal() {
        let first = Record::installed(image(Slot::A, 0));
        let mut uncut = journal();
        save(&mut uncut, first).unwrap();
        for (call, prefix) in uncut.cuts() {
            let mut flash = journal();
            flash.cut = Some((call, prefix));
            let result = save(&mut flash, first);
            flash.cut = None;
            let found = load(&mut flash);
            if let Ok(saved) = result {
                assert_eq!(found, Ok(Some(saved)));
            } else {
                assert_eq!(found, Ok(None), "call {call}, prefix {prefix}");
                assert!(save(&mut flash, first).is_ok(), "the save can be repeated");
            }
        }
    }

    #[test]
    fn a_blank_journal_is_empty_and_a_dirty_one_is_corrupt() {
        let mut flash = journal();
        assert_eq!(load(&mut flash), Ok(None));
        flash.bytes[SECTORS * SECTOR_SIZE - 1] = 0;
        assert_eq!(load(&mut flash), Err(StateError::Corrupt));
    }

    #[test]
    fn saves_alternate_sectors_and_the_highest_generation_wins() {
        let mut flash = journal();
        let first = save(&mut flash, Record::installed(image(Slot::A, 0))).unwrap();
        assert_eq!(first.generation(), 1);
        let second = save(&mut flash, first.begin(image(Slot::B, 1)).unwrap()).unwrap();
        assert_eq!(second.generation(), 2);
        assert_eq!(load(&mut flash), Ok(Some(second)));
        assert_eq!(&flash.bytes[..RECORD_SIZE], &encode(&first).unwrap());
        assert_eq!(
            &flash.bytes[SECTOR_SIZE..SECTOR_SIZE + RECORD_SIZE],
            &encode(&second).unwrap()
        );
    }

    #[test]
    fn two_sectors_of_one_generation_must_hold_the_same_record() {
        let record = Record {
            generation: 3,
            ..Record::installed(image(Slot::A, 0))
        };
        let mut flash = journal();
        flash.bytes[..RECORD_SIZE].copy_from_slice(&encode(&record).unwrap());
        flash.bytes[SECTOR_SIZE..SECTOR_SIZE + RECORD_SIZE]
            .copy_from_slice(&encode(&record).unwrap());
        assert_eq!(load(&mut flash), Ok(Some(record)));

        let other = Record {
            generation: 3,
            ..Record::installed(image(Slot::A, 1))
        };
        flash.bytes[SECTOR_SIZE..SECTOR_SIZE + RECORD_SIZE]
            .copy_from_slice(&encode(&other).unwrap());
        assert_eq!(load(&mut flash), Err(StateError::Corrupt));
    }

    #[test]
    fn a_flash_ending_inside_the_second_sector_is_unsuitable() {
        let mut flash = SimFlash::new(JOURNAL_OFFSET, SECTORS * SECTOR_SIZE - 4);
        assert_eq!(
            load(&mut flash),
            Err(StateError::Flash(FlashError::Unsuitable))
        );
    }

    #[test]
    fn generation_overflow_is_refused() {
        let mut flash = journal();
        let last = Record {
            generation: u64::MAX,
            ..Record::installed(image(Slot::A, 0))
        };
        flash.bytes[..RECORD_SIZE].copy_from_slice(&encode(&last).unwrap());
        assert_eq!(save(&mut flash, last), Err(StateError::Corrupt));
    }

    #[test]
    fn a_power_cut_at_any_call_and_byte_keeps_a_committed_record() {
        let mut base = journal();
        let installed = Record {
            rejected: Some(Version::new(3, 0, 0)),
            ..Record::installed(image(Slot::A, 0))
        };
        save(&mut base, installed).unwrap();
        // The next save erases a sector that already holds a committed record.
        let old = save(&mut base, installed).unwrap();
        let next = old.begin(image(Slot::B, 1)).unwrap();
        base.start_log();
        let mut uncut = base.clone();
        let new = save(&mut uncut, next).unwrap();
        assert_eq!(new.update(), Update::Receiving(image(Slot::B, 1)));
        let mut landed_after_commit = false;
        for (call, prefix) in uncut.cuts() {
            let mut flash = base.clone();
            flash.cut = Some((call, prefix));
            let result = save(&mut flash, next);
            flash.cut = None;
            let found = load(&mut flash).unwrap().unwrap();
            // Success exactly when the new record is the one that counts.
            match result {
                Ok(saved) => {
                    assert_eq!((saved, found), (new, new), "call {call}, prefix {prefix}");
                    landed_after_commit = true;
                }
                Err(StateError::Flash(_)) => {
                    assert_eq!(found, old, "call {call}, prefix {prefix}");
                }
                Err(other) => panic!("call {call}, prefix {prefix}: {other:?}"),
            }
            assert_eq!(found.rejected(), old.rejected());
        }
        assert!(
            landed_after_commit,
            "a cut after the commit word reports success"
        );
    }
}
