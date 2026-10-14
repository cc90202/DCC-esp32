//! The ESP-IDF `otadata` partition: two sectors, each starting with a
//! 32-byte entry; the bootloader boots the slot of the valid entry with the
//! higher sequence number.
//!
//! Entry: sequence `u32`, 20 label bytes left erased, state `u32`, CRC of
//! the sequence. Sequence `n` selects slot `(n - 1) % 2`. The CRC is the
//! reflected `0xEDB88320` with initial value 0 and final XOR `0xFFFFFFFF`.

use embedded_storage::nor_flash::NorFlash;

use dcc_ota_format::fields::le_u32;

use crate::{
    FlashError, StateError,
    layout::{OTADATA_OFFSET, SECTOR_SIZE, Slot},
    storage::{self, check_geometry, crc32_update, erase_sector, sector, verify, verify_erased},
};

const SECTORS: usize = 2;
const ENTRY_LEN: usize = 32;
const STATE_AT: usize = 24;
const CRC_AT: usize = 28;
/// `ESP_OTA_IMG_INVALID`: the bootloader never boots this entry.
const INVALID: u32 = 3;
/// `ESP_OTA_IMG_ABORTED`: the bootloader never boots this entry.
const ABORTED: u32 = 4;

/// States the firmware publishes; reads keep the raw state the bootloader
/// may have written.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub(crate) enum EntryState {
    /// Boot this slot; ESP-IDF names it `ESP_OTA_IMG_NEW`. The bootloader
    /// without rollback keeps booting it, so the journal, not this state,
    /// makes the trial happen once.
    New = 0,
    /// The slot is confirmed.
    Valid = 2,
}

/// One decoded, CRC-valid entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Entry {
    /// Selection counter, never 0 or `u32::MAX`.
    seq: u32,
    /// Raw state word, as the bootloader may have rewritten it.
    state: u32,
}

impl Entry {
    /// The slot this entry selects.
    #[must_use]
    pub(crate) fn slot(self) -> Slot {
        selected_slot(self.seq)
    }

    /// Whether the entry is in `state`.
    #[must_use]
    pub(crate) fn is(self, state: EntryState) -> bool {
        self.state == state as u32
    }
}

/// The slot sequence number `seq` (at least 1) selects.
fn selected_slot(seq: u32) -> Slot {
    Slot::ALL[((seq - 1) % 2) as usize]
}

/// CRC over the little-endian sequence number.
pub(crate) fn crc(seq: u32) -> u32 {
    !crc32_update(0, &seq.to_le_bytes())
}

fn raw<F: NorFlash>(flash: &mut F, index: usize) -> Result<[u8; ENTRY_LEN], FlashError> {
    let mut bytes = [0; ENTRY_LEN];
    storage::read(flash, sector(OTADATA_OFFSET, index), &mut bytes)?;
    Ok(bytes)
}

fn decode(bytes: [u8; ENTRY_LEN]) -> Option<Entry> {
    let seq = le_u32(&bytes, 0);
    let state = le_u32(&bytes, STATE_AT);
    (seq != 0
        && seq != u32::MAX
        && state != INVALID
        && state != ABORTED
        && crc(seq) == le_u32(&bytes, CRC_AT))
    .then_some(Entry { seq, state })
}

/// Reads both entries.
///
/// # Errors
/// Returns a [`FlashError`] for an unsuitable flash or a failed read.
pub(crate) fn read<F: NorFlash>(flash: &mut F) -> Result<[Option<Entry>; SECTORS], FlashError> {
    check_geometry(flash, OTADATA_OFFSET as usize + SECTORS * SECTOR_SIZE)?;
    Ok([decode(raw(flash, 0)?), decode(raw(flash, 1)?)])
}

/// The entry the bootloader follows, and its sector; equal sequences pick
/// sector 0, as ESP-IDF's `seq[0] >= seq[1]`.
pub(crate) fn active(entries: [Option<Entry>; SECTORS]) -> Option<(usize, Entry)> {
    match entries {
        [Some(a), Some(b)] if b.seq > a.seq => Some((1, b)),
        [Some(a), _] => Some((0, a)),
        [None, Some(b)] => Some((1, b)),
        [None, None] => None,
    }
}

/// Whether the bootloader already selects `slot` in `state`.
///
/// # Errors
/// As [`read`].
pub(crate) fn is_selected<F: NorFlash>(
    flash: &mut F,
    slot: Slot,
    state: EntryState,
) -> Result<bool, FlashError> {
    Ok(active(read(flash)?).is_some_and(|(_, e)| e.slot() == slot && e.is(state)))
}

/// Makes the bootloader boot `slot` in `state`, writing the older sector.
///
/// The CRC is written last, so a cut write leaves an entry the bootloader
/// ignores and the previous selection stands.
///
/// # Errors
/// Returns [`StateError::Corrupt`] when the sequence would overflow, and
/// [`StateError::Flash`] for a failed or mismatched write.
pub(crate) fn select<F: NorFlash>(
    flash: &mut F,
    slot: Slot,
    state: EntryState,
) -> Result<(), StateError> {
    let old = active(read(flash)?);
    let mut seq = old
        .map_or(0, |(_, e)| e.seq)
        .checked_add(1)
        .ok_or(StateError::Corrupt)?;
    if selected_slot(seq) != slot {
        seq = seq.checked_add(1).ok_or(StateError::Corrupt)?;
    }
    if seq == u32::MAX {
        return Err(StateError::Corrupt);
    }
    let base = sector(OTADATA_OFFSET, old.map_or(0, |(index, _)| 1 - index));
    let mut entry = [storage::ERASED; ENTRY_LEN];
    entry[..4].copy_from_slice(&seq.to_le_bytes());
    entry[STATE_AT..CRC_AT].copy_from_slice(&(state as u32).to_le_bytes());
    erase_sector(flash, base)?;
    storage::write(flash, base, &entry[..CRC_AT])?;
    verify(flash, base, &entry)?;
    entry[CRC_AT..].copy_from_slice(&crc(seq).to_le_bytes());
    storage::write(flash, base + CRC_AT as u32, &entry[CRC_AT..])?;
    Ok(verify(flash, base, &entry)?)
}

/// Erases every entry that selects `slot`, which must not be the selected
/// one, before that slot is rewritten.
///
/// # Errors
/// Returns [`StateError::Corrupt`] when `slot` is selected, which the
/// journal never allows for the slot it writes, and [`StateError::Flash`]
/// for a failed erase.
pub(crate) fn remove_slot<F: NorFlash>(flash: &mut F, slot: Slot) -> Result<(), StateError> {
    if active(read(flash)?).is_some_and(|(_, e)| e.slot() == slot) {
        return Err(StateError::Corrupt);
    }
    for index in 0..SECTORS {
        let seq = le_u32(&raw(flash, index)?, 0);
        if seq == 0 || seq == u32::MAX || selected_slot(seq) != slot {
            continue;
        }
        let base = sector(OTADATA_OFFSET, index);
        erase_sector(flash, base)?;
        verify_erased(flash, base, SECTOR_SIZE)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_flash::SimFlash;

    fn otadata() -> SimFlash {
        SimFlash::new(OTADATA_OFFSET, 2 * SECTOR_SIZE)
    }

    fn put(flash: &mut SimFlash, seq: u32, state: u32) {
        flash.bytes[..4].copy_from_slice(&seq.to_le_bytes());
        flash.bytes[24..28].copy_from_slice(&state.to_le_bytes());
        flash.bytes[28..32].copy_from_slice(&crc(seq).to_le_bytes());
    }

    #[test]
    fn crc_matches_esp_idf_vectors() {
        // esp-bootloader-esp-idf 0.4: crc32_le(0xffffffff, seq LE).
        assert_eq!(crc(1), 0x4743_989a);
        assert_eq!(crc(2), 0x55f6_3774);
        assert_eq!(crc(3), 0xed4a_5011);
        assert_eq!(crc(0xffff_fffe), 0x99f8_b879);
    }

    #[test]
    fn select_writes_the_bootloader_entry_and_alternates_sectors() {
        let mut flash = otadata();
        assert_eq!(read(&mut flash), Ok([None, None]));
        select(&mut flash, Slot::A, EntryState::Valid).unwrap();
        let mut expected = [0xFF; 32];
        expected[..4].copy_from_slice(&1u32.to_le_bytes());
        expected[24..28].copy_from_slice(&2u32.to_le_bytes());
        expected[28..].copy_from_slice(&[0x9a, 0x98, 0x43, 0x47]);
        assert_eq!(flash.bytes[..32], expected);
        select(&mut flash, Slot::B, EntryState::New).unwrap();
        assert_eq!(
            active(read(&mut flash).unwrap()),
            Some((1, Entry { seq: 2, state: 0 }))
        );
        assert_eq!(is_selected(&mut flash, Slot::B, EntryState::New), Ok(true));
        assert_eq!(remove_slot(&mut flash, Slot::B), Err(StateError::Corrupt));
        remove_slot(&mut flash, Slot::A).unwrap();
        assert_eq!(read(&mut flash).unwrap()[0], None);
    }

    #[test]
    fn a_selection_matches_only_its_own_slot_and_state() {
        let mut flash = otadata();
        select(&mut flash, Slot::B, EntryState::New).unwrap();
        assert_eq!(is_selected(&mut flash, Slot::B, EntryState::New), Ok(true));
        assert_eq!(
            is_selected(&mut flash, Slot::B, EntryState::Valid),
            Ok(false)
        );
        assert_eq!(is_selected(&mut flash, Slot::A, EntryState::New), Ok(false));
    }

    #[test]
    fn removing_a_slot_no_entry_selects_erases_nothing() {
        let mut flash = otadata();
        select(&mut flash, Slot::B, EntryState::Valid).unwrap();
        flash.start_log();
        assert_eq!(remove_slot(&mut flash, Slot::A), Ok(()));
        assert!(flash.erased.is_empty(), "erased {:x?}", flash.erased);
    }

    #[test]
    fn a_flash_ending_inside_the_second_sector_is_unsuitable() {
        let mut flash = SimFlash::new(OTADATA_OFFSET, 2 * SECTOR_SIZE - 4);
        assert_eq!(read(&mut flash), Err(FlashError::Unsuitable));
    }

    #[test]
    fn invalid_entries_are_ignored_and_overflow_is_refused() {
        let mut flash = otadata();
        for state in [INVALID, ABORTED] {
            put(&mut flash, 1, state);
            assert_eq!(read(&mut flash).unwrap(), [None, None]);
        }
        put(&mut flash, 1, 2);
        flash.bytes[28] ^= 1;
        assert_eq!(read(&mut flash).unwrap(), [None, None]);
        for seq in [0, u32::MAX] {
            put(&mut flash, seq, 2);
            assert_eq!(read(&mut flash).unwrap(), [None, None]);
        }
        let entry = Entry { seq: 7, state: 2 };
        assert_eq!(active([Some(entry), Some(entry)]), Some((0, entry)));
        put(&mut flash, u32::MAX - 1, 2);
        let before = flash.bytes.clone();
        assert_eq!(
            select(&mut flash, Slot::A, EntryState::New),
            Err(StateError::Corrupt)
        );
        assert_eq!(flash.bytes, before);
    }

    #[test]
    fn a_power_cut_during_select_keeps_one_of_the_two_selections() {
        let mut base = otadata();
        select(&mut base, Slot::B, EntryState::Valid).unwrap();
        select(&mut base, Slot::A, EntryState::Valid).unwrap();
        let old = active(read(&mut base).unwrap()).unwrap();
        base.start_log();
        let mut uncut = base.clone();
        select(&mut uncut, Slot::B, EntryState::New).unwrap();
        let cuts: std::vec::Vec<_> = uncut.cuts().collect();
        let new = active(read(&mut uncut).unwrap()).unwrap();
        for (call, prefix) in cuts {
            let mut flash = base.clone();
            flash.cut = Some((call, prefix));
            let result = select(&mut flash, Slot::B, EntryState::New);
            assert!(
                matches!(result, Err(StateError::Flash(_))),
                "call {call}, prefix {prefix}: {result:?}"
            );
            flash.cut = None;
            let selected = active(read(&mut flash).unwrap()).unwrap();
            assert!(
                selected == old || selected == new,
                "call {call}, prefix {prefix}"
            );
            assert_eq!(flash.bytes[SECTOR_SIZE..], base.bytes[SECTOR_SIZE..]);
        }
    }
}
