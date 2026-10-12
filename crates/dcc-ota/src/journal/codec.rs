//! Journal schema 2, 172 bytes, little endian; the rest of the sector stays
//! erased.
//!
//! | Bytes    | Field                                                 |
//! |----------|-------------------------------------------------------|
//! | 0..8     | magic `DCC-JNL2`                                      |
//! | 8..10    | schema, 2                                             |
//! | 10       | phase                                                 |
//! | 11       | outcome                                               |
//! | 12       | hold track off, 0 or 1                                |
//! | 13       | presence bits: candidate 1, previous 2, rejected 4    |
//! | 16..24   | generation, never 0                                   |
//! | 24..68   | current image                                         |
//! | 68..112  | candidate image                                       |
//! | 112..156 | previous image                                        |
//! | 156..162 | rejected version                                      |
//! | 164..168 | CRC-32/ISO-HDLC of bytes 0..164                       |
//! | 168..172 | commit word, zero, written last                       |
//!
//! An image is slot, zero, version, length, SHA-256 (44 bytes). Absent
//! fields and every unnamed byte are zero; decoding rejects anything else,
//! including a phase, outcome and image combination no transition makes.
//! The CRC catches torn writes; it does not authenticate the record.
//!
//! | Update                    | Phase  | Outcome | Candidate | Previous |
//! |---------------------------|--------|---------|-----------|----------|
//! | none                      | 0      | 0       | -         | -        |
//! | receiving, ready, trying  | 1 to 3 | 0       | yes       | -        |
//! | ended, confirmed          | 0      | 1       | current   | yes      |
//! | ended, aborted            | 0      | 2       | yes       | -        |
//! | ended, rolled back        | 4      | 3       | yes       | -        |
//! | ended, not bootable       | 0      | 4       | yes       | -        |
//!
//! A candidate other than the current image is always in the other slot.

use dcc_ota_format::{
    Digest, ImageLen, Version,
    fields::{array_at, le_u32},
};

use super::{Ending, Record, Update};
use crate::{StateError, image::ImageId, layout::Slot, storage::crc32_update};

/// Encoded record length.
pub(super) const RECORD_SIZE: usize = 172;
/// Bytes written before the commit word.
pub(super) const BODY_LEN: usize = 168;
const MAGIC: &[u8; 8] = b"DCC-JNL2";
const SCHEMA: u16 = 2;
const SCHEMA_AT: usize = 8;
const PHASE_AT: usize = 10;
const OUTCOME_AT: usize = 11;
const HOLD_AT: usize = 12;
const PRESENCE_AT: usize = 13;
const RESERVED_AT: usize = 14;
const GENERATION_AT: usize = 16;
const IMAGE_LEN: usize = 44;
const CURRENT_AT: usize = 24;
const CANDIDATE_AT: usize = 68;
const PREVIOUS_AT: usize = 112;
const REJECTED_AT: usize = 156;
const CRC_AT: usize = 164;
const HAS_CANDIDATE: u8 = 1;
const HAS_PREVIOUS: u8 = 2;
const HAS_REJECTED: u8 = 4;

/// No update in flight, whatever the last one did.
const SETTLED: u8 = 0;
const RECEIVING: u8 = 1;
const READY: u8 = 2;
const TRYING: u8 = 3;
/// Settled too; kept apart because schema 2 names a rollback by phase as
/// well as by outcome.
const ROLLED_BACK: u8 = 4;

const NONE: u8 = 0;
const OK: u8 = 1;
const ABORTED: u8 = 2;
const REJECTED: u8 = 3;
const NOT_BOOTABLE: u8 = 4;

fn crc(bytes: &[u8]) -> u32 {
    !crc32_update(!0, bytes)
}

/// Image fields: slot, a zero byte, version, length, SHA-256.
const SLOT_AT: usize = 0;
const IMAGE_RESERVED_AT: usize = 1;
const VERSION_AT: usize = 2;
const LEN_AT: usize = 8;
const DIGEST_AT: usize = 12;

fn put_image(bytes: &mut [u8], id: ImageId) {
    bytes[SLOT_AT] = id.slot().index();
    bytes[VERSION_AT..LEN_AT].copy_from_slice(&id.version().to_le_bytes());
    bytes[LEN_AT..DIGEST_AT].copy_from_slice(&id.image_len().get().to_le_bytes());
    bytes[DIGEST_AT..IMAGE_LEN].copy_from_slice(id.digest().as_bytes());
}

fn get_image(bytes: &[u8]) -> Result<ImageId, StateError> {
    let (Some(slot), 0, Some(len)) = (
        Slot::from_index(bytes[SLOT_AT]),
        bytes[IMAGE_RESERVED_AT],
        ImageLen::new(le_u32(bytes, LEN_AT)),
    ) else {
        return Err(StateError::Corrupt);
    };
    Ok(ImageId::new(
        slot,
        Version::from_le_bytes(array_at(bytes, VERSION_AT)),
        len,
        Digest::new(array_at(bytes, DIGEST_AT)),
    ))
}

/// Phase byte, outcome byte, candidate and previous image of `update`.
const fn fields(update: Update) -> (u8, u8, Option<ImageId>, Option<ImageId>) {
    match update {
        Update::None => (SETTLED, NONE, None, None),
        Update::Receiving(c) => (RECEIVING, NONE, Some(c), None),
        Update::Ready(c) => (READY, NONE, Some(c), None),
        Update::Trying(c) => (TRYING, NONE, Some(c), None),
        Update::Ended(c, Ending::Confirmed { previous }) => (SETTLED, OK, Some(c), Some(previous)),
        Update::Ended(c, Ending::Aborted) => (SETTLED, ABORTED, Some(c), None),
        Update::Ended(c, Ending::RolledBack) => (ROLLED_BACK, REJECTED, Some(c), None),
        Update::Ended(c, Ending::NotBootable) => (SETTLED, NOT_BOOTABLE, Some(c), None),
    }
}

/// The update the stored fields describe, for a record whose current image
/// is `current`; `None` for a combination no transition makes.
fn update(
    phase: u8,
    outcome: u8,
    candidate: Option<ImageId>,
    previous: Option<ImageId>,
    current: ImageId,
) -> Option<Update> {
    let Some(c) = candidate else {
        return (phase == SETTLED && outcome == NONE && previous.is_none()).then_some(Update::None);
    };
    if let Some(previous) = previous {
        // Only a confirmation keeps the image it replaced.
        let confirmed =
            phase == SETTLED && outcome == OK && c == current && previous.slot() != current.slot();
        return confirmed.then_some(Update::Ended(c, Ending::Confirmed { previous }));
    }
    if c.slot() == current.slot() {
        return None;
    }
    Some(match (phase, outcome) {
        (RECEIVING, NONE) => Update::Receiving(c),
        (READY, NONE) => Update::Ready(c),
        (TRYING, NONE) => Update::Trying(c),
        (SETTLED, ABORTED) => Update::Ended(c, Ending::Aborted),
        (ROLLED_BACK, REJECTED) => Update::Ended(c, Ending::RolledBack),
        (SETTLED, NOT_BOOTABLE) => Update::Ended(c, Ending::NotBootable),
        _ => return None,
    })
}

/// Encodes a saved record, commit word included.
///
/// # Errors
/// Returns [`StateError::Corrupt`] for generation 0, which only an unsaved
/// record has.
pub(super) fn encode(r: &Record) -> Result<[u8; RECORD_SIZE], StateError> {
    if r.generation == 0 {
        return Err(StateError::Corrupt);
    }
    let (phase, outcome, candidate, previous) = fields(r.update);
    let mut b = [0; RECORD_SIZE];
    b[..8].copy_from_slice(MAGIC);
    b[SCHEMA_AT..PHASE_AT].copy_from_slice(&SCHEMA.to_le_bytes());
    b[PHASE_AT] = phase;
    b[OUTCOME_AT] = outcome;
    b[HOLD_AT] = u8::from(r.hold_track_off);
    b[GENERATION_AT..CURRENT_AT].copy_from_slice(&r.generation.to_le_bytes());
    put_image(&mut b[CURRENT_AT..CANDIDATE_AT], r.current);
    if let Some(id) = candidate {
        b[PRESENCE_AT] |= HAS_CANDIDATE;
        put_image(&mut b[CANDIDATE_AT..PREVIOUS_AT], id);
    }
    if let Some(id) = previous {
        b[PRESENCE_AT] |= HAS_PREVIOUS;
        put_image(&mut b[PREVIOUS_AT..REJECTED_AT], id);
    }
    if let Some(version) = r.rejected {
        b[PRESENCE_AT] |= HAS_REJECTED;
        b[REJECTED_AT..REJECTED_AT + Version::ENCODED_LEN].copy_from_slice(&version.to_le_bytes());
    }
    let crc = crc(&b[..CRC_AT]);
    b[CRC_AT..BODY_LEN].copy_from_slice(&crc.to_le_bytes());
    Ok(b)
}

/// Checks the envelope, the zero bytes and the CRC before any value.
fn check_envelope(b: &[u8; RECORD_SIZE]) -> Result<(), StateError> {
    let absent_is_zero = |bit: u8, range: core::ops::Range<usize>| {
        b[PRESENCE_AT] & bit != 0 || b[range].iter().all(|&byte| byte == 0)
    };
    if &b[..8] != MAGIC
        || b[SCHEMA_AT..PHASE_AT] != SCHEMA.to_le_bytes()
        || b[HOLD_AT] > 1
        || b[PRESENCE_AT] > HAS_CANDIDATE | HAS_PREVIOUS | HAS_REJECTED
        || b[RESERVED_AT..GENERATION_AT] != [0, 0]
        || b[REJECTED_AT + Version::ENCODED_LEN..CRC_AT] != [0, 0]
        || !absent_is_zero(HAS_CANDIDATE, CANDIDATE_AT..PREVIOUS_AT)
        || !absent_is_zero(HAS_PREVIOUS, PREVIOUS_AT..REJECTED_AT)
        || !absent_is_zero(
            HAS_REJECTED,
            REJECTED_AT..REJECTED_AT + Version::ENCODED_LEN,
        )
        || b[BODY_LEN..] != [0; 4]
        || crc(&b[..CRC_AT]) != le_u32(b, CRC_AT)
    {
        return Err(StateError::Corrupt);
    }
    Ok(())
}

/// Decodes a committed record.
///
/// # Errors
/// Returns [`StateError::Corrupt`] for anything [`encode`] cannot produce.
pub(super) fn decode(b: &[u8; RECORD_SIZE]) -> Result<Record, StateError> {
    check_envelope(b)?;
    let optional = |bit: u8, at: usize| {
        (b[PRESENCE_AT] & bit != 0)
            .then(|| get_image(&b[at..at + IMAGE_LEN]))
            .transpose()
    };
    let generation = u64::from_le_bytes(array_at(b, GENERATION_AT));
    if generation == 0 {
        return Err(StateError::Corrupt);
    }
    let current = get_image(&b[CURRENT_AT..CANDIDATE_AT])?;
    let candidate = optional(HAS_CANDIDATE, CANDIDATE_AT)?;
    let previous = optional(HAS_PREVIOUS, PREVIOUS_AT)?;
    let update = update(b[PHASE_AT], b[OUTCOME_AT], candidate, previous, current)
        .ok_or(StateError::Corrupt)?;
    Ok(Record {
        generation,
        current,
        update,
        rejected: (b[PRESENCE_AT] & HAS_REJECTED != 0)
            .then(|| Version::from_le_bytes(array_at(b, REJECTED_AT))),
        hold_track_off: b[HOLD_AT] != 0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::journal::tests::image;

    /// A record with every optional field present.
    fn full_record() -> Record {
        Record {
            generation: 7,
            current: image(Slot::B, 3),
            update: Update::Ended(
                image(Slot::B, 3),
                Ending::Confirmed {
                    previous: image(Slot::A, 2),
                },
            ),
            rejected: Some(Version::new(2, 1, 0)),
            hold_track_off: true,
        }
    }

    fn reseal(mut b: [u8; RECORD_SIZE]) -> [u8; RECORD_SIZE] {
        let crc = crc(&b[..CRC_AT]);
        b[CRC_AT..BODY_LEN].copy_from_slice(&crc.to_le_bytes());
        b
    }

    #[test]
    fn the_wire_layout_is_fixed() {
        let r = Record {
            generation: 1,
            ..Record::installed(ImageId::new(
                Slot::A,
                Version::new(1, 2, 3),
                ImageLen::new(4096).unwrap(),
                Digest::new([0x55; 32]),
            ))
        };
        let b = encode(&r).unwrap();
        assert_eq!(
            &b[..24],
            b"DCC-JNL2\x02\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00"
        );
        assert_eq!(&b[24..36], &[0, 0, 1, 0, 2, 0, 3, 0, 0, 16, 0, 0]);
        assert_eq!(&b[36..68], &[0x55; 32]);
        assert_eq!(&b[68..164], &[0; 96]);
        assert_eq!(&b[168..], &[0; 4]);
        assert_eq!(decode(&b), Ok(r));
    }

    #[test]
    fn every_update_round_trips_and_every_flipped_bit_is_caught() {
        let c = image(Slot::A, 4);
        for update in [
            Update::Receiving(c),
            Update::Ready(c),
            Update::Trying(c),
            Update::Ended(c, Ending::Aborted),
            Update::Ended(c, Ending::NotBootable),
            Update::Ended(c, Ending::RolledBack),
        ] {
            let r = Record {
                update,
                ..full_record()
            };
            assert_eq!(decode(&encode(&r).unwrap()), Ok(r), "{update:?}");
        }
        let b = encode(&full_record()).unwrap();
        assert_eq!(decode(&b), Ok(full_record()));
        for i in 0..RECORD_SIZE {
            let mut bad = b;
            bad[i] ^= 1;
            assert_eq!(decode(&bad), Err(StateError::Corrupt), "byte {i}");
        }
    }

    #[test]
    fn noncanonical_values_are_corrupt_even_with_a_valid_crc() {
        let b = encode(&full_record()).unwrap();
        for (offset, value) in [
            (8, 1),   // schema
            (10, 5),  // phase
            (10, 3),  // a trial does not keep the image it would replace
            (11, 5),  // outcome
            (11, 2),  // an abort does not keep the image it would replace
            (12, 2),  // hold flag
            (13, 8),  // unknown presence bit
            (14, 1),  // reserved
            (24, 2),  // slot
            (25, 1),  // image reserved byte
            (33, 0),  // length below one sector
            (16, 0),  // generation 0
            (68, 0),  // confirmed candidate is not the current image
            (112, 1), // previous image in the running slot
            (162, 1), // reserved
        ] {
            let mut bad = b;
            bad[offset] = value;
            assert_eq!(
                decode(&reseal(bad)),
                Err(StateError::Corrupt),
                "offset {offset}"
            );
        }
        // Absent fields must be zero, and a confirmation needs its candidate.
        let mut no_candidate = b;
        no_candidate[13] &= !HAS_CANDIDATE;
        assert_eq!(decode(&reseal(no_candidate)), Err(StateError::Corrupt));
        no_candidate[CANDIDATE_AT..PREVIOUS_AT].fill(0);
        assert_eq!(decode(&reseal(no_candidate)), Err(StateError::Corrupt));
        // An update in flight needs its candidate in the other slot.
        let trying = Record {
            update: Update::Trying(image(Slot::A, 4)),
            ..full_record()
        };
        let mut same_slot = encode(&trying).unwrap();
        same_slot[CANDIDATE_AT] = 1;
        assert_eq!(decode(&reseal(same_slot)), Err(StateError::Corrupt));
        // The older schema is not read.
        let mut legacy = b;
        legacy[..8].copy_from_slice(b"DCC-JNL1");
        legacy[8] = 1;
        assert_eq!(decode(&reseal(legacy)), Err(StateError::Corrupt));
    }
}
