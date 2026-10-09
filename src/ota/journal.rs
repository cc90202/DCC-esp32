//! Journal schema 2: all integers LE; 172 bytes, remainder of sector erased.
//! 0..8 magic, 8..10 schema, 10 phase, 11 outcome, 12 hold, 13 option bits,
//! 14..16 zero, 16..24 generation; three 44-byte identities at 24,68,112.
//! Identity: slot, zero, version[3] u16, length u32, SHA256. Absent = all zero.
//! 156..162 rejected version, 162..164 zero; option mask 0x04 (bit index 2) marks its presence.
//! 164..168 CRC32/ISO-HDLC of body; 168..172 aligned zero commit, written last.
use super::{Error, ImageId, JOURNAL_OFFSET, SECTOR_SIZE, Version};
use embedded_storage::nor_flash::NorFlash;

pub const RECORD_SIZE: usize = 172;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Phase {
    Confirmed = 0,
    Receiving = 1,
    Ready = 2,
    Trying = 3,
    RolledBack = 4,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Outcome {
    None = 0,
    Ok = 1,
    Aborted = 2,
    RolledBack = 3,
    NotBootable = 4,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Record {
    pub generation: u64,
    pub phase: Phase,
    pub current: ImageId,
    pub candidate: Option<ImageId>,
    pub previous: Option<ImageId>,
    pub rejected: Option<Version>,
    pub outcome: Outcome,
    pub hold_track_off: bool,
}

impl Record {
    /// Returns the highest release rejected by a failed boot, not a transfer.
    pub fn rejected_version(self) -> Option<Version> {
        self.rejected
    }

    /// Raises the failed-boot floor without discarding an older rejection.
    pub fn reject_candidate(&mut self) {
        if let Some(candidate) = self.candidate {
            self.rejected = Some(
                self.rejected
                    .map_or(candidate.version, |floor| floor.max(candidate.version)),
            );
        }
    }

    /// Starts a transfer to the inactive slot, retaining the failed-boot floor.
    pub fn begin(mut self, target: ImageId) -> Result<Self, Error> {
        self.current.validate()?;
        target.validate()?;
        if !matches!(self.phase, Phase::Confirmed | Phase::RolledBack)
            || target.slot == self.current.slot
        {
            return Err(Error::Forbidden);
        }
        self.phase = Phase::Receiving;
        self.candidate = Some(target);
        self.previous = None;
        self.outcome = Outcome::None;
        self.hold_track_off = true;
        Ok(self)
    }

    /// Ends an unactivated transfer; this never rejects a release.
    pub fn abort(mut self, outcome: Outcome) -> Result<Self, Error> {
        self.current.validate()?;
        let candidate = self.candidate.ok_or(Error::Corrupt)?.validate()?;
        if !matches!(self.phase, Phase::Receiving | Phase::Ready)
            || candidate.slot == self.current.slot
            || self.previous.is_some()
            || !matches!(outcome, Outcome::Aborted | Outcome::NotBootable)
        {
            return Err(Error::Forbidden);
        }
        self.phase = Phase::Confirmed;
        self.outcome = outcome;
        self.hold_track_off = true;
        Ok(self)
    }
}

fn crc(bytes: &[u8]) -> u32 {
    let mut c = !0u32;
    for &b in bytes {
        c ^= b as u32;
        for _ in 0..8 {
            c = (c >> 1) ^ (0xedb88320 & 0u32.wrapping_sub(c & 1));
        }
    }
    !c
}
fn put_image(b: &mut [u8], id: ImageId) -> Result<(), Error> {
    id.validate()?;
    b[0] = id.slot;
    for i in 0..3 {
        b[2 + i * 2..4 + i * 2].copy_from_slice(&id.version.0[i].to_le_bytes());
    }
    b[8..12].copy_from_slice(&id.len.to_le_bytes());
    b[12..44].copy_from_slice(&id.digest);
    Ok(())
}
fn get_image(b: &[u8]) -> Result<ImageId, Error> {
    if b[1] != 0 {
        return Err(Error::Corrupt);
    }
    let mut v = [0; 3];
    for i in 0..3 {
        v[i] = u16::from_le_bytes(b[2 + i * 2..4 + i * 2].try_into().unwrap());
    }
    ImageId {
        slot: b[0],
        version: Version(v),
        len: u32::from_le_bytes(b[8..12].try_into().unwrap()),
        digest: b[12..44].try_into().unwrap(),
    }
    .validate()
}
pub fn encode(r: Record) -> Result<[u8; RECORD_SIZE], Error> {
    if r.generation == 0 {
        return Err(Error::Corrupt);
    }
    let mut b = [0; RECORD_SIZE];
    b[..8].copy_from_slice(b"DCC-JNL2");
    b[8..10].copy_from_slice(&2u16.to_le_bytes());
    b[10] = r.phase as u8;
    b[11] = r.outcome as u8;
    b[12] = r.hold_track_off as u8;
    b[13] = r.candidate.is_some() as u8
        | ((r.previous.is_some() as u8) << 1)
        | ((r.rejected.is_some() as u8) << 2);
    b[16..24].copy_from_slice(&r.generation.to_le_bytes());
    put_image(&mut b[24..68], r.current)?;
    if let Some(id) = r.candidate {
        put_image(&mut b[68..112], id)?;
    }
    if let Some(id) = r.previous {
        put_image(&mut b[112..156], id)?;
    }
    if let Some(version) = r.rejected {
        for (i, value) in version.0.iter().enumerate() {
            b[156 + i * 2..158 + i * 2].copy_from_slice(&value.to_le_bytes());
        }
    }
    let c = crc(&b[..164]);
    b[164..168].copy_from_slice(&c.to_le_bytes());
    Ok(b)
}

/// Checks the schema envelope and canonical absent fields before decoding values.
fn validate_canonical_record(b: &[u8; RECORD_SIZE]) -> Result<(), Error> {
    if &b[..8] != b"DCC-JNL2"
        || b[8..10] != [2, 0]
        || b[12] > 1
        || b[13] > 7
        || b[14..16] != [0, 0]
        || b[162..164] != [0; 2]
        || (b[13] & 1 == 0 && b[68..112] != [0; 44])
        || (b[13] & 2 == 0 && b[112..156] != [0; 44])
        || (b[13] & 4 == 0 && b[156..162] != [0; 6])
        || b[168..] != [0; 4]
        || crc(&b[..164]) != u32::from_le_bytes(b[164..168].try_into().unwrap())
    {
        return Err(Error::Corrupt);
    }
    Ok(())
}

pub fn decode(b: &[u8; RECORD_SIZE]) -> Result<Record, Error> {
    validate_canonical_record(b)?;
    let phase = match b[10] {
        0 => Phase::Confirmed,
        1 => Phase::Receiving,
        2 => Phase::Ready,
        3 => Phase::Trying,
        4 => Phase::RolledBack,
        _ => return Err(Error::Corrupt),
    };
    let outcome = match b[11] {
        0 => Outcome::None,
        1 => Outcome::Ok,
        2 => Outcome::Aborted,
        3 => Outcome::RolledBack,
        4 => Outcome::NotBootable,
        _ => return Err(Error::Corrupt),
    };
    let optional = |bit, start| {
        (b[13] & bit != 0)
            .then(|| get_image(&b[start..start + 44]))
            .transpose()
    };
    let generation = u64::from_le_bytes(b[16..24].try_into().unwrap());
    if generation == 0 {
        return Err(Error::Corrupt);
    }
    Ok(Record {
        generation,
        phase,
        outcome,
        current: get_image(&b[24..68])?,
        candidate: optional(1, 68)?,
        previous: optional(2, 112)?,
        rejected: (b[13] & 4 != 0).then(|| {
            Version(core::array::from_fn(|i| {
                u16::from_le_bytes([b[156 + i * 2], b[157 + i * 2]])
            }))
        }),
        hold_track_off: b[12] != 0,
    })
}
fn bounds<F: NorFlash>(f: &F) -> Result<(), Error> {
    if F::READ_SIZE == 0
        || 4 % F::READ_SIZE != 0
        || F::WRITE_SIZE == 0
        || 4 % F::WRITE_SIZE != 0
        || F::ERASE_SIZE == 0
        || !SECTOR_SIZE.is_multiple_of(F::ERASE_SIZE)
        || f.capacity() < JOURNAL_OFFSET as usize + 2 * SECTOR_SIZE
    {
        return Err(Error::Flash);
    }
    Ok(())
}
fn scan<F: NorFlash>(f: &mut F) -> Result<Option<(usize, Record)>, Error> {
    bounds(f)?;
    let mut latest: Option<(usize, Record)> = None;
    let mut erased = true;
    for sector in 0..2 {
        let base = JOURNAL_OFFSET + (sector * SECTOR_SIZE) as u32;
        let mut bytes = [0; RECORD_SIZE];
        f.read(base, &mut bytes).map_err(|_| Error::Flash)?;
        erased &= bytes.iter().all(|&x| x == 255);
        let mut chunk = [0; 128];
        for offset in (RECORD_SIZE..SECTOR_SIZE).step_by(128) {
            let len = (SECTOR_SIZE - offset).min(128);
            f.read(base + offset as u32, &mut chunk[..len])
                .map_err(|_| Error::Flash)?;
            erased &= chunk[..len].iter().all(|&x| x == 255);
        }
        if let Ok(r) = decode(&bytes) {
            if let Some((_, old)) = latest
                && old.generation == r.generation
                && old != r
            {
                return Err(Error::Corrupt);
            }
            if latest.is_none_or(|(_, old)| r.generation > old.generation) {
                latest = Some((sector, r));
            }
        }
    }
    if latest.is_none() && !erased {
        return Err(Error::Corrupt);
    }
    Ok(latest)
}
pub fn load<F: NorFlash>(flash: &mut F) -> Result<Option<Record>, Error> {
    Ok(scan(flash)?.map(|(_, r)| r))
}
pub fn save<F: NorFlash>(flash: &mut F, mut record: Record) -> Result<Record, Error> {
    let old = scan(flash)?;
    record.generation = match old {
        Some((_, r)) => r.generation.checked_add(1).ok_or(Error::Corrupt)?,
        None => 1,
    };
    let bytes = encode(record)?;
    let sector = old.map_or(0, |(i, _)| 1 - i);
    let base = JOURNAL_OFFSET + (sector * SECTOR_SIZE) as u32;
    flash
        .erase(base, base + SECTOR_SIZE as u32)
        .map_err(|_| Error::Flash)?;
    flash.write(base, &bytes[..168]).map_err(|_| Error::Flash)?;
    let mut check = [0; RECORD_SIZE];
    flash.read(base, &mut check).map_err(|_| Error::Flash)?;
    if check[..168] != bytes[..168] || check[168..] != [255; 4] {
        return Err(Error::Flash);
    }
    flash.write(base + 168, &[0; 4]).map_err(|_| Error::Flash)?;
    flash.read(base, &mut check).map_err(|_| Error::Flash)?;
    if check != bytes {
        return Err(Error::Flash);
    }
    Ok(record)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind, ReadNorFlash};
    #[derive(Debug, Clone, Copy)]
    pub struct Cut;
    impl NorFlashError for Cut {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }
    #[derive(Clone)]
    pub struct Mock {
        pub bytes: [u8; 8192],
        pub base: u32,
        pub calls: usize,
        pub fail: Option<(usize, usize)>,
    }
    impl Mock {
        pub fn new(base: u32) -> Self {
            Self {
                bytes: [255; 8192],
                base,
                calls: 0,
                fail: None,
            }
        }
        fn step(&mut self, len: usize) -> (usize, bool) {
            let call = self.calls;
            self.calls += 1;
            match self.fail {
                Some((n, p)) if n == call => (p.min(len), true),
                _ => (len, false),
            }
        }
    }
    impl ErrorType for Mock {
        type Error = Cut;
    }
    impl ReadNorFlash for Mock {
        const READ_SIZE: usize = 4;
        fn capacity(&self) -> usize {
            self.base as usize + 8192
        }
        fn read(&mut self, offset: u32, b: &mut [u8]) -> Result<(), Cut> {
            assert_eq!(offset % 4, 0);
            assert_eq!(b.len() % 4, 0);
            let (_, fail) = self.step(0);
            if fail {
                return Err(Cut);
            }
            let i = (offset - self.base) as usize;
            b.copy_from_slice(&self.bytes[i..i + b.len()]);
            Ok(())
        }
    }
    impl NorFlash for Mock {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = 4096;
        fn erase(&mut self, from: u32, to: u32) -> Result<(), Cut> {
            assert_eq!(from % 4096, 0);
            assert_eq!(to - from, 4096);
            let (n, fail) = self.step((to - from) as usize);
            let i = (from - self.base) as usize;
            self.bytes[i..i + n].fill(255);
            if fail { Err(Cut) } else { Ok(()) }
        }
        fn write(&mut self, offset: u32, b: &[u8]) -> Result<(), Cut> {
            assert_eq!(offset % 4, 0);
            assert_eq!(b.len() % 4, 0);
            let (n, fail) = self.step(b.len());
            let i = (offset - self.base) as usize;
            for (dst, &src) in self.bytes[i..i + n].iter_mut().zip(b) {
                assert_eq!(*dst & src, src);
                *dst &= src;
            }
            if fail { Err(Cut) } else { Ok(()) }
        }
    }
    pub fn record() -> Record {
        Record {
            generation: 99,
            phase: Phase::Confirmed,
            current: ImageId {
                slot: 0,
                version: Version([1, 2, 3]),
                len: 4096,
                digest: [0x55; 32],
            },
            candidate: None,
            previous: None,
            rejected: None,
            outcome: Outcome::None,
            hold_track_off: true,
        }
    }
    #[test]
    fn canonical_and_blank() {
        let mut f = Mock::new(JOURNAL_OFFSET);
        assert_eq!(load(&mut f), Ok(None));
        f.bytes[8191] = 0;
        assert_eq!(load(&mut f), Err(Error::Corrupt));
        f.bytes.fill(255);
        let r = save(&mut f, record()).unwrap();
        assert_eq!(r.generation, 1);
        let b = encode(r).unwrap();
        assert_eq!(
            &b[..24],
            b"DCC-JNL2\x02\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00"
        );
        assert_eq!(&b[24..36], &[0, 0, 1, 0, 2, 0, 3, 0, 0, 16, 0, 0]);
        assert_eq!(&b[36..68], &[0x55; 32]);
        assert_eq!(&b[68..156], &[0; 88]);
        assert_eq!(&b[156..164], &[0; 8]);
        assert_eq!(&b[164..168], &crc(&b[..164]).to_le_bytes());
        assert_eq!(&b[168..], &[0; 4]);
        assert_eq!(decode(&b), Ok(r));
        for i in 0..RECORD_SIZE {
            let mut bad = b;
            bad[i] ^= 1;
            assert_eq!(decode(&bad), Err(Error::Corrupt));
        }
        let mut bad = r;
        bad.current.slot = 2;
        assert_eq!(encode(bad), Err(Error::Corrupt));
        bad = r;
        bad.current.len = 4095;
        assert_eq!(encode(bad), Err(Error::Corrupt));
        bad = r;
        bad.current.len = super::super::SLOT_SIZE + 1;
        assert_eq!(encode(bad), Err(Error::Corrupt));
        let mut b = encode(Record {
            generation: u64::MAX,
            ..r
        })
        .unwrap();
        f.bytes[..RECORD_SIZE].copy_from_slice(&b);
        assert_eq!(save(&mut f, r), Err(Error::Corrupt));
        b[13] = 8;
        let c = crc(&b[..164]);
        b[164..168].copy_from_slice(&c.to_le_bytes());
        assert_eq!(decode(&b), Err(Error::Corrupt));
    }
    #[test]
    fn schema_canonical_fields_and_option_identities() {
        let r = Record {
            generation: 1,
            ..record()
        };
        for phase in [
            Phase::Confirmed,
            Phase::Receiving,
            Phase::Ready,
            Phase::Trying,
            Phase::RolledBack,
        ] {
            for outcome in [
                Outcome::None,
                Outcome::Ok,
                Outcome::Aborted,
                Outcome::RolledBack,
                Outcome::NotBootable,
            ] {
                let r = Record {
                    phase,
                    outcome,
                    candidate: Some(ImageId {
                        slot: 1,
                        ..r.current
                    }),
                    previous: Some(r.current),
                    rejected: Some(Version([2, 1, 0])),
                    ..r
                };
                assert_eq!(decode(&encode(r).unwrap()), Ok(r));
            }
        }
        for (offset, value) in [
            (8, 1),
            (10, 5),
            (11, 5),
            (12, 2),
            (13, 8),
            (14, 1),
            (25, 1),
            (24, 2),
            (68, 1),
            (112, 1),
            (156, 1),
            (162, 1),
            (163, 1),
            (16, 0),
            (33, 0),
        ] {
            let mut b = encode(r).unwrap();
            b[offset] = value;
            let c = crc(&b[..164]);
            b[164..168].copy_from_slice(&c.to_le_bytes());
            assert_eq!(decode(&b), Err(Error::Corrupt), "offset {offset}");
        }
    }
    #[test]
    fn failed_transfer_remains_retryable_but_failed_boot_does_not() {
        let mut r = record();
        r.candidate = Some(ImageId {
            slot: 1,
            version: Version([2, 1, 0]),
            ..r.current
        });
        r.phase = Phase::Confirmed;
        r.outcome = Outcome::NotBootable;
        assert_eq!(
            decode(&encode(r).unwrap()).unwrap().rejected_version(),
            None
        );
        r.phase = Phase::RolledBack;
        r.reject_candidate();
        assert_eq!(
            decode(&encode(r).unwrap()).unwrap().rejected_version(),
            Some(Version([2, 1, 0]))
        );
        r.outcome = Outcome::RolledBack;
        assert_eq!(r.rejected_version(), Some(Version([2, 1, 0])));
    }
    #[test]
    fn rejected_floor_survives_replacement_abort_restart_and_later_failure() {
        let mut f = Mock::new(JOURNAL_OFFSET);
        let mut r = record();
        let candidate = ImageId {
            slot: 1,
            version: Version([3, 0, 0]),
            ..r.current
        };
        r.candidate = Some(candidate);
        r.reject_candidate();
        r.phase = Phase::RolledBack;
        r = save(&mut f, r).unwrap();
        for version in [Version([2, 0, 0]), Version([4, 0, 0])] {
            let receiving = r
                .begin(ImageId {
                    version,
                    ..candidate
                })
                .unwrap();
            save(&mut f, receiving).unwrap();
            let restarted = load(&mut f).unwrap().unwrap();
            assert_eq!(restarted.rejected_version(), Some(Version([3, 0, 0])));
            let aborted = restarted.abort(Outcome::NotBootable).unwrap();
            save(&mut f, aborted).unwrap();
            r = load(&mut f).unwrap().unwrap();
            assert_eq!(r.rejected_version(), Some(Version([3, 0, 0])));
        }
        for version in [Version([2, 0, 0]), Version([5, 0, 0])] {
            r = r
                .begin(ImageId {
                    version,
                    ..candidate
                })
                .unwrap();
            r.phase = Phase::Trying;
            r.reject_candidate();
            r.phase = Phase::RolledBack;
            save(&mut f, r).unwrap();
            r = load(&mut f).unwrap().unwrap();
            assert_eq!(r.rejected_version(), Some(version.max(Version([3, 0, 0]))));
        }
        let b = encode(r).unwrap();
        assert_ne!(b[13] & 4, 0);
        assert_eq!(&b[156..164], &[5, 0, 0, 0, 0, 0, 0, 0]);
        let zero = Record {
            rejected: Some(Version::default()),
            ..r
        };
        assert_eq!(decode(&encode(zero).unwrap()), Ok(zero));
        let mut legacy = encode(r).unwrap();
        legacy[..8].copy_from_slice(b"DCC-JNL1");
        legacy[8] = 1;
        f.bytes.fill(255);
        f.bytes[..RECORD_SIZE].copy_from_slice(&legacy);
        assert_eq!(load(&mut f), Err(Error::Corrupt));
    }
    #[test]
    fn every_call_and_torn_byte_preserves_committed_record() {
        let mut base = Mock::new(JOURNAL_OFFSET);
        let rejected = Record {
            rejected: Some(Version([3, 0, 0])),
            ..record()
        };
        save(&mut base, rejected).unwrap();
        let old = save(&mut base, rejected).unwrap(); // next erase includes an older committed record
        let next = Record {
            phase: Phase::Receiving,
            candidate: Some(ImageId {
                slot: 1,
                ..old.current
            }),
            ..old
        };
        base.calls = 0;
        let mut good = base.clone();
        let new = save(&mut good, next).unwrap();
        for call in 0..good.calls {
            // Every API operation can fail; every byte prefix can be torn on mutation.
            let sizes = if call == good.calls - 5 {
                4096
            } else if call == good.calls - 4 {
                168
            } else if call == good.calls - 2 {
                4
            } else {
                0
            };
            for prefix in 0..=sizes {
                let mut f = base.clone();
                f.fail = Some((call, prefix));
                assert_eq!(save(&mut f, next), Err(Error::Flash));
                f.fail = None;
                let found = load(&mut f).unwrap().unwrap();
                assert!(found == old || found == new, "call {call}, prefix {prefix}");
                assert_eq!(found.rejected_version(), old.rejected_version());
                assert_eq!(&f.bytes[4096..4096 + RECORD_SIZE], &encode(old).unwrap());
            }
        }
    }
}
