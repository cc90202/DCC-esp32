//! ESP-IDF entry: seq LE, 20 erased label bytes, state LE, CRC(seq) LE.
//! CRC is reflected polynomial 0xedb88320, init 0, xorout 0xffffffff.
use super::{Error, OTADATA_OFFSET, SECTOR_SIZE};
use embedded_storage::nor_flash::NorFlash;

/// States that firmware may publish; reads retain the bootloader's raw state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum State {
    New = 0,
    Valid = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Entry {
    pub seq: u32,
    pub state: u32,
}
pub fn crc(seq: u32) -> u32 {
    let mut c = 0u32;
    for b in seq.to_le_bytes() {
        c ^= b as u32;
        for _ in 0..8 {
            c = (c >> 1) ^ (0xedb88320 & 0u32.wrapping_sub(c & 1));
        }
    }
    !c
}
fn bounds<F: NorFlash>(f: &F) -> Result<(), Error> {
    if F::READ_SIZE == 0
        || 4 % F::READ_SIZE != 0
        || F::WRITE_SIZE == 0
        || 4 % F::WRITE_SIZE != 0
        || F::ERASE_SIZE == 0
        || !SECTOR_SIZE.is_multiple_of(F::ERASE_SIZE)
        || f.capacity() < OTADATA_OFFSET as usize + 2 * SECTOR_SIZE
    {
        return Err(Error::Flash);
    }
    Ok(())
}
fn raw<F: NorFlash>(f: &mut F, i: usize) -> Result<[u8; 32], Error> {
    let mut b = [0; 32];
    f.read(OTADATA_OFFSET + (i * SECTOR_SIZE) as u32, &mut b)
        .map_err(|_| Error::Flash)?;
    Ok(b)
}
fn decode(b: [u8; 32]) -> Option<Entry> {
    let seq = u32::from_le_bytes(b[..4].try_into().unwrap());
    let state = u32::from_le_bytes(b[24..28].try_into().unwrap());
    (seq != 0
        && seq != u32::MAX
        && state != 3
        && state != 4
        && crc(seq) == u32::from_le_bytes(b[28..].try_into().unwrap()))
    .then_some(Entry { seq, state })
}
pub fn read<F: NorFlash>(flash: &mut F) -> Result<[Option<Entry>; 2], Error> {
    bounds(flash)?;
    Ok([decode(raw(flash, 0)?), decode(raw(flash, 1)?)])
}
/// Equal sequences select sector zero, matching IDF's `seq[0] >= seq[1]`.
pub fn active(entries: [Option<Entry>; 2]) -> Option<(usize, Entry)> {
    match entries {
        [Some(a), Some(b)] if b.seq > a.seq => Some((1, b)),
        [Some(a), _] => Some((0, a)),
        [None, Some(b)] => Some((1, b)),
        _ => None,
    }
}
pub fn select<F: NorFlash>(flash: &mut F, slot: u8, state: State) -> Result<(), Error> {
    if slot > 1 {
        return Err(Error::Corrupt);
    }
    let old = active(read(flash)?);
    let mut seq = old
        .map_or(0, |(_, e)| e.seq)
        .checked_add(1)
        .ok_or(Error::Corrupt)?;
    if (seq - 1) % 2 != slot as u32 {
        seq = seq.checked_add(1).ok_or(Error::Corrupt)?;
    }
    if seq == 0 || seq == u32::MAX {
        return Err(Error::Corrupt);
    }
    let i = old.map_or(0, |(i, _)| 1 - i);
    let base = OTADATA_OFFSET + (i * SECTOR_SIZE) as u32;
    let mut b = [255; 32];
    b[..4].copy_from_slice(&seq.to_le_bytes());
    b[24..28].copy_from_slice(&(state as u32).to_le_bytes());
    flash
        .erase(base, base + SECTOR_SIZE as u32)
        .map_err(|_| Error::Flash)?;
    flash.write(base, &b[..28]).map_err(|_| Error::Flash)?;
    if raw(flash, i)? != b {
        return Err(Error::Flash);
    }
    b[28..].copy_from_slice(&crc(seq).to_le_bytes());
    flash.write(base + 28, &b[28..]).map_err(|_| Error::Flash)?;
    if raw(flash, i)? != b {
        return Err(Error::Flash);
    }
    Ok(())
}
pub fn remove_slot<F: NorFlash>(flash: &mut F, slot: u8) -> Result<(), Error> {
    if slot > 1 {
        return Err(Error::Corrupt);
    }
    let selected = active(read(flash)?);
    if selected.is_some_and(|(_, e)| (e.seq - 1) % 2 == slot as u32) {
        return Err(Error::Forbidden);
    }
    for i in 0..2 {
        let b = raw(flash, i)?;
        let seq = u32::from_le_bytes(b[..4].try_into().unwrap());
        if seq != 0 && seq != u32::MAX && (seq - 1) % 2 == slot as u32 {
            let base = OTADATA_OFFSET + (i * SECTOR_SIZE) as u32;
            flash
                .erase(base, base + SECTOR_SIZE as u32)
                .map_err(|_| Error::Flash)?;
            for offset in (0..SECTOR_SIZE).step_by(32) {
                let mut check = [0; 32];
                flash
                    .read(base + offset as u32, &mut check)
                    .map_err(|_| Error::Flash)?;
                if check != [255; 32] {
                    return Err(Error::Flash);
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::super::journal::tests::Mock;
    use super::*;
    #[test]
    fn idf_vectors_selection_and_removal() {
        // esp-bootloader-esp-idf 0.4 uses crc32_le(0xffffffff, seq LE):
        // its complemented API seed is reflected init=0, final xor=ffffffff.
        assert_eq!(crc(1), 0x4743989a);
        assert_eq!(crc(2), 0x55f63774);
        assert_eq!(crc(3), 0xed4a5011);
        assert_eq!(crc(0xfffffffe), 0x99f8b879);
        let mut f = Mock::new(OTADATA_OFFSET);
        assert_eq!(read(&mut f), Ok([None, None]));
        select(&mut f, 0, State::Valid).unwrap();
        assert_eq!(
            &f.bytes[..32],
            &[
                1, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                255, 255, 255, 255, 255, 255, 2, 0, 0, 0, 0x9a, 0x98, 0x43, 0x47
            ]
        );
        select(&mut f, 1, State::New).unwrap();
        assert_eq!(
            active(read(&mut f).unwrap()),
            Some((1, Entry { seq: 2, state: 0 }))
        );
        assert_eq!(remove_slot(&mut f, 1), Err(Error::Forbidden));
        remove_slot(&mut f, 0).unwrap();
        assert_eq!(read(&mut f).unwrap()[0], None);
        select(&mut f, 0, State::Valid).unwrap();
        remove_slot(&mut f, 1).unwrap(); // erase to exclusive partition end
        for state in [3u32, 4] {
            f.bytes[24..28].copy_from_slice(&state.to_le_bytes());
            assert_eq!(read(&mut f).unwrap(), [None, None]);
        }
        f.bytes[24..28].copy_from_slice(&2u32.to_le_bytes());
        f.bytes[28] ^= 1;
        assert_eq!(read(&mut f).unwrap(), [None, None]);
        let entry = Entry { seq: 7, state: 2 };
        assert_eq!(active([Some(entry), Some(entry)]), Some((0, entry)));
        for seq in [0u32, u32::MAX] {
            f.bytes[..4].copy_from_slice(&seq.to_le_bytes());
            f.bytes[28..32].copy_from_slice(&crc(seq).to_le_bytes());
            assert_eq!(read(&mut f).unwrap(), [None, None]);
        }
        let seq = u32::MAX - 1;
        f.bytes[..4].copy_from_slice(&seq.to_le_bytes());
        f.bytes[28..32].copy_from_slice(&crc(seq).to_le_bytes());
        let before = f.bytes;
        assert_eq!(select(&mut f, 0, State::New), Err(Error::Corrupt));
        assert_eq!(f.bytes, before);
    }
    #[test]
    fn activation_power_cuts_preserve_selection_and_state() {
        let mut base = Mock::new(OTADATA_OFFSET);
        select(&mut base, 1, State::Valid).unwrap();
        select(&mut base, 0, State::Valid).unwrap();
        base.calls = 0;
        let old = active(read(&mut base).unwrap()).unwrap();
        base.calls = 0;
        let mut good = base.clone();
        select(&mut good, 1, State::New).unwrap();
        let calls = good.calls;
        let target = active(read(&mut good).unwrap()).unwrap();
        for call in 0..calls {
            let len = match call {
                2 => 4096,
                3 => 28,
                5 => 4,
                _ => 0,
            };
            for prefix in 0..=len {
                let mut f = base.clone();
                f.fail = Some((call, prefix));
                assert_eq!(select(&mut f, 1, State::New), Err(Error::Flash));
                f.fail = None;
                let selected = active(read(&mut f).unwrap()).unwrap();
                assert!(
                    selected == old || selected == target,
                    "call {call}, prefix {prefix}"
                );
                assert_eq!(&f.bytes[4096..], &base.bytes[4096..]);
            }
        }
    }
}
