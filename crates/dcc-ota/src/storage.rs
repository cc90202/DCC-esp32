//! Flash access shared by the journal, otadata, image and writer modules:
//! geometry checks, sector erases, verified reads, and the CRC their
//! records use.

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

use crate::{
    FlashError,
    layout::{SECTOR_LEN, SECTOR_SIZE},
};

/// Whether a driver's read or write unit divides the 4-byte words the
/// update code uses.
const fn divides_word(size: usize) -> bool {
    size != 0 && 4 % size == 0
}

/// Checks that `F` can do the aligned word reads the update code issues and
/// that it reaches `end`.
pub(crate) fn check_read_geometry<F: ReadNorFlash>(
    flash: &F,
    end: usize,
) -> Result<(), FlashError> {
    if !divides_word(F::READ_SIZE) || flash.capacity() < end {
        return Err(FlashError::Unsuitable);
    }
    Ok(())
}

/// As [`check_read_geometry`], plus word writes and sector erases.
pub(crate) fn check_geometry<F: NorFlash>(flash: &F, end: usize) -> Result<(), FlashError> {
    check_read_geometry(flash, end)?;
    if !divides_word(F::WRITE_SIZE)
        || F::ERASE_SIZE == 0
        || !SECTOR_SIZE.is_multiple_of(F::ERASE_SIZE)
    {
        return Err(FlashError::Unsuitable);
    }
    Ok(())
}

/// Start of sector `index` of a region starting at `base`.
pub(crate) fn sector(base: u32, index: usize) -> u32 {
    base + index as u32 * SECTOR_LEN
}

/// Erases the sector starting at `base`.
pub(crate) fn erase_sector<F: NorFlash>(flash: &mut F, base: u32) -> Result<(), FlashError> {
    flash
        .erase(base, base + SECTOR_LEN)
        .map_err(FlashError::driver)
}

/// Reads exactly `bytes.len()` bytes at `offset`.
pub(crate) fn read<F: ReadNorFlash>(
    flash: &mut F,
    offset: u32,
    bytes: &mut [u8],
) -> Result<(), FlashError> {
    flash.read(offset, bytes).map_err(FlashError::driver)
}

/// Writes `bytes` at `offset`.
pub(crate) fn write<F: NorFlash>(
    flash: &mut F,
    offset: u32,
    bytes: &[u8],
) -> Result<(), FlashError> {
    flash.write(offset, bytes).map_err(FlashError::driver)
}

/// Reads back `expected.len()` bytes at `offset` and compares them.
///
/// # Errors
/// Returns [`FlashError::ReadBack`] on any difference.
pub(crate) fn verify<F: ReadNorFlash>(
    flash: &mut F,
    offset: u32,
    expected: &[u8],
) -> Result<(), FlashError> {
    let mut chunk = [0; 64];
    for (index, part) in expected.chunks(chunk.len()).enumerate() {
        let at = offset + (index * chunk.len()) as u32;
        read(flash, at, &mut chunk[..part.len()])?;
        if chunk[..part.len()] != *part {
            return Err(FlashError::ReadBack);
        }
    }
    Ok(())
}

/// The value of every byte of an erased sector.
pub(crate) const ERASED: u8 = 0xFF;

/// Checks that `len` bytes at `offset` are erased.
///
/// # Errors
/// Returns [`FlashError::ReadBack`] if any byte was written.
pub(crate) fn verify_erased<F: ReadNorFlash>(
    flash: &mut F,
    offset: u32,
    len: usize,
) -> Result<(), FlashError> {
    const CHUNK: [u8; 64] = [ERASED; 64];
    for done in (0..len).step_by(CHUNK.len()) {
        let n = (len - done).min(CHUNK.len());
        verify(flash, offset + done as u32, &CHUNK[..n])?;
    }
    Ok(())
}

/// Reflected CRC-32 (polynomial `0xEDB88320`) continuing from `state`; the
/// caller picks the initial value and final XOR of its format.
pub(crate) fn crc32_update(mut state: u32, bytes: &[u8]) -> u32 {
    const POLYNOMIAL: u32 = 0xEDB8_8320;
    for &byte in bytes {
        state ^= u32::from(byte);
        for _ in 0..8 {
            state = (state >> 1) ^ (POLYNOMIAL & 0u32.wrapping_sub(state & 1));
        }
    }
    state
}
