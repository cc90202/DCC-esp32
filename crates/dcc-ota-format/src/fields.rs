//! Field access for fixed binary records: the image prefix, the package
//! header and the device's own records all index at constant offsets.

/// Copies the `N` bytes at `at`.
///
/// # Panics
/// If `bytes` ends before `at + N`; callers index inside records whose
/// length they have checked.
#[must_use]
pub fn array_at<const N: usize>(bytes: &[u8], at: usize) -> [u8; N] {
    let mut array = [0; N];
    array.copy_from_slice(&bytes[at..at + N]);
    array
}

/// Reads the little-endian `u16` at `at`.
///
/// # Panics
/// As [`array_at`].
#[must_use]
pub fn le_u16(bytes: &[u8], at: usize) -> u16 {
    u16::from_le_bytes(array_at(bytes, at))
}

/// Reads the little-endian `u32` at `at`.
///
/// # Panics
/// As [`array_at`].
#[must_use]
pub fn le_u32(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(array_at(bytes, at))
}

/// Writes `bytes` as lowercase hexadecimal.
pub(crate) fn write_hex(f: &mut core::fmt::Formatter<'_>, bytes: &[u8]) -> core::fmt::Result {
    bytes.iter().try_for_each(|byte| write!(f, "{byte:02x}"))
}
