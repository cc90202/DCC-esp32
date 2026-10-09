//! ESP-IDF application image validation, without allocation or HAL dependencies.
use embedded_storage::nor_flash::ReadNorFlash;
use sha2::{Digest, Sha256};

use super::{
    Error, SLOT_SIZE, Version,
    package::{Manifest, PROJECT, parse_version},
};

/// Embed exactly one 44-byte record in loaded rodata: marker, LE kind, public key.
pub const BUILD_MARKER: [u8; 8] = *b"DCCKEY01";
/// Encoded marker, build kind and public-key record size in bytes.
pub const BUILD_METADATA_LEN: usize = 44;

/// Release signing policy classification; not evidence of authenticity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum BuildKind {
    Release = 0,
    Dev = 1,
    Fault = 2,
}

/// Logical record; embed `encode()`'s bytes, not this Rust struct's memory layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BuildMetadata {
    /// Signing-policy classification; this field alone cannot prove authenticity.
    pub kind: BuildKind,
    /// Ed25519 key that the installed firmware will trust for its next update.
    pub public_key: [u8; 32],
}

impl BuildMetadata {
    /// Encodes the fixed wire record without validating the public key.
    pub const fn encode(self) -> [u8; BUILD_METADATA_LEN] {
        let mut record = [0; BUILD_METADATA_LEN];
        let kind = (self.kind as u32).to_le_bytes();
        let mut i = 0;
        while i < 8 {
            record[i] = BUILD_MARKER[i];
            i += 1;
        }
        i = 0;
        while i < 4 {
            record[8 + i] = kind[i];
            i += 1;
        }
        i = 0;
        while i < 32 {
            record[12 + i] = self.public_key[i];
            i += 1;
        }
        record
    }
}

/// Finds exactly one build record and validates its kind and non-weak public key.
/// The embedded key is untrusted until the image itself is authenticated.
///
/// # Errors
/// Returns [`Error::BadImage`] for a missing, duplicate, truncated or invalid record.
pub fn build_metadata(image: &[u8]) -> Result<(BuildKind, [u8; 32]), Error> {
    let mut found = None;
    for (offset, window) in image.windows(8).enumerate() {
        if window != BUILD_MARKER {
            continue;
        }
        if found.is_some() {
            return Err(Error::BadImage);
        }
        let record = image
            .get(offset..offset + BUILD_METADATA_LEN)
            .ok_or(Error::BadImage)?;
        let kind = match le32(&record[8..12])? {
            0 => BuildKind::Release,
            1 => BuildKind::Dev,
            2 => BuildKind::Fault,
            _ => return Err(Error::BadImage),
        };
        let key: [u8; 32] = record[12..44].try_into().map_err(|_| Error::BadImage)?;
        let verifying =
            ed25519_dalek::VerifyingKey::from_bytes(&key).map_err(|_| Error::BadImage)?;
        if verifying.is_weak() {
            return Err(Error::BadImage);
        }
        found = Some((kind, key));
    }
    found.ok_or(Error::BadImage)
}

fn le32(bytes: &[u8]) -> Result<u32, Error> {
    Ok(u32::from_le_bytes(
        bytes.try_into().map_err(|_| Error::BadImage)?,
    ))
}

fn header(bytes: &[u8]) -> Result<(u8, bool), Error> {
    let h = bytes.get(..24).ok_or(Error::BadImage)?;
    if h[0] != 0xE9
        || !(1..=16).contains(&h[1])
        || h[3] & 0xF0 != 0x30
        || h[12..14] != [13, 0]
        || h[23] > 1
    {
        return Err(Error::BadImage);
    }
    Ok((h[1], h[23] == 1))
}

fn padded_string(bytes: &[u8]) -> Result<&str, Error> {
    let end = bytes.iter().position(|b| *b == 0).ok_or(Error::BadImage)?;
    if bytes[end..].iter().any(|b| *b != 0) {
        return Err(Error::BadImage);
    }
    core::str::from_utf8(&bytes[..end]).map_err(|_| Error::BadImage)
}

fn prefix_version(bytes: &[u8]) -> Result<Version, Error> {
    header(bytes)?;
    let prefix = bytes.get(..288).ok_or(Error::BadImage)?;
    let first_len = le32(&prefix[28..32])?;
    if !(256..=SLOT_SIZE).contains(&first_len)
        || first_len % 4 != 0
        || le32(&prefix[32..36])? != 0xABCD5432
    {
        return Err(Error::BadImage);
    }
    if padded_string(&prefix[80..112])?.as_bytes() != PROJECT {
        return Err(Error::BadImage);
    }
    parse_version(padded_string(&prefix[48..80])?).map_err(|_| Error::BadImage)
}

/// Checks the first 288 bytes against the manifest's version and image bounds.
/// Does not verify the complete image, digest or signature; authenticate the
/// manifest separately before trusting its fields.
///
/// # Errors
/// Returns [`Error::BadImage`] for a malformed prefix or inconsistent metadata.
pub fn check_prefix(bytes: &[u8], manifest: &Manifest) -> Result<(), Error> {
    if !(4096..=SLOT_SIZE).contains(&manifest.image_len)
        || prefix_version(bytes)? != manifest.version
        || le32(bytes.get(28..32).ok_or(Error::BadImage)?)? > manifest.image_len - 32
    {
        return Err(Error::BadImage);
    }
    Ok(())
}

fn segment_end(offset: u32, length: u32, limit: u32) -> Result<u32, Error> {
    if !length.is_multiple_of(4) {
        return Err(Error::BadImage);
    }
    offset
        .checked_add(8)
        .and_then(|v| v.checked_add(length))
        .filter(|end| *end <= limit)
        .ok_or(Error::BadImage)
}

fn trailer_end(offset: u32, hash: bool, limit: u32) -> Result<u32, Error> {
    // espflash 4.3.0: zero padding then XOR checksum at the last byte of a 16-byte block.
    offset
        .checked_add(16)
        .map(|v| v & !15)
        .and_then(|v| v.checked_add(if hash { 32 } else { 0 }))
        .filter(|end| *end <= limit)
        .ok_or(Error::BadImage)
}

/// Validate an exact non-merged application image, including ESP XOR and optional SHA.
/// These unkeyed integrity checks do not authenticate the image or its embedded key.
///
/// # Errors
/// Returns [`Error::BadImage`] for invalid size, headers, segments, padding or
/// checksum, and [`Error::DigestMismatch`] for a mismatched optional ESP SHA-256.
pub fn inspect(bytes: &[u8]) -> Result<(Version, usize), Error> {
    if bytes.len() > SLOT_SIZE as usize || bytes.len() < 4096 {
        return Err(Error::BadImage);
    }
    let version = prefix_version(bytes)?;
    let (count, hash) = header(bytes)?;
    let mut offset = 24u32;
    let mut checksum = 0xEF;
    for _ in 0..count {
        let start = offset as usize;
        let segment = bytes.get(start..start + 8).ok_or(Error::BadImage)?;
        let end = segment_end(offset, le32(&segment[4..8])?, bytes.len() as u32)?;
        for b in &bytes[start + 8..end as usize] {
            checksum ^= b;
        }
        offset = end;
    }
    let end = trailer_end(offset, hash, bytes.len() as u32)? as usize;
    if end != bytes.len() {
        return Err(Error::BadImage);
    }
    let checksum_end = end - if hash { 32 } else { 0 };
    if bytes[offset as usize..checksum_end - 1]
        .iter()
        .any(|b| *b != 0)
        || bytes[checksum_end - 1] != checksum
    {
        return Err(Error::BadImage);
    }
    if hash && Sha256::digest(&bytes[..checksum_end])[..] != bytes[checksum_end..] {
        return Err(Error::DigestMismatch);
    }
    Ok((version, end))
}

/// Discover the encoded length with bounded, aligned header reads only.
/// This is NOT integrity validation: bootstrap must hash/check the returned image.
///
/// # Errors
/// Returns [`Error::BadImage`] for unsupported alignment, out-of-range slot bounds
/// or malformed image headers, and [`Error::Flash`] when a flash read fails.
pub fn encoded_length<F: ReadNorFlash>(
    flash: &mut F,
    base: u32,
    slot_size: u32,
) -> Result<u32, Error> {
    if !(4096..=SLOT_SIZE).contains(&slot_size)
        || F::READ_SIZE == 0
        || 4 % F::READ_SIZE != 0
        || !(base as usize).is_multiple_of(F::READ_SIZE)
    {
        return Err(Error::BadImage);
    }
    let end = base.checked_add(slot_size).ok_or(Error::BadImage)?;
    if end as u64 > flash.capacity() as u64 {
        return Err(Error::BadImage);
    }
    let mut prefix = [0; 288];
    flash.read(base, &mut prefix).map_err(|_| Error::Flash)?;
    prefix_version(&prefix)?;
    let (count, hash) = header(&prefix)?;
    let mut offset = 24;
    for _ in 0..count {
        if offset > slot_size - 8 {
            return Err(Error::BadImage);
        }
        let mut segment = [0; 8];
        flash
            .read(base + offset, &mut segment)
            .map_err(|_| Error::Flash)?;
        offset = segment_end(offset, le32(&segment[4..8])?, slot_size)?;
    }
    let len = trailer_end(offset, hash, slot_size)?;
    if len < 4096 {
        return Err(Error::BadImage);
    }
    Ok(len)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn fixture(hash: bool) -> Vec<u8> {
        let mut image = std::vec![0; 4096 + if hash { 32 } else { 0 }];
        image[0] = 0xE9;
        image[1] = 1;
        image[3] = 0x30;
        image[12] = 13;
        image[23] = hash as u8;
        image[28..32].copy_from_slice(&4060u32.to_le_bytes());
        image[32..36].copy_from_slice(&0xABCD5432u32.to_le_bytes());
        image[48..53].copy_from_slice(b"1.2.3");
        image[80..89].copy_from_slice(PROJECT);
        image[4095] = image[32..4092].iter().fold(0xEF, |a, b| a ^ b);
        if hash {
            let digest = Sha256::digest(&image[..4096]);
            image[4096..].copy_from_slice(&digest);
        }
        image
    }

    #[test]
    fn image_integrity_and_prefix() {
        for hash in [false, true] {
            let image = fixture(hash);
            assert_eq!(inspect(&image), Ok((Version([1, 2, 3]), image.len())));
            let mut manifest = Manifest {
                version: Version([1, 2, 3]),
                image_len: image.len() as u32,
                digest: [0; 32],
                key_id: [0; 8],
            };
            assert_eq!(check_prefix(&image[..288], &manifest), Ok(()));
            manifest.version = Version([1, 2, 4]);
            assert_eq!(check_prefix(&image, &manifest), Err(Error::BadImage));
            for offset in [0, 1, 3, 12, 28, 32, 48, 80, 90, 4092, 4095] {
                let mut bad = image.clone();
                bad[offset] ^= if offset == 3 { 0x10 } else { 1 };
                assert!(inspect(&bad).is_err(), "offset {offset}");
            }
            for len in 0..image.len() {
                assert!(inspect(&image[..len]).is_err());
            }
            let mut bad = image.clone();
            bad.push(0);
            assert!(inspect(&bad).is_err());
            if hash {
                let mut bad = image;
                bad[4096] ^= 1;
                assert_eq!(inspect(&bad), Err(Error::DigestMismatch));
            }
        }
    }

    #[test]
    fn metadata_is_unique_and_valid() {
        let key = ed25519_dalek::SigningKey::from_bytes(&[42; 32])
            .verifying_key()
            .to_bytes();
        let mut record = Vec::from(BUILD_MARKER);
        record.extend_from_slice(&1u32.to_le_bytes());
        record.extend_from_slice(&key);
        assert_eq!(build_metadata(&record), Ok((BuildKind::Dev, key)));
        for kind in [BuildKind::Release, BuildKind::Dev, BuildKind::Fault] {
            let encoded = BuildMetadata {
                kind,
                public_key: key,
            }
            .encode();
            assert_eq!(build_metadata(&encoded), Ok((kind, key)));
        }
        for len in 0..44 {
            assert!(build_metadata(&record[..len]).is_err());
        }
        let mut duplicate = record.clone();
        duplicate.extend_from_slice(&record);
        assert!(build_metadata(&duplicate).is_err());
        record[8] = 3;
        assert!(build_metadata(&record).is_err());
    }

    struct Flash(Vec<u8>);
    impl embedded_storage::nor_flash::ErrorType for Flash {
        type Error = embedded_storage::nor_flash::NorFlashErrorKind;
    }
    impl ReadNorFlash for Flash {
        const READ_SIZE: usize = 4;
        fn capacity(&self) -> usize {
            self.0.len()
        }
        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            assert_eq!(offset % 4, 0);
            assert_eq!(bytes.len() % 4, 0);
            let source = self
                .0
                .get(offset as usize..offset as usize + bytes.len())
                .ok_or(embedded_storage::nor_flash::NorFlashErrorKind::OutOfBounds)?;
            bytes.copy_from_slice(source);
            Ok(())
        }
    }
    #[test]
    fn bounded_length_discovery() {
        let mut flash = Flash(fixture(false));
        assert_eq!(encoded_length(&mut flash, 0, 4096), Ok(4096));
        assert!(encoded_length(&mut flash, u32::MAX - 3, 4096).is_err());
        flash.0[28..32].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(encoded_length(&mut flash, 0, 4096).is_err());
        let mut flash = Flash(fixture(true));
        assert_eq!(encoded_length(&mut flash, 0, 4128), Ok(4128));
        assert!(encoded_length(&mut flash, 0, 4096).is_err());
    }

    #[test]
    fn segment_boundaries_and_aligned_trailer() {
        // Two segments: the second ends exactly on a 16-byte boundary,
        // so the checksum needs an entire additional block.
        let mut image = fixture(false);
        image.resize(4112, 0);
        image[1] = 2;
        image[28..32].copy_from_slice(&256u32.to_le_bytes());
        image[292..296].copy_from_slice(&3800u32.to_le_bytes());
        image[4095] = 0;
        image[4111] = image[32..288]
            .iter()
            .chain(image[296..4096].iter())
            .fold(0xEF, |a, b| a ^ b);
        assert_eq!(inspect(&image), Ok((Version([1, 2, 3]), 4112)));
        let mut flash = Flash(image.clone());
        assert_eq!(encoded_length(&mut flash, 0, 4112), Ok(4112));
        for length in [1u32, 3801, u32::MAX - 3] {
            let mut bad = image.clone();
            bad[292..296].copy_from_slice(&length.to_le_bytes());
            assert!(inspect(&bad).is_err());
            assert!(encoded_length(&mut Flash(bad), 0, 4112).is_err());
        }
        for count in [0, 17, 255] {
            let mut bad = image.clone();
            bad[1] = count;
            assert!(inspect(&bad).is_err());
        }
    }
}
