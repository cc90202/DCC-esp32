//! ESP-IDF application images: structural checks, length discovery and the
//! build record that names the key the image trusts next.
//!
//! The first [`PREFIX_LEN`] bytes of an image:
//!
//! | Bytes    | Field                                                  |
//! |----------|--------------------------------------------------------|
//! | 0        | magic `0xE9`                                           |
//! | 1        | segment count, 1 to 16                                 |
//! | 3        | flash size in the high nibble, `3` for 8 MB            |
//! | 12..14   | chip id, 13 (ESP32-C6)                                 |
//! | 15..17   | lowest chip revision, major × 100 + minor              |
//! | 17..19   | highest chip revision, major × 100 + minor             |
//! | 23       | 1 when a SHA-256 follows the checksum                  |
//! | 24..32   | first segment header: load address, length             |
//! | 32..36   | application descriptor magic `0xABCD5432`              |
//! | 48..80   | version, zero padded                                   |
//! | 80..112  | project name, zero padded                              |
//!
//! Segments follow one another, each an 8-byte header and its data; then
//! zero padding up to a 16-byte block whose last byte is the XOR checksum,
//! then the optional SHA-256. Every check here is unkeyed; authenticity
//! comes only from the package signature.

mod build;
pub(crate) mod header;
mod scanner;

pub use build::{BUILD_METADATA_LEN, BuildKind, BuildMetadata, BuildScanner, EmbeddedKey};
pub use header::{PREFIX_LEN, Prefix};
pub use scanner::ImageScanner;

use crate::{ImageLen, Version};

/// Why an application image is refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageError {
    /// Not a well-formed ESP-IDF image for this device, or not the image
    /// the package header describes.
    Malformed,
    /// The image does not hash to the SHA-256 it must have.
    DigestMismatch,
    /// The build record is missing, repeated or invalid, or names a signing
    /// policy the device refuses.
    BadBuild,
    /// The header excludes the revision of this chip, so the bootloader
    /// would refuse to start the image.
    WrongChipRevision,
}

impl ImageError {
    /// Returns the stable machine-readable code.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::Malformed | Self::BadBuild => "bad_image",
            Self::DigestMismatch => "digest_mismatch",
            Self::WrongChipRevision => "wrong_chip_revision",
        }
    }
}

crate::coded_error!(ImageError);

/// What a whole image declares about itself once checked.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Inspected {
    /// Version the descriptor declares.
    pub version: Version,
    /// Length of the image.
    pub len: ImageLen,
}

/// Validates a complete non-merged image: headers, segments, padding, XOR
/// checksum and the optional appended SHA-256.
///
/// # Errors
/// Returns [`ImageError::Malformed`] for any structural fault and
/// [`ImageError::DigestMismatch`] for a wrong appended SHA-256.
pub fn inspect(bytes: &[u8]) -> Result<Inspected, ImageError> {
    let len = ImageLen::of(bytes).ok_or(ImageError::Malformed)?;
    let prefix = Prefix::parse(bytes)?;
    let mut scanner = ImageScanner::new();
    scanner.feed(bytes);
    scanner.finish(len)?;
    Ok(Inspected {
        version: prefix.version(),
        len,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::fixture;

    #[test]
    fn a_valid_image_passes_and_every_structural_fault_is_caught() {
        for hash in [false, true] {
            let image = fixture("1.2.3", 4096 + if hash { 32 } else { 0 }, hash);
            assert_eq!(
                inspect(&image).map(|i| i.version),
                Ok(Version::new(1, 2, 3))
            );
            for offset in [0, 1, 3, 12, 28, 32, 48, 80, 90, 4092, 4095] {
                let mut bad = image.clone();
                bad[offset] ^= if offset == 3 { 0x10 } else { 1 };
                assert!(inspect(&bad).is_err(), "offset {offset}");
            }
            for len in 0..image.len() {
                assert!(inspect(&image[..len]).is_err());
            }
            let mut longer = image.clone();
            longer.push(0);
            assert!(inspect(&longer).is_err());
            if hash {
                let mut bad = image;
                bad[4096] ^= 1;
                assert_eq!(inspect(&bad), Err(ImageError::DigestMismatch));
            }
        }
    }
}
