//! The start of every image, header, first segment header and application
//! descriptor, and the walk over the segment headers that finds where the
//! image ends.

use super::ImageError;
use crate::{
    ImageLen, Version,
    fields::{le_u16, le_u32},
    target::{ChipRevision, ESP32C6_CHIP_ID, PROJECT},
};

/// Image header, first segment header and application descriptor.
pub const PREFIX_LEN: usize = 288;
pub(crate) const IMAGE_MAGIC: u8 = 0xE9;
/// Image header fields: magic, segment count, flash size, chip id, the
/// range of chip revisions the image runs on, and the flag of an appended
/// SHA-256.
const MAGIC_AT: usize = 0;
pub(crate) const SEGMENTS_AT: usize = 1;
pub(crate) const FLASH_SIZE_AT: usize = 3;
pub(crate) const CHIP_ID_AT: usize = 12;
pub(crate) const MIN_REVISION_AT: usize = 15;
pub(crate) const MAX_REVISION_AT: usize = 17;
pub(crate) const HASH_FLAG_AT: usize = 23;
const MAX_SEGMENTS: u8 = 16;
/// Flash-size nibble for 8 MB, in byte 3.
pub(crate) const FLASH_8MB: u8 = 0x30;
/// Byte 3 bits that hold the flash size.
const FLASH_SIZE_MASK: u8 = 0xF0;
pub(crate) const IMAGE_HEADER_LEN: u32 = 24;
pub(crate) const SEGMENT_HEADER_LEN: u32 = 8;
/// Offset of a segment's length inside its header.
pub(crate) const SEGMENT_LEN_AT: usize = 4;
/// Offset of the first segment's length in the image.
pub(crate) const FIRST_SEGMENT_LEN_AT: usize = IMAGE_HEADER_LEN as usize + SEGMENT_LEN_AT;
/// The descriptor alone fills 256 bytes of the first segment.
const MIN_FIRST_SEGMENT: u32 = 256;
pub(crate) const APP_DESC_AT: usize = (IMAGE_HEADER_LEN + SEGMENT_HEADER_LEN) as usize;
pub(crate) const APP_DESC_MAGIC: u32 = 0xABCD_5432;
pub(crate) const VERSION_FIELD: core::ops::Range<usize> = 48..80;
pub(crate) const PROJECT_FIELD: core::ops::Range<usize> = 80..112;
const CHECKSUM_ALIGN: u32 = 16;
pub(crate) const CHECKSUM_SEED: u8 = 0xEF;
pub(crate) const APPENDED_HASH_LEN: u32 = 32;

/// The fields of the 24-byte image header the update relies on.
#[derive(Debug, Clone, Copy)]
pub(super) struct Header {
    pub segments: u8,
    pub appended_hash: bool,
}

impl Header {
    /// Reads the segment count and the appended-hash flag.
    pub fn parse(header: &[u8]) -> Result<Self, ImageError> {
        if !(1..=MAX_SEGMENTS).contains(&header[SEGMENTS_AT]) || header[HASH_FLAG_AT] > 1 {
            return Err(ImageError::Malformed);
        }
        Ok(Self {
            segments: header[SEGMENTS_AT],
            appended_hash: header[HASH_FLAG_AT] == 1,
        })
    }

    pub const fn hash_len(self) -> u32 {
        if self.appended_hash {
            APPENDED_HASH_LEN
        } else {
            0
        }
    }
}

/// End of the segment whose header sits at `offset` and declares
/// `length` data bytes, within one slot.
pub(super) fn segment_end(offset: u32, length: u32) -> Result<u32, ImageError> {
    offset
        .checked_add(SEGMENT_HEADER_LEN)
        .and_then(|start| start.checked_add(length))
        .filter(|end| *end <= ImageLen::MAX.get() && length.is_multiple_of(4))
        .ok_or(ImageError::Malformed)
}

/// Image length once the segment data ends at `data_end`: espflash 4.3.0
/// pads with zeroes to the last byte of a 16-byte block, which holds the
/// XOR checksum, then appends the optional SHA-256.
pub(super) fn trailer_end(data_end: u32, header: Header) -> Result<u32, ImageError> {
    data_end
        .checked_add(CHECKSUM_ALIGN)
        .map(|v| v & !(CHECKSUM_ALIGN - 1))
        .and_then(|v| v.checked_add(header.hash_len()))
        .filter(|end| *end <= ImageLen::MAX.get())
        .ok_or(ImageError::Malformed)
}

fn padded_string(bytes: &[u8]) -> Result<&str, ImageError> {
    let end = bytes
        .iter()
        .position(|b| *b == 0)
        .ok_or(ImageError::Malformed)?;
    if bytes[end..].iter().any(|b| *b != 0) {
        return Err(ImageError::Malformed);
    }
    core::str::from_utf8(&bytes[..end]).map_err(|_| ImageError::Malformed)
}

/// The checked first [`PREFIX_LEN`] bytes of an image.
#[derive(Debug, Clone, Copy)]
pub struct Prefix {
    header: Header,
    version: Version,
    first_segment_end: u32,
    min_revision: ChipRevision,
    /// `None` when the header sets no upper bound.
    max_revision: Option<ChipRevision>,
}

impl Prefix {
    /// Checks the image header and application descriptor at the start of
    /// `bytes`.
    ///
    /// # Errors
    /// Returns [`ImageError::Malformed`] for a short or malformed prefix, or
    /// one built for another chip, flash size or project.
    pub fn parse(bytes: &[u8]) -> Result<Self, ImageError> {
        let p = bytes.get(..PREFIX_LEN).ok_or(ImageError::Malformed)?;
        let header = Header::parse(p)?;
        let first_segment_len = le_u32(p, FIRST_SEGMENT_LEN_AT);
        if p[MAGIC_AT] != IMAGE_MAGIC
            || p[FLASH_SIZE_AT] & FLASH_SIZE_MASK != FLASH_8MB
            || p[CHIP_ID_AT..CHIP_ID_AT + 2] != ESP32C6_CHIP_ID.to_le_bytes()
            || !(MIN_FIRST_SEGMENT..=ImageLen::MAX.get()).contains(&first_segment_len)
            || !first_segment_len.is_multiple_of(4)
            || le_u32(p, APP_DESC_AT) != APP_DESC_MAGIC
            || padded_string(&p[PROJECT_FIELD])?.as_bytes() != PROJECT
        {
            return Err(ImageError::Malformed);
        }
        let version =
            Version::parse(padded_string(&p[VERSION_FIELD])?).map_err(|_| ImageError::Malformed)?;
        Ok(Self {
            header,
            version,
            first_segment_end: first_segment_len + IMAGE_HEADER_LEN + SEGMENT_HEADER_LEN,
            min_revision: ChipRevision::from_full(le_u16(p, MIN_REVISION_AT)),
            // ESP-IDF v5.5 `bootloader_common_loader.c`: 0 and 0xFFFF leave
            // the maximum unset (`IS_FIELD_SET`).
            max_revision: match le_u16(p, MAX_REVISION_AT) {
                0 | u16::MAX => None,
                full => Some(ChipRevision::from_full(full)),
            },
        })
    }

    /// Version the descriptor declares.
    #[must_use]
    pub const fn version(&self) -> Version {
        self.version
    }

    /// Checks the prefix against the signed version and length.
    ///
    /// # Errors
    /// Returns [`ImageError::Malformed`] for another version or a first
    /// segment that cannot fit `len`.
    pub fn matches(&self, version: Version, len: ImageLen) -> Result<(), ImageError> {
        if self.version != version || self.first_segment_end > len.get() {
            return Err(ImageError::Malformed);
        }
        Ok(())
    }

    /// Checks that the bootloader will start the image on a chip of
    /// `revision`: it refuses one outside the range the header declares.
    ///
    /// # Errors
    /// Returns [`ImageError::WrongChipRevision`] outside that range.
    pub fn runs_on(&self, revision: ChipRevision) -> Result<(), ImageError> {
        if revision >= self.min_revision && self.max_revision.is_none_or(|max| revision <= max) {
            Ok(())
        } else {
            Err(ImageError::WrongChipRevision)
        }
    }

    /// Finds the encoded length of the image from its segment headers alone;
    /// `read_u32(at)` reads the little-endian word at image offset `at`.
    ///
    /// Not an integrity check: the caller hashes the returned range.
    ///
    /// # Errors
    /// Returns [`ImageError::Malformed`] for malformed headers or an image
    /// past one slot, or `read_u32`'s error.
    pub fn encoded_len<E: From<ImageError>>(
        &self,
        mut read_u32: impl FnMut(u32) -> Result<u32, E>,
    ) -> Result<ImageLen, E> {
        let mut offset = IMAGE_HEADER_LEN;
        for _ in 0..self.header.segments {
            if offset > ImageLen::MAX.get() - SEGMENT_HEADER_LEN {
                return Err(ImageError::Malformed.into());
            }
            offset = segment_end(offset, read_u32(offset + SEGMENT_LEN_AT as u32)?)?;
        }
        let end = trailer_end(offset, self.header)?;
        ImageLen::new(end).ok_or_else(|| ImageError::Malformed.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::fixture;

    #[test]
    fn prefix_must_declare_the_signed_version() {
        // The first segment of this fixture ends at byte 8176.
        let image = fixture("1.2.3", 8192, false);
        let prefix = Prefix::parse(&image[..PREFIX_LEN]).unwrap();
        let len = |n| ImageLen::new(n).unwrap();
        assert_eq!(prefix.matches(Version::new(1, 2, 3), len(8176)), Ok(()));
        assert_eq!(
            prefix.matches(Version::new(1, 2, 4), len(8192)),
            Err(ImageError::Malformed)
        );
        assert_eq!(
            prefix.matches(Version::new(1, 2, 3), len(8175)),
            Err(ImageError::Malformed)
        );
    }

    #[test]
    fn prefix_runs_only_on_the_declared_chip_revisions() {
        let mut image = fixture("1.2.3", 4096, false);
        image[MIN_REVISION_AT..MIN_REVISION_AT + 2].copy_from_slice(&1u16.to_le_bytes());
        image[MAX_REVISION_AT..MAX_REVISION_AT + 2].copy_from_slice(&99u16.to_le_bytes());
        let prefix = Prefix::parse(&image[..PREFIX_LEN]).unwrap();
        assert_eq!(prefix.runs_on(ChipRevision::new(0, 1)), Ok(()));
        assert_eq!(prefix.runs_on(ChipRevision::new(0, 99)), Ok(()));
        for refused in [ChipRevision::new(0, 0), ChipRevision::new(1, 0)] {
            assert_eq!(
                prefix.runs_on(refused),
                Err(ImageError::WrongChipRevision),
                "{refused}"
            );
        }
    }

    #[test]
    fn a_maximum_of_zero_or_all_ones_sets_no_upper_bound() {
        // As the bootloader reads it: such an image starts on any revision
        // from its minimum up.
        for unset in [0u16, u16::MAX] {
            let mut image = fixture("1.2.3", 4096, false);
            image[MIN_REVISION_AT..MIN_REVISION_AT + 2].copy_from_slice(&1u16.to_le_bytes());
            image[MAX_REVISION_AT..MAX_REVISION_AT + 2].copy_from_slice(&unset.to_le_bytes());
            let prefix = Prefix::parse(&image[..PREFIX_LEN]).unwrap();
            assert_eq!(prefix.runs_on(ChipRevision::new(3, 0)), Ok(()), "{unset}");
            assert_eq!(
                prefix.runs_on(ChipRevision::new(0, 0)),
                Err(ImageError::WrongChipRevision),
                "{unset}"
            );
        }
    }

    #[test]
    fn length_discovery_reads_only_headers_and_stays_in_one_slot() {
        let mut image = fixture("1.2.3", 4128, true);
        let read = |image: &[u8]| {
            let mut reads = 0;
            let len = Prefix::parse(image)?.encoded_len(|at| {
                reads += 1;
                Ok::<_, ImageError>(le_u32(image, at as usize))
            });
            len.map(|len| (len.get(), reads))
        };
        assert_eq!(read(&image), Ok((4128, 1)));
        image[FIRST_SEGMENT_LEN_AT..FIRST_SEGMENT_LEN_AT + 4]
            .copy_from_slice(&(ImageLen::MAX.get() - 4).to_le_bytes());
        assert_eq!(read(&image), Err(ImageError::Malformed));
    }

    #[test]
    fn segment_counts_and_lengths_are_bounded() {
        for length in [0u32, 3, u32::MAX - 3] {
            let mut bad = fixture("1.2.3", 4096, false);
            bad[FIRST_SEGMENT_LEN_AT..FIRST_SEGMENT_LEN_AT + 4]
                .copy_from_slice(&length.to_le_bytes());
            assert!(super::super::inspect(&bad).is_err(), "{length}");
        }
        for count in [0, 17, 255] {
            let mut bad = fixture("1.2.3", 4096, false);
            bad[1] = count;
            assert!(super::super::inspect(&bad).is_err());
        }
    }
}
