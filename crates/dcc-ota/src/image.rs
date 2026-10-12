//! Images in the application slots: the identity the journal trusts, their
//! hash read back from flash, and their length found from the headers.

use dcc_ota_format::{Digest, DigestStream, ImageLen, Version, image::Prefix, package::Manifest};
use embedded_storage::nor_flash::ReadNorFlash;

use crate::{
    Error, FlashError,
    layout::{SECTOR_LEN, SECTOR_SIZE, SLOT_SIZE, Slot},
    storage::{self, check_read_geometry},
};

/// Identity of an image in a slot: what the journal trusts after boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImageId {
    slot: Slot,
    version: Version,
    len: ImageLen,
    digest: Digest,
}

impl ImageId {
    /// Names the `len`-byte image in `slot`.
    #[must_use]
    pub const fn new(slot: Slot, version: Version, len: ImageLen, digest: Digest) -> Self {
        Self {
            slot,
            version,
            len,
            digest,
        }
    }

    /// The identity the authenticated `manifest`'s image has once written
    /// into `slot`.
    #[must_use]
    pub const fn of(manifest: &Manifest, slot: Slot) -> Self {
        Self::new(
            slot,
            manifest.fields().version(),
            manifest.fields().image_len(),
            manifest.fields().digest(),
        )
    }

    /// Slot holding the image.
    #[must_use]
    pub const fn slot(&self) -> Slot {
        self.slot
    }

    /// Release version.
    #[must_use]
    pub const fn version(&self) -> Version {
        self.version
    }

    /// Image length.
    #[must_use]
    pub const fn image_len(&self) -> ImageLen {
        self.len
    }

    /// SHA-256 of the image.
    #[must_use]
    pub const fn digest(&self) -> Digest {
        self.digest
    }
}

/// Hashes an image in flash one sector per step, so the caller can yield
/// or release the flash between steps.
#[derive(Debug, Clone)]
pub(crate) struct Hasher {
    base: u32,
    len: u32,
    done: u32,
    hash: DigestStream,
    digest: Option<Digest>,
}

impl Hasher {
    /// Starts hashing the first `len` bytes of `slot`.
    #[must_use]
    pub(crate) fn new(slot: Slot, len: ImageLen) -> Self {
        Self {
            base: slot.offset(),
            len: len.get(),
            done: 0,
            hash: DigestStream::new(),
            digest: None,
        }
    }

    /// Hashes the next sector from `bytes`, read by [`Self::step`] or held
    /// by the caller; only the image's own bytes count.
    pub(crate) fn feed(&mut self, bytes: &[u8; SECTOR_SIZE]) {
        if self.digest.is_some() {
            return;
        }
        let n = self.next_len();
        self.hash.update(&bytes[..n]);
        self.done += n as u32;
        if self.done == self.len {
            self.digest = Some(self.hash.finish());
        }
    }

    /// Whether any byte has been hashed.
    pub(crate) fn started(&self) -> bool {
        self.done != 0
    }

    /// The digest, once every byte has been hashed.
    #[must_use]
    pub(crate) fn digest(&self) -> Option<Digest> {
        self.digest
    }

    fn next_len(&self) -> usize {
        (self.len - self.done).min(SECTOR_LEN) as usize
    }

    /// Reads and hashes the next sector into `buffer`; returns the digest
    /// once the whole image is hashed, and the same digest on later calls.
    ///
    /// # Errors
    /// Returns [`FlashError::Driver`] when a read fails.
    pub(crate) fn step<F: ReadNorFlash>(
        &mut self,
        flash: &mut F,
        buffer: &mut [u8; SECTOR_SIZE],
    ) -> Result<Option<Digest>, FlashError> {
        if self.digest.is_none() {
            let n = self.next_len();
            // Reads stay word-aligned; the padding is not hashed.
            storage::read(
                flash,
                self.base + self.done,
                &mut buffer[..n.next_multiple_of(4)],
            )?;
            self.feed(buffer);
        }
        Ok(self.digest)
    }
}

/// Finds the encoded length of the image in `slot` from its headers alone.
///
/// Not an integrity check: the caller hashes the returned range.
///
/// # Errors
/// Returns [`Error::Image`] for malformed headers or an image past the
/// slot, and [`Error::Flash`] for an unsuitable flash or a failed read.
pub(crate) fn encoded_length<F: ReadNorFlash>(
    flash: &mut F,
    slot: Slot,
) -> Result<ImageLen, Error> {
    let base = slot.offset();
    check_read_geometry(flash, (base + SLOT_SIZE) as usize)?;
    let mut prefix = [0; dcc_ota_format::image::PREFIX_LEN];
    storage::read(flash, base, &mut prefix)?;
    Prefix::parse(&prefix)?.encoded_len(|at| {
        let mut word = [0; 4];
        storage::read(flash, base + at, &mut word)?;
        Ok::<_, Error>(u32::from_le_bytes(word))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_flash::SimFlash;
    use dcc_ota_format::{image::ImageError, testing::fixture};

    #[test]
    fn length_discovery_stays_inside_the_flash() {
        let image = fixture("1.2.3", 4128, true);
        let mut flash = SimFlash::new(Slot::A.offset(), SLOT_SIZE as usize);
        flash.bytes[..image.len()].copy_from_slice(&image);
        assert_eq!(
            encoded_length(&mut flash, Slot::A).map(ImageLen::get),
            Ok(4128)
        );
        assert_eq!(
            encoded_length(&mut flash, Slot::B),
            Err(Error::Flash(FlashError::Unsuitable)),
            "slot past capacity"
        );
        flash.bytes[1] = 0;
        assert_eq!(
            encoded_length(&mut flash, Slot::A),
            Err(Error::Image(ImageError::Malformed))
        );
    }

    #[test]
    fn the_hasher_repeats_its_digest() {
        let image = fixture("1.2.3", 9120, false);
        let mut flash = SimFlash::new(Slot::A.offset(), SLOT_SIZE as usize);
        flash.bytes[..image.len()].copy_from_slice(&image);
        let mut hasher = Hasher::new(Slot::A, ImageLen::of(&image).unwrap());
        let mut buffer = [0; SECTOR_SIZE];
        let digest = loop {
            if let Some(digest) = hasher.step(&mut flash, &mut buffer).unwrap() {
                break digest;
            }
        };
        assert_eq!(digest, Digest::of(&image));
        assert_eq!(hasher.step(&mut flash, &mut buffer), Ok(Some(digest)));
    }
}
