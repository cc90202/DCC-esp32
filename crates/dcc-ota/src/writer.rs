//! Streams an image into the inactive slot.
//!
//! The slot's first sector is erased before any byte is accepted and is
//! written last, only after the streamed hash and an independent read-back
//! hash both match the signed digest: until then the bootloader cannot boot
//! a partial image.

use dcc_ota_format::{
    DigestStream,
    image::{BuildKind, BuildScanner, ImageError, ImageScanner, Prefix},
    package::Manifest,
    target::ChipRevision,
};
use embedded_storage::nor_flash::NorFlash;

use crate::{
    Error, FlashError,
    image::{Hasher, ImageId},
    layout::{SECTOR_LEN, SECTOR_SIZE, SLOT_SIZE, Slot},
    storage::{self, ERASED, check_geometry, erase_sector, verify},
};

/// Why a transfer into the inactive slot failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferError {
    /// Writing or reading the slot failed.
    Flash(FlashError),
    /// The streamed or read-back image was refused.
    Image(ImageError),
    /// More bytes than the image holds, or the stream ended short.
    Length,
    /// A step was called out of order.
    OutOfOrder,
}

impl From<FlashError> for TransferError {
    fn from(error: FlashError) -> Self {
        Self::Flash(error)
    }
}

impl From<ImageError> for TransferError {
    fn from(error: ImageError) -> Self {
        Self::Image(error)
    }
}

impl From<TransferError> for Error {
    fn from(error: TransferError) -> Self {
        match error {
            TransferError::Flash(error) => Self::Flash(error),
            TransferError::Image(error) => Self::Image(error),
            // The stream did not match the length its signed header names.
            TransferError::Length => {
                Self::Package(dcc_ota_format::package::PackageError::Malformed)
            }
            TransferError::OutOfOrder => Self::Forbidden,
        }
    }
}

impl core::fmt::Display for TransferError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Flash(error) => error.fmt(f),
            Self::Image(error) => write!(f, "image refused ({error})"),
            Self::Length => f.write_str("stream length differs from the package"),
            Self::OutOfOrder => f.write_str("transfer step out of order"),
        }
    }
}

impl core::error::Error for TransferError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Flash(error) => Some(error),
            Self::Image(error) => Some(error),
            Self::Length | Self::OutOfOrder => None,
        }
    }
}

/// The two sector buffers a transfer needs, 8 KiB in all.
pub struct Buffers {
    first: [u8; SECTOR_SIZE],
    sector: [u8; SECTOR_SIZE],
}

impl Buffers {
    /// Zeroed buffers. Zero keeps a `static` of them in `.bss` instead of
    /// 8 KiB of `.data` copied from flash at boot; the writer fills them
    /// with the erased value before each image.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            first: [0; SECTOR_SIZE],
            sector: [0; SECTOR_SIZE],
        }
    }
}

impl core::fmt::Debug for Buffers {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("Buffers { .. }")
    }
}

impl Default for Buffers {
    fn default() -> Self {
        Self::new()
    }
}

/// Progress through the publication order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Receiving,
    Sealed,
    Verified,
    Published,
}

/// One transfer into the inactive slot.
pub struct Writer<'a> {
    target: ImageId,
    device: BuildKind,
    chip: ChipRevision,
    build: BuildScanner,
    layout: ImageScanner,
    buffers: &'a mut Buffers,
    received: u32,
    filled: usize,
    hash: DigestStream,
    readback: Hasher,
    step: Step,
}

impl core::fmt::Debug for Writer<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Writer")
            .field("target", &self.target)
            .field("received", &self.received)
            .field("step", &self.step)
            .finish_non_exhaustive()
    }
}

/// An offset inside the target slot, kept apart from flash addresses.
#[derive(Clone, Copy)]
struct SlotOffset(u32);

impl<'a> Writer<'a> {
    /// Starts writing the authenticated `manifest`'s image into the slot
    /// that is not `booted`, erasing its first sector before accepting any
    /// byte. `device` is the kind of the running build: the image must be
    /// one it [accepts](BuildKind::accepts). `chip` is the revision of this
    /// chip: the image must declare it, or the bootloader would refuse it.
    ///
    /// # Errors
    /// Returns a [`FlashError`] for an unsuitable flash or a failed erase.
    pub(crate) fn new<F: NorFlash>(
        manifest: Manifest,
        booted: Slot,
        device: BuildKind,
        chip: ChipRevision,
        buffers: &'a mut Buffers,
        flash: &mut F,
    ) -> Result<Self, FlashError> {
        let target = ImageId::of(&manifest, booted.other());
        check_geometry(flash, (target.slot().offset() + SLOT_SIZE) as usize)?;
        erase_sector(flash, target.slot().offset())?;
        buffers.first.fill(ERASED);
        buffers.sector.fill(ERASED);
        Ok(Self {
            target,
            device,
            chip,
            build: BuildScanner::new(),
            layout: ImageScanner::new(),
            buffers,
            received: 0,
            filled: 0,
            hash: DigestStream::new(),
            readback: Hasher::new(target.slot(), target.image_len()),
            step: Step::Receiving,
        })
    }

    /// Identity of the image being written.
    #[must_use]
    pub const fn target(&self) -> ImageId {
        self.target
    }

    /// Bytes accepted so far.
    #[must_use]
    pub const fn received(&self) -> u32 {
        self.received
    }

    /// Bytes already durable in flash; the first sector is not counted until
    /// the whole image has been verified and that sector is committed last.
    #[must_use]
    pub const fn written(&self) -> u32 {
        match self.step {
            Step::Receiving => {
                let complete = self.received - self.received % SECTOR_LEN;
                complete.saturating_sub(SECTOR_LEN)
            }
            Step::Sealed | Step::Verified => self.target.image_len().get() - SECTOR_LEN,
            Step::Published => self.target.image_len().get(),
        }
    }

    fn address(&self, offset: SlotOffset) -> u32 {
        self.target.slot().offset() + offset.0
    }

    /// Accepts the next bytes, writing each completed sector except the first.
    ///
    /// # Errors
    /// Returns [`TransferError::OutOfOrder`] after [`Self::seal`],
    /// [`TransferError::Length`] past the image length,
    /// [`TransferError::Image`] when the first sector does not match the
    /// manifest, and [`TransferError::Flash`] for a failed write.
    pub(crate) fn push<F: NorFlash>(
        &mut self,
        flash: &mut F,
        mut bytes: &[u8],
    ) -> Result<(), TransferError> {
        if self.step != Step::Receiving {
            return Err(TransferError::OutOfOrder);
        }
        if bytes.len() > (self.target.image_len().get() - self.received) as usize {
            return Err(TransferError::Length);
        }
        while !bytes.is_empty() {
            let n = bytes.len().min(SECTOR_SIZE - self.filled);
            let buffer = if self.received < SECTOR_LEN {
                &mut self.buffers.first
            } else {
                &mut self.buffers.sector
            };
            buffer[self.filled..self.filled + n].copy_from_slice(&bytes[..n]);
            self.hash.update(&bytes[..n]);
            self.build.feed(&bytes[..n]);
            self.layout.feed(&bytes[..n]);
            self.received += n as u32;
            self.filled += n;
            bytes = &bytes[n..];
            if self.filled == SECTOR_SIZE {
                if self.received == SECTOR_LEN {
                    let prefix = Prefix::parse(&self.buffers.first)?;
                    prefix.matches(self.target.version(), self.target.image_len())?;
                    prefix.runs_on(self.chip)?;
                } else {
                    self.write_sector(flash, SlotOffset(self.received - SECTOR_LEN))?;
                }
                self.filled = 0;
                self.buffers.sector.fill(ERASED);
            }
        }
        Ok(())
    }

    fn write_sector<F: NorFlash>(
        &self,
        flash: &mut F,
        offset: SlotOffset,
    ) -> Result<(), FlashError> {
        let base = self.address(offset);
        erase_sector(flash, base)?;
        storage::write(flash, base, &self.buffers.sector)
    }

    /// Ends the stream: checks the streamed hash and the build record, then
    /// writes the last sector. A failed seal ends the transfer: the caller
    /// aborts it.
    ///
    /// # Errors
    /// Returns [`TransferError::OutOfOrder`] when already sealed,
    /// [`TransferError::Length`] for an incomplete stream,
    /// [`TransferError::Image`] for a wrong hash, a malformed image, a
    /// missing build record or a build kind this device refuses, and
    /// [`TransferError::Flash`] for a failed write.
    pub(crate) fn seal<F: NorFlash>(&mut self, flash: &mut F) -> Result<(), TransferError> {
        if self.step != Step::Receiving {
            return Err(TransferError::OutOfOrder);
        }
        if self.received != self.target.image_len().get() {
            return Err(TransferError::Length);
        }
        if self.hash.finish() != self.target.digest() {
            return Err(ImageError::DigestMismatch.into());
        }
        // Only now are the bytes known to be the signed ones. The image must
        // be whole and end exactly at the signed length, or the bootloader
        // or the boot check would refuse it after the restart.
        self.layout.finish(self.target.image_len())?;
        if !self.device.accepts(self.build.finish()?.kind()) {
            return Err(ImageError::BadBuild.into());
        }
        if self.filled != 0 {
            self.write_sector(flash, SlotOffset(self.received - self.filled as u32))?;
        }
        self.step = Step::Sealed;
        Ok(())
    }

    /// Hashes the next sector read back from flash; call until it returns
    /// `true`, yielding between calls.
    ///
    /// # Errors
    /// Returns [`TransferError::OutOfOrder`] before [`Self::seal`],
    /// [`TransferError::Image`] when the read-back image differs and
    /// [`TransferError::Flash`] for a failed read.
    pub(crate) fn verify_step<F: NorFlash>(
        &mut self,
        flash: &mut F,
    ) -> Result<bool, TransferError> {
        match self.step {
            Step::Receiving => return Err(TransferError::OutOfOrder),
            Step::Verified | Step::Published => return Ok(true),
            Step::Sealed => {}
        }
        let digest = if self.readback.started() {
            self.readback.step(flash, &mut self.buffers.sector)?
        } else {
            // The first sector is not in flash yet: hash the held copy.
            self.readback.feed(&self.buffers.first);
            self.readback.digest()
        };
        let Some(digest) = digest else {
            return Ok(false);
        };
        if digest != self.target.digest() {
            return Err(ImageError::DigestMismatch.into());
        }
        self.step = Step::Verified;
        Ok(true)
    }

    /// Writes the first sector, which makes the image bootable.
    ///
    /// # Errors
    /// Returns [`TransferError::OutOfOrder`] unless verified and not yet
    /// published, and [`TransferError::Flash`] for a failed write.
    pub(crate) fn commit_first<F: NorFlash>(&mut self, flash: &mut F) -> Result<(), TransferError> {
        if self.step != Step::Verified {
            return Err(TransferError::OutOfOrder);
        }
        let base = self.address(SlotOffset(0));
        storage::write(flash, base, &self.buffers.first)?;
        verify(flash, base, &self.buffers.first)?;
        self.step = Step::Published;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{layout::FLASH_SIZE, test_flash::SimFlash};
    use dcc_ota_format::{
        Version,
        image::BuildMetadata,
        testing::{fixture, manifest as signed, reseal, signing_key},
    };

    /// Revision of the simulated chip, v0.1 like the bench boards.
    const CHIP: ChipRevision = ChipRevision::new(0, 1);

    fn manifest(image: &[u8]) -> Manifest {
        signed(Version::new(0, 2, 0), image)
    }

    /// An image of `len` bytes carrying a build record of `kind`.
    fn release_image(len: usize, kind: BuildKind) -> std::vec::Vec<u8> {
        let mut image = fixture("0.2.0", len, false);
        let key = signing_key().verifying_key();
        let record = BuildMetadata {
            kind,
            public_key: key.to_bytes(),
        }
        .encode();
        image[300..300 + record.len()].copy_from_slice(&record);
        reseal(&mut image, false);
        image
    }

    #[test]
    fn any_chunking_publishes_only_after_read_back_and_never_touches_the_running_slot() {
        for (booted, chunk, len) in [
            (Slot::A, 1, 9120),
            (Slot::A, 4093, 9120),
            (Slot::A, 4096, 9120),
            (Slot::B, 17, 9120),
            (Slot::B, 5000, 9120),
            (Slot::A, 4093, SLOT_SIZE as usize),
        ] {
            let image = release_image(len, BuildKind::Release);
            let target = booted.other().offset() as usize;
            let running = booted.offset() as usize;
            let mut flash = SimFlash::new(0, FLASH_SIZE);
            flash.bytes[running..running + SLOT_SIZE as usize].fill(42);
            flash.bytes[target..target + SECTOR_SIZE].fill(0);
            let mut buffers = Buffers::new();
            let mut writer = Writer::new(
                manifest(&image),
                booted,
                BuildKind::Release,
                CHIP,
                &mut buffers,
                &mut flash,
            )
            .unwrap();
            assert!(
                flash.bytes[target..target + SECTOR_SIZE]
                    .iter()
                    .all(|&b| b == 0xFF)
            );
            assert_eq!(
                writer.commit_first(&mut flash),
                Err(TransferError::OutOfOrder)
            );
            for bytes in image.chunks(chunk) {
                writer.push(&mut flash, bytes).unwrap();
            }
            writer.seal(&mut flash).unwrap();
            assert_eq!(flash.bytes[target], 0xFF, "first sector held back");
            while !writer.verify_step(&mut flash).unwrap() {}
            writer.commit_first(&mut flash).unwrap();
            assert_eq!(&flash.bytes[target..target + image.len()], &image[..]);
            assert!(
                flash.bytes[running..running + SLOT_SIZE as usize]
                    .iter()
                    .all(|&b| b == 42)
            );
            assert!(
                flash
                    .erased
                    .iter()
                    .all(|&o| (target..target + SLOT_SIZE as usize).contains(&(o as usize)))
            );
            assert_eq!(
                flash
                    .erased
                    .iter()
                    .filter(|&&o| o as usize == target)
                    .count(),
                1
            );
        }
    }

    #[test]
    fn a_one_sector_image_is_verified_from_the_held_first_sector() {
        let image = release_image(SECTOR_SIZE, BuildKind::Release);
        let mut flash = SimFlash::new(0, FLASH_SIZE);
        let mut buffers = Buffers::new();
        let mut writer = Writer::new(
            manifest(&image),
            Slot::A,
            BuildKind::Release,
            CHIP,
            &mut buffers,
            &mut flash,
        )
        .unwrap();
        writer.push(&mut flash, &image).unwrap();
        writer.seal(&mut flash).unwrap();
        assert_eq!(writer.verify_step(&mut flash), Ok(true));
        writer.commit_first(&mut flash).unwrap();
        let target = Slot::B.offset() as usize;
        assert_eq!(&flash.bytes[target..target + SECTOR_SIZE], &image[..]);
    }

    #[test]
    fn progress_counts_received_bytes_and_durable_bytes_apart() {
        let image = release_image(9120, BuildKind::Release);
        let mut flash = SimFlash::new(0, FLASH_SIZE);
        let mut buffers = Buffers::new();
        let mut writer = Writer::new(
            manifest(&image),
            Slot::A,
            BuildKind::Release,
            CHIP,
            &mut buffers,
            &mut flash,
        )
        .unwrap();
        let mut progress = std::vec::Vec::new();
        for chunk in image.chunks(5000) {
            writer.push(&mut flash, chunk).unwrap();
            progress.push((writer.received(), writer.written()));
        }
        writer.seal(&mut flash).unwrap();
        progress.push((writer.received(), writer.written()));
        while !writer.verify_step(&mut flash).unwrap() {}
        writer.commit_first(&mut flash).unwrap();
        progress.push((writer.received(), writer.written()));
        // The first sector counts as durable only once it is committed last.
        assert_eq!(
            progress,
            [(5000, 0), (9120, 4096), (9120, 5024), (9120, 9120)]
        );
    }

    #[test]
    fn a_read_back_mismatch_never_publishes_the_image() {
        let image = release_image(9120, BuildKind::Release);
        let mut flash = SimFlash::new(0, FLASH_SIZE);
        let mut buffers = Buffers::new();
        let mut writer = Writer::new(
            manifest(&image),
            Slot::A,
            BuildKind::Release,
            CHIP,
            &mut buffers,
            &mut flash,
        )
        .unwrap();
        writer.push(&mut flash, &image).unwrap();
        writer.seal(&mut flash).unwrap();
        flash.bytes[Slot::B.offset() as usize + 0x1021] ^= 1;
        assert_eq!(writer.verify_step(&mut flash), Ok(false));
        assert_eq!(writer.verify_step(&mut flash), Ok(false));
        assert_eq!(
            writer.verify_step(&mut flash),
            Err(ImageError::DigestMismatch.into())
        );
        assert_eq!(flash.bytes[Slot::B.offset() as usize], 0xFF);
        assert_eq!(
            writer.commit_first(&mut flash),
            Err(TransferError::OutOfOrder)
        );
    }

    #[test]
    fn a_wrong_stream_is_refused() {
        let image = release_image(9120, BuildKind::Release);
        let mut flash = SimFlash::new(0, FLASH_SIZE);
        let mut buffers = Buffers::new();
        let mut tampered = image.clone();
        tampered[5000] ^= 1;
        let mut writer = Writer::new(
            manifest(&image),
            Slot::A,
            BuildKind::Release,
            CHIP,
            &mut buffers,
            &mut flash,
        )
        .unwrap();
        writer.push(&mut flash, &tampered).unwrap();
        assert_eq!(
            writer.push(&mut flash, &[0]),
            Err(TransferError::Length),
            "past the end"
        );
        assert_eq!(
            writer.seal(&mut flash),
            Err(ImageError::DigestMismatch.into())
        );
        let mut writer = Writer::new(
            manifest(&image),
            Slot::A,
            BuildKind::Release,
            CHIP,
            &mut buffers,
            &mut flash,
        )
        .unwrap();
        let mut other_version = image;
        other_version[48..53].copy_from_slice(b"0.3.0");
        assert_eq!(
            writer.push(&mut flash, &other_version[..SECTOR_SIZE]),
            Err(ImageError::Malformed.into())
        );
    }

    #[test]
    fn a_release_device_refuses_a_bench_image() {
        let image = release_image(9120, BuildKind::Fault);
        let mut flash = SimFlash::new(0, FLASH_SIZE);
        let mut buffers = Buffers::new();
        let mut writer = Writer::new(
            manifest(&image),
            Slot::A,
            BuildKind::Release,
            CHIP,
            &mut buffers,
            &mut flash,
        )
        .unwrap();
        writer.push(&mut flash, &image).unwrap();
        assert_eq!(writer.seal(&mut flash), Err(ImageError::BadBuild.into()));
        let mut bench = Writer::new(
            manifest(&image),
            Slot::A,
            BuildKind::Fault,
            CHIP,
            &mut buffers,
            &mut flash,
        )
        .unwrap();
        bench.push(&mut flash, &image).unwrap();
        assert_eq!(bench.seal(&mut flash), Ok(()));
    }

    #[test]
    fn an_image_the_bootloader_would_refuse_on_this_chip_is_refused_at_its_first_sector() {
        let image = release_image(9120, BuildKind::Release);
        let mut flash = SimFlash::new(0, FLASH_SIZE);
        let mut buffers = Buffers::new();
        let mut writer = Writer::new(
            manifest(&image),
            Slot::A,
            BuildKind::Release,
            ChipRevision::new(1, 0),
            &mut buffers,
            &mut flash,
        )
        .unwrap();
        assert_eq!(
            writer.push(&mut flash, &image[..SECTOR_SIZE]),
            Err(ImageError::WrongChipRevision.into())
        );
        assert_eq!(flash.bytes[Slot::B.offset() as usize], 0xFF);
    }

    #[test]
    fn a_signed_image_with_a_wrong_checksum_is_never_published() {
        let mut image = release_image(9120, BuildKind::Release);
        image[9119] ^= 1;
        let mut flash = SimFlash::new(0, FLASH_SIZE);
        let mut buffers = Buffers::new();
        let mut writer = Writer::new(
            manifest(&image),
            Slot::A,
            BuildKind::Release,
            CHIP,
            &mut buffers,
            &mut flash,
        )
        .unwrap();
        writer.push(&mut flash, &image).unwrap();
        assert_eq!(writer.seal(&mut flash), Err(ImageError::Malformed.into()));
        assert_eq!(flash.bytes[Slot::B.offset() as usize], 0xFF);
    }
}
