//! Streaming NOR writer. Sector zero is retained until BOTH hashes match.
use embedded_storage::nor_flash::NorFlash;
use sha2::{Digest, Sha256};

use super::{Error, ImageId, SECTOR_SIZE, SLOT_OFFSETS, image, package::Manifest};

pub struct Buffers {
    pub first: [u8; SECTOR_SIZE],
    pub sector: [u8; SECTOR_SIZE],
}
impl Default for Buffers {
    fn default() -> Self {
        Self {
            first: [255; SECTOR_SIZE],
            sector: [255; SECTOR_SIZE],
        }
    }
}

pub struct Writer<'a> {
    target: ImageId,
    manifest: Manifest,
    buffers: &'a mut Buffers,
    received: u32,
    filled: usize,
    hash: Sha256,
    sealed: bool,
    verified: bool,
    readback_offset: u32,
    readback_hash: Sha256,
}

impl<'a> Writer<'a> {
    /// Invalidates the inactive slot's first sector before accepting any bytes.
    pub fn new<F: NorFlash>(
        manifest: Manifest,
        booted: u8,
        buffers: &'a mut Buffers,
        flash: &mut F,
    ) -> Result<Self, Error> {
        if booted > 1 {
            return Err(Error::Corrupt);
        }
        let target = ImageId {
            slot: 1 - booted,
            version: manifest.version,
            len: manifest.image_len,
            digest: manifest.digest,
        }
        .validate()?;
        let base = SLOT_OFFSETS[target.slot as usize];
        if F::READ_SIZE == 0
            || 4 % F::READ_SIZE != 0
            || F::WRITE_SIZE == 0
            || 4 % F::WRITE_SIZE != 0
            || F::ERASE_SIZE == 0
            || !SECTOR_SIZE.is_multiple_of(F::ERASE_SIZE)
            || flash.capacity() < base as usize + super::SLOT_SIZE as usize
        {
            return Err(Error::Flash);
        }
        flash
            .erase(base, base + SECTOR_SIZE as u32)
            .map_err(|_| Error::Flash)?;
        buffers.first.fill(255);
        buffers.sector.fill(255);
        Ok(Self {
            target,
            manifest,
            buffers,
            received: 0,
            filled: 0,
            hash: Sha256::new(),
            sealed: false,
            verified: false,
            readback_offset: 0,
            readback_hash: Sha256::new(),
        })
    }
    /// Returns the immutable inactive-slot identity.
    pub fn target(&self) -> ImageId {
        self.target
    }
    pub fn received(&self) -> u32 {
        self.received
    }
    fn base(&self) -> u32 {
        SLOT_OFFSETS[self.target.slot as usize]
    }

    pub fn push<F: NorFlash>(&mut self, flash: &mut F, mut bytes: &[u8]) -> Result<(), Error> {
        if self.sealed || bytes.len() > (self.target.len - self.received) as usize {
            return Err(Error::BadPackage);
        }
        while !bytes.is_empty() {
            let n = bytes.len().min(SECTOR_SIZE - self.filled);
            if self.received < SECTOR_SIZE as u32 {
                self.buffers.first[self.filled..self.filled + n].copy_from_slice(&bytes[..n]);
            } else {
                self.buffers.sector[self.filled..self.filled + n].copy_from_slice(&bytes[..n]);
            }
            self.hash.update(&bytes[..n]);
            self.received += n as u32;
            self.filled += n;
            bytes = &bytes[n..];
            if self.filled == SECTOR_SIZE {
                if self.received == SECTOR_SIZE as u32 {
                    image::check_prefix(&self.buffers.first, &self.manifest)?;
                } else {
                    self.write_sector(flash, self.received - SECTOR_SIZE as u32)?;
                }
                self.filled = 0;
                self.buffers.sector.fill(255);
            }
        }
        Ok(())
    }
    fn write_sector<F: NorFlash>(&self, flash: &mut F, offset: u32) -> Result<(), Error> {
        let base = self.base() + offset;
        flash
            .erase(base, base + SECTOR_SIZE as u32)
            .map_err(|_| Error::Flash)?;
        flash
            .write(base, &self.buffers.sector)
            .map_err(|_| Error::Flash)
    }
    pub fn seal<F: NorFlash>(&mut self, flash: &mut F) -> Result<(), Error> {
        if self.sealed || self.received != self.target.len {
            return Err(Error::BadPackage);
        }
        if self.hash.clone().finalize()[..] != self.manifest.digest {
            return Err(Error::DigestMismatch);
        }
        if self.filled != 0 {
            self.write_sector(flash, self.received - self.filled as u32)?;
        }
        self.sealed = true;
        Ok(())
    }
    /// One bounded read/hash step, allowing the caller to yield between sectors.
    /// Returns true only after an independent read-back hash matches the manifest.
    pub fn verify_step<F: NorFlash>(&mut self, flash: &mut F) -> Result<bool, Error> {
        if !self.sealed {
            return Err(Error::BadPackage);
        }
        if self.verified {
            return Ok(true);
        }
        let offset = self.readback_offset;
        let n = (self.target.len - offset).min(SECTOR_SIZE as u32) as usize;
        if offset == 0 {
            self.readback_hash.update(&self.buffers.first[..n]);
        } else {
            let aligned = n.next_multiple_of(4);
            flash
                .read(self.base() + offset, &mut self.buffers.sector[..aligned])
                .map_err(|_| Error::Flash)?;
            self.readback_hash.update(&self.buffers.sector[..n]);
        }
        self.readback_offset += n as u32;
        if self.readback_offset == self.target.len {
            if self.readback_hash.clone().finalize()[..] != self.target.digest {
                return Err(Error::DigestMismatch);
            }
            self.verified = true;
        }
        Ok(self.verified)
    }
    pub fn commit_first<F: NorFlash>(&mut self, flash: &mut F) -> Result<(), Error> {
        if !self.verified {
            return Err(Error::Forbidden);
        }
        flash
            .write(self.base(), &self.buffers.first)
            .map_err(|_| Error::Flash)?;
        flash
            .read(self.base(), &mut self.buffers.sector)
            .map_err(|_| Error::Flash)?;
        if self.buffers.first != self.buffers.sector {
            return Err(Error::Flash);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind, ReadNorFlash};
    #[derive(Debug)]
    struct Failure;
    impl NorFlashError for Failure {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }
    struct Flash {
        bytes: std::vec::Vec<u8>,
        erases: std::vec::Vec<u32>,
    }
    impl ErrorType for Flash {
        type Error = Failure;
    }
    impl ReadNorFlash for Flash {
        const READ_SIZE: usize = 4;
        fn capacity(&self) -> usize {
            self.bytes.len()
        }
        fn read(&mut self, o: u32, b: &mut [u8]) -> Result<(), Failure> {
            b.copy_from_slice(&self.bytes[o as usize..o as usize + b.len()]);
            Ok(())
        }
    }
    impl NorFlash for Flash {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = 4096;
        fn erase(&mut self, a: u32, b: u32) -> Result<(), Failure> {
            self.erases.push(a);
            self.bytes[a as usize..b as usize].fill(255);
            Ok(())
        }
        fn write(&mut self, o: u32, b: &[u8]) -> Result<(), Failure> {
            for (d, s) in self.bytes[o as usize..].iter_mut().zip(b) {
                assert_eq!(*d & s, *s);
                *d &= s;
            }
            Ok(())
        }
    }
    fn fixture() -> std::vec::Vec<u8> {
        let mut b = std::vec![17; 9120];
        b[..288].fill(0);
        b[0] = 0xE9;
        b[1] = 1;
        b[3] = 0x30;
        b[12] = 13;
        b[28..32].copy_from_slice(&9000u32.to_le_bytes());
        b[32..36].copy_from_slice(&0xABCD5432u32.to_le_bytes());
        b[48..53].copy_from_slice(b"0.2.0");
        b[80..89].copy_from_slice(b"dcc-esp32");
        b
    }
    #[test]
    fn arbitrary_chunks_never_publish_before_readback_and_do_not_touch_running_slot() {
        let image = fixture();
        let manifest = Manifest {
            version: super::super::Version([0, 2, 0]),
            image_len: image.len() as u32,
            digest: Sha256::digest(&image).into(),
            key_id: [0; 8],
        };
        for (booted, chunk, len) in [
            (0, 1, 9120),
            (0, 17, 9120),
            (0, 4093, 9120),
            (0, 4096, 9120),
            (0, 5000, 9120),
            (1, 17, 9120),
            (1, 5000, 9120),
            (0, 4093, 0x300000),
            (1, 5000, 0x300000),
        ] {
            let mut image = image.clone();
            image.resize(len, 73);
            let manifest = Manifest {
                image_len: len as u32,
                digest: Sha256::digest(&image).into(),
                ..manifest
            };
            // Expected physical bounds are independent of the writer constants.
            let (running, target) = if booted == 0 {
                (0x10000, 0x310000)
            } else {
                (0x310000, 0x10000)
            };
            let mut f = Flash {
                bytes: std::vec![255;0x800000],
                erases: std::vec![],
            };
            f.bytes[running..running + 0x300000].fill(42);
            f.bytes[0x610000..].fill(99);
            let mut buffers = Buffers::default();
            f.bytes[target..target + SECTOR_SIZE].fill(0);
            let mut w = Writer::new(manifest, booted, &mut buffers, &mut f).unwrap();
            assert_eq!(w.target().slot, 1 - booted);
            assert_eq!(w.received(), 0);
            assert_eq!(f.erases, [target as u32]);
            assert!(
                f.bytes[target..target + SECTOR_SIZE]
                    .iter()
                    .all(|&b| b == 255)
            );
            assert_eq!(w.commit_first(&mut f), Err(Error::Forbidden));
            let mut received = 0;
            for bytes in image.chunks(chunk) {
                w.push(&mut f, bytes).unwrap();
                received += bytes.len() as u32;
                assert_eq!(w.received(), received);
            }
            assert_eq!(f.bytes[target], 255);
            w.seal(&mut f).unwrap();
            while !w.verify_step(&mut f).unwrap() {}
            w.commit_first(&mut f).unwrap();
            assert_eq!(&f.bytes[target..target + image.len()], &image);
            assert!(
                f.bytes[running..running + 0x300000]
                    .iter()
                    .all(|&b| b == 42)
            );
            assert!(f.bytes[0x610000..].iter().all(|&b| b == 99));
            assert!(
                f.erases
                    .iter()
                    .all(|&o| (target..target + 0x300000).contains(&(o as usize)))
            );
            assert_eq!(f.erases.iter().filter(|&&o| o == target as u32).count(), 1);
        }
        let mut f = Flash {
            bytes: std::vec![255;0x610000],
            erases: std::vec![],
        };
        let mut buffers = Buffers::default();
        let mut w = Writer::new(manifest, 0, &mut buffers, &mut f).unwrap();
        w.push(&mut f, &image).unwrap();
        w.seal(&mut f).unwrap();
        f.bytes[0x311021] ^= 1;
        assert_eq!(w.verify_step(&mut f), Ok(false));
        assert_eq!(w.verify_step(&mut f), Ok(false));
        assert_eq!(w.verify_step(&mut f), Err(Error::DigestMismatch));
        assert_eq!(f.bytes[0x310000], 255);
        assert_eq!(w.commit_first(&mut f), Err(Error::Forbidden));
    }
}
