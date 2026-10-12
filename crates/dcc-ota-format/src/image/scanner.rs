//! Structural check of an image while it streams past.

use super::{
    ImageError,
    header::{
        APPENDED_HASH_LEN, CHECKSUM_SEED, Header, IMAGE_HEADER_LEN, SEGMENT_HEADER_LEN,
        SEGMENT_LEN_AT, segment_end, trailer_end,
    },
};
use crate::{DigestStream, ImageLen, fields::le_u32};

/// Validates an image while it streams past, without holding it: segment
/// headers, the XOR checksum over the segment data, the zero padding before
/// it and the optional appended SHA-256, the same rules
/// [`inspect`](super::inspect) applies.
#[derive(Debug, Clone)]
pub struct ImageScanner {
    /// Bytes seen so far.
    seen: u32,
    image_header: [u8; IMAGE_HEADER_LEN as usize],
    header: Option<Header>,
    segments_left: u8,
    /// Where the next segment header starts.
    next: u32,
    segment_header: [u8; SEGMENT_HEADER_LEN as usize],
    /// End of the data of the segment being read.
    data_end: u32,
    checksum: u8,
    /// Position of the checksum byte, once every segment header was read.
    checksum_at: Option<u32>,
    checksum_seen: Option<u8>,
    hash: DigestStream,
    appended: [u8; APPENDED_HASH_LEN as usize],
    broken: bool,
}

impl Default for ImageScanner {
    fn default() -> Self {
        Self::new()
    }
}

impl ImageScanner {
    /// Nothing scanned yet.
    #[must_use]
    pub fn new() -> Self {
        Self {
            seen: 0,
            image_header: [0; IMAGE_HEADER_LEN as usize],
            header: None,
            segments_left: 0,
            next: IMAGE_HEADER_LEN,
            segment_header: [0; SEGMENT_HEADER_LEN as usize],
            data_end: IMAGE_HEADER_LEN,
            checksum: CHECKSUM_SEED,
            checksum_at: None,
            checksum_seen: None,
            hash: DigestStream::new(),
            appended: [0; APPENDED_HASH_LEN as usize],
            broken: false,
        }
    }

    /// Scans the next bytes of the image.
    pub fn feed(&mut self, bytes: &[u8]) {
        let first = self.seen;
        for &byte in bytes {
            if self.broken {
                return;
            }
            let at = self.seen;
            self.seen += 1;
            self.scan(at, byte);
        }
        // The appended SHA-256 covers everything before it.
        if self.header.is_some_and(|h| h.appended_hash) || first < IMAGE_HEADER_LEN {
            let limit = self.checksum_at.map_or(u32::MAX, |at| at + 1);
            let hashed = limit.min(self.seen).saturating_sub(first) as usize;
            self.hash.update(&bytes[..hashed.min(bytes.len())]);
        }
    }

    fn scan(&mut self, at: u32, byte: u8) {
        if at < IMAGE_HEADER_LEN {
            self.image_header[at as usize] = byte;
            if at + 1 == IMAGE_HEADER_LEN {
                self.start_segments();
            }
            return;
        }
        if let Some(checksum_at) = self.checksum_at {
            self.scan_trailer(at, checksum_at, byte);
        } else if at < self.data_end {
            self.checksum ^= byte;
        } else {
            let index = (at - self.next) as usize;
            self.segment_header[index] = byte;
            if index + 1 == self.segment_header.len() {
                self.end_segment_header();
            }
        }
    }

    fn start_segments(&mut self) {
        match Header::parse(&self.image_header) {
            Ok(header) => {
                self.header = Some(header);
                self.segments_left = header.segments;
            }
            Err(_) => self.broken = true,
        }
    }

    fn end_segment_header(&mut self) {
        let length = le_u32(&self.segment_header, SEGMENT_LEN_AT);
        let Ok(end) = segment_end(self.next, length) else {
            self.broken = true;
            return;
        };
        self.data_end = end;
        self.next = end;
        self.segments_left -= 1;
        if self.segments_left == 0
            && let Some(header) = self.header
        {
            match trailer_end(end, header) {
                Ok(trailer) => self.checksum_at = Some(trailer - header.hash_len() - 1),
                Err(_) => self.broken = true,
            }
        }
    }

    fn scan_trailer(&mut self, at: u32, checksum_at: u32, byte: u8) {
        if at < self.data_end {
            self.checksum ^= byte;
        } else if at < checksum_at {
            // Padding before the checksum must be zero.
            self.broken |= byte != 0;
        } else if at == checksum_at {
            self.checksum_seen = Some(byte);
        } else {
            match self.appended.get_mut((at - checksum_at - 1) as usize) {
                Some(slot) if self.header.is_some_and(|h| h.appended_hash) => *slot = byte,
                _ => self.broken = true,
            }
        }
    }

    /// Checks that the image is complete, consistent and exactly `len`
    /// bytes long.
    ///
    /// # Errors
    /// Returns [`ImageError::Malformed`] for any structural fault, and
    /// [`ImageError::DigestMismatch`] for a wrong appended SHA-256.
    pub fn finish(&mut self, len: ImageLen) -> Result<(), ImageError> {
        if self.broken {
            return Err(ImageError::Malformed);
        }
        let (Some(header), Some(checksum_at)) = (self.header, self.checksum_at) else {
            return Err(ImageError::Malformed);
        };
        if self.seen != len.get()
            || checksum_at + 1 + header.hash_len() != len.get()
            || self.checksum_seen != Some(self.checksum)
        {
            return Err(ImageError::Malformed);
        }
        if header.appended_hash && *self.hash.finish().as_bytes() != self.appended {
            return Err(ImageError::DigestMismatch);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{image::header::FIRST_SEGMENT_LEN_AT, testing::fixture};

    #[test]
    fn the_streamed_image_must_be_whole_and_end_at_the_signed_length() {
        let len = |n| ImageLen::new(n).unwrap();
        let mut image = fixture("1.2.3", 4128, true);
        for chunk in [1, 13, 4096] {
            let mut layout = ImageScanner::new();
            image.chunks(chunk).for_each(|part| layout.feed(part));
            assert_eq!(layout.finish(len(4128)), Ok(()), "chunk {chunk}");
            assert_eq!(
                layout.finish(len(4144)),
                Err(ImageError::Malformed),
                "trailing bytes"
            );
        }
        let mut wrong_checksum = image.clone();
        wrong_checksum[4095] ^= 1;
        let mut scanner = ImageScanner::new();
        scanner.feed(&wrong_checksum);
        assert_eq!(scanner.finish(len(4128)), Err(ImageError::Malformed));
        image[FIRST_SEGMENT_LEN_AT] ^= 3;
        let mut broken = ImageScanner::new();
        broken.feed(&image);
        assert_eq!(broken.finish(len(4128)), Err(ImageError::Malformed));
    }
}
