//! The build record: marker, signing policy and the key the image trusts
//! for its next update, embedded once in the image's read-only data.

use ed25519_dalek::VerifyingKey;

use super::ImageError;
use crate::fields::{array_at, le_u32};

/// Marker that opens the build record in the image's read-only data.
///
/// Only [`BuildMetadata::encode`] may use these bytes, at compile time: the
/// image must contain the marker exactly once, so runtime code compares
/// against [`MARKER_WINDOW`] instead and never holds a copy of it.
pub(crate) const BUILD_MARKER: [u8; 8] = *b"DCCKEY01";
/// The marker as [`window`] builds it: bytes shifted in from the low end,
/// so the first byte ends up most significant. Stored in memory little
/// endian, this value is the marker reversed, never the marker itself.
const MARKER_WINDOW: u64 = u64::from_be_bytes(BUILD_MARKER);
/// Marker, build kind as little-endian `u32`, key.
pub const BUILD_METADATA_LEN: usize = 44;
pub(crate) const BUILD_KIND_AT: usize = 8;
const BUILD_KEY_AT: usize = 12;
/// Bytes of the record after the marker: kind and key.
const BUILD_BODY_LEN: usize = BUILD_METADATA_LEN - BUILD_KIND_AT;

/// Shifts `byte` into an eight-byte window of the stream.
const fn window(window: u64, byte: u8) -> u64 {
    (window << 8) | byte as u64
}

/// Whether `bytes` start with the build marker.
fn starts_with_marker(bytes: &[u8]) -> bool {
    bytes.len() >= BUILD_MARKER.len()
        && bytes[..BUILD_MARKER.len()]
            .iter()
            .fold(0, |w, &b| window(w, b))
            == MARKER_WINDOW
}

/// Signing policy a build was made for; says nothing about authenticity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum BuildKind {
    /// Production build.
    Release = 0,
    /// Bench build with injected failures; value 1 was once a bench build
    /// without them and is no longer valid.
    Fault = 2,
}

impl BuildKind {
    /// Whether a device running this kind of build accepts an update to an
    /// image of `kind`: a release device takes release images only, a bench
    /// device takes any.
    #[must_use]
    pub const fn accepts(self, kind: Self) -> bool {
        match self {
            Self::Release => matches!(kind, Self::Release),
            Self::Fault => true,
        }
    }
}

impl TryFrom<u32> for BuildKind {
    type Error = ImageError;

    fn try_from(value: u32) -> Result<Self, ImageError> {
        match value {
            0 => Ok(Self::Release),
            2 => Ok(Self::Fault),
            _ => Err(ImageError::BadBuild),
        }
    }
}

/// The raw record a build embeds; embed [`Self::encode`]'s bytes, not this
/// struct's memory layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BuildMetadata {
    /// Signing policy of the build.
    pub kind: BuildKind,
    /// Raw Ed25519 key the build trusts for its next update.
    pub public_key: [u8; 32],
}

impl BuildMetadata {
    /// Encodes the record: marker, kind, key.
    #[must_use]
    pub const fn encode(self) -> [u8; BUILD_METADATA_LEN] {
        let mut record = [0; BUILD_METADATA_LEN];
        let (marker, rest) = record.split_at_mut(BUILD_KIND_AT);
        marker.copy_from_slice(&BUILD_MARKER);
        let (kind, key) = rest.split_at_mut(BUILD_KEY_AT - BUILD_KIND_AT);
        kind.copy_from_slice(&(self.kind as u32).to_le_bytes());
        key.copy_from_slice(&self.public_key);
        record
    }
}

/// A decoded build record: the policy and the strong key it trusts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EmbeddedKey {
    kind: BuildKind,
    key: VerifyingKey,
}

impl EmbeddedKey {
    /// Signing policy of the build.
    #[must_use]
    pub const fn kind(&self) -> BuildKind {
        self.kind
    }

    /// Key the build trusts for its next update; never weak.
    #[must_use]
    pub const fn key(&self) -> VerifyingKey {
        self.key
    }

    /// Decodes a whole record, refusing a wrong marker, an unknown kind or a
    /// weak key.
    ///
    /// # Errors
    /// Returns [`ImageError::BadBuild`] for a wrong marker, kind or key.
    pub fn decode(record: &[u8; BUILD_METADATA_LEN]) -> Result<Self, ImageError> {
        if !starts_with_marker(record) {
            return Err(ImageError::BadBuild);
        }
        Self::decode_body(&array_at(record, BUILD_KIND_AT))
    }

    /// Decodes the kind and key that follow the marker.
    fn decode_body(body: &[u8; BUILD_BODY_LEN]) -> Result<Self, ImageError> {
        let key = VerifyingKey::from_bytes(&array_at(body, BUILD_KEY_AT - BUILD_KIND_AT))
            .map_err(|_| ImageError::BadBuild)?;
        if key.is_weak() {
            return Err(ImageError::BadBuild);
        }
        Ok(Self {
            kind: BuildKind::try_from(le_u32(body, 0))?,
            key,
        })
    }

    /// Finds the only build record in `image`.
    ///
    /// # Errors
    /// Returns [`ImageError::BadBuild`] for a missing, repeated, truncated
    /// or invalid record.
    pub fn find(image: &[u8]) -> Result<Self, ImageError> {
        let mut scanner = BuildScanner::new();
        scanner.feed(image);
        scanner.finish()
    }
}

/// Finds the build record while an image streams past, without holding
/// the image: it must appear exactly once.
#[derive(Debug, Clone, Default)]
pub struct BuildScanner {
    /// The last eight bytes seen.
    window: u64,
    /// Bytes seen, to ignore a window not yet full.
    seen: usize,
    /// The record body being captured, and how much of it.
    capture: Option<([u8; BUILD_BODY_LEN], usize)>,
    found: Option<[u8; BUILD_BODY_LEN]>,
    repeated: bool,
}

impl BuildScanner {
    /// Nothing scanned yet.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            window: 0,
            seen: 0,
            capture: None,
            found: None,
            repeated: false,
        }
    }

    /// Scans the next bytes of the image.
    pub fn feed(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            if let Some((body, len)) = &mut self.capture {
                body[*len] = byte;
                *len += 1;
                if *len == BUILD_BODY_LEN {
                    self.repeated |= self.found.is_some();
                    self.found = Some(*body);
                    self.capture = None;
                }
            }
            self.window = window(self.window, byte);
            self.seen += 1;
            if self.seen >= BUILD_MARKER.len() && self.window == MARKER_WINDOW {
                self.repeated |= self.capture.is_some() || self.found.is_some();
                self.capture = Some(([0; BUILD_BODY_LEN], 0));
            }
        }
    }

    /// The one record the image carried.
    ///
    /// # Errors
    /// Returns [`ImageError::BadBuild`] for a missing, repeated, truncated
    /// or invalid record.
    pub fn finish(&self) -> Result<EmbeddedKey, ImageError> {
        match (&self.found, self.repeated || self.capture.is_some()) {
            (Some(body), false) => EmbeddedKey::decode_body(body),
            _ => Err(ImageError::BadBuild),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{fixture, signing_key};

    #[test]
    fn the_build_record_must_be_unique_and_carry_a_strong_key() {
        let key = signing_key().verifying_key();
        let record = |kind| {
            BuildMetadata {
                kind,
                public_key: key.to_bytes(),
            }
            .encode()
        };
        for kind in [BuildKind::Release, BuildKind::Fault] {
            assert_eq!(
                EmbeddedKey::find(&record(kind)),
                Ok(EmbeddedKey { kind, key })
            );
            for len in 0..BUILD_METADATA_LEN {
                assert!(EmbeddedKey::find(&record(kind)[..len]).is_err());
            }
            assert!(EmbeddedKey::find(&[record(kind), record(kind)].concat()).is_err());
        }
        for unknown in [1, 3] {
            let mut unknown_kind = record(BuildKind::Fault);
            unknown_kind[BUILD_KIND_AT] = unknown;
            assert!(EmbeddedKey::find(&unknown_kind).is_err());
        }
        let weak = BuildMetadata {
            kind: BuildKind::Fault,
            public_key: [0; 32],
        }
        .encode();
        assert!(EmbeddedKey::find(&weak).is_err());
    }

    #[test]
    fn the_record_is_found_across_any_chunking() {
        let key = signing_key().verifying_key();
        let record = BuildMetadata {
            kind: BuildKind::Fault,
            public_key: key.to_bytes(),
        }
        .encode();
        let mut image = fixture("1.2.3", 4096, false);
        image[300..300 + BUILD_METADATA_LEN].copy_from_slice(&record);
        for chunk in [1, 7, 43, 4096] {
            let mut scanner = BuildScanner::new();
            image.chunks(chunk).for_each(|part| scanner.feed(part));
            assert_eq!(
                scanner.finish(),
                Ok(EmbeddedKey {
                    kind: BuildKind::Fault,
                    key
                }),
                "chunk {chunk}"
            );
        }
        let mut twice = image.clone();
        twice[600..600 + BUILD_METADATA_LEN].copy_from_slice(&record);
        let mut scanner = BuildScanner::new();
        scanner.feed(&twice);
        assert_eq!(scanner.finish(), Err(ImageError::BadBuild));
        let mut truncated = BuildScanner::new();
        truncated.feed(&record[..20]);
        assert_eq!(truncated.finish(), Err(ImageError::BadBuild));
        assert!(BuildKind::Release.accepts(BuildKind::Release));
        assert!(!BuildKind::Release.accepts(BuildKind::Fault));
        assert!(BuildKind::Fault.accepts(BuildKind::Release));
        assert!(BuildKind::Fault.accepts(BuildKind::Fault));
    }
}
