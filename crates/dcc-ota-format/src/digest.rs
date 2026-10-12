//! SHA-256 digests of images, whole or streamed.

use sha2::{Digest as _, Sha256};

use crate::fields::write_hex;

/// SHA-256 of an application image.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Digest([u8; 32]);

impl Digest {
    /// Wraps raw digest bytes.
    #[must_use]
    pub const fn new(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    /// Hashes `bytes`.
    #[must_use]
    pub fn of(bytes: &[u8]) -> Self {
        Self(Sha256::digest(bytes).into())
    }

    /// Returns the raw digest bytes.
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

impl core::fmt::Display for Digest {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write_hex(f, &self.0)
    }
}

/// A SHA-256 computed over bytes that arrive in pieces.
#[derive(Debug, Clone, Default)]
pub struct DigestStream(Sha256);

impl DigestStream {
    /// Nothing hashed yet.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Hashes the next `bytes`.
    pub fn update(&mut self, bytes: &[u8]) {
        self.0.update(bytes);
    }

    /// The digest of every byte so far; the stream starts over.
    pub fn finish(&mut self) -> Digest {
        Digest(core::mem::take(&mut self.0).finalize().into())
    }
}
