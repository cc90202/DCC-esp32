//! The signed package header (`.dccfw`): 192 bytes of canonical metadata
//! followed by their Ed25519 signature. All integers are little endian.
//!
//! | Bytes    | Field                      |
//! |----------|----------------------------|
//! | 0..8     | magic `DCC-OTA1`           |
//! | 8..10    | header length, 256         |
//! | 12..14   | ESP chip id, 13 (ESP32-C6) |
//! | 16..22   | version                    |
//! | 24..28   | image length               |
//! | 32..64   | SHA-256 of the image       |
//! | 64..96   | project name, zero padded  |
//! | 96..104  | signing key id             |
//! | 192..256 | signature over bytes 0..192|
//!
//! Every other byte is zero; decoding rejects any other spelling. A decoded
//! [`UnverifiedManifest`] becomes a [`Manifest`] only through
//! [`Manifest::authenticate`], so code that writes an image can only be
//! handed an authenticated one.

use ed25519_dalek::{Signature, VerifyingKey};
use sha2::{Digest as _, Sha256};

use crate::{
    Digest, ImageLen, Version,
    fields::{array_at, le_u32, write_hex},
    target::{ESP32C6_CHIP_ID, PROJECT},
};

/// Size of the header, including its signature.
pub const HEADER_LEN: usize = 256;
/// Bytes covered by the signature.
pub const SIGNED_LEN: usize = 192;
const MAGIC: &[u8; 8] = b"DCC-OTA1";
const HEADER_LEN_AT: usize = 8;
const CHIP_AT: usize = 12;
const VERSION_AT: usize = 16;
const IMAGE_LEN_AT: usize = 24;
const DIGEST_AT: usize = 32;
const PROJECT_AT: usize = 64;
const KEY_ID_AT: usize = 96;
/// [`HEADER_LEN`] as the header's own length field.
const HEADER_LEN_FIELD: u16 = 256;
const _: () = assert!(HEADER_LEN_FIELD as usize == HEADER_LEN);

/// First 8 bytes of the SHA-256 of a public key.
///
/// It only names the key a package claims; it authenticates nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeyId([u8; 8]);

impl KeyId {
    /// Names `key`.
    #[must_use]
    pub fn of(key: &VerifyingKey) -> Self {
        Self(array_at(&Sha256::digest(key.as_bytes()), 0))
    }
}

impl core::fmt::Display for KeyId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write_hex(f, &self.0)
    }
}

/// Why a package header is refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PackageError {
    /// The header is malformed, or the package length does not match it.
    Malformed,
    /// The image or the request is larger than an application slot.
    TooLarge,
    /// The header names a signing key this device does not trust.
    UnknownKey,
    /// The signature does not verify with the trusted key.
    BadSignature,
}

impl PackageError {
    /// Returns the stable machine-readable code.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::Malformed => "bad_package",
            Self::TooLarge => "too_large",
            Self::UnknownKey => "unknown_key",
            Self::BadSignature => "bad_signature",
        }
    }
}

crate::coded_error!(PackageError, Ineligible);

/// Why an authentic release may not be installed on this device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ineligible {
    /// The release is the one already running.
    SameVersion,
    /// The release is older than the one running.
    Downgrade,
    /// The release is at or below one that already failed its trial boot.
    Rejected,
}

impl Ineligible {
    /// Returns the stable machine-readable code.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::SameVersion => "same_version",
            Self::Downgrade => "downgrade",
            Self::Rejected => "rejected_version",
        }
    }
}

/// Builds a header with a zero signature; sign bytes `0..SIGNED_LEN`.
#[must_use]
pub fn unsigned_header(
    version: Version,
    image_len: ImageLen,
    digest: Digest,
    key_id: KeyId,
) -> [u8; HEADER_LEN] {
    let mut header = [0; HEADER_LEN];
    header[..8].copy_from_slice(MAGIC);
    header[HEADER_LEN_AT..HEADER_LEN_AT + 2].copy_from_slice(&HEADER_LEN_FIELD.to_le_bytes());
    header[CHIP_AT..CHIP_AT + 2].copy_from_slice(&ESP32C6_CHIP_ID.to_le_bytes());
    header[VERSION_AT..VERSION_AT + Version::ENCODED_LEN].copy_from_slice(&version.to_le_bytes());
    header[IMAGE_LEN_AT..IMAGE_LEN_AT + 4].copy_from_slice(&image_len.get().to_le_bytes());
    header[DIGEST_AT..DIGEST_AT + 32].copy_from_slice(digest.as_bytes());
    header[PROJECT_AT..PROJECT_AT + PROJECT.len()].copy_from_slice(PROJECT);
    header[KEY_ID_AT..KEY_ID_AT + 8].copy_from_slice(&key_id.0);
    header
}

/// Header fields as decoded, before any signature check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnverifiedManifest {
    version: Version,
    image_len: ImageLen,
    digest: Digest,
    key_id: KeyId,
}

impl UnverifiedManifest {
    /// Decodes a header without checking its signature.
    ///
    /// # Errors
    /// Returns [`PackageError::Malformed`] for an image shorter than
    /// [`ImageLen::MIN`] or any noncanonical byte, and
    /// [`PackageError::TooLarge`] for an image longer than a slot.
    pub fn decode(header: &[u8; HEADER_LEN]) -> Result<Self, PackageError> {
        let len = le_u32(header, IMAGE_LEN_AT);
        let image_len = ImageLen::new(len).ok_or(if len > ImageLen::MAX.get() {
            PackageError::TooLarge
        } else {
            PackageError::Malformed
        })?;
        let manifest = Self {
            version: Version::from_le_bytes(array_at(header, VERSION_AT)),
            image_len,
            digest: Digest::new(array_at(header, DIGEST_AT)),
            key_id: KeyId(array_at(header, KEY_ID_AT)),
        };
        let canonical = unsigned_header(
            manifest.version,
            manifest.image_len,
            manifest.digest,
            manifest.key_id,
        );
        if header[..SIGNED_LEN] != canonical[..SIGNED_LEN] {
            return Err(PackageError::Malformed);
        }
        Ok(manifest)
    }

    /// Release version the header claims.
    #[must_use]
    pub const fn version(&self) -> Version {
        self.version
    }

    /// Image length, without this header.
    #[must_use]
    pub const fn image_len(&self) -> ImageLen {
        self.image_len
    }

    /// SHA-256 the header claims for the image.
    #[must_use]
    pub const fn digest(&self) -> Digest {
        self.digest
    }

    /// Key the package claims to be signed with.
    #[must_use]
    pub const fn key_id(&self) -> KeyId {
        self.key_id
    }
}

/// Header fields whose signature verified with a trusted key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Manifest(UnverifiedManifest);

impl Manifest {
    /// Decodes `header` and checks its signature with the trusted `key`.
    ///
    /// The image bytes still have to match the [digest](UnverifiedManifest::digest);
    /// device version policy is [`Self::check_eligibility`].
    ///
    /// # Errors
    /// Propagates [`UnverifiedManifest::decode`] errors; returns
    /// [`PackageError::UnknownKey`] when the header names another key and
    /// [`PackageError::BadSignature`] when the signature does not verify.
    pub fn authenticate(
        header: &[u8; HEADER_LEN],
        key: &VerifyingKey,
    ) -> Result<Self, PackageError> {
        let manifest = UnverifiedManifest::decode(header)?;
        if manifest.key_id != KeyId::of(key) {
            return Err(PackageError::UnknownKey);
        }
        let signature =
            Signature::from_slice(&header[SIGNED_LEN..]).map_err(|_| PackageError::BadSignature)?;
        key.verify_strict(&header[..SIGNED_LEN], &signature)
            .map_err(|_| PackageError::BadSignature)?;
        Ok(Self(manifest))
    }

    /// The authenticated fields: version, image length and digest.
    #[must_use]
    pub const fn fields(&self) -> &UnverifiedManifest {
        &self.0
    }

    /// Checks the version against the running release and the highest
    /// release that already failed a trial boot.
    ///
    /// # Errors
    /// Returns [`Ineligible::SameVersion`] or [`Ineligible::Downgrade`]
    /// against `running`, then [`Ineligible::Rejected`] at or below
    /// `rejected`.
    pub fn check_eligibility(
        &self,
        running: Version,
        rejected: Option<Version>,
    ) -> Result<(), Ineligible> {
        let version = self.0.version;
        if version == running {
            return Err(Ineligible::SameVersion);
        }
        if version < running {
            return Err(Ineligible::Downgrade);
        }
        if rejected.is_some_and(|floor| version <= floor) {
            return Err(Ineligible::Rejected);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{manifest, signed, signing_key};
    use ed25519_dalek::{Signer, SigningKey};

    #[test]
    fn only_a_signed_canonical_header_authenticates() {
        let len = ImageLen::new(4096).unwrap();
        let header = signed(Version::new(1, 2, 3), len, Digest::new([7; 32]));
        let key = signing_key().verifying_key();
        assert!(Manifest::authenticate(&header, &key).is_ok());
        let other = SigningKey::from_bytes(&[5; 32]).verifying_key();
        assert_eq!(
            Manifest::authenticate(&header, &other),
            Err(PackageError::UnknownKey)
        );
        // Reserved or fixed bytes changed, even when re-signed: not canonical.
        for offset in [10, 14, 22, 28, 73, 104, 191] {
            let mut bad = header;
            bad[offset] = 1;
            let signature = signing_key().sign(&bad[..SIGNED_LEN]).to_bytes();
            bad[SIGNED_LEN..].copy_from_slice(&signature);
            assert_eq!(
                Manifest::authenticate(&bad, &key),
                Err(PackageError::Malformed),
                "{offset}"
            );
        }
        let mut forged = header;
        forged[DIGEST_AT] ^= 1;
        assert_eq!(
            Manifest::authenticate(&forged, &key),
            Err(PackageError::BadSignature)
        );
    }

    #[test]
    fn a_header_length_outside_one_slot_is_refused() {
        let header = signed(Version::default(), ImageLen::MIN, Digest::default());
        let with_len = |len: u32| {
            let mut bad = header;
            bad[IMAGE_LEN_AT..IMAGE_LEN_AT + 4].copy_from_slice(&len.to_le_bytes());
            UnverifiedManifest::decode(&bad)
        };
        assert_eq!(
            with_len(ImageLen::MIN.get() - 1),
            Err(PackageError::Malformed)
        );
        assert!(with_len(ImageLen::MAX.get()).is_ok());
        assert_eq!(
            with_len(ImageLen::MAX.get() + 1),
            Err(PackageError::TooLarge)
        );
    }

    #[test]
    fn eligibility_refuses_same_older_and_rejected_versions() {
        let manifest = manifest(Version::new(1, 2, 3), &[0; 4096]);
        let check = |running, rejected| manifest.check_eligibility(running, rejected);
        assert_eq!(check(Version::default(), None), Ok(()));
        assert_eq!(
            check(Version::new(1, 2, 3), None),
            Err(Ineligible::SameVersion)
        );
        assert_eq!(
            check(Version::new(2, 0, 0), None),
            Err(Ineligible::Downgrade)
        );
        let floor = |patch| Some(Version::new(1, 2, patch));
        assert_eq!(
            check(Version::default(), floor(3)),
            Err(Ineligible::Rejected)
        );
        assert_eq!(
            check(Version::default(), floor(4)),
            Err(Ineligible::Rejected)
        );
        assert_eq!(check(Version::default(), floor(2)), Ok(()));
    }
}
