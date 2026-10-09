//! Canonical, signed DCC firmware package header (all integers little endian).
use ed25519_dalek::{Signature, VerifyingKey};
use sha2::{Digest, Sha256};

use super::{Error, SLOT_SIZE, Version};

/// Size of the canonical header, including its Ed25519 signature.
pub const HEADER_LEN: usize = 256;
pub(crate) const PROJECT: &[u8] = b"dcc-esp32";

/// Canonical package metadata; decoding alone does not establish authenticity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Manifest {
    /// Release version advertised by the application image.
    pub version: Version,
    /// Application image length in bytes, excluding this header.
    pub image_len: u32,
    /// SHA-256 digest of the complete application image.
    pub digest: [u8; 32],
    /// Identifier of the signing public key, not a trust anchor.
    pub key_id: [u8; 8],
}

/// Returns the truncated SHA-256 identifier of a raw public key.
/// An identifier match is not authentication; use a trusted key to verify signatures.
pub fn key_id(key: &[u8; 32]) -> [u8; 8] {
    let hash = Sha256::digest(key);
    let mut id = [0; 8];
    id.copy_from_slice(&hash[..8]);
    id
}

/// Parses three canonical decimal `u16` components without leading zeroes.
///
/// # Errors
/// Returns [`Error::BadPackage`] for noncanonical syntax or component overflow.
pub fn parse_version(text: &str) -> Result<Version, Error> {
    let mut parts = text.split('.');
    let mut version = [0u16; 3];
    for value in &mut version {
        let part = parts.next().ok_or(Error::BadPackage)?;
        if part.is_empty() || (part.len() > 1 && part.starts_with('0')) {
            return Err(Error::BadPackage);
        }
        for digit in part.bytes() {
            if !digit.is_ascii_digit() {
                return Err(Error::BadPackage);
            }
            *value = value
                .checked_mul(10)
                .and_then(|v| v.checked_add((digit - b'0') as u16))
                .ok_or(Error::BadPackage)?;
        }
    }
    if parts.next().is_some() {
        return Err(Error::BadPackage);
    }
    Ok(Version(version))
}

impl Manifest {
    /// Produces the signing preimage in bytes 0..192; the signature tail is zero.
    /// Does not validate the supplied digest, key or application image.
    ///
    /// # Errors
    /// Returns [`Error::TooLarge`] above the slot size or [`Error::BadPackage`]
    /// for an image shorter than 4096 bytes.
    pub fn encode_unsigned(
        version: Version,
        image_len: u32,
        digest: [u8; 32],
        key: &[u8; 32],
    ) -> Result<[u8; HEADER_LEN], Error> {
        if image_len > SLOT_SIZE {
            return Err(Error::TooLarge);
        }
        if image_len < 4096 {
            return Err(Error::BadPackage);
        }
        let mut header = [0; HEADER_LEN];
        header[..8].copy_from_slice(b"DCC-OTA1");
        header[8..10].copy_from_slice(&256u16.to_le_bytes());
        header[12..14].copy_from_slice(&13u16.to_le_bytes());
        for (i, value) in version.0.iter().enumerate() {
            header[16 + i * 2..18 + i * 2].copy_from_slice(&value.to_le_bytes());
        }
        header[24..28].copy_from_slice(&image_len.to_le_bytes());
        header[32..64].copy_from_slice(&digest);
        header[64..64 + PROJECT.len()].copy_from_slice(PROJECT);
        header[96..104].copy_from_slice(&key_id(key));
        Ok(header)
    }

    /// Decodes a canonical header without authenticating its signature.
    ///
    /// # Errors
    /// Returns [`Error::BadPackage`] for malformed/noncanonical fields or a
    /// short image length, and [`Error::TooLarge`] for an oversized image length.
    pub fn decode(header: &[u8]) -> Result<Self, Error> {
        if header.len() != HEADER_LEN {
            return Err(Error::BadPackage);
        }
        let version = Version(core::array::from_fn(|i| {
            u16::from_le_bytes([header[16 + i * 2], header[17 + i * 2]])
        }));
        let image_len =
            u32::from_le_bytes(header[24..28].try_into().map_err(|_| Error::BadPackage)?);
        let digest = header[32..64].try_into().map_err(|_| Error::BadPackage)?;
        let canonical = Self::encode_unsigned(version, image_len, digest, &[0; 32])?;
        // Compare all fields, including reserved bytes, except the key identifier.
        if header[..96] != canonical[..96] || header[104..192] != canonical[104..192] {
            return Err(Error::BadPackage);
        }
        Ok(Self {
            version,
            image_len,
            digest,
            key_id: header[96..104].try_into().unwrap(),
        })
    }

    /// Decodes and authenticates a header, independently of device version policy.
    /// The caller must supply a trusted public key and separately verify image
    /// bytes against the authenticated digest; this verifies only the header.
    ///
    /// # Errors
    /// Propagates decoding errors; returns [`Error::UnknownKey`] for a mismatched
    /// identifier or invalid key, and [`Error::BadSignature`] for an invalid signature.
    pub fn authenticate(header: &[u8], public_key: &[u8; 32]) -> Result<Self, Error> {
        let manifest = Self::decode(header)?;
        if manifest.key_id != key_id(public_key) {
            return Err(Error::UnknownKey);
        }
        let key = VerifyingKey::from_bytes(public_key).map_err(|_| Error::UnknownKey)?;
        let signature = Signature::from_slice(&header[192..]).map_err(|_| Error::BadSignature)?;
        key.verify_strict(&header[..192], &signature)
            .map_err(|_| Error::BadSignature)?;
        Ok(manifest)
    }

    /// Checks the running release and persistent failed-boot floor.
    /// Call only after authentication; a decoded manifest alone is untrusted.
    ///
    /// # Errors
    /// Returns [`Error::SameVersion`] or [`Error::Downgrade`] relative to the
    /// running version, then [`Error::RejectedVersion`] at or below the failed-boot floor.
    pub fn check_eligibility(
        &self,
        running: Version,
        rejected: Option<Version>,
    ) -> Result<(), Error> {
        if self.version == running {
            return Err(Error::SameVersion);
        }
        if self.version < running {
            return Err(Error::Downgrade);
        }
        if rejected.is_some_and(|floor| self.version <= floor) {
            return Err(Error::RejectedVersion);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};

    #[test]
    fn canonical_versions() {
        assert_eq!(parse_version("65535.0.42"), Ok(Version([65535, 0, 42])));
        for bad in [
            "",
            "1.2",
            "1.2.3.4",
            "01.2.3",
            "1.02.3",
            "1.2.03",
            "65536.0.0",
            "1.2.3-dev",
            "+1.2.3",
            "1.2. 3",
            "1..3",
        ] {
            assert!(parse_version(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn signature_and_policy() {
        let signing = SigningKey::from_bytes(&[42; 32]);
        let key = signing.verifying_key().to_bytes();
        let version = Version([1, 2, 3]);
        let mut header = Manifest::encode_unsigned(version, 4096, [7; 32], &key).unwrap();
        let signature = signing.sign(&header[..192]).to_bytes();
        header[192..].copy_from_slice(&signature);
        let verify = |h: &[u8], run, reject| {
            let manifest = Manifest::authenticate(h, &key)?;
            manifest.check_eligibility(run, reject)?;
            Ok(manifest)
        };
        assert!(verify(&header, Version::default(), None).is_ok());
        assert_eq!(verify(&header, version, None), Err(Error::SameVersion));
        assert_eq!(
            verify(&header, Version([2, 0, 0]), None),
            Err(Error::Downgrade)
        );
        assert_eq!(
            verify(&header, Version::default(), Some(version)),
            Err(Error::RejectedVersion)
        );
        assert_eq!(
            verify(&header, Version::default(), Some(Version([1, 2, 4]))),
            Err(Error::RejectedVersion)
        );
        assert!(verify(&header, Version::default(), Some(Version([1, 2, 2]))).is_ok());
        assert_eq!(
            Manifest::authenticate(&header, &[5; 32]),
            Err(Error::UnknownKey)
        );
        for offset in [10, 14, 22, 28, 73, 104, 191] {
            let mut bad = header;
            bad[offset] = 1;
            assert_eq!(
                verify(&bad, Version::default(), None),
                Err(Error::BadPackage)
            );
            let sig = signing.sign(&bad[..192]).to_bytes();
            bad[192..].copy_from_slice(&sig);
            assert_eq!(
                verify(&bad, Version::default(), None),
                Err(Error::BadPackage)
            );
        }
        header[32] ^= 1;
        assert_eq!(
            verify(&header, Version::default(), None),
            Err(Error::BadSignature)
        );
        for len in 0..256 {
            assert!(verify(&header[..len], Version::default(), None).is_err());
        }
        assert!(Manifest::encode_unsigned(version, 4095, [0; 32], &key).is_err());
        assert!(Manifest::encode_unsigned(version, SLOT_SIZE, [0; 32], &key).is_ok());
        assert_eq!(
            Manifest::encode_unsigned(version, SLOT_SIZE + 1, [0; 32], &key),
            Err(Error::TooLarge)
        );
    }
}
