//! The Ed25519 key this build trusts for its next update.
//!
//! The key travels inside the image as a [`BuildMetadata`] record, so
//! `ota-pack` can refuse to sign an image whose key differs from the signer.

use dcc_ota::Error;
use dcc_ota_format::image::{BUILD_METADATA_LEN, BuildKind, BuildMetadata, EmbeddedKey};
use ed25519_dalek::VerifyingKey;

#[cfg(feature = "ota-dev-key")]
const KEY: [u8; 32] = *include_bytes!("../../keys/ota_dev.pub");
#[cfg(not(feature = "ota-dev-key"))]
const KEY: [u8; 32] = *include_bytes!("../../keys/ota_release.pub");
/// The kind of this build; updates must be images it accepts.
#[cfg(feature = "ota-fault-inject")]
pub(super) const BUILD_KIND: BuildKind = BuildKind::Fault;
#[cfg(all(feature = "ota-dev-key", not(feature = "ota-fault-inject")))]
pub(super) const BUILD_KIND: BuildKind = BuildKind::Dev;
#[cfg(not(feature = "ota-dev-key"))]
pub(super) const BUILD_KIND: BuildKind = BuildKind::Release;

/// Kept in the linked image even though only [`trusted_key`] reads it.
#[used]
#[unsafe(link_section = ".rodata.ota_key")]
static BUILD_RECORD: [u8; BUILD_METADATA_LEN] = BuildMetadata {
    kind: BUILD_KIND,
    public_key: KEY,
}
.encode();

/// Returns the trusted key, read from the embedded record.
///
/// # Errors
/// Returns [`Error::Corrupt`] if the record is not a valid, strong key:
/// this build cannot verify any update.
pub(super) fn trusted_key() -> Result<VerifyingKey, Error> {
    // A volatile read keeps the record in the image and makes the check read
    // the bytes that ship, not a constant the compiler folded.
    // SAFETY: `BUILD_RECORD` is an immutable, aligned static; nothing writes it.
    let record = unsafe { core::ptr::read_volatile(&raw const BUILD_RECORD) };
    EmbeddedKey::decode(&record)
        .map(|embedded| embedded.key())
        .map_err(|error| {
            defmt::error!("OTA: embedded key unusable: {}", error.code());
            Error::Corrupt
        })
}
