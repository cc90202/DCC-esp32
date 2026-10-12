//! Fixtures for tests: well-formed images and packages signed with a fixed
//! key. Never part of a firmware build.

use ed25519_dalek::{Signer, SigningKey};

use crate::{
    Digest, ImageLen, Version,
    fields::le_u32,
    image::{
        PREFIX_LEN,
        header::{
            APP_DESC_AT, APP_DESC_MAGIC, CHECKSUM_SEED, CHIP_ID_AT, FIRST_SEGMENT_LEN_AT,
            FLASH_8MB, FLASH_SIZE_AT, HASH_FLAG_AT, IMAGE_MAGIC, PROJECT_FIELD, SEGMENTS_AT,
            VERSION_FIELD,
        },
    },
    package::{HEADER_LEN, KeyId, Manifest, SIGNED_LEN, unsigned_header},
    target::{ESP32C6_CHIP_ID, PROJECT},
};

/// A one-segment image of exactly `len` bytes for `version`.
///
/// # Panics
/// If `len` does not end a checksum block.
#[must_use]
pub fn fixture(version: &str, len: usize, appended_hash: bool) -> Vec<u8> {
    let hash_len = if appended_hash { 32 } else { 0 };
    let body = len - hash_len;
    let mut image = vec![0; len];
    image[0] = IMAGE_MAGIC;
    image[SEGMENTS_AT] = 1;
    image[FLASH_SIZE_AT] = FLASH_8MB;
    image[CHIP_ID_AT..CHIP_ID_AT + 2].copy_from_slice(&ESP32C6_CHIP_ID.to_le_bytes());
    image[HASH_FLAG_AT] = u8::from(appended_hash);
    let segment = (body - 48) as u32 & !15;
    image[FIRST_SEGMENT_LEN_AT..FIRST_SEGMENT_LEN_AT + 4].copy_from_slice(&segment.to_le_bytes());
    image[APP_DESC_AT..APP_DESC_AT + 4].copy_from_slice(&APP_DESC_MAGIC.to_le_bytes());
    image[VERSION_FIELD.start..VERSION_FIELD.start + version.len()]
        .copy_from_slice(version.as_bytes());
    image[PROJECT_FIELD.start..PROJECT_FIELD.start + PROJECT.len()].copy_from_slice(PROJECT);
    let end = APP_DESC_AT + segment as usize;
    for (i, byte) in image[PREFIX_LEN..end].iter_mut().enumerate() {
        *byte = (i % 251) as u8;
    }
    assert_eq!(
        (end + 16) & !15,
        body,
        "fixture length must end a checksum block"
    );
    reseal(&mut image, appended_hash);
    image
}

/// Recomputes the checksum and appended hash of a one-segment image after
/// its data changed.
pub fn reseal(image: &mut [u8], appended_hash: bool) {
    let body = image.len() - if appended_hash { 32 } else { 0 };
    let end = APP_DESC_AT + le_u32(image, FIRST_SEGMENT_LEN_AT) as usize;
    image[body - 1] = image[APP_DESC_AT..end]
        .iter()
        .fold(CHECKSUM_SEED, |a, b| a ^ b);
    if appended_hash {
        let digest = Digest::of(&image[..body]);
        image[body..].copy_from_slice(digest.as_bytes());
    }
}

/// The fixed test signing key.
#[must_use]
pub fn signing_key() -> SigningKey {
    SigningKey::from_bytes(&[42; 32])
}

/// A header signed with [`signing_key`].
#[must_use]
pub fn signed(version: Version, image_len: ImageLen, digest: Digest) -> [u8; HEADER_LEN] {
    let signing = signing_key();
    let key_id = KeyId::of(&signing.verifying_key());
    let mut header = unsigned_header(version, image_len, digest, key_id);
    let signature = signing.sign(&header[..SIGNED_LEN]).to_bytes();
    header[SIGNED_LEN..].copy_from_slice(&signature);
    header
}

/// An authenticated manifest for `image`.
///
/// # Panics
/// If `image` is not a possible image length.
#[must_use]
pub fn manifest(version: Version, image: &[u8]) -> Manifest {
    let len = ImageLen::of(image).expect("fixture length");
    let header = signed(version, len, Digest::of(image));
    Manifest::authenticate(&header, &signing_key().verifying_key()).expect("signed fixture")
}
