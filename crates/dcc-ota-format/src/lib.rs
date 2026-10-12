//! The firmware package of the DCC command station, as it travels from the
//! build machine to the device, without any flash or HAL:
//!
//! - [`Version`] and [`ImageLen`] are the release number and the image
//!   length every other format carries;
//! - [`image`] checks an ESP-IDF application image, whole or streamed, and
//!   finds the build record naming the key the image trusts next;
//! - [`package`] builds, decodes and authenticates the signed header that
//!   precedes the image.
//!
//! The host tool and the firmware run this same code, so a package the tool
//! accepts is one the device accepts.
#![cfg_attr(not(any(test, feature = "test-support")), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod coded;
mod digest;
pub mod fields;
pub mod image;
mod len;
pub mod package;
pub mod target;
#[cfg(any(test, feature = "test-support"))]
pub mod testing;
mod version;

pub use digest::{Digest, DigestStream};
pub use len::ImageLen;
pub use version::{Version, VersionError};
