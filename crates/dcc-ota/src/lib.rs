//! Signed over-the-air updates for the DCC command station, without a HAL.
//!
//! Everything an update decides lives here, from the bytes on the wire to the
//! records in flash; the firmware only adds tasks, locks and hardware:
//!
//! - [`image`] names the images in the two application slots and hashes
//!   them back from flash; the package and image formats themselves are
//!   [`dcc_ota_format`], shared with the host tool;
//! - [`http`] frames the update requests the device accepts;
//! - [`writer`] streams an image into the inactive slot, publishing it only
//!   after a read-back hash;
//! - [`journal`] and [`otadata`] persist the update history and the
//!   bootloader's selection so that a power cut at any byte leaves one
//!   consistent state, and [`sequence`] orders their writes for each
//!   transition;
//! - [`admission`], [`trial`], [`status`] and [`notice`] hold the policies the
//!   runtime applies: who may start an update, when a trial is confirmed, what
//!   the status page and the display show;
//! - [`runtime`] runs them in order on a [`runtime::Device`]: boot check,
//!   trial, transfer and the way back to a trusted image.
//!
//! Flash access goes through [`embedded_storage::nor_flash::NorFlash`], so
//! every power-loss path is testable on the host with a simulated flash.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod admission;
mod error;
pub mod http;
pub mod image;
pub mod journal;
pub mod layout;
pub mod notice;
mod otadata;
pub mod pacing;
pub mod runtime;
mod sequence;
pub mod status;
mod storage;
#[cfg(test)]
mod test_flash;
pub mod trial;
pub mod uptime;
pub mod writer;

pub use error::{Error, Expired, FlashError, Refusal, RequestError, StateError};
