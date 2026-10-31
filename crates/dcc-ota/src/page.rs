//! The upload page the device serves for updates.
//!
//! It checks the package size before uploading and explains every error code
//! the device may answer with; the tests keep both in step with this crate
//! and [`dcc_ota_format`].

#[cfg(test)]
mod contract;

/// The upload page, served as-is.
pub const PAGE: &str = include_str!("page.html");
