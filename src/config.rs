//! Shared runtime tuning constants.
//!
//! Keep only cross-cutting values here. Module-local timings that are tightly
//! coupled to a single implementation should stay next to that code.

/// Ignore short-detector transients during early boot and radio bring-up.
pub const TRACK_SHORT_BOOT_BLANKING_MS: u64 = 5_000;

/// Z21 client inactivity window before keepalive handling escalates.
pub const Z21_KEEPALIVE_TIMEOUT_MS: u64 = 30_000;
