//! The device every package is built for.

/// Name of the chip, as tools print it.
pub const CHIP_NAME: &str = "ESP32-C6";
/// Project name both the package header and the image descriptor carry.
pub const PROJECT_NAME: &str = "dcc-esp32";
/// Chip id ESP-IDF writes for the ESP32-C6.
pub(crate) const ESP32C6_CHIP_ID: u16 = 13;
/// Silicon revision of a chip, as ESP-IDF counts it: major × 100 + minor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ChipRevision(u16);

impl ChipRevision {
    /// Revision `major`.`minor`.
    #[must_use]
    pub const fn new(major: u8, minor: u8) -> Self {
        Self(major as u16 * 100 + minor as u16)
    }

    /// The revision as an image header stores it.
    #[must_use]
    pub const fn from_full(full: u16) -> Self {
        Self(full)
    }
}

impl core::fmt::Display for ChipRevision {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "v{}.{}", self.0 / 100, self.0 % 100)
    }
}

/// [`PROJECT_NAME`] as the bytes the formats store.
pub(crate) const PROJECT: &[u8] = PROJECT_NAME.as_bytes();
