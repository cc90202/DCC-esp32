//! The device every package is built for.

/// Name of the chip, as tools print it.
pub const CHIP_NAME: &str = "ESP32-C6";
/// Project name both the package header and the image descriptor carry.
pub const PROJECT_NAME: &str = "dcc-esp32";
/// Chip id ESP-IDF writes for the ESP32-C6.
pub(crate) const ESP32C6_CHIP_ID: u16 = 13;
/// [`PROJECT_NAME`] as the bytes the formats store.
pub(crate) const PROJECT: &[u8] = PROJECT_NAME.as_bytes();
