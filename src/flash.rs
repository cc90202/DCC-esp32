//! The one handle on the SPI flash, shared by the update runtime and the
//! Wi-Fi configuration store.
//!
//! The update runtime opens it first at boot, before any flash read; every
//! later user locks it for one bounded operation and never across network
//! I/O.

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::{Mutex, MutexGuard},
    once_lock::OnceLock,
};
use esp_storage::FlashStorage;

/// The flash driver type.
pub(crate) type Flash = FlashStorage<'static>;
/// Exclusive access to the flash for one operation.
pub(crate) type FlashGuard = MutexGuard<'static, CriticalSectionRawMutex, Flash>;

static FLASH: OnceLock<Mutex<CriticalSectionRawMutex, Flash>> = OnceLock::new();

/// Opens the flash; later calls are ignored.
pub(crate) fn init(peripheral: esp_hal::peripherals::FLASH<'static>) {
    let _ = FLASH.init(Mutex::new(FlashStorage::new(peripheral)));
}

/// Locks the flash; `None` before [`init`].
pub(crate) async fn lock() -> Option<FlashGuard> {
    Some(FLASH.try_get()?.lock().await)
}
