//! Firmware side of the over-the-air update: the tasks, the locks and the
//! hardware behind [`dcc_ota::runtime`], which decides every step.
//!
//! - [`device`] is this chip as the runtime's device: flash, clock, track,
//!   display, watchdog and the shared state;
//! - [`boot`] starts the runtime and reads the partition table;
//! - [`monitor`] feeds the watchdog and ticks the trial;
//! - [`interlock`] holds the track-power handshakes with the fault manager,
//!   [`watchdog`] the RTC watchdog that covers every critical phase;
//! - [`notify`] and [`key`] hold the display lines and the trusted key.

mod boot;
mod device;
mod interlock;
mod key;
mod monitor;
mod notify;
mod watchdog;

use dcc_ota::{Error, admission::Operation};

pub(crate) use boot::start;
pub(crate) use device::{enter_recovery, in_recovery, on_trial, status_json};
pub(crate) use monitor::{complete_boot, withdraw_trial};
pub(crate) use notify::notify;

impl From<crate::clock::TimedOut> for Error {
    fn from(_: crate::clock::TimedOut) -> Self {
        Self::Timeout
    }
}

/// Holds the update lock of this device; dropping it releases the lock.
pub(crate) type UpdateGuard = dcc_ota::runtime::UpdateGuard<'static, device::Esp>;

/// Takes the update lock for `operation` and stops the track.
///
/// # Errors
/// Returns the admission refusal, or [`Error::Timeout`] when the watchdog or
/// the fault manager does not answer in time.
pub(crate) async fn acquire(operation: Operation) -> Result<UpdateGuard, Error> {
    device::runtime().acquire(operation).await
}
