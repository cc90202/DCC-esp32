//! The one place where firmware time meets the HAL-free policies: `Uptime`
//! and `core::time::Duration` on one side, embassy and esp-hal types on the
//! other.

use core::time::Duration;

use dcc_ota::uptime::Uptime;
use embassy_time::{Instant, with_timeout};

/// Now, as the HAL-free policies count time.
pub(crate) fn now() -> Uptime {
    since_boot(Instant::now())
}

/// Milliseconds since boot on the embassy clock.
pub(crate) fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// An embassy instant as an [`Uptime`].
pub(crate) fn since_boot(instant: Instant) -> Uptime {
    Uptime::new(Duration::from_micros(instant.as_micros()))
}

/// An [`Uptime`] as an embassy deadline.
pub(crate) fn deadline(uptime: Uptime) -> Instant {
    Instant::from_micros(saturating_micros(uptime.since_boot()))
}

/// A policy duration for the embassy timers.
pub(crate) fn ticks(duration: Duration) -> embassy_time::Duration {
    embassy_time::Duration::from_micros(saturating_micros(duration))
}

/// A policy duration for esp-hal peripherals.
pub(crate) fn hal(duration: Duration) -> esp_hal::time::Duration {
    esp_hal::time::Duration::from_micros(saturating_micros(duration))
}

/// A wait that hit its limit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct TimedOut;

/// Runs `future` for at most `limit`.
///
/// # Errors
/// Returns [`TimedOut`] when `limit` passes first.
pub(crate) async fn timed<F: Future>(limit: Duration, future: F) -> Result<F::Output, TimedOut> {
    with_timeout(ticks(limit), future)
        .await
        .map_err(|_| TimedOut)
}

fn saturating_micros(duration: Duration) -> u64 {
    u64::try_from(duration.as_micros()).unwrap_or(u64::MAX)
}
