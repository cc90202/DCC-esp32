//! How long an upload may take, so a stalled client cannot hold the
//! update lock: a deadline for the whole transfer, a minimum rate over each
//! window, and a cap on silence between two chunks.

use core::time::Duration;

use crate::{Expired, uptime::Uptime};

/// Whole upload, flash work included.
pub const UPLOAD_TIMEOUT: Duration = Duration::from_secs(300);
/// Longest pause between two received chunks.
pub(crate) const IDLE_TIMEOUT: Duration = Duration::from_secs(10);
/// Window over which the minimum rate is measured.
pub(crate) const RATE_WINDOW: Duration = Duration::from_secs(15);
/// Minimum average rate over a window, flash time included.
pub(crate) const MIN_BYTES_PER_SECOND: u64 = 1024;

/// The time budget of one upload.
#[derive(Debug, Clone, Copy)]
pub struct Pacing {
    end: Uptime,
    window_start: Uptime,
    window_bytes: u32,
}

impl Pacing {
    /// An upload that started at `start`.
    #[must_use]
    pub fn new(start: Uptime) -> Self {
        Self {
            end: start + UPLOAD_TIMEOUT,
            window_start: start,
            window_bytes: 0,
        }
    }

    /// When a reader waiting for data must wake next: the end of the upload,
    /// of the current rate window, or of the allowed silence after
    /// `last_data`.
    #[must_use]
    pub fn next_wake(&self, last_data: Uptime) -> Uptime {
        self.end
            .min(self.window_start + RATE_WINDOW)
            .min(last_data + IDLE_TIMEOUT)
    }

    /// Whether the client has been silent for too long.
    #[must_use]
    pub fn idle_expired(now: Uptime, last_data: Uptime) -> bool {
        now - last_data >= IDLE_TIMEOUT
    }

    /// Fails past the upload deadline, or below the minimum rate over a
    /// finished window, which then starts a new window. `received` counts
    /// every byte since the start.
    ///
    /// # Errors
    /// Returns [`Expired`].
    pub fn check(&mut self, now: Uptime, received: u32) -> Result<(), Expired> {
        if now >= self.end {
            return Err(Expired);
        }
        let elapsed = now - self.window_start;
        if elapsed >= RATE_WINDOW {
            // Scale the minimum with the real elapsed time if a step ran late.
            let bytes = u64::from(received - self.window_bytes);
            let needed = elapsed.as_millis() * u128::from(MIN_BYTES_PER_SECOND) / 1000;
            if u128::from(bytes) < needed {
                return Err(Expired);
            }
            self.window_start = now;
            self.window_bytes = received;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(seconds: u64) -> Uptime {
        Uptime::new(Duration::from_secs(seconds))
    }

    #[test]
    fn a_slow_window_fails_and_a_fast_one_starts_the_next() {
        let mut pacing = Pacing::new(at(0));
        assert_eq!(pacing.check(at(14), 0), Ok(()), "window not over yet");
        assert_eq!(pacing.check(at(15), 15 * 1024), Ok(()));
        assert_eq!(pacing.next_wake(at(15)), at(25), "silence ends first");
        assert_eq!(pacing.check(at(30), 15 * 1024 + 15 * 1023), Err(Expired));
    }

    #[test]
    fn the_whole_upload_has_a_deadline_and_silence_is_capped() {
        let mut pacing = Pacing::new(at(0));
        assert_eq!(pacing.check(at(300), u32::MAX), Err(Expired));
        assert!(!Pacing::idle_expired(at(9), at(0)));
        assert!(Pacing::idle_expired(at(10), at(0)));
        assert_eq!(Pacing::new(at(0)).next_wake(at(0)), at(10));
    }
}
