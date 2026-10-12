//! Instants since boot, kept apart from the durations between them.

use core::{
    ops::{Add, Sub},
    time::Duration,
};

/// A point in time, measured from boot.
///
/// Instants add durations and subtract into durations; two instants never
/// add, so a deadline cannot be built from two timestamps by mistake.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Uptime(Duration);

impl Uptime {
    /// Boot itself.
    pub const BOOT: Self = Self(Duration::ZERO);

    /// The instant `since_boot` after boot.
    #[must_use]
    pub const fn new(since_boot: Duration) -> Self {
        Self(since_boot)
    }

    /// Time elapsed since boot.
    #[must_use]
    pub const fn since_boot(self) -> Duration {
        self.0
    }
}

impl Add<Duration> for Uptime {
    type Output = Self;

    fn add(self, duration: Duration) -> Self {
        Self(self.0.saturating_add(duration))
    }
}

impl Sub for Uptime {
    type Output = Duration;

    /// Time from `earlier` to `self`; zero if `earlier` is later.
    fn sub(self, earlier: Self) -> Duration {
        self.0.saturating_sub(earlier.0)
    }
}
