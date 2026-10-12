//! The trial of a new image: confirm it after a continuous healthy minute,
//! roll it back if that never happens within three minutes of the trial's
//! start.
//!
//! Instants are [`Uptime`]s, spans [`Duration`]s. The health epoch counts
//! outages the runtime noticed between two observations, so a brief drop
//! the polling missed still restarts the healthy minute.

use core::{num::NonZeroU32, time::Duration};

use crate::uptime::Uptime;

/// Longest gap between two heartbeats of a live task.
pub(crate) const MAX_SILENCE: Duration = Duration::from_millis(2500);
/// How long the system must stay healthy without interruption.
pub(crate) const HEALTHY_SPAN: Duration = Duration::from_secs(60);
/// Time after the trial starts; past it the trial fails.
pub(crate) const DEADLINE: Duration = Duration::from_secs(180);

/// When a task last reported it is alive: milliseconds since boot,
/// wrapping. Wrapping is fine, as only recent gaps are compared.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Beat(NonZeroU32);

impl Beat {
    /// The beat at `millis` since boot, wrapped to 32 bits.
    #[must_use]
    pub fn at(millis: u64) -> Self {
        // Zero is how a stored beat says "never": move it to 1 ms.
        Self(NonZeroU32::new(millis as u32).unwrap_or(NonZeroU32::MIN))
    }

    /// The beat as stored in an atomic.
    #[must_use]
    pub const fn get(self) -> u32 {
        self.0.get()
    }

    /// A stored beat; `None` for a task that never beat.
    #[must_use]
    pub fn stored(raw: u32) -> Option<Self> {
        NonZeroU32::new(raw).map(Self)
    }

    fn since(self, earlier: Self) -> Duration {
        Duration::from_millis(u64::from(self.0.get().wrapping_sub(earlier.0.get())))
    }
}

/// Whether a task whose previous beat was `previous` and whose next one is
/// `now` was silent long enough to count as an outage.
#[must_use]
pub fn is_outage(previous: Option<Beat>, now: Beat) -> bool {
    previous.is_some_and(|previous| now.since(previous) >= MAX_SILENCE)
}

/// Whether every task has beaten within [`MAX_SILENCE`] of `now`; a task
/// that never beat is not alive.
pub fn all_alive(beats: impl IntoIterator<Item = Option<Beat>>, now: Beat) -> bool {
    beats
        .into_iter()
        .all(|last| last.is_some_and(|last| now.since(last) < MAX_SILENCE))
}

/// One reading of the system's health.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Health {
    /// Every task and network service is alive right now.
    pub alive: bool,
    /// Outages counted since boot; a change means one happened since the
    /// last reading, even if it is over.
    pub outages: u32,
}

/// What the trial calls for now.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Decision {
    /// Keep watching.
    Wait,
    /// Healthy long enough: confirm the image.
    Confirm,
    /// Deadline passed: roll the image back.
    Rollback,
}

/// Health history of one trial.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Window {
    start: Uptime,
    healthy_since: Option<Uptime>,
    outages: u32,
}

impl Window {
    /// A trial that started at `start`.
    #[must_use]
    pub const fn new(start: Uptime) -> Self {
        Self {
            start,
            healthy_since: None,
            outages: 0,
        }
    }

    /// Records one reading and decides.
    pub fn observe(&mut self, now: Uptime, health: Health) -> Decision {
        if !health.alive || health.outages != self.outages {
            self.healthy_since = None;
        }
        self.outages = health.outages;
        if health.alive {
            self.healthy_since.get_or_insert(now);
        }
        if now - self.start >= DEADLINE {
            Decision::Rollback
        } else if self.eligible(now, health) {
            Decision::Confirm
        } else {
            Decision::Wait
        }
    }

    /// How much of the healthy minute is still to go; `None` while the
    /// system is not healthy, as the minute has not started.
    #[must_use]
    pub fn left(&self, now: Uptime) -> Option<Duration> {
        self.healthy_since
            .map(|since| HEALTHY_SPAN.saturating_sub(now - since))
    }

    /// Whether confirming is still allowed. Check again right before the
    /// durable commit, after any wait the confirmation needed: an outage in
    /// between shows up as a new outage count.
    #[must_use]
    pub fn eligible(&self, now: Uptime, health: Health) -> bool {
        health.alive
            && health.outages == self.outages
            && now - self.start < DEADLINE
            && self
                .healthy_since
                .is_some_and(|since| now - since >= HEALTHY_SPAN)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ms(value: u64) -> Uptime {
        Uptime::new(Duration::from_millis(value))
    }

    fn health(alive: bool, outages: u32) -> Health {
        Health { alive, outages }
    }

    #[test]
    fn an_outage_restarts_the_healthy_minute() {
        let mut w = Window::new(ms(0));
        assert_eq!(w.observe(ms(7_000), health(true, 3)), Decision::Wait);
        assert_eq!(w.observe(ms(67_000), health(true, 3)), Decision::Confirm);
        // The link dropped and came back while confirmation waited.
        assert!(!w.eligible(ms(67_500), health(true, 4)));
        assert_eq!(w.observe(ms(68_000), health(true, 4)), Decision::Wait);
        assert_eq!(w.observe(ms(127_999), health(true, 4)), Decision::Wait);
        assert_eq!(w.observe(ms(128_000), health(true, 4)), Decision::Confirm);
        assert!(!w.eligible(ms(128_001), health(false, 4)));
    }

    #[test]
    fn the_time_left_counts_down_the_healthy_minute() {
        let mut w = Window::new(ms(0));
        w.observe(ms(1_000), health(false, 0));
        assert_eq!(w.left(ms(1_000)), None, "not healthy yet");
        w.observe(ms(2_000), health(true, 0));
        assert_eq!(w.left(ms(20_000)), Some(Duration::from_secs(42)));
        assert_eq!(w.left(ms(90_000)), Some(Duration::ZERO));
        w.observe(ms(30_000), health(true, 1));
        assert_eq!(
            w.left(ms(30_000)),
            Some(HEALTHY_SPAN),
            "an outage restarts it"
        );
    }

    #[test]
    fn the_deadline_wins_over_a_late_confirmation() {
        let mut w = Window::new(ms(1_000));
        assert_eq!(w.observe(ms(121_000), health(true, 0)), Decision::Wait);
        assert!(!w.eligible(ms(181_000), health(true, 0)));
        assert_eq!(w.observe(ms(181_000), health(true, 0)), Decision::Rollback);
        let mut w = Window::new(ms(1_000));
        w.observe(ms(120_999), health(true, 0));
        assert_eq!(w.observe(ms(180_999), health(true, 0)), Decision::Confirm);
        assert!(!w.eligible(ms(181_001), health(true, 0)));
    }

    #[test]
    fn heartbeats_survive_the_millisecond_counter_wrapping() {
        let near_wrap = Beat::at(u64::from(u32::MAX) - 1000);
        let after_wrap = Beat::at(1 << 32 | 1000);
        assert!(all_alive([Some(near_wrap)], after_wrap), "2001 ms apart");
        let later = Beat::at(1 << 32 | 1600);
        assert!(!all_alive([Some(near_wrap)], later), "2601 ms apart");
        assert!(
            !all_alive([None, Some(after_wrap)], after_wrap),
            "a task never beat"
        );
        assert!(!is_outage(None, after_wrap), "first beat");
        assert!(!is_outage(Some(near_wrap), after_wrap));
        assert!(is_outage(Some(near_wrap), later));
        assert_eq!(Beat::at(1 << 32).get(), 1, "zero never means a beat");
    }
}
