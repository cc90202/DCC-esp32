//! Continuous trial window; a loss epoch remembers outages between polls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decision {
    Wait,
    Confirm,
    Rollback,
}
pub struct Window {
    start: u64,
    since: Option<u64>,
    epoch: u32,
}
impl Window {
    pub const fn new(start: u64) -> Self {
        Self {
            start,
            since: None,
            epoch: 0,
        }
    }
    pub fn observe(&mut self, now: u64, healthy: bool, epoch: u32) -> Decision {
        if !healthy || epoch != self.epoch {
            self.since = None;
        }
        self.epoch = epoch;
        if healthy {
            self.since.get_or_insert(now);
        }
        if now.saturating_sub(self.start) >= 180000 {
            Decision::Rollback
        } else if self.eligible(now, healthy, epoch) {
            Decision::Confirm
        } else {
            Decision::Wait
        }
    }
    /// Recheck after any asynchronous confirmation prerequisite, immediately
    /// before the durable commit, not merely before waiting for Stop ACK.
    pub fn eligible(&self, now: u64, healthy: bool, epoch: u32) -> bool {
        healthy
            && epoch == self.epoch
            && now.saturating_sub(self.start) < 180000
            && self
                .since
                .is_some_and(|since| now.saturating_sub(since) >= 60000)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn brief_outage_and_confirmation_wait_cannot_preserve_old_window() {
        let mut w = Window::new(0);
        assert_eq!(w.observe(7000, true, 3), Decision::Wait);
        assert_eq!(w.observe(67000, true, 3), Decision::Confirm);
        // Link fell then recovered while Stop/fence ACK was pending.
        assert!(!w.eligible(67500, true, 4));
        assert_eq!(w.observe(68000, true, 4), Decision::Wait);
        assert_eq!(w.observe(127999, true, 4), Decision::Wait);
        assert_eq!(w.observe(128000, true, 4), Decision::Confirm);
        assert!(!w.eligible(128001, false, 4));
    }
    #[test]
    fn absolute_deadline_wins_over_confirmation_and_late_ack() {
        let mut w = Window::new(1000);
        assert_eq!(w.observe(121000, true, 0), Decision::Wait);
        assert!(!w.eligible(181000, true, 0));
        assert_eq!(w.observe(181000, true, 0), Decision::Rollback);
        let mut w = Window::new(1000);
        w.observe(120999, true, 0);
        assert_eq!(w.observe(180999, true, 0), Decision::Confirm);
        assert!(!w.eligible(181001, true, 0));
    }
}
