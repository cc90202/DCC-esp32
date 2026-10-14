//! Pure decision for one iteration of the fault-manager loop.
//!
//! [`FaultLoop`] owns the committed policy, the next power-fence generation
//! and an event deferred by an interrupted fence. [`FaultLoop::step`] decides
//! what one event requires (the candidate policy, whether the stop is an
//! interlock stop, and whether a power fence is needed) without touching GPIO,
//! channels or timers; the firmware task performs the side effects and then
//! commits or discards the candidate.

use super::policy::{FaultDecision, FaultPolicy};
use crate::dcc::PowerGeneration;
use crate::system_status::{FaultEvent, FaultManagerState};

/// Loop state carried between fault events.
#[derive(Debug)]
pub(super) struct FaultLoop {
    policy: FaultPolicy,
    next_generation: u32,
    pending: Option<FaultEvent>,
}

/// What one event requires from the firmware task.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct StepPlan {
    /// Policy to commit once the power transition has been applied.
    pub(super) candidate: FaultPolicy,
    /// Decision returned by the policy for this event.
    pub(super) decision: FaultDecision,
    /// A stop while the update interlock is engaged: its effects are
    /// published before the fence and its policy is already committed.
    pub(super) interlock_stop: bool,
    /// Generation of the power fence to run before applying track power.
    pub(super) fence: Option<PowerGeneration>,
}

impl FaultLoop {
    /// Starts from the initial policy with the first fence generation at 1.
    #[must_use]
    pub(super) const fn new() -> Self {
        Self {
            policy: FaultPolicy::new(),
            next_generation: 1,
            pending: None,
        }
    }

    /// Decides what `event` requires, given whether the track output is on
    /// and whether the update interlock is engaged.
    ///
    /// A fence is needed for an interlock stop, or when the decision enables
    /// a track that is off; requesting one consumes a generation, which skips
    /// zero when it wraps. An interlock stop commits its candidate here, so
    /// the stop holds even if the fence is later interrupted or times out.
    #[must_use]
    pub(super) fn step(&mut self, event: FaultEvent, track_on: bool, inhibited: bool) -> StepPlan {
        let mut candidate = self.policy;
        let decision = candidate.handle(event);
        let interlock_stop = event == FaultEvent::StopPressed && inhibited;
        if interlock_stop {
            self.policy = candidate;
        }
        let fence = (interlock_stop || (decision.track_enabled && !track_on)).then(|| {
            let generation = PowerGeneration::new(self.next_generation);
            self.next_generation = self.next_generation.wrapping_add(1).max(1);
            generation
        });
        StepPlan {
            candidate,
            decision,
            interlock_stop,
            fence,
        }
    }

    /// State of the committed policy.
    #[must_use]
    pub(super) const fn state(&self) -> FaultManagerState {
        self.policy.state()
    }

    /// Whether the committed policy wants the track powered.
    #[must_use]
    pub(super) const fn track_enabled(&self) -> bool {
        self.policy.track_enabled()
    }

    /// Commits the candidate policy of `plan`; nothing else changes.
    pub(super) fn commit(&mut self, plan: &StepPlan) {
        self.policy = plan.candidate;
    }

    /// Keeps an event that interrupted a fence for the next iteration,
    /// leaving the committed policy untouched.
    pub(super) fn defer(&mut self, interrupting: FaultEvent) {
        self.pending = Some(interrupting);
    }

    /// Takes the deferred event, if any.
    pub(super) fn take_pending(&mut self) -> Option<FaultEvent> {
        self.pending.take()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::system_status::FaultCause;

    /// A loop whose policy has been armed and committed with the track on.
    fn armed() -> FaultLoop {
        let mut fault_loop = FaultLoop::new();
        let plan = fault_loop.step(FaultEvent::TrackPowerArmed, false, false);
        fault_loop.commit(&plan);
        fault_loop
    }

    #[test]
    fn fence_plan_is_requested_only_when_track_is_off_or_interlock() {
        let mut fault_loop = armed();

        // Powering a track that is already on needs no fence.
        let plan = fault_loop.step(FaultEvent::ResumeShortPressed, true, false);
        assert_eq!(plan.fence, None);
        // A plain stop disables the track and needs no fence.
        let plan = fault_loop.step(FaultEvent::StopPressed, true, false);
        assert!(!plan.interlock_stop);
        assert_eq!(plan.fence, None);
        // Powering a track that is off fences, using the next generation.
        let plan = fault_loop.step(FaultEvent::ResumeShortPressed, false, false);
        assert_eq!(plan.fence, Some(PowerGeneration::new(2)));
        // An interlock stop fences even though it disables the track.
        let plan = fault_loop.step(FaultEvent::StopPressed, false, true);
        assert!(plan.interlock_stop);
        assert!(!plan.decision.track_enabled);
        assert_eq!(plan.fence, Some(PowerGeneration::new(3)));
    }

    #[test]
    fn interlock_stop_commits_in_step_itself() {
        let mut fault_loop = armed();

        let plain = fault_loop.step(FaultEvent::StopPressed, true, false);
        assert_eq!(plain.decision.state, FaultManagerState::EstopLatched);
        assert_eq!(fault_loop.state(), FaultManagerState::Normal);

        let interlock = fault_loop.step(FaultEvent::StopPressed, true, true);
        assert!(interlock.interlock_stop);
        assert_eq!(fault_loop.state(), FaultManagerState::EstopLatched);
        assert!(!fault_loop.track_enabled());
    }

    #[test]
    fn interrupted_fence_defer_leaves_policy_untouched_and_stores_pending() {
        let mut fault_loop = armed();
        let stop = fault_loop.step(FaultEvent::StopPressed, true, false);
        fault_loop.commit(&stop);

        let resume = fault_loop.step(FaultEvent::ResumeShortPressed, false, false);
        assert!(resume.fence.is_some());
        let short = FaultEvent::FaultLatched(FaultCause::TrackShort);
        fault_loop.defer(short);

        assert_eq!(fault_loop.state(), FaultManagerState::EstopLatched);
        assert_eq!(fault_loop.take_pending(), Some(short));
        assert_eq!(fault_loop.take_pending(), None);
    }

    #[test]
    fn commit_sets_the_policy_only() {
        let mut fault_loop = FaultLoop::new();
        let plan = fault_loop.step(FaultEvent::TrackPowerArmed, false, false);
        assert_eq!(fault_loop.state(), FaultManagerState::Normal);
        assert!(!fault_loop.track_enabled());

        fault_loop.defer(FaultEvent::StopPressed);
        fault_loop.commit(&plan);
        assert!(fault_loop.track_enabled());
        assert_eq!(fault_loop.take_pending(), Some(FaultEvent::StopPressed));
        // Committing does not consume a fence generation.
        fault_loop.commit(&plan);
        let stop = fault_loop.step(FaultEvent::StopPressed, true, false);
        fault_loop.commit(&stop);
        let resume = fault_loop.step(FaultEvent::ResumeLongPressed, false, false);
        assert_eq!(resume.fence, Some(PowerGeneration::new(2)));
    }

    #[test]
    fn new_starts_the_generation_at_one() {
        let mut fault_loop = FaultLoop::new();

        let plan = fault_loop.step(FaultEvent::TrackPowerArmed, false, false);

        assert_eq!(plan.fence, Some(PowerGeneration::new(1)));
    }

    #[test]
    fn generation_wraps_past_zero_to_one() {
        let mut fault_loop = armed();
        fault_loop.next_generation = u32::MAX;

        let last = fault_loop.step(FaultEvent::ResumeShortPressed, false, false);
        let wrapped = fault_loop.step(FaultEvent::ResumeShortPressed, false, false);

        assert_eq!(last.fence, Some(PowerGeneration::new(u32::MAX)));
        assert_eq!(wrapped.fence, Some(PowerGeneration::new(1)));
    }
}
