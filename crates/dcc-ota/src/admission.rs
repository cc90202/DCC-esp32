//! Who may start an update or the Wi-Fi setup, and when.
//!
//! The runtime evaluates and takes the lock inside one critical section, so
//! two requests can never both be admitted.

use crate::{Refusal, uptime::Uptime};

/// Where the update service is in its life since boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lifecycle {
    /// The boot check has not finished.
    Starting,
    /// A new image is on trial.
    Trial,
    /// Normal operation, before boot has armed the track power.
    Running,
    /// Normal operation with the track power armed: updates may start.
    Armed,
    /// Update state is unusable; only a USB flash repairs it.
    Recovery,
}

/// A step that moves the service along its life.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// The boot check started a candidate's trial.
    TrialBoot,
    /// The boot check runs the confirmed image.
    SettledBoot,
    /// The candidate on trial is confirmed.
    Confirm,
    /// Boot armed the track power.
    Arming,
}

impl Lifecycle {
    /// Where `step` leads. Each step moves only its own source state, so
    /// nothing ever leaves recovery.
    #[must_use]
    pub const fn after(self, step: Step) -> Self {
        match (self, step) {
            (Self::Starting, Step::TrialBoot) => Self::Trial,
            (Self::Starting, Step::SettledBoot) | (Self::Trial, Step::Confirm) => Self::Running,
            (Self::Running, Step::Arming) => Self::Armed,
            (other, _) => other,
        }
    }

    /// Normal operation, armed or not.
    #[must_use]
    pub const fn is_running(self) -> bool {
        matches!(self, Self::Running | Self::Armed)
    }
}

/// Operation asking for the update lock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Operation {
    /// An image transfer, or its pre-upload check.
    Update,
    /// Saving the Wi-Fi setup flag before a restart.
    Provisioning,
}

/// Everything the admission decision depends on, read at one instant.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Admission {
    /// Service state.
    pub lifecycle: Lifecycle,
    /// Another operation holds the lock.
    pub busy: bool,
    /// The track is powered.
    pub track_on: bool,
    /// No operation before this time, after a failed transfer.
    pub backoff_until: Uptime,
    /// No update before this time, after the last one started.
    pub next_update: Uptime,
}

impl Admission {
    /// Admits `operation` at `now`, or says why not.
    ///
    /// # Errors
    /// In order of precedence: [`Refusal::Recovery`],
    /// [`Refusal::PendingVerify`] during a trial, [`Refusal::BootNotReady`]
    /// before arming, [`Refusal::Busy`] while locked or backing off, and for
    /// an update [`Refusal::TrackOn`] while the track is powered.
    pub fn check(&self, now: Uptime, operation: Operation) -> Result<(), Refusal> {
        match self.lifecycle {
            Lifecycle::Recovery => return Err(Refusal::Recovery),
            Lifecycle::Trial => return Err(Refusal::PendingVerify),
            Lifecycle::Starting | Lifecycle::Running => return Err(Refusal::BootNotReady),
            Lifecycle::Armed => {}
        }
        if self.busy || now < self.backoff_until {
            return Err(Refusal::Busy);
        }
        if operation == Operation::Update {
            if now < self.next_update {
                return Err(Refusal::Busy);
            }
            if self.track_on {
                return Err(Refusal::TrackOn);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::time::Duration;

    #[test]
    fn precedence_and_the_update_only_rules() {
        let secs = |s| Uptime::new(Duration::from_secs(s));
        let mut admission = Admission {
            lifecycle: Lifecycle::Recovery,
            busy: true,
            track_on: true,
            backoff_until: secs(1),
            next_update: secs(2),
        };
        for (lifecycle, expected) in [
            (Lifecycle::Recovery, Refusal::Recovery),
            (Lifecycle::Trial, Refusal::PendingVerify),
            (Lifecycle::Starting, Refusal::BootNotReady),
            (Lifecycle::Running, Refusal::BootNotReady),
            (Lifecycle::Armed, Refusal::Busy),
        ] {
            admission.lifecycle = lifecycle;
            for operation in [Operation::Update, Operation::Provisioning] {
                assert_eq!(admission.check(secs(3), operation), Err(expected));
            }
        }
        admission.busy = false;
        let just_before = |at: Uptime| {
            let earlier = at.since_boot().checked_sub(Duration::from_millis(1));
            Uptime::new(earlier.expect("every probed instant is after boot"))
        };
        for operation in [Operation::Update, Operation::Provisioning] {
            assert_eq!(
                admission.check(just_before(secs(1)), operation),
                Err(Refusal::Busy)
            );
        }
        assert_eq!(admission.check(secs(1), Operation::Provisioning), Ok(()));
        assert_eq!(
            admission.check(just_before(secs(2)), Operation::Update),
            Err(Refusal::Busy)
        );
        assert_eq!(
            admission.check(secs(2), Operation::Update),
            Err(Refusal::TrackOn)
        );
        admission.track_on = false;
        assert_eq!(admission.check(secs(2), Operation::Update), Ok(()));
    }

    #[test]
    fn recovery_is_final_and_each_step_moves_only_its_own_state() {
        let steps = [
            Step::TrialBoot,
            Step::SettledBoot,
            Step::Confirm,
            Step::Arming,
        ];
        for step in steps {
            assert_eq!(Lifecycle::Recovery.after(step), Lifecycle::Recovery);
        }
        assert_eq!(Lifecycle::Starting.after(Step::TrialBoot), Lifecycle::Trial);
        assert_eq!(Lifecycle::Trial.after(Step::SettledBoot), Lifecycle::Trial);
        assert_eq!(Lifecycle::Starting.after(Step::Arming), Lifecycle::Starting);
        assert_eq!(
            Lifecycle::Trial.after(Step::Confirm).after(Step::Arming),
            Lifecycle::Armed
        );
    }
}
