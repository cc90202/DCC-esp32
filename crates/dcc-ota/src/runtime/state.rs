//! What the update service is and holds, shared by every task that runs
//! part of the update and read by the status page.

use core::time::Duration;

use crate::{
    Error,
    admission::{Lifecycle, Step},
    journal::Record,
    layout::Slot,
    status::{BootId, Stage, Transfer},
    uptime::Uptime,
};

/// The update service's shared state; the device keeps one and hands it out
/// behind a lock through [`Device::state`](super::Device::state). Only the
/// runtime changes it; the device reads it and sets the boot identifier.
#[derive(Debug, Clone, Copy)]
pub struct State {
    /// Changed only through the transition methods below.
    lifecycle: Lifecycle,
    /// Who holds the update lock.
    pub(crate) lock: Lock,
    /// The journal record as last loaded or saved.
    pub(crate) record: Option<Record>,
    /// Slot of the running image.
    pub(crate) slot: Slot,
    /// Random per-boot identifier for the status page.
    pub(crate) boot_id: BootId,
    /// The current or last operation.
    pub(crate) progress: Progress,
    /// When the next operation may start.
    pub(crate) limits: Limits,
    /// Boot could not arm the track yet; the next tick retries.
    pub(crate) arming_pending: bool,
    /// What is left of the trial's healthy minute; `None` outside a trial
    /// or while the system is not healthy.
    pub(crate) trial_left: Option<Duration>,
    /// This boot keeps the track off until the operator powers it: it is
    /// the first one after an update, or a restore happened during it.
    pub(crate) track_held: bool,
}

impl State {
    /// The state before the boot check.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            lifecycle: Lifecycle::Starting,
            lock: Lock::Free,
            record: None,
            slot: Slot::A,
            boot_id: BootId::new(0),
            progress: Progress {
                transfer: Transfer {
                    received: 0,
                    written: 0,
                    total: 0,
                },
                stage: Stage::Boot,
                reason: None,
            },
            limits: Limits {
                next_update: Uptime::BOOT,
                backoff_until: Uptime::BOOT,
            },
            arming_pending: false,
            trial_left: None,
            track_held: false,
        }
    }

    /// Where the service is in its life since boot.
    #[must_use]
    pub const fn lifecycle(&self) -> Lifecycle {
        self.lifecycle
    }

    /// Whether an operation holds the update lock.
    #[must_use]
    pub fn busy(&self) -> bool {
        self.lock != Lock::Free
    }

    /// Names this boot on the status page, so the page notices a restart.
    pub fn set_boot_id(&mut self, id: BootId) {
        self.boot_id = id;
    }

    /// Moves the service along its life; see [`Lifecycle::after`].
    pub(crate) fn advance(&mut self, step: Step) {
        self.lifecycle = self.lifecycle.after(step);
    }

    /// Stops every update path for good, recording why.
    pub(crate) fn enter_recovery(&mut self, error: Error) {
        self.lifecycle = Lifecycle::Recovery;
        self.progress.reason = Some(error);
        self.progress.stage = Stage::Recovery;
    }

    /// Whether this boot keeps the track off until the operator powers it.
    #[must_use]
    pub(crate) const fn holds_track_off(&self) -> bool {
        self.track_held
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

/// The update lock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Lock {
    /// No operation runs.
    Free,
    /// An update or setup operation runs.
    Held,
    /// A transfer was dropped mid-way: the lock stays held and the next
    /// tick restarts, which cleans up from the journal.
    Abandoned,
}

/// The current or last operation, as the status page shows it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Progress {
    /// Byte counts of the transfer.
    pub(crate) transfer: Transfer,
    /// Where the operation stands.
    pub(crate) stage: Stage,
    /// Why it failed, if it did.
    pub(crate) reason: Option<Error>,
}

/// When the next operation may start.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Limits {
    /// Earliest time of the next update, after the last one started.
    pub(crate) next_update: Uptime,
    /// Earliest time of the next operation, after a failed transfer.
    pub(crate) backoff_until: Uptime,
}
