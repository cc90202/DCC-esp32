//! Whether the critical tasks are alive, for the trial of a new image.
//!
//! Each task reports a heartbeat, each HTTP server instance its own; the
//! trial calls the system healthy only while every heartbeat is recent and
//! the network services are up. Any late heartbeat or network loss counts
//! as an outage, so one between two trial readings still restarts the
//! healthy minute.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use dcc_ota::trial::{Beat, Health, all_alive, is_outage};

/// The tasks that report heartbeats.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Task {
    /// DCC engine, after each consumed RMT slot or idle wake-up.
    Dcc,
    /// Packet scheduler loop.
    Scheduler,
    /// Fault manager loop.
    Fault,
    /// Z21 UDP loop.
    Net,
    /// First update HTTP server instance.
    HttpA,
    /// Second update HTTP server instance.
    HttpB,
}

impl Task {
    /// Every task, one heartbeat slot each, in slot order.
    const ALL: [Self; 6] = [
        Self::Dcc,
        Self::Scheduler,
        Self::Fault,
        Self::Net,
        Self::HttpA,
        Self::HttpB,
    ];
    const COUNT: usize = Self::ALL.len();

    /// The update HTTP server instances.
    pub(crate) const HTTP: [Self; 2] = [Self::HttpA, Self::HttpB];

    fn slot(self) -> usize {
        self as usize
    }
}

const _: () = {
    let mut i = 0;
    while i < Task::COUNT {
        assert!(
            Task::ALL[i] as usize == i,
            "Task::ALL must follow slot order"
        );
        i += 1;
    }
};

/// Each task's last [`Beat`], 0 until it first beats.
static BEATS: [AtomicU32; Task::COUNT] = [const { AtomicU32::new(0) }; Task::COUNT];
static OUTAGES: AtomicU32 = AtomicU32::new(0);
static NETWORK: AtomicBool = AtomicBool::new(false);
static Z21: AtomicBool = AtomicBool::new(false);
static BOOT_COMPLETE: AtomicBool = AtomicBool::new(false);

fn now() -> Beat {
    Beat::at(crate::clock::now_ms())
}

/// Records that `task` is alive.
pub(crate) fn beat(task: Task) {
    let now = now();
    let previous = Beat::stored(BEATS[task.slot()].swap(now.get(), Ordering::Relaxed));
    if is_outage(previous, now) {
        OUTAGES.fetch_add(1, Ordering::Relaxed);
    }
}

/// Reports whether the station has a link and an address.
pub(crate) fn network_ready(ready: bool) {
    if NETWORK.swap(ready, Ordering::Relaxed) && !ready {
        OUTAGES.fetch_add(1, Ordering::Relaxed);
    }
}

/// Reports that the Z21 socket is bound.
pub(crate) fn z21_ready() {
    Z21.store(true, Ordering::Relaxed);
}

/// Reports that every critical task finished its start-up.
pub(crate) fn boot_complete() {
    BOOT_COMPLETE.store(true, Ordering::Release);
}

/// Number of outages seen since boot.
fn outages() -> u32 {
    OUTAGES.load(Ordering::Relaxed)
}

/// One reading of the system's health for the trial. Its two values are
/// read one after the other, not atomically together; the trial reads again
/// right before committing, so a change in between only delays it.
pub(crate) fn health() -> Health {
    Health {
        alive: healthy(),
        outages: outages(),
    }
}

/// Whether every critical task and network service is alive right now.
fn healthy() -> bool {
    let now = now();
    BOOT_COMPLETE.load(Ordering::Acquire)
        && NETWORK.load(Ordering::Relaxed)
        && Z21.load(Ordering::Relaxed)
        && all_alive(
            BEATS
                .iter()
                .map(|beat| Beat::stored(beat.load(Ordering::Relaxed))),
            now,
        )
}
