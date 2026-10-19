//! The track-power handshakes with the fault manager: stop the track and
//! wait until it is fenced, arm the power and wait until it is applied.

use core::{cell::Cell, time::Duration};

use dcc_ota::Error;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

use crate::{
    clock::timed,
    fault_manager::{INTERLOCK_STOP_DONE, TRACK_POWER_ARMED},
    station_status::FaultEvent,
    system_status::channels::FaultEventSender,
};

/// Longest wait for the fault manager to apply a stop or an arming.
const ACK_TIMEOUT: Duration = Duration::from_secs(2);

/// The fault manager's queue, attached at start-up.
static FAULTS: critical_section::Mutex<Cell<Option<FaultEventSender>>> =
    critical_section::Mutex::new(Cell::new(None));

/// Connects the handshakes to the fault manager.
pub(super) fn attach(sender: FaultEventSender) {
    critical_section::with(|cs| FAULTS.borrow(cs).set(Some(sender)));
}

/// Sends `event` and waits for the fault manager to signal `done`.
///
/// A stale signal from an earlier request is cleared first; one still in
/// flight can only report the same effect, as every such request does the
/// same thing.
async fn send_and_wait(
    event: FaultEvent,
    done: &Signal<CriticalSectionRawMutex, ()>,
) -> Result<(), Error> {
    let sender = critical_section::with(|cs| FAULTS.borrow(cs).get()).ok_or(Error::Unavailable)?;
    done.reset();
    Ok(timed(ACK_TIMEOUT, async {
        sender.send(event).await;
        done.wait().await;
    })
    .await?)
}

/// Stops the track and waits until every frame queued before the stop has
/// left the engine.
///
/// # Errors
/// Returns [`Error::Timeout`] if the fault manager does not confirm in time.
pub(super) async fn stop_and_wait() -> Result<(), Error> {
    crate::track_authority::invalidate_and_disable();
    send_and_wait(FaultEvent::StopPressed, &INTERLOCK_STOP_DONE).await
}

/// Arms track power and waits until it is applied.
///
/// # Errors
/// Returns [`Error::Timeout`] if the fault manager does not confirm in time.
pub(super) async fn arm() -> Result<(), Error> {
    send_and_wait(FaultEvent::TrackPowerArmed, &TRACK_POWER_ARMED).await
}
