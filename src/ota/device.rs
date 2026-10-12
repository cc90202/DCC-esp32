//! The ESP32-C6 as the update runtime's [`Device`]: flash, clock, track,
//! display, watchdog and the state every update task shares.

use core::{cell::RefCell, time::Duration};

use dcc_ota::{
    Error,
    admission::Lifecycle,
    notice::Notice,
    runtime::{Clock, Device, Event, Feedback, Restart, Runtime, State, Storage, Track},
    status::JSON_CAPACITY,
    trial::Health,
    uptime::Uptime,
};
use dcc_ota_format::{Version, image::BuildKind};
use ed25519_dalek::VerifyingKey;
use embassy_time::Timer;

use super::{interlock, key, notify, watchdog};
use crate::{
    clock::{ticks, timed},
    flash::{self, Flash},
    liveness,
    track_output::{TrackOutput, set_inhibited},
};

/// Version of this build, checked at compile time.
const RUNNING_VERSION: Version = match Version::parse(env!("CARGO_PKG_VERSION")) {
    Ok(version) => version,
    Err(_) => panic!("CARGO_PKG_VERSION must be major.minor.patch"),
};
/// Longest wait for the watchdog to guard an operation.
const WATCHDOG_WAIT: Duration = Duration::from_secs(2);

/// The device; its state lives in statics, so one value serves every task.
pub(crate) struct Esp;

static STATE: critical_section::Mutex<RefCell<State>> =
    critical_section::Mutex::new(RefCell::new(State::new()));

/// The update runtime on this device.
pub(super) fn runtime() -> Runtime<'static, Esp> {
    Runtime::new(&Esp)
}

/// Where the update service is in its life since boot.
pub(super) fn lifecycle() -> Lifecycle {
    Esp.state(|s| s.lifecycle())
}

/// A new image is on trial.
pub(crate) fn on_trial() -> bool {
    lifecycle() == Lifecycle::Trial
}

/// Update state is unusable; only a USB flash repairs it.
pub(crate) fn in_recovery() -> bool {
    lifecycle() == Lifecycle::Recovery
}

/// Renders `GET /update/status`.
///
/// # Errors
/// Returns [`Error::Internal`] if the document overflows its buffer.
pub(crate) fn status_json() -> Result<heapless::String<JSON_CAPACITY>, Error> {
    runtime().status()
}

/// Restarts the chip when a step of the runtime asks for it.
pub(super) fn obey(restart: Option<Restart>) {
    if restart.is_some() {
        crate::restart::restart();
    }
}

/// Stops every update path for good: the track stays off, the flash is no
/// longer written, and the display asks for a USB flash.
pub(crate) fn enter_recovery(error: Error) {
    runtime().enter_recovery(error);
}

impl Clock for Esp {
    fn now(&self) -> Uptime {
        crate::clock::now()
    }

    async fn pause(&self, duration: Duration) {
        Timer::after(ticks(duration)).await;
    }
}

impl Track for Esp {
    fn is_on(&self) -> bool {
        TrackOutput.is_track_enabled()
    }

    fn inhibit(&self, inhibited: bool) {
        set_inhibited(inhibited);
    }

    async fn stop(&self) -> Result<(), Error> {
        interlock::stop_and_wait().await
    }

    async fn arm(&self) -> Result<(), Error> {
        interlock::arm().await
    }
}

impl Storage for Esp {
    type Flash = Flash;

    async fn flash<R>(&self, step: impl FnOnce(&mut Flash) -> R) -> Option<R> {
        let mut flash = flash::lock().await?;
        Some(step(&mut flash))
    }
}

impl Feedback for Esp {
    fn show(&self, notice: Notice) {
        notify::notify(notice);
    }

    fn report(&self, event: Event) {
        report(event);
    }
}

impl Device for Esp {
    fn version(&self) -> Version {
        RUNNING_VERSION
    }

    fn build_kind(&self) -> BuildKind {
        key::BUILD_KIND
    }

    fn state<R>(&self, change: impl FnOnce(&mut State) -> R) -> R {
        critical_section::with(|cs| change(&mut STATE.borrow(cs).borrow_mut()))
    }

    fn health(&self) -> Health {
        liveness::health()
    }

    fn trusted_key(&self) -> Result<VerifyingKey, Error> {
        key::trusted_key()
    }

    async fn guarded(&self) -> Result<(), Error> {
        watchdog::ON.reset();
        Ok(timed(WATCHDOG_WAIT, watchdog::ON.wait()).await?)
    }

    fn recovery_entered(&self, error: Error) {
        crate::track_authority::invalidate_and_disable();
        defmt::error!(
            "OTA recovery: {}; track disabled, restore via USB",
            defmt::Display2Format(&error)
        );
        notify::show_recovery_until_delivered();
    }
}

/// Logs one event of the update runtime.
fn report(event: Event) {
    match event {
        Event::CandidateFailedBootCheck(error) => defmt::error!(
            "OTA: new image failed its boot check ({}); rolling back",
            error.code()
        ),
        Event::ArmingRetried(error) => defmt::warn!(
            "boot: track arming not confirmed, retrying: {}",
            error.code()
        ),
        Event::ConfirmPostponed(error) => {
            defmt::warn!("OTA confirm postponed: {}", error.code());
        }
        Event::ConfirmRetried(error) => {
            defmt::warn!("OTA confirm not saved, retrying: {}", error.code());
        }
        Event::ConfirmedRestarting(error) => {
            defmt::warn!("OTA confirmed; restarting to finish ({})", error.code());
        }
        Event::TransferAbandoned => {
            defmt::error!("OTA: transfer abandoned; restarting to clean up");
        }
    }
}
