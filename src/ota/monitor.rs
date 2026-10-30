//! The monitor task: feeds the watchdog during critical phases, keeps the
//! display footer current and ticks the update runtime, which runs the
//! trial of a new image.

use core::time::Duration;

use dcc_ota::{
    Error,
    admission::Lifecycle,
    runtime::{Device as _, Track as _, Trial},
};
use embassy_time::Timer;

use super::{
    device::{Esp, obey, runtime},
    notify::{clear_once_track_on, show_recovery_until_delivered},
    watchdog::Watchdog,
};
use crate::{
    clock::{now, ticks},
    liveness,
};

/// Longest boot check: past it the watchdog is no longer fed, so a boot
/// check stuck on an await ends in a restart. A candidate is already on
/// trial by then, so that restart rolls it back.
const STARTUP_LIMIT: Duration = Duration::from_secs(60);
/// Period of the monitor loop.
const MONITOR_PERIOD: Duration = Duration::from_secs(1);

/// Bench builds may inject one failure into a trial, chosen at build time.
#[cfg(feature = "ota-fault-inject")]
mod fault {
    /// The failure `OTA_FAULT` selects.
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub(super) enum Fault {
        None,
        Panic,
        Hang,
        HealthTimeout,
    }

    pub(super) const SELECTED: Fault = match option_env!("OTA_FAULT") {
        None => Fault::None,
        Some(name) => match name.as_bytes() {
            b"none" => Fault::None,
            b"panic" => Fault::Panic,
            b"hang" => Fault::Hang,
            b"health-timeout" => Fault::HealthTimeout,
            _ => panic!("OTA_FAULT must be none, panic, hang or health-timeout"),
        },
    };
}

/// Called once every critical task is ready: arms the track unless a trial
/// or recovery keeps it off.
pub(crate) async fn complete_boot() {
    #[cfg(feature = "ota-fault-inject")]
    #[expect(
        clippy::panic,
        reason = "the injected fault is a deliberate panic during a trial boot"
    )]
    if super::on_trial() {
        match fault::SELECTED {
            fault::Fault::Panic => panic!("OTA bench injected panic"),
            fault::Fault::Hang => loop {
                core::hint::spin_loop();
            },
            // Never report boot complete, so the trial cannot be healthy.
            fault::Fault::HealthTimeout => return,
            fault::Fault::None => {}
        }
    }
    liveness::boot_complete();
    runtime().arm_after_boot().await;
}

/// Leaves the trial without blaming the candidate and restarts into the
/// confirmed image: the operator chose Wi-Fi setup.
///
/// # Errors
/// Returns [`Error::Forbidden`] outside a trial, or the restore error.
pub(crate) async fn withdraw_trial() -> Result<(), Error> {
    obey(runtime().withdraw_trial().await?);
    Ok(())
}

#[embassy_executor::task]
pub(super) async fn monitor_task(lpwr: esp_hal::peripherals::LPWR<'static>) -> ! {
    let mut watchdog = Watchdog::start(lpwr);
    let mut trial = Trial::default();
    loop {
        let (lifecycle, busy) = Esp.state(|s| (s.lifecycle(), s.busy()));
        let stuck_starting =
            lifecycle == Lifecycle::Starting && now().since_boot() >= STARTUP_LIMIT;
        if stuck_starting {
            defmt::error!("OTA: boot check stuck; letting the watchdog restart");
        } else if matches!(lifecycle, Lifecycle::Starting | Lifecycle::Trial) || busy {
            watchdog.guard();
        } else {
            watchdog.release();
        }
        if lifecycle == Lifecycle::Recovery {
            show_recovery_until_delivered();
        }
        clear_once_track_on(Esp.is_on());
        obey(runtime().tick(&mut trial).await);
        Timer::after(ticks(MONITOR_PERIOD)).await;
    }
}
