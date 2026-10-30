//! The RTC watchdog that guards every critical phase.
//!
//! Its period bounds one tick of the monitor task, which feeds it: a
//! confirmation or a rollback inside that tick must finish well within
//! [`TIMEOUT`]. Each await they contain has its own shorter deadline.

use core::time::Duration;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::rtc_cntl::{Rtc, RwdtStage, RwdtStageAction};

use crate::clock::hal;

/// Watchdog period.
const TIMEOUT: Duration = Duration::from_secs(10);
/// Time the watchdog interrupt has to restart cleanly before the hardware
/// resets the system on its own.
const GRACE: Duration = Duration::from_secs(2);

/// Signalled on every feed, so an operation can wait until it is guarded.
/// It wakes one waiter only; only the holder of the update lock waits on it,
/// and there is one holder at a time.
pub(super) static ON: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Restarts through [`crate::restart::restart`] when the first stage
/// expires, so the radio is reset too.
#[esp_hal::handler(priority = esp_hal::interrupt::Priority::max())]
fn expired() {
    crate::restart::restart();
}

/// The RTC watchdog, enabled only while something critical runs.
pub(super) struct Watchdog {
    rtc: Rtc<'static>,
    enabled: bool,
}

impl Watchdog {
    /// Takes the RTC and starts guarding at once: boot is critical.
    pub(super) fn start(lpwr: esp_hal::peripherals::LPWR<'static>) -> Self {
        let mut rtc = Rtc::new(lpwr);
        rtc.set_interrupt_handler(expired);
        let mut watchdog = Self {
            rtc,
            enabled: false,
        };
        watchdog.guard();
        watchdog
    }

    /// Enables the watchdog if needed and feeds it. Stage 0 interrupts into
    /// [`expired`]; stage 1 resets the system if even that cannot run.
    pub(super) fn guard(&mut self) {
        if !self.enabled {
            let rwdt = &mut self.rtc.rwdt;
            rwdt.enable();
            rwdt.set_timeout(RwdtStage::Stage0, hal(TIMEOUT));
            rwdt.listen();
            rwdt.set_timeout(RwdtStage::Stage1, hal(GRACE));
            rwdt.set_stage_action(RwdtStage::Stage1, RwdtStageAction::ResetSystem);
            self.enabled = true;
        }
        self.rtc.rwdt.feed();
        ON.signal(());
    }

    /// Disables the watchdog outside critical phases.
    pub(super) fn release(&mut self) {
        if self.enabled {
            self.rtc.rwdt.disable();
            self.enabled = false;
        }
    }
}
