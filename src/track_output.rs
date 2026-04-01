//! Track output ownership boundary and empty RailCom cutout driver.
//!
//! This module is the sole owner of the physical H-bridge enable pin (`GPIO18`)
//! and the sole place where the empty cutout lifecycle is implemented.
//!
//! Current phase:
//! - normal track enable/disable is active,
//! - the scheduler decides whether a packet is allowed to open a cutout,
//! - the RMT boundary ISR executes only the physical cutout start,
//! - a hardware TIMG timer closes the cutout after a fixed duration,
//! - the remaining low-level hardening step is to replace HAL-backed `*_fast()` wrappers with more direct register-level operations if stress testing ever shows margin issues,
//! - no RailCom detector/UART logic is involved yet.

use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering};

use critical_section::with as critical_section_with;
use esp_hal::Blocking;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::handler;
use esp_hal::interrupt::{InterruptHandler, Priority};
use esp_hal::ram;
use esp_hal::timer::OneShotTimer;
use esp_hal::timer::timg;
use static_cell::StaticCell;

const CUTOUT_DURATION_US: u32 = 460;

static CUTOUT_REQUEST_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_STARTED_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_SKIPPED_DISABLED_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_ENDED_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_SCHEDULE_FAIL_COUNT: AtomicU32 = AtomicU32::new(0);

static TRACK_ENABLED: AtomicBool = AtomicBool::new(false);
static CUTOUT_ACTIVE: AtomicBool = AtomicBool::new(false);
static TRACK_OUTPUT_HW: StaticCell<TrackOutputHw> = StaticCell::new();
static TRACK_OUTPUT_HW_PTR: AtomicPtr<TrackOutputHw> = AtomicPtr::new(core::ptr::null_mut());

struct TrackOutputHw {
    enable: Output<'static>,
    cutout_timer: OneShotTimer<'static, Blocking>,
}

/// Snapshot of track-output cutout counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct TrackOutputStats {
    pub cutout_request_count: u32,
    pub cutout_started_count: u32,
    pub cutout_skipped_disabled_count: u32,
    pub cutout_ended_count: u32,
    pub cutout_schedule_fail_count: u32,
}

/// Handle used by normal runtime code.
///
/// The actual hardware state lives in a singleton so tasks and ISRs reach the
/// same physical owner without duplicating GPIO18 control.
pub struct TrackOutput;

impl TrackOutput {
    /// Initialize the singleton track-output owner.
    #[must_use]
    pub fn new(
        gpio18: esp_hal::peripherals::GPIO18<'static>,
        timer0: timg::Timer<'static>,
    ) -> Self {
        let mut cutout_timer = OneShotTimer::new(timer0);
        cutout_timer.set_interrupt_handler(InterruptHandler::new(
            cutout_timer_interrupt.handler().aligned_ptr(),
            Priority::Priority2,
        ));
        cutout_timer.listen();

        let hw = TRACK_OUTPUT_HW.init(TrackOutputHw {
            enable: Output::new(gpio18, Level::Low, OutputConfig::default()),
            cutout_timer,
        });
        let prev = TRACK_OUTPUT_HW_PTR.swap(hw as *const _ as *mut _, Ordering::AcqRel);
        assert!(prev.is_null(), "TrackOutput must be initialized only once");

        TRACK_ENABLED.store(false, Ordering::Release);
        CUTOUT_ACTIVE.store(false, Ordering::Release);

        Self
    }

    /// Applies the normal track power state.
    pub fn set_track_enabled(&mut self, enabled: bool) {
        TRACK_ENABLED.store(enabled, Ordering::Release);

        critical_section_with(|_| {
            let Some(hw) = hw_mut() else {
                return;
            };

            // During a cutout, the track must remain disabled until the timer ISR
            // decides whether to restore it.
            if !CUTOUT_ACTIVE.load(Ordering::Acquire) {
                if enabled {
                    enable_high_fast(hw);
                } else {
                    enable_low_fast(hw);
                }
            } else if !enabled {
                enable_low_fast(hw);
            }
        });
    }
}

#[inline(always)]
fn hw_mut() -> Option<&'static mut TrackOutputHw> {
    let ptr = TRACK_OUTPUT_HW_PTR.load(Ordering::Acquire);
    if ptr.is_null() {
        None
    } else {
        // SAFETY: initialized exactly once from a StaticCell and then kept for the
        // whole program lifetime. Access is serialized by execution context rules:
        // task-side accesses run inside critical sections; ISR-side accesses only
        // occur from the dedicated RMT/timer handlers.
        Some(unsafe { &mut *ptr })
    }
}

// NOTE: These `*_fast()` wrappers intentionally isolate the ISR/timer critical path.
// Today they still use esp-hal internally; if future multi-loco/WiFi stress tests show
// timing margin issues, only these wrappers should be rewritten to more direct
// register-level access. Keep policy and branching out of this layer.
#[inline(always)]
fn enable_low_fast(hw: &mut TrackOutputHw) {
    hw.enable.set_low();
}

#[inline(always)]
fn enable_high_fast(hw: &mut TrackOutputHw) {
    hw.enable.set_high();
}

#[inline(always)]
fn arm_cutout_timer_fast(hw: &mut TrackOutputHw) -> bool {
    hw.cutout_timer
        .schedule(esp_hal::time::Duration::from_micros(
            CUTOUT_DURATION_US as u64,
        ))
        .is_ok()
}

#[inline(always)]
fn clear_cutout_timer_interrupt_fast(hw: &mut TrackOutputHw) {
    hw.cutout_timer.clear_interrupt();
}

#[inline(always)]
fn stop_cutout_timer_fast(hw: &mut TrackOutputHw) {
    hw.cutout_timer.stop();
}

/// Requests a physical empty cutout from the RMT packet-boundary ISR.
///
/// This function is intentionally minimal: no policy, no mutex, only the
/// hardware transition required to start the cutout.
#[inline(always)]
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".rwtext"))]
pub fn request_cutout_from_isr() {
    CUTOUT_REQUEST_COUNT.fetch_add(1, Ordering::Relaxed);

    if !TRACK_ENABLED.load(Ordering::Acquire)
        || CUTOUT_ACTIVE.swap(true, Ordering::AcqRel)
        || hw_mut().is_none()
    {
        CUTOUT_SKIPPED_DISABLED_COUNT.fetch_add(1, Ordering::Relaxed);
        CUTOUT_ACTIVE.store(false, Ordering::Release);
        return;
    }

    let hw = hw_mut().expect("TrackOutput must be initialized before ISR use");
    enable_low_fast(hw);

    if arm_cutout_timer_fast(hw) {
        CUTOUT_STARTED_COUNT.fetch_add(1, Ordering::Relaxed);
    } else {
        CUTOUT_ACTIVE.store(false, Ordering::Release);
        if TRACK_ENABLED.load(Ordering::Acquire) {
            enable_high_fast(hw);
        } else {
            enable_low_fast(hw);
        }
        CUTOUT_SCHEDULE_FAIL_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

/// Returns a copy of the current cutout statistics.
#[must_use]
pub fn stats() -> TrackOutputStats {
    TrackOutputStats {
        cutout_request_count: CUTOUT_REQUEST_COUNT.load(Ordering::Acquire),
        cutout_started_count: CUTOUT_STARTED_COUNT.load(Ordering::Acquire),
        cutout_skipped_disabled_count: CUTOUT_SKIPPED_DISABLED_COUNT.load(Ordering::Acquire),
        cutout_ended_count: CUTOUT_ENDED_COUNT.load(Ordering::Acquire),
        cutout_schedule_fail_count: CUTOUT_SCHEDULE_FAIL_COUNT.load(Ordering::Acquire),
    }
}

#[handler(priority = Priority::Priority2)]
#[ram]
fn cutout_timer_interrupt() {
    let Some(hw) = hw_mut() else {
        return;
    };

    clear_cutout_timer_interrupt_fast(hw);
    stop_cutout_timer_fast(hw);
    CUTOUT_ACTIVE.store(false, Ordering::Release);

    if TRACK_ENABLED.load(Ordering::Acquire) {
        enable_high_fast(hw);
    } else {
        enable_low_fast(hw);
    }

    CUTOUT_ENDED_COUNT.fetch_add(1, Ordering::Relaxed);
}
