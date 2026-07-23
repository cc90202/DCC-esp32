//! Track short-circuit detector using an active-low conditioned GPIO signal.
//!
//! The external current/fault detector conditions the track-driver signal into
//! a clean 3.3V digital output for GPIO3.
//!
//! - Normal operation: GPIO3 = HIGH (3.3V)
//! - Short circuit:    GPIO3 = LOW  (0V)  → falling edge triggers fault
//!
//! # Overview
//!
//! Monitors GPIO3 (digital input from 74HC14 Schmitt trigger) for track short-circuit detection.
//! The signal is active-low: GPIO3=LOW means short detected. A falling-edge interrupt opens a
//! qualification window: the pin is sampled every millisecond and the fault latches only when
//! the line stays LOW for most of the window (15 of 20 samples), mirroring commercial command
//! stations that require an overcurrent to persist for tens of milliseconds. Motor inrush and
//! wheel micro-shorts on turnouts produce sub-millisecond pulses and are ignored; the DRV8874's
//! hardware current limit protects the output during qualification. A confirmed short emits
//! `FaultEvent::FaultLatched(TrackShort)` to the fault manager.
//!
//! # Hardware Architecture
//!
//! - **GPIO3** - Active-low digital short signal from the detector
//! - **External detector** - Conditions the track-driver current/fault signal
//! - **Boot blanking** - 5 seconds after power-up (ignore spurious shorts)
//! - **Fail-safe trip** - GPIO18 is disabled once the short is confirmed (worst case ~20 ms)
//!
//! # Boot Sequence
//!
//! 1. Task starts, begins boot blanking (5 seconds)
//! 2. During boot blanking: GPIO3 changes are ignored
//! 3. After 5 seconds: normal operation, falling edges trigger fault detection
//! 4. On short detected: disables track output immediately, then emits FaultEvent
//!
//! # State Recovery
//!
//! When the short is resolved (GPIO3 goes HIGH again):
//! - Fault remains latched (operator must press long-resume button or service clears)
//! - Waiting for manual intervention prevents repeated on/off cycling

#[cfg(target_arch = "riscv32")]
use crate::config::TRACK_SHORT_BOOT_BLANKING_MS;
#[cfg(target_arch = "riscv32")]
use core::sync::atomic::{AtomicU32, Ordering};
#[cfg(target_arch = "riscv32")]
use embassy_futures::select::{Either, select};
#[cfg(target_arch = "riscv32")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(target_arch = "riscv32")]
use embassy_sync::channel::Sender;
#[cfg(target_arch = "riscv32")]
use embassy_sync::watch;
#[cfg(target_arch = "riscv32")]
use embassy_time::{Duration, Timer};
#[cfg(target_arch = "riscv32")]
use esp_hal::gpio::{Input, InputConfig, Pull};

/// Settling delay after the fault manager re-enters Normal, before re-arming
/// short detection. Lets the track driver wake up and the decoder's bulk/keep-alive
/// capacitor charge through the rails without a transient OCP pulse on FAULT
/// being interpreted as a persistent short.
#[cfg(target_arch = "riscv32")]
const RECOVERY_SETTLE_MS: u64 = 100;

#[cfg(target_arch = "riscv32")]
static SHORT_EDGE_SUPPRESSED_UNTIL_MS: AtomicU32 = AtomicU32::new(0);

#[cfg(target_arch = "riscv32")]
pub fn suppress_short_edges_for_ms(duration_ms: u64) {
    let until = embassy_time::Instant::now()
        .as_millis()
        .saturating_add(duration_ms) as u32;
    SHORT_EDGE_SUPPRESSED_UNTIL_MS.store(until, Ordering::Release);
}

#[cfg(target_arch = "riscv32")]
fn short_edges_suppressed() -> bool {
    ms_before(
        embassy_time::Instant::now().as_millis() as u32,
        SHORT_EDGE_SUPPRESSED_UNTIL_MS.load(Ordering::Acquire),
    )
}

#[cfg(any(test, target_arch = "riscv32"))]
fn ms_before(now: u32, until: u32) -> bool {
    now != until && until.wrapping_sub(now) < (1 << 31)
}

/// Total pin samples taken after a falling edge before declaring the window
/// inconclusive. With 1 ms sampling this gives a 20 ms qualification window,
/// in line with commercial DCC command stations that require an overcurrent
/// to persist for tens of milliseconds before cutting track power.
#[cfg(any(test, target_arch = "riscv32"))]
const SHORT_CONFIRM_SAMPLES: u16 = 20;

/// Minimum LOW samples within the window required to latch a short (75%).
/// Motor inrush and wheel micro-shorts on turnouts produce sub-millisecond
/// fault pulses that never accumulate this many LOW samples; a dead short
/// keeps the line LOW and confirms after 15 ms.
#[cfg(any(test, target_arch = "riscv32"))]
const SHORT_CONFIRM_MIN_LOW_SAMPLES: u16 = 15;

/// Interval between pin samples during short qualification.
#[cfg(target_arch = "riscv32")]
const SHORT_CONFIRM_SAMPLE_INTERVAL_MS: u64 = 1;

/// Integrating qualifier for the short signal: counts LOW samples after a
/// falling edge and decides as early as possible.
///
/// `record` returns `Some(true)` once enough LOW samples accumulated to latch
/// a short, `Some(false)` once so many HIGH samples were seen that the LOW
/// threshold can no longer be reached, and `None` while still undecided.
#[cfg(any(test, target_arch = "riscv32"))]
struct ShortQualifier {
    low: u16,
    high: u16,
}

#[cfg(any(test, target_arch = "riscv32"))]
impl ShortQualifier {
    const fn new() -> Self {
        Self { low: 0, high: 0 }
    }

    fn record(&mut self, is_low: bool) -> Option<bool> {
        if is_low {
            self.low += 1;
        } else {
            self.high += 1;
        }
        if self.low >= SHORT_CONFIRM_MIN_LOW_SAMPLES {
            Some(true)
        } else if self.high > SHORT_CONFIRM_SAMPLES - SHORT_CONFIRM_MIN_LOW_SAMPLES {
            Some(false)
        } else {
            None
        }
    }
}

/// Build the GPIO3 input pin for short-circuit detection (active-low, pull-up).
#[cfg(target_arch = "riscv32")]
#[must_use]
pub fn new_short_detect_input(gpio3: esp_hal::peripherals::GPIO3<'static>) -> Input<'static> {
    let config = InputConfig::default().with_pull(Pull::Up);
    Input::new(gpio3, config)
}

/// Sample GPIO3 every [`SHORT_CONFIRM_SAMPLE_INTERVAL_MS`] and return whether
/// the short persists long enough to be treated as real. Returns as soon as
/// the verdict is decided (worst case [`SHORT_CONFIRM_SAMPLES`] ms). During
/// qualification the DRV8874's hardware current limit keeps the output safe.
#[cfg(target_arch = "riscv32")]
async fn confirm_short(pin: &Input<'static>) -> bool {
    let mut qualifier = ShortQualifier::new();
    loop {
        if let Some(verdict) = qualifier.record(pin.is_low()) {
            return verdict;
        }
        Timer::after(Duration::from_millis(SHORT_CONFIRM_SAMPLE_INTERVAL_MS)).await;
    }
}

#[cfg(target_arch = "riscv32")]
async fn report_track_short(
    fault_sender: &Sender<'static, CriticalSectionRawMutex, crate::system_status::FaultEvent, 16>,
) {
    crate::track_output::emergency_disable();
    defmt::warn!("Short circuit detected on GPIO3!");
    fault_sender
        .send(crate::system_status::FaultEvent::FaultLatched(
            crate::system_status::FaultCause::TrackShort,
        ))
        .await;
}

#[cfg(target_arch = "riscv32")]
#[embassy_executor::task]
pub async fn short_detector_task(
    mut pin: Input<'static>,
    fault_sender: Sender<'static, CriticalSectionRawMutex, crate::system_status::FaultEvent, 16>,
    mut fault_state_receiver: watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        crate::fault_manager::FaultManagerState,
        1,
    >,
    ready_sender: Sender<'static, CriticalSectionRawMutex, crate::system_status::BootReadyEvent, 9>,
) -> ! {
    // Boot blanking: ignore transients from WiFi radio startup and DCC stabilization.
    Timer::after(Duration::from_millis(TRACK_SHORT_BOOT_BLANKING_MS)).await;

    let initial_low = pin.is_low();
    if initial_low && confirm_short(&pin).await {
        // A persistent LOW after blanking means the track is already faulted.
        report_track_short(&fault_sender).await;
    }

    ready_sender
        .send(crate::system_status::BootReadyEvent::ShortDetector)
        .await;

    let initial_state = if initial_low { "LOW" } else { "HIGH" };
    defmt::info!(
        "Short detector active after {}ms blanking - GPIO3 = {}",
        TRACK_SHORT_BOOT_BLANKING_MS,
        initial_state
    );

    let mut fault_state = fault_state_receiver.get().await;

    loop {
        while !matches!(fault_state, crate::fault_manager::FaultManagerState::Normal) {
            fault_state = fault_state_receiver.changed().await;
        }

        Timer::after(Duration::from_millis(RECOVERY_SETTLE_MS)).await;

        if pin.is_low() {
            if short_edges_suppressed() {
                Timer::after(Duration::from_millis(10)).await;
                continue;
            }
            if confirm_short(&pin).await {
                report_track_short(&fault_sender).await;
                fault_state = fault_state_receiver.changed().await;
                continue;
            }
            // Transient cleared during qualification: fall through and arm
            // the falling-edge wait below.
        }

        // Wait for the 74HC14 to signal overcurrent, but stop arming GPIO3 as
        // soon as the fault manager leaves Normal. Otherwise a wait started
        // before an intentional stop can survive into the next resume cycle.
        match select(pin.wait_for_falling_edge(), fault_state_receiver.changed()).await {
            Either::First(()) => {
                if short_edges_suppressed() {
                    fault_state = fault_state_receiver.get().await;
                    continue;
                }
                if confirm_short(&pin).await {
                    report_track_short(&fault_sender).await;
                    fault_state = fault_state_receiver.changed().await;
                    defmt::info!("Short detector re-armed");
                } else {
                    defmt::info!("GPIO3 transient ignored (short did not persist)");
                    fault_state = fault_state_receiver.get().await;
                }
            }
            Either::Second(next_state) => {
                fault_state = next_state;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{SHORT_CONFIRM_MIN_LOW_SAMPLES, SHORT_CONFIRM_SAMPLES, ShortQualifier, ms_before};

    #[test]
    fn suppression_window_is_active_before_deadline() {
        assert!(ms_before(100, 350));
        assert!(!ms_before(350, 350));
        assert!(!ms_before(351, 350));
    }

    #[test]
    fn suppression_window_handles_u32_wrap() {
        assert!(ms_before(u32::MAX - 10, 20));
        assert!(!ms_before(20, u32::MAX - 10));
    }

    #[test]
    fn solid_low_confirms_as_soon_as_threshold_reached() {
        let mut q = ShortQualifier::new();
        for _ in 0..SHORT_CONFIRM_MIN_LOW_SAMPLES - 1 {
            assert_eq!(q.record(true), None);
        }
        assert_eq!(q.record(true), Some(true));
    }

    #[test]
    fn brief_glitch_is_rejected_early() {
        let mut q = ShortQualifier::new();
        // A few LOW samples from the inrush pulse, then the line recovers HIGH.
        assert_eq!(q.record(true), None);
        assert_eq!(q.record(true), None);
        let max_highs = SHORT_CONFIRM_SAMPLES - SHORT_CONFIRM_MIN_LOW_SAMPLES;
        for _ in 0..max_highs {
            assert_eq!(q.record(false), None);
        }
        // One more HIGH makes reaching the LOW threshold impossible.
        assert_eq!(q.record(false), Some(false));
    }

    #[test]
    fn intermittent_short_above_duty_threshold_confirms() {
        // A bouncing dead short (3 LOW, 1 HIGH) must still confirm once
        // enough LOW samples accumulate within the window.
        let mut q = ShortQualifier::new();
        let mut verdict = None;
        let mut n = 0u16;
        while verdict.is_none() && n < 2 * SHORT_CONFIRM_SAMPLES {
            let is_low = n % 4 != 3;
            verdict = q.record(is_low);
            n += 1;
        }
        assert_eq!(verdict, Some(true));
    }

    #[test]
    fn undecided_window_returns_none() {
        let mut q = ShortQualifier::new();
        assert_eq!(q.record(true), None);
        assert_eq!(q.record(false), None);
    }
}
