//! Track short-circuit detector using an active-low GPIO signal.
//!
//! GPIO3 is wired to the FAULT pin of the DRV8874 carrier. That pin is open
//! drain and can only pull down, so the line is held up by the pull-up inside
//! the ESP32-C6, enabled here.
//!
//! - Normal operation: GPIO3 = HIGH (3.3V)
//! - Short circuit:    GPIO3 = LOW  (0V)  → falling edge triggers fault
//!
//! Monitors GPIO3 for track short-circuit detection.
//! The signal is active-low: GPIO3=LOW means short detected. A falling-edge interrupt opens a
//! qualification window: the pin is sampled every millisecond and the fault latches only when
//! the line stays LOW for most of the window (15 of 20 samples), mirroring commercial command
//! stations that require an overcurrent to persist for tens of milliseconds. Motor inrush and
//! wheel micro-shorts on turnouts produce sub-millisecond pulses and are ignored; the DRV8874's
//! hardware current limit protects the output during qualification. A confirmed short emits
//! `FaultEvent::FaultLatched(TrackShort)` to the fault manager.
//!
//! For the first 5 seconds after power-up the task blanks the input entirely:
//! the power-up transient trips the detector often enough that acting on it
//! would be useless. After that, falling edges are qualified as described
//! above, and a confirmed short disables the track output through GPIO18 before
//! the fault event is even sent, which takes about 20ms in the worst case.
//!
//! When the short clears and GPIO3 goes HIGH again, the fault stays latched.
//! Recovery is deliberately manual, through the long-resume button or a service
//! action, because auto-recovery on a persistent short turns into an on/off
//! cycle that stresses the driver.

#[cfg(any(test, target_arch = "riscv32"))]
mod qualifier;

#[cfg(target_arch = "riscv32")]
use crate::station_status::FaultManagerState;
#[cfg(target_arch = "riscv32")]
use crate::system_status::channels::{BootReadySender, FaultEventSender, FaultStateReceiver};
#[cfg(target_arch = "riscv32")]
use crate::system_status::{BootReadyEvent, announce_ready};
#[cfg(target_arch = "riscv32")]
use core::cell::Cell;
#[cfg(target_arch = "riscv32")]
use embassy_futures::select::{Either, select};
#[cfg(target_arch = "riscv32")]
use embassy_time::{Duration, Timer};
#[cfg(target_arch = "riscv32")]
use esp_hal::gpio::{Input, InputConfig, Pull};
#[cfg(target_arch = "riscv32")]
use qualifier::{ShortQualifier, ShortSuppression};

#[cfg(target_arch = "riscv32")]
const TRACK_SHORT_BOOT_BLANKING_MS: u64 = 5_000;

/// Settling delay after the fault manager re-enters Normal, before re-arming
/// short detection. Lets the track driver wake up and the decoder's bulk/keep-alive
/// capacitor charge through the rails without a transient OCP pulse on FAULT
/// being interpreted as a persistent short.
#[cfg(target_arch = "riscv32")]
const RECOVERY_SETTLE_MS: u64 = 100;

#[cfg(target_arch = "riscv32")]
static SHORT_SUPPRESSION: critical_section::Mutex<Cell<ShortSuppression>> =
    critical_section::Mutex::new(Cell::new(ShortSuppression { until_ms: 0 }));

#[cfg(target_arch = "riscv32")]
pub fn suppress_short_edges_for(duration: Duration) {
    critical_section::with(|cs| {
        SHORT_SUPPRESSION
            .borrow(cs)
            .set(ShortSuppression::starting_at(
                crate::clock::now_ms(),
                duration.as_millis(),
            ));
    });
}

#[cfg(target_arch = "riscv32")]
fn short_edges_suppressed() -> bool {
    critical_section::with(|cs| {
        SHORT_SUPPRESSION
            .borrow(cs)
            .get()
            .is_active_at(crate::clock::now_ms())
    })
}

/// Interval between pin samples during short qualification.
#[cfg(target_arch = "riscv32")]
const SHORT_CONFIRM_SAMPLE_INTERVAL_MS: u64 = 1;

/// Build the GPIO3 input pin for short-circuit detection (active-low, pull-up).
#[cfg(target_arch = "riscv32")]
#[must_use]
pub fn new_short_detect_input(gpio3: esp_hal::peripherals::GPIO3<'static>) -> Input<'static> {
    let config = InputConfig::default().with_pull(Pull::Up);
    Input::new(gpio3, config)
}

/// Sample GPIO3 every [`SHORT_CONFIRM_SAMPLE_INTERVAL_MS`] and return whether
/// the short persists long enough to be treated as real. Returns as soon as
/// the verdict is decided (worst case [`SHORT_CONFIRM_SAMPLES`](qualifier::SHORT_CONFIRM_SAMPLES) ms). During
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
async fn report_track_short(fault_sender: &FaultEventSender) {
    if !crate::track_output::emergency_disable() {
        defmt::error!("track short detected before output hardware initialization");
    }
    defmt::warn!("Short circuit detected on GPIO3!");
    fault_sender
        .send(crate::station_status::FaultEvent::FaultLatched(
            crate::station_status::FaultCause::TrackShort,
        ))
        .await;
}

/// Reports a confirmed short, then waits for the fault manager to publish
/// the state it moves to and returns that state.
#[cfg(target_arch = "riscv32")]
async fn report_and_wait(
    fault_sender: &FaultEventSender,
    fault_state_receiver: &mut FaultStateReceiver,
) -> FaultManagerState {
    report_track_short(fault_sender).await;
    fault_state_receiver.changed().await
}

#[cfg(target_arch = "riscv32")]
#[embassy_executor::task]
pub async fn short_detector_task(
    mut pin: Input<'static>,
    fault_sender: FaultEventSender,
    mut fault_state_receiver: FaultStateReceiver,
    ready_sender: BootReadySender,
) -> ! {
    // Boot blanking: ignore transients from WiFi radio startup and DCC stabilization.
    Timer::after(Duration::from_millis(TRACK_SHORT_BOOT_BLANKING_MS)).await;

    let initial_low = pin.is_low();
    if initial_low && confirm_short(&pin).await {
        // A persistent LOW after blanking means the track is already faulted.
        report_track_short(&fault_sender).await;
    }

    announce_ready(ready_sender, BootReadyEvent::ShortDetector).await;

    let initial_state = if initial_low { "LOW" } else { "HIGH" };
    defmt::info!(
        "Short detector active after {}ms blanking, GPIO3 = {}",
        TRACK_SHORT_BOOT_BLANKING_MS,
        initial_state
    );

    let mut fault_state = fault_state_receiver.get().await;

    loop {
        while !matches!(fault_state, FaultManagerState::Normal) {
            fault_state = fault_state_receiver.changed().await;
        }

        Timer::after(Duration::from_millis(RECOVERY_SETTLE_MS)).await;

        if pin.is_low() {
            if short_edges_suppressed() {
                Timer::after(Duration::from_millis(10)).await;
                continue;
            }
            if confirm_short(&pin).await {
                fault_state = report_and_wait(&fault_sender, &mut fault_state_receiver).await;
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
                    fault_state = report_and_wait(&fault_sender, &mut fault_state_receiver).await;
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
