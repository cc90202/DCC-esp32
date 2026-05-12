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
//! The signal is active-low: GPIO3=LOW means short detected. A falling-edge interrupt triggers
//! short detection, which emits `FaultEvent::FaultLatched(TrackShort)` to the fault manager.
//!
//! # Hardware Architecture
//!
//! - **GPIO3** — Active-low digital short signal from the detector
//! - **External detector** — Conditions the track-driver current/fault signal
//! - **Boot blanking** — 5 seconds after power-up (ignore spurious shorts)
//! - **Debounce** — 50ms hysteresis on digital short signal
//!
//! # Boot Sequence
//!
//! 1. Task starts, begins boot blanking (5 seconds)
//! 2. During boot blanking: GPIO3 changes are ignored
//! 3. After 5 seconds: normal operation, falling edges trigger fault detection
//! 4. On short detected: waits 50ms for debounce confirmation, then emits FaultEvent
//!
//! # State Recovery
//!
//! When the short is resolved (GPIO3 goes HIGH again):
//! - Fault remains latched (operator must press long-resume button or service clears)
//! - Waiting for manual intervention prevents repeated on/off cycling

#[cfg(target_arch = "riscv32")]
use crate::config::TRACK_SHORT_BOOT_BLANKING_MS;
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

#[cfg(target_arch = "riscv32")]
const DEBOUNCE_MS: u64 = 50;

/// Settling delay after the fault manager re-enters Normal, before re-arming
/// short detection. Lets the track driver wake up and the decoder's bulk/keep-alive
/// capacitor charge through the rails without a transient OCP pulse on FAULT
/// being interpreted as a persistent short.
#[cfg(target_arch = "riscv32")]
const RECOVERY_SETTLE_MS: u64 = 100;

/// Build the GPIO3 input pin for short-circuit detection (active-low, pull-up).
#[cfg(target_arch = "riscv32")]
#[must_use]
pub fn new_short_detect_input(gpio3: esp_hal::peripherals::GPIO3<'static>) -> Input<'static> {
    let config = InputConfig::default().with_pull(Pull::Up);
    Input::new(gpio3, config)
}

#[cfg(target_arch = "riscv32")]
async fn report_track_short(
    fault_sender: &Sender<'static, CriticalSectionRawMutex, crate::system_status::FaultEvent, 16>,
) {
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
    if initial_low {
        // A persistent LOW after blanking means the track is already faulted.
        report_track_short(&fault_sender).await;
    }

    ready_sender
        .send(crate::system_status::BootReadyEvent::ShortDetector)
        .await;

    let initial_state = if initial_low { "LOW" } else { "HIGH" };
    defmt::info!(
        "Short detector active after {}ms blanking — GPIO3 = {}",
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
            report_track_short(&fault_sender).await;
            fault_state = fault_state_receiver.changed().await;
            continue;
        }

        // Wait for the 74HC14 to signal overcurrent (falling edge).
        pin.wait_for_falling_edge().await;

        // Debounce: confirm the pin is still LOW after DEBOUNCE_MS.
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if pin.is_high() {
            continue; // Spurious spike — ignore.
        }

        report_track_short(&fault_sender).await;
        fault_state = fault_state_receiver.changed().await;
        defmt::info!("Short detector re-armed");
    }
}
