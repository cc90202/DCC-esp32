//! Physical control button tasks with interrupt-backed edge wait and software debounce.
//!
//! # Overview
//!
//! Provides two Embassy async tasks for the physical control buttons on the ESP32-C6:
//! - **Stop Button (GPIO22)** — Triggers emergency stop (e-stop latch state)
//! - **Resume Button (GPIO21)** — Short press clears e-stop; long press (≥2s) force-clears any latched fault
//!
//! Both tasks implement:
//! - Debounce on press and release (30ms hysteresis)
//! - Long-press detection (resume button only)
//! - Direct integration with the fault manager via channel
//!
//! # Examples
//!
//! **Spawning the button tasks (in main.rs):**
//!
//! ```no_run
//! use esp_hal::gpio::Input;
//! use dcc_esp32::control_buttons::{stop_button_task, resume_button_task};
//!
//! // Assuming button GPIO pins were already split from `peripherals.GPIO`
//! // and configured as Input with pull-ups enabled:
//! // let stop_button: Input<'static> = ...;
//! // let resume_button: Input<'static> = ...;
//!
//! // Get the fault event channel sender from fault_manager_task setup
//! // let fault_sender = ...;
//!
//! // Spawn the tasks (returns ! — never returns)
//! // embassy_executor::task::spawn(stop_button_task(stop_button, fault_sender.clone()));
//! // embassy_executor::task::spawn(resume_button_task(resume_button, fault_sender));
//! ```
//!
//! # Button Behavior
//!
//! **Stop Button:**
//! - Press → sends `FaultEvent::StopPressed` → transitions to EstopLatched state → cuts track power
//! - Release → no action (state remains latched)
//!
//! **Resume Button:**
//! - Press + release (<2s) → sends `FaultEvent::ResumeShortPressed` → clears e-stop (if in EstopLatched)
//! - Press + hold (≥2s) → sends `FaultEvent::ResumeLongPressed` → force-clears any latched fault
//!
//! # Hardware
//!
//! Buttons are wired with pull-up resistors (GPIO22 and GPIO21 internal pull-up enabled).
//! Press pulls the line LOW; release lets pull-up drive it HIGH.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Timer, with_timeout};
use esp_hal::gpio::{Input, InputConfig, Pull};

use crate::control_logic::{
    DebouncedPress, DebouncedRelease, ResumePressKind, classify_resume_press,
    debounce_active_low_press, debounce_active_low_release,
};

const DEBOUNCE_MS: u64 = 30;
const RESUME_LONG_PRESS_MS: u64 = 2_000;

async fn wait_for_debounced_press(button: &mut Input<'static>) {
    loop {
        if button.is_low() {
            Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
            if matches!(
                debounce_active_low_press(true, button.is_low()),
                DebouncedPress::Confirmed
            ) {
                return;
            }
        }

        button.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if matches!(
            debounce_active_low_press(true, button.is_low()),
            DebouncedPress::Confirmed
        ) {
            return;
        }
    }
}

async fn wait_for_debounced_release(button: &mut Input<'static>) {
    loop {
        if button.is_high() {
            Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
            if matches!(
                debounce_active_low_release(true, button.is_high()),
                DebouncedRelease::Confirmed
            ) {
                return;
            }
        }

        button.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if matches!(
            debounce_active_low_release(true, button.is_high()),
            DebouncedRelease::Confirmed
        ) {
            return;
        }
    }
}

async fn is_long_press(button: &mut Input<'static>, threshold: Duration) -> bool {
    if with_timeout(threshold, wait_for_debounced_release(button))
        .await
        .is_ok()
    {
        matches!(
            classify_resume_press(0, RESUME_LONG_PRESS_MS),
            ResumePressKind::Short
        )
    } else {
        // Timed out while still pressed -> long press. Ensure release before returning.
        wait_for_debounced_release(button).await;
        matches!(
            classify_resume_press(RESUME_LONG_PRESS_MS, RESUME_LONG_PRESS_MS),
            ResumePressKind::Long
        )
    }
}

#[embassy_executor::task]
pub async fn stop_button_task(
    mut stop_button: Input<'static>,
    fault_sender: Sender<'static, CriticalSectionRawMutex, crate::fault_manager::FaultEvent, 16>,
    ready_sender: Sender<'static, CriticalSectionRawMutex, crate::boot::BootReadyEvent, 9>,
) -> ! {
    ready_sender
        .send(crate::boot::BootReadyEvent::StopButton)
        .await;
    loop {
        wait_for_debounced_press(&mut stop_button).await;
        defmt::info!("STOP pressed");
        fault_sender
            .send(crate::fault_manager::FaultEvent::StopPressed)
            .await;

        wait_for_debounced_release(&mut stop_button).await;
    }
}

#[embassy_executor::task]
pub async fn resume_button_task(
    mut resume_button: Input<'static>,
    fault_sender: Sender<'static, CriticalSectionRawMutex, crate::fault_manager::FaultEvent, 16>,
    ready_sender: Sender<'static, CriticalSectionRawMutex, crate::boot::BootReadyEvent, 9>,
) -> ! {
    ready_sender
        .send(crate::boot::BootReadyEvent::ResumeButton)
        .await;
    loop {
        wait_for_debounced_press(&mut resume_button).await;
        defmt::info!("RESUME pressed");

        let long_press = is_long_press(
            &mut resume_button,
            Duration::from_millis(RESUME_LONG_PRESS_MS),
        )
        .await;
        let event = if long_press {
            crate::fault_manager::FaultEvent::ResumeLongPressed
        } else {
            crate::fault_manager::FaultEvent::ResumeShortPressed
        };
        if long_press {
            defmt::info!("RESUME long press");
        } else {
            defmt::info!("RESUME short press");
        }
        fault_sender.send(event).await;
    }
}

/// Build an input pin configured for active-low button usage.
#[must_use]
pub fn new_button_input(pin: impl esp_hal::gpio::InputPin + 'static) -> Input<'static> {
    let input_config = InputConfig::default().with_pull(Pull::Up);
    Input::new(pin, input_config)
}

#[cfg(test)]
mod tests {
    use crate::control_logic::{
        DebouncedPress, DebouncedRelease, ResumePressKind, classify_resume_press,
        debounce_active_low_press, debounce_active_low_release,
    };

    #[test]
    fn test_press_debounce_helper_matches_task_expectation() {
        assert_eq!(
            debounce_active_low_press(true, true),
            DebouncedPress::Confirmed
        );
        assert_eq!(
            debounce_active_low_press(true, false),
            DebouncedPress::IgnoredBounce
        );
    }

    #[test]
    fn test_release_debounce_helper_matches_task_expectation() {
        assert_eq!(
            debounce_active_low_release(true, true),
            DebouncedRelease::Confirmed
        );
        assert_eq!(
            debounce_active_low_release(true, false),
            DebouncedRelease::IgnoredBounce
        );
    }

    #[test]
    fn test_resume_press_classification_threshold() {
        assert_eq!(classify_resume_press(1_999, 2_000), ResumePressKind::Short);
        assert_eq!(classify_resume_press(2_000, 2_000), ResumePressKind::Long);
    }
}
