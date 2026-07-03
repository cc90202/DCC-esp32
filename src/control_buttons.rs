//! Physical control button tasks with interrupt-backed edge wait and software debounce.
//!
//! # Overview
//!
//! Provides two Embassy async tasks for the physical control buttons on the ESP32-C6:
//! - **Stop Button (GPIO22)** — Triggers emergency stop (e-stop latch state)
//! - **Resume Button (GPIO21)** — Short press clears e-stop; long press force-clears any
//!   latched fault; provisioning hold requests runtime WiFi setup
//!
//! Both tasks implement:
//! - Debounce on press and release (30ms hysteresis)
//! - Resume press classification (<2s, 2s..10s, >=10s)
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
//! // let provisioning_sender = ...;
//!
//! // Spawn the tasks (returns ! — never returns)
//! // embassy_executor::task::spawn(stop_button_task(stop_button, fault_sender, ready_sender));
//! // embassy_executor::task::spawn(resume_button_task(resume_button, fault_sender, provisioning_sender, ready_sender));
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
//! - Press + release (2s..10s) → sends `FaultEvent::ResumeLongPressed` → force-clears any latched fault
//! - Press + hold (>=10s) → sends `ProvisioningRequest::Requested` → later task handles WiFi setup mode
//!
//! # Hardware
//!
//! Buttons are wired with pull-up resistors (GPIO22 and GPIO21 internal pull-up enabled).
//! Press pulls the line LOW; release lets pull-up drive it HIGH.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Duration, Timer, with_timeout};
use esp_hal::gpio::{Input, InputConfig, Pull};

use crate::control_logic::{
    DebouncedPress, DebouncedRelease, RESUME_LONG_PRESS_MS, RESUME_PROVISIONING_PRESS_MS,
    ResumeButtonAction, ResumePress, debounce_active_low_press, debounce_active_low_release,
    resume_action_for_press,
};

const DEBOUNCE_MS: u64 = 30;

pub type ProvisioningRequestChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, ProvisioningRequest, 1>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum ProvisioningRequest {
    Requested,
}

#[embassy_executor::task]
pub async fn provisioning_request_task(
    receiver: Receiver<'static, CriticalSectionRawMutex, ProvisioningRequest, 1>,
) -> ! {
    loop {
        match receiver.receive().await {
            ProvisioningRequest::Requested => {
                defmt::warn!("WiFi provisioning requested; runtime transition not implemented yet");
            }
        }
    }
}

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

/// Cancellation-safe by construction: no state is held across `.await`
/// points, so dropping this future mid-debounce during `with_timeout`
/// simply restarts the loop on the next call. Keep it that way; adding side
/// effects here would break classification.
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

async fn classify_resume_button_press(button: &mut Input<'static>) -> ResumePress {
    if with_timeout(
        Duration::from_millis(RESUME_LONG_PRESS_MS),
        wait_for_debounced_release(button),
    )
    .await
    .is_ok()
    {
        return ResumePress::Short;
    }

    let provisioning_window_ms = RESUME_PROVISIONING_PRESS_MS - RESUME_LONG_PRESS_MS;
    if with_timeout(
        Duration::from_millis(provisioning_window_ms),
        wait_for_debounced_release(button),
    )
    .await
    .is_ok()
    {
        return ResumePress::Long;
    }

    wait_for_debounced_release(button).await;
    ResumePress::Provisioning
}

async fn send_resume_action(
    action: ResumeButtonAction,
    fault_sender: Sender<'static, CriticalSectionRawMutex, crate::system_status::FaultEvent, 16>,
    provisioning_sender: Sender<'static, CriticalSectionRawMutex, ProvisioningRequest, 1>,
) {
    match action {
        ResumeButtonAction::ResumeShortFault => {
            fault_sender
                .send(crate::system_status::FaultEvent::ResumeShortPressed)
                .await;
        }
        ResumeButtonAction::ResumeLongFault => {
            fault_sender
                .send(crate::system_status::FaultEvent::ResumeLongPressed)
                .await;
        }
        ResumeButtonAction::RequestWifiProvisioning => {
            if provisioning_sender
                .try_send(ProvisioningRequest::Requested)
                .is_err()
            {
                defmt::warn!("WiFi provisioning request already pending");
            }
        }
    }
}

#[embassy_executor::task]
pub async fn stop_button_task(
    mut stop_button: Input<'static>,
    fault_sender: Sender<'static, CriticalSectionRawMutex, crate::system_status::FaultEvent, 16>,
    ready_sender: Sender<'static, CriticalSectionRawMutex, crate::system_status::BootReadyEvent, 9>,
) -> ! {
    ready_sender
        .send(crate::system_status::BootReadyEvent::StopButton)
        .await;
    loop {
        wait_for_debounced_press(&mut stop_button).await;
        defmt::info!("STOP pressed");
        fault_sender
            .send(crate::system_status::FaultEvent::StopPressed)
            .await;

        wait_for_debounced_release(&mut stop_button).await;
    }
}

#[embassy_executor::task]
pub async fn resume_button_task(
    mut resume_button: Input<'static>,
    fault_sender: Sender<'static, CriticalSectionRawMutex, crate::system_status::FaultEvent, 16>,
    provisioning_sender: Sender<'static, CriticalSectionRawMutex, ProvisioningRequest, 1>,
    ready_sender: Sender<'static, CriticalSectionRawMutex, crate::system_status::BootReadyEvent, 9>,
) -> ! {
    ready_sender
        .send(crate::system_status::BootReadyEvent::ResumeButton)
        .await;
    loop {
        wait_for_debounced_press(&mut resume_button).await;
        defmt::info!("RESUME pressed");

        let press = classify_resume_button_press(&mut resume_button).await;
        match press {
            ResumePress::Short => defmt::info!("RESUME short press"),
            ResumePress::Long => defmt::info!("RESUME long press"),
            ResumePress::Provisioning => defmt::info!("RESUME provisioning hold"),
        }
        send_resume_action(
            resume_action_for_press(press),
            fault_sender,
            provisioning_sender,
        )
        .await;
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
        DebouncedPress, DebouncedRelease, debounce_active_low_press, debounce_active_low_release,
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
}
