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
//! The runtime provisioning request itself is only *emitted* here; the
//! coordinator that disables track output, persists the next-boot flag and
//! reboots lives in `boot.rs`.
//!
//! # Button Behavior
//!
//! **Stop Button:**
//! - Press → cuts track power immediately, then sends `FaultEvent::StopPressed`
//!   to latch EstopLatched state
//! - Release → no action (state remains latched)
//!
//! **Resume Button:**
//! - Press + release (<2s) → sends `FaultEvent::ResumeShortPressed` → clears e-stop (if in EstopLatched)
//! - Press + release (2s..10s) → sends `FaultEvent::ResumeLongPressed` → force-clears any latched fault
//! - Press + hold (>=10s) → sends `ProvisioningRequest::Requested` → boot coordinator handles WiFi setup mode
//!
//! # Hardware
//!
//! Buttons are wired with pull-up resistors (GPIO22 and GPIO21 internal pull-up enabled).
//! Press pulls the line LOW; release lets pull-up drive it HIGH.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::gpio::{Input, InputConfig, Pull};

use crate::control_logic::{
    RESUME_PROVISIONING_PRESS_MS, ResumeButtonAction, ResumePress, classify_resume_press,
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

/// Return whether GPIO21 is held through the full provisioning window at boot.
///
/// Returns immediately with `false` when the button is not pressed at
/// decision time, so a normal boot never waits.
pub async fn wait_for_boot_provisioning_override(button: &mut Input<'static>) -> bool {
    if button.is_high() {
        return false;
    }

    Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
    if button.is_high() {
        return false;
    }

    with_timeout(
        Duration::from_millis(RESUME_PROVISIONING_PRESS_MS),
        wait_for_debounced_release(button),
    )
    .await
    .is_err()
}

async fn wait_for_debounced_press(button: &mut Input<'static>) {
    loop {
        if button.is_low() {
            Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
            if button.is_low() {
                return;
            }
        }

        button.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if button.is_low() {
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
            if button.is_high() {
                return;
            }
        }

        button.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if button.is_high() {
            return;
        }
    }
}

/// Measure the confirmed press until debounced release, then classify the
/// duration through the pure `classify_resume_press` policy.
async fn classify_resume_button_press(button: &mut Input<'static>) -> ResumePress {
    let pressed_at = Instant::now();
    wait_for_debounced_release(button).await;
    classify_resume_press(pressed_at.elapsed().as_millis())
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
        crate::track_safety::disable_track_intentionally();
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
