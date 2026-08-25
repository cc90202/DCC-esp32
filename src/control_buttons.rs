//! Physical control button tasks with interrupt-backed edge wait and software debounce.
//!
//! Two Embassy async tasks, one per physical button on the ESP32-C6. The stop
//! button on GPIO22 triggers an emergency stop and latches the e-stop state.
//! The resume button on GPIO21 does three different things depending on how
//! long it is held: a short press clears the e-stop, a long press force-clears
//! any latched fault, and a longer hold requests runtime WiFi setup.
//!
//! Both tasks debounce press and release with 30ms of hysteresis, classify the
//! resume press into its three bands (under 2s, 2s to 10s, 10s or more), and
//! talk to the fault manager over a channel.
//!
//! The runtime provisioning request itself is only *emitted* here; the
//! coordinator that disables track output, persists the next-boot flag and
//! reboots lives in `boot.rs`.
//!
//! # Button behaviour
//!
//! Pressing stop cuts track power immediately, then sends
//! `FaultEvent::StopPressed` to latch the `EstopLatched` state. Releasing it
//! does nothing: the state stays latched.
//!
//! Resume is classified on release, or on the 10 second timeout:
//!
//! - under 2s: `FaultEvent::ResumeShortPressed`, which clears the e-stop if one
//!   is latched
//! - 2s to 10s: `FaultEvent::ResumeLongPressed`, which force-clears any latched
//!   fault
//! - 10s or more: `ProvisioningRequest::Requested`, which the boot coordinator
//!   turns into WiFi setup mode
//!
//! # Hardware
//!
//! Buttons are wired with pull-up resistors (GPIO22 and GPIO21 internal pull-up enabled).
//! Press pulls the line LOW; release lets pull-up drive it HIGH.

use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::gpio::{Input, InputConfig, Pull};

use crate::control_logic::{
    RESUME_PROVISIONING_PRESS_MS, ResumeButtonAction, ResumePress, classify_resume_press,
    resume_action_for_press,
};
use crate::runtime_channels::{
    BootReadySender, FaultEventSender, RuntimeChannel, RuntimeSender, announce_ready,
};
use crate::system_status::{BootReadyEvent, FaultEvent};

const DEBOUNCE_MS: u64 = 30;

pub type ProvisioningRequestChannel = RuntimeChannel<ProvisioningRequest, 1>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum ProvisioningRequest {
    Requested,
}

#[derive(Clone, Copy)]
enum ButtonLevel {
    Pressed,
    Released,
}

impl ButtonLevel {
    fn is_active(self, button: &Input<'static>) -> bool {
        match self {
            Self::Pressed => button.is_low(),
            Self::Released => button.is_high(),
        }
    }
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
        wait_for_debounced_level(button, ButtonLevel::Released),
    )
    .await
    .is_err()
}

/// Cancellation-safe by construction: no state is held across `.await`
/// points, so dropping this future mid-debounce during `with_timeout`
/// simply restarts the loop on the next call. Keep it that way; adding side
/// effects here would break classification.
async fn wait_for_debounced_level(button: &mut Input<'static>, target: ButtonLevel) {
    loop {
        if target.is_active(button) {
            Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
            if target.is_active(button) {
                return;
            }
        }

        match target {
            ButtonLevel::Pressed => button.wait_for_falling_edge().await,
            ButtonLevel::Released => button.wait_for_rising_edge().await,
        }
        Timer::after(Duration::from_millis(DEBOUNCE_MS)).await;
        if target.is_active(button) {
            return;
        }
    }
}

/// Measure the confirmed press until debounced release, then classify the
/// duration through the pure `classify_resume_press` policy.
async fn classify_resume_button_press(button: &mut Input<'static>) -> ResumePress {
    let pressed_at = Instant::now();
    wait_for_debounced_level(button, ButtonLevel::Released).await;
    classify_resume_press(pressed_at.elapsed().as_millis())
}

async fn send_resume_action(
    action: ResumeButtonAction,
    fault_sender: FaultEventSender,
    provisioning_sender: RuntimeSender<ProvisioningRequest, 1>,
) {
    match action {
        ResumeButtonAction::ResumeShortFault => {
            fault_sender.send(FaultEvent::ResumeShortPressed).await;
        }
        ResumeButtonAction::ResumeLongFault => {
            fault_sender.send(FaultEvent::ResumeLongPressed).await;
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
    fault_sender: FaultEventSender,
    ready_sender: BootReadySender,
) -> ! {
    announce_ready(ready_sender, BootReadyEvent::StopButton).await;
    loop {
        wait_for_debounced_level(&mut stop_button, ButtonLevel::Pressed).await;
        defmt::info!("STOP pressed");
        crate::track_safety::disable_track_intentionally();
        fault_sender.send(FaultEvent::StopPressed).await;

        wait_for_debounced_level(&mut stop_button, ButtonLevel::Released).await;
    }
}

#[embassy_executor::task]
pub async fn resume_button_task(
    mut resume_button: Input<'static>,
    fault_sender: FaultEventSender,
    provisioning_sender: RuntimeSender<ProvisioningRequest, 1>,
    ready_sender: BootReadySender,
) -> ! {
    announce_ready(ready_sender, BootReadyEvent::ResumeButton).await;
    loop {
        wait_for_debounced_level(&mut resume_button, ButtonLevel::Pressed).await;
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
