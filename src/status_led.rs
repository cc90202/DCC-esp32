//! Status LED task and GPIO output mapping.
//!
//! Runs as an Embassy async task that consumes `StatusModel` updates and drives
//! GPIO14 (green LED) and GPIO15 (red LED) to indicate system state.
//!
//! Booting shows brief green pulses on a 125ms period, and WiFi connection
//! shows red blinking on a 500ms period. While a new firmware is on trial the
//! green and red LEDs alternate every 500ms. A running station whose track is
//! off blinks green once a second; the green LED goes solid only while the
//! track is powered. Solid red means either the operator pressed the stop
//! button or a hardware fault is latched, such as a track short or a CV error.
//!
//! # Examples
//!
//! Spawning the LED task (in main.rs):
//!
//! ```no_run
//! use dcc_esp32::status_led::status_led_task;
//! use esp_hal::gpio::Output;
//!
//! // Assuming GPIO14 and GPIO15 were already split from `peripherals.GPIO`
//! // and configured as outputs:
//! // let green_led: Output<'static> = ...;
//! // let red_led: Output<'static> = ...;
//!
//! // Get the status event receiver from status_model_task setup
//! // let status_receiver = ...;
//!
//! // Spawn the LED task
//! // embassy_executor::task::spawn(status_led_task(status_receiver, green_led, red_led));
//! ```
//!
//! # Hardware
//!
//! The green LED is on GPIO14 and the red one on GPIO15, both active high.
//! Both can be driven directly from ESP32 GPIO (push-pull output, ~20mA max per pin).
//! Add 200Ω current-limiting resistors in series if using >5mA LEDs.

use crate::application::StatusModel;
use crate::system_status::announce_ready;
use crate::system_status::channels::{BootReadySender, DisplaySender, SystemStatusReceiver};
use crate::system_status::{BootReadyEvent, DisplayEvent, LedState, SystemStatusEvent};
use embassy_time::{Duration, Timer, with_timeout};
use esp_hal::gpio::{Level, Output};

const BOOT_BLINK_MS: u64 = 125;

#[inline]
fn set_both_off(green: &mut Output<'static>, red: &mut Output<'static>) {
    green.set_low();
    red.set_low();
}

/// What the LEDs show: the system state, refined while the station runs by
/// the update trial and by whether the track is actually powered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Shown {
    State(LedState),
    /// A new firmware is on trial; the track stays off until it is confirmed.
    Trial,
    /// The station runs but the track is off, waiting for the operator.
    TrackOff,
}

fn shown(state: LedState) -> Shown {
    if state != LedState::Running {
        Shown::State(state)
    } else if crate::ota::on_trial() {
        Shown::Trial
    } else if !crate::track_output::TrackOutput.is_track_enabled() {
        Shown::TrackOff
    } else {
        Shown::State(state)
    }
}

fn apply_led_output(
    shown: Shown,
    blink_on: bool,
    green: &mut Output<'static>,
    red: &mut Output<'static>,
) {
    set_both_off(green, red);

    let state = match shown {
        Shown::State(state) => state,
        Shown::Trial => {
            if blink_on {
                green.set_high();
            } else {
                red.set_high();
            }
            return;
        }
        Shown::TrackOff => {
            if blink_on {
                green.set_high();
            }
            return;
        }
    };
    match state {
        LedState::Booting => {
            if blink_on {
                green.set_high();
            }
        }
        LedState::WifiConnecting => {
            if blink_on {
                red.set_high();
            }
        }
        LedState::Running => {
            green.set_high();
        }
        LedState::EstopActive => {
            red.set_high();
        }
        LedState::FaultLatched => {
            // Fault and E-stop intentionally share the same solid-red output:
            // both states mean track output is disabled and require operator action.
            red.set_high();
        }
    }
}

const WIFI_BLINK_MS: u64 = 500;
const TRIAL_BLINK_MS: u64 = 500;
const TRACK_OFF_BLINK_MS: u64 = 1000;
/// How often a running station checks the trial and the track, which
/// change without a status event.
const RUNNING_POLL_MS: u64 = 250;

fn blink_period_for(shown: Shown) -> Option<Duration> {
    match shown {
        Shown::State(LedState::Booting) => Some(Duration::from_millis(BOOT_BLINK_MS)),
        Shown::State(LedState::WifiConnecting) => Some(Duration::from_millis(WIFI_BLINK_MS)),
        Shown::Trial => Some(Duration::from_millis(TRIAL_BLINK_MS)),
        Shown::TrackOff => Some(Duration::from_millis(TRACK_OFF_BLINK_MS)),
        Shown::State(LedState::Running) => Some(Duration::from_millis(RUNNING_POLL_MS)),
        Shown::State(LedState::EstopActive | LedState::FaultLatched) => None,
    }
}

fn apply_status_event(
    model: &mut StatusModel,
    prev_state: &mut LedState,
    event: SystemStatusEvent,
    display_sender: &DisplaySender,
) {
    model.apply(event);
    let new_state = model.led_state();
    if new_state != *prev_state {
        *prev_state = new_state;
        let _ = display_sender.try_send(DisplayEvent::SystemState(new_state));
    }
}

#[embassy_executor::task]
pub async fn status_led_task(
    status_receiver: SystemStatusReceiver,
    mut green_led: Output<'static>,
    mut red_led: Output<'static>,
    display_sender: DisplaySender,
    ready_sender: BootReadySender,
) -> ! {
    let mut model = StatusModel::new();
    let mut blink_on = true;
    let mut prev_state = model.led_state();

    // Both LEDs start off until the first event is processed.
    set_both_off(&mut green_led, &mut red_led);
    announce_ready(ready_sender, BootReadyEvent::StatusLed).await;

    let mut prev_shown = shown(model.led_state());
    loop {
        let now_shown = shown(model.led_state());
        if now_shown != prev_shown {
            prev_shown = now_shown;
            blink_on = true;
        }
        apply_led_output(now_shown, blink_on, &mut green_led, &mut red_led);

        if let Some(period) = blink_period_for(now_shown) {
            if let Ok(event) = with_timeout(period, status_receiver.receive()).await {
                blink_on = true;
                apply_status_event(&mut model, &mut prev_state, event, &display_sender);
            } else {
                blink_on = !blink_on;
            }
        } else {
            let event = status_receiver.receive().await;
            blink_on = true;
            apply_status_event(&mut model, &mut prev_state, event, &display_sender);
        }
    }
}

/// Provisioning-mode LED indication.
///
/// Reuses the WiFi-connecting pattern (red blink, 500 ms) for the whole
/// setup session. Provisioning mode has no `StatusModel` consumers, so this
/// deliberately bypasses the event-driven `status_led_task`.
#[embassy_executor::task]
pub async fn provisioning_led_task(
    mut green_led: Output<'static>,
    mut red_led: Output<'static>,
) -> ! {
    green_led.set_low();
    red_led.set_low();
    loop {
        red_led.toggle();
        Timer::after(Duration::from_millis(WIFI_BLINK_MS)).await;
    }
}

/// Build an output pin configured for status LEDs (active-high).
#[must_use]
pub fn new_led_output(pin: impl esp_hal::gpio::OutputPin + 'static) -> Output<'static> {
    let output_config = esp_hal::gpio::OutputConfig::default();
    Output::new(pin, Level::Low, output_config)
}
