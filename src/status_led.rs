//! Status LED task and GPIO output mapping.
//!
//! # Overview
//!
//! Runs as an Embassy async task that consumes `StatusModel` updates and drives
//! GPIO14 (green LED) and GPIO15 (red LED) to indicate system state.
//!
//! LED patterns:
//! - **Booting**: Green blinking (125ms period) — brief pulses during startup
//! - **WiFi Connecting**: Red blinking (500ms period) — network connection in progress
//! - **Running**: Solid green — ready to control trains
//! - **E-Stop Active**: Solid red — operator pressed stop button
//! - **Fault Latched**: Solid red — hardware fault (track short, CV error, etc.)
//!
//! # Examples
//!
//! **Spawning the LED task (in main.rs):**
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
//! - **GPIO14** — Green LED (active high)
//! - **GPIO15** — Red LED (active high)
//!
//! Both LEDs can be driven directly from ESP32 GPIO (push-pull output, ~20mA max per pin).
//! Add 200Ω current-limiting resistors in series if using >5mA LEDs.

use crate::system_status::{LedState, StatusModel};
use embassy_time::{Duration, with_timeout};
use esp_hal::gpio::{Level, Output};

const BOOT_BLINK_MS: u64 = 125;

#[inline]
fn set_both_off(green: &mut Output<'static>, red: &mut Output<'static>) {
    green.set_low();
    red.set_low();
}

fn apply_led_output(
    state: LedState,
    blink_on: bool,
    green: &mut Output<'static>,
    red: &mut Output<'static>,
) {
    set_both_off(green, red);

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

fn blink_period_for(state: LedState) -> Option<Duration> {
    match state {
        LedState::Booting => Some(Duration::from_millis(BOOT_BLINK_MS)),
        LedState::WifiConnecting => Some(Duration::from_millis(WIFI_BLINK_MS)),
        LedState::Running | LedState::EstopActive | LedState::FaultLatched => None,
    }
}

async fn apply_status_event(
    model: &mut StatusModel,
    prev_state: &mut LedState,
    event: crate::system_status::SystemStatusEvent,
    display_sender: &embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        crate::display::DisplayEvent,
        8,
    >,
) {
    model.apply(event);
    let new_state = model.led_state();
    if new_state != *prev_state {
        *prev_state = new_state;
        let _ = display_sender.try_send(crate::display::DisplayEvent::SystemState(new_state));
    }
}

#[embassy_executor::task]
pub async fn status_led_task(
    status_receiver: embassy_sync::channel::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        crate::system_status::SystemStatusEvent,
        16,
    >,
    mut green_led: Output<'static>,
    mut red_led: Output<'static>,
    display_sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        crate::display::DisplayEvent,
        8,
    >,
    ready_sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        crate::boot::BootReadyEvent,
        9,
    >,
) -> ! {
    let mut model = StatusModel::new();
    let mut blink_on = true;
    let mut prev_state = model.led_state();

    // Both LEDs start off until the first event is processed.
    set_both_off(&mut green_led, &mut red_led);
    ready_sender
        .send(crate::boot::BootReadyEvent::StatusLed)
        .await;

    loop {
        let state = model.led_state();
        apply_led_output(state, blink_on, &mut green_led, &mut red_led);

        if let Some(period) = blink_period_for(state) {
            if let Ok(event) = with_timeout(period, status_receiver.receive()).await {
                blink_on = true;
                apply_status_event(&mut model, &mut prev_state, event, &display_sender).await;
            } else {
                blink_on = !blink_on;
            }
        } else {
            let event = status_receiver.receive().await;
            blink_on = true;
            apply_status_event(&mut model, &mut prev_state, event, &display_sender).await;
        }
    }
}

/// Build an output pin configured for status LEDs (active-high).
#[must_use]
pub fn new_led_output(pin: impl esp_hal::gpio::OutputPin + 'static) -> Output<'static> {
    let output_config = esp_hal::gpio::OutputConfig::default();
    Output::new(pin, Level::Low, output_config)
}
