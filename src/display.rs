//! OLED display event types and async task.

use crate::system_status::{FaultCause, LedState};

/// Boot sequence milestones shown on the display during initialisation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum BootStep {
    PeripheralsInit,
    WifiConnecting,
    WifiConnected,
    DccEngineReady,
    SystemRunning,
}

/// Events that update the OLED display content.
#[derive(Debug, Clone)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum DisplayEvent {
    BootProgress(BootStep),
    IpAssigned([u8; 4]),
    SystemState(LedState),
    ActiveLocoCount(u8),
    Fault(FaultCause),
    FaultCleared,
    Message(heapless::String<21>),
}

#[cfg(target_arch = "riscv32")]
pub type DisplayChannel = embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    DisplayEvent,
    8,
>;

#[cfg(target_arch = "riscv32")]
use defmt::warn;
#[cfg(target_arch = "riscv32")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(target_arch = "riscv32")]
use embassy_sync::channel::{Receiver, Sender};
#[cfg(target_arch = "riscv32")]
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
#[cfg(target_arch = "riscv32")]
use ssd1306::{I2CDisplayInterface, Ssd1306Async, mode::BufferedGraphicsModeAsync, prelude::*};

#[cfg(target_arch = "riscv32")]
fn boot_step_label(step: BootStep) -> &'static str {
    match step {
        BootStep::PeripheralsInit => "Init peripherals",
        BootStep::WifiConnecting => "WiFi connecting",
        BootStep::WifiConnected => "WiFi connected",
        BootStep::DccEngineReady => "DCC engine ready",
        BootStep::SystemRunning => "System running",
    }
}

#[cfg(target_arch = "riscv32")]
fn led_state_label(state: LedState) -> &'static str {
    match state {
        LedState::Booting => "Booting",
        LedState::WifiConnecting => "WiFi...",
        LedState::Running => "Running",
        LedState::EstopActive => "E-STOP",
        LedState::FaultLatched => "FAULT",
    }
}

#[cfg(target_arch = "riscv32")]
fn fault_label(cause: FaultCause) -> &'static str {
    match cause {
        FaultCause::CvService => "CV Service",
        FaultCause::TrackShort => "Track Short",
        FaultCause::Estop => "E-Stop",
        FaultCause::Internal => "Internal",
    }
}

/// Format an IPv4 address into a heapless string (max "255.255.255.255" = 15 chars).
#[cfg(target_arch = "riscv32")]
fn format_ip(addr: [u8; 4]) -> heapless::String<15> {
    use core::fmt::Write;
    let mut s = heapless::String::new();
    let _ = write!(s, "{}.{}.{}.{}", addr[0], addr[1], addr[2], addr[3]);
    s
}

#[cfg(target_arch = "riscv32")]
async fn render(
    display: &mut Ssd1306Async<
        I2CInterface<esp_hal::i2c::master::I2c<'static, esp_hal::Async>>,
        DisplaySize128x64,
        BufferedGraphicsModeAsync<DisplaySize128x64>,
    >,
    boot_step: BootStep,
    ip: Option<[u8; 4]>,
    state: LedState,
    loco_count: u8,
    fault: Option<FaultCause>,
    message: &Option<heapless::String<21>>,
) {
    use core::fmt::Write;

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let _ = display.clear(BinaryColor::Off);

    // Yellow zone (pixels 0-15): title
    let _ = Text::new("DCC Command Station", Point::new(0, 10), style).draw(display);

    // Blue zone (pixels 16-63): skip gap, start content at Y=26
    let mut line: heapless::String<21> = heapless::String::new();
    let _ = write!(line, "State: {}", led_state_label(state));
    let _ = Text::new(&line, Point::new(0, 26), style).draw(display);

    line.clear();
    if let Some(addr) = ip {
        let ip_str = format_ip(addr);
        let _ = write!(line, "IP: {}", ip_str);
    } else {
        let _ = write!(line, "IP: ---.---.---.---");
    }
    let _ = Text::new(&line, Point::new(0, 36), style).draw(display);

    line.clear();
    let _ = write!(line, "Locos: {}", loco_count);
    let _ = Text::new(&line, Point::new(0, 46), style).draw(display);

    if let Some(cause) = fault {
        line.clear();
        let _ = write!(line, "Fault: {}", fault_label(cause));
        let _ = Text::new(&line, Point::new(0, 56), style).draw(display);
    }

    // Last row: boot step during boot, or message after boot
    if !matches!(boot_step, BootStep::SystemRunning) {
        let _ = Text::new(boot_step_label(boot_step), Point::new(0, 63), style).draw(display);
    } else if let Some(msg) = message {
        let _ = Text::new(msg, Point::new(0, 63), style).draw(display);
    }

    let _ = display.flush().await;
}

/// Embassy task: receives display events, updates local state, redraws the OLED.
#[cfg(target_arch = "riscv32")]
pub async fn display_task(
    i2c: Option<esp_hal::i2c::master::I2c<'static, esp_hal::Async>>,
    receiver: Receiver<'static, CriticalSectionRawMutex, DisplayEvent, 8>,
    ready_sender: Sender<'static, CriticalSectionRawMutex, crate::boot::BootReadyEvent, 9>,
) -> ! {
    let Some(i2c) = i2c else {
        warn!("display: disabled, draining display events");
        ready_sender
            .send(crate::boot::BootReadyEvent::DisplayDegraded(
                crate::boot::OptionalPeripheralInit::DisplayUnavailable,
            ))
            .await;
        loop {
            let _ = receiver.receive().await;
        }
    };

    // Allow the SSD1306 to settle after power-on before sending commands.
    embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    if display.init().await.is_err() {
        warn!("display: SSD1306 init failed, draining display events");
        ready_sender
            .send(crate::boot::BootReadyEvent::DisplayDegraded(
                crate::boot::OptionalPeripheralInit::DisplayInit,
            ))
            .await;
        loop {
            let _ = receiver.receive().await;
        }
    }

    ready_sender
        .send(crate::boot::BootReadyEvent::DisplayReady)
        .await;

    let mut boot_step = BootStep::PeripheralsInit;
    let mut ip: Option<[u8; 4]> = None;
    let mut state = LedState::Booting;
    let mut loco_count: u8 = 0;
    let mut fault: Option<FaultCause> = None;
    let mut message: Option<heapless::String<21>> = None;

    // Initial render (shows "PeripheralsInit" boot step).
    render(
        &mut display,
        boot_step,
        ip,
        state,
        loco_count,
        fault,
        &message,
    )
    .await;

    loop {
        let event = receiver.receive().await;
        match event {
            DisplayEvent::BootProgress(s) => boot_step = s,
            DisplayEvent::IpAssigned(addr) => ip = Some(addr),
            DisplayEvent::SystemState(s) => state = s,
            DisplayEvent::ActiveLocoCount(n) => loco_count = n,
            DisplayEvent::Fault(f) => fault = Some(f),
            DisplayEvent::FaultCleared => fault = None,
            DisplayEvent::Message(m) => message = Some(m),
        }
        render(
            &mut display,
            boot_step,
            ip,
            state,
            loco_count,
            fault,
            &message,
        )
        .await;
    }
}
