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
use embassy_time::{Duration, Instant, with_timeout};
#[cfg(target_arch = "riscv32")]
use embedded_graphics::{
    geometry::Size,
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
#[cfg(target_arch = "riscv32")]
use ssd1306::{I2CDisplayInterface, Ssd1306Async, mode::BufferedGraphicsModeAsync, prelude::*};

/// Braille dot spinner frames — each u8 is the Unicode offset from U+2800,
/// which directly encodes the dot bitmask (2 cols x 3 rows).
#[cfg(target_arch = "riscv32")]
const THROBBER_FRAMES: [u8; 10] = [
    0x0B, // ⠋
    0x19, // ⠙
    0x39, // ⠹
    0x38, // ⠸
    0x3C, // ⠼
    0x34, // ⠴
    0x26, // ⠦
    0x27, // ⠧
    0x07, // ⠇
    0x0F, // ⠏
];

#[cfg(target_arch = "riscv32")]
const THROBBER_INTERVAL: Duration = Duration::from_millis(80);

/// Draw braille-style dot spinner at the given top-left position.
/// Each dot is a 2x2 filled rectangle in a 2-column x 3-row grid.
#[cfg(target_arch = "riscv32")]
fn draw_throbber<D: DrawTarget<Color = BinaryColor>>(target: &mut D, origin: Point, mask: u8) {
    let dot = Size::new(2, 2);
    let style = PrimitiveStyle::with_fill(BinaryColor::On);

    // Bit-to-position: column-major order matching braille encoding.
    // (row, col) — col X offset: 0 or 3, row Y offset: 0, 3, 6.
    const MAP: [(i32, i32); 6] = [
        (0, 0), // bit 0: row 0, col 0
        (3, 0), // bit 1: row 1, col 0
        (6, 0), // bit 2: row 2, col 0
        (0, 3), // bit 3: row 0, col 1
        (3, 3), // bit 4: row 1, col 1
        (6, 3), // bit 5: row 2, col 1
    ];

    for (bit, &(dy, dx)) in MAP.iter().enumerate() {
        if mask & (1 << bit) != 0 {
            let _ = Rectangle::new(Point::new(origin.x + dx, origin.y + dy), dot)
                .into_styled(style)
                .draw(target);
        }
    }
}

#[cfg(target_arch = "riscv32")]
fn apply_display_event(
    event: DisplayEvent,
    boot_step: &mut BootStep,
    ip: &mut Option<[u8; 4]>,
    state: &mut LedState,
    loco_count: &mut u8,
    fault: &mut Option<FaultCause>,
    message: &mut Option<heapless::String<21>>,
) {
    match event {
        DisplayEvent::BootProgress(s) => *boot_step = s,
        DisplayEvent::IpAssigned(addr) => *ip = Some(addr),
        DisplayEvent::SystemState(s) => *state = s,
        DisplayEvent::ActiveLocoCount(n) => *loco_count = n,
        DisplayEvent::Fault(f) => *fault = Some(f),
        DisplayEvent::FaultCleared => *fault = None,
        DisplayEvent::Message(m) => *message = Some(m),
    }
}

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
#[allow(clippy::too_many_arguments)]
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
    throbber_frame: u8,
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

    // Animated throbber next to state label when running.
    if state == LedState::Running {
        draw_throbber(
            display,
            Point::new(96, 18),
            THROBBER_FRAMES[throbber_frame as usize % THROBBER_FRAMES.len()],
        );
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
    let mut throbber_frame: u8 = 0;

    // Initial render (shows "PeripheralsInit" boot step).
    render(
        &mut display,
        boot_step,
        ip,
        state,
        loco_count,
        fault,
        &message,
        throbber_frame,
    )
    .await;

    loop {
        if state == LedState::Running {
            let mut next_throbber_tick = Instant::now() + THROBBER_INTERVAL;
            loop {
                let now = Instant::now();
                if now >= next_throbber_tick {
                    throbber_frame = (throbber_frame + 1) % THROBBER_FRAMES.len() as u8;
                    next_throbber_tick += THROBBER_INTERVAL;
                    break;
                }

                let wait = next_throbber_tick - now;
                match with_timeout(wait, receiver.receive()).await {
                    Ok(event) => {
                        apply_display_event(
                            event,
                            &mut boot_step,
                            &mut ip,
                            &mut state,
                            &mut loco_count,
                            &mut fault,
                            &mut message,
                        );
                        if state != LedState::Running {
                            throbber_frame = 0;
                            break;
                        }
                    }
                    Err(_) => {
                        throbber_frame = (throbber_frame + 1) % THROBBER_FRAMES.len() as u8;
                        next_throbber_tick += THROBBER_INTERVAL;
                        break;
                    }
                }
            }
        } else {
            throbber_frame = 0;
            let event = receiver.receive().await;
            apply_display_event(
                event,
                &mut boot_step,
                &mut ip,
                &mut state,
                &mut loco_count,
                &mut fault,
                &mut message,
            );
        }
        render(
            &mut display,
            boot_step,
            ip,
            state,
            loco_count,
            fault,
            &message,
            throbber_frame,
        )
        .await;
    }
}
