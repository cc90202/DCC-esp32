//! OLED display event types and async task.

use crate::system_status::{BootStep, DisplayEvent, FaultCause, LedState};

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
const THROBBER_INTERVAL: Duration = Duration::from_millis(50);

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

/// Aggregated OLED display state, updated one [`DisplayEvent`] at a time.
///
/// Mirrors the reducer pattern used by [`crate::application::StatusModel`]:
/// a single struct owns all display fields so [`Self::apply`] and `render`
/// don't need to thread six independent `&mut` parameters through the task.
#[derive(Debug, Clone)]
pub struct DisplayModel {
    boot_step: BootStep,
    ip: Option<[u8; 4]>,
    provisioning_ssid: Option<heapless::String<14>>,
    provisioning_setup_url: Option<&'static str>,
    state: LedState,
    loco_count: u8,
    fault: Option<FaultCause>,
    message: Option<heapless::String<21>>,
}

impl DisplayModel {
    /// Builds a new model in the initial boot state.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            boot_step: BootStep::PeripheralsInit,
            ip: None,
            provisioning_ssid: None,
            provisioning_setup_url: None,
            state: LedState::Booting,
            loco_count: 0,
            fault: None,
            message: None,
        }
    }

    /// Applies one display event, updating the relevant field.
    pub fn apply(&mut self, event: DisplayEvent) {
        match event {
            DisplayEvent::BootProgress(s) => self.boot_step = s,
            DisplayEvent::IpAssigned(addr) => self.ip = Some(addr),
            DisplayEvent::ProvisioningMode { ssid, setup_url } => {
                self.provisioning_ssid = Some(ssid);
                self.provisioning_setup_url = Some(setup_url);
            }
            DisplayEvent::SystemState(s) => self.state = s,
            DisplayEvent::ActiveLocoCount(n) => self.loco_count = n,
            DisplayEvent::Fault(f) => self.fault = Some(f),
            DisplayEvent::FaultCleared => self.fault = None,
            DisplayEvent::Message(m) => self.message = Some(m),
        }
    }

    /// Returns the current LED-equivalent system state shown on screen.
    #[must_use]
    pub const fn state(&self) -> LedState {
        self.state
    }
}

impl Default for DisplayModel {
    fn default() -> Self {
        Self::new()
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
async fn render(
    display: &mut Ssd1306Async<
        I2CInterface<esp_hal::i2c::master::I2c<'static, esp_hal::Async>>,
        DisplaySize128x64,
        BufferedGraphicsModeAsync<DisplaySize128x64>,
    >,
    model: &DisplayModel,
    throbber_frame: u8,
) {
    use core::fmt::Write;

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let _ = display.clear(BinaryColor::Off);

    if let (Some(ssid), Some(setup_url)) = (&model.provisioning_ssid, model.provisioning_setup_url)
    {
        let _ = Text::new("DCC WiFi Setup", Point::new(0, 10), style).draw(display);

        let mut line: heapless::String<21> = heapless::String::new();
        let _ = write!(line, "AP: {}", ssid);
        let _ = Text::new(&line, Point::new(0, 26), style).draw(display);

        let _ = Text::new(setup_url, Point::new(0, 36), style).draw(display);
        let _ = Text::new("Binario spento", Point::new(0, 46), style).draw(display);

        if let Some(cause) = model.fault {
            line.clear();
            let _ = write!(line, "Fault: {}", fault_label(cause));
            let _ = Text::new(&line, Point::new(0, 56), style).draw(display);
        } else {
            let _ = Text::new("Salva e riavvia", Point::new(0, 56), style).draw(display);
        }

        let _ = display.flush().await;
        return;
    }

    // Yellow zone (pixels 0-15): title
    let _ = Text::new("DCC Command Station", Point::new(0, 10), style).draw(display);

    // Blue zone (pixels 16-63): skip gap, start content at Y=26
    let mut line: heapless::String<21> = heapless::String::new();
    let _ = write!(line, "State: {}", led_state_label(model.state));
    let _ = Text::new(&line, Point::new(0, 26), style).draw(display);

    line.clear();
    if let Some(addr) = model.ip {
        let ip_str = format_ip(addr);
        let _ = write!(line, "IP: {}", ip_str);
    } else {
        let _ = write!(line, "IP: ---.---.---.---");
    }
    let _ = Text::new(&line, Point::new(0, 36), style).draw(display);

    line.clear();
    let _ = write!(line, "Locos: {}", model.loco_count);
    let _ = Text::new(&line, Point::new(0, 46), style).draw(display);

    if let Some(cause) = model.fault {
        line.clear();
        let _ = write!(line, "Fault: {}", fault_label(cause));
        let _ = Text::new(&line, Point::new(0, 56), style).draw(display);
    }

    // Animated throbber next to state label when running.
    if model.state == LedState::Running {
        draw_throbber(
            display,
            Point::new(96, 18),
            THROBBER_FRAMES[throbber_frame as usize % THROBBER_FRAMES.len()],
        );
    }

    // Last row: boot step during boot, or message after boot
    if !matches!(model.boot_step, BootStep::SystemRunning) {
        let _ = Text::new(boot_step_label(model.boot_step), Point::new(0, 63), style).draw(display);
    } else if let Some(msg) = &model.message {
        let _ = Text::new(msg, Point::new(0, 63), style).draw(display);
    }

    let _ = display.flush().await;
}

/// Embassy task: receives display events, updates local state, redraws the OLED.
#[cfg(target_arch = "riscv32")]
pub async fn display_task(
    i2c: Option<esp_hal::i2c::master::I2c<'static, esp_hal::Async>>,
    receiver: Receiver<'static, CriticalSectionRawMutex, DisplayEvent, 8>,
    ready_sender: Sender<'static, CriticalSectionRawMutex, crate::system_status::BootReadyEvent, 9>,
) -> ! {
    let Some(i2c) = i2c else {
        warn!("display: disabled, draining display events");
        ready_sender
            .send(crate::system_status::BootReadyEvent::DisplayDegraded(
                crate::system_status::OptionalPeripheralInit::DisplayUnavailable,
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
            .send(crate::system_status::BootReadyEvent::DisplayDegraded(
                crate::system_status::OptionalPeripheralInit::DisplayInit,
            ))
            .await;
        loop {
            let _ = receiver.receive().await;
        }
    }

    ready_sender
        .send(crate::system_status::BootReadyEvent::DisplayReady)
        .await;

    let mut model = DisplayModel::new();
    let mut throbber_frame: u8 = 0;

    // Initial render (shows "PeripheralsInit" boot step).
    render(&mut display, &model, throbber_frame).await;

    loop {
        if model.state == LedState::Running {
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
                        model.apply(event);
                        if model.state != LedState::Running {
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
            model.apply(event);
        }
        render(&mut display, &model, throbber_frame).await;
    }
}

#[cfg(test)]
mod tests {
    use crate::net::provisioning::net_config::SETUP_URL;

    use super::*;

    #[test]
    fn test_new_model_starts_in_initial_boot_state() {
        let model = DisplayModel::new();
        assert_eq!(model.boot_step, BootStep::PeripheralsInit);
        assert_eq!(model.state(), LedState::Booting);
        assert_eq!(model.ip, None);
        assert!(model.provisioning_ssid.is_none());
        assert!(model.provisioning_setup_url.is_none());
        assert_eq!(model.loco_count, 0);
        assert_eq!(model.fault, None);
        assert!(model.message.is_none());
    }

    #[test]
    fn test_apply_updates_only_targeted_field() {
        let mut model = DisplayModel::new();

        model.apply(DisplayEvent::BootProgress(BootStep::WifiConnecting));
        assert_eq!(model.boot_step, BootStep::WifiConnecting);

        model.apply(DisplayEvent::IpAssigned([192, 168, 4, 1]));
        assert_eq!(model.ip, Some([192, 168, 4, 1]));

        let mut ssid: heapless::String<14> = heapless::String::new();
        ssid.push_str("DCC-Setup-ABCD")
            .expect("fits in 14-byte buffer");
        model.apply(DisplayEvent::ProvisioningMode {
            ssid,
            setup_url: SETUP_URL,
        });
        assert_eq!(model.provisioning_ssid.as_deref(), Some("DCC-Setup-ABCD"));
        assert_eq!(model.provisioning_setup_url, Some(SETUP_URL));

        model.apply(DisplayEvent::SystemState(LedState::Running));
        assert_eq!(model.state(), LedState::Running);

        model.apply(DisplayEvent::ActiveLocoCount(3));
        assert_eq!(model.loco_count, 3);

        model.apply(DisplayEvent::Fault(FaultCause::TrackShort));
        assert_eq!(model.fault, Some(FaultCause::TrackShort));

        model.apply(DisplayEvent::FaultCleared);
        assert_eq!(model.fault, None);

        let mut text: heapless::String<21> = heapless::String::new();
        text.push_str("hello").expect("fits in 21-byte buffer");
        model.apply(DisplayEvent::Message(text));
        assert_eq!(model.message.as_deref(), Some("hello"));
    }
}
