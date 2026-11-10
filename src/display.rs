//! OLED display event types and async task.

use crate::system_status::{BootStep, DisplayEvent, ProvisioningSsid};

use crate::station_status::{FaultCause, LedState};

#[cfg(target_arch = "riscv32")]
use crate::system_status::channels::{BootReadySender, DisplayReceiver};
#[cfg(target_arch = "riscv32")]
use crate::system_status::{BootReadyEvent, OptionalPeripheralInit, announce_ready};
#[cfg(target_arch = "riscv32")]
use core::convert::Infallible;
#[cfg(target_arch = "riscv32")]
use defmt::{info, warn};
#[cfg(target_arch = "riscv32")]
use embassy_time::{Duration, Instant, Timer, with_deadline, with_timeout};
#[cfg(target_arch = "riscv32")]
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
#[cfg(target_arch = "riscv32")]
use ssd1306::{I2CDisplayInterface, Ssd1306Async, mode::BufferedGraphicsModeAsync, prelude::*};

/// Braille dot spinner frames; each u8 is the Unicode offset from U+2800,
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

/// How long the locomotive row stays on one group of addresses, and the
/// fault row on one logon warning, before moving to the next.
#[cfg(all(target_arch = "riscv32", feature = "dcc-a"))]
const TURN_INTERVAL: Duration = Duration::from_secs(3);

/// Longest one transfer to the panel may take before it is abandoned.
///
/// A full frame is 1 KiB, about 25 ms at 400 kHz. Without a limit, a transfer
/// that never completes parks the task forever and the screen freezes on its
/// last frame while the rest of the station keeps running.
#[cfg(target_arch = "riscv32")]
const TRANSFER_DEADLINE: Duration = Duration::from_millis(100);

/// Pause between attempts to bring the panel back after a failed transfer.
#[cfg(target_arch = "riscv32")]
const RECONNECT_INTERVAL: Duration = Duration::from_secs(1);

/// Why a transfer to the panel failed.
#[cfg(target_arch = "riscv32")]
#[derive(Clone, Copy, Debug, defmt::Format)]
enum TransferError {
    /// The I2C driver reported an error (no acknowledge, lost arbitration).
    Bus,
    /// The transfer did not finish within [`TRANSFER_DEADLINE`].
    Timeout,
}

/// Run one panel transfer under [`TRANSFER_DEADLINE`].
///
/// Dropping an unfinished esp-hal I2C future resets the controller and clears
/// the bus, so abandoning a stuck transfer leaves the driver usable.
#[cfg(target_arch = "riscv32")]
async fn transfer<E>(operation: impl Future<Output = Result<(), E>>) -> Result<(), TransferError> {
    match with_timeout(TRANSFER_DEADLINE, operation).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(_)) => Err(TransferError::Bus),
        Err(_) => Err(TransferError::Timeout),
    }
}

/// Top-left corner of the throbber, next to the state label.
#[cfg(target_arch = "riscv32")]
const THROBBER_ORIGIN: Point = Point::new(96, 18);

/// Area the throbber covers: two 2-pixel columns at X offsets 0 and 3, three
/// 2-pixel rows at Y offsets 0, 3 and 6.
#[cfg(target_arch = "riscv32")]
const THROBBER_SIZE: Size = Size::new(6, 8);

/// Draw braille-style dot spinner at the given top-left position.
/// Each dot is a 2x2 filled rectangle in a 2-column x 3-row grid.
#[cfg(target_arch = "riscv32")]
fn draw_throbber<D: DrawTarget<Color = BinaryColor>>(target: &mut D, origin: Point, mask: u8) {
    let dot = Size::new(2, 2);
    let style = PrimitiveStyle::with_fill(BinaryColor::On);

    // Bit-to-position: column-major order matching braille encoding.
    // (row, col): col X offset: 0 or 3, row Y offset: 0, 3, 6.
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

/// Characters that fit on one row: a 6-pixel font on a 128-pixel panel.
pub const ROW_COLUMNS: usize = 21;

/// One display row, never wider than the panel.
pub type RowText = heapless::String<ROW_COLUMNS>;

/// Aggregated OLED display state, updated one [`DisplayEvent`] at a time.
///
/// Mirrors the reducer pattern used by [`crate::application::StatusModel`]:
/// a single struct owns all display fields so [`Self::apply`] and `render`
/// don't need to thread six independent `&mut` parameters through the task.
#[derive(Debug, Clone)]
pub struct DisplayModel {
    boot_step: BootStep,
    ip: Option<[u8; 4]>,
    provisioning_ssid: Option<ProvisioningSsid>,
    provisioning_setup_url: Option<&'static str>,
    state: LedState,
    #[cfg(not(feature = "dcc-a"))]
    loco_count: u8,
    fault: Option<FaultCause>,
    message: Option<RowText>,
    #[cfg(feature = "dcc-a")]
    logon: crate::system_status::LogonStatus,
    /// Address of a locomotive just put on the rails, and for how many more
    /// turns it is announced.
    #[cfg(feature = "dcc-a")]
    new_loco: Option<(u16, u8)>,
    /// Counts the turns of the rows that show one thing after the other.
    #[cfg(feature = "dcc-a")]
    turn: u32,
}

/// Turns a new locomotive stays announced; see [`DisplayModel::advance_turn`].
#[cfg(feature = "dcc-a")]
const NEW_LOCO_TURNS: u8 = 2;

/// Label of the row that lists the locomotives.
#[cfg(feature = "dcc-a")]
const LOCOS_LABEL: &str = "Locos:";

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
            #[cfg(not(feature = "dcc-a"))]
            loco_count: 0,
            fault: None,
            message: None,
            #[cfg(feature = "dcc-a")]
            logon: crate::system_status::LogonStatus {
                addresses: heapless::Vec::new(),
                conflicts: heapless::Vec::new(),
                failed: 0,
            },
            #[cfg(feature = "dcc-a")]
            new_loco: None,
            #[cfg(feature = "dcc-a")]
            turn: 0,
        }
    }

    /// Applies one display event, updating the relevant field.
    pub fn apply(&mut self, event: DisplayEvent) {
        match event {
            DisplayEvent::BootProgress(s) => {
                if self.boot_step != BootStep::UsbRecovery {
                    self.boot_step = s;
                }
            }
            DisplayEvent::IpAssigned(addr) => self.ip = Some(addr),
            DisplayEvent::ProvisioningMode { ssid, setup_url } => {
                self.provisioning_ssid = Some(ssid);
                self.provisioning_setup_url = Some(setup_url);
            }
            DisplayEvent::SystemState(s) => self.state = s,
            #[cfg(not(feature = "dcc-a"))]
            DisplayEvent::ActiveLocoCount(n) => self.loco_count = n,
            DisplayEvent::Fault(f) => self.fault = Some(f),
            DisplayEvent::FaultCleared => self.fault = None,
            DisplayEvent::Message(m) => self.message = Some(m),
            DisplayEvent::MessageCleared => self.message = None,
            #[cfg(feature = "dcc-a")]
            DisplayEvent::LogonStatus(status) => self.logon = status,
            #[cfg(feature = "dcc-a")]
            DisplayEvent::NewLoco(address) => self.new_loco = Some((address, NEW_LOCO_TURNS)),
        }
    }

    /// The row that tells which locomotives are in use: how many without
    /// DCC-A, their addresses with it.
    #[cfg(not(feature = "dcc-a"))]
    #[must_use]
    pub fn loco_row(&self) -> RowText {
        use core::fmt::Write;

        let mut row = RowText::new();
        let _ = write!(row, "Locos: {}", self.loco_count);
        row
    }

    /// The row that tells which locomotives are in use: how many without
    /// DCC-A, their addresses with it.
    ///
    /// Addresses that do not fit are shown a group at a time, forth and back
    /// through the groups at each turn.
    #[cfg(feature = "dcc-a")]
    #[must_use]
    pub fn loco_row(&self) -> RowText {
        use core::fmt::Write;

        let addresses = &self.logon.addresses;
        let groups = address_groups(addresses).count();
        let mut row = RowText::new();
        let _ = row.push_str(LOCOS_LABEL);
        match address_groups(addresses).nth(there_and_back(self.turn, groups)) {
            Some(group) => {
                for address in group {
                    let _ = write!(row, " {address}");
                }
            }
            None => {
                let _ = row.push_str(" -");
            }
        }
        row
    }

    /// What the fault row shows while no fault is latched: a locomotive
    /// just put on the rails, else the logon warnings one per turn.
    #[cfg(feature = "dcc-a")]
    #[must_use]
    pub fn notice_row(&self) -> Option<RowText> {
        use core::fmt::Write;

        let mut row = RowText::new();
        if let Some((address, _)) = self.new_loco {
            let _ = write!(row, "New loco: {address}");
            return Some(row);
        }
        let conflicts = self.logon.conflicts.len();
        let warnings = conflicts + usize::from(self.logon.failed > 0);
        if warnings == 0 {
            return None;
        }
        match self.logon.conflicts.get(self.turn as usize % warnings) {
            Some(conflict) => {
                let _ = write!(row, "Conflict {}->{}", conflict.wanted, conflict.assigned);
            }
            None => {
                let _ = write!(row, "Logon failed: {}", self.logon.failed);
            }
        }
        Some(row)
    }

    /// Moves the rows that show one thing after the other to the next one.
    /// Returns whether the screen changed.
    #[cfg(feature = "dcc-a")]
    pub fn advance_turn(&mut self) -> bool {
        let before = (self.loco_row(), self.notice_row());
        self.turn = self.turn.wrapping_add(1);
        self.new_loco = match self.new_loco {
            Some((address, turns @ 2..)) => Some((address, turns - 1)),
            _ => None,
        };
        before != (self.loco_row(), self.notice_row())
    }

    /// Returns the current LED-equivalent system state shown on screen.
    ///
    /// Exists for the tests below, which assert on the model without a screen.
    #[cfg(test)]
    #[must_use]
    pub const fn state(&self) -> LedState {
        self.state
    }

    /// Returns the final row, preserving a terminal USB-recovery instruction.
    ///
    /// A message wins over the boot step: the network reports `WifiConnected`
    /// after boot has already reached `SystemRunning`, and keying messages on
    /// the step hid every one of them behind "WiFi connected".
    #[must_use]
    pub fn footer(&self) -> &str {
        match (self.boot_step, self.message.as_deref()) {
            (BootStep::UsbRecovery, _) => boot_step_label(BootStep::UsbRecovery),
            (_, Some(message)) => message,
            (BootStep::SystemRunning, None) => "",
            (step, None) => boot_step_label(step),
        }
    }
}

impl Default for DisplayModel {
    fn default() -> Self {
        Self::new()
    }
}

/// Splits `addresses` into the runs that fit on the locomotive row after
/// its label, each address behind a space.
#[cfg(feature = "dcc-a")]
fn address_groups(addresses: &[u16]) -> impl Iterator<Item = &[u16]> {
    const ROOM: usize = ROW_COLUMNS - LOCOS_LABEL.len();
    let mut rest = addresses;
    core::iter::from_fn(move || {
        let mut width = 0;
        let fitting = rest
            .iter()
            .take_while(|address| {
                width += 1 + decimal_digits(**address);
                width <= ROOM
            })
            .count();
        // Any address fits alone, so a group is empty only at the end.
        let (group, remaining) = rest.split_at(fitting);
        rest = remaining;
        (!group.is_empty()).then_some(group)
    })
}

#[cfg(feature = "dcc-a")]
fn decimal_digits(value: u16) -> usize {
    value.checked_ilog10().map_or(1, |log| log as usize + 1)
}

/// Position at `turn` when walking `count` places forth and back:
/// 0, 1, 2, 1, 0, 1, ... for three.
#[cfg(feature = "dcc-a")]
fn there_and_back(turn: u32, count: usize) -> usize {
    if count < 2 {
        return 0;
    }
    let period = 2 * (count - 1);
    let step = turn as usize % period;
    if step < count { step } else { period - step }
}

fn boot_step_label(step: BootStep) -> &'static str {
    match step {
        BootStep::PeripheralsInit => "Init peripherals",
        BootStep::WifiConnecting => "WiFi connecting",
        BootStep::WifiConnected => "WiFi connected",
        BootStep::DccEngineReady => "DCC engine ready",
        BootStep::SystemRunning => "System running",
        BootStep::UsbRecovery => "OTA: USB recovery",
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
        FaultCause::TrackShort => "Track Short",
        FaultCause::Estop => "E-Stop",
        FaultCause::Internal => "Internal",
    }
}

/// Title row: the station and the firmware version it runs.
#[cfg(target_arch = "riscv32")]
const TITLE: &str = concat!("DCC Station v", env!("CARGO_PKG_VERSION"));
#[cfg(target_arch = "riscv32")]
const _: () = assert!(TITLE.len() <= ROW_COLUMNS, "title wider than the display");

/// Format an IPv4 address into a heapless string (max "255.255.255.255" = 15 chars).
#[cfg(target_arch = "riscv32")]
fn format_ip(addr: [u8; 4]) -> heapless::String<15> {
    use core::fmt::Write;
    let mut s = heapless::String::new();
    let _ = write!(s, "{}.{}.{}.{}", addr[0], addr[1], addr[2], addr[3]);
    s
}

#[cfg(target_arch = "riscv32")]
/// The OLED panel driven by the display task.
#[cfg(target_arch = "riscv32")]
type Oled = Ssd1306Async<
    I2CInterface<esp_hal::i2c::master::I2c<'static, esp_hal::Async>>,
    DisplaySize128x64,
    BufferedGraphicsModeAsync<DisplaySize128x64>,
>;

/// Draw the fault row near the bottom of the panel, reusing `line` as scratch.
#[cfg(target_arch = "riscv32")]
fn draw_fault_row(
    target: &mut Oled,
    line: &mut RowText,
    cause: FaultCause,
    style: MonoTextStyle<'_, BinaryColor>,
) {
    use core::fmt::Write;

    line.clear();
    let _ = write!(line, "Fault: {}", fault_label(cause));
    let _ = Text::new(line, Point::new(0, 56), style).draw(target);
}

/// Advance the throbber by one frame and schedule the next tick.
#[cfg(target_arch = "riscv32")]
fn advance_throbber(frame: &mut u8, next_tick: &mut Instant) {
    *frame = (*frame + 1) % THROBBER_FRAMES.len() as u8;
    *next_tick += THROBBER_INTERVAL;
}

/// Report the display as degraded, then consume display events forever so
/// their senders never block on a full channel.
#[cfg(target_arch = "riscv32")]
async fn drain_degraded(
    receiver: DisplayReceiver,
    ready_sender: BootReadySender,
    cause: OptionalPeripheralInit,
) -> ! {
    announce_ready(ready_sender, BootReadyEvent::DisplayDegraded(cause)).await;
    loop {
        let _ = receiver.receive().await;
    }
}

#[cfg(target_arch = "riscv32")]
async fn render(
    display: &mut Oled,
    model: &DisplayModel,
    throbber_frame: u8,
) -> Result<(), TransferError> {
    use core::fmt::Write;

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let _ = display.clear(BinaryColor::Off);

    if model.boot_step != BootStep::UsbRecovery
        && let (Some(ssid), Some(setup_url)) =
            (&model.provisioning_ssid, model.provisioning_setup_url)
    {
        let _ = Text::new("DCC WiFi Setup", Point::new(0, 10), style).draw(display);

        let mut line = RowText::new();
        let _ = write!(line, "AP: {ssid}");
        let _ = Text::new(&line, Point::new(0, 26), style).draw(display);

        let _ = Text::new(setup_url, Point::new(0, 36), style).draw(display);
        let _ = Text::new("Track off", Point::new(0, 46), style).draw(display);

        if let Some(cause) = model.fault {
            draw_fault_row(display, &mut line, cause, style);
        } else {
            let _ = Text::new("Save and restart", Point::new(0, 56), style).draw(display);
        }

        return transfer(display.flush()).await;
    }

    // Yellow zone (pixels 0-15): title
    let _ = Text::new(TITLE, Point::new(0, 10), style).draw(display);

    // Blue zone (pixels 16-63): skip gap, start content at Y=26
    let mut line = RowText::new();
    let _ = write!(line, "State: {}", led_state_label(model.state));
    let _ = Text::new(&line, Point::new(0, 26), style).draw(display);

    line.clear();
    if let Some(addr) = model.ip {
        let ip_str = format_ip(addr);
        let _ = write!(line, "IP: {ip_str}");
    } else {
        let _ = write!(line, "IP: ---.---.---.---");
    }
    let _ = Text::new(&line, Point::new(0, 36), style).draw(display);

    let _ = Text::new(&model.loco_row(), Point::new(0, 46), style).draw(display);

    if let Some(cause) = model.fault {
        draw_fault_row(display, &mut line, cause, style);
    }
    // A latched fault keeps the row; a logon notice only borrows it, and
    // gives way to a footer message, whose row its letters would run into.
    #[cfg(feature = "dcc-a")]
    if model.fault.is_none()
        && model.footer().is_empty()
        && let Some(notice) = model.notice_row()
    {
        let _ = Text::new(&notice, Point::new(0, 56), style).draw(display);
    }

    // Animated throbber next to state label when running.
    if model.state == LedState::Running {
        draw_throbber(display, THROBBER_ORIGIN, throbber_mask(throbber_frame));
    }

    let _ = Text::new(model.footer(), Point::new(0, 63), style).draw(display);

    transfer(display.flush()).await
}

/// Redraw only the throbber area and flush that window.
///
/// The buffered mode sends just the pixels touched since the last flush, so
/// a throbber tick costs a few bytes on the I2C bus instead of the whole
/// frame buffer that [`render`] sends.
#[cfg(target_arch = "riscv32")]
async fn render_throbber(display: &mut Oled, throbber_frame: u8) -> Result<(), TransferError> {
    let _ = Rectangle::new(THROBBER_ORIGIN, THROBBER_SIZE)
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(display);
    draw_throbber(display, THROBBER_ORIGIN, throbber_mask(throbber_frame));
    transfer(display.flush()).await
}

#[cfg(target_arch = "riscv32")]
fn throbber_mask(throbber_frame: u8) -> u8 {
    THROBBER_FRAMES[usize::from(throbber_frame) % THROBBER_FRAMES.len()]
}

/// Embassy task: receives display events, updates local state, redraws the
/// OLED. A model change redraws the whole frame; a throbber tick redraws the
/// throbber alone. A failed transfer re-initializes the panel and redraws it.
#[cfg(target_arch = "riscv32")]
pub async fn display_task(
    i2c: Option<esp_hal::i2c::master::I2c<'static, esp_hal::Async>>,
    receiver: DisplayReceiver,
    ready_sender: BootReadySender,
) -> ! {
    let Some(i2c) = i2c else {
        warn!("display: disabled, draining display events");
        drain_degraded(
            receiver,
            ready_sender,
            OptionalPeripheralInit::DisplayUnavailable,
        )
        .await
    };

    // Allow the SSD1306 to settle after power-on before sending commands.
    Timer::after(Duration::from_millis(100)).await;

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    let mut model = DisplayModel::new();
    let mut throbber_frame: u8 = 0;

    // Initial render shows the "PeripheralsInit" boot step.
    let connected = match transfer(display.init()).await {
        Ok(()) => {
            announce_ready(ready_sender, BootReadyEvent::DisplayReady).await;
            render(&mut display, &model, throbber_frame).await
        }
        Err(error) => {
            warn!("display: SSD1306 init failed ({}), retrying", error);
            announce_ready(
                ready_sender,
                BootReadyEvent::DisplayDegraded(OptionalPeripheralInit::DisplayInit),
            )
            .await;
            Err(error)
        }
    };
    if connected.is_err() {
        reconnect(&mut display, &mut model, &receiver, throbber_frame).await;
    }

    loop {
        let Err(error) = drive(&mut display, &mut model, &receiver, &mut throbber_frame).await;
        warn!("display: transfer failed ({}), re-initializing", error);
        reconnect(&mut display, &mut model, &receiver, throbber_frame).await;
    }
}

/// Apply display events and redraw until a transfer to the panel fails.
#[cfg(target_arch = "riscv32")]
async fn drive(
    display: &mut Oled,
    model: &mut DisplayModel,
    receiver: &DisplayReceiver,
    throbber_frame: &mut u8,
) -> Result<Infallible, TransferError> {
    #[cfg(feature = "dcc-a")]
    let mut next_turn = Instant::now() + TURN_INTERVAL;
    loop {
        if model.state != LedState::Running {
            *throbber_frame = 0;
            // The rows that take turns keep moving while stopped, also when
            // events keep coming.
            #[cfg(feature = "dcc-a")]
            let Ok(event) = with_deadline(next_turn, receiver.receive()).await else {
                next_turn = Instant::now() + TURN_INTERVAL;
                if model.advance_turn() {
                    render(display, model, *throbber_frame).await?;
                }
                continue;
            };
            #[cfg(not(feature = "dcc-a"))]
            let event = receiver.receive().await;
            model.apply(event);
            render(display, model, *throbber_frame).await?;
            continue;
        }

        let mut next_throbber_tick = Instant::now() + THROBBER_INTERVAL;
        while model.state == LedState::Running {
            let now = Instant::now();
            #[cfg(feature = "dcc-a")]
            if now >= next_turn {
                next_turn = now + TURN_INTERVAL;
                if model.advance_turn() {
                    render(display, model, *throbber_frame).await?;
                }
            }
            if now >= next_throbber_tick {
                advance_throbber(throbber_frame, &mut next_throbber_tick);
                render_throbber(display, *throbber_frame).await?;
                continue;
            }

            if let Ok(event) = with_timeout(next_throbber_tick - now, receiver.receive()).await {
                model.apply(event);
                render(display, model, *throbber_frame).await?;
            } else {
                advance_throbber(throbber_frame, &mut next_throbber_tick);
                render_throbber(display, *throbber_frame).await?;
            }
        }
    }
}

/// Re-initialize the panel once per [`RECONNECT_INTERVAL`] until it accepts a
/// full frame again.
///
/// Events keep being applied meanwhile, so their senders never block on a full
/// channel and the first frame after recovery shows the current state.
#[cfg(target_arch = "riscv32")]
async fn reconnect(
    display: &mut Oled,
    model: &mut DisplayModel,
    receiver: &DisplayReceiver,
    throbber_frame: u8,
) {
    let mut attempts: u32 = 0;
    loop {
        let retry_at = Instant::now() + RECONNECT_INTERVAL;
        while let Ok(event) = with_deadline(retry_at, receiver.receive()).await {
            model.apply(event);
        }

        attempts += 1;
        // `init` clears the frame buffer, so the render that follows sends
        // the whole frame rather than only the pixels changed since the
        // failed transfer.
        if transfer(display.init()).await.is_ok()
            && render(display, model, throbber_frame).await.is_ok()
        {
            info!("display: panel back after {} attempt(s)", attempts);
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use dcc_provisioning::net_config::SETUP_URL;

    use super::*;

    #[test]
    fn test_new_model_starts_in_initial_boot_state() {
        let model = DisplayModel::new();
        assert_eq!(model.boot_step, BootStep::PeripheralsInit);
        assert_eq!(model.state(), LedState::Booting);
        assert_eq!(model.ip, None);
        assert!(model.provisioning_ssid.is_none());
        assert!(model.provisioning_setup_url.is_none());
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

        let mut ssid = ProvisioningSsid::new();
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

        model.apply(DisplayEvent::Fault(FaultCause::TrackShort));
        assert_eq!(model.fault, Some(FaultCause::TrackShort));

        model.apply(DisplayEvent::FaultCleared);
        assert_eq!(model.fault, None);

        let mut text = RowText::new();
        text.push_str("hello").expect("fits in 21-byte buffer");
        model.apply(DisplayEvent::Message(text));
        assert_eq!(model.message.as_deref(), Some("hello"));
    }

    #[test]
    fn message_shows_after_wifi_reports_late() {
        // Real order on the board: boot completes, then DHCP finishes.
        let mut model = DisplayModel::new();
        model.apply(DisplayEvent::BootProgress(BootStep::SystemRunning));
        model.apply(DisplayEvent::BootProgress(BootStep::WifiConnected));
        assert_eq!(model.footer(), "WiFi connected");
        model.apply(DisplayEvent::Message(
            heapless::String::try_from("OTA: testing 0.1.3").unwrap(),
        ));
        assert_eq!(model.footer(), "OTA: testing 0.1.3");
        model.apply(DisplayEvent::MessageCleared);
        assert_eq!(model.footer(), "WiFi connected");
    }

    #[test]
    fn cleared_message_empties_the_running_footer() {
        let mut model = DisplayModel::new();
        model.apply(DisplayEvent::BootProgress(BootStep::SystemRunning));
        model.apply(DisplayEvent::Message(
            heapless::String::try_from("OK 0.1.1: enable Z21").unwrap(),
        ));
        assert_eq!(model.footer(), "OK 0.1.1: enable Z21");
        model.apply(DisplayEvent::MessageCleared);
        assert_eq!(model.footer(), "");
    }

    #[test]
    fn recovery_instruction_survives_boot_progress_and_messages() {
        let mut model = DisplayModel::new();
        assert_eq!(model.footer(), "Init peripherals");
        model.apply(DisplayEvent::BootProgress(BootStep::UsbRecovery));
        assert_eq!(model.footer(), "OTA: USB recovery");
        model.apply(DisplayEvent::BootProgress(BootStep::SystemRunning));
        model.apply(DisplayEvent::Message(
            heapless::String::try_from("WiFi connecting").unwrap(),
        ));
        assert_eq!(model.footer(), "OTA: USB recovery");
        assert_eq!(model.state(), LedState::Booting);
    }

    #[cfg(feature = "dcc-a")]
    mod logon {
        use super::*;
        use crate::system_status::{AddressConflict, LogonStatus};

        fn model_with(
            addresses: &[u16],
            conflicts: &[AddressConflict],
            failed: u8,
        ) -> DisplayModel {
            let mut model = DisplayModel::new();
            model.apply(DisplayEvent::LogonStatus(LogonStatus {
                addresses: heapless::Vec::from_slice(addresses).unwrap(),
                conflicts: heapless::Vec::from_slice(conflicts).unwrap(),
                failed,
            }));
            model
        }

        /// The locomotive row at each of the next `turns` turns.
        fn loco_rows(model: &mut DisplayModel, turns: usize) -> Vec<RowText> {
            (0..turns)
                .map(|_| {
                    let row = model.loco_row();
                    model.advance_turn();
                    row
                })
                .collect()
        }

        #[test]
        fn the_loco_row_lists_the_addresses_or_a_dash() {
            assert_eq!(model_with(&[], &[], 0).loco_row(), "Locos: -");
            let mut model = model_with(&[3, 4, 9_999], &[], 0);
            assert_eq!(model.loco_row(), "Locos: 3 4 9999");
            // One group never moves, so the screen stays as it is.
            assert!(!model.advance_turn());
            assert_eq!(model.loco_row(), "Locos: 3 4 9999");
        }

        #[test]
        fn addresses_that_do_not_fit_are_shown_in_groups_forth_and_back() {
            let mut model =
                model_with(&[3, 4, 12, 100, 1_000, 2_000, 9_998, 9_999, 10_239], &[], 0);

            let rows = loco_rows(&mut model, 5);

            assert_eq!(
                rows,
                [
                    "Locos: 3 4 12 100",
                    "Locos: 1000 2000 9998",
                    "Locos: 9999 10239",
                    "Locos: 1000 2000 9998",
                    "Locos: 3 4 12 100",
                ]
            );
            assert!(rows.iter().all(|row| row.len() <= ROW_COLUMNS));
        }

        #[test]
        fn the_widest_addresses_fill_the_row_two_at_a_time() {
            let widest: Vec<u16> = (10_230..=10_239).collect();
            let mut model = model_with(&widest, &[], 0);

            let rows = loco_rows(&mut model, 5);

            assert_eq!(rows[0], "Locos: 10230 10231");
            assert_eq!(rows[4], "Locos: 10238 10239");
        }

        #[test]
        fn two_groups_alternate() {
            let mut model = model_with(&[1_000, 2_000, 3_000, 4_000], &[], 0);

            let rows = loco_rows(&mut model, 4);

            assert_eq!(rows[0], "Locos: 1000 2000 3000");
            assert_eq!(rows[1], "Locos: 4000");
            assert_eq!(rows[2..], rows[..2]);
        }

        #[test]
        fn a_shorter_list_is_shown_from_wherever_the_turns_are() {
            let mut model = model_with(&[1_000, 2_000, 3_000, 4_000, 5_000, 6_000, 7_000], &[], 0);
            loco_rows(&mut model, 2);

            model.apply(DisplayEvent::LogonStatus(LogonStatus {
                addresses: heapless::Vec::from_slice(&[3]).unwrap(),
                ..LogonStatus::default()
            }));

            assert_eq!(model.loco_row(), "Locos: 3");
        }

        #[test]
        fn silent_decoders_alone_are_told_on_the_notice_row() {
            let model = model_with(&[], &[], 1);

            assert_eq!(model.notice_row().as_deref(), Some("Logon failed: 1"));
        }

        #[test]
        fn the_screen_changes_when_the_new_loco_notice_ends() {
            let mut model = model_with(&[3], &[], 0);
            model.apply(DisplayEvent::NewLoco(3));

            assert!(!model.advance_turn());
            assert!(model.advance_turn());
            assert_eq!(model.notice_row(), None);
        }

        #[test]
        fn warnings_take_turns_on_the_notice_row() {
            let conflicts = [
                AddressConflict {
                    wanted: 3,
                    assigned: 9_999,
                },
                AddressConflict {
                    wanted: 10_239,
                    assigned: 9_998,
                },
            ];
            let mut model = model_with(&[3], &conflicts, 2);

            let notices: Vec<_> = (0..4)
                .map(|_| {
                    let notice = model.notice_row().expect("a warning");
                    assert!(model.advance_turn());
                    notice
                })
                .collect();

            assert_eq!(
                notices,
                [
                    "Conflict 3->9999",
                    "Conflict 10239->9998",
                    "Logon failed: 2",
                    "Conflict 3->9999",
                ]
            );
        }

        #[test]
        fn no_warning_leaves_the_notice_row_empty() {
            assert_eq!(model_with(&[3, 4], &[], 0).notice_row(), None);
        }

        #[test]
        fn a_new_loco_is_announced_for_two_turns_ahead_of_the_warnings() {
            let conflict = AddressConflict {
                wanted: 3,
                assigned: 9_999,
            };
            let mut model = model_with(&[3], &[conflict], 0);
            model.apply(DisplayEvent::NewLoco(10_239));

            let notices: Vec<_> = (0..3)
                .map(|_| {
                    let notice = model.notice_row().expect("a notice");
                    model.advance_turn();
                    notice
                })
                .collect();

            assert_eq!(
                notices,
                ["New loco: 10239", "New loco: 10239", "Conflict 3->9999"]
            );
        }

        #[test]
        fn the_last_status_replaces_the_one_before() {
            let mut model = model_with(&[3], &[], 1);

            model.apply(DisplayEvent::LogonStatus(LogonStatus::default()));

            assert_eq!(model.loco_row(), "Locos: -");
            assert_eq!(model.notice_row(), None);
        }
    }

    #[cfg(not(feature = "dcc-a"))]
    #[test]
    fn the_loco_row_counts_the_locomotives_without_dcc_a() {
        let mut model = DisplayModel::new();
        model.apply(DisplayEvent::ActiveLocoCount(2));

        assert_eq!(model.loco_row(), "Locos: 2");
    }
}
