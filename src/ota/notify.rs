//! Update lines on the display's footer, with the little state they need.

use core::cell::RefCell;

use dcc_ota::notice::{NOTICE_COLUMNS, Notice};

use crate::{
    display::{ROW_COLUMNS, RowText},
    runtime_channels::DisplaySender,
    system_status::{BootStep, DisplayEvent},
};

/// What the display footer shows; only this module touches it.
struct Footer {
    display: Option<DisplaySender>,
    /// The line asks the operator to power the track from Z21.
    awaits_track: bool,
    /// The display has received the USB recovery screen.
    recovery_shown: bool,
}

static FOOTER: critical_section::Mutex<RefCell<Footer>> =
    critical_section::Mutex::new(RefCell::new(Footer {
        display: None,
        awaits_track: false,
        recovery_shown: false,
    }));

fn footer<R>(f: impl FnOnce(&mut Footer) -> R) -> R {
    critical_section::with(|cs| f(&mut FOOTER.borrow(cs).borrow_mut()))
}

const _: () = assert!(
    NOTICE_COLUMNS <= ROW_COLUMNS,
    "notices wider than a display row"
);

/// `notice` as a display row; it always fits, as checked above.
fn row(notice: Notice) -> RowText {
    RowText::try_from(notice.text().as_str()).unwrap_or_default()
}

/// Connects the footer to the display task.
pub(super) fn attach(display: DisplaySender) {
    footer(|f| f.display = Some(display));
}

/// Shows one update step on the display's last row.
///
/// Best effort: a full display queue drops the line and the next step
/// overwrites it anyway. Lines that ask for the track stay until the
/// operator powers it.
pub(crate) fn notify(notice: Notice) {
    let display = footer(|f| {
        f.awaits_track = notice.awaits_track();
        f.display
    });
    if let Some(display) = display {
        let _ = display.try_send(DisplayEvent::Message(row(notice)));
    }
}

/// Clears a line that asked for the track once the track is powered.
pub(super) fn clear_once_track_on(track_on: bool) {
    let Some(display) = footer(|f| f.display.filter(|_| f.awaits_track)) else {
        return;
    };
    if track_on && display.try_send(DisplayEvent::MessageCleared).is_ok() {
        footer(|f| f.awaits_track = false);
    }
}

/// Shows the terminal USB recovery screen until the display has it; the
/// monitor calls this every second, so a full queue only delays it.
pub(super) fn show_recovery_until_delivered() {
    let display = footer(|f| f.display.filter(|_| !f.recovery_shown));
    if let Some(display) = display
        && display
            .try_send(DisplayEvent::BootProgress(BootStep::UsbRecovery))
            .is_ok()
    {
        footer(|f| f.recovery_shown = true);
    }
}
