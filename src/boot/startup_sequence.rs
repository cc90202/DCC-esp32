//! DCC packet sequences sent while the physical track output is still disabled.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Timer};

use crate::cutout::CutoutMode;
use crate::dcc::{DccFrame, DccPacket};

pub(super) async fn send_power_on_idle_burst(
    sender: &Sender<'static, CriticalSectionRawMutex, DccFrame, 16>,
) {
    // About 160 ms of clean signal prevents a decoder from interpreting the
    // initial transition as direct current and entering analogue mode.
    for _ in 0..20 {
        sender
            .send(DccFrame::new(DccPacket::Idle, CutoutMode::None))
            .await;
    }
}

pub(super) async fn send_decoder_reset_sequence(
    sender: &Sender<'static, CriticalSectionRawMutex, DccFrame, 16>,
) {
    // NMRA S-9.2.4 requires at least three reset packets before normal use.
    for _ in 0..5 {
        sender
            .send(DccFrame::new(DccPacket::Reset, CutoutMode::None))
            .await;
    }
    Timer::after(Duration::from_millis(100)).await;
}
