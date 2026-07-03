//! ISR-backed RailCom UART capture path.
//!
//! The active ESP32-C6 runtime uses the cutout ISR to delimit a UART1 FIFO
//! snapshot, then this task drains those snapshots into the normal RailCom
//! parser.
//!
//! Honesty note: this path does not detect UART framing errors or line
//! glitches. It only classifies a window as "oversized" when the captured
//! FIFO snapshot exceeds `MAX_CAPTURE_BYTES`, which is an overflow condition,
//! not evidence of corrupted data. If framing-error detection is ever needed,
//! it must be read from the UART1 hardware status/interrupt flags (e.g.
//! `RXFIFO_OVF`, parity/frame error bits) at capture time - no such logic
//! exists in this module today.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use esp_hal::Async;
use esp_hal::uart::UartRx;
use heapless::Vec;

use crate::railcom::pipeline::{
    RailcomChannel, RailcomRxWindow, RailcomRxWindowError, process_rx_window,
    record_oversized_window, record_rx_overflows,
};
use crate::railcom::uart_reader::{RailcomRxOutput, RailcomUartWindowError};

const CAPTURE_RING_CAPACITY: usize = 64;
const MAX_CAPTURE_BYTES: usize = 6;

static CAPTURE_WRITE_COUNT: AtomicU32 = AtomicU32::new(0);
static CAPTURE_READY: AtomicBool = AtomicBool::new(false);
static CAPTURE_SEQUENCES: [AtomicU32; CAPTURE_RING_CAPACITY] =
    [const { AtomicU32::new(0) }; CAPTURE_RING_CAPACITY];
static CAPTURE_CHANNELS: [AtomicU8; CAPTURE_RING_CAPACITY] =
    [const { AtomicU8::new(0) }; CAPTURE_RING_CAPACITY];
static CAPTURE_LENS: [AtomicU8; CAPTURE_RING_CAPACITY] =
    [const { AtomicU8::new(0) }; CAPTURE_RING_CAPACITY];
static CAPTURE_BYTES: [[AtomicU8; MAX_CAPTURE_BYTES]; CAPTURE_RING_CAPACITY] =
    [const { [const { AtomicU8::new(0) }; MAX_CAPTURE_BYTES] }; CAPTURE_RING_CAPACITY];

type CaptureNotifyChannel = embassy_sync::channel::Channel<CriticalSectionRawMutex, u8, 1>;
static CAPTURE_NOTIFY: CaptureNotifyChannel = CaptureNotifyChannel::new();

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct CapturedWindow {
    packet_sequence: u32,
    channel: RailcomChannel,
    len: usize,
    bytes: [u8; MAX_CAPTURE_BYTES],
}

struct RailcomIsrCaptureUart {
    _uart_rx: UartRx<'static, Async>,
}

impl RailcomIsrCaptureUart {
    /// # Panics
    ///
    /// Panics if called more than once: this type owns the sole UART1 RX
    /// data path (see the SAFETY comments on the ISR-side register helpers
    /// below), so a second instance would race the first over FIFO access.
    /// Mirrors the double-init guard in `TrackOutput::new`.
    fn new(uart_rx: UartRx<'static, Async>) -> Self {
        let was_ready = CAPTURE_READY.swap(true, Ordering::AcqRel);
        assert!(
            !was_ready,
            "RailcomIsrCaptureUart must be initialized only once"
        );
        Self { _uart_rx: uart_rx }
    }
}

#[inline(always)]
fn uart1_rx_fifo_count() -> Option<u16> {
    if !CAPTURE_READY.load(Ordering::Acquire) {
        return None;
    }
    // SAFETY: `RailcomIsrCaptureUart` consumes the HAL UART1 RX owner before
    // setting CAPTURE_READY. After that, this module is the only UART1 RX
    // data path. The ISR-side helpers touch the RX FIFO data/count registers
    // and (in `uart1_reset_rx_fifo`) the `conf0.rxfifo_rst` control bit that
    // resets that same FIFO - no other UART1 register is written.
    let regs = unsafe { &*esp_hal::peripherals::UART1::ptr() };
    Some(regs.status().read().rxfifo_cnt().bits() as u16)
}

#[inline(always)]
fn uart1_read_byte() -> Option<u8> {
    if !CAPTURE_READY.load(Ordering::Acquire) {
        return None;
    }
    // SAFETY: same UART1 ownership invariant as `uart1_rx_fifo_count`.
    let regs = unsafe { &*esp_hal::peripherals::UART1::ptr() };
    Some(regs.fifo().read().rxfifo_rd_byte().bits())
}

#[inline(always)]
fn uart1_reset_rx_fifo() {
    if !CAPTURE_READY.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: same UART1 ownership invariant as `uart1_rx_fifo_count`.
    let regs = unsafe { &*esp_hal::peripherals::UART1::ptr() };
    regs.conf0().modify(|_, w| w.rxfifo_rst().set_bit());
    regs.conf0().modify(|_, w| w.rxfifo_rst().clear_bit());
}

#[inline(always)]
pub fn open_window_from_isr() {
    uart1_reset_rx_fifo();
}

// Publication protocol note (contrast with the seqlock pattern documented on
// `track_output::record_railcom_packet_metadata_from_isr`): this ring does
// *not* need a zero-sentinel pre-store. All per-slot fields below are written
// with `Relaxed` ordering, but the single `CAPTURE_WRITE_COUNT` store that
// follows them uses `Release`, and `drain_captured_windows` only reads a slot
// after an `Acquire` load of `CAPTURE_WRITE_COUNT` proves that slot's writes
// already happened (single ISR-side writer, so program order plus one
// Release/Acquire pair on the counter is enough to publish the whole slot
// atomically as a unit). Do not "harmonize" this with the other ring by
// adding a zero pre-store here, and do not drop the final `Release` store
// order relative to the per-field stores above it.
#[inline(always)]
pub fn close_window_from_isr(packet_sequence: u32, channel: RailcomChannel) {
    let write_count = CAPTURE_WRITE_COUNT.load(Ordering::Relaxed);
    let slot = write_count as usize % CAPTURE_RING_CAPACITY;
    let Some(fifo_len) = uart1_rx_fifo_count().map(|len| len as usize) else {
        return;
    };
    CAPTURE_SEQUENCES[slot].store(packet_sequence, Ordering::Relaxed);
    CAPTURE_CHANNELS[slot].store(u8::from(channel), Ordering::Relaxed);
    CAPTURE_LENS[slot].store(fifo_len.min(u8::MAX as usize) as u8, Ordering::Relaxed);
    for cell in &CAPTURE_BYTES[slot][..fifo_len.min(MAX_CAPTURE_BYTES)] {
        let Some(byte) = uart1_read_byte() else {
            break;
        };
        cell.store(byte, Ordering::Relaxed);
    }

    CAPTURE_WRITE_COUNT.store(write_count.wrapping_add(1), Ordering::Release);
    let _ = CAPTURE_NOTIFY.sender().try_send(0);
}

// See `track_output::drain_cutout_runtime_events` for the sibling ring-drain
// implementation and why the two are kept separate instead of merged into one
// generic helper (different overflow arithmetic and per-slot verification).
fn drain_captured_windows(
    next_read: &mut u32,
    out: &mut Vec<CapturedWindow, CAPTURE_RING_CAPACITY>,
) {
    let write = CAPTURE_WRITE_COUNT.load(Ordering::Acquire);
    let available = write.wrapping_sub(*next_read);
    if available > CAPTURE_RING_CAPACITY as u32 {
        let dropped = available - CAPTURE_RING_CAPACITY as u32;
        record_rx_overflows(dropped);
        *next_read = write.wrapping_sub(CAPTURE_RING_CAPACITY as u32);
    }

    let mut remaining = write
        .wrapping_sub(*next_read)
        .min(CAPTURE_RING_CAPACITY as u32);
    while remaining > 0 && out.len() < CAPTURE_RING_CAPACITY {
        let slot = *next_read as usize % CAPTURE_RING_CAPACITY;
        let Ok(channel) = RailcomChannel::try_from(CAPTURE_CHANNELS[slot].load(Ordering::Acquire))
        else {
            *next_read = next_read.wrapping_add(1);
            remaining -= 1;
            continue;
        };
        let len = CAPTURE_LENS[slot].load(Ordering::Acquire) as usize;
        let mut bytes = [0u8; MAX_CAPTURE_BYTES];
        for index in 0..len.min(MAX_CAPTURE_BYTES) {
            bytes[index] = CAPTURE_BYTES[slot][index].load(Ordering::Acquire);
        }

        let _ = out.push(CapturedWindow {
            packet_sequence: CAPTURE_SEQUENCES[slot].load(Ordering::Acquire),
            channel,
            len,
            bytes,
        });
        *next_read = next_read.wrapping_add(1);
        remaining -= 1;
    }
}

async fn send_captured_window(
    sender: &Sender<'static, CriticalSectionRawMutex, RailcomRxOutput, 8>,
    window: CapturedWindow,
) {
    if window.len > MAX_CAPTURE_BYTES {
        record_rx_overflows(1);
        record_oversized_window();
        sender
            .send(RailcomRxOutput::WindowError(
                RailcomUartWindowError::WindowTooLong {
                    packet_sequence: window.packet_sequence,
                    channel: window.channel,
                    provided_len: window.len,
                    max_len: MAX_CAPTURE_BYTES,
                },
            ))
            .await;
        return;
    }

    match RailcomRxWindow::try_new(
        window.packet_sequence,
        window.channel,
        &window.bytes[..window.len.min(MAX_CAPTURE_BYTES)],
    ) {
        Ok(rx_window) => {
            sender
                .send(RailcomRxOutput::WindowProcessed(process_rx_window(
                    rx_window,
                )))
                .await;
        }
        Err(RailcomRxWindowError::WindowTooLong {
            provided_len,
            max_len,
        }) => {
            sender
                .send(RailcomRxOutput::WindowError(
                    RailcomUartWindowError::WindowTooLong {
                        packet_sequence: window.packet_sequence,
                        channel: window.channel,
                        provided_len,
                        max_len,
                    },
                ))
                .await;
        }
    }
}

pub fn capture_notify_receiver() -> Receiver<'static, CriticalSectionRawMutex, u8, 1> {
    CAPTURE_NOTIFY.receiver()
}

pub async fn railcom_isr_capture_task(
    uart_rx: UartRx<'static, Async>,
    result_sender: Sender<'static, CriticalSectionRawMutex, RailcomRxOutput, 8>,
) -> ! {
    let _uart_owner = RailcomIsrCaptureUart::new(uart_rx);
    let capture_receiver = capture_notify_receiver();
    let mut next_read = 0u32;

    loop {
        capture_receiver.receive().await;

        let mut windows = Vec::<CapturedWindow, CAPTURE_RING_CAPACITY>::new();
        drain_captured_windows(&mut next_read, &mut windows);
        for window in windows {
            send_captured_window(&result_sender, window).await;
        }
    }
}
