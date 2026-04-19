#[cfg(target_arch = "riscv32")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(target_arch = "riscv32")]
use esp_hal::uart::{Config, Parity, RxConfig, StopBits};

use crate::railcom::pipeline::RailcomChannel;
use crate::railcom::uart_adapter::{
    RailcomUartAdapter, RailcomUartAdapterOutput, RailcomUartEvent, RailcomUartWindowInputError,
};

/// Preferred UART RX pin for RailCom reception.
///
/// The current board review leaves `GPIO5` free and it is the preferred input
/// for the detector output. `GPIO4` is owned by `track_output` as the fast
/// DRV8874 EN/IN1 RailCom cutout control.
pub const RAILCOM_UART_RX_GPIO_NUM: u8 = 5;
pub const RAILCOM_UART_BAUDRATE: u32 = 250_000;
const RAILCOM_UART_DRAIN_CHUNK: usize = 16;
// Hard cap on drain iterations so a wedged FIFO (or sustained DCC-edge noise)
// cannot keep us inside `drain_uart_rx_fifo` past the 460 µs cutout budget.
// ESP32-C6 RX FIFO is 128 bytes → 8 × 16-byte chunks is a complete drain.
const RAILCOM_UART_DRAIN_MAX_ITERATIONS: usize = 8;

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomRxOutput {
    WindowProcessed(RailcomRxResult),
    WindowCorrupted(RailcomCorruptedWindow),
    WindowError(RailcomUartWindowError),
    AdapterError(RailcomUartAdapterError),
}

/// Per-byte UART noise observed while a RailCom window was open, tagged with
/// the kind of disturbance and the context needed by downstream logging.
///
/// `kind` reuses the same `RailcomUartWindowInputError` variants the adapter
/// consumes, so the reader and the adapter stay in lock-step by construction:
/// a new disturbance variant can't be added on one side without the compiler
/// flagging the missing match arm on the other.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomUartReaderNoise {
    pub kind: RailcomUartWindowInputError,
    pub packet_sequence: u32,
    pub channel: RailcomChannel,
    pub after_first_byte: bool,
}

#[cfg(target_arch = "riscv32")]
pub type RailcomUartRuntimeResultChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, RailcomRxOutput, 8>;

#[cfg(target_arch = "riscv32")]
#[must_use]
pub fn railcom_uart_rx_config() -> Config {
    Config::default()
        .with_baudrate(RAILCOM_UART_BAUDRATE)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1)
        .with_rx(
            RxConfig::default()
                .with_fifo_full_threshold(1)
                .with_timeout(1),
        )
}

#[cfg(target_arch = "riscv32")]
fn drain_uart_rx_fifo(uart_rx: &mut UartRx<'static, Async>) {
    let mut drain_buf = [0u8; RAILCOM_UART_DRAIN_CHUNK];

    // Tolerate errors during drain: the FIFO is full of DCC-period garbage
    // and may have overflow/glitch flags set. Clear everything silently.
    let _ = uart_rx.check_for_errors();
    for _ in 0..RAILCOM_UART_DRAIN_MAX_ITERATIONS {
        match uart_rx.read_buffered(&mut drain_buf) {
            Ok(0) => return,
            Ok(_) => continue,
            Err(_) => {
                // Error flag was set — clear it and bail; a sustained
                // noise source will be handled on the next window.
                let _ = uart_rx.check_for_errors();
                return;
            }
        }
    }
}

#[cfg(target_arch = "riscv32")]
async fn send_runtime_output(
    sender: &Sender<'static, CriticalSectionRawMutex, RailcomUartRuntimeOutput, 8>,
    output: RailcomUartRuntimeOutput,
) {
    sender.send(output).await;
}

/// UART-side RailCom reader task.
///
/// This task remains outside the RMT and cutout ISR paths. Another module is
/// expected to send `Open/Close` window commands derived from the actual cutout
/// timing; this task only owns UART RX, drains stale FIFO bytes before opening
/// a new window, forwards received bytes to the software adapter, and reports
/// structured reader/adapter outcomes.
#[cfg(target_arch = "riscv32")]
pub async fn railcom_uart_reader_task(
    mut uart_rx: UartRx<'static, Async>,
    control_receiver: Receiver<'static, CriticalSectionRawMutex, RailcomWindowControl, 8>,
    result_sender: Sender<'static, CriticalSectionRawMutex, RailcomUartRuntimeOutput, 8>,
) -> ! {
    let mut adapter = RailcomUartAdapter::new();
    let mut byte_buf = [0u8; 1];

    loop {
        let open = match control_receiver.receive().await {
            RailcomWindowControl::Open {
                packet_sequence,
                channel,
            } => (packet_sequence, channel),
            RailcomWindowControl::Close => {
                send_runtime_output(
                    &result_sender,
                    RailcomUartRuntimeOutput::ReaderError(RailcomUartReaderError::CloseWithoutOpen),
                )
                .await;
                continue;
            }
        };

        // Drain stale bytes. Tolerates errors from DCC-period noise.
        drain_uart_rx_fifo(&mut uart_rx);

        if let Some(output) = adapter.handle_event(RailcomUartEvent::BeginWindow {
            packet_sequence: open.0,
            channel: open.1,
        }) {
            send_runtime_output(&result_sender, RailcomUartRuntimeOutput::Adapter(output)).await;
        }

        let mut saw_byte = false;

        loop {
            match select(
                control_receiver.receive(),
                uart_rx.read_async(&mut byte_buf),
            )
            .await
            {
                Either::First(RailcomWindowControl::Close) => {
                    if let Some(output) = adapter.handle_event(RailcomUartEvent::EndWindow) {
                        send_runtime_output(
                            &result_sender,
                            RailcomUartRuntimeOutput::Adapter(output),
                        )
                        .await;
                    }
                    break;
                }
                Either::First(RailcomWindowControl::Open {
                    packet_sequence,
                    channel,
                }) => {
                    send_runtime_output(
                        &result_sender,
                        RailcomUartRuntimeOutput::ReaderError(RailcomUartReaderError::NestedOpen {
                            active_packet_sequence: open.0,
                            active_channel: open.1,
                            new_packet_sequence: packet_sequence,
                            new_channel: channel,
                        }),
                    )
                    .await;
                }
                Either::Second(Ok(read)) => {
                    for &byte in &byte_buf[..read] {
                        saw_byte = true;
                        if let Some(output) = adapter.handle_event(RailcomUartEvent::Byte(byte)) {
                            send_runtime_output(
                                &result_sender,
                                RailcomUartRuntimeOutput::Adapter(output),
                            )
                            .await;
                        }
                    }
                }
                Either::Second(Err(err)) => {
                    // Mid-window error handling must stay inside the 460 µs
                    // cutout budget. check_for_errors() is a single MMIO and
                    // clears the error flag so read_async can progress on the
                    // next byte; any remaining corrupt bytes are drained by
                    // the window-start drain on the *next* Open, which runs
                    // outside the cutout.
                    let _ = uart_rx.check_for_errors();
                    match err {
                        RxError::FifoOverflowed => {
                            crate::railcom::pipeline::record_uart_overflow();
                            send_runtime_output(
                                &result_sender,
                                RailcomUartRuntimeOutput::ReaderError(RailcomUartReaderError::Rx(
                                    err,
                                )),
                            )
                            .await;
                            // Data irrecoverable — abandon the window.
                            adapter = RailcomUartAdapter::new();
                            break;
                        }
                        RxError::GlitchOccurred | RxError::FrameFormatViolated => {
                            let kind = match err {
                                RxError::GlitchOccurred => {
                                    crate::railcom::pipeline::record_uart_glitch();
                                    RailcomUartWindowInputError::Glitch
                                }
                                RxError::FrameFormatViolated => {
                                    crate::railcom::pipeline::record_uart_framing_error();
                                    RailcomUartWindowInputError::Framing
                                }
                                _ => unreachable!("outer match arm limits err to these two"),
                            };
                            let _ = adapter.handle_event(RailcomUartEvent::InputError(kind));
                            send_runtime_output(
                                &result_sender,
                                RailcomUartRuntimeOutput::ReaderNoise(RailcomUartReaderNoise {
                                    kind,
                                    packet_sequence: open.0,
                                    channel: open.1,
                                    after_first_byte: saw_byte,
                                }),
                            )
                            .await;
                            continue;
                        }
                        _ => {
                            send_runtime_output(
                                &result_sender,
                                RailcomUartRuntimeOutput::ReaderError(RailcomUartReaderError::Rx(
                                    err,
                                )),
                            )
                            .await;
                            continue;
                        }
                    }
                }
            }
        }
    }
}
