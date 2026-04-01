#[cfg(target_arch = "riscv32")]
use embassy_futures::select::{Either, select};
#[cfg(target_arch = "riscv32")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(target_arch = "riscv32")]
use embassy_sync::channel::{Receiver, Sender};
#[cfg(target_arch = "riscv32")]
use esp_hal::Async;
#[cfg(target_arch = "riscv32")]
use esp_hal::uart::{Config, Parity, RxConfig, RxError, StopBits, UartRx};

use crate::railcom::pipeline::RailcomChannel;
use crate::railcom::uart_adapter::{
    RailcomUartAdapter, RailcomUartAdapterOutput, RailcomUartEvent,
};

/// Preferred UART RX pin for experimental RailCom reception.
///
/// The current board review leaves `GPIO5` free and it is the preferred input
/// for the detector output. `GPIO4` remains free as a useful debug/marker pin.
pub const RAILCOM_UART_RX_GPIO_NUM: u8 = 5;
pub const RAILCOM_DEBUG_GPIO_NUM: u8 = 4;
pub const RAILCOM_UART_BAUDRATE: u32 = 250_000;
const RAILCOM_UART_DRAIN_CHUNK: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomWindowControl {
    Open {
        packet_sequence: u32,
        channel: RailcomChannel,
    },
    Close,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomUartReaderError {
    CloseWithoutOpen,
    NestedOpen {
        active_packet_sequence: u32,
        active_channel: RailcomChannel,
        new_packet_sequence: u32,
        new_channel: RailcomChannel,
    },
    Rx(RxError),
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomUartRuntimeOutput {
    Adapter(RailcomUartAdapterOutput),
    ReaderError(RailcomUartReaderError),
}

#[cfg(target_arch = "riscv32")]
pub type RailcomWindowControlChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, RailcomWindowControl, 8>;

#[cfg(target_arch = "riscv32")]
pub type RailcomUartRuntimeResultChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, RailcomUartRuntimeOutput, 8>;

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
fn drain_uart_rx_fifo(uart_rx: &mut UartRx<'static, Async>) -> Result<(), RxError> {
    let mut drain_buf = [0u8; RAILCOM_UART_DRAIN_CHUNK];

    while uart_rx.read_ready() {
        let read = uart_rx.read_buffered(&mut drain_buf)?;
        if read == 0 {
            break;
        }
    }

    Ok(())
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

        if let Err(err) = drain_uart_rx_fifo(&mut uart_rx) {
            send_runtime_output(
                &result_sender,
                RailcomUartRuntimeOutput::ReaderError(RailcomUartReaderError::Rx(err)),
            )
            .await;
            adapter = RailcomUartAdapter::new();
            continue;
        }

        if let Some(output) = adapter.handle_event(RailcomUartEvent::BeginWindow {
            packet_sequence: open.0,
            channel: open.1,
        }) {
            send_runtime_output(&result_sender, RailcomUartRuntimeOutput::Adapter(output)).await;
        }

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
                    send_runtime_output(
                        &result_sender,
                        RailcomUartRuntimeOutput::ReaderError(RailcomUartReaderError::Rx(err)),
                    )
                    .await;
                    let _ = drain_uart_rx_fifo(&mut uart_rx);
                    adapter = RailcomUartAdapter::new();
                    break;
                }
            }
        }
    }
}
