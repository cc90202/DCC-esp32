use core::fmt;
#[cfg(target_arch = "riscv32")]
use esp_hal::uart::{Config, Parity, RxConfig, StopBits};

use crate::railcom::pipeline::RailcomRxResult;
use crate::railcom::{PacketSequence, RailcomChannel};

/// RailCom UART baud rate mandated by the protocol.
pub const RAILCOM_UART_BAUDRATE: u32 = 250_000;

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomRxOutput {
    WindowProcessed(RailcomRxResult),
    WindowError(RailcomUartWindowError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomUartWindowError {
    WindowTooLong {
        packet_sequence: PacketSequence,
        channel: RailcomChannel,
        provided_len: usize,
        max_len: usize,
    },
}

impl fmt::Display for RailcomUartWindowError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WindowTooLong {
                packet_sequence,
                channel,
                provided_len,
                max_len,
            } => write!(
                formatter,
                "RailCom window {packet_sequence:?}/{channel:?} has {provided_len} bytes; maximum is {max_len}"
            ),
        }
    }
}

impl core::error::Error for RailcomUartWindowError {}

/// Captured receive windows queued between the capture ISR and the dispatch
/// task. The depth is a concurrency invariant: it decides when the capture side
/// blocks. The channel and the sender below share this constant; the receiver
/// side is still spelled out by hand in `runtime_dispatch`.
pub(crate) const RX_WINDOW_DEPTH: usize = 8;

pub(crate) type RailcomUartRuntimeResultChannel =
    crate::runtime_channels::RuntimeChannel<RailcomRxOutput, RX_WINDOW_DEPTH>;
pub(crate) type RailcomRxOutputSender =
    crate::runtime_channels::RuntimeSender<RailcomRxOutput, RX_WINDOW_DEPTH>;
pub(crate) type RailcomRxOutputReceiver =
    crate::runtime_channels::RuntimeReceiver<RailcomRxOutput, RX_WINDOW_DEPTH>;

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
