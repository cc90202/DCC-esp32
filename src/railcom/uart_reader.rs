use core::fmt;
#[cfg(target_arch = "riscv32")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
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
