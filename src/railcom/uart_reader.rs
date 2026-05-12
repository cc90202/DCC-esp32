#[cfg(target_arch = "riscv32")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(target_arch = "riscv32")]
use esp_hal::uart::{Config, Parity, RxConfig, StopBits};

use crate::railcom::pipeline::{RailcomChannel, RailcomRxResult};
use crate::railcom::uart_adapter::{RailcomUartAdapterError, RailcomUartWindowError};

/// Preferred UART RX pin for RailCom reception.
///
/// The current board review leaves `GPIO5` free and it is the preferred input
/// for the detector output. `GPIO4` is owned by `track_output` as the fast
/// DRV8874 EN/IN1 RailCom cutout control.
pub const RAILCOM_UART_RX_GPIO_NUM: u8 = 5;
pub const RAILCOM_UART_BAUDRATE: u32 = 250_000;

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomRxOutput {
    WindowProcessed(RailcomRxResult),
    WindowCorrupted(RailcomCorruptedWindow),
    WindowError(RailcomUartWindowError),
    AdapterError(RailcomUartAdapterError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomCorruptedWindow {
    pub packet_sequence: u32,
    pub channel: RailcomChannel,
    pub raw_len: usize,
    pub raw_bytes: [u8; 6],
    pub glitch_count: u8,
    pub framing_error_count: u8,
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
