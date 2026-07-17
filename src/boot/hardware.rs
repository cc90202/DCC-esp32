//! ESP32-C6 peripheral initialization used by the boot composition root.

use esp_hal::gpio::{Flex, InputConfig, Level, Pull};
use esp_hal::rmt::{Rmt, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::UartRx;

use crate::dcc_runtime::build_idle_rmt_buffer;
use crate::railcom::uart_reader::railcom_uart_rx_config;
use crate::rmt_dcc as rmt_driver;

use super::{BootError, CriticalHardwareInit, DccSelfCheckError};

pub(super) fn start_async_runtime(
    timg0: esp_hal::peripherals::TIMG0<'static>,
    software_interrupt: esp_hal::peripherals::SW_INTERRUPT<'static>,
) {
    let timer_group = TimerGroup::new(timg0);
    let software_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(software_interrupt);
    esp_rtos::start(timer_group.timer0, software_interrupt.software_interrupt0);
}

pub(super) fn initialize_display_bus(
    i2c0: esp_hal::peripherals::I2C0<'static>,
    sda: esp_hal::peripherals::GPIO19<'static>,
    scl: esp_hal::peripherals::GPIO20<'static>,
) -> Result<esp_hal::i2c::master::I2c<'static, esp_hal::Async>, BootError> {
    esp_hal::i2c::master::I2c::new(
        i2c0,
        esp_hal::i2c::master::Config::default().with_frequency(esp_hal::time::Rate::from_khz(400)),
    )
    .map(|i2c| i2c.with_sda(sda).with_scl(scl).into_async())
    .map_err(|_| {
        BootError::OptionalPeripheralInit(crate::system_status::OptionalPeripheralInit::DisplayI2c)
    })
}

pub(super) fn initialize_dcc_rmt(
    rmt: esp_hal::peripherals::RMT<'static>,
    dcc_pin: esp_hal::peripherals::GPIO2<'static>,
) -> Result<(), BootError> {
    let rmt = Rmt::new(rmt, Rate::from_mhz(80))
        .map_err(|_| BootError::CriticalHardwareInit(CriticalHardwareInit::Rmt))?;

    // One 1 us RMT channel owns the DCC waveform. GPIO4 remains under the
    // timer-driven TrackOutput owner.
    let dcc_channel = rmt
        .channel0
        .configure_tx(
            dcc_pin,
            TxChannelConfig::default()
                .with_clk_divider(80)
                .with_idle_output_level(Level::Low)
                .with_idle_output(true)
                .with_memsize(3),
        )
        .map_err(|_| BootError::CriticalHardwareInit(CriticalHardwareInit::RmtChannel0))?;
    let idle_rmt = build_idle_rmt_buffer()
        .map_err(|_| BootError::DccSelfCheck(DccSelfCheckError::IdleWaveformBuild))?;

    rmt_driver::init(dcc_channel, idle_rmt.as_slice())
        .map_err(|_| BootError::CriticalHardwareInit(CriticalHardwareInit::RmtDriver))
}

pub(super) fn initialize_railcom_receiver(
    uart1: esp_hal::peripherals::UART1<'static>,
    rx_pin: esp_hal::peripherals::GPIO5<'static>,
) -> Result<UartRx<'static, esp_hal::Async>, BootError> {
    // The planned single 74HC14 gate drives GPIO5 LOW while RailCom is idle.
    // Keep that physical idle level deterministic while the detector is
    // disconnected; the input inverter then presents the UART with its normal
    // logical HIGH idle. Configure the pull before freezing the Flex as a
    // peripheral input, otherwise `UartRx::with_rx` cannot change it.
    let mut rx_pin = Flex::new(rx_pin);
    rx_pin.apply_input_config(&InputConfig::default().with_pull(Pull::Down));
    let rx_pin = rx_pin.peripheral_input().with_input_inverter(true);
    UartRx::new(uart1, railcom_uart_rx_config())
        .map_err(|_| BootError::CriticalHardwareInit(CriticalHardwareInit::RailcomUart))
        .map(|uart| uart.with_rx(rx_pin).into_async())
}
