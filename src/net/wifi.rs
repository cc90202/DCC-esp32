//! WiFi bring-up: esp-radio Embassy tasks, connection state machine, `NetInitError`.
//!
//! ESP32-C6 only, gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

extern crate alloc;

use alloc::string::ToString;
use core::fmt;
use defmt::{info, warn};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Timer};
use esp_radio::wifi::{
    AuthMethod, ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent,
};

use super::radio::WifiBringupError;
use crate::net::wifi_config::WifiCredentials;
use crate::system_status::SystemStatusEvent;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConnectionState {
    Connecting,
    Connected,
}

/// Preserved embassy-net failure returned while binding the Z21 UDP socket.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UdpBindError(embassy_net::udp::BindError);

impl From<embassy_net::udp::BindError> for UdpBindError {
    fn from(error: embassy_net::udp::BindError) -> Self {
        Self(error)
    }
}

impl fmt::Display for UdpBindError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "embassy-net UDP bind error: {:?}", self.0)
    }
}

impl core::error::Error for UdpBindError {}

impl defmt::Format for UdpBindError {
    fn format(&self, formatter: defmt::Formatter) {
        defmt::write!(formatter, "{:?}", defmt::Debug2Format(&self.0));
    }
}

#[derive(Debug, Clone, Copy)]
pub enum NetInitError {
    WifiBringup(WifiBringupError),
    WifiRunnerSpawn,
    ConnectionSpawn,
    HttpSpawn,
    UdpBind(UdpBindError),
}

impl defmt::Format for NetInitError {
    fn format(&self, formatter: defmt::Formatter) {
        match self {
            Self::WifiBringup(error) => defmt::write!(formatter, "{:?}", error),
            Self::WifiRunnerSpawn => defmt::write!(formatter, "wifi runner spawn failed"),
            Self::ConnectionSpawn => defmt::write!(formatter, "WiFi connection spawn failed"),
            Self::HttpSpawn => defmt::write!(formatter, "OTA HTTP spawn failed"),
            Self::UdpBind(error) => defmt::write!(formatter, "UDP bind failed: {:?}", error),
        }
    }
}

impl fmt::Display for NetInitError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WifiBringup(error) => write!(formatter, "{error}"),
            Self::WifiRunnerSpawn => formatter.write_str("failed to spawn wifi_runner_task"),
            Self::ConnectionSpawn => formatter.write_str("failed to spawn connection_task"),
            Self::HttpSpawn => formatter.write_str("failed to spawn OTA HTTP task"),
            Self::UdpBind(error) => write!(formatter, "UDP bind on Z21 port failed: {error:?}"),
        }
    }
}

impl core::error::Error for NetInitError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::WifiBringup(error) => Some(error),
            Self::UdpBind(error) => Some(error),
            Self::WifiRunnerSpawn | Self::ConnectionSpawn | Self::HttpSpawn => None,
        }
    }
}

/// Build the WiFi client mode config from validated runtime credentials.
pub(super) fn client_mode_config(credentials: &WifiCredentials) -> ModeConfig {
    ModeConfig::Client(
        ClientConfig::default()
            .with_ssid(credentials.ssid().to_string())
            .with_password(credentials.password().to_string())
            .with_auth_method(AuthMethod::Wpa2Personal),
    )
}

/// Runs the embassy-net stack driver in a dedicated Embassy task, following the official example.
#[embassy_executor::task]
pub(super) async fn wifi_runner_task(
    mut runner: embassy_net::Runner<'static, WifiDevice<'static>>,
) {
    runner.run().await
}

/// Handles WiFi connect and automatic reconnect.
#[embassy_executor::task]
pub(super) async fn connection_task(
    mut controller: WifiController<'static>,
    status_sender: Sender<'static, CriticalSectionRawMutex, SystemStatusEvent, 16>,
) {
    let mut state = ConnectionState::Connecting;

    loop {
        match state {
            ConnectionState::Connecting => {
                status_sender.send(SystemStatusEvent::WifiConnecting).await;
                match controller.connect_async().await {
                    Ok(_) => {
                        info!("WiFi connected");
                        status_sender.send(SystemStatusEvent::WifiConnected).await;
                        state = ConnectionState::Connected;
                    }
                    Err(_) => {
                        crate::ota::runtime::network_ready(false);
                        warn!("WiFi connect failed, retrying in 5s");
                        status_sender
                            .send(SystemStatusEvent::WifiDisconnected)
                            .await;
                        Timer::after(Duration::from_secs(5)).await;
                    }
                }
            }
            ConnectionState::Connected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                crate::ota::runtime::network_ready(false);
                warn!("WiFi disconnected, reconnecting in 5s...");
                status_sender
                    .send(SystemStatusEvent::WifiDisconnected)
                    .await;
                Timer::after(Duration::from_secs(5)).await;
                state = ConnectionState::Connecting;
            }
        }
    }
}
