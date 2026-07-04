//! WiFi bring-up: esp-radio Embassy tasks, connection state machine, `NetInitError`.
//!
//! ESP32-C6 only — gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

extern crate alloc;

use alloc::string::ToString;
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum NetInitError {
    WifiBringup(WifiBringupError),
    WifiRunnerSpawn,
    ConnectionSpawn,
    UdpBind,
}

impl NetInitError {
    pub(crate) const fn as_str(self) -> &'static str {
        match self {
            Self::WifiBringup(error) => error.as_str(),
            Self::WifiRunnerSpawn => "failed to spawn wifi_runner_task",
            Self::ConnectionSpawn => "failed to spawn connection_task",
            Self::UdpBind => "UDP bind on Z21 port failed",
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

/// Runs the embassy-net stack driver — dedicated Embassy task per official example.
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
