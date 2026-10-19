//! WiFi bring-up: esp-radio Embassy tasks, connection state machine, `NetInitError`.
//!
//! ESP32-C6 only, gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

extern crate alloc;

use alloc::string::ToString;
use core::fmt;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::{Config, DhcpConfig, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_radio::wifi::{
    AuthMethod, ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent,
};
use static_cell::StaticCell;

use super::radio::{self, WifiBringupError};
use crate::net::wifi_config::WifiCredentials;
use crate::station_status::SystemStatusEvent;
use crate::system_status::DisplayEvent;
use crate::system_status::channels::{DisplaySender, SystemStatusSender};

static NET_RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();

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
        defmt::write!(formatter, "{}", defmt::Display2Format(self));
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
        defmt::write!(formatter, "{}", defmt::Display2Format(self));
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

/// Starts the radio and the embassy-net stack, spawns the WiFi tasks and waits for DHCP.
pub(super) async fn start_network_stack(
    spawner: Spawner,
    wifi: esp_hal::peripherals::WIFI<'static>,
    credentials: &WifiCredentials,
    status_sender: SystemStatusSender,
    display_sender: DisplaySender,
) -> Result<Stack<'static>, NetInitError> {
    let (wifi_ctrl, interfaces) = radio::start_wifi(wifi, &client_mode_config(credentials))
        .await
        .map_err(NetInitError::WifiBringup)?;
    info!("WiFi started, connecting to SSID: {}", credentials.ssid());

    let net_config = Config::dhcpv4(DhcpConfig::default());
    let seed = radio::stack_seed();
    let resources = NET_RESOURCES.init(StackResources::new());
    let (stack, runner) = embassy_net::new(interfaces.sta, net_config, resources, seed);

    spawner
        .spawn(wifi_runner_task(runner))
        .map_err(|_| NetInitError::WifiRunnerSpawn)?;
    spawner
        .spawn(connection_task(wifi_ctrl, status_sender))
        .map_err(|_| NetInitError::ConnectionSpawn)?;

    info!("Waiting for DHCP...");
    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        let address = config.address.address();
        info!("Network up, IP: {}", defmt::Display2Format(&address));
        let _ = display_sender.try_send(DisplayEvent::IpAssigned(address.octets()));
        let _ = display_sender.try_send(DisplayEvent::BootProgress(
            crate::system_status::BootStep::WifiConnected,
        ));
    } else {
        info!("Network up, DHCP configured (no IPv4?)");
    }

    Ok(stack)
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
    status_sender: SystemStatusSender,
) {
    let mut state = ConnectionState::Connecting;

    loop {
        match state {
            ConnectionState::Connecting => {
                status_sender.send(SystemStatusEvent::WifiConnecting).await;
                if controller.connect_async().await.is_ok() {
                    info!("WiFi connected");
                    status_sender.send(SystemStatusEvent::WifiConnected).await;
                    state = ConnectionState::Connected;
                } else {
                    crate::liveness::network_ready(false);
                    warn!("WiFi connect failed, retrying in 5s");
                    status_sender
                        .send(SystemStatusEvent::WifiDisconnected)
                        .await;
                    Timer::after(Duration::from_secs(5)).await;
                }
            }
            ConnectionState::Connected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                crate::liveness::network_ready(false);
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
