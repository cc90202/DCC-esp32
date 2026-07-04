//! SoftAP bring-up for WiFi provisioning mode.

extern crate alloc;

use alloc::string::ToString;
use core::fmt::Write;

use defmt::info;
use embassy_executor::Spawner;
use embassy_net::{Config, Ipv4Address, Ipv4Cidr, StackResources, StaticConfigV4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use esp_hal::rng::Rng;
use esp_radio::wifi::{AccessPointConfig, AuthMethod, ModeConfig};
use heapless::String;
use static_cell::StaticCell;

use crate::net::radio::{self, WifiBringupError};
use crate::net::wifi::wifi_runner_task;
use crate::net::wifi_config::WifiCredentialsStore;
use crate::system_status::DisplayEvent;

use super::dhcp_server::run_dhcp_server;
use super::http_server::run_http_server;
use super::net_config::{PREFIX_LEN, SERVER_IP_OCTETS, SETUP_URL};

const AP_SSID_PREFIX: &str = "DCC-Setup-";
const AP_PASSWORD: &str = "dcc-setup";

static PROVISIONING_NET_RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum ProvisioningApError {
    WifiBringup(WifiBringupError),
    WifiRunnerSpawn,
    DhcpServerSpawn,
    SsidBuild,
}

impl ProvisioningApError {
    pub(crate) const fn as_str(self) -> &'static str {
        match self {
            Self::WifiBringup(error) => error.as_str(),
            Self::WifiRunnerSpawn => "failed to spawn provisioning wifi_runner_task",
            Self::DhcpServerSpawn => "failed to spawn provisioning DHCP server",
            Self::SsidBuild => "WiFi AP SSID build failed",
        }
    }
}

fn provisioning_ssid() -> Result<String<14>, ProvisioningApError> {
    let mac = esp_radio::wifi::ap_mac();
    let mut ssid = String::new();
    write!(&mut ssid, "{}{:02X}{:02X}", AP_SSID_PREFIX, mac[4], mac[5])
        .map_err(|_| ProvisioningApError::SsidBuild)?;
    Ok(ssid)
}

fn access_point_mode_config(ssid: &str) -> ModeConfig {
    ModeConfig::AccessPoint(
        AccessPointConfig::default()
            .with_ssid(ssid.to_string())
            .with_password(AP_PASSWORD.to_string())
            .with_auth_method(AuthMethod::Wpa2Personal)
            .with_max_connections(1),
    )
}

fn provisioning_net_config() -> Config {
    Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(
            Ipv4Address::new(
                SERVER_IP_OCTETS[0],
                SERVER_IP_OCTETS[1],
                SERVER_IP_OCTETS[2],
                SERVER_IP_OCTETS[3],
            ),
            PREFIX_LEN,
        ),
        gateway: None,
        dns_servers: Default::default(),
    })
}

/// Starts provisioning SoftAP mode and keeps the AP stack alive.
///
/// This safe setup mode does not start DCC, RailCom, Z21, or track output.
pub async fn run_provisioning_ap<S>(
    spawner: Spawner,
    wifi_peripheral: esp_hal::peripherals::WIFI<'static>,
    mut store: S,
    display_sender: Sender<'static, CriticalSectionRawMutex, DisplayEvent, 8>,
) -> Result<(), ProvisioningApError>
where
    S: WifiCredentialsStore,
{
    let ssid = provisioning_ssid()?;

    // The controller must stay alive for the whole provisioning session;
    // dropping it would stop the radio.
    let (_wifi_ctrl, interfaces) =
        radio::start_wifi(wifi_peripheral, &access_point_mode_config(ssid.as_str()))
            .await
            .map_err(ProvisioningApError::WifiBringup)?;

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let resources = PROVISIONING_NET_RESOURCES.init(StackResources::new());
    let (stack, runner) =
        embassy_net::new(interfaces.ap, provisioning_net_config(), resources, seed);

    spawner
        .spawn(wifi_runner_task(runner))
        .map_err(|_| ProvisioningApError::WifiRunnerSpawn)?;

    stack.wait_config_up().await;
    spawner
        .spawn(run_dhcp_server(stack))
        .map_err(|_| ProvisioningApError::DhcpServerSpawn)?;

    info!("WiFi provisioning AP started");
    info!("AP SSID: {}", ssid.as_str());
    info!("Provisioning HTTP ready after DHCP lease: {}", SETUP_URL);
    display_sender
        .send(DisplayEvent::ProvisioningMode {
            ssid,
            setup_url: SETUP_URL,
        })
        .await;

    // Never returns: the `!` type coerces into the `Result`, so `Ok` is
    // unreachable and this function only ever exits through the `?` above.
    run_http_server(stack, &mut store).await;
}
