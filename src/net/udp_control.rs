//! Embassy WiFi + Z21 UDP control task.
//!
//! ESP32-C6 only — gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.
//!
//! This is the wiring module: it binds the UDP socket, drives the
//! `select3` event loop, and constructs [`Z21Ctx`] per incoming frame. Command
//! dispatch lives in `z21_dispatch`, WiFi bring-up in `wifi`, POM client logic
//! in `pom_client`, and RailCom sighting lookups in `railcom_lookup`.

use core::cell::Cell;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Instant, Timer};
use esp_hal::rng::Rng;
use static_cell::StaticCell;

use crate::config::Z21_KEEPALIVE_TIMEOUT_MS;
use crate::dcc::{LogicalSpeed, PomRequest, PomResponse, SchedulerCommand};
use crate::net::wifi_config::WifiCredentials;
use crate::net::z21_dispatch::{encode_current_system_state, handle_packet, handle_status_event};
use crate::net::z21_proto::{self, HEADER_SYSTEMSTATE_GETDATA, HEADER_XBUS};
use crate::net::{LocoSlots, loco_is_moving};
use crate::system_status::{DisplayEvent, FaultEvent, StatusModel, SystemStatusEvent};

pub use super::wifi::NetInitError;
use super::{radio, wifi};

const Z21_PORT: u16 = 21105;
const DECEL_STEP_MS: u64 = 500;

// Static buffers — avoids heap allocation in the hot UDP path.
static RX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
static TX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
static TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
static NET_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

pub struct NetTaskChannels {
    pub net_status: Receiver<'static, CriticalSectionRawMutex, SystemStatusEvent, 8>,
    pub status_sender: Sender<'static, CriticalSectionRawMutex, SystemStatusEvent, 16>,
    pub display_sender: Sender<'static, CriticalSectionRawMutex, DisplayEvent, 8>,
    pub ready_sender:
        Sender<'static, CriticalSectionRawMutex, crate::system_status::BootReadyEvent, 9>,
    pub pom_request_sender: Sender<'static, CriticalSectionRawMutex, PomRequest, 1>,
    pub pom_response_receiver: Receiver<'static, CriticalSectionRawMutex, PomResponse, 1>,
}

/// Shared dependencies threaded through the Z21 command handlers.
///
/// Grouping the channels and shared state here avoids a 7-9 argument sprawl
/// on every handler; per-call buffers (`out`, `loco_slots`) stay explicit
/// because they are mutated per invocation. Visible to sibling `net`
/// submodules (`z21_dispatch`, `pom_client`) that implement the handlers.
pub(super) struct Z21Ctx<'a> {
    pub(super) scheduler_sender: &'a Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
    pub(super) fault_sender: &'a Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>,
    pub(super) status_model: &'a StatusModel,
    pub(super) pom_request_sender: &'a Sender<'static, CriticalSectionRawMutex, PomRequest, 1>,
    pub(super) pom_response_receiver:
        &'a Receiver<'static, CriticalSectionRawMutex, PomResponse, 1>,
    /// Local (non-atomic) POM request id counter — this task is the sole
    /// producer of POM requests, so a `Cell` is enough.
    pub(super) next_pom_request_id: &'a Cell<u32>,
}

/// Main net task — WiFi init, DHCP, Z21 UDP control loop.
///
/// Spawns `wifi::wifi_runner_task` and `wifi::connection_task` internally,
/// following the official esp-hal embassy WiFi example pattern.
pub async fn net_task(
    spawner: Spawner,
    wifi: esp_hal::peripherals::WIFI<'static>,
    credentials: WifiCredentials,
    scheduler_sender: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
    fault_sender: Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>,
    channels: NetTaskChannels,
) -> Result<(), NetInitError> {
    let NetTaskChannels {
        net_status,
        status_sender,
        display_sender,
        ready_sender,
        pom_request_sender,
        pom_response_receiver,
    } = channels;

    let controller = radio::init_controller().map_err(NetInitError::EspRadioInit)?;

    let (mut wifi_ctrl, interfaces) =
        esp_radio::wifi::new(controller, wifi, esp_radio::wifi::Config::default())
            .map_err(|_| NetInitError::WifiInit)?;

    wifi_ctrl
        .set_config(&wifi::client_mode_config(&credentials))
        .map_err(|_| NetInitError::WifiSetConfig)?;
    wifi_ctrl
        .start_async()
        .await
        .map_err(|_| NetInitError::WifiStart)?;
    info!("WiFi started, connecting to SSID: {}", credentials.ssid());

    let net_config = Config::dhcpv4(Default::default());
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let resources = NET_RESOURCES.init(StackResources::new());
    let (stack, runner) = embassy_net::new(interfaces.sta, net_config, resources, seed);

    // Spawn runner and connection tasks — same pattern as official examples.
    spawner
        .spawn(wifi::wifi_runner_task(runner))
        .map_err(|_| NetInitError::WifiRunnerSpawn)?;
    spawner
        .spawn(wifi::connection_task(wifi_ctrl, status_sender))
        .map_err(|_| NetInitError::ConnectionSpawn)?;

    info!("Waiting for DHCP...");
    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        info!(
            "Network up — IP: {}",
            defmt::Display2Format(&config.address.address())
        );
        let addr = config.address.address();
        let _ = display_sender.try_send(DisplayEvent::IpAssigned(addr.octets()));
        let _ = display_sender.try_send(DisplayEvent::BootProgress(
            crate::system_status::BootStep::WifiConnected,
        ));
    } else {
        info!("Network up — DHCP configured (no IPv4?)");
    }

    // Set up UDP socket.
    let rx_meta = RX_META.init([PacketMetadata::EMPTY; 16]);
    let rx_buf = RX_BUF.init([0u8; 1024]);
    let tx_meta = TX_META.init([PacketMetadata::EMPTY; 16]);
    let tx_buf = TX_BUF.init([0u8; 1024]);

    let mut socket = UdpSocket::new(stack, rx_meta, rx_buf, tx_meta, tx_buf);
    socket.bind(Z21_PORT).map_err(|_| NetInitError::UdpBind)?;
    info!("Z21 UDP listening on port {}", Z21_PORT);
    ready_sender
        .send(crate::system_status::BootReadyEvent::Net)
        .await;

    let mut loco_slots: LocoSlots = [None; 12];
    let mut client_endpoint: Option<embassy_net::IpEndpoint> = None;
    let mut last_rx_ms: u64 = 0;
    let mut last_decel_ms: u64 = 0;
    let mut all_stopped = false;
    let mut status_model = StatusModel::new();
    let next_pom_request_id = Cell::new(1u32);

    let mut recv_buf = [0u8; 256];
    let mut send_buf = [0u8; 64];

    loop {
        let now_ms = Instant::now().as_millis();

        let next_wake = if client_endpoint.is_some()
            && now_ms.saturating_sub(last_rx_ms) >= Z21_KEEPALIVE_TIMEOUT_MS
            && !all_stopped
        {
            Instant::from_millis(last_decel_ms + DECEL_STEP_MS)
        } else {
            Instant::from_millis(last_rx_ms + Z21_KEEPALIVE_TIMEOUT_MS + 1)
        };

        use embassy_futures::select::{Either3, select3};

        match select3(
            socket.recv_from(&mut recv_buf),
            net_status.receive(),
            Timer::at(next_wake),
        )
        .await
        {
            // ── Incoming UDP packet ──────────────────────────────────
            Either3::First(Ok((n, meta))) => {
                let header = if n >= 4 {
                    u16::from_le_bytes([recv_buf[2], recv_buf[3]])
                } else {
                    0
                };
                // Suppress per-second polling noise (SystemState, XBus status)
                let is_polling = header == HEADER_SYSTEMSTATE_GETDATA || header == HEADER_XBUS;
                if !is_polling {
                    let xheader = if header == HEADER_XBUS && n >= 5 {
                        recv_buf[4]
                    } else {
                        0
                    };
                    info!(
                        "UDP rx {} bytes from {} — header=0x{:04X} xheader=0x{:02X}",
                        n,
                        defmt::Display2Format(&meta.endpoint),
                        header,
                        xheader
                    );
                }

                last_rx_ms = Instant::now().as_millis();
                client_endpoint = Some(meta.endpoint);
                all_stopped = false;

                // A Z21 UDP datagram may contain multiple concatenated frames.
                // Process each frame and send its response individually.
                let ctx = Z21Ctx {
                    scheduler_sender: &scheduler_sender,
                    fault_sender: &fault_sender,
                    status_model: &status_model,
                    pom_request_sender: &pom_request_sender,
                    pom_response_receiver: &pom_response_receiver,
                    next_pom_request_id: &next_pom_request_id,
                };
                for frame in z21_proto::iter_frames(&recv_buf[..n]) {
                    let resp_len = handle_packet(frame, &mut loco_slots, &mut send_buf, &ctx).await;
                    if resp_len > 0
                        && socket
                            .send_to(&send_buf[..resp_len], meta.endpoint)
                            .await
                            .is_err()
                    {
                        warn!("Z21 UDP send failed");
                    }
                }
            }
            Either3::First(Err(_)) => {
                // Receive error — continue.
            }

            // ── System status event ──────────────────────────────────
            Either3::Second(event) => {
                status_model.apply(event);
                if let Some(ep) = client_endpoint {
                    if matches!(event, SystemStatusEvent::FaultLatched(_)) {
                        let n = z21_proto::encode_bc_track_power(false, &mut send_buf);
                        let _ = socket.send_to(&send_buf[..n], ep).await;
                        let n = z21_proto::encode_bc_stopped(&mut send_buf);
                        let _ = socket.send_to(&send_buf[..n], ep).await;
                        let n = encode_current_system_state(&status_model, &mut send_buf);
                        let _ = socket.send_to(&send_buf[..n], ep).await;
                    } else if matches!(event, SystemStatusEvent::FaultCleared) {
                        let n = z21_proto::encode_bc_track_power(true, &mut send_buf);
                        let _ = socket.send_to(&send_buf[..n], ep).await;
                        let n = encode_current_system_state(&status_model, &mut send_buf);
                        let _ = socket.send_to(&send_buf[..n], ep).await;
                    } else {
                        let n = handle_status_event(event, &mut send_buf);
                        if n > 0 {
                            let _ = socket.send_to(&send_buf[..n], ep).await;
                        }
                    }
                }
            }

            // ── Timer tick (keepalive deceleration) ──────────────────
            Either3::Third(_) => {
                if let Some(ep) = client_endpoint {
                    let elapsed = Instant::now().as_millis().saturating_sub(last_rx_ms);
                    if elapsed >= Z21_KEEPALIVE_TIMEOUT_MS {
                        last_decel_ms = Instant::now().as_millis();
                        let any_moving =
                            handle_keepalive_timeout(&mut loco_slots, &scheduler_sender).await;
                        if !any_moving && !all_stopped {
                            all_stopped = true;
                            scheduler_sender
                                .send(SchedulerCommand::EmergencyStopAll)
                                .await;
                            let n = z21_proto::encode_bc_stopped(&mut send_buf);
                            let _ = socket.send_to(&send_buf[..n], ep).await;
                            client_endpoint = None;
                            info!("Client keepalive timeout: all locos stopped");
                        }
                    }
                }
            }
        }
    }
}

async fn handle_keepalive_timeout(
    loco_slots: &mut LocoSlots,
    scheduler_sender: &Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
) -> bool {
    let mut any_moving = false;
    for slot in loco_slots.iter_mut().flatten() {
        if loco_is_moving(slot.speed, slot.format) {
            slot.speed -= 1;
            any_moving = true;
            let Some(speed) = LogicalSpeed::new(slot.speed, slot.format) else {
                warn!(
                    "keepalive decel addr={} speed={} invalid for format={:?}, skipping",
                    slot.address.value(),
                    slot.speed,
                    slot.format
                );
                continue;
            };
            scheduler_sender
                .send(SchedulerCommand::SetSpeed {
                    address: slot.address,
                    speed,
                    direction: slot.direction,
                    format: slot.format,
                })
                .await;
        }
    }
    any_moving
}
