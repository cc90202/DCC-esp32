//! Embassy WiFi + Z21 UDP control task.
//!
//! ESP32-C6 only, gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.
//!
//! This is the wiring module: it binds the UDP socket, drives the
//! `select3` event loop, and constructs [`Z21Ctx`] per incoming frame. Command
//! dispatch lives in `z21_dispatch`, WiFi bring-up in `wifi`, POM client logic
//! in `pom_client`, and RailCom sighting lookups in `railcom_lookup`.

use core::cell::Cell;
use core::sync::atomic::{AtomicU32, Ordering};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config, StackResources};
use embassy_time::{Instant, Timer};
use esp_hal::rng::Rng;
use static_cell::StaticCell;

use crate::application::LocoSlots;
use crate::application::StatusModel;
use crate::application::client_safety::ClientSafetyPolicy;
use crate::application::track_control::{StatusBroadcast, TrackStatus, plan_status_broadcast};
use crate::config::Z21_KEEPALIVE_TIMEOUT_MS;
use crate::net::wifi_config::WifiCredentials;
use crate::net::z21_context::{LocoCtx, PomCtx, TrackCtx, Z21Ctx};
use crate::net::z21_dispatch::{encode_system_state, handle_packet};
use crate::runtime_channels::{
    BootReadySender, DisplaySender, FaultEventSender, LocoRequestSender, LocoResponseReceiver,
    NetStatusReceiver, PomRequestSender, PomResponseReceiver, SchedulerCommandSender,
    SystemStatusSender, announce_ready,
};
use crate::system_status::{BootReadyEvent, DisplayEvent, FaultEvent};
use crate::z21::{self as z21_proto, HEADER_SYSTEMSTATE_GETDATA, HEADER_XBUS};

pub use super::wifi::NetInitError;
use super::{radio, wifi};

const Z21_PORT: u16 = 21105;
// Static buffers: avoids heap allocation in the hot UDP path.
static RX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
static TX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
static TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
static NET_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
static STATUS_BROADCAST_SEND_FAILURE_COUNT: AtomicU32 = AtomicU32::new(0);
static UDP_RECEIVE_FAILURE_COUNT: AtomicU32 = AtomicU32::new(0);

#[must_use]
pub(crate) fn status_broadcast_send_failure_count() -> u32 {
    STATUS_BROADCAST_SEND_FAILURE_COUNT.load(Ordering::Relaxed)
}

#[must_use]
pub(crate) fn udp_receive_failure_count() -> u32 {
    UDP_RECEIVE_FAILURE_COUNT.load(Ordering::Relaxed)
}

pub struct NetTaskChannels {
    pub net_status: NetStatusReceiver,
    pub status_sender: SystemStatusSender,
    pub display_sender: DisplaySender,
    pub ready_sender: BootReadySender,
    pub pom_request_sender: PomRequestSender,
    pub pom_response_receiver: PomResponseReceiver,
    pub loco_request_sender: LocoRequestSender,
    pub loco_response_receiver: LocoResponseReceiver,
}

/// Main net task: WiFi init, DHCP, Z21 UDP control loop.
///
/// Spawns `wifi::wifi_runner_task` and `wifi::connection_task` internally,
/// following the official esp-hal embassy WiFi example pattern.
pub async fn net_task(
    spawner: Spawner,
    wifi: esp_hal::peripherals::WIFI<'static>,
    credentials: WifiCredentials,
    scheduler_sender: SchedulerCommandSender,
    fault_sender: FaultEventSender,
    channels: NetTaskChannels,
) -> Result<(), NetInitError> {
    let NetTaskChannels {
        net_status,
        status_sender,
        display_sender,
        ready_sender,
        pom_request_sender,
        pom_response_receiver,
        loco_request_sender,
        loco_response_receiver,
    } = channels;

    let (wifi_ctrl, interfaces) = radio::start_wifi(wifi, &wifi::client_mode_config(&credentials))
        .await
        .map_err(NetInitError::WifiBringup)?;
    info!("WiFi started, connecting to SSID: {}", credentials.ssid());

    let net_config = Config::dhcpv4(Default::default());
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let resources = NET_RESOURCES.init(StackResources::new());
    let (stack, runner) = embassy_net::new(interfaces.sta, net_config, resources, seed);

    // Spawn runner and connection tasks, same pattern as official examples.
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
            "Network up, IP: {}",
            defmt::Display2Format(&config.address.address())
        );
        let addr = config.address.address();
        let _ = display_sender.try_send(DisplayEvent::IpAssigned(addr.octets()));
        let _ = display_sender.try_send(DisplayEvent::BootProgress(
            crate::system_status::BootStep::WifiConnected,
        ));
    } else {
        info!("Network up, DHCP configured (no IPv4?)");
    }

    // Set up UDP socket.
    let rx_meta = RX_META.init([PacketMetadata::EMPTY; 16]);
    let rx_buf = RX_BUF.init([0u8; 1024]);
    let tx_meta = TX_META.init([PacketMetadata::EMPTY; 16]);
    let tx_buf = TX_BUF.init([0u8; 1024]);

    let mut socket = UdpSocket::new(stack, rx_meta, rx_buf, tx_meta, tx_buf);
    socket.bind(Z21_PORT).map_err(|_| NetInitError::UdpBind)?;
    info!("Z21 UDP listening on port {}", Z21_PORT);
    announce_ready(ready_sender, BootReadyEvent::Net).await;

    let mut loco_slots: LocoSlots = [None; 12];
    let mut client_safety = ClientSafetyPolicy::new(Z21_KEEPALIVE_TIMEOUT_MS);
    let mut status_model = StatusModel::new();
    let next_pom_request_id = Cell::new(1u32);
    let next_loco_request_id = Cell::new(1u32);

    let mut recv_buf = [0u8; 256];
    let mut send_buf = [0u8; 64];

    loop {
        let now_ms = Instant::now().as_millis();

        let next_wake = Instant::from_millis(client_safety.next_wake_ms(now_ms));

        use embassy_futures::select::{Either3, select3};

        match select3(
            socket.recv_from(&mut recv_buf),
            net_status.receive(),
            Timer::at(next_wake),
        )
        .await
        {
            // --- Incoming UDP packet ---
            Either3::First(Ok((n, meta))) => {
                let kind = z21_proto::frame_kind(&recv_buf[..n]);
                // Suppress per-second polling noise (SystemState, XBus status)
                let is_polling =
                    kind.header == HEADER_SYSTEMSTATE_GETDATA || kind.header == HEADER_XBUS;
                if !is_polling {
                    info!(
                        "UDP rx {} bytes from {}: header=0x{:04X} xheader=0x{:02X}",
                        n,
                        defmt::Display2Format(&meta.endpoint),
                        kind.header,
                        kind.xheader
                    );
                }

                client_safety.observe_packet(meta.endpoint, Instant::now().as_millis());

                // A Z21 UDP datagram may contain multiple concatenated frames.
                // Process each frame and send its response individually.
                let ctx = Z21Ctx {
                    track: TrackCtx {
                        fault_sender: &fault_sender,
                        status_model: &status_model,
                    },
                    loco: LocoCtx {
                        status_model: &status_model,
                        request_sender: &loco_request_sender,
                        response_receiver: &loco_response_receiver,
                        next_request_id: &next_loco_request_id,
                    },
                    pom: PomCtx {
                        scheduler_sender: &scheduler_sender,
                        status_model: &status_model,
                        request_sender: &pom_request_sender,
                        response_receiver: &pom_response_receiver,
                        next_request_id: &next_pom_request_id,
                        loco: LocoCtx {
                            status_model: &status_model,
                            request_sender: &loco_request_sender,
                            response_receiver: &loco_response_receiver,
                            next_request_id: &next_loco_request_id,
                        },
                    },
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
            Either3::First(Err(_err)) => {
                UDP_RECEIVE_FAILURE_COUNT.fetch_add(1, Ordering::Relaxed);
                warn!("Z21 UDP receive failed");
            }

            // --- System status event ---
            Either3::Second(event) => {
                status_model.apply(event);
                if let Some(ep) = client_safety.active_client() {
                    let status = TrackStatus::from_model(status_model);
                    let plan = plan_status_broadcast(event, status);
                    for message in plan.messages.into_iter().flatten() {
                        let n = match message {
                            StatusBroadcast::TrackPower(enabled) => {
                                z21_proto::encode_bc_track_power(enabled, &mut send_buf)
                            }
                            StatusBroadcast::Stopped => z21_proto::encode_bc_stopped(&mut send_buf),
                            StatusBroadcast::SystemState(status) => {
                                encode_system_state(status, &mut send_buf)
                            }
                        };
                        if socket.send_to(&send_buf[..n], ep).await.is_err() {
                            STATUS_BROADCAST_SEND_FAILURE_COUNT.fetch_add(1, Ordering::Relaxed);
                            warn!("Z21 status broadcast send failed");
                        }
                    }
                }
            }

            // --- Timer tick (client safety timeout) ---
            Either3::Third(_) => {
                if let Some(timeout) = client_safety.on_timer(Instant::now().as_millis()) {
                    // Cut the bridge immediately; the fault manager then
                    // latches the stop and synchronises scheduler/status.
                    crate::track_safety::disable_track_intentionally();
                    fault_sender.send(FaultEvent::StopPressed).await;
                    let n = z21_proto::encode_bc_stopped(&mut send_buf);
                    if socket
                        .send_to(&send_buf[..n], timeout.client)
                        .await
                        .is_err()
                    {
                        STATUS_BROADCAST_SEND_FAILURE_COUNT.fetch_add(1, Ordering::Relaxed);
                        warn!("Z21 stopped broadcast send failed after client timeout");
                    }
                    info!("Client keepalive timeout: track output disabled");
                }
            }
        }
    }
}
