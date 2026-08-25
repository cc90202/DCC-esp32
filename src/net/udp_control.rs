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
use embassy_net::{Config, IpAddress, IpEndpoint, Stack, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::Instant;
use esp_hal::rng::Rng;
use static_cell::StaticCell;

use crate::application::LocoSlots;
use crate::application::StatusModel;
use crate::application::client_safety::{
    LeaseActivity, LeaseDecision, LeaseEpoch, LeasePermit, LeaseRequest,
};
use crate::application::track_control::{StatusBroadcast, TrackStatus, plan_status_broadcast};
use crate::net::client_watchdog::LeaseTripNotification;
use crate::net::loco_client::reset_for_lease;
use crate::net::wifi_config::WifiCredentials;
use crate::net::z21_context::{LocoCtx, PomCtx, TrackCtx, Z21Ctx};
use crate::net::z21_dispatch::{
    LeaseClassification, classify_lease_activity, encode_system_state, encoded_len, handle_command,
};
use crate::runtime_channels::{
    BootReadySender, DisplaySender, FaultEventSender, LocoRequestSender, LocoResponseReceiver,
    NetStatusReceiver, PomRequestSender, PomResponseReceiver, SystemStatusSender, announce_ready,
};
use crate::system_status::{BootReadyEvent, DisplayEvent};
use crate::z21::{self as z21_proto, HEADER_SYSTEMSTATE_GETDATA, HEADER_XBUS};

pub use super::wifi::NetInitError;
use super::{radio, wifi};

const Z21_PORT: u16 = 21105;
const MAX_FRAMES_PER_DATAGRAM: usize = 8;
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

pub(crate) struct NetTaskChannels {
    pub(crate) net_status: NetStatusReceiver,
    pub(crate) status_sender: SystemStatusSender,
    pub(crate) display_sender: DisplaySender,
    pub(crate) ready_sender: BootReadySender,
    pub(crate) pom_request_sender: PomRequestSender,
    pub(crate) pom_response_receiver: PomResponseReceiver,
    pub(crate) loco_request_sender: LocoRequestSender,
    pub(crate) loco_response_receiver: LocoResponseReceiver,
    pub(crate) lease_request_sender:
        Sender<'static, CriticalSectionRawMutex, LeaseRequest<IpAddress>, 1>,
    pub(crate) lease_response_receiver:
        Receiver<'static, CriticalSectionRawMutex, LeaseDecision<IpAddress>, 1>,
    pub(crate) lease_trip_receiver:
        Receiver<'static, CriticalSectionRawMutex, LeaseTripNotification, 1>,
}

struct Z21LoopIo {
    net_status: NetStatusReceiver,
    fault_sender: FaultEventSender,
    pom_request_sender: PomRequestSender,
    pom_response_receiver: PomResponseReceiver,
    loco_request_sender: LocoRequestSender,
    loco_response_receiver: LocoResponseReceiver,
    lease_request_sender: Sender<'static, CriticalSectionRawMutex, LeaseRequest<IpAddress>, 1>,
    lease_response_receiver:
        Receiver<'static, CriticalSectionRawMutex, LeaseDecision<IpAddress>, 1>,
    lease_trip_receiver: Receiver<'static, CriticalSectionRawMutex, LeaseTripNotification, 1>,
}

struct Z21LoopState {
    loco_slots: LocoSlots,
    lease_owner: Option<IpEndpoint>,
    scheduler_epoch: Option<LeaseEpoch>,
    status_model: StatusModel,
    next_pom_request_id: Cell<u32>,
    next_loco_request_id: Cell<u32>,
}

impl Z21LoopState {
    fn new() -> Self {
        Self {
            loco_slots: [None; 12],
            lease_owner: None,
            scheduler_epoch: None,
            status_model: StatusModel::new(),
            next_pom_request_id: Cell::new(1),
            next_loco_request_id: Cell::new(1),
        }
    }
}

async fn start_network_stack(
    spawner: Spawner,
    wifi: esp_hal::peripherals::WIFI<'static>,
    credentials: &WifiCredentials,
    status_sender: SystemStatusSender,
    display_sender: DisplaySender,
) -> Result<Stack<'static>, NetInitError> {
    let (wifi_ctrl, interfaces) = radio::start_wifi(wifi, &wifi::client_mode_config(credentials))
        .await
        .map_err(NetInitError::WifiBringup)?;
    info!("WiFi started, connecting to SSID: {}", credentials.ssid());

    let net_config = Config::dhcpv4(Default::default());
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let resources = NET_RESOURCES.init(StackResources::new());
    let (stack, runner) = embassy_net::new(interfaces.sta, net_config, resources, seed);

    spawner
        .spawn(wifi::wifi_runner_task(runner))
        .map_err(|_| NetInitError::WifiRunnerSpawn)?;
    spawner
        .spawn(wifi::connection_task(wifi_ctrl, status_sender))
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

fn bind_z21_socket(stack: Stack<'static>) -> Result<UdpSocket<'static>, NetInitError> {
    let rx_meta = RX_META.init([PacketMetadata::EMPTY; 16]);
    let rx_buf = RX_BUF.init([0u8; 1024]);
    let tx_meta = TX_META.init([PacketMetadata::EMPTY; 16]);
    let tx_buf = TX_BUF.init([0u8; 1024]);
    let mut socket = UdpSocket::new(stack, rx_meta, rx_buf, tx_meta, tx_buf);
    socket.bind(Z21_PORT).map_err(|_| NetInitError::UdpBind)?;
    info!("Z21 UDP listening on port {}", Z21_PORT);
    Ok(socket)
}

async fn handle_udp_datagram(
    socket: &mut UdpSocket<'_>,
    datagram: &[u8],
    endpoint: IpEndpoint,
    send_buf: &mut [u8],
    state: &mut Z21LoopState,
    io: &Z21LoopIo,
) {
    let observed_at_ms = Instant::now().as_millis();
    let kind = z21_proto::frame_kind(datagram);
    let is_polling = kind.header == HEADER_SYSTEMSTATE_GETDATA || kind.header == HEADER_XBUS;
    if !is_polling {
        info!(
            "UDP rx {} bytes from {}: header=0x{:04X} xheader=0x{:02X}",
            datagram.len(),
            defmt::Display2Format(&endpoint),
            kind.header,
            kind.xheader
        );
    }

    for (frame_index, boundary) in z21_proto::iter_frames(datagram).enumerate() {
        if frame_index == MAX_FRAMES_PER_DATAGRAM {
            warn!("Z21 datagram frame limit exceeded; trailing frames discarded");
            break;
        }
        let frame = match boundary {
            Ok(frame) => frame,
            Err(error) => {
                warn!("Z21 invalid frame boundary: {:?}", error);
                let response_len = encoded_len(z21_proto::encode_unknown_command(send_buf));
                if response_len > 0
                    && socket
                        .send_to(&send_buf[..response_len], endpoint)
                        .await
                        .is_err()
                {
                    warn!("Z21 UDP send failed");
                }
                break;
            }
        };
        let command = match z21_proto::parse_frame(frame) {
            Ok(command) => command,
            Err(error) => {
                warn!("Z21 parse error: {:?}", error);
                let response_len = encoded_len(z21_proto::encode_unknown_command(send_buf));
                if response_len > 0
                    && socket
                        .send_to(&send_buf[..response_len], endpoint)
                        .await
                        .is_err()
                {
                    warn!("Z21 UDP send failed");
                }
                continue;
            }
        };
        let authorization = authorize_command(
            classify_lease_activity(command),
            endpoint,
            observed_at_ms,
            &io.lease_request_sender,
            &io.lease_response_receiver,
        )
        .await;
        if authorization.owner_update {
            state.lease_owner = authorization.owner;
        }
        if !authorization.execute {
            warn!("Z21 command rejected by controller lease");
            continue;
        }
        if let Some(permit) = authorization.permit
            && state.scheduler_epoch != Some(permit.epoch())
        {
            if !reset_for_lease(
                &io.loco_request_sender,
                &io.loco_response_receiver,
                &state.next_loco_request_id,
                permit,
            )
            .await
            {
                warn!("Z21 command rejected: controller lease synchronization failed");
                continue;
            }
            state.scheduler_epoch = Some(permit.epoch());
        }
        let ctx = Z21Ctx {
            track: TrackCtx {
                fault_sender: &io.fault_sender,
                status_model: &state.status_model,
                lease_permit: authorization.permit,
            },
            loco: LocoCtx {
                status_model: &state.status_model,
                lease_permit: authorization.permit,
                request_sender: &io.loco_request_sender,
                response_receiver: &io.loco_response_receiver,
                next_request_id: &state.next_loco_request_id,
            },
            pom: PomCtx {
                status_model: &state.status_model,
                lease_permit: authorization.permit,
                request_sender: &io.pom_request_sender,
                response_receiver: &io.pom_response_receiver,
                next_request_id: &state.next_pom_request_id,
                loco: LocoCtx {
                    status_model: &state.status_model,
                    lease_permit: authorization.permit,
                    request_sender: &io.loco_request_sender,
                    response_receiver: &io.loco_response_receiver,
                    next_request_id: &state.next_loco_request_id,
                },
            },
        };
        let response_len =
            handle_command(command, frame, &mut state.loco_slots, send_buf, &ctx).await;
        if response_len > 0
            && socket
                .send_to(&send_buf[..response_len], endpoint)
                .await
                .is_err()
        {
            warn!("Z21 UDP send failed");
        }
    }
}

async fn broadcast_status(
    socket: &mut UdpSocket<'_>,
    event: crate::system_status::SystemStatusEvent,
    send_buf: &mut [u8],
    state: &mut Z21LoopState,
) {
    state.status_model.apply(event);
    let Some(endpoint) = state.lease_owner else {
        return;
    };

    let status = TrackStatus::from_model(state.status_model);
    let plan = plan_status_broadcast(event, status);
    for message in plan.messages.into_iter().flatten() {
        let response_len = match message {
            StatusBroadcast::TrackPower(enabled) => {
                encoded_len(z21_proto::encode_bc_track_power(enabled, send_buf))
            }
            StatusBroadcast::Stopped => encoded_len(z21_proto::encode_bc_stopped(send_buf)),
            StatusBroadcast::SystemState(status) => encode_system_state(status, send_buf),
        };
        if response_len > 0
            && socket
                .send_to(&send_buf[..response_len], endpoint)
                .await
                .is_err()
        {
            STATUS_BROADCAST_SEND_FAILURE_COUNT.fetch_add(1, Ordering::Relaxed);
            warn!("Z21 status broadcast send failed");
        }
    }
}

async fn run_z21_loop(mut socket: UdpSocket<'static>, io: Z21LoopIo) -> ! {
    let mut state = Z21LoopState::new();
    let mut recv_buf = [0u8; 256];
    let mut send_buf = [0u8; 64];

    loop {
        use embassy_futures::select::{Either3, select3};

        match select3(
            socket.recv_from(&mut recv_buf),
            io.net_status.receive(),
            io.lease_trip_receiver.receive(),
        )
        .await
        {
            Either3::First(Ok((len, metadata))) => {
                handle_udp_datagram(
                    &mut socket,
                    &recv_buf[..len],
                    metadata.endpoint,
                    &mut send_buf,
                    &mut state,
                    &io,
                )
                .await;
            }
            Either3::First(Err(_error)) => {
                UDP_RECEIVE_FAILURE_COUNT.fetch_add(1, Ordering::Relaxed);
                warn!("Z21 UDP receive failed");
            }
            Either3::Second(event) => {
                broadcast_status(&mut socket, event, &mut send_buf, &mut state).await;
            }
            Either3::Third(trip) => {
                let owner_endpoint = state.lease_owner.filter(|owner| owner.addr == trip.client);
                if owner_endpoint.is_some() {
                    state.lease_owner = None;
                }
                state.scheduler_epoch = None;
                let response_len = encoded_len(z21_proto::encode_bc_stopped(&mut send_buf));
                if let Some(endpoint) = owner_endpoint
                    && response_len > 0
                    && socket
                        .send_to(&send_buf[..response_len], endpoint)
                        .await
                        .is_err()
                {
                    STATUS_BROADCAST_SEND_FAILURE_COUNT.fetch_add(1, Ordering::Relaxed);
                    warn!("Z21 stopped broadcast send failed after controller lease trip");
                }
                info!("Controller lease tripped: track output disabled");
            }
        }
    }
}

/// Main net task: WiFi init, DHCP, Z21 UDP control loop.
///
/// Spawns `wifi::wifi_runner_task` and `wifi::connection_task` internally,
/// following the official esp-hal embassy WiFi example pattern.
pub(crate) async fn net_task(
    spawner: Spawner,
    wifi: esp_hal::peripherals::WIFI<'static>,
    credentials: WifiCredentials,
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
        lease_request_sender,
        lease_response_receiver,
        lease_trip_receiver,
    } = channels;

    let stack =
        start_network_stack(spawner, wifi, &credentials, status_sender, display_sender).await?;
    let socket = bind_z21_socket(stack)?;
    announce_ready(ready_sender, BootReadyEvent::Net).await;

    run_z21_loop(
        socket,
        Z21LoopIo {
            net_status,
            fault_sender,
            pom_request_sender,
            pom_response_receiver,
            loco_request_sender,
            loco_response_receiver,
            lease_request_sender,
            lease_response_receiver,
            lease_trip_receiver,
        },
    )
    .await
}

#[derive(Debug, Clone, Copy)]
struct CommandAuthorization {
    execute: bool,
    permit: Option<LeasePermit>,
    owner_update: bool,
    owner: Option<IpEndpoint>,
}

async fn authorize_command(
    classification: LeaseClassification,
    endpoint: IpEndpoint,
    observed_at_ms: u64,
    request_sender: &Sender<'static, CriticalSectionRawMutex, LeaseRequest<IpAddress>, 1>,
    response_receiver: &Receiver<'static, CriticalSectionRawMutex, LeaseDecision<IpAddress>, 1>,
) -> CommandAuthorization {
    let LeaseClassification::Activity(activity) = classification else {
        return CommandAuthorization {
            execute: true,
            permit: None,
            owner_update: false,
            owner: None,
        };
    };
    request_sender
        .send(LeaseRequest {
            // Mobile Z21 clients commonly rotate their UDP source port. The
            // controller lease is tied to the host address; the latest full
            // endpoint is retained separately for replies and broadcasts.
            client: endpoint.addr,
            activity,
            observed_at_ms,
        })
        .await;
    match response_receiver.receive().await {
        LeaseDecision::Acquired(permit) | LeaseDecision::Renewed(permit) => CommandAuthorization {
            execute: true,
            permit: Some(permit),
            owner_update: true,
            owner: Some(endpoint),
        },
        LeaseDecision::Maintained(_) => CommandAuthorization {
            execute: true,
            permit: None,
            owner_update: true,
            owner: Some(endpoint),
        },
        LeaseDecision::Rejected(_) => CommandAuthorization {
            execute: matches!(activity, LeaseActivity::KeepAlive),
            permit: None,
            owner_update: false,
            owner: None,
        },
        LeaseDecision::Trip { .. } => CommandAuthorization {
            execute: false,
            permit: None,
            owner_update: true,
            owner: None,
        },
    }
}
