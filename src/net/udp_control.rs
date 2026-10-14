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
use embassy_net::{IpEndpoint, Stack};
use embassy_time::{Duration, with_timeout};
use static_cell::StaticCell;

use crate::application::LocoSlots;
use crate::application::StatusModel;
use crate::application::client_safety::{LeaseRequest, LeaseVerdict, lease_verdict};
use crate::application::track_control::{StatusBroadcast, TrackStatus, plan_status_broadcast};
use crate::application::{LeaseEpoch, LeasePermit};
use crate::dcc::LOCO_SLOT_CAPACITY;
use crate::dcc::channels::{
    LocoRequestSender, LocoResponseReceiver, PomRequestSender, PomResponseReceiver,
};
use crate::macros::bench_diag;
use crate::net::client_watchdog::{LeaseRequestSender, LeaseResponseReceiver, LeaseTripReceiver};
use crate::net::loco_client::reset_for_lease;
use crate::net::wifi_config::WifiCredentials;
use crate::net::z21_context::{LocoChannels, PomChannels, Z21Ctx};
use crate::net::z21_dispatch::{
    LeaseClassification, classify_lease_activity, encode_system_state, encoded_len, handle_command,
};
use crate::system_status::channels::{
    BootReadySender, DisplaySender, FaultEventSender, NetStatusReceiver, SystemStatusSender,
};
use crate::system_status::{BootReadyEvent, announce_ready};
use crate::z21 as z21_proto;
#[cfg(feature = "bench-diag")]
use crate::z21::{HEADER_SYSTEMSTATE_GETDATA, HEADER_XBUS};

use super::reply::send_reply;
use super::wifi::{NetInitError, start_network_stack};

const Z21_PORT: u16 = 21105;
const MAX_FRAMES_PER_DATAGRAM: usize = 8;
/// Longest wait for a datagram before the Z21 loop reports a heartbeat.
const Z21_BEAT_PERIOD: Duration = Duration::from_millis(500);
// Static buffers: avoids heap allocation in the hot UDP path.
static RX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
static TX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
static TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
static STATUS_BROADCAST_SEND_FAILURE_COUNT: AtomicU32 = AtomicU32::new(0);
static UDP_RECEIVE_FAILURE_COUNT: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "bench-diag")]
#[must_use]
pub(crate) fn status_broadcast_send_failure_count() -> u32 {
    STATUS_BROADCAST_SEND_FAILURE_COUNT.load(Ordering::Relaxed)
}

#[cfg(feature = "bench-diag")]
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
    pub(crate) lease_request_sender: LeaseRequestSender,
    pub(crate) lease_response_receiver: LeaseResponseReceiver,
    pub(crate) lease_trip_receiver: LeaseTripReceiver,
}

struct Z21LoopIo {
    net_status: NetStatusReceiver,
    fault_sender: FaultEventSender,
    pom_request_sender: PomRequestSender,
    pom_response_receiver: PomResponseReceiver,
    loco_request_sender: LocoRequestSender,
    loco_response_receiver: LocoResponseReceiver,
    lease_request_sender: LeaseRequestSender,
    lease_response_receiver: LeaseResponseReceiver,
    lease_trip_receiver: LeaseTripReceiver,
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
            loco_slots: [None; LOCO_SLOT_CAPACITY],
            lease_owner: None,
            scheduler_epoch: None,
            status_model: StatusModel::new(),
            next_pom_request_id: Cell::new(1),
            next_loco_request_id: Cell::new(1),
        }
    }
}

fn bind_z21_socket(stack: Stack<'static>) -> Result<UdpSocket<'static>, NetInitError> {
    let rx_meta = RX_META.init([PacketMetadata::EMPTY; 16]);
    let rx_buf = RX_BUF.init([0u8; 1024]);
    let tx_meta = TX_META.init([PacketMetadata::EMPTY; 16]);
    let tx_buf = TX_BUF.init([0u8; 1024]);
    let mut socket = UdpSocket::new(stack, rx_meta, rx_buf, tx_meta, tx_buf);
    socket
        .bind(Z21_PORT)
        .map_err(|error| NetInitError::UdpBind(error.into()))?;
    info!("Z21 UDP listening on port {}", Z21_PORT);
    Ok(socket)
}

/// Bench diagnostic: one line per received Z21 datagram that is not a poll.
///
/// Compiled out unless `bench-diag` is enabled. A throttle in motion sends a
/// steady stream of datagrams, and each defmt line holds a critical section
/// that delays the DCC waveform and cutout interrupts. Parse errors, rejected
/// commands and send failures are logged unconditionally elsewhere.
#[cfg_attr(
    not(feature = "bench-diag"),
    expect(
        unused_variables,
        reason = "the diagnostic macro removes its arguments when bench diagnostics are disabled"
    )
)]
fn log_datagram(datagram: &[u8], endpoint: IpEndpoint) {
    bench_diag! {
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
    }
}

async fn handle_udp_datagram(
    socket: &UdpSocket<'_>,
    datagram: &[u8],
    endpoint: IpEndpoint,
    send_buf: &mut [u8],
    state: &mut Z21LoopState,
    io: &Z21LoopIo,
) {
    let observed_at_ms = crate::clock::now_ms();
    log_datagram(datagram, endpoint);

    for (frame_index, boundary) in z21_proto::iter_frames(datagram).enumerate() {
        if frame_index == MAX_FRAMES_PER_DATAGRAM {
            warn!("Z21 datagram frame limit exceeded; trailing frames discarded");
            break;
        }
        let frame = match boundary {
            Ok(frame) => frame,
            Err(error) => {
                warn!("Z21 invalid frame boundary: {:?}", error);
                send_unknown_command_response(socket, endpoint, send_buf).await;
                break;
            }
        };
        let command = match z21_proto::parse_frame(frame) {
            Ok(command) => command,
            Err(error) => {
                warn!("Z21 parse error: {:?}", error);
                send_unknown_command_response(socket, endpoint, send_buf).await;
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
        let lease_permit = match authorization {
            CommandAuthorization::ExecuteWithoutLeaseUpdate => None,
            CommandAuthorization::ExecuteForOwner { owner, permit } => {
                state.lease_owner = Some(owner);
                permit
            }
            CommandAuthorization::Reject => {
                warn!("Z21 command rejected by controller lease");
                continue;
            }
            CommandAuthorization::Trip => {
                state.lease_owner = None;
                warn!("Z21 command rejected by controller lease");
                continue;
            }
        };
        if let Some(permit) = lease_permit
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
            status_model: &state.status_model,
            lease_permit,
            fault_sender: &io.fault_sender,
            loco: LocoChannels {
                request_sender: &io.loco_request_sender,
                response_receiver: &io.loco_response_receiver,
                next_request_id: &state.next_loco_request_id,
            },
            pom: PomChannels {
                request_sender: &io.pom_request_sender,
                response_receiver: &io.pom_response_receiver,
                next_request_id: &state.next_pom_request_id,
            },
        };
        let response_len =
            handle_command(command, frame, &mut state.loco_slots, send_buf, &ctx).await;
        if send_reply(socket, send_buf, response_len, endpoint)
            .await
            .is_err()
        {
            warn!("Z21 UDP send failed");
        }
    }
}

async fn send_unknown_command_response(
    socket: &UdpSocket<'_>,
    endpoint: IpEndpoint,
    send_buf: &mut [u8],
) {
    let response_len = encoded_len(z21_proto::encode_unknown_command(send_buf));
    if send_reply(socket, send_buf, response_len, endpoint)
        .await
        .is_err()
    {
        warn!("Z21 UDP send failed");
    }
}

async fn broadcast_status(
    socket: &UdpSocket<'_>,
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
        if send_reply(socket, send_buf, response_len, endpoint)
            .await
            .is_err()
        {
            STATUS_BROADCAST_SEND_FAILURE_COUNT.fetch_add(1, Ordering::Relaxed);
            warn!("Z21 status broadcast send failed");
        }
    }
}

async fn run_z21_loop(socket: UdpSocket<'static>, io: Z21LoopIo) -> ! {
    let mut state = Z21LoopState::new();
    let mut recv_buf = [0u8; 256];
    let mut send_buf = [0u8; 64];

    loop {
        use embassy_futures::select::{Either3, select3};

        crate::liveness::beat(crate::liveness::Task::Net);
        let Ok(event) = with_timeout(
            Z21_BEAT_PERIOD,
            select3(
                socket.recv_from(&mut recv_buf),
                io.net_status.receive(),
                io.lease_trip_receiver.receive(),
            ),
        )
        .await
        else {
            continue;
        };
        match event {
            Either3::First(Ok((len, metadata))) => {
                handle_udp_datagram(
                    &socket,
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
                broadcast_status(&socket, event, &mut send_buf, &mut state).await;
            }
            Either3::Third(trip) => {
                let owner_endpoint = state.lease_owner.filter(|owner| owner.addr == trip.client);
                if owner_endpoint.is_some() {
                    state.lease_owner = None;
                }
                state.scheduler_epoch = None;
                let response_len = encoded_len(z21_proto::encode_bc_stopped(&mut send_buf));
                if let Some(endpoint) = owner_endpoint
                    && send_reply(&socket, &send_buf, response_len, endpoint)
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
    crate::liveness::z21_ready();
    for task in crate::liveness::Task::HTTP {
        spawner
            .spawn(super::update_http::server_task(stack, task))
            .map_err(|_| NetInitError::HttpSpawn)?;
    }
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
enum CommandAuthorization {
    ExecuteWithoutLeaseUpdate,
    ExecuteForOwner {
        owner: IpEndpoint,
        permit: Option<LeasePermit>,
    },
    Reject,
    Trip,
}

async fn authorize_command(
    classification: LeaseClassification,
    endpoint: IpEndpoint,
    observed_at_ms: u64,
    request_sender: &LeaseRequestSender,
    response_receiver: &LeaseResponseReceiver,
) -> CommandAuthorization {
    let LeaseClassification::Activity(activity) = classification else {
        return CommandAuthorization::ExecuteWithoutLeaseUpdate;
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
    match lease_verdict(&response_receiver.receive().await, activity) {
        LeaseVerdict::ExecuteWithoutLeaseUpdate => CommandAuthorization::ExecuteWithoutLeaseUpdate,
        LeaseVerdict::ExecuteForOwner { permit } => CommandAuthorization::ExecuteForOwner {
            owner: endpoint,
            permit,
        },
        LeaseVerdict::Reject => CommandAuthorization::Reject,
        LeaseVerdict::Trip => CommandAuthorization::Trip,
    }
}
