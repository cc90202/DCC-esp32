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
use crate::net::z21_dispatch::{encode_current_system_state, handle_packet, handle_status_event};
use crate::net::z21_proto::{self, HEADER_SYSTEMSTATE_GETDATA, HEADER_XBUS};
use crate::net::{LocoSlots, loco_is_moving};
use crate::system_status::{DisplayEvent, FaultEvent, StatusModel, SystemStatusEvent};

    use crate::config::Z21_KEEPALIVE_TIMEOUT_MS;
    use crate::dcc::cv::drain_channel;
    use crate::dcc::{
        FunctionIndex, LogicalSpeed, PomCv, PomRequest, PomResponse, SchedulerCommand,
    };
    use crate::display::DisplayEvent;
    use crate::fault_manager::FaultEvent;
    const WIFI_SSID: &str = env!("WIFI_SSID");
    const WIFI_PASS: &str = env!("WIFI_PASS");
    use crate::net::z21_proto::{
        self, FunctionAction, HEADER_SYSTEMSTATE_GETDATA, HEADER_XBUS, Z21Command,
    };
    use crate::net::{LocoSlots, LocoState, loco_commands_allowed, loco_is_moving};
    use crate::system_status::{DisplayEvent, FaultEvent, StatusModel, SystemStatusEvent};

const Z21_PORT: u16 = 21105;
const DECEL_STEP_MS: u64 = 500;

// Static buffers — avoids heap allocation in the hot UDP path.
static RX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
static TX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
static TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
static NET_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
// Controller must be 'static so WifiController and WifiDevice are 'static too.
static RADIO_CONTROLLER: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();

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

    let controller = esp_radio::init().map_err(|_| NetInitError::EspRadioInit)?;
    let controller = RADIO_CONTROLLER.init(controller);

    let (mut wifi_ctrl, interfaces) =
        esp_radio::wifi::new(controller, wifi, esp_radio::wifi::Config::default())
            .map_err(|_| NetInitError::WifiInit)?;

    wifi_ctrl
        .set_config(&wifi::client_mode_config())
        .map_err(|_| NetInitError::WifiSetConfig)?;
    wifi_ctrl
        .start_async()
        .await
        .map_err(|_| NetInitError::WifiStart)?;
    info!("WiFi started, connecting to SSID: {}", wifi::WIFI_SSID);

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

    pub struct NetTaskChannels {
        pub net_status: Receiver<'static, CriticalSectionRawMutex, SystemStatusEvent, 8>,
        pub status_sender: Sender<'static, CriticalSectionRawMutex, SystemStatusEvent, 16>,
        pub display_sender: Sender<'static, CriticalSectionRawMutex, DisplayEvent, 8>,
        pub ready_sender:
            Sender<'static, CriticalSectionRawMutex, crate::system_status::BootReadyEvent, 9>,
        pub pom_request_sender: Sender<'static, CriticalSectionRawMutex, PomRequest, 1>,
        pub pom_response_receiver: Receiver<'static, CriticalSectionRawMutex, PomResponse, 1>,
    }

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

    /// Handles WiFi connect and automatic reconnect.
    #[embassy_executor::task]
    async fn connection_task(
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

    // ── Main net task ─────────────────────────────────────────────────────────

    /// Main net task — WiFi init, DHCP, Z21 UDP control loop.
    ///
    /// Spawns `wifi_runner_task` and `connection_task` internally, following
    /// the official esp-hal embassy WiFi example pattern.
    pub async fn net_task(
        spawner: Spawner,
        wifi: esp_hal::peripherals::WIFI<'static>,
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

        let controller = esp_radio::init().map_err(|_| NetInitError::EspRadioInit)?;
        let controller = RADIO_CONTROLLER.init(controller);

        let (mut wifi_ctrl, interfaces) =
            esp_radio::wifi::new(controller, wifi, esp_radio::wifi::Config::default())
                .map_err(|_| NetInitError::WifiInit)?;

        let client_config = ClientConfig::default()
            .with_ssid(WIFI_SSID.to_string())
            .with_password(WIFI_PASS.to_string())
            .with_auth_method(AuthMethod::Wpa2Personal);

        wifi_ctrl
            .set_config(&ModeConfig::Client(client_config))
            .map_err(|_| NetInitError::WifiSetConfig)?;
        wifi_ctrl
            .start_async()
            .await
            .map_err(|_| NetInitError::WifiStart)?;
        info!("WiFi started, connecting to SSID: {}", WIFI_SSID);

        let net_config = Config::dhcpv4(Default::default());
        let rng = Rng::new();
        let seed = (rng.random() as u64) << 32 | rng.random() as u64;
        let resources = NET_RESOURCES.init(StackResources::new());
        let (stack, runner) = embassy_net::new(interfaces.sta, net_config, resources, seed);

        // Spawn runner and connection tasks — same pattern as official examples.
        spawner
            .spawn(wifi_runner_task(runner))
            .map_err(|_| NetInitError::WifiRunnerSpawn)?;
        spawner
            .spawn(connection_task(wifi_ctrl, status_sender))
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
            Instant::from_millis(last_rx_ms + Z21_KEEPALIVE_TIMEOUT_MS + 1)
        };

        use embassy_futures::select::{Either3, select3};

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
                    };
                    for frame in z21_proto::iter_frames(&recv_buf[..n]) {
                        let resp_len =
                            handle_packet(frame, &mut loco_slots, &mut send_buf, &ctx).await;
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

    /// Shared dependencies threaded through the Z21 command handlers.
    ///
    /// Grouping the channels and shared state here avoids a 7-9 argument sprawl
    /// on every handler; per-call buffers (`out`, `loco_slots`) stay explicit
    /// because they are mutated per invocation.
    struct Z21Ctx<'a> {
        scheduler_sender: &'a Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
        fault_sender: &'a Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>,
        status_model: &'a StatusModel,
        pom_request_sender: &'a Sender<'static, CriticalSectionRawMutex, PomRequest, 1>,
        pom_response_receiver: &'a Receiver<'static, CriticalSectionRawMutex, PomResponse, 1>,
    }

    /// Process one incoming Z21 frame; dispatch commands and build response.
    async fn handle_packet(
        buf: &[u8],
        loco_slots: &mut LocoSlots,
        out: &mut [u8],
        ctx: &Z21Ctx<'_>,
    ) -> usize {
        let cmd = match z21_proto::parse_frame(buf) {
            Ok(c) => c,
            Err(e) => {
                warn!("Z21 parse error: {:?}", e);
                return z21_proto::encode_unknown_command(out);
            }
        };

        log_command(cmd);
        dispatch_command(cmd, buf, loco_slots, out, ctx).await
    }

    fn log_command(cmd: Z21Command) {
        // Polling commands arrive every ~1s — avoid log noise.
        if !matches!(cmd, Z21Command::GetSystemState | Z21Command::GetStatus) {
            info!("Z21 cmd: {:?}", cmd);
        }
    }

    fn encode_current_system_state(status_model: &StatusModel, out: &mut [u8]) -> usize {
        z21_proto::encode_system_state(
            status_model.track_power_on(),
            status_model.estop_active(),
            status_model.short_circuit(),
            out,
        )
    }

    fn encode_current_status(status_model: &StatusModel, out: &mut [u8]) -> usize {
        z21_proto::encode_status(
            status_model.track_power_on(),
            status_model.estop_active(),
            out,
        )
    }

    async fn dispatch_command(
        cmd: Z21Command,
        raw_frame: &[u8],
        loco_slots: &mut LocoSlots,
        out: &mut [u8],
        ctx: &Z21Ctx<'_>,
    ) -> usize {
        match cmd {
            Z21Command::GetSerialNumber => {
                z21_proto::encode_serial_number(DEFAULT_Z21_SERIAL_NUMBER, out)
            }
            Z21Command::GetCode => z21_proto::encode_code(out),
            Z21Command::GetHwInfo => z21_proto::encode_hwinfo(out),
            Z21Command::GetSystemState => encode_current_system_state(ctx.status_model, out),
            Z21Command::Logoff => 0,
            // After SetBroadcastFlags the app expects an immediate unsolicited
            // LAN_SYSTEMSTATE_DATACHANGED push to learn current track state.
            // Without it the app waits indefinitely and never enters main UI.
            Z21Command::SetBroadcastFlags { .. } => {
                encode_current_system_state(ctx.status_model, out)
            }
            Z21Command::GetXBusVersion => z21_proto::encode_xbus_version(out),
            Z21Command::GetFirmwareVersion => z21_proto::encode_firmware_version(out),
            Z21Command::GetStatus => encode_current_status(ctx.status_model, out),
            Z21Command::SetTrackPowerOn => {
                ctx.fault_sender.send(FaultEvent::ResumeShortPressed).await;
                0
            }
            Z21Command::SetTrackPowerOff => {
                ctx.fault_sender.send(FaultEvent::StopPressed).await;
                0
            }
            Z21Command::SetStop => {
                ctx.fault_sender.send(FaultEvent::StopPressed).await;
                z21_proto::encode_bc_stopped(out)
            }
            Z21Command::SetLocoEstop { address } => {
                ctx.scheduler_sender
                    .send(SchedulerCommand::EmergencyStop { address })
                    .await;
                z21_proto::encode_bc_stopped(out)
            }
            Z21Command::GetLocoMode { address } => z21_proto::encode_loco_mode(address, out),
            Z21Command::SetLocoMode { address, mode } => {
                if mode != 0 {
                    warn!(
                        "loco mode addr={} unsupported mode={} (DCC only)",
                        address.value(),
                        mode
                    );
                }
                0
            }
            Z21Command::GetLocoInfo { address } => {
                let known_to_net = z21_proto::find_slot(loco_slots, address).is_some();
                if let Some(state) = z21_proto::find_or_insert(loco_slots, address) {
                    let state = *state;
                    let len = z21_proto::encode_loco_info(&state, out);
                    if !known_to_net {
                        info!(
                            "loco info addr={} created DCC refresh slot",
                            address.value()
                        );
                        if let Some(speed) = LogicalSpeed::new(state.speed, state.format) {
                            ctx.scheduler_sender
                                .send(SchedulerCommand::SetSpeed {
                                    address,
                                    speed,
                                    direction: state.direction,
                                    format: state.format,
                                })
                                .await;
                        } else {
                            warn!(
                                "loco info addr={} speed={} invalid for format={:?}, refresh slot not seeded",
                                address.value(),
                                state.speed,
                                state.format
                            );
                        }
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

    async fn handle_cv_pom_write(
        out: &mut [u8],
        ctx: &Z21Ctx<'_>,
        address: crate::dcc::DccAddress,
        cv: u16,
        value: u8,
    ) -> usize {
        let Some(cv_addr) = PomCv::new(cv) else {
            warn!("POM write addr={} invalid cv={}", address.value(), cv);
            return z21_proto::encode_cv_nack(out);
        };

        if !ctx.status_model.pom_allowed() {
            warn!(
                "POM write addr={} cv={} rejected (track_on={} estop={} fault={:?})",
                address.value(),
                cv,
                ctx.status_model.track_power_on(),
                ctx.status_model.estop_active(),
                ctx.status_model.fault_cause()
            );
            return z21_proto::encode_cv_nack(out);
        }

        ctx.scheduler_sender
            .send(SchedulerCommand::SuspendRailcomDiscovery)
            .await;

        match request_pom(
            ctx.pom_request_sender,
            ctx.pom_response_receiver,
            |request_id| PomRequest::Write {
                request_id,
                address,
                cv: cv_addr,
                value,
            },
        )
        .await
        {
            Some(PomResponse::Ack { .. }) => z21_proto::encode_cv_result(cv, value, out),
            Some(PomResponse::Nack { .. }) => z21_proto::encode_cv_nack(out),
            Some(PomResponse::Value { .. }) | None => {
                warn!(
                    "POM write addr={} cv={} completed without ACK",
                    address.value(),
                    cv
                );
                z21_proto::encode_cv_nack(out)
            }
        }
    }

    async fn handle_cv_pom_read(
        loco_slots: &mut LocoSlots,
        out: &mut [u8],
        ctx: &Z21Ctx<'_>,
        address: crate::dcc::DccAddress,
        cv: u16,
    ) -> usize {
        let Some(cv_addr) = PomCv::new(cv) else {
            warn!("POM read addr={} invalid cv={}", address.value(), cv);
            return z21_proto::encode_cv_nack(out);
        };

        if !ctx.status_model.pom_allowed() {
            warn!(
                "POM read addr={} cv={} rejected (track_on={} estop={} fault={:?})",
                address.value(),
                cv,
                ctx.status_model.track_power_on(),
                ctx.status_model.estop_active(),
                ctx.status_model.fault_cause()
            );
            return z21_proto::encode_cv_nack(out);
        }

        ctx.scheduler_sender
            .send(SchedulerCommand::SuspendRailcomDiscovery)
            .await;

        let format = z21_proto::find_or_insert(loco_slots, address)
            .map(|slot| slot.format)
            .unwrap_or(crate::dcc::SpeedFormat::Speed128);
        ctx.scheduler_sender
            .send(SchedulerCommand::EnsureRailcomRefresh { address, format })
            .await;

        match request_pom(
            ctx.pom_request_sender,
            ctx.pom_response_receiver,
            |request_id| PomRequest::Read {
                request_id,
                address,
                cv: cv_addr,
            },
        )
        .await
        {
            Some(PomResponse::Value { value, .. }) => z21_proto::encode_cv_result(cv, value, out),
            Some(PomResponse::Ack { .. }) | Some(PomResponse::Nack { .. }) | None => {
                warn!(
                    "POM read addr={} cv={} completed without value",
                    address.value(),
                    cv
                );
                z21_proto::encode_cv_nack(out)
            }
        }
    }

    async fn handle_set_loco_drive(
        loco_slots: &mut LocoSlots,
        out: &mut [u8],
        ctx: &Z21Ctx<'_>,
        address: crate::dcc::DccAddress,
        speed: u8,
        direction: crate::dcc::Direction,
        format: crate::dcc::SpeedFormat,
    ) -> usize {
        if !loco_commands_allowed(ctx.status_model.track_power_on()) {
            warn!(
                "loco drive addr={} rejected while track power is OFF (estop={} short={} track_on={})",
                address.value(),
                ctx.status_model.estop_active(),
                ctx.status_model.short_circuit(),
                ctx.status_model.track_power_on()
            );
            return encode_rejected_loco_info(loco_slots, out, address);
        }

        let Some(logical_speed) = LogicalSpeed::new(speed, format) else {
            warn!(
                "loco drive addr={} speed={} invalid for format={:?}, command dropped",
                address.value(),
                speed,
                format
            );
            return encode_rejected_loco_info(loco_slots, out, address);
        };

        let Some(slot) = z21_proto::find_or_insert(loco_slots, address) else {
            warn!("SetLocoDrive: all 12 slots full and running, command dropped");
            return 0;
        };

        slot.speed = logical_speed.value();
        slot.direction = direction;
        slot.format = format;

        info!(
            "loco drive addr={} fmt={:?} dir={:?} speed={}",
            address.value(),
            format,
            direction,
            speed
        );
        ctx.scheduler_sender
            .send(SchedulerCommand::SetSpeed {
                address,
                speed: logical_speed,
                direction,
                format,
            })
            .await;

        z21_proto::encode_loco_info(slot, out)
    }

    // Log raw header/xheader to help identify protocol gaps.
    fn log_unknown_command(buf: &[u8]) {
        let header = if buf.len() >= 4 {
            u16::from_le_bytes([buf[2], buf[3]])
        } else {
            0
        };
        let xheader = if buf.len() >= 5 && header == HEADER_XBUS {
            buf[4]
        } else {
            0
        };

        if header == HEADER_XBUS {
            warn!(
                "Z21 unknown XBus: xheader=0x{:02X} db0=0x{:02X} len={}",
                xheader,
                if buf.len() >= 6 { buf[5] } else { 0 },
                buf.len()
            );
        } else {
            warn!(
                "Z21 unknown top-level: header=0x{:04X} len={}",
                header,
                buf.len()
            );
        }
    }

    async fn handle_set_loco_function(
        loco_slots: &mut LocoSlots,
        out: &mut [u8],
        ctx: &Z21Ctx<'_>,
        address: crate::dcc::DccAddress,
        function: u8,
        action: FunctionAction,
    ) -> usize {
        if !loco_commands_allowed(ctx.status_model.track_power_on()) {
            warn!(
                "loco function addr={} rejected while track power is OFF (estop={} short={} track_on={})",
                address.value(),
                ctx.status_model.estop_active(),
                ctx.status_model.short_circuit(),
                ctx.status_model.track_power_on()
            );
            return encode_rejected_loco_info(loco_slots, out, address);
        }

        let Some(fi) = FunctionIndex::new(function) else {
            warn!(
                "loco function addr={} rejected: unsupported function index {}",
                address.value(),
                function
            );
            return encode_rejected_loco_info(loco_slots, out, address);
        };

        let Some(slot) = z21_proto::find_or_insert(loco_slots, address) else {
            return 0;
        };

        let bit = 1u32 << function;
        let enabled = match action {
            FunctionAction::On => {
                slot.functions |= bit;
                true
            }
            FunctionAction::Off => {
                slot.functions &= !bit;
                false
            }
            FunctionAction::Toggle => {
                let was_enabled = (slot.functions & bit) != 0;
                if was_enabled {
                    slot.functions &= !bit;
                } else {
                    slot.functions |= bit;
                }
                !was_enabled
            }
        };

        info!(
            "loco fn addr={} f{} action={:?} enabled={} speed={} fmt={:?}",
            address.value(),
            function,
            action,
            enabled,
            slot.speed,
            slot.format
        );
        ctx.scheduler_sender
            .send(SchedulerCommand::SetFunction {
                address,
                function: fi,
                enabled,
            })
            .await;

        z21_proto::encode_loco_info(slot, out)
    }

    /// Encode a broadcast response for a SystemStatusEvent.
    ///
    /// `FaultLatched` and `FaultCleared` are handled inline in the event loop
    /// (they require multiple frames including `LAN_SYSTEMSTATE_DATACHANGED`).
    fn handle_status_event(event: SystemStatusEvent, out: &mut [u8]) -> usize {
        match event {
            SystemStatusEvent::EstopActive => z21_proto::encode_bc_stopped(out),
            SystemStatusEvent::EstopCleared => z21_proto::encode_bc_track_power(true, out),
            _ => 0,
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
