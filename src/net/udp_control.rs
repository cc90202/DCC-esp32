//! Embassy WiFi + Z21 UDP control task.
//!
//! ESP32-C6 only — gated behind `#[cfg(target_arch = "riscv32")]`.

#[cfg(target_arch = "riscv32")]
mod imp {
    extern crate alloc;

    use alloc::string::ToString;
    use defmt::{info, warn};
    use embassy_executor::Spawner;
    use embassy_net::udp::{PacketMetadata, UdpSocket};
    use embassy_net::{Config, StackResources};
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::channel::{Receiver, Sender};
    use embassy_time::{Duration, Instant, Timer};
    use esp_hal::rng::Rng;
    use esp_radio::wifi::{
        AuthMethod, ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent,
    };
    use static_cell::StaticCell;

    use crate::config::Z21_KEEPALIVE_TIMEOUT_MS;
    use crate::dcc::{FunctionIndex, SchedulerCommand};
    use crate::display::DisplayEvent;
    use crate::fault_manager::FaultEvent;
    const WIFI_SSID: &str = env!("WIFI_SSID");
    const WIFI_PASS: &str = env!("WIFI_PASS");
    use crate::net::z21_proto::{
        self, FunctionAction, HEADER_SYSTEMSTATE_GETDATA, HEADER_XBUS, Z21Command,
    };
    use crate::net::{LocoSlots, LocoState, loco_commands_allowed, loco_is_moving};
    use crate::system_status::{StatusModel, SystemStatusEvent};

    const Z21_PORT: u16 = 21105;
    const DECEL_STEP_MS: u64 = 500;
    /// Arbitrary device serial reported to Z21 apps (no real meaning).
    const DEFAULT_Z21_SERIAL_NUMBER: u32 = 0xC0FFEE01;

    // Static buffers — avoids heap allocation in the hot UDP path.
    static RX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    static TX_META: StaticCell<[PacketMetadata; 16]> = StaticCell::new();
    static TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    static NET_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    // Controller must be 'static so WifiController and WifiDevice are 'static too.
    static RADIO_CONTROLLER: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();

    // ── Embassy tasks (runner + connection) ───────────────────────────────────

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum ConnectionState {
        Connecting,
        Connected,
    }

    pub struct NetTaskChannels {
        pub net_status: Receiver<'static, CriticalSectionRawMutex, SystemStatusEvent, 8>,
        pub status_sender: Sender<'static, CriticalSectionRawMutex, SystemStatusEvent, 16>,
        pub display_sender: Sender<'static, CriticalSectionRawMutex, DisplayEvent, 8>,
        pub ready_sender: Sender<'static, CriticalSectionRawMutex, crate::boot::BootReadyEvent, 9>,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
    pub enum NetInitError {
        EspRadioInit,
        WifiInit,
        WifiSetConfig,
        WifiStart,
        WifiRunnerSpawn,
        ConnectionSpawn,
        UdpBind,
    }

    impl NetInitError {
        pub const fn message(self) -> &'static str {
            match self {
                Self::EspRadioInit => "esp-radio init failed",
                Self::WifiInit => "WiFi init failed",
                Self::WifiSetConfig => "WiFi set_config failed",
                Self::WifiStart => "WiFi start failed",
                Self::WifiRunnerSpawn => "failed to spawn wifi_runner_task",
                Self::ConnectionSpawn => "failed to spawn connection_task",
                Self::UdpBind => "UDP bind on Z21 port failed",
            }
        }
    }

    /// Runs the embassy-net stack driver — dedicated Embassy task per official example.
    #[embassy_executor::task]
    async fn wifi_runner_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) {
        runner.run().await
    }

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
                crate::display::BootStep::WifiConnected,
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
        ready_sender.send(crate::boot::BootReadyEvent::Net).await;

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
                    for frame in z21_proto::iter_frames(&recv_buf[..n]) {
                        let resp_len = handle_packet(
                            frame,
                            &mut loco_slots,
                            &mut send_buf,
                            &scheduler_sender,
                            &fault_sender,
                            &status_model,
                        )
                        .await;
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
                scheduler_sender
                    .send(SchedulerCommand::SetSpeed {
                        address: slot.address,
                        speed: slot.speed,
                        direction: slot.direction,
                        format: slot.format,
                    })
                    .await;
                any_moving = true;
            }
        }
        any_moving
    }

    /// Process one incoming Z21 frame; dispatch commands and build response.
    async fn handle_packet(
        buf: &[u8],
        loco_slots: &mut LocoSlots,
        out: &mut [u8],
        scheduler_sender: &Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
        fault_sender: &Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>,
        status_model: &StatusModel,
    ) -> usize {
        let cmd = match z21_proto::parse_frame(buf) {
            Ok(c) => c,
            Err(e) => {
                warn!("Z21 parse error: {:?}", e);
                return z21_proto::encode_unknown_command(out);
            }
        };

        log_command(cmd);
        dispatch_command(
            cmd,
            buf,
            loco_slots,
            out,
            scheduler_sender,
            fault_sender,
            status_model,
        )
        .await
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
        scheduler_sender: &Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
        fault_sender: &Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>,
        status_model: &StatusModel,
    ) -> usize {
        match cmd {
            Z21Command::GetSerialNumber => {
                z21_proto::encode_serial_number(DEFAULT_Z21_SERIAL_NUMBER, out)
            }
            Z21Command::GetCode => z21_proto::encode_code(out),
            Z21Command::GetHwInfo => z21_proto::encode_hwinfo(out),
            Z21Command::GetSystemState => encode_current_system_state(status_model, out),
            Z21Command::Logoff => 0,
            // After SetBroadcastFlags the app expects an immediate unsolicited
            // LAN_SYSTEMSTATE_DATACHANGED push to learn current track state.
            // Without it the app waits indefinitely and never enters main UI.
            Z21Command::SetBroadcastFlags { .. } => encode_current_system_state(status_model, out),
            Z21Command::GetXBusVersion => z21_proto::encode_xbus_version(out),
            Z21Command::GetStatus => encode_current_status(status_model, out),
            Z21Command::SetTrackPowerOn => {
                fault_sender.send(FaultEvent::ResumeShortPressed).await;
                0
            }
            Z21Command::SetTrackPowerOff => {
                fault_sender.send(FaultEvent::StopPressed).await;
                0
            }
            Z21Command::SetStop => {
                fault_sender.send(FaultEvent::StopPressed).await;
                z21_proto::encode_bc_stopped(out)
            }
            Z21Command::SetLocoEstop { address } => {
                scheduler_sender
                    .send(SchedulerCommand::EmergencyStop { address })
                    .await;
                z21_proto::encode_bc_stopped(out)
            }
            Z21Command::GetLocoInfo { address } => {
                if let Some(state) = z21_proto::find_or_insert(loco_slots, address) {
                    z21_proto::encode_loco_info(state, out)
                } else {
                    z21_proto::encode_unknown_command(out)
                }
            }
            Z21Command::SetLocoDrive {
                address,
                speed,
                direction,
                format,
            } => {
                handle_set_loco_drive(
                    loco_slots,
                    out,
                    scheduler_sender,
                    status_model,
                    address,
                    speed,
                    direction,
                    format,
                )
                .await
            }
            Z21Command::SetLocoFunction {
                address,
                function,
                action,
            } => {
                handle_set_loco_function(
                    loco_slots,
                    out,
                    scheduler_sender,
                    status_model,
                    address,
                    function,
                    action,
                )
                .await
            }
            // Turnout info — no accessory decoder support; reply with state=unknown to keep
            // the app in sync and suppress repeated warn logs.
            Z21Command::GetTurnoutInfo { address } => z21_proto::encode_turnout_info(address, out),
            Z21Command::Unknown => {
                log_unknown_command(raw_frame);
                0
            }
        }
    }

    fn encode_rejected_loco_info(
        loco_slots: &LocoSlots,
        out: &mut [u8],
        address: crate::dcc::DccAddress,
    ) -> usize {
        if let Some(state) = z21_proto::find_slot(loco_slots, address) {
            return z21_proto::encode_loco_info(state, out);
        }

        let state = LocoState {
            address,
            speed: 0,
            direction: crate::dcc::Direction::Forward,
            format: crate::dcc::SpeedFormat::Speed128,
            functions: 0,
        };
        z21_proto::encode_loco_info(&state, out)
    }

    #[allow(
        clippy::too_many_arguments,
        reason = "Protocol handlers receive explicit state and channel dependencies."
    )]
    async fn handle_set_loco_drive(
        loco_slots: &mut LocoSlots,
        out: &mut [u8],
        scheduler_sender: &Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
        status_model: &StatusModel,
        address: crate::dcc::DccAddress,
        speed: u8,
        direction: crate::dcc::Direction,
        format: crate::dcc::SpeedFormat,
    ) -> usize {
        if !loco_commands_allowed(status_model.track_power_on()) {
            warn!(
                "loco drive addr={} rejected while track power is OFF (estop={} short={} track_on={})",
                address.value(),
                status_model.estop_active(),
                status_model.short_circuit(),
                status_model.track_power_on()
            );
            return encode_rejected_loco_info(loco_slots, out, address);
        }

        let Some(slot) = z21_proto::find_or_insert(loco_slots, address) else {
            warn!("SetLocoDrive: all 12 slots full and running, command dropped");
            return 0;
        };

        slot.speed = speed;
        slot.direction = direction;
        slot.format = format;

        info!(
            "loco drive addr={} fmt={:?} dir={:?} speed={}",
            address.value(),
            format,
            direction,
            speed
        );
        scheduler_sender
            .send(SchedulerCommand::SetSpeed {
                address,
                speed,
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
        scheduler_sender: &Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
        status_model: &StatusModel,
        address: crate::dcc::DccAddress,
        function: u8,
        action: FunctionAction,
    ) -> usize {
        if !loco_commands_allowed(status_model.track_power_on()) {
            warn!(
                "loco function addr={} rejected while track power is OFF (estop={} short={} track_on={})",
                address.value(),
                status_model.estop_active(),
                status_model.short_circuit(),
                status_model.track_power_on()
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
        scheduler_sender
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

#[cfg(target_arch = "riscv32")]
pub use imp::{NetInitError, NetTaskChannels, net_task};
