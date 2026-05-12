//! Boot-time orchestration for the firmware runtime.
//!
//! This module owns the startup sequence that wires hardware resources,
//! spawns Embassy tasks, performs DCC self-checks and only then energises the
//! track output. The goal is to keep `main` focused on top-level policy while
//! the boot ordering remains explicit and logged in one place.

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Duration, Timer, with_timeout};
use esp_hal::gpio::{Flex, Level, Output};
use esp_hal::rmt::{Rmt, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use static_cell::StaticCell;

use crate::control_buttons::{new_button_input, resume_button_task, stop_button_task};
use crate::dcc::dcc_engine_task;
use crate::dcc::engine::DccPacketChannel;
use crate::dcc::packet_scheduler_task;
use crate::dcc::rmt_driver;
use crate::dcc::{
    DccAddress, DccFrame, DccPacket, Direction, PomRailcomResultChannel, PomRequestChannel,
    PomResponseChannel, PomTxStartedChannel, SchedulerCommand, SchedulerCommandChannel,
    pom_actor_task,
};
use crate::fault_manager::{
    FaultEvent, FaultEventChannel, FaultManagerState, FaultManagerTaskContext, FaultStateWatch,
    fault_manager_task,
};
use crate::net::udp_control::{NetInitError, NetTaskChannels, net_task};
use crate::railcom::parser::{RailcomLogonResponse, parse_logon_response_48};
use crate::railcom::pipeline::{RailcomChannel, RailcomRxOutcome};
use crate::railcom::pom_dispatch::{evaluate_pom_window, pom_cutout_monitor_task};
use crate::railcom::uart_reader::{
    RailcomRxOutput, RailcomUartRuntimeResultChannel, railcom_uart_rx_config,
};
use crate::short_detector::{new_short_detect_input, short_detector_task};
use crate::status_led::{new_led_output, status_led_task};
use crate::system_status::{NetStatusChannel, SystemStatusChannel, SystemStatusEvent};
use crate::track_output::TrackOutput;

// Static channels/signals shared across Embassy tasks.
static DCC_CHANNEL: StaticCell<DccPacketChannel> = StaticCell::new();
static SCHEDULER_COMMANDS: StaticCell<SchedulerCommandChannel> = StaticCell::new();
static SYSTEM_STATUS: SystemStatusChannel = SystemStatusChannel::new();
static NET_STATUS: NetStatusChannel = NetStatusChannel::new();
static FAULT_CHANNEL: FaultEventChannel = FaultEventChannel::new();
static FAULT_STATE: FaultStateWatch = FaultStateWatch::new_with(FaultManagerState::Normal);
static DISPLAY_CHANNEL: DisplayChannel = DisplayChannel::new();
static BOOT_READY: BootReadyChannel = BootReadyChannel::new();
static BOOT_FAILURE: BootFailureChannel = BootFailureChannel::new();
static RAILCOM_RUNTIME_RESULTS: RailcomUartRuntimeResultChannel =
    RailcomUartRuntimeResultChannel::new();
static POM_REQUESTS: PomRequestChannel = PomRequestChannel::new();
static POM_RESPONSES: PomResponseChannel = PomResponseChannel::new();
static POM_TX_STARTED: PomTxStartedChannel = PomTxStartedChannel::new();
static POM_RAILCOM_RESULTS: PomRailcomResultChannel = PomRailcomResultChannel::new();

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum BootAction {
    Reset,
    DegradedMode,
}

#[derive(Clone, Copy)]
pub enum BootError {
    OptionalPeripheralInit(OptionalPeripheralInit),
    DccSelfCheck(DccSelfCheckError),
    CriticalHardwareInit(CriticalHardwareInit),
    CriticalTaskSpawn(CriticalTask),
    CriticalTaskInit(CriticalTaskInit),
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum DccSelfCheckError {
    IdlePacketEncoding,
    IdleWaveformBuild,
    ResetPacketEncoding,
    ShortAddress3Invalid,
    Speed28PacketEncoding,
    LongAddress1000Invalid,
    Speed128PacketEncoding,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CriticalHardwareInit {
    Rmt,
    RmtChannel0,
    RmtDriver,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CriticalTask {
    Display,
    DccEngine,
    StatusLed,
    Scheduler,
    RailcomDiag,
    PomActor,
    PomCutoutMonitor,
    RailcomIsrCapture,
    RailcomUartDispatch,
    Net,
    FaultManager,
    StopButton,
    ResumeButton,
    ShortDetector,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CriticalTaskInit {
    Net(NetInitError),
    FaultStateReceiverUnavailable,
    ReadinessTimeout,
}

pub type BootFailureChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, CriticalTaskInit, 4>;

struct NetTaskWrapperContext {
    spawner: Spawner,
    wifi: esp_hal::peripherals::WIFI<'static>,
    scheduler_sender: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
    fault_sender: Sender<'static, CriticalSectionRawMutex, crate::system_status::FaultEvent, 16>,
    channels: NetTaskChannels,
    failure_sender: Sender<'static, CriticalSectionRawMutex, CriticalTaskInit, 4>,
}

impl BootError {
    pub const fn action(self) -> BootAction {
        match self {
            Self::OptionalPeripheralInit(_) => BootAction::DegradedMode,
            Self::DccSelfCheck(_)
            | Self::CriticalHardwareInit(_)
            | Self::CriticalTaskSpawn(_)
            | Self::CriticalTaskInit(_) => BootAction::Reset,
        }
    }

    pub const fn message(self) -> &'static str {
        match self {
            Self::OptionalPeripheralInit(OptionalPeripheralInit::DisplayI2c) => {
                "display I2C init failed"
            }
            Self::OptionalPeripheralInit(OptionalPeripheralInit::DisplayInit) => {
                "display SSD1306 init failed"
            }
            Self::OptionalPeripheralInit(OptionalPeripheralInit::DisplayUnavailable) => {
                "display disabled"
            }
            Self::DccSelfCheck(DccSelfCheckError::IdlePacketEncoding) => {
                "idle packet encoding failed"
            }
            Self::DccSelfCheck(DccSelfCheckError::IdleWaveformBuild) => {
                "idle waveform build failed"
            }
            Self::DccSelfCheck(DccSelfCheckError::ResetPacketEncoding) => {
                "reset packet encoding failed"
            }
            Self::DccSelfCheck(DccSelfCheckError::ShortAddress3Invalid) => {
                "short address 3 must be valid"
            }
            Self::DccSelfCheck(DccSelfCheckError::Speed28PacketEncoding) => {
                "speed28 packet encoding failed"
            }
            Self::DccSelfCheck(DccSelfCheckError::LongAddress1000Invalid) => {
                "long address 1000 must be valid"
            }
            Self::DccSelfCheck(DccSelfCheckError::Speed128PacketEncoding) => {
                "speed128 packet encoding failed"
            }
            Self::CriticalHardwareInit(CriticalHardwareInit::Rmt) => "RMT init failed",
            Self::CriticalHardwareInit(CriticalHardwareInit::RmtChannel0) => {
                "RMT channel0 configure failed"
            }
            Self::CriticalHardwareInit(CriticalHardwareInit::RmtDriver) => {
                "RMT ISR driver init failed"
            }
            Self::CriticalTaskSpawn(CriticalTask::Display) => "failed to spawn display_task",
            Self::CriticalTaskSpawn(CriticalTask::DccEngine) => "failed to spawn dcc_engine_task",
            Self::CriticalTaskSpawn(CriticalTask::StatusLed) => "failed to spawn status_led_task",
            Self::CriticalTaskSpawn(CriticalTask::Scheduler) => "failed to spawn scheduler_task",
            Self::CriticalTaskSpawn(CriticalTask::RailcomDiag) => {
                "failed to spawn railcom_diag_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::PomActor) => "failed to spawn pom_actor_task",
            Self::CriticalTaskSpawn(CriticalTask::PomCutoutMonitor) => {
                "failed to spawn pom_cutout_monitor_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::RailcomIsrCapture) => {
                "failed to spawn railcom_isr_capture_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::RailcomUartDispatch) => {
                "failed to spawn railcom_uart_runtime_dispatch_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::Net) => "failed to spawn net_task",
            Self::CriticalTaskSpawn(CriticalTask::FaultManager) => {
                "failed to spawn fault_manager_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::StopButton) => "failed to spawn stop_button_task",
            Self::CriticalTaskSpawn(CriticalTask::ResumeButton) => {
                "failed to spawn resume_button_task"
            }
            Self::CriticalTaskSpawn(CriticalTask::ShortDetector) => {
                "failed to spawn short_detector_task"
            }
            Self::CriticalTaskInit(CriticalTaskInit::FaultStateReceiverUnavailable) => {
                "fault-state watch receiver already taken"
            }
            Self::CriticalTaskInit(CriticalTaskInit::Net(error)) => error.as_str(),
            Self::CriticalTaskInit(CriticalTaskInit::ReadinessTimeout) => {
                "critical task readiness timeout"
            }
        }
    }
}

fn log_degraded_boot_error(error: BootError) {
    warn!("boot: degraded mode: {}", error.message());
}

async fn announce_ready(
    sender: Sender<'static, CriticalSectionRawMutex, BootReadyEvent, 9>,
    event: BootReadyEvent,
) {
    sender.send(event).await;
    info!("boot: ready ack from {:?}", event);
}

async fn wait_for_runtime_ready(
    receiver: Receiver<'static, CriticalSectionRawMutex, BootReadyEvent, 9>,
    failure_receiver: Receiver<'static, CriticalSectionRawMutex, CriticalTaskInit, 4>,
) -> Result<(), BootError> {
    use embassy_futures::select::{Either, select};

    const READINESS_TIMEOUT: Duration = Duration::from_secs(10);
    const READY_DCC_ENGINE: u16 = 1 << 0;
    const READY_STATUS_LED: u16 = 1 << 1;
    const READY_SCHEDULER: u16 = 1 << 2;
    const READY_NET: u16 = 1 << 3;
    const READY_FAULT_MANAGER: u16 = 1 << 4;
    const READY_STOP_BUTTON: u16 = 1 << 5;
    const READY_RESUME_BUTTON: u16 = 1 << 6;
    const READY_SHORT_DETECTOR: u16 = 1 << 7;
    const REQUIRED_READY_MASK: u16 = READY_DCC_ENGINE
        | READY_STATUS_LED
        | READY_SCHEDULER
        | READY_NET
        | READY_FAULT_MANAGER
        | READY_STOP_BUTTON
        | READY_RESUME_BUTTON
        | READY_SHORT_DETECTOR;

    let mut ready_mask = 0u16;

    while ready_mask != REQUIRED_READY_MASK {
        match with_timeout(
            READINESS_TIMEOUT,
            select(receiver.receive(), failure_receiver.receive()),
        )
        .await
        {
            Ok(Either::First(event)) => {
                info!("boot: readiness received from {:?}", event);
                match event {
                    BootReadyEvent::DisplayReady => {}
                    BootReadyEvent::DisplayDegraded(reason) => {
                        log_degraded_boot_error(BootError::OptionalPeripheralInit(reason));
                    }
                    BootReadyEvent::DccEngine => ready_mask |= READY_DCC_ENGINE,
                    BootReadyEvent::StatusLed => ready_mask |= READY_STATUS_LED,
                    BootReadyEvent::Scheduler => ready_mask |= READY_SCHEDULER,
                    BootReadyEvent::Net => ready_mask |= READY_NET,
                    BootReadyEvent::FaultManager => ready_mask |= READY_FAULT_MANAGER,
                    BootReadyEvent::StopButton => ready_mask |= READY_STOP_BUTTON,
                    BootReadyEvent::ResumeButton => ready_mask |= READY_RESUME_BUTTON,
                    BootReadyEvent::ShortDetector => ready_mask |= READY_SHORT_DETECTOR,
                }
            }
            Ok(Either::Second(failure)) => return Err(BootError::CriticalTaskInit(failure)),
            Err(_) => {
                return Err(BootError::CriticalTaskInit(
                    CriticalTaskInit::ReadinessTimeout,
                ));
            }
        };
    }

    Ok(())
}

fn log_packet_bytes(label: &str, packet: DccPacket, error: BootError) -> Result<(), BootError> {
    let bytes = packet.to_bytes().map_err(|_| error)?;
    info!("{}: {:?}", label, bytes.as_slice());
    Ok(())
}

fn verify_boot_packet_encoding() -> Result<(), BootError> {
    info!("boot: running DCC packet self-check");

    log_packet_bytes(
        "Idle packet",
        DccPacket::Idle,
        BootError::DccSelfCheck(DccSelfCheckError::IdlePacketEncoding),
    )?;
    crate::dcc::build_idle_rmt_buffer()
        .map_err(|_| BootError::DccSelfCheck(DccSelfCheckError::IdleWaveformBuild))?;
    log_packet_bytes(
        "Reset packet",
        DccPacket::Reset,
        BootError::DccSelfCheck(DccSelfCheckError::ResetPacketEncoding),
    )?;

    let short_addr_3 = DccAddress::new_short(3).ok_or(BootError::DccSelfCheck(
        DccSelfCheckError::ShortAddress3Invalid,
    ))?;
    let speed28 = DccPacket::speed_28step(short_addr_3, 14, Direction::Forward).ok_or(
        BootError::DccSelfCheck(DccSelfCheckError::Speed28PacketEncoding),
    )?;
    log_packet_bytes(
        "Speed28 packet (addr=3, fwd, spd=14)",
        speed28,
        BootError::DccSelfCheck(DccSelfCheckError::Speed28PacketEncoding),
    )?;

    let long_addr_1000 = DccAddress::new_long(1000).ok_or(BootError::DccSelfCheck(
        DccSelfCheckError::LongAddress1000Invalid,
    ))?;
    let speed128 = DccPacket::speed_128step(long_addr_1000, 64, Direction::Reverse).ok_or(
        BootError::DccSelfCheck(DccSelfCheckError::Speed128PacketEncoding),
    )?;
    log_packet_bytes(
        "Speed128 packet (addr=1000, rev, spd=64)",
        speed128,
        BootError::DccSelfCheck(DccSelfCheckError::Speed128PacketEncoding),
    )?;

    info!("boot: DCC packet self-check complete");
    Ok(())
}

/// Wrapper task for the DCC engine (single RMT channel on the DRV8874 PH/IN2 input).
#[embassy_executor::task]
async fn dcc_engine_task_wrapper(
    receiver: Receiver<'static, CriticalSectionRawMutex, DccPacket, 16>,
    fault_sender: Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>,
    ready_sender: Sender<'static, CriticalSectionRawMutex, BootReadyEvent, 9>,
) -> ! {
    announce_ready(ready_sender, BootReadyEvent::DccEngine).await;
    dcc_engine_task(receiver, fault_sender).await
}

/// Wrapper task for the Z21 WiFi UDP net task
#[embassy_executor::task]
async fn net_task_wrapper(context: NetTaskWrapperContext) {
    let NetTaskWrapperContext {
        spawner,
        wifi,
        scheduler_sender,
        fault_sender,
        channels,
        failure_sender,
    } = context;

    match net_task(spawner, wifi, scheduler_sender, fault_sender, channels).await {
        Ok(()) => unreachable!("net_task runs forever after successful initialization"),
        Err(error) => {
            defmt::error!("boot: network init failed: {}", error.as_str());
            failure_sender.send(CriticalTaskInit::Net(error)).await;
        }
    }
}

/// Wrapper task for the OLED display
#[embassy_executor::task]
async fn display_task_wrapper(
    i2c: Option<esp_hal::i2c::master::I2c<'static, esp_hal::Async>>,
    receiver: Receiver<'static, CriticalSectionRawMutex, DisplayEvent, 8>,
    ready_sender: Sender<'static, CriticalSectionRawMutex, BootReadyEvent, 9>,
) -> ! {
    crate::display::display_task(i2c, receiver, ready_sender).await
}

/// Wrapper task for the packet scheduler
#[embassy_executor::task]
async fn scheduler_task_wrapper(
    command_receiver: Receiver<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
    sender: Sender<'static, CriticalSectionRawMutex, DccFrame, 16>,
    display_sender: Sender<'static, CriticalSectionRawMutex, DisplayEvent, 8>,
    ready_sender: Sender<'static, CriticalSectionRawMutex, BootReadyEvent, 9>,
) -> ! {
    announce_ready(ready_sender, BootReadyEvent::Scheduler).await;
    packet_scheduler_task(command_receiver, sender, display_sender).await
}

#[embassy_executor::task]
async fn railcom_isr_capture_task_wrapper(
    uart_rx: UartRx<'static, esp_hal::Async>,
    result_sender: Sender<'static, CriticalSectionRawMutex, RailcomRxOutput, 8>,
) -> ! {
    crate::railcom::isr_capture::railcom_isr_capture_task(uart_rx, result_sender).await
}

#[embassy_executor::task]
async fn railcom_uart_runtime_dispatch_task(
    receiver: Receiver<'static, CriticalSectionRawMutex, RailcomRxOutput, 8>,
    pom_result_sender: Sender<'static, CriticalSectionRawMutex, crate::dcc::PomRailcomResult, 4>,
    scheduler_sender: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
) -> ! {
    let mut pending_logon_ch1: Option<(u32, [u8; 2])> = None;
    let mut logon_select_sent_for: Option<(u16, u32)> = None;

    loop {
        let output = receiver.receive().await;
        match output {
            RailcomRxOutput::WindowProcessed(result) => {
                if matches!(result.outcome, RailcomRxOutcome::Parsed) {
                    debug!(
                        "railcom rx ok: packet={} channel={:?} raw_len={} raw={:?} items={:?}",
                        result.window.packet_sequence,
                        result.window.channel,
                        result.window.raw_len,
                        result.window.raw_slice(),
                        result.items.as_slice(),
                    );
                } else if !matches!(result.outcome, RailcomRxOutcome::Empty) {
                    debug!(
                        "railcom rx nonempty: packet={} channel={:?} raw_len={} raw={:?} outcome={:?}",
                        result.window.packet_sequence,
                        result.window.channel,
                        result.window.raw_len,
                        result.window.raw_slice(),
                        result.outcome,
                    );
                }

                let direct_sighting = crate::railcom::loco_tracker::record_loco_identification(
                    result.window.packet_sequence,
                    result.items.as_slice(),
                );
                let packet_metadata =
                    crate::track_output::railcom_packet_metadata(result.window.packet_sequence);
                let target_address = packet_metadata.and_then(|metadata| metadata.target_address);
                let target_sighting = direct_sighting.or_else(|| {
                    target_address.and_then(|target| {
                        crate::railcom::loco_tracker::record_target_from_matching_address_fragment(
                            result.window.packet_sequence,
                            target,
                            result.items.as_slice(),
                        )
                    })
                });
                if let Some(sighting) = target_sighting {
                    // First sighting is a state transition (interesting); repeats
                    // are per-cutout hot-path output and belong at debug level.
                    if sighting.seen_count == 1 {
                        info!(
                            "railcom loco identified: addr={} kind={:?} packet={}",
                            sighting.address.value(),
                            sighting.address.kind(),
                            sighting.packet_sequence,
                        );
                    } else {
                        debug!(
                            "railcom loco: addr={} kind={:?} packet={} seen={}",
                            sighting.address.value(),
                            sighting.address.kind(),
                            sighting.packet_sequence,
                            sighting.seen_count,
                        );
                    }
                }

                match result.window.channel {
                    RailcomChannel::Channel1 if result.window.raw_len == 2 => {
                        let raw = result.window.raw_slice();
                        pending_logon_ch1 = Some((result.window.packet_sequence, [raw[0], raw[1]]));
                    }
                    RailcomChannel::Channel2 => {
                        if let Some((packet_sequence, channel1_raw)) = pending_logon_ch1
                            && packet_sequence == result.window.packet_sequence
                        {
                            if let Ok(response) =
                                parse_logon_response_48(&channel1_raw, result.window.raw_slice())
                            {
                                match response {
                                    RailcomLogonResponse::DecoderId(logon_id) => {
                                        info!(
                                            "railcom logon id: manufacturer={} decoder_id={} packet={}",
                                            logon_id.manufacturer_id,
                                            logon_id.decoder_id,
                                            result.window.packet_sequence,
                                        );
                                        let selected =
                                            (logon_id.manufacturer_id, logon_id.decoder_id);
                                        if logon_select_sent_for != Some(selected) {
                                            let packet = DccPacket::LogonSelect {
                                                manufacturer_id: logon_id.manufacturer_id,
                                                decoder_id: logon_id.decoder_id,
                                                subcommand: 0xff,
                                            };
                                            if scheduler_sender
                                                .try_send(SchedulerCommand::RailcomTelemetry {
                                                    packet,
                                                })
                                                .is_ok()
                                            {
                                                logon_select_sent_for = Some(selected);
                                                info!(
                                                    "railcom logon select queued: manufacturer={} decoder_id={}",
                                                    logon_id.manufacturer_id, logon_id.decoder_id,
                                                );
                                            } else {
                                                warn!(
                                                    "railcom logon select: scheduler channel full"
                                                );
                                            }
                                        }
                                    }
                                    RailcomLogonResponse::Select(logon_select) => {
                                        let sighting =
                                            crate::railcom::loco_tracker::record_identified_address(
                                                result.window.packet_sequence,
                                                logon_select.address,
                                            );
                                        info!(
                                            "railcom logon address: addr={} kind={:?} packet={} seen={}",
                                            sighting.address.value(),
                                            sighting.address.kind(),
                                            sighting.packet_sequence,
                                            sighting.seen_count,
                                        );
                                    }
                                }
                            }
                            pending_logon_ch1 = None;
                        }
                    }
                    RailcomChannel::Channel1 => {}
                }

                if let Some(pom_window) = evaluate_pom_window(
                    result.window.packet_sequence,
                    result.window.channel,
                    packet_metadata,
                    result.items.as_slice(),
                ) {
                    debug!(
                        "railcom pom rx: packet={} channel={:?} metadata_pom={} pom_read={} target={:?} raw_len={} raw={:?} outcome={:?} items={:?}",
                        result.window.packet_sequence,
                        result.window.channel,
                        pom_window.metadata_pom,
                        pom_window.pom_read_candidate,
                        pom_window.target_address,
                        result.window.raw_len,
                        result.window.raw_slice(),
                        result.outcome,
                        result.items.as_slice(),
                    );
                    if let Some(pom_result) = pom_window.result {
                        if pom_window.highlight_result {
                            info!(">>>> railcom pom result: {:?} <<<<", pom_result);
                        } else {
                            info!("railcom pom result: {:?}", pom_result);
                        }
                        if pom_result_sender.try_send(pom_result).is_err() {
                            warn!("railcom uart: pom result channel full");
                        }
                    }
                }
            }
            RailcomRxOutput::WindowCorrupted(window) => {
                info!(
                    "railcom uart corrupted: packet={} channel={:?} raw_len={} glitches={} framing={} raw={:?}",
                    window.packet_sequence,
                    window.channel,
                    window.raw_len,
                    window.glitch_count,
                    window.framing_error_count,
                    &window.raw_bytes[..window.raw_len]
                );
            }
            other => {
                warn!("railcom rx window assembly: {:?}", other);
            }
        }
    }
}

#[embassy_executor::task]
async fn railcom_diag_task() -> ! {
    loop {
        Timer::after(Duration::from_secs(10)).await;

        let diag = crate::railcom::diagnostics();

        info!(
            "railcom diag: boundary={} cutout_grant={} cutout_pom={} logon_sent={} search_sent={} search_throttled={} skip_budget={} skip_priority={} started={} ended={} schedule_fail={} evt_drop={} evt_notify_fail={} rx_windows={} rx_empty={} rx_bytes={} rx_ok={} rx_err={} rx_corrupted={} rx_overflow={} ch1_win={} ch1_empty={} ch2_win={} ch2_empty={} ack={} nack={} adr_high={} adr_low={} loco_id_windows={} loco_id_ok={} loco_id_invalid={}",
            diag.boundary.packet_boundary_count,
            diag.scheduler.cutout_granted_count,
            diag.scheduler.cutout_granted_pom_count,
            diag.scheduler.track_logon_sent_count,
            diag.scheduler.track_search_sent_count,
            diag.scheduler.track_search_throttled_count,
            diag.scheduler.cutout_skipped_budget_count,
            diag.scheduler.cutout_skipped_priority_count,
            diag.track_output.cutout_request_count,
            diag.track_output.cutout_skipped_disabled_count,
            diag.track_output.cutout_started_count,
            diag.track_output.cutout_ended_count,
            diag.track_output.cutout_schedule_fail_count,
            diag.track_output.cutout_event_dropped_count,
            diag.track_output.cutout_event_notify_fail_count,
            diag.rx.rx_window_count(),
            diag.rx.rx_empty_window_count,
            diag.rx.rx_windows_with_bytes_count(),
            diag.rx.rx_parse_ok_count,
            diag.rx.rx_parse_err_count,
            diag.rx.rx_corrupted_window_count,
            diag.rx.rx_overflow_count,
            diag.rx.ch1_window_count,
            diag.rx.ch1_empty_count,
            diag.rx.ch2_window_count,
            diag.rx.ch2_empty_count,
            diag.rx.rx_ack_count,
            diag.rx.rx_nack_count,
            diag.rx.rx_adr_high_count,
            diag.rx.rx_adr_low_count,
            diag.loco.identification_window_count,
            diag.loco.identified_loco_count,
            diag.loco.invalid_identification_count,
        );
    }
}

pub async fn run(
    spawner: Spawner,
    peripherals: esp_hal::peripherals::Peripherals,
) -> Result<(), BootError> {
    info!("boot: starting runtime bootstrap");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);
    info!("boot: Embassy runtime initialized");

    // I2C for OLED display: GPIO19=SDA, GPIO20=SCL, 400 kHz.
    info!("boot: initializing I2C display bus");
    let i2c = match esp_hal::i2c::master::I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(esp_hal::time::Rate::from_khz(400)),
    ) {
        Ok(i2c) => Some(
            i2c.with_sda(peripherals.GPIO19)
                .with_scl(peripherals.GPIO20)
                .into_async(),
        ),
        Err(_) => {
            let error = BootError::OptionalPeripheralInit(OptionalPeripheralInit::DisplayI2c);
            debug_assert_eq!(error.action(), BootAction::DegradedMode);
            log_degraded_boot_error(error);
            None
        }
    };

    spawner
        .spawn(display_task_wrapper(
            i2c,
            DISPLAY_CHANNEL.receiver(),
            BOOT_READY.sender(),
        ))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::Display))?;
    info!("boot: display task spawned");

    // Initialize system status signal first to track startup progression.
    SYSTEM_STATUS.send(SystemStatusEvent::BootStarted).await;
    NET_STATUS.send(SystemStatusEvent::BootStarted).await;

    verify_boot_packet_encoding()?;

    info!("boot: initializing RMT peripheral on GPIO2");

    // 80 MHz RMT base clock; DCC channel uses divider=80 -> 1 us/tick (NMRA S-9.1).
    // channel1 is reserved for WS2812 internal LED at divider=1 (12.5 ns/tick).
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80))
        .map_err(|_| BootError::CriticalHardwareInit(CriticalHardwareInit::Rmt))?;

    // DCC channel: divider=80 -> 80MHz/80 = 1MHz = 1us/tick (NMRA S-9.1 timings unchanged).
    //
    // memsize: 3 -> 144 RAM slots (3 x 48). Two blocks fit idle; the third
    // gives the ISR backend room for RCN-218 LOGON_SELECT packets.
    let tx_config = TxChannelConfig::default()
        .with_clk_divider(80)
        .with_idle_output_level(Level::Low)
        .with_idle_output(true)
        .with_memsize(3);

    // Single RMT channel on GPIO2. With DRV8874 PH/EN wiring, GPIO2 drives
    // PH/IN2 and GPIO4 owns the fast EN/IN1 RailCom cutout.
    let tx_channel = rmt
        .channel0
        .configure_tx(peripherals.GPIO2, tx_config)
        .map_err(|_| BootError::CriticalHardwareInit(CriticalHardwareInit::RmtChannel0))?;
    let idle_rmt = crate::dcc::build_idle_rmt_buffer()
        .map_err(|_| BootError::DccSelfCheck(DccSelfCheckError::IdleWaveformBuild))?;
    rmt_driver::init(tx_channel, idle_rmt.as_slice())
        .map_err(|_| BootError::CriticalHardwareInit(CriticalHardwareInit::RmtDriver))?;

    info!("boot: RMT initialized and DCC output mapped to GPIO2");
    DISPLAY_CHANNEL
        .send(DisplayEvent::BootProgress(BootStep::DccEngineReady))
        .await;

    // Configure status LEDs (active-high): GPIO14=green, GPIO15=red.
    let green_led: Output<'static> = new_led_output(peripherals.GPIO14);
    let red_led: Output<'static> = new_led_output(peripherals.GPIO15);

    let channel = DCC_CHANNEL.init(DccPacketChannel::new());
    let sender = channel.sender();
    let receiver = channel.receiver();
    info!("boot: DCC packet channel initialized");

    let scheduler_commands = SCHEDULER_COMMANDS.init(SchedulerCommandChannel::new());
    let command_receiver = scheduler_commands.receiver();
    info!("boot: scheduler command channel initialized");

    // DRV8874 SLEEP master enable: GPIO18 push-pull, held LOW during init so
    // the track sees no signal until the DCC waveform is already stable.
    // RailCom cutout: GPIO4 drives the fast EN/IN1 cutout path and is held
    // HIGH outside timed cutout windows.
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    let track_output = TrackOutput::new(peripherals.GPIO18, peripherals.GPIO4, timg1.timer0);
    info!("boot: DRV8874 SLEEP GPIO18 held LOW; RailCom cutout GPIO4 held inactive HIGH");

    spawner
        .spawn(dcc_engine_task_wrapper(
            receiver,
            FAULT_CHANNEL.sender(),
            BOOT_READY.sender(),
        ))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::DccEngine))?;
    info!("boot: DCC engine task spawned");

    spawner
        .spawn(status_led_task(
            SYSTEM_STATUS.receiver(),
            green_led,
            red_led,
            DISPLAY_CHANNEL.sender(),
            BOOT_READY.sender(),
        ))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::StatusLed))?;
    info!("boot: status LED task spawned");

    spawner
        .spawn(scheduler_task_wrapper(
            command_receiver,
            sender,
            DISPLAY_CHANNEL.sender(),
            BOOT_READY.sender(),
        ))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::Scheduler))?;
    info!("boot: scheduler task spawned");

    spawner
        .spawn(railcom_diag_task())
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::RailcomDiag))?;
    info!("boot: RailCom diagnostics task spawned");

    spawner
        .spawn(pom_actor_task(
            POM_REQUESTS.receiver(),
            POM_RESPONSES.sender(),
            POM_TX_STARTED.receiver(),
            POM_RAILCOM_RESULTS.receiver(),
            scheduler_commands.sender(),
        ))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::PomActor))?;
    spawner
        .spawn(pom_cutout_monitor_task(POM_TX_STARTED.sender()))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::PomCutoutMonitor))?;
    info!("boot: POM actor and cutout monitor tasks spawned");

    // RailCom RX wiring: UART RX on GPIO5. The cutout timer opens/closes
    // ISR-side capture windows and the task below drains UART1 FIFO snapshots.
    // Gate 6 of the 74HC14 has a weak pull-down; bypass by wiring GPIO5 directly
    // to pin 10 (gate 5 output) and invert the UART RX polarity in firmware.
    let railcom_rx_pin = Flex::new(peripherals.GPIO5)
        .peripheral_input()
        .with_input_inverter(true);
    let railcom_uart_rx = UartRx::new(peripherals.UART1, railcom_uart_rx_config())
        .map_err(|_| BootError::CriticalHardwareInit(CriticalHardwareInit::RailcomUart))?
        .with_rx(railcom_rx_pin)
        .into_async();

    spawner
        .spawn(railcom_isr_capture_task_wrapper(
            railcom_uart_rx,
            RAILCOM_RUNTIME_RESULTS.sender(),
        ))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::RailcomIsrCapture))?;
    spawner
        .spawn(railcom_uart_runtime_dispatch_task(
            RAILCOM_RUNTIME_RESULTS.receiver(),
            POM_RAILCOM_RESULTS.sender(),
            scheduler_commands.sender(),
        ))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::RailcomUartDispatch))?;
    info!("boot: RailCom ISR capture spawned on GPIO5 (POM dispatcher active)");

    spawner
        .spawn(net_task_wrapper(NetTaskWrapperContext {
            spawner,
            wifi: peripherals.WIFI,
            scheduler_sender: scheduler_commands.sender(),
            fault_sender: FAULT_CHANNEL.sender(),
            channels: NetTaskChannels {
                net_status: NET_STATUS.receiver(),
                status_sender: SYSTEM_STATUS.sender(),
                display_sender: DISPLAY_CHANNEL.sender(),
                ready_sender: BOOT_READY.sender(),
            },
            failure_sender: BOOT_FAILURE.sender(),
        }))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::Net))?;
    info!("boot: network task spawned");
    DISPLAY_CHANNEL
        .send(DisplayEvent::BootProgress(BootStep::WifiConnecting))
        .await;

    info!("boot: core runtime spawned, establishing DCC output");

    // Power-on idle burst: establishes a stable DCC waveform before the track
    // is energised. 20 idle packets ≈ 160 ms of clean signal ensures the decoder
    // does not see a DC transient and enter analog mode at startup.
    for _ in 0..20 {
        sender.send(DccPacket::Idle).await;
    }
    info!("boot: power-on idle burst complete (20 packets)");

    spawner
        .spawn(fault_manager_task(FaultManagerTaskContext {
            receiver: FAULT_CHANNEL.receiver(),
            scheduler_sender: scheduler_commands.sender(),
            status_sender: SYSTEM_STATUS.sender(),
            net_status_sender: NET_STATUS.sender(),
            hbridge_enable,
            display_sender: DISPLAY_CHANNEL.sender(),
            state_sender: FAULT_STATE.sender(),
            ready_sender: BOOT_READY.sender(),
        }))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::FaultManager))?;
    info!("boot: fault manager task spawned");

    // Control buttons: GPIO22=stop (active-low), GPIO21=resume (active-low).
    // Spawned before the reset sequence so button events are never missed.
    let stop_btn = new_button_input(peripherals.GPIO22);
    let resume_btn = new_button_input(peripherals.GPIO21);
    spawner
        .spawn(stop_button_task(
            stop_btn,
            FAULT_CHANNEL.sender(),
            BOOT_READY.sender(),
        ))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::StopButton))?;
    spawner
        .spawn(resume_button_task(
            resume_btn,
            FAULT_CHANNEL.sender(),
            BOOT_READY.sender(),
        ))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::ResumeButton))?;
    info!("boot: control button tasks spawned (GPIO22=stop, GPIO21=resume)");

    // NMRA S-9.2.4: send >=3 Reset packets so the decoder clears its state and
    // enters normal operations mode. Without this some decoders ignore commands.
    for _ in 0..5 {
        sender.send(DccPacket::Reset).await;
    }
    info!("boot: reset sequence complete (5 packets)");

    // Brief delay to let decoder finish reset processing.
    Timer::after(Duration::from_millis(100)).await;

    // Track short detector: GPIO3 monitors the active-low conditioned
    // overcurrent/fault output from the external detector.
    // Spawned after reset + stabilization so the decoder is quiescent.
    let short_pin = new_short_detect_input(peripherals.GPIO3);
    let fault_state_receiver = FAULT_STATE.receiver().ok_or(BootError::CriticalTaskInit(
        CriticalTaskInit::FaultStateReceiverUnavailable,
    ))?;

    spawner
        .spawn(short_detector_task(
            short_pin,
            FAULT_CHANNEL.sender(),
            fault_state_receiver,
            BOOT_READY.sender(),
        ))
        .map_err(|_| BootError::CriticalTaskSpawn(CriticalTask::ShortDetector))?;
    info!("boot: short detector task spawned (GPIO3 interrupt)");

    info!("boot: waiting for critical task readiness");
    wait_for_runtime_ready(BOOT_READY.receiver(), BOOT_FAILURE.receiver()).await?;
    FAULT_CHANNEL.send(FaultEvent::TrackPowerArmed).await;
    info!("boot: track power armed after critical runtime readiness");
    DISPLAY_CHANNEL
        .send(DisplayEvent::BootProgress(BootStep::SystemRunning))
        .await;
    SYSTEM_STATUS.send(SystemStatusEvent::BootCompleted).await;
    NET_STATUS.send(SystemStatusEvent::BootCompleted).await;
    info!("boot: critical runtime ready, boot completed");

    info!("boot: bootstrap complete");
    Ok(())
}
