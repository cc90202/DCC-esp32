//! Boot-time orchestration for the firmware runtime.
//!
//! This module owns the startup sequence that wires hardware resources,
//! spawns Embassy tasks, performs DCC self-checks and only then energises the
//! track output. The goal is to keep `main` focused on top-level policy while
//! the boot ordering remains explicit and logged in one place.

mod error;
mod hardware;
mod provisioning;
mod readiness;
mod runtime_tasks;
mod self_check;
mod startup_sequence;

pub(crate) use error::BootFailureChannel;
use error::log_degraded_boot_error;
pub use error::{
    BootAction, BootError, CriticalHardwareInit, CriticalTaskInit, DccSelfCheckError,
    WifiConfigInitError,
};
use hardware::{
    initialize_dcc_rmt, initialize_display_bus, initialize_railcom_receiver, start_async_runtime,
};
use provisioning::{open_wifi_config_store, provisioning_request_task};
use readiness::wait_for_runtime_ready;
#[cfg(feature = "bench-diag")]
use runtime_tasks::railcom_diag_task;
use runtime_tasks::{
    NetTaskWrapperContext, dcc_engine_task_wrapper, display_task_wrapper, net_task_wrapper,
    railcom_isr_capture_task_wrapper, scheduler_task_wrapper,
};
use self_check::verify_boot_packet_encoding;
use startup_sequence::{send_decoder_reset_sequence, send_power_on_idle_burst};

use defmt::{info, warn};
use embassy_executor::{SpawnToken, Spawner};
use esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN;
use esp_hal::gpio::{Input, Output};
use esp_hal::timer::timg::TimerGroup;
use static_cell::StaticCell;

use crate::control_buttons::{
    ProvisioningRequestChannel, new_button_input, resume_button_task, stop_button_task,
    wait_for_boot_provisioning_override,
};
use crate::dcc::channels::{
    DccFrameSender, DccPacketChannel, LocoRequestChannel, LocoResponseChannel,
    PomRailcomResultChannel, PomRequestChannel, PomResponseChannel, PomTxStartedChannel,
    PowerFenceAckChannel, PowerQuiesceChannel, SchedulerCommandChannel,
};
use crate::dcc::pom_actor_task;
use crate::fault_manager::{
    FaultEffectsSignal, FaultEffectsTaskContext, FaultManagerTaskContext, fault_effects_task,
    fault_manager_task,
};
use crate::net::client_watchdog::{
    ClientWatchdogContext, LeaseRequestChannel, LeaseResponseChannel, LeaseTripChannel,
    client_watchdog_task,
};
use crate::net::provisioning::run_provisioning_ap;
use crate::net::udp_control::NetTaskChannels;
use crate::net::wifi_config::{
    EspWifiConfigStoreError, ProvisioningDecision, SetupFlag, WifiCredentials,
    load_wifi_credentials_and_decision,
};
use crate::railcom::pom_dispatch::pom_cutout_monitor_task;
use crate::railcom::runtime_dispatch::railcom_uart_runtime_dispatch_task;
use crate::railcom::uart_reader::RailcomUartRuntimeResultChannel;
use crate::short_detector::{new_short_detect_input, short_detector_task};
use crate::station_status::{FaultManagerState, SystemStatusEvent};
use crate::status_led::{new_led_output, provisioning_led_task, status_led_task};
use crate::system_status::channels::{
    BootReadyChannel, DisplayChannel, FaultEventChannel, FaultStateWatch, NetStatusChannel,
    SystemStatusChannel,
};
use crate::system_status::{BootStep, CriticalTask, DisplayEvent};
use crate::track_output::TrackOutput;

// Static channels/signals shared across Embassy tasks.
static DCC_CHANNEL: StaticCell<DccPacketChannel> = StaticCell::new();
static SCHEDULER_COMMANDS: StaticCell<SchedulerCommandChannel> = StaticCell::new();
static POWER_QUIESCE: PowerQuiesceChannel = PowerQuiesceChannel::new();
static POWER_FENCE_ACKS: PowerFenceAckChannel = PowerFenceAckChannel::new();
static LOCO_REQUESTS: LocoRequestChannel = LocoRequestChannel::new();
static LOCO_RESPONSES: LocoResponseChannel = LocoResponseChannel::new();
static SYSTEM_STATUS: SystemStatusChannel = SystemStatusChannel::new();
static NET_STATUS: NetStatusChannel = NetStatusChannel::new();
static FAULT_CHANNEL: FaultEventChannel = FaultEventChannel::new();
static FAULT_STATE: FaultStateWatch = FaultStateWatch::new_with(FaultManagerState::Normal);
static FAULT_EFFECTS: FaultEffectsSignal = FaultEffectsSignal::new();
static DISPLAY_CHANNEL: DisplayChannel = DisplayChannel::new();
static BOOT_READY: BootReadyChannel = BootReadyChannel::new();
static BOOT_FAILURE: BootFailureChannel = BootFailureChannel::new();
static LEASE_REQUESTS: LeaseRequestChannel = LeaseRequestChannel::new();
static LEASE_RESPONSES: LeaseResponseChannel = LeaseResponseChannel::new();
static LEASE_TRIPS: LeaseTripChannel = LeaseTripChannel::new();
static PROVISIONING_REQUESTS: ProvisioningRequestChannel = ProvisioningRequestChannel::new();
static RAILCOM_RUNTIME_RESULTS: RailcomUartRuntimeResultChannel =
    RailcomUartRuntimeResultChannel::new();
static POM_REQUESTS: PomRequestChannel = PomRequestChannel::new();
static POM_RESPONSES: PomResponseChannel = PomResponseChannel::new();
static POM_TX_STARTED: PomTxStartedChannel = PomTxStartedChannel::new();
static POM_RAILCOM_RESULTS: PomRailcomResultChannel = PomRailcomResultChannel::new();
static WIFI_PARTITION_TABLE_BUFFER: StaticCell<[u8; PARTITION_TABLE_MAX_LEN]> = StaticCell::new();

fn spawn_critical<S>(
    spawner: Spawner,
    token: SpawnToken<S>,
    task: CriticalTask,
) -> Result<(), BootError> {
    spawner
        .spawn(token)
        .map_err(|_| BootError::CriticalTaskSpawn(task))
}

struct WifiBootDecision {
    decision: ProvisioningDecision,
    credentials: Option<WifiCredentials>,
    partition_table_buffer: &'static mut [u8; PARTITION_TABLE_MAX_LEN],
    resume_button: Input<'static>,
}

struct DccCore {
    sender: DccFrameSender,
    scheduler_commands: &'static SchedulerCommandChannel,
    track_output: TrackOutput,
}

struct DccCoreHardware {
    rmt: esp_hal::peripherals::RMT<'static>,
    dcc_pin: esp_hal::peripherals::GPIO2<'static>,
    timer: esp_hal::peripherals::TIMG1<'static>,
    track_enable_pin: esp_hal::peripherals::GPIO18<'static>,
    cutout_pin: esp_hal::peripherals::GPIO4<'static>,
    green_led_pin: esp_hal::peripherals::GPIO14<'static>,
    red_led_pin: esp_hal::peripherals::GPIO15<'static>,
}

fn spawn_display(
    spawner: &Spawner,
    i2c0: esp_hal::peripherals::I2C0<'static>,
    sda: esp_hal::peripherals::GPIO19<'static>,
    scl: esp_hal::peripherals::GPIO20<'static>,
) -> Result<(), BootError> {
    info!("boot: initializing I2C display bus");
    let i2c = match initialize_display_bus(i2c0, sda, scl) {
        Ok(i2c) => Some(i2c),
        Err(error) => {
            debug_assert_eq!(error.action(), BootAction::DegradedMode);
            log_degraded_boot_error(error);
            None
        }
    };

    spawn_critical(
        *spawner,
        display_task_wrapper(i2c, DISPLAY_CHANNEL.receiver(), BOOT_READY.sender()),
        CriticalTask::Display,
    )?;
    info!("boot: display task spawned");
    Ok(())
}

async fn load_boot_wifi_decision(
    resume_pin: esp_hal::peripherals::GPIO21<'static>,
) -> Result<WifiBootDecision, BootError> {
    let mut resume_button = new_button_input(resume_pin);
    let boot_button_override = wait_for_boot_provisioning_override(&mut resume_button).await;
    let mut flash = provisioning::lock_flash().await?;
    let partition_table_buffer = WIFI_PARTITION_TABLE_BUFFER.init([0; PARTITION_TABLE_MAX_LEN]);
    // A trial or recovery boot must not use up a pending setup request.
    let flag = if crate::ota::on_trial() || crate::ota::in_recovery() {
        SetupFlag::Keep
    } else {
        SetupFlag::Consume
    };

    // Single decision path: button override, persistent flag and stored
    // credentials all flow through `decide_provisioning`.
    let (decision, credentials) = {
        let mut store = open_wifi_config_store(&mut flash, partition_table_buffer)?;
        load_wifi_credentials_and_decision(&mut store, boot_button_override, flag)
            .map_err(WifiConfigInitError::Store)?
    };

    Ok(WifiBootDecision {
        decision,
        credentials,
        partition_table_buffer,
        resume_button,
    })
}

async fn start_dcc_core(
    spawner: &Spawner,
    hardware: DccCoreHardware,
) -> Result<DccCore, BootError> {
    let DccCoreHardware {
        rmt,
        dcc_pin,
        timer,
        track_enable_pin,
        cutout_pin,
        green_led_pin,
        red_led_pin,
    } = hardware;

    verify_boot_packet_encoding()?;

    info!("boot: initializing RMT peripheral on GPIO2");
    initialize_dcc_rmt(rmt, dcc_pin)?;
    info!("boot: RMT initialized and DCC output mapped to GPIO2");
    DISPLAY_CHANNEL
        .send(DisplayEvent::BootProgress(BootStep::DccEngineReady))
        .await;

    let green_led: Output<'static> = new_led_output(green_led_pin);
    let red_led: Output<'static> = new_led_output(red_led_pin);

    let channel = DCC_CHANNEL.init(DccPacketChannel::new());
    let sender = channel.sender();
    let receiver = channel.receiver();
    info!("boot: DCC packet channel initialized");

    let scheduler_commands = SCHEDULER_COMMANDS.init(SchedulerCommandChannel::new());
    let command_receiver = scheduler_commands.receiver();
    info!("boot: scheduler command channel initialized");

    // The track sees no signal until the DCC waveform is stable. GPIO4 is
    // timer-driven for RailCom cutouts.
    let timg1 = TimerGroup::new(timer);
    let track_output = TrackOutput::new(track_enable_pin, cutout_pin, timg1.timer0);
    info!("boot: track SLEEP GPIO18 held LOW; RailCom cutout/run GPIO4 held inactive HIGH");

    spawn_critical(
        *spawner,
        client_watchdog_task(ClientWatchdogContext {
            request_receiver: LEASE_REQUESTS.receiver(),
            response_sender: LEASE_RESPONSES.sender(),
            trip_sender: LEASE_TRIPS.sender(),
            fault_sender: FAULT_CHANNEL.sender(),
            ready_sender: BOOT_READY.sender(),
        }),
        CriticalTask::LeaseWatchdog,
    )?;

    spawn_critical(
        *spawner,
        dcc_engine_task_wrapper(
            receiver,
            FAULT_CHANNEL.sender(),
            POWER_FENCE_ACKS.sender(),
            BOOT_READY.sender(),
        ),
        CriticalTask::DccEngine,
    )?;
    info!("boot: DCC engine task spawned");

    spawn_critical(
        *spawner,
        status_led_task(
            SYSTEM_STATUS.receiver(),
            green_led,
            red_led,
            DISPLAY_CHANNEL.sender(),
            BOOT_READY.sender(),
        ),
        CriticalTask::StatusLed,
    )?;
    info!("boot: status LED task spawned");

    spawn_critical(
        *spawner,
        scheduler_task_wrapper(
            POWER_QUIESCE.receiver(),
            command_receiver,
            LOCO_REQUESTS.receiver(),
            LOCO_RESPONSES.sender(),
            sender,
            DISPLAY_CHANNEL.sender(),
            BOOT_READY.sender(),
        ),
        CriticalTask::Scheduler,
    )?;
    info!("boot: scheduler task spawned");

    #[cfg(feature = "bench-diag")]
    {
        spawn_critical(*spawner, railcom_diag_task(), CriticalTask::RailcomDiag)?;
        info!("boot: RailCom diagnostics task spawned");
    }

    spawn_critical(
        *spawner,
        pom_actor_task(
            POM_REQUESTS.receiver(),
            POM_RESPONSES.sender(),
            POM_TX_STARTED.receiver(),
            POM_RAILCOM_RESULTS.receiver(),
            scheduler_commands.sender(),
        ),
        CriticalTask::PomActor,
    )?;
    spawn_critical(
        *spawner,
        pom_cutout_monitor_task(POM_TX_STARTED.sender()),
        CriticalTask::PomCutoutMonitor,
    )?;
    info!("boot: POM actor and cutout monitor tasks spawned");

    Ok(DccCore {
        sender,
        scheduler_commands,
        track_output,
    })
}

fn spawn_railcom_runtime(
    spawner: &Spawner,
    uart: esp_hal::peripherals::UART1<'static>,
    rx_pin: esp_hal::peripherals::GPIO5<'static>,
    scheduler_commands: &'static SchedulerCommandChannel,
) -> Result<(), BootError> {
    let railcom_uart_rx = initialize_railcom_receiver(uart, rx_pin)?;
    spawn_critical(
        *spawner,
        railcom_isr_capture_task_wrapper(railcom_uart_rx, RAILCOM_RUNTIME_RESULTS.sender()),
        CriticalTask::RailcomIsrCapture,
    )?;
    spawn_critical(
        *spawner,
        railcom_uart_runtime_dispatch_task(
            RAILCOM_RUNTIME_RESULTS.receiver(),
            POM_RAILCOM_RESULTS.sender(),
            scheduler_commands.sender(),
        ),
        CriticalTask::RailcomUartDispatch,
    )?;
    info!("boot: RailCom ISR capture spawned on GPIO5 (POM dispatcher active)");
    Ok(())
}

async fn spawn_network_runtime(
    spawner: Spawner,
    wifi: esp_hal::peripherals::WIFI<'static>,
    credentials: WifiCredentials,
) -> Result<(), BootError> {
    spawn_critical(
        spawner,
        net_task_wrapper(NetTaskWrapperContext {
            spawner,
            wifi,
            credentials,
            fault_sender: FAULT_CHANNEL.sender(),
            channels: NetTaskChannels {
                net_status: NET_STATUS.receiver(),
                status_sender: SYSTEM_STATUS.sender(),
                display_sender: DISPLAY_CHANNEL.sender(),
                ready_sender: BOOT_READY.sender(),
                pom_request_sender: POM_REQUESTS.sender(),
                pom_response_receiver: POM_RESPONSES.receiver(),
                loco_request_sender: LOCO_REQUESTS.sender(),
                loco_response_receiver: LOCO_RESPONSES.receiver(),
                lease_request_sender: LEASE_REQUESTS.sender(),
                lease_response_receiver: LEASE_RESPONSES.receiver(),
                lease_trip_receiver: LEASE_TRIPS.receiver(),
            },
            failure_sender: BOOT_FAILURE.sender(),
        }),
        CriticalTask::Net,
    )?;
    info!("boot: network task spawned");
    DISPLAY_CHANNEL
        .send(DisplayEvent::BootProgress(BootStep::WifiConnecting))
        .await;
    Ok(())
}

struct SafetyRuntimeResources {
    partition_table_buffer: &'static mut [u8; PARTITION_TABLE_MAX_LEN],
    resume_button: Input<'static>,
    stop_pin: esp_hal::peripherals::GPIO22<'static>,
    short_detect_pin: esp_hal::peripherals::GPIO3<'static>,
}

async fn activate_safe_runtime(
    spawner: &Spawner,
    core: DccCore,
    resources: SafetyRuntimeResources,
) -> Result<(), BootError> {
    let DccCore {
        sender,
        scheduler_commands,
        track_output,
    } = core;
    let SafetyRuntimeResources {
        partition_table_buffer,
        resume_button,
        stop_pin,
        short_detect_pin,
    } = resources;

    info!("boot: core runtime spawned, establishing DCC output");
    send_power_on_idle_burst(sender).await;
    info!("boot: power-on idle burst complete (20 packets)");

    spawn_critical(
        *spawner,
        fault_effects_task(FaultEffectsTaskContext {
            effects_signal: &FAULT_EFFECTS,
            scheduler_sender: scheduler_commands.sender(),
            status_sender: SYSTEM_STATUS.sender(),
            net_status_sender: NET_STATUS.sender(),
            display_sender: DISPLAY_CHANNEL.sender(),
        }),
        CriticalTask::FaultEffects,
    )?;
    spawn_critical(
        *spawner,
        fault_manager_task(FaultManagerTaskContext {
            receiver: FAULT_CHANNEL.receiver(),
            power_quiesce_sender: POWER_QUIESCE.sender(),
            power_fence_ack_receiver: POWER_FENCE_ACKS.receiver(),
            track_output,
            state_sender: FAULT_STATE.sender(),
            effects_signal: &FAULT_EFFECTS,
            ready_sender: BOOT_READY.sender(),
        }),
        CriticalTask::FaultManager,
    )?;
    info!("boot: fault manager task spawned");

    spawn_critical(
        *spawner,
        provisioning_request_task(partition_table_buffer, PROVISIONING_REQUESTS.receiver()),
        CriticalTask::ProvisioningRequest,
    )?;
    info!("boot: provisioning request task spawned");

    let stop_button = new_button_input(stop_pin);
    spawn_critical(
        *spawner,
        stop_button_task(stop_button, FAULT_CHANNEL.sender(), BOOT_READY.sender()),
        CriticalTask::StopButton,
    )?;
    spawn_critical(
        *spawner,
        resume_button_task(
            resume_button,
            FAULT_CHANNEL.sender(),
            PROVISIONING_REQUESTS.sender(),
            BOOT_READY.sender(),
        ),
        CriticalTask::ResumeButton,
    )?;
    info!("boot: control button tasks spawned (GPIO22=stop, GPIO21=resume)");

    send_decoder_reset_sequence(sender).await;
    info!("boot: reset sequence complete (5 packets)");

    let short_pin = new_short_detect_input(short_detect_pin);
    let fault_state_receiver = FAULT_STATE
        .receiver()
        .ok_or(CriticalTaskInit::FaultStateReceiverUnavailable)?;
    spawn_critical(
        *spawner,
        short_detector_task(
            short_pin,
            FAULT_CHANNEL.sender(),
            fault_state_receiver,
            BOOT_READY.sender(),
        ),
        CriticalTask::ShortDetector,
    )?;
    info!("boot: short detector task spawned (GPIO3 interrupt)");

    info!("boot: waiting for critical task readiness");
    wait_for_runtime_ready(BOOT_READY.receiver(), BOOT_FAILURE.receiver()).await?;
    crate::ota::complete_boot().await;
    info!("boot: runtime ready; OTA policy controls track arming");
    DISPLAY_CHANNEL
        .send(DisplayEvent::BootProgress(BootStep::SystemRunning))
        .await;
    SYSTEM_STATUS.send(SystemStatusEvent::BootCompleted).await;
    NET_STATUS.send(SystemStatusEvent::BootCompleted).await;
    info!("boot: critical runtime ready, boot completed");
    Ok(())
}

struct SetupHardware {
    wifi: esp_hal::peripherals::WIFI<'static>,
    green_led_pin: esp_hal::peripherals::GPIO14<'static>,
    red_led_pin: esp_hal::peripherals::GPIO15<'static>,
}

/// Runs the Wi-Fi setup access point; DCC, RailCom, Z21 and the track stay
/// off. Returns only on failure, or at once in update recovery.
async fn run_setup_mode(
    spawner: Spawner,
    hardware: SetupHardware,
    partition_table_buffer: &'static mut [u8; PARTITION_TABLE_MAX_LEN],
) -> Result<(), BootError> {
    // A candidate cannot prove itself without the network: leave its trial
    // without blaming it; that restart keeps the setup request for the
    // confirmed image.
    if crate::ota::on_trial()
        && let Err(error) = crate::ota::withdraw_trial().await
    {
        crate::ota::enter_recovery(error);
    }
    if crate::ota::in_recovery() {
        return Ok(());
    }
    warn!("boot: DCC, RailCom, Z21, and track output remain disabled");

    // Setup-mode LED feedback: red blink, same pattern as WiFi connecting.
    spawn_critical(
        spawner,
        provisioning_led_task(
            new_led_output(hardware.green_led_pin),
            new_led_output(hardware.red_led_pin),
        ),
        CriticalTask::ProvisioningLed,
    )?;

    // Setup mode is terminal and ends in a restart, and no update runs in it:
    // holding the flash for its whole life is intended.
    let mut flash = provisioning::lock_flash().await?;
    let store = open_wifi_config_store(&mut flash, partition_table_buffer)?;
    run_provisioning_ap(spawner, hardware.wifi, store, DISPLAY_CHANNEL.sender())
        .await
        .map_err(|error| CriticalTaskInit::ProvisioningAp(error).into())
}

/// The Wi-Fi boot decision. In update recovery a failure ends the boot
/// quietly instead: the recovery screen must stay, and a reset loop caused
/// by a boot error would hide it.
async fn wifi_decision_unless_recovery(
    resume_pin: esp_hal::peripherals::GPIO21<'static>,
) -> Result<Option<WifiBootDecision>, BootError> {
    match load_boot_wifi_decision(resume_pin).await {
        Ok(decision) => Ok(Some(decision)),
        Err(error) if crate::ota::in_recovery() => {
            defmt::error!("Recovery WiFi unavailable: {}", error.message());
            Ok(None)
        }
        Err(error) => Err(error),
    }
}

pub async fn run(
    spawner: Spawner,
    mut peripherals: esp_hal::peripherals::Peripherals,
) -> Result<(), BootError> {
    info!("boot: starting runtime bootstrap");

    // Hold the physical bridge LOW before any flash access or boot decision,
    // until `TrackOutput` takes the pin over in `start_dcc_core`.
    let early_off = Output::new(
        peripherals.GPIO18.reborrow(),
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    start_async_runtime(peripherals.TIMG0, peripherals.SW_INTERRUPT);
    info!("boot: Embassy runtime initialized");
    // The display starts first, so it can show an update rolling back
    // during the boot check below.
    spawn_display(
        &spawner,
        peripherals.I2C0,
        peripherals.GPIO19,
        peripherals.GPIO20,
    )?;
    crate::ota::start(
        spawner,
        peripherals.FLASH,
        peripherals.LPWR,
        FAULT_CHANNEL.sender(),
        DISPLAY_CHANNEL.sender(),
    )
    .await;

    SYSTEM_STATUS.send(SystemStatusEvent::BootStarted).await;
    NET_STATUS.send(SystemStatusEvent::BootStarted).await;

    let Some(wifi_decision) = wifi_decision_unless_recovery(peripherals.GPIO21).await? else {
        return Ok(());
    };
    let WifiBootDecision {
        decision,
        credentials,
        partition_table_buffer,
        resume_button,
    } = wifi_decision;

    let credentials = match decision {
        ProvisioningDecision::StationMode => credentials.ok_or(WifiConfigInitError::Store(
            EspWifiConfigStoreError::MissingCredentials,
        ))?,
        ProvisioningDecision::ProvisioningMode(reason) => {
            warn!(
                "boot: WiFi provisioning selected ({:?}); safe setup mode active",
                reason
            );
            return run_setup_mode(
                spawner,
                SetupHardware {
                    wifi: peripherals.WIFI,
                    green_led_pin: peripherals.GPIO14,
                    red_led_pin: peripherals.GPIO15,
                },
                partition_table_buffer,
            )
            .await;
        }
    };

    // `Output` has no `Drop`, so the pin stays LOW after this line; the drop
    // keeps GPIO18 borrowed until here, so nothing else can claim it before
    // `TrackOutput` takes it over.
    #[expect(clippy::drop_non_drop, reason = "ends the borrow of GPIO18 here")]
    drop(early_off);
    let core = start_dcc_core(
        &spawner,
        DccCoreHardware {
            rmt: peripherals.RMT,
            dcc_pin: peripherals.GPIO2,
            timer: peripherals.TIMG1,
            track_enable_pin: peripherals.GPIO18,
            cutout_pin: peripherals.GPIO4,
            green_led_pin: peripherals.GPIO14,
            red_led_pin: peripherals.GPIO15,
        },
    )
    .await?;
    spawn_railcom_runtime(
        &spawner,
        peripherals.UART1,
        peripherals.GPIO5,
        core.scheduler_commands,
    )?;
    spawn_network_runtime(spawner, peripherals.WIFI, credentials).await?;

    activate_safe_runtime(
        &spawner,
        core,
        SafetyRuntimeResources {
            partition_table_buffer,
            resume_button,
            stop_pin: peripherals.GPIO22,
            short_detect_pin: peripherals.GPIO3,
        },
    )
    .await?;

    info!("boot: bootstrap complete");
    Ok(())
}
