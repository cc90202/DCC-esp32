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
mod runtime_channels;
mod runtime_tasks;
mod self_check;
mod startup_sequence;

use error::log_degraded_boot_error;
pub use error::{
    BootAction, BootError, BootFailureChannel, CriticalHardwareInit, CriticalTask,
    CriticalTaskInit, DccSelfCheckError, WifiConfigInitError,
};
use hardware::{
    initialize_dcc_rmt, initialize_display_bus, initialize_railcom_receiver, start_async_runtime,
};
use provisioning::{open_wifi_config_store, provisioning_request_task};
use readiness::wait_for_runtime_ready;
use runtime_tasks::{
    NetTaskWrapperContext, dcc_engine_task_wrapper, display_task_wrapper, net_task_wrapper,
    railcom_diag_task, railcom_isr_capture_task_wrapper, scheduler_task_wrapper,
};
use self_check::verify_boot_packet_encoding;
use startup_sequence::{send_decoder_reset_sequence, send_power_on_idle_burst};

use defmt::{info, warn};
use embassy_executor::{SpawnToken, Spawner};
use esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN;
use esp_hal::gpio::Output;
use esp_hal::timer::timg::TimerGroup;
use esp_storage::FlashStorage;
use static_cell::StaticCell;

use crate::control_buttons::{
    ProvisioningRequestChannel, new_button_input, resume_button_task, stop_button_task,
    wait_for_boot_provisioning_override,
};
use crate::dcc::{
    LocoRequestChannel, LocoResponseChannel, PomRailcomResultChannel, PomRequestChannel,
    PomResponseChannel, PomTxStartedChannel, SchedulerCommandChannel, pom_actor_task,
};
use crate::dcc_runtime::DccPacketChannel;
use crate::fault_manager::{
    FaultManagerState, FaultManagerTaskContext, FaultStateWatch, fault_manager_task,
};
use crate::net::provisioning::run_provisioning_ap;
use crate::net::udp_control::NetTaskChannels;
use crate::net::wifi_config::{
    ProvisioningDecision, StoreError, load_wifi_credentials_and_decision,
};
use crate::railcom::pom_dispatch::pom_cutout_monitor_task;
use crate::railcom::runtime_dispatch::railcom_uart_runtime_dispatch_task;
use crate::railcom::uart_reader::RailcomUartRuntimeResultChannel;
use crate::short_detector::{new_short_detect_input, short_detector_task};
use crate::status_led::{new_led_output, provisioning_led_task, status_led_task};
use crate::system_status::{BootStep, DisplayEvent, FaultEvent, SystemStatusEvent};
use crate::track_output::TrackOutput;
use runtime_channels::{
    BootReadyChannel, DisplayChannel, FaultEventChannel, NetStatusChannel, SystemStatusChannel,
};

// Static channels/signals shared across Embassy tasks.
static DCC_CHANNEL: StaticCell<DccPacketChannel> = StaticCell::new();
static SCHEDULER_COMMANDS: StaticCell<SchedulerCommandChannel> = StaticCell::new();
static LOCO_REQUESTS: LocoRequestChannel = LocoRequestChannel::new();
static LOCO_RESPONSES: LocoResponseChannel = LocoResponseChannel::new();
static SYSTEM_STATUS: SystemStatusChannel = SystemStatusChannel::new();
static NET_STATUS: NetStatusChannel = NetStatusChannel::new();
static FAULT_CHANNEL: FaultEventChannel = FaultEventChannel::new();
static FAULT_STATE: FaultStateWatch = FaultStateWatch::new_with(FaultManagerState::Normal);
static DISPLAY_CHANNEL: DisplayChannel = DisplayChannel::new();
static BOOT_READY: BootReadyChannel = BootReadyChannel::new();
static BOOT_FAILURE: BootFailureChannel = BootFailureChannel::new();
static PROVISIONING_REQUESTS: ProvisioningRequestChannel = ProvisioningRequestChannel::new();
static RAILCOM_RUNTIME_RESULTS: RailcomUartRuntimeResultChannel =
    RailcomUartRuntimeResultChannel::new();
static POM_REQUESTS: PomRequestChannel = PomRequestChannel::new();
static POM_RESPONSES: PomResponseChannel = PomResponseChannel::new();
static POM_TX_STARTED: PomTxStartedChannel = PomTxStartedChannel::new();
static POM_RAILCOM_RESULTS: PomRailcomResultChannel = PomRailcomResultChannel::new();
static WIFI_PARTITION_TABLE_BUFFER: StaticCell<[u8; PARTITION_TABLE_MAX_LEN]> = StaticCell::new();

fn spawn_critical<S>(
    spawner: &Spawner,
    token: SpawnToken<S>,
    task: CriticalTask,
) -> Result<(), BootError> {
    spawner
        .spawn(token)
        .map_err(|_| BootError::CriticalTaskSpawn(task))
}

pub async fn run(
    spawner: Spawner,
    peripherals: esp_hal::peripherals::Peripherals,
) -> Result<(), BootError> {
    info!("boot: starting runtime bootstrap");

    start_async_runtime(peripherals.TIMG0, peripherals.SW_INTERRUPT);
    info!("boot: Embassy runtime initialized");

    // I2C for OLED display: GPIO19=SDA, GPIO20=SCL, 400 kHz.
    info!("boot: initializing I2C display bus");
    let i2c = match initialize_display_bus(peripherals.I2C0, peripherals.GPIO19, peripherals.GPIO20)
    {
        Ok(i2c) => Some(i2c),
        Err(error) => {
            debug_assert_eq!(error.action(), BootAction::DegradedMode);
            log_degraded_boot_error(error);
            None
        }
    };

    spawn_critical(
        &spawner,
        display_task_wrapper(i2c, DISPLAY_CHANNEL.receiver(), BOOT_READY.sender()),
        CriticalTask::Display,
    )?;
    info!("boot: display task spawned");

    // Initialize system status signal first to track startup progression.
    SYSTEM_STATUS.send(SystemStatusEvent::BootStarted).await;
    NET_STATUS.send(SystemStatusEvent::BootStarted).await;

    let mut resume_btn = new_button_input(peripherals.GPIO21);
    let boot_button_override = wait_for_boot_provisioning_override(&mut resume_btn).await;
    let mut flash = FlashStorage::new(peripherals.FLASH);
    let partition_table_buffer = WIFI_PARTITION_TABLE_BUFFER.init([0; PARTITION_TABLE_MAX_LEN]);

    // Single decision path: button override, persistent flag and stored
    // credentials all flow through `decide_provisioning`, which also clears
    // the persistent flag once it has been observed.
    let (wifi_decision, wifi_credentials) = {
        let mut store = open_wifi_config_store(&mut flash, partition_table_buffer)?;
        load_wifi_credentials_and_decision(&mut store, boot_button_override).map_err(|error| {
            BootError::CriticalTaskInit(CriticalTaskInit::WifiConfig(WifiConfigInitError::Store(
                error,
            )))
        })?
    };

    let wifi_credentials = match wifi_decision {
        ProvisioningDecision::StationMode => {
            wifi_credentials.ok_or(BootError::CriticalTaskInit(CriticalTaskInit::WifiConfig(
                WifiConfigInitError::Store(StoreError::MissingCredentials),
            )))?
        }
        ProvisioningDecision::ProvisioningMode(reason) => {
            warn!(
                "boot: WiFi provisioning selected ({:?}); safe setup mode active",
                reason
            );
            warn!("boot: DCC, RailCom, Z21, and track output remain disabled");

            // Setup-mode LED feedback: red blink, same pattern as WiFi connecting.
            spawn_critical(
                &spawner,
                provisioning_led_task(
                    new_led_output(peripherals.GPIO14),
                    new_led_output(peripherals.GPIO15),
                ),
                CriticalTask::ProvisioningLed,
            )?;

            let store = open_wifi_config_store(&mut flash, partition_table_buffer)?;
            return run_provisioning_ap(spawner, peripherals.WIFI, store, DISPLAY_CHANNEL.sender())
                .await
                .map_err(|error| {
                    BootError::CriticalTaskInit(CriticalTaskInit::ProvisioningAp(error))
                });
        }
    };

    verify_boot_packet_encoding()?;

    info!("boot: initializing RMT peripheral on GPIO2");
    initialize_dcc_rmt(peripherals.RMT, peripherals.GPIO2)?;

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

    // Track SLEEP master enable: GPIO18 push-pull, held LOW during init so
    // the track sees no signal until the DCC waveform is already stable.
    // GPIO4 is timer-driven: HIGH outside cutouts and LOW from 26 us through
    // 454 us after the final DCC packet edge.
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    let track_output = TrackOutput::new(peripherals.GPIO18, peripherals.GPIO4, timg1.timer0);
    info!("boot: track SLEEP GPIO18 held LOW; RailCom cutout/run GPIO4 held inactive HIGH");

    spawn_critical(
        &spawner,
        dcc_engine_task_wrapper(receiver, FAULT_CHANNEL.sender(), BOOT_READY.sender()),
        CriticalTask::DccEngine,
    )?;
    info!("boot: DCC engine task spawned");

    spawn_critical(
        &spawner,
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
        &spawner,
        scheduler_task_wrapper(
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

    spawn_critical(&spawner, railcom_diag_task(), CriticalTask::RailcomDiag)?;
    info!("boot: RailCom diagnostics task spawned");

    spawn_critical(
        &spawner,
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
        &spawner,
        pom_cutout_monitor_task(POM_TX_STARTED.sender()),
        CriticalTask::PomCutoutMonitor,
    )?;
    info!("boot: POM actor and cutout monitor tasks spawned");

    // RailCom RX wiring: UART RX on GPIO5. The cutout timer opens/closes
    // ISR-side capture windows and the task below drains UART1 FIFO snapshots.
    // Gate 6 of the 74HC14 has a weak pull-down; bypass by wiring GPIO5 directly
    // to pin 10 (gate 5 output) and invert the UART RX polarity in firmware.
    let railcom_uart_rx = initialize_railcom_receiver(peripherals.UART1, peripherals.GPIO5)?;

    spawn_critical(
        &spawner,
        railcom_isr_capture_task_wrapper(railcom_uart_rx, RAILCOM_RUNTIME_RESULTS.sender()),
        CriticalTask::RailcomIsrCapture,
    )?;
    spawn_critical(
        &spawner,
        railcom_uart_runtime_dispatch_task(
            RAILCOM_RUNTIME_RESULTS.receiver(),
            POM_RAILCOM_RESULTS.sender(),
            scheduler_commands.sender(),
        ),
        CriticalTask::RailcomUartDispatch,
    )?;
    info!("boot: RailCom ISR capture spawned on GPIO5 (POM dispatcher active)");

    spawn_critical(
        &spawner,
        net_task_wrapper(NetTaskWrapperContext {
            spawner,
            wifi: peripherals.WIFI,
            credentials: wifi_credentials,
            scheduler_sender: scheduler_commands.sender(),
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
            },
            failure_sender: BOOT_FAILURE.sender(),
        }),
        CriticalTask::Net,
    )?;
    info!("boot: network task spawned");
    DISPLAY_CHANNEL
        .send(DisplayEvent::BootProgress(BootStep::WifiConnecting))
        .await;

    info!("boot: core runtime spawned, establishing DCC output");

    send_power_on_idle_burst(&sender).await;
    info!("boot: power-on idle burst complete (20 packets)");

    spawn_critical(
        &spawner,
        fault_manager_task(FaultManagerTaskContext {
            receiver: FAULT_CHANNEL.receiver(),
            scheduler_sender: scheduler_commands.sender(),
            status_sender: SYSTEM_STATUS.sender(),
            net_status_sender: NET_STATUS.sender(),
            track_output,
            display_sender: DISPLAY_CHANNEL.sender(),
            state_sender: FAULT_STATE.sender(),
            ready_sender: BOOT_READY.sender(),
        }),
        CriticalTask::FaultManager,
    )?;
    info!("boot: fault manager task spawned");

    spawn_critical(
        &spawner,
        provisioning_request_task(
            flash,
            partition_table_buffer,
            FAULT_CHANNEL.sender(),
            PROVISIONING_REQUESTS.receiver(),
        ),
        CriticalTask::ProvisioningRequest,
    )?;
    info!("boot: provisioning request task spawned");

    // Control buttons: GPIO22=stop (active-low), GPIO21=resume (active-low).
    // Spawned before the reset sequence so button events are never missed.
    let stop_btn = new_button_input(peripherals.GPIO22);
    spawn_critical(
        &spawner,
        stop_button_task(stop_btn, FAULT_CHANNEL.sender(), BOOT_READY.sender()),
        CriticalTask::StopButton,
    )?;
    spawn_critical(
        &spawner,
        resume_button_task(
            resume_btn,
            FAULT_CHANNEL.sender(),
            PROVISIONING_REQUESTS.sender(),
            BOOT_READY.sender(),
        ),
        CriticalTask::ResumeButton,
    )?;
    info!("boot: control button tasks spawned (GPIO22=stop, GPIO21=resume)");

    send_decoder_reset_sequence(&sender).await;
    info!("boot: reset sequence complete (5 packets)");

    // Track short detector: GPIO3 monitors the active-low conditioned
    // overcurrent/fault output from the external detector.
    // Spawned after reset + stabilization so the decoder is quiescent.
    let short_pin = new_short_detect_input(peripherals.GPIO3);
    let fault_state_receiver = FAULT_STATE.receiver().ok_or(BootError::CriticalTaskInit(
        CriticalTaskInit::FaultStateReceiverUnavailable,
    ))?;

    spawn_critical(
        &spawner,
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
