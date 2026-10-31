//! Thin Embassy task adapters used by the boot composition root.

use embassy_executor::Spawner;
use esp_hal::uart::UartRx;

// Used only by the bench diagnostics dump task below.
#[cfg(feature = "bench-diag")]
use {
    crate::railcom::RailcomChannel,
    defmt::info,
    embassy_time::{Duration, Timer},
};

use super::error::CriticalTaskInitSender;
use crate::dcc::channels::{
    DccFrameReceiver, DccFrameSender, PowerFenceAckSender, PowerQuiesceReceiver,
};
use crate::dcc::packet_scheduler_task;
use crate::dcc::scheduler::channels::{
    LocoRequestReceiver, LocoResponseSender, SchedulerCommandReceiver,
};
use crate::dcc_runtime::dcc_engine_task;
use crate::net::udp_control::{NetTaskChannels, net_task};
use crate::net::wifi_config::WifiCredentials;
use crate::railcom::uart_reader::RailcomRxOutputSender;
use crate::system_status::channels::{
    BootReadySender, DisplayReceiver, DisplaySender, FaultEventSender,
};
use crate::system_status::{BootReadyEvent, announce_ready};

use super::CriticalTaskInit;

pub(super) struct NetTaskWrapperContext {
    pub(super) spawner: Spawner,
    pub(super) wifi: esp_hal::peripherals::WIFI<'static>,
    pub(super) credentials: WifiCredentials,
    pub(super) fault_sender: FaultEventSender,
    pub(super) channels: NetTaskChannels,
    pub(super) failure_sender: CriticalTaskInitSender,
}

#[embassy_executor::task]
pub(super) async fn dcc_engine_task_wrapper(
    receiver: DccFrameReceiver,
    fault_sender: FaultEventSender,
    fence_ack_sender: PowerFenceAckSender,
    ready_sender: BootReadySender,
) -> ! {
    announce_ready(ready_sender, BootReadyEvent::DccEngine).await;
    dcc_engine_task(receiver, fault_sender, fence_ack_sender).await
}

#[embassy_executor::task]
pub(super) async fn net_task_wrapper(context: NetTaskWrapperContext) {
    let NetTaskWrapperContext {
        spawner,
        wifi,
        credentials,
        fault_sender,
        channels,
        failure_sender,
    } = context;

    let runtime_fault_sender = fault_sender;
    match net_task(spawner, wifi, credentials, fault_sender, channels).await {
        Ok(()) => {
            crate::track_safety::disable_track_intentionally();
            defmt::error!("boot: network task exited unexpectedly; track output disabled");
            if runtime_fault_sender
                .try_send(crate::station_status::FaultEvent::FaultLatched(
                    crate::station_status::FaultCause::Internal,
                ))
                .is_err()
            {
                defmt::error!("boot: failed to report unexpected network task exit");
            }
        }
        Err(error) => {
            defmt::error!(
                "boot: network init failed: {}",
                defmt::Display2Format(&error)
            );
            failure_sender.send(CriticalTaskInit::Net(error)).await;
        }
    }
}

#[embassy_executor::task]
pub(super) async fn display_task_wrapper(
    i2c: Option<esp_hal::i2c::master::I2c<'static, esp_hal::Async>>,
    receiver: DisplayReceiver,
    ready_sender: BootReadySender,
) -> ! {
    crate::display::display_task(i2c, receiver, ready_sender).await
}

#[embassy_executor::task]
pub(super) async fn scheduler_task_wrapper(
    power_quiesce_receiver: PowerQuiesceReceiver,
    command_receiver: SchedulerCommandReceiver,
    loco_request_receiver: LocoRequestReceiver,
    loco_response_sender: LocoResponseSender,
    sender: DccFrameSender,
    display_sender: DisplaySender,
    ready_sender: BootReadySender,
) -> ! {
    announce_ready(ready_sender, BootReadyEvent::Scheduler).await;
    packet_scheduler_task(
        power_quiesce_receiver,
        command_receiver,
        loco_request_receiver,
        loco_response_sender,
        sender,
        display_sender,
    )
    .await
}

#[embassy_executor::task]
pub(super) async fn railcom_isr_capture_task_wrapper(
    uart_rx: UartRx<'static, esp_hal::Async>,
    result_sender: RailcomRxOutputSender,
) -> ! {
    crate::railcom_capture::railcom_isr_capture_task(uart_rx, result_sender).await
}

/// Snapshot of every RailCom-related counter the bench dump reports.
///
/// Assembled here rather than in `railcom`: each module publishes its own
/// counters, and putting a view that spans several of them inside one of them
/// made `railcom` depend on `track_output` for bookkeeping alone, which closed
/// a cycle through the capture ISR.
///
/// Gated with the task below, because it exists only to feed it.
#[cfg(feature = "bench-diag")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(super) struct RailcomDiagnostics {
    pub boundary: crate::railcom::RailcomStats,
    pub scheduler: crate::dcc::scheduler::RailcomSchedulerStats,
    pub track_output: crate::track_output::TrackOutputStats,
    pub rx: crate::railcom::pipeline::RailcomRxStats,
    /// Per-channel receive counters, ordered as channel 1 followed by channel 2.
    pub rx_channels: [crate::railcom::pipeline::RailcomChannelStats;
        crate::railcom::pipeline::RAILCOM_CHANNEL_COUNT],
    pub loco: crate::railcom::loco_tracker::RailcomLocoTrackerStats,
}

#[cfg(feature = "bench-diag")]
#[must_use]
fn railcom_diagnostics() -> RailcomDiagnostics {
    RailcomDiagnostics {
        boundary: crate::railcom::stats(),
        scheduler: crate::dcc::scheduler::railcom_scheduler_stats(),
        track_output: crate::track_output::stats(),
        rx: crate::railcom::pipeline::railcom_rx_stats(),
        rx_channels: crate::railcom::pipeline::railcom_rx_channel_stats(),
        loco: crate::railcom::loco_tracker::railcom_loco_tracker_stats(),
    }
}

/// Bench diagnostic: periodic dump of the RailCom, POM and network counters.
///
/// Compiled out unless `bench-diag` is enabled. The line carries ~50 fields,
/// far more than the 64-byte USB-serial FIFO, and defmt writes the whole frame
/// inside one critical section: when the FIFO fills, the writer spins with
/// interrupts disabled until the host drains it. The counters themselves are
/// always maintained; only this dump is gated.
#[cfg(feature = "bench-diag")]
#[embassy_executor::task]
pub(super) async fn railcom_diag_task() -> ! {
    loop {
        Timer::after(Duration::from_secs(10)).await;
        let diag = railcom_diagnostics();
        let rmt_timing = crate::rmt_dcc::timing_stats();
        let pom = crate::dcc::pom_runtime_stats();
        let railcom_getdata_no_data = crate::net::railcom_getdata_no_data_count();
        let loco_response_timeout = crate::net::loco_response_timeout_count();
        let loco_command_rejected = crate::net::loco_command_rejected_count();
        let status_broadcast_send_failure = crate::net::status_broadcast_send_failure_count();
        let udp_receive_failure = crate::net::udp_receive_failure_count();
        let lease_round_trips = crate::net::lease_round_trip_count();
        let uart_raw = crate::railcom_capture::uart1_raw_diag();
        info!("railcom uart1 raw: {:?}", uart_raw);

        info!(
            "railcom diag: boundary={} rmt_isr_max_us={} rmt_cutout_request_max_us={} cutout_grant={} cutout_pom={} logon_sent={} search_sent={} search_throttled={} skip_budget={} skip_priority={} request={} skip_disabled={} started={} ended={} schedule_fail={} stale_recovery={} invalid_state={} deadline_late_max_us={} evt_drop={} evt_notify_fail={} rx_windows={} rx_empty={} rx_bytes={} rx_ok={} rx_err={} rx_oversized={} rx_overflow={} pom_forwarded={} pom_dropped={} pom_tx_timeout={} pom_response_timeout={} pom_stale={} pom_wrong_target={} getdata_no_data={} loco_response_timeout={} loco_command_rejected={} status_broadcast_send_failure={} udp_receive_failure={} ch1_win={} ch1_empty={} ch2_win={} ch2_empty={} ack={} nack={} busy={} adr_high={} adr_low={} loco_id_windows={} loco_id_ok={} loco_id_invalid={} loco_id_unrepresentable={} lease_round_trips={}",
            diag.boundary.packet_boundary_count,
            rmt_timing.max_isr_duration_us,
            rmt_timing.max_cutout_request_latency_us,
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
            diag.track_output.cutout_stale_recovery_count,
            diag.track_output.cutout_invalid_state_count,
            diag.track_output.cutout_max_deadline_lateness_us,
            diag.track_output.cutout_event_dropped_count,
            diag.track_output.cutout_event_notify_fail_count,
            diag.rx.rx_window_count(),
            diag.rx.rx_empty_window_count,
            diag.rx.rx_windows_with_bytes_count(),
            diag.rx.rx_parse_ok_count,
            diag.rx.rx_parse_err_count,
            diag.rx.rx_oversized_window_count,
            diag.rx.rx_overflow_count,
            diag.rx.pom_result_forwarded_count,
            diag.rx.pom_result_dropped_count,
            pom.tx_start_timeout_count,
            pom.response_timeout_count,
            pom.stale_result_count,
            pom.wrong_target_result_count,
            railcom_getdata_no_data,
            loco_response_timeout,
            loco_command_rejected,
            status_broadcast_send_failure,
            udp_receive_failure,
            diag.rx_channels[RailcomChannel::Channel1.index()].window_count,
            diag.rx_channels[RailcomChannel::Channel1.index()].empty_count,
            diag.rx_channels[RailcomChannel::Channel2.index()].window_count,
            diag.rx_channels[RailcomChannel::Channel2.index()].empty_count,
            diag.rx.rx_ack_count,
            diag.rx.rx_nack_count,
            diag.rx.rx_busy_count,
            diag.rx.rx_adr_high_count,
            diag.rx.rx_adr_low_count,
            diag.loco.identification_window_count,
            diag.loco.identified_loco_count,
            diag.loco.invalid_identification_count,
            diag.loco.unrepresentable_address_count,
            lease_round_trips,
        );
    }
}
