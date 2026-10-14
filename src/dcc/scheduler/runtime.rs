//! Embassy adapter for the pure scheduler application core.

use embassy_futures::yield_now;
use embassy_time::Instant;

use crate::cutout::CutoutMode;
use crate::dcc::DccFrame;
use crate::dcc::channels::{
    DccFrameSender, LocoRequestReceiver, LocoResponseSender, PowerQuiesceReceiver,
    SchedulerCommandReceiver,
};
use crate::system_status::DisplayEvent;
use crate::system_status::channels::DisplaySender;

use super::SchedulerCommand;
use super::core::{CommandOutcome, SchedulerCore};
use super::railcom_discovery::RailcomDiscovery;
use super::railcom_policy::{PacketClass, RailcomCutoutBudget, record_track_search_throttled};
use super::slot_manager::{PacketAuthority, pom_cutout, scheduler_invariant_recovery_count};

pub async fn packet_scheduler_task(
    power_quiesce_receiver: PowerQuiesceReceiver,
    command_receiver: SchedulerCommandReceiver,
    loco_request_receiver: LocoRequestReceiver,
    loco_response_sender: LocoResponseSender,
    sender: DccFrameSender,
    display_sender: DisplaySender,
) -> ! {
    let mut core = SchedulerCore::new();
    let mut prev_slot_count: usize = 0;
    let mut railcom_budget = RailcomCutoutBudget::new();
    let mut discovery = RailcomDiscovery::new(Instant::now());
    let mut observed_invariant_recoveries = scheduler_invariant_recovery_count();

    loop {
        if let Ok(generation) = power_quiesce_receiver.try_receive() {
            core.quiesce_for_power_on();
            sender.send(DccFrame::fence(generation)).await;
        }

        // Commands are processed with at most one packet interval (~8 ms) of latency.
        while let Ok(command) = command_receiver.try_receive() {
            if let SchedulerCommand::ProgramOnMain { permit, .. } = command
                && !crate::track_authority::accepts_current(permit)
            {
                defmt::warn!("scheduler rejected expired POM lease");
                continue;
            }
            match core.handle_command(command) {
                CommandOutcome::Applied => {}
                CommandOutcome::Rejected => defmt::warn!("scheduler command rejected"),
                CommandOutcome::SuspendRailcomDiscovery => {
                    discovery.suspend(Instant::now());
                }
                CommandOutcome::RestartRailcomDiscovery => {
                    discovery.restart(Instant::now());
                    defmt::info!("railcom discovery: restarted after track enable");
                }
            }
        }

        core.flush_loco_response_with(|response| {
            loco_response_sender
                .try_send(response)
                .map_err(|embassy_sync::channel::TrySendError::Full(response)| response)
        });

        while core.loco_response_ready()
            && let Ok(message) = loco_request_receiver.try_receive()
        {
            core.handle_loco_request(
                message,
                Instant::now().as_ticks(),
                crate::track_authority::accepts_current,
                crate::track_authority::accept_epoch,
            );
            if !core.flush_loco_response_with(|response| {
                loco_response_sender
                    .try_send(response)
                    .map_err(|embassy_sync::channel::TrySendError::Full(response)| response)
            }) {
                defmt::warn!(
                    "loco response queue full; retaining request id={}",
                    message.request_id.value()
                );
            }
        }

        let slot_count = core.slot_count();
        if slot_count != prev_slot_count {
            prev_slot_count = slot_count;
            if display_sender
                .try_send(DisplayEvent::ActiveLocoCount(slot_count as u8))
                .is_err()
            {
                defmt::warn!("scheduler: active loco display event dropped");
            }
        }

        let scheduled = core.next_packet();
        let mut packet = scheduled.packet;
        let class = scheduled.class;
        let invariant_recoveries = scheduler_invariant_recovery_count();
        if invariant_recoveries != observed_invariant_recoveries {
            defmt::error!(
                "scheduler: recovered invalid slot refresh state; count={}",
                invariant_recoveries
            );
            observed_invariant_recoveries = invariant_recoveries;
        }
        let now = Instant::now();
        let idle_discovery_due = matches!(class, PacketClass::Idle) && discovery.is_due(now);
        let cutout_requested = !matches!(class, PacketClass::Idle) || idle_discovery_due;
        let cutout_allowed = if cutout_requested {
            railcom_budget.allow_cutout_for(class)
        } else {
            railcom_budget.note_packet_without_cutout();
            false
        };

        if idle_discovery_due {
            if cutout_allowed {
                packet = discovery.take_next_idle_packet(now);
            } else {
                record_track_search_throttled();
            }
        }

        let cutout = match (cutout_allowed, class, packet) {
            (true, PacketClass::Programming, packet) => {
                pom_cutout(packet).unwrap_or(CutoutMode::PomWrite)
            }
            (true, _, _) => CutoutMode::Telemetry,
            (false, _, _) => CutoutMode::None,
        };
        if !core.authorize_for_emission(scheduled, crate::track_authority::accepts) {
            defmt::warn!("scheduler: discarded stale POM before emission");
            continue;
        }
        let pom_context = cutout_allowed
            .then(|| core.pom_context_for_packet(packet))
            .flatten();
        let cutout = pom_context.map_or(cutout, |(_, active_cutout)| active_cutout);
        let mut frame = DccFrame::new(packet, cutout);
        if let Some((request_id, _)) = pom_context {
            frame = frame.with_pom_request_id(request_id);
        }

        // POM authority is rechecked on every retry while waiting for engine capacity.
        let emitted = match scheduled.authority {
            PacketAuthority::None => {
                sender.send(frame).await;
                true
            }
            PacketAuthority::Pom { request_id, permit } => loop {
                if !crate::track_authority::accepts(permit) {
                    core.discard_pom(request_id, permit);
                    break false;
                }
                match sender.try_send(frame) {
                    Ok(()) => break true,
                    Err(embassy_sync::channel::TrySendError::Full(_)) => yield_now().await,
                }
            },
        };
        if !emitted {
            defmt::warn!("scheduler: POM authority expired while awaiting engine capacity");
            continue;
        }
        crate::liveness::beat(crate::liveness::Task::Scheduler);
        yield_now().await;
    }
}
