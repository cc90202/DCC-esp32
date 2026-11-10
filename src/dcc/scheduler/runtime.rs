//! Embassy adapter for the pure scheduler application core.

use embassy_futures::yield_now;
use embassy_time::Instant;

use super::channels::{LocoRequestReceiver, LocoResponseSender, SchedulerCommandReceiver};
use crate::cutout::CutoutMode;
use crate::dcc::DccFrame;
use crate::dcc::channels::{DccFrameSender, PowerQuiesceReceiver};
use crate::macros::bench_diag;
use crate::system_status::DisplayEvent;
#[cfg(feature = "dcc-a")]
use crate::system_status::LogonStatus;
use crate::system_status::channels::DisplaySender;

use super::SchedulerCommand;
use super::core::{CommandOutcome, SchedulerCore};
#[cfg(feature = "dcc-a")]
use super::logon::LogonEvent;
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
    #[cfg(not(feature = "dcc-a"))]
    let mut prev_slot_count: usize = 0;
    let mut railcom_budget = RailcomCutoutBudget::new();
    let mut discovery = RailcomDiscovery::new(Instant::now());
    let mut observed_invariant_recoveries = scheduler_invariant_recovery_count();
    #[cfg(feature = "dcc-a")]
    let mut logon_display = LogonDisplay::new(Instant::now());
    #[cfg(feature = "dcc-a")]
    set_logon_identity(&mut core);

    loop {
        if let Ok(generation) = power_quiesce_receiver.try_receive() {
            core.quiesce_for_power_on();
            sender.send(DccFrame::fence(generation)).await;
        }

        // Commands are picked up within one packet interval (~8 ms), and the
        // frame they produce waits behind at most one queued frame.
        while let Ok(command) = command_receiver.try_receive() {
            apply_command(
                &mut core,
                &mut discovery,
                #[cfg(feature = "dcc-a")]
                display_sender,
                command,
            );
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

        #[cfg(not(feature = "dcc-a"))]
        report_slot_count(&core, display_sender, &mut prev_slot_count);
        #[cfg(feature = "dcc-a")]
        logon_display.turn(&mut core, display_sender, Instant::now());

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
        bench_diag! {
            frame = frame.with_picked_at(now);
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
        core.packet_emitted(packet);
        #[cfg(feature = "dcc-a")]
        core.logon_packet_emitted(packet, Instant::now());
        crate::liveness::beat(crate::liveness::Task::Scheduler);
        yield_now().await;
    }
}

/// Tells the display how many locomotives the apps drive, when it changed.
#[cfg(not(feature = "dcc-a"))]
fn report_slot_count(core: &SchedulerCore, display: DisplaySender, shown: &mut usize) {
    let slot_count = core.slot_count();
    if slot_count != *shown {
        *shown = slot_count;
        if display
            .try_send(DisplayEvent::ActiveLocoCount(slot_count as u8))
            .is_err()
        {
            defmt::warn!("scheduler: active loco display event dropped");
        }
    }
}

/// Tells the display about DCC-A: the locomotives in use and the warnings.
#[cfg(feature = "dcc-a")]
struct LogonDisplay {
    /// The picture the display has.
    shown: LogonStatus,
    next_look: Instant,
}

#[cfg(feature = "dcc-a")]
impl LogonDisplay {
    /// How often the picture is compared with what the display has. It
    /// changes a few times per track power-on and when an app picks a
    /// locomotive.
    const LOOK_INTERVAL: embassy_time::Duration = embassy_time::Duration::from_millis(500);

    fn new(now: Instant) -> Self {
        Self {
            shown: LogonStatus::default(),
            next_look: now,
        }
    }

    /// Gives the decoder table its turn and passes on what changed.
    fn turn(&mut self, core: &mut SchedulerCore, display: DisplaySender, now: Instant) {
        report_logon_event(core.advance_logon(now), display);
        if now < self.next_look {
            return;
        }
        self.next_look = now + Self::LOOK_INTERVAL;
        let status = core.logon_status();
        // A picture that finds the queue full is sent at the next look.
        if status != self.shown
            && display
                .try_send(DisplayEvent::LogonStatus(status.clone()))
                .is_ok()
        {
            self.shown = status;
        }
    }
}

/// Logs a logon change and shows a decoder that logs on late.
///
/// Logon changes are rare, one or two per decoder at each track power-on.
#[cfg(feature = "dcc-a")]
fn report_logon_event(event: Option<LogonEvent>, display: DisplaySender) {
    match event {
        None => {}
        Some(LogonEvent::Registered {
            uid,
            address,
            wanted,
            late,
        }) => {
            if address == wanted {
                defmt::info!(
                    "logon: decoder {} registered on address {}",
                    uid,
                    address.value(),
                );
            } else {
                defmt::warn!(
                    "logon: address {} wanted by decoder {} is taken, assigned {}",
                    wanted.value(),
                    uid,
                    address.value(),
                );
            }
            if late {
                // Missing it costs a few seconds of notice; the address is
                // in the status picture either way.
                let _ = display.try_send(DisplayEvent::NewLoco(address.value()));
            }
        }
        Some(LogonEvent::Failed { uid, command }) => {
            defmt::warn!("logon: decoder {} gave no answer to {}", uid, command);
        }
        Some(LogonEvent::TableFull(uid)) => {
            defmt::warn!("logon: no room in the table for decoder {}", uid);
        }
    }
}

/// Gives the decoder table the station id, taken from the board's MAC
/// address, and the session to start from.
#[cfg(feature = "dcc-a")]
fn set_logon_identity(core: &mut SchedulerCore) {
    // The session only has to differ from the one of the last boot; with no
    // memory of it, any value is as good as another. The radio that feeds
    // the generator may not run yet, so the time this task started at is
    // mixed in.
    let [random, ..] = esp_hal::rng::Rng::new().random().to_le_bytes();
    let [uptime, ..] = Instant::now().as_ticks().to_le_bytes();
    core.set_logon_identity(
        super::logon::station_id_from(esp_hal::efuse::Efuse::mac_address()),
        random ^ uptime,
    );
}

fn apply_command(
    core: &mut SchedulerCore,
    discovery: &mut RailcomDiscovery,
    #[cfg(feature = "dcc-a")] display: DisplaySender,
    command: SchedulerCommand,
) {
    if let SchedulerCommand::ProgramOnMain { permit, .. } = command
        && !crate::track_authority::accepts_current(permit)
    {
        defmt::warn!("scheduler rejected expired POM lease");
        return;
    }
    #[cfg(feature = "dcc-a")]
    if let SchedulerCommand::LogonAnswer(answer) = command {
        report_logon_event(core.logon_answer(answer, Instant::now()), display);
        return;
    }
    match core.handle_command(command) {
        CommandOutcome::Applied => {}
        CommandOutcome::Rejected => defmt::warn!("scheduler command rejected"),
        CommandOutcome::RestartRailcomDiscovery => {
            discovery.restart(Instant::now());
            #[cfg(feature = "dcc-a")]
            core.track_powered(Instant::now());
            defmt::info!("railcom discovery: restarted after track enable");
        }
    }
}
