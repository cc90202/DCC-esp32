//! Framework-independent scheduler application core.
//!
//! The core owns authoritative locomotive state and correlated response
//! backpressure. It describes runtime-only discovery actions without knowing
//! about Embassy channels or clocks.

#[cfg(target_arch = "riscv32")]
use crate::cutout::CutoutMode;
#[cfg(any(test, target_arch = "riscv32"))]
use crate::dcc::PomRequestId;
use dcc_protocol::packet::DccPacket;

#[cfg(feature = "dcc-a")]
use super::logon::{LogonEvent, LogonTable};
use super::railcom_policy::PacketClass;
use super::slot_manager::{
    PacketAuthority, ScheduledPacket, SlotManager, allowed_slot_visits_without_function,
};
use super::{
    LocoRequestMessage, LocoRequestResult, LocoResponse, SchedulerCommand, SchedulerRequest,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum CommandOutcome {
    Applied,
    Rejected,
    RestartRailcomDiscovery,
}

#[derive(Debug, Default)]
struct LocoResponseOutbox {
    pending: Option<LocoResponse>,
}

impl LocoResponseOutbox {
    fn is_ready(&self) -> bool {
        self.pending.is_none()
    }

    fn stage(&mut self, response: LocoResponse) {
        debug_assert!(self.pending.is_none());
        self.pending = Some(response);
    }

    fn flush_with(
        &mut self,
        try_send: impl FnOnce(LocoResponse) -> Result<(), LocoResponse>,
    ) -> bool {
        let Some(response) = self.pending.take() else {
            return true;
        };
        if let Err(response) = try_send(response) {
            self.pending = Some(response);
            return false;
        }
        true
    }
}

/// Pure application core used by the Embassy scheduler actor.
pub(super) struct SchedulerCore {
    slots: SlotManager,
    loco_response_outbox: LocoResponseOutbox,
    #[cfg(feature = "dcc-a")]
    logon: LogonTable,
}

impl SchedulerCore {
    #[must_use]
    pub(super) fn new() -> Self {
        Self {
            slots: SlotManager::new(),
            loco_response_outbox: LocoResponseOutbox::default(),
            #[cfg(feature = "dcc-a")]
            logon: LogonTable::new(),
        }
    }

    /// Apply a command or describe the runtime discovery action it requires.
    pub(super) fn handle_command(&mut self, command: SchedulerCommand) -> CommandOutcome {
        match command {
            SchedulerCommand::RestartRailcomDiscovery => CommandOutcome::RestartRailcomDiscovery,
            _ if self.slots.apply_command(command) => CommandOutcome::Applied,
            SchedulerCommand::EmergencyStopAll
            | SchedulerCommand::ProgramOnMain { .. }
            | SchedulerCommand::CloseProgramOnMain { .. }
            | SchedulerCommand::Pause
            | SchedulerCommand::Resume => CommandOutcome::Rejected,
            // The runtime takes logon answers before they get here.
            #[cfg(feature = "dcc-a")]
            SchedulerCommand::LogonAnswer(_) => CommandOutcome::Rejected,
        }
    }

    #[must_use]
    pub(super) fn loco_response_ready(&self) -> bool {
        self.loco_response_outbox.is_ready()
    }

    /// Apply one request against authoritative state and stage its response.
    pub(super) fn handle_loco_request(
        &mut self,
        message: LocoRequestMessage,
        now_ticks: u64,
        accepts: impl Fn(crate::authority::LeasePermit) -> bool,
        accept_epoch: impl FnOnce(crate::authority::LeasePermit) -> bool,
    ) {
        debug_assert!(self.loco_response_outbox.is_ready());
        let response =
            handle_loco_request_message(&mut self.slots, message, now_ticks, accepts, accept_epoch);
        self.loco_response_outbox.stage(response);
    }

    /// Attempt to deliver the staged response, retaining it when the adapter is full.
    pub(super) fn flush_loco_response_with(
        &mut self,
        try_send: impl FnOnce(LocoResponse) -> Result<(), LocoResponse>,
    ) -> bool {
        self.loco_response_outbox.flush_with(try_send)
    }

    #[must_use]
    pub(super) fn slot_count(&self) -> usize {
        self.slots.slot_count()
    }

    /// Build the next classified packet using the current slot-count budget.
    pub(super) fn next_packet(&mut self) -> ScheduledPacket {
        let slot_count = self.slots.slot_count();
        self.slots
            .build_next_packet_classified_with_function_budget(
                allowed_slot_visits_without_function(slot_count.max(1)),
            )
            .map_or(
                ScheduledPacket {
                    packet: DccPacket::Idle,
                    class: PacketClass::Idle,
                    authority: PacketAuthority::None,
                },
                |(packet, class)| ScheduledPacket {
                    packet,
                    class,
                    authority: self.slots.take_dequeued_authority(),
                },
            )
    }

    /// Report the packet that was really handed to the track output.
    #[cfg(target_arch = "riscv32")]
    pub(super) fn packet_emitted(&mut self, packet: DccPacket) {
        self.slots.note_emitted(packet);
    }

    /// Hand the decoder table what came back after a logon command; an
    /// address a locomotive is driven on counts as taken.
    #[cfg(feature = "dcc-a")]
    pub(super) fn logon_answer(
        &mut self,
        answer: super::LogonAnswer,
        now: embassy_time::Instant,
    ) -> Option<LogonEvent> {
        let slots = &self.slots;
        self.logon
            .on_answer(answer, now, |address| slots.has_slot(address))
    }

    /// Give the decoder table its turn: close an answer that never came,
    /// then queue the next select or assign once the last one is gone.
    ///
    /// While emission is paused the track may be without power: a command
    /// already sent stops being awaited and is sent again afterwards, so no
    /// attempt is spent on decoders that cannot answer.
    #[cfg(feature = "dcc-a")]
    pub(super) fn advance_logon(&mut self, now: embassy_time::Instant) -> Option<LogonEvent> {
        if self.slots.is_paused() {
            self.logon.suspend();
            return None;
        }
        let event = self.logon.expire(now);
        if self.slots.railcom_telemetry_is_free() {
            // A queued packet that left the queue without reaching the track
            // was dropped with the rest of the queue.
            self.logon.requeue();
            if let Some(packet) = self.logon.next_packet(now) {
                let queued = self.slots.railcom_telemetry_packet(packet);
                debug_assert!(queued, "the telemetry place was free");
            }
        }
        event
    }

    /// Start the wait for the answer to a logon command now on the track.
    #[cfg(feature = "dcc-a")]
    pub(super) fn logon_packet_emitted(&mut self, packet: DccPacket, now: embassy_time::Instant) {
        self.logon.packet_emitted(packet, now);
    }

    pub(super) fn discard_pom(
        &mut self,
        request_id: PomRequestId,
        permit: crate::authority::LeasePermit,
    ) {
        self.slots.discard_pom(request_id, permit);
    }

    #[cfg(target_arch = "riscv32")]
    pub(super) fn quiesce_for_power_on(&mut self) {
        self.slots.quiesce_for_power_on();
    }

    pub(super) fn authorize_for_emission(
        &mut self,
        scheduled: ScheduledPacket,
        accepts: impl FnOnce(crate::authority::LeasePermit) -> bool,
    ) -> bool {
        let PacketAuthority::Pom { request_id, permit } = scheduled.authority else {
            return true;
        };
        if accepts(permit) {
            return true;
        }
        self.discard_pom(request_id, permit);
        false
    }

    #[must_use]
    #[cfg(target_arch = "riscv32")]
    pub(super) fn pom_context_for_packet(
        &self,
        packet: DccPacket,
    ) -> Option<(PomRequestId, CutoutMode)> {
        self.slots.pom_context_for_packet(packet)
    }
}

pub(super) fn handle_loco_request_message(
    slot_manager: &mut SlotManager,
    message: LocoRequestMessage,
    now_ticks: u64,
    accepts: impl Fn(crate::authority::LeasePermit) -> bool,
    accept_epoch: impl FnOnce(crate::authority::LeasePermit) -> bool,
) -> LocoResponse {
    let result = if message.deadline.is_expired_at(now_ticks) {
        LocoRequestResult::Expired
    } else {
        match message.request {
            SchedulerRequest::ResetForLease { permit } if accepts(permit) => {
                if accept_epoch(permit) {
                    slot_manager.reset_for_lease(permit);
                    LocoRequestResult::LeaseReset(permit.epoch())
                } else {
                    LocoRequestResult::Rejected
                }
            }
            SchedulerRequest::ResetForLease { .. } => LocoRequestResult::Rejected,
            SchedulerRequest::Loco { request, permit } => {
                if !request.requires_lease()
                    || permit
                        .is_some_and(|permit| accepts(permit) && slot_manager.accepts_lease(permit))
                {
                    slot_manager.handle_loco_request(request)
                } else {
                    LocoRequestResult::Rejected
                }
            }
        }
    };
    LocoResponse {
        request_id: message.request_id,
        result,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dcc::{DccAddress, Direction, LogicalSpeed, SpeedFormat};

    fn address() -> DccAddress {
        DccAddress::new_short(3).expect("valid test address")
    }

    fn permit() -> crate::authority::LeasePermit {
        crate::authority::LeasePermit::new(crate::authority::LeaseEpoch::new(1), u64::MAX)
    }

    #[test]
    fn discovery_commands_become_runtime_outcomes_without_touching_slots() {
        let mut core = SchedulerCore::new();

        assert_eq!(
            core.handle_command(SchedulerCommand::RestartRailcomDiscovery),
            CommandOutcome::RestartRailcomDiscovery
        );
        assert_eq!(core.slot_count(), 0);
    }

    #[test]
    fn staged_response_is_retained_until_delivery_succeeds() {
        let mut core = SchedulerCore::new();
        core.handle_loco_request(
            LocoRequestMessage {
                request_id: super::super::LocoRequestId::new(41),
                request: SchedulerRequest::Loco {
                    request: super::super::LocoRequest::GetState { address: address() },
                    permit: Some(permit()),
                },
                deadline: super::super::LocoRequestDeadline::from_ticks(101),
            },
            100,
            |_| true,
            |_| true,
        );

        assert!(!core.flush_loco_response_with(Err));
        assert!(!core.loco_response_ready());

        let mut delivered = None;
        assert!(core.flush_loco_response_with(|response| {
            delivered = Some(response);
            Ok(())
        }));
        assert_eq!(
            delivered,
            Some(LocoResponse {
                request_id: super::super::LocoRequestId::new(41),
                result: LocoRequestResult::NotFound,
            })
        );
        assert!(core.loco_response_ready());
    }

    #[test]
    fn accepted_command_mutates_authoritative_slot_state() {
        let mut core = SchedulerCore::new();
        let permit = permit();
        core.slots.reset_for_lease(permit);
        core.handle_loco_request(
            LocoRequestMessage {
                request_id: super::super::LocoRequestId::new(42),
                request: SchedulerRequest::Loco {
                    request: super::super::LocoRequest::SetSpeed {
                        address: address(),
                        speed: LogicalSpeed::new(4, SpeedFormat::Speed128).unwrap(),
                        direction: Direction::Forward,
                    },
                    permit: Some(permit),
                },
                deadline: super::super::LocoRequestDeadline::from_ticks(101),
            },
            100,
            |_| true,
            |_| true,
        );
        assert_eq!(core.slot_count(), 1);

        assert_eq!(
            core.handle_command(SchedulerCommand::EmergencyStopAll),
            CommandOutcome::Applied
        );
        let scheduled = core.next_packet();
        assert_eq!(scheduled.packet, DccPacket::BroadcastStop);
        assert_eq!(scheduled.class, PacketClass::Safety);
    }

    #[test]
    fn revoked_pom_is_discarded_at_emission_boundary() {
        let mut core = SchedulerCore::new();
        let permit = permit();
        core.slots.reset_for_lease(permit);
        assert_eq!(
            core.handle_command(SchedulerCommand::ProgramOnMain {
                request_id: PomRequestId::new(77),
                permit,
                packet: DccPacket::PomReadByte {
                    address: address(),
                    cv: crate::dcc::PomCv::new(1).expect("valid CV"),
                },
            }),
            CommandOutcome::Applied
        );

        let safety = core.next_packet();
        assert!(core.authorize_for_emission(safety, |_| false));

        let programming = core.next_packet();
        assert_eq!(programming.class, PacketClass::Programming);
        assert!(!core.authorize_for_emission(programming, |_| false));
        assert_ne!(core.next_packet().class, PacketClass::Programming);
    }

    #[test]
    fn discarding_a_pom_drops_only_the_matching_request_and_epoch() {
        let mut core = SchedulerCore::new();
        let permit = permit();
        let other_epoch =
            crate::authority::LeasePermit::new(crate::authority::LeaseEpoch::new(2), u64::MAX);
        let read = DccPacket::PomReadByte {
            address: address(),
            cv: crate::dcc::PomCv::new(1).expect("valid CV"),
        };
        core.slots.reset_for_lease(permit);
        for _ in 0..2 {
            let command = SchedulerCommand::ProgramOnMain {
                request_id: PomRequestId::new(77),
                permit,
                packet: read,
            };
            assert_eq!(core.handle_command(command), CommandOutcome::Applied);
        }
        assert_eq!(core.next_packet().class, PacketClass::Safety);

        core.discard_pom(PomRequestId::new(78), permit);
        core.discard_pom(PomRequestId::new(77), other_epoch);
        assert!(core.slots.pom_context_for_packet(read).is_some());
        assert_eq!(core.next_packet().class, PacketClass::Programming);

        core.discard_pom(PomRequestId::new(77), permit);
        assert_eq!(core.slots.pom_context_for_packet(read), None);
        assert_ne!(core.next_packet().class, PacketClass::Programming);
    }

    #[cfg(feature = "dcc-a")]
    mod logon {
        use embassy_time::Instant;

        use super::*;
        use crate::dcc::{DecoderUid, LogonAnswer};
        use dcc_protocol::packet::LogonCommand;

        const NOW: Instant = Instant::from_secs(10);
        const UID: DecoderUid = DecoderUid {
            manufacturer_id: 145,
            decoder_id: 7,
        };
        const SELECT: DccPacket = DccPacket::LogonSelect {
            manufacturer_id: 145,
            decoder_id: 7,
            subcommand: 0xff,
        };

        fn answer(core: &mut SchedulerCore, answer: LogonAnswer) -> Option<LogonEvent> {
            core.logon_answer(answer, NOW)
        }

        /// Takes the next packet and puts it on the track.
        fn emit(core: &mut SchedulerCore) -> ScheduledPacket {
            core.advance_logon(NOW);
            let scheduled = core.next_packet();
            core.logon_packet_emitted(scheduled.packet, NOW);
            scheduled
        }

        fn drive(core: &mut SchedulerCore, address: DccAddress) {
            assert!(core.slots.set_speed(
                address,
                LogicalSpeed::zero(SpeedFormat::Speed128),
                Direction::Forward
            ));
        }

        #[test]
        fn an_announced_decoder_is_selected_with_a_telemetry_cutout() {
            let mut core = SchedulerCore::new();
            assert_eq!(answer(&mut core, LogonAnswer::Announced(UID)), None);

            let scheduled = emit(&mut core);

            assert_eq!(scheduled.packet, SELECT);
            assert_eq!(scheduled.class, PacketClass::Telemetry);
            assert_eq!(emit(&mut core).packet, DccPacket::Idle);
        }

        #[test]
        fn a_decoder_asking_for_a_driven_address_gets_a_temporary_one() {
            let mut core = SchedulerCore::new();
            drive(&mut core, address());
            answer(&mut core, LogonAnswer::Announced(UID));
            while emit(&mut core).packet != SELECT {}

            answer(&mut core, LogonAnswer::WantsAddress(address()));
            let assigned = loop {
                if let DccPacket::LogonAssign { address, .. } = emit(&mut core).packet {
                    break address;
                }
            };

            assert_eq!(assigned, DccAddress::new_long(9_999).unwrap());
            assert_eq!(
                answer(&mut core, LogonAnswer::Confirmed),
                Some(LogonEvent::Registered {
                    uid: UID,
                    address: assigned,
                    wanted: address(),
                })
            );
        }

        #[test]
        fn no_logon_attempt_is_spent_while_emission_is_paused() {
            let mut core = SchedulerCore::new();
            answer(&mut core, LogonAnswer::Announced(UID));
            let long_after = NOW + embassy_time::Duration::from_secs(60);

            // More rounds than a decoder has attempts.
            for _ in 0..10 {
                assert_eq!(emit(&mut core).packet, SELECT);
                assert!(core.handle_command(SchedulerCommand::Pause) == CommandOutcome::Applied);
                assert_eq!(core.advance_logon(long_after), None);
                assert!(core.handle_command(SchedulerCommand::Resume) == CommandOutcome::Applied);
                assert_eq!(core.advance_logon(long_after), None);
            }

            assert_eq!(emit(&mut core).packet, SELECT);
        }

        #[test]
        fn an_answer_lost_on_a_powered_track_spends_the_attempts() {
            let mut core = SchedulerCore::new();
            answer(&mut core, LogonAnswer::Announced(UID));
            let long_after = NOW + embassy_time::Duration::from_secs(60);

            assert_eq!(emit(&mut core).packet, SELECT);
            assert_eq!(core.advance_logon(long_after), None);
            assert_eq!(emit(&mut core).packet, SELECT);
            assert_eq!(core.advance_logon(long_after), None);
            assert_eq!(emit(&mut core).packet, SELECT);

            assert_eq!(
                core.advance_logon(long_after),
                Some(LogonEvent::Failed {
                    uid: UID,
                    command: LogonCommand::Select,
                })
            );
        }

        #[test]
        fn a_select_dropped_with_the_queue_goes_out_afterwards() {
            let mut core = SchedulerCore::new();
            answer(&mut core, LogonAnswer::Announced(UID));
            core.advance_logon(NOW);

            core.slots.reset_for_lease(permit());

            assert!(
                (0..4).any(|_| emit(&mut core).packet == SELECT),
                "the dropped select never went out"
            );
        }
    }
}
