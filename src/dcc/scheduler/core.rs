//! Framework-independent scheduler application core.
//!
//! The core owns authoritative locomotive state and correlated response
//! backpressure. It describes runtime-only discovery actions without knowing
//! about Embassy channels or clocks.

#[cfg(target_arch = "riscv32")]
use crate::cutout::CutoutMode;
#[cfg(target_arch = "riscv32")]
use crate::dcc::PomRequestId;
use crate::dcc::packet::DccPacket;

use super::railcom_policy::PacketClass;
use super::slot_manager::{SlotManager, allowed_slot_visits_without_function};
use super::{LocoRequestMessage, LocoRequestResult, LocoResponse, SchedulerCommand};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum CommandOutcome {
    Applied,
    Rejected,
    SuspendRailcomDiscovery,
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
}

impl SchedulerCore {
    #[must_use]
    pub(super) fn new() -> Self {
        Self {
            slots: SlotManager::new(),
            loco_response_outbox: LocoResponseOutbox::default(),
        }
    }

    /// Apply a command or describe the runtime discovery action it requires.
    pub(super) fn handle_command(&mut self, command: SchedulerCommand) -> CommandOutcome {
        match command {
            SchedulerCommand::SuspendRailcomDiscovery => CommandOutcome::SuspendRailcomDiscovery,
            SchedulerCommand::RestartRailcomDiscovery => CommandOutcome::RestartRailcomDiscovery,
            _ if self.slots.apply_command(command) => CommandOutcome::Applied,
            _ => CommandOutcome::Rejected,
        }
    }

    #[must_use]
    pub(super) fn loco_response_ready(&self) -> bool {
        self.loco_response_outbox.is_ready()
    }

    /// Apply one request against authoritative state and stage its response.
    pub(super) fn handle_loco_request(&mut self, message: LocoRequestMessage, now_ticks: u64) {
        debug_assert!(self.loco_response_outbox.is_ready());
        let response = handle_loco_request_message(&mut self.slots, message, now_ticks);
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
    pub(super) fn next_packet(&mut self) -> (DccPacket, PacketClass) {
        let slot_count = self.slots.slot_count();
        self.slots
            .build_next_packet_classified_with_function_budget(
                allowed_slot_visits_without_function(slot_count.max(1)),
            )
            .unwrap_or((DccPacket::Idle, PacketClass::Idle))
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
) -> LocoResponse {
    let result = if message.deadline.is_expired_at(now_ticks) {
        LocoRequestResult::Expired
    } else {
        slot_manager.handle_loco_request(message.request)
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

    #[test]
    fn discovery_commands_become_runtime_outcomes_without_touching_slots() {
        let mut core = SchedulerCore::new();

        assert_eq!(
            core.handle_command(SchedulerCommand::SuspendRailcomDiscovery),
            CommandOutcome::SuspendRailcomDiscovery
        );
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
                request: super::super::LocoRequest::GetState { address: address() },
                deadline: super::super::LocoRequestDeadline::from_ticks(101),
            },
            100,
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
        core.handle_loco_request(
            LocoRequestMessage {
                request_id: super::super::LocoRequestId::new(42),
                request: super::super::LocoRequest::SetSpeed {
                    address: address(),
                    speed: LogicalSpeed::new(4, SpeedFormat::Speed128).unwrap(),
                    direction: Direction::Forward,
                },
                deadline: super::super::LocoRequestDeadline::from_ticks(101),
            },
            100,
        );
        assert_eq!(core.slot_count(), 1);

        assert_eq!(
            core.handle_command(SchedulerCommand::EmergencyStopAll),
            CommandOutcome::Applied
        );
        let (packet, class) = core.next_packet();
        assert_eq!(packet, DccPacket::BroadcastStop);
        assert_eq!(class, PacketClass::Safety);
    }
}
