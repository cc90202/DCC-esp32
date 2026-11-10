//! Packet selection and emission-priority policy for scheduler slots.

use dcc_protocol::packet::{DccPacket, Direction};

#[cfg(test)]
use super::allowed_slot_visits_without_function;
use super::{
    DirtyFunctionSelection, PacketAuthority, PacketClass, RefreshPlan,
    STOPPED_REFRESH_PERIOD_TURNS, Slot, SlotManager,
};

/// Function-refresh budget, in visits, of a stopped slot served beside moving
/// ones: one function group and one zero speed per pair of visits.
const STOPPED_SLOT_FUNCTION_BUDGET: u8 = 2;

/// Whether `next` after `previous` would be two different packets to one
/// decoder address back to back.
pub(in crate::dcc::scheduler) fn same_address_different_packet(
    previous: DccPacket,
    next: DccPacket,
) -> bool {
    previous
        .railcom_target_address()
        .is_some_and(|address| next.railcom_target_address() == Some(address))
        && previous != next
}

/// Which slots a refresh scan may serve.
#[derive(Clone, Copy, PartialEq, Eq)]
enum RefreshTier {
    /// Moving slots, POM refresh slots and stopped slots whose period elapsed.
    Due,
    /// Stopped slots not yet due, used only when no due slot can be served.
    Filler,
}

impl SlotManager {
    /// Build the next packet to transmit.
    #[cfg(test)]
    pub(crate) fn build_next_packet(&mut self) -> Option<DccPacket> {
        self.build_next_packet_with_function_budget(allowed_slot_visits_without_function(
            self.slots.len(),
        ))
    }

    /// Build next packet with an explicit function-refresh budget in slot visits.
    #[cfg(test)]
    pub(crate) fn build_next_packet_with_function_budget(
        &mut self,
        max_slot_visits_without_function: u8,
    ) -> Option<DccPacket> {
        let packet = self
            .build_next_packet_classified_with_function_budget(max_slot_visits_without_function)
            .map(|(packet, _)| packet);
        // The runtime sends an idle on an empty turn and reports what it sent.
        self.note_emitted(packet.unwrap_or(DccPacket::Idle));
        packet
    }

    fn take_pending_safety_packet(&mut self) -> Option<DccPacket> {
        if self.pending_broadcast_estop {
            self.pending_broadcast_estop = false;
            return Some(DccPacket::BroadcastStop);
        }

        let address = self.pending_estop_targets.pop_front()?;
        let direction = self
            .slot_index(address)
            .map_or(Direction::Reverse, |index| self.slots[index].direction);
        Some(DccPacket::EmergencyStop { address, direction })
    }

    /// Slot indices starting at `start` (normalised into range) and wrapping once.
    fn scan_order_from(&self, start: usize) -> impl Iterator<Item = usize> + use<> {
        let len = self.slots.len();
        let start = if start < len { start } else { 0 };
        (0..len).map(move |offset| (start + offset) % len)
    }

    /// Whether sending `packet` now would put two different packets to one
    /// address back to back, which NMRA S-9.2 and RCN-211 forbid (the decoder
    /// needs 5 ms between them). Identical repeats are allowed.
    fn blocks(&self, packet: DccPacket) -> bool {
        self.last_packet
            .is_some_and(|last| same_address_different_packet(last, packet))
    }

    /// Record the packet that really went out on the track, idle included;
    /// the spacing rule on the next turn is measured from it.
    pub(crate) fn note_emitted(&mut self, packet: DccPacket) {
        if matches!(packet, DccPacket::LogonEnable { .. }) {
            crate::dcc::scheduler::railcom_policy::record_track_logon_sent();
        }
        self.last_packet = Some(packet);
    }

    fn take_dirty_speed_packet(&mut self) -> Option<DccPacket> {
        for index in self.scan_order_from(self.dirty_cursor) {
            if !self.slots[index].dirty_speed {
                continue;
            }
            let packet = self.slots[index].speed_packet();
            if packet.is_some_and(|packet| self.blocks(packet)) {
                continue;
            }

            self.dirty_cursor = (index + 1) % self.slots.len();
            let slot = &mut self.slots[index];
            if slot.dirty_speed_retries == 0 {
                slot.dirty_speed = false;
            } else {
                slot.dirty_speed_retries -= 1;
            }
            slot.last_sent = slot.last_sent.wrapping_add(1);
            if packet.is_some() {
                return packet;
            }
            #[cfg(target_arch = "riscv32")]
            defmt::warn!(
                "invalid speed/format state for addr={}, dropping dirty speed",
                slot.address.value()
            );
        }
        None
    }

    fn take_dirty_function_packet(&mut self) -> DirtyFunctionSelection {
        for index in self.scan_order_from(self.dirty_cursor) {
            let slot = &self.slots[index];
            if slot.dirty_function_groups == 0 {
                continue;
            }
            let group = slot.next_dirty_group();
            let packet = group.map(|group| slot.function_packet_for_group(group));
            if packet.is_some_and(|packet| self.blocks(packet)) {
                continue;
            }

            self.dirty_cursor = (index + 1) % self.slots.len();
            let slot = &mut self.slots[index];
            if let Some(group) = group {
                slot.consume_dirty_group(group);
            }
            slot.last_sent = slot.last_sent.wrapping_add(1);
            return DirtyFunctionSelection::Selected(packet);
        }
        DirtyFunctionSelection::NoPendingGroup
    }

    fn take_pending_telemetry_packet(&mut self) -> Option<DccPacket> {
        self.pending_railcom_telemetry.take()
    }

    /// Whether a stopped slot's refresh period has elapsed.
    fn stopped_slot_is_due(&self, slot: &Slot) -> bool {
        self.refresh_turn.wrapping_sub(slot.stopped_refresh_due) < u32::MAX / 2
    }

    /// Whether a slot's refresh is served at the reduced rate of a stopped
    /// locomotive beside moving ones.
    fn refresh_is_reduced(slot: &Slot, share_turns: bool) -> bool {
        share_turns && !slot.is_moving() && !slot.pom_refresh_only
    }

    fn refresh_candidate(&self, slot: &Slot, tier: RefreshTier, share_turns: bool) -> bool {
        // A slot with dirty traffic reached the refresh stage only because that
        // traffic is blocked; any packet to it now would keep the block alive.
        if slot.has_pending_transmission() {
            return false;
        }
        let due = !Self::refresh_is_reduced(slot, share_turns) || self.stopped_slot_is_due(slot);
        due == (tier == RefreshTier::Due)
    }

    /// First slot from `start`, in `tier`, whose planned refresh packet is not
    /// blocked, with its plan.
    fn find_refresh_slot(
        &self,
        start: usize,
        tier: RefreshTier,
        share_turns: bool,
        max_slot_visits_without_function: u8,
    ) -> Option<(usize, RefreshPlan)> {
        self.scan_order_from(start)
            .filter(|&index| self.refresh_candidate(&self.slots[index], tier, share_turns))
            .map(|index| {
                let slot = &self.slots[index];
                let budget = if Self::refresh_is_reduced(slot, share_turns) {
                    STOPPED_SLOT_FUNCTION_BUDGET
                } else {
                    max_slot_visits_without_function
                };
                (index, slot.plan_refresh(budget))
            })
            .find(|(_, plan)| !plan.packet.is_some_and(|packet| self.blocks(packet)))
    }

    /// One refresh turn: serve the first slot from the cursor that is eligible
    /// and not blocked. Stopped slots beside moving ones are due only every
    /// `STOPPED_REFRESH_PERIOD_TURNS`; they still fill, in their own rotation,
    /// a turn no due slot can take, so an idle goes out only when every
    /// candidate is blocked.
    fn next_refresh_packet(&mut self, max_slot_visits_without_function: u8) -> Option<DccPacket> {
        let len = self.slots.len();
        self.refresh_turn = self.refresh_turn.wrapping_add(1);
        let share_turns = self.slots.iter().any(Slot::is_moving);

        let attempts = [(self.next_index, RefreshTier::Due)]
            .into_iter()
            .chain(share_turns.then_some((self.filler_cursor, RefreshTier::Filler)));
        let (index, plan, tier) = attempts.into_iter().find_map(|(start, tier)| {
            self.find_refresh_slot(start, tier, share_turns, max_slot_visits_without_function)
                .map(|(index, plan)| (index, plan, tier))
        })?;

        let refresh_turn = self.refresh_turn;
        let reduced = Self::refresh_is_reduced(&self.slots[index], share_turns);
        let slot = &mut self.slots[index];
        slot.commit_refresh(&plan);
        slot.last_sent = slot.last_sent.wrapping_add(1);
        if reduced {
            slot.stopped_refresh_due = refresh_turn.wrapping_add(STOPPED_REFRESH_PERIOD_TURNS);
        }
        if plan.packet.is_none() {
            #[cfg(target_arch = "riscv32")]
            defmt::warn!(
                "invalid refresh packet for addr={}, skipping",
                slot.address.value()
            );
        }
        let cursor = match tier {
            RefreshTier::Due => &mut self.next_index,
            RefreshTier::Filler => &mut self.filler_cursor,
        };
        *cursor = (index + 1) % len;
        plan.packet
    }

    pub(in crate::dcc::scheduler) fn build_next_packet_classified_with_function_budget(
        &mut self,
        max_slot_visits_without_function: u8,
    ) -> Option<(DccPacket, PacketClass)> {
        self.dequeued_authority = PacketAuthority::None;
        if self.paused {
            return None;
        }

        if let Some(packet) = self.take_pending_safety_packet() {
            return Some((packet, PacketClass::Safety));
        }
        if let Some(pending) = self.pending_pom.pop_front() {
            self.dequeued_authority =
                pending
                    .permit
                    .map_or(PacketAuthority::None, |permit| PacketAuthority::Pom {
                        request_id: pending.request_id,
                        permit,
                    });
            return Some((pending.packet, PacketClass::Programming));
        }

        // Ahead of the commands: the logon enable it carries has a deadline
        // (RCN-218 §3.2) that a busy throttle must not push back, and it is
        // one packet in dozens.
        if let Some(packet) = self.take_pending_telemetry_packet() {
            return Some((packet, PacketClass::Telemetry));
        }
        if self.slots.is_empty() {
            return None;
        }

        if let Some(packet) = self.take_dirty_speed_packet() {
            return Some((packet, PacketClass::Command));
        }
        match self.take_dirty_function_packet() {
            DirtyFunctionSelection::Selected(packet) => {
                return packet.map(|packet| (packet, PacketClass::Command));
            }
            DirtyFunctionSelection::NoPendingGroup => {}
        }

        self.next_refresh_packet(max_slot_visits_without_function)
            .map(|packet| (packet, PacketClass::Refresh))
    }
}
