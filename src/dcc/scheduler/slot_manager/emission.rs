//! Packet selection and emission-priority policy for scheduler slots.

use crate::dcc::packet::{DccPacket, Direction};

#[cfg(test)]
use super::allowed_slot_visits_without_function;
use super::{DirtyFunctionSelection, PacketAuthority, PacketClass, SlotManager};

impl SlotManager {
    /// Build the next packet to transmit.
    #[cfg(test)]
    pub fn build_next_packet(&mut self) -> Option<DccPacket> {
        self.build_next_packet_with_function_budget(allowed_slot_visits_without_function(
            self.slots.len(),
        ))
    }

    /// Build next packet with an explicit function-refresh budget in slot visits.
    #[cfg(test)]
    pub fn build_next_packet_with_function_budget(
        &mut self,
        max_slot_visits_without_function: u8,
    ) -> Option<DccPacket> {
        self.build_next_packet_classified_with_function_budget(max_slot_visits_without_function)
            .map(|(packet, _)| packet)
    }

    fn take_pending_safety_packet(&mut self) -> Option<DccPacket> {
        if self.pending_broadcast_estop {
            self.pending_broadcast_estop = false;
            return Some(DccPacket::BroadcastStop);
        }

        if self.pending_estop_targets.is_empty() {
            return None;
        }

        let address = self.pending_estop_targets.remove(0);
        let direction = self
            .slots
            .iter()
            .find(|slot| slot.address == address)
            .map(|slot| slot.direction)
            .unwrap_or(Direction::Reverse);
        Some(DccPacket::EmergencyStop { address, direction })
    }

    fn take_dirty_speed_packet(&mut self) -> Option<DccPacket> {
        for slot in &mut self.slots {
            if !slot.dirty_speed {
                continue;
            }

            if slot.dirty_speed_retries == 0 {
                slot.dirty_speed = false;
            } else {
                slot.dirty_speed_retries -= 1;
            }
            let packet = slot.speed_packet();
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
        let Some(slot) = self
            .slots
            .iter_mut()
            .find(|slot| slot.dirty_function_groups != 0)
        else {
            return DirtyFunctionSelection::NoPendingGroup;
        };
        let packet = slot.next_dirty_function_packet();
        slot.last_sent = slot.last_sent.wrapping_add(1);
        DirtyFunctionSelection::Selected(packet)
    }

    fn take_pending_telemetry_packet(&mut self) -> Option<DccPacket> {
        self.pending_railcom_telemetry.take()
    }

    fn next_refresh_packet(&mut self, max_slot_visits_without_function: u8) -> Option<DccPacket> {
        if self.next_index >= self.slots.len() {
            self.next_index = 0;
        }

        let slot = &mut self.slots[self.next_index];
        let packet = slot.next_refresh_packet_with_budget(max_slot_visits_without_function);
        slot.last_sent = slot.last_sent.wrapping_add(1);
        if packet.is_none() {
            #[cfg(target_arch = "riscv32")]
            defmt::warn!(
                "invalid refresh packet for addr={}, skipping",
                slot.address.value()
            );
        }

        self.next_index = (self.next_index + 1) % self.slots.len();
        packet
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
        if !self.pending_pom.is_empty() {
            let pending = self.pending_pom.remove(0);
            self.dequeued_authority =
                pending
                    .permit
                    .map_or(PacketAuthority::None, |permit| PacketAuthority::Pom {
                        request_id: pending.request_id,
                        permit,
                    });
            return Some((pending.packet, PacketClass::Programming));
        }

        if self.slots.is_empty() {
            return self
                .take_pending_telemetry_packet()
                .map(|packet| (packet, PacketClass::Telemetry));
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
        if let Some(packet) = self.take_pending_telemetry_packet() {
            return Some((packet, PacketClass::Telemetry));
        }

        self.next_refresh_packet(max_slot_visits_without_function)
            .map(|packet| (packet, PacketClass::Refresh))
    }
}
