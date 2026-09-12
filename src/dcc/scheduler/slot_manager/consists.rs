//! Consist membership and slot-removal operations.

use heapless::Vec;

use super::{Consist, ConsistMember, MAX_CONSIST_MEMBERS, SlotManager};
use crate::dcc::{ConsistId, DccAddress, Direction, LogicalSpeed};

impl SlotManager {
    /// Create an empty consist with given ID.
    #[must_use]
    pub fn create_consist(&mut self, id: ConsistId) -> bool {
        if self.consists.iter().any(|consist| consist.id == id) {
            return true;
        }
        if self.consists.is_full() {
            return false;
        }
        self.consists
            .push(Consist {
                id,
                members: Vec::new(),
            })
            .is_ok()
    }

    /// Add a locomotive to a consist.
    #[must_use]
    pub fn add_to_consist(
        &mut self,
        id: ConsistId,
        address: DccAddress,
        reverse_in_consist: bool,
    ) -> bool {
        let Some(consist) = self.consists.iter_mut().find(|consist| consist.id == id) else {
            return false;
        };

        if consist
            .members
            .iter()
            .any(|member| member.address == address)
        {
            return true;
        }
        if consist.members.is_full() {
            return false;
        }

        consist
            .members
            .push(ConsistMember {
                address,
                reverse_in_consist,
            })
            .is_ok()
    }

    /// Remove a locomotive from a consist.
    #[must_use]
    pub fn remove_from_consist(&mut self, id: ConsistId, address: DccAddress) -> bool {
        let Some(consist) = self.consists.iter_mut().find(|consist| consist.id == id) else {
            return false;
        };

        if let Some(position) = consist
            .members
            .iter()
            .position(|member| member.address == address)
        {
            consist.members.swap_remove(position);
            true
        } else {
            false
        }
    }

    /// Apply a speed command to all consist members.
    #[must_use]
    pub fn set_consist_speed(
        &mut self,
        id: ConsistId,
        speed: LogicalSpeed,
        direction: Direction,
    ) -> usize {
        let Some(consist) = self.consists.iter().find(|consist| consist.id == id) else {
            return 0;
        };

        let mut members: Vec<ConsistMember, MAX_CONSIST_MEMBERS> = Vec::new();
        members.extend(consist.members.iter().copied());

        members
            .into_iter()
            .filter(|member| {
                let direction = if member.reverse_in_consist {
                    match direction {
                        Direction::Forward => Direction::Reverse,
                        Direction::Reverse => Direction::Forward,
                    }
                } else {
                    direction
                };
                self.set_speed(member.address, speed, direction)
            })
            .count()
    }

    /// Remove a stopped locomotive slot with no pending transmission.
    #[must_use]
    pub fn remove_slot(&mut self, address: DccAddress) -> bool {
        let Some(index) = self.slots.iter().position(|slot| slot.address == address) else {
            return false;
        };
        let slot = &self.slots[index];
        if !slot.speed.is_zero()
            || slot.has_pending_transmission()
            || self.pending_estop_targets.contains(&address)
        {
            return false;
        }

        self.slots.swap_remove(index);
        self.advance_mutation_sequence();
        self.remove_address_from_consists(address);
        if self.next_index >= self.slots.len() {
            self.next_index = 0;
        }
        true
    }

    pub(super) fn remove_address_from_consists(&mut self, address: DccAddress) {
        for consist in &mut self.consists {
            consist.members.retain(|member| member.address != address);
        }
    }
}
