//! Locomotive slot admission, mutation, and emergency-stop policy.

use crate::authority::LeasePermit;
use crate::dcc::packet::{DccAddress, Direction};
use crate::dcc::scheduler::{
    FunctionChange, LocoRequest, LocoRequestResult, LocoSnapshot, LogicalSpeed, SpeedFormat,
};

use super::{DIRTY_STOP_RETRY_COUNT, Slot, SlotAdmission, SlotManager, speed_retry_count};

impl SlotManager {
    /// Return a copy of the scheduler-owned state for one locomotive.
    #[must_use]
    pub fn loco_snapshot(&self, address: DccAddress) -> Option<LocoSnapshot> {
        self.slots
            .iter()
            .find(|slot| slot.address == address)
            .map(Slot::snapshot)
    }

    pub(super) fn advance_mutation_sequence(&mut self) -> u64 {
        self.mutation_sequence = self.mutation_sequence.wrapping_add(1);
        self.mutation_sequence
    }

    fn update_speed_at(&mut self, index: usize, speed: LogicalSpeed, direction: Direction) {
        let was_stopped = self.slots[index].speed.is_zero();
        let is_stopped = speed.is_zero();
        let stopped_since = match (was_stopped, is_stopped) {
            (false, true) => Some(self.advance_mutation_sequence()),
            (true, false) => {
                self.advance_mutation_sequence();
                None
            }
            _ => self.slots[index].stopped_since,
        };

        let slot = &mut self.slots[index];
        slot.speed = speed;
        slot.stopped_since = stopped_since;
        slot.direction = direction;
        slot.dirty_speed = true;
        slot.dirty_speed_retries = speed_retry_count(speed);
        slot.speed_commanded = true;
    }

    fn oldest_replaceable_stopped_slot_index(&self) -> Option<usize> {
        let now = self.mutation_sequence;
        self.slots
            .iter()
            .enumerate()
            .filter(|(_, slot)| {
                !slot.has_pending_transmission()
                    && !self.pending_estop_targets.contains(&slot.address)
            })
            .filter_map(|(index, slot)| {
                slot.stopped_since
                    .map(|stopped_since| (index, now.wrapping_sub(stopped_since)))
            })
            .fold(None, |oldest, candidate| match oldest {
                Some((_, oldest_age)) if oldest_age >= candidate.1 => oldest,
                _ => Some(candidate),
            })
            .map(|(index, _)| index)
    }

    pub(super) fn admit_new_slot(
        &mut self,
        mut slot: Slot,
        replace_stopped: bool,
    ) -> SlotAdmission {
        debug_assert!(
            self.slots
                .iter()
                .all(|existing| existing.address != slot.address)
        );

        if !self.slots.is_full() {
            self.prepare_admitted_slot(&mut slot);
            if self.slots.push(slot).is_err() {
                return SlotAdmission::Full;
            }
            return SlotAdmission::Inserted;
        }

        if !replace_stopped {
            return SlotAdmission::Full;
        }

        let Some(index) = self.oldest_replaceable_stopped_slot_index() else {
            return SlotAdmission::Full;
        };
        let removed = self.slots[index].address;
        self.prepare_admitted_slot(&mut slot);
        self.slots[index] = slot;
        self.remove_address_from_consists(removed);
        SlotAdmission::Replaced { removed }
    }

    fn prepare_admitted_slot(&mut self, slot: &mut Slot) {
        let sequence = self.advance_mutation_sequence();
        slot.stopped_since = slot.speed.is_zero().then_some(sequence);
    }

    fn result_from_admission(
        &self,
        address: DccAddress,
        admission: SlotAdmission,
    ) -> LocoRequestResult {
        let snapshot = || self.loco_snapshot(address);
        match admission {
            SlotAdmission::Inserted => {
                snapshot().map_or(LocoRequestResult::Rejected, LocoRequestResult::Inserted)
            }
            SlotAdmission::Replaced { removed } => snapshot()
                .map_or(LocoRequestResult::Rejected, |inserted| {
                    LocoRequestResult::Replaced { removed, inserted }
                }),
            SlotAdmission::Full => LocoRequestResult::Full,
        }
    }

    #[cfg(test)]
    pub(in crate::dcc::scheduler) fn set_mutation_sequence_for_test(&mut self, sequence: u64) {
        self.mutation_sequence = sequence;
    }

    #[cfg(test)]
    pub(in crate::dcc::scheduler) fn stopped_since_for_test(
        &self,
        address: DccAddress,
    ) -> Option<u64> {
        self.slots
            .iter()
            .find(|slot| slot.address == address)
            .and_then(|slot| slot.stopped_since)
    }

    /// Apply a request from an external adapter and report the resulting state.
    ///
    /// A new command may atomically replace the oldest stopped slot only after
    /// every accepted speed, function, and emergency-stop packet for that slot
    /// has been handed to the DCC output queue. Refresh-only requests never
    /// replace a slot. If no stopped slot is safe to replace, the request returns
    /// [`LocoRequestResult::Full`] without mutation.
    #[must_use]
    pub fn handle_loco_request(&mut self, request: LocoRequest) -> LocoRequestResult {
        match request {
            LocoRequest::GetState { address } => self
                .loco_snapshot(address)
                .map_or(LocoRequestResult::NotFound, LocoRequestResult::Found),
            LocoRequest::EnsureRefresh { address, format } => {
                if let Some(snapshot) = self.loco_snapshot(address) {
                    return LocoRequestResult::Found(snapshot);
                }
                if !self.ensure_railcom_refresh_slot(address, format) {
                    return LocoRequestResult::Full;
                }
                let Some(snapshot) = self.loco_snapshot(address) else {
                    return LocoRequestResult::Rejected;
                };
                LocoRequestResult::Inserted(snapshot)
            }
            LocoRequest::SetSpeed {
                address,
                speed,
                direction,
            } => {
                if let Some(index) = self.slots.iter().position(|slot| slot.address == address) {
                    self.update_speed_at(index, speed, direction);
                    return self
                        .loco_snapshot(address)
                        .map_or(LocoRequestResult::Rejected, LocoRequestResult::Updated);
                }
                let slot = Slot::new(address, speed, direction);
                let admission = self.admit_new_slot(slot, true);
                self.result_from_admission(address, admission)
            }
            LocoRequest::SetFunction {
                address,
                function,
                change,
            } => {
                if let Some(slot) = self.slots.iter_mut().find(|slot| slot.address == address) {
                    let enabled = match change {
                        FunctionChange::Enable => true,
                        FunctionChange::Disable => false,
                        FunctionChange::Toggle => !slot.function_enabled(function.get()),
                    };
                    slot.set_function(function, enabled);
                    return LocoRequestResult::Updated(slot.snapshot());
                }

                let mut slot = Slot::new(
                    address,
                    LogicalSpeed::zero(SpeedFormat::Speed28),
                    Direction::Forward,
                );
                slot.dirty_speed = false;
                slot.speed_commanded = false;
                let enabled = !matches!(change, FunctionChange::Disable);
                slot.set_function(function, enabled);
                let admission = self.admit_new_slot(slot, true);
                self.result_from_admission(address, admission)
            }
            LocoRequest::EmergencyStop { address } => {
                if !self.request_emergency_stop(address) {
                    return LocoRequestResult::NotFound;
                }
                let Some(snapshot) = self.loco_snapshot(address) else {
                    return LocoRequestResult::Rejected;
                };
                LocoRequestResult::Updated(snapshot)
            }
        }
    }

    /// Set a validated logical speed for a locomotive.
    #[must_use]
    pub fn set_speed(
        &mut self,
        address: DccAddress,
        speed: LogicalSpeed,
        direction: Direction,
    ) -> bool {
        if let Some(index) = self.slots.iter().position(|slot| slot.address == address) {
            self.update_speed_at(index, speed, direction);
            return true;
        }

        matches!(
            self.admit_new_slot(Slot::new(address, speed, direction), false),
            SlotAdmission::Inserted
        )
    }

    #[cfg(test)]
    #[must_use]
    pub fn set_speed_with_format(
        &mut self,
        address: DccAddress,
        speed: LogicalSpeed,
        direction: Direction,
        format: SpeedFormat,
    ) -> bool {
        speed.format() == format && self.set_speed(address, speed, direction)
    }

    /// Request global emergency stop.
    pub fn request_emergency_stop_all(&mut self) {
        let stopped_since = self
            .slots
            .iter()
            .any(|slot| !slot.speed.is_zero())
            .then(|| self.advance_mutation_sequence());
        for slot in self.slots.iter_mut() {
            if !slot.speed.is_zero() {
                slot.stopped_since = stopped_since;
            }
            slot.speed = LogicalSpeed::zero(slot.speed.format());
            slot.dirty_speed = true;
        }
        self.pending_broadcast_estop = true;
        self.pending_estop_targets.clear();
    }

    /// Establish a fresh mutation epoch after stopping previous controller work.
    pub(crate) fn reset_for_lease(&mut self, permit: LeasePermit) {
        self.request_emergency_stop_all();
        self.pending_pom.clear();
        self.active_pom = None;
        self.pending_railcom_telemetry = None;
        self.accepted_lease_epoch = Some(permit.epoch());
    }

    /// Stop active work and drain authority-sensitive queues before a power-on
    /// fence. The accepted lease remains unchanged.
    #[cfg(target_arch = "riscv32")]
    pub(crate) fn quiesce_for_power_on(&mut self) {
        self.request_emergency_stop_all();
        self.pending_pom.clear();
        self.active_pom = None;
        self.pending_railcom_telemetry = None;
        self.paused = true;
    }

    #[must_use]
    pub(crate) fn accepts_lease(&self, permit: LeasePermit) -> bool {
        self.accepted_lease_epoch == Some(permit.epoch())
    }

    /// Request emergency stop for a single locomotive.
    #[must_use]
    pub fn request_emergency_stop(&mut self, address: DccAddress) -> bool {
        let Some(index) = self.slots.iter().position(|slot| slot.address == address) else {
            return false;
        };

        if !self.slots[index].speed.is_zero() {
            self.slots[index].stopped_since = Some(self.advance_mutation_sequence());
        }
        let slot = &mut self.slots[index];
        slot.speed = LogicalSpeed::zero(slot.speed.format());
        slot.dirty_speed = true;
        slot.dirty_speed_retries = DIRTY_STOP_RETRY_COUNT;

        if !self.pending_estop_targets.contains(&address)
            && self.pending_estop_targets.push(address).is_err()
        {
            #[cfg(target_arch = "riscv32")]
            defmt::warn!(
                "e-stop queue full; dropping request for addr={}",
                address.value()
            );
            return false;
        }
        true
    }
}
