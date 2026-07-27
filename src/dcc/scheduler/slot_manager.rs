use core::sync::atomic::{AtomicU32, Ordering};

use heapless::Vec;

use crate::cutout::CutoutMode;
use crate::dcc::PomRequestId;
use crate::dcc::packet::{DccAddress, DccPacket, Direction};
use crate::dcc::speed28::logical_to_nmra_packet_speed;

use super::railcom_policy::PacketClass;
use super::{
    ConsistId, FunctionChange, FunctionIndex, FunctionState, LocoRequest, LocoRequestResult,
    LocoSnapshot, LogicalSpeed, SchedulerCommand, SpeedFormat,
};

/// Maximum number of active locomotive slots.
const MAX_SLOTS: usize = 12;
/// Maximum number of consists managed in software.
const MAX_CONSISTS: usize = 8;
/// Maximum members per consist.
const MAX_CONSIST_MEMBERS: usize = 8;
/// Scheduler target period to revisit one slot in normal conditions.
const TARGET_SLOT_PERIOD_MS: u64 = 120;
/// Minimum scheduler tick to avoid tight loops under high slot count.
const MIN_TICK_MS: u64 = 5;
static SCHEDULER_INVARIANT_RECOVERY_COUNT: AtomicU32 = AtomicU32::new(0);
/// Maximum allowed interval between refreshes of active function groups.
pub(super) const MAX_FUNCTION_REFRESH_MS: u64 = 400;
/// Additional immediate retransmissions after a function state change.
pub(super) const DIRTY_FUNCTION_RETRY_COUNT: u8 = 6;
/// Additional immediate retransmissions after a stop command.
pub(super) const DIRTY_STOP_RETRY_COUNT: u8 = 4;
/// Small bounded queue for programming-on-main packet sequences.
///
/// POM read needs at least two CV-access packets: the command itself and a
/// follow-up packet to the same address where decoders such as the ZIMO
/// reference implementation can emit app:pom in the RailCom cutout.
pub(crate) const PENDING_POM_CAPACITY: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ConsistMember {
    address: DccAddress,
    reverse_in_consist: bool,
}

#[derive(Debug)]
#[cfg_attr(test, derive(Clone, PartialEq, Eq))]
struct Consist {
    id: ConsistId,
    members: Vec<ConsistMember, MAX_CONSIST_MEMBERS>,
}

/// A locomotive slot holding the current command state.
#[derive(Debug)]
#[cfg_attr(test, derive(Clone, PartialEq, Eq))]
struct Slot {
    address: DccAddress,
    // Logical speed value kept in runtime state.
    // Speed28 uses protocol semantics: 0=stop, 1..=28=steps.
    // Conversion to NMRA packet semantics happens only when building DCC packets.
    speed: LogicalSpeed,
    /// Mutation sequence at which this locomotive most recently became stopped.
    /// `None` means that the locomotive is currently moving.
    stopped_since: Option<u64>,
    direction: Direction,
    // Bit 0 = FL, bits 1..28 = F1..F28.
    functions: u32,
    dirty_speed: bool,
    dirty_speed_retries: u8,
    // True once a SetLocoDrive command has been received for this slot.
    // Speed packets are only included in the cyclic refresh when this is set.
    speed_commanded: bool,
    // Dirty function groups bitmask: bit0=FG1, bit1=FG2A, bit2=FG2B, bit3=FG3, bit4=FG4.
    dirty_function_groups: u8,
    // Remaining immediate retries for each dirty function group. Each group uses
    // one 3-bit lane inside this u16 (groups 0..4).
    dirty_function_retries: u16,
    // Tracks which function groups have ever been explicitly addressed.
    // Ensures groups are still refreshed after all functions in them are off.
    known_groups: u8,
    refresh_speed_next: bool,
    refresh_turns_since_function: u8,
    next_function_group: u8,
    last_sent: u32,
    #[cfg(test)]
    deadline_enforced_refreshes: u32,
}

impl Slot {
    fn new(address: DccAddress, speed: LogicalSpeed, direction: Direction) -> Self {
        Self {
            address,
            speed,
            stopped_since: None,
            direction,
            functions: 0,
            dirty_speed: true,
            dirty_speed_retries: speed_retry_count(speed),
            speed_commanded: true,
            dirty_function_groups: 0,
            dirty_function_retries: 0,
            known_groups: 0,
            refresh_speed_next: true,
            refresh_turns_since_function: 0,
            next_function_group: 0,
            last_sent: 0,
            #[cfg(test)]
            deadline_enforced_refreshes: 0,
        }
    }

    fn snapshot(&self) -> LocoSnapshot {
        LocoSnapshot {
            address: self.address,
            speed: self.speed,
            direction: self.direction,
            // Slot mutations only accept FunctionIndex (F0-F28). Truncation is
            // explicit here so a future internal representation change cannot
            // leak reserved bits through the public snapshot.
            functions: FunctionState::from_bits_truncate(self.functions),
        }
    }

    fn has_any_functions(&self) -> bool {
        self.functions != 0 || self.known_groups != 0
    }

    fn set_function(&mut self, function: FunctionIndex, enabled: bool) {
        let function = function.get();
        let bit = 1u32 << function;
        if enabled {
            self.functions |= bit;
        } else {
            self.functions &= !bit;
        }

        let group = function_group(function);
        let group_bit = 1u8 << group;
        self.dirty_function_groups |= group_bit;
        self.set_dirty_retries(group, DIRTY_FUNCTION_RETRY_COUNT);
        self.known_groups |= group_bit;
    }

    fn has_pending_transmission(&self) -> bool {
        self.dirty_speed || self.dirty_function_groups != 0
    }

    fn speed_packet(&self) -> Option<DccPacket> {
        match self.speed.format() {
            SpeedFormat::Speed28 => DccPacket::speed_28step(
                self.address,
                logical_to_nmra_packet_speed(self.speed.value())?,
                self.direction,
            ),
            SpeedFormat::Speed128 => {
                DccPacket::speed_128step(self.address, self.speed.value(), self.direction)
            }
        }
    }

    fn function_packet_for_group(&self, group: u8) -> DccPacket {
        match group {
            0 => DccPacket::FunctionGroup1 {
                address: self.address,
                fl: self.function_enabled(0),
                f1: self.function_enabled(1),
                f2: self.function_enabled(2),
                f3: self.function_enabled(3),
                f4: self.function_enabled(4),
            },
            1 => DccPacket::FunctionGroup2A {
                address: self.address,
                f5: self.function_enabled(5),
                f6: self.function_enabled(6),
                f7: self.function_enabled(7),
                f8: self.function_enabled(8),
            },
            2 => DccPacket::FunctionGroup2B {
                address: self.address,
                f9: self.function_enabled(9),
                f10: self.function_enabled(10),
                f11: self.function_enabled(11),
                f12: self.function_enabled(12),
            },
            3 => DccPacket::FunctionGroup3 {
                address: self.address,
                f13: self.function_enabled(13),
                f14: self.function_enabled(14),
                f15: self.function_enabled(15),
                f16: self.function_enabled(16),
                f17: self.function_enabled(17),
                f18: self.function_enabled(18),
                f19: self.function_enabled(19),
                f20: self.function_enabled(20),
            },
            _ => DccPacket::FunctionGroup4 {
                address: self.address,
                f21: self.function_enabled(21),
                f22: self.function_enabled(22),
                f23: self.function_enabled(23),
                f24: self.function_enabled(24),
                f25: self.function_enabled(25),
                f26: self.function_enabled(26),
                f27: self.function_enabled(27),
                f28: self.function_enabled(28),
            },
        }
    }

    fn function_enabled(&self, function: u8) -> bool {
        let bit = 1u32 << function;
        (self.functions & bit) != 0
    }

    fn next_dirty_function_packet(&mut self) -> Option<DccPacket> {
        if self.dirty_function_groups == 0 {
            return None;
        }

        for offset in 0..5 {
            let group = (self.next_function_group + offset) % 5;
            let bit = 1u8 << group;
            if (self.dirty_function_groups & bit) != 0 {
                let retries = self.dirty_retries(group);
                if retries <= 1 {
                    self.dirty_function_groups &= !bit;
                    self.set_dirty_retries(group, 0);
                } else {
                    self.set_dirty_retries(group, retries - 1);
                }
                self.next_function_group = (group + 1) % 5;
                return Some(self.function_packet_for_group(group));
            }
        }
        None
    }

    fn next_refresh_packet_with_budget(
        &mut self,
        max_slot_visits_without_function: u8,
    ) -> Option<DccPacket> {
        if !self.speed_commanded {
            if !self.has_any_functions() {
                return None;
            }
            let packet = self.next_active_function_packet();
            if packet.is_none() {
                record_scheduler_invariant_recovery();
            }
            return packet;
        }

        if !self.has_any_functions() {
            return self.speed_packet();
        }

        let must_send_function =
            self.refresh_turns_since_function + 1 >= max_slot_visits_without_function.max(1);

        if must_send_function && let Some(packet) = self.next_active_function_packet() {
            self.refresh_turns_since_function = 0;
            self.refresh_speed_next = true;
            #[cfg(test)]
            {
                self.deadline_enforced_refreshes = self.deadline_enforced_refreshes.wrapping_add(1);
            }
            return Some(packet);
        }

        if !self.refresh_speed_next {
            self.refresh_speed_next = true;
            if let Some(packet) = self.next_active_function_packet() {
                self.refresh_turns_since_function = 0;
                return Some(packet);
            }
            record_scheduler_invariant_recovery();
            self.refresh_turns_since_function = self.refresh_turns_since_function.saturating_add(1);
            return self.speed_packet();
        }

        self.refresh_speed_next = false;
        self.refresh_turns_since_function = self.refresh_turns_since_function.saturating_add(1);
        self.speed_packet()
    }

    fn next_active_function_packet(&mut self) -> Option<DccPacket> {
        let active_groups = self.active_function_group_mask();
        if active_groups == 0 {
            return None;
        }

        for offset in 0..5 {
            let group = (self.next_function_group + offset) % 5;
            let bit = 1u8 << group;
            if (active_groups & bit) != 0 {
                self.next_function_group = (group + 1) % 5;
                return Some(self.function_packet_for_group(group));
            }
        }
        None
    }

    fn active_function_group_mask(&self) -> u8 {
        let mut mask = 0u8;
        // F0(FL)..F4 -> FunctionGroup1
        if (self.functions & 0b0000_0000_0000_0000_0000_0000_0001_1111) != 0 {
            mask |= 1 << 0;
        }
        // F5..F8 -> FunctionGroup2A
        if (self.functions & 0b0000_0000_0000_0000_0000_0001_1110_0000) != 0 {
            mask |= 1 << 1;
        }
        // F9..F12 -> FunctionGroup2B
        if (self.functions & 0b0000_0000_0000_0000_0001_1110_0000_0000) != 0 {
            mask |= 1 << 2;
        }
        // F13..F20 -> FunctionGroup3
        if (self.functions & 0b0000_0000_0001_1111_1110_0000_0000_0000) != 0 {
            mask |= 1 << 3;
        }
        // F21..F28 -> FunctionGroup4
        if (self.functions & 0b0001_1111_1110_0000_0000_0000_0000_0000) != 0 {
            mask |= 1 << 4;
        }
        mask | self.known_groups
    }

    fn dirty_retries(&self, group: u8) -> u8 {
        let shift = u16::from(group) * 3;
        ((self.dirty_function_retries >> shift) & 0x07) as u8
    }

    fn set_dirty_retries(&mut self, group: u8, retries: u8) {
        let shift = u16::from(group) * 3;
        let mask = !(0x07u16 << shift);
        self.dirty_function_retries =
            (self.dirty_function_retries & mask) | (u16::from(retries.min(7)) << shift);
    }
}

fn record_scheduler_invariant_recovery() {
    SCHEDULER_INVARIANT_RECOVERY_COUNT.fetch_add(1, Ordering::Relaxed);
}

#[cfg(any(test, target_arch = "riscv32"))]
pub(super) fn scheduler_invariant_recovery_count() -> u32 {
    SCHEDULER_INVARIANT_RECOVERY_COUNT.load(Ordering::Acquire)
}

#[cfg(test)]
mod invariant_recovery_tests {
    use super::*;

    fn invalid_group_slot() -> Slot {
        let mut slot = Slot::new(
            DccAddress::new_short(3).unwrap(),
            LogicalSpeed::zero(SpeedFormat::Speed128),
            Direction::Forward,
        );
        slot.functions = 0;
        slot.known_groups = 0x80;
        slot
    }

    #[test]
    fn invalid_function_only_state_degrades_to_no_packet() {
        let before = scheduler_invariant_recovery_count();
        let mut slot = invalid_group_slot();
        slot.speed_commanded = false;

        assert_eq!(slot.next_refresh_packet_with_budget(1), None);
        assert_eq!(scheduler_invariant_recovery_count().wrapping_sub(before), 1);
    }

    #[test]
    fn invalid_mixed_state_degrades_to_speed_packet() {
        let before = scheduler_invariant_recovery_count();
        let mut slot = invalid_group_slot();
        slot.refresh_speed_next = false;

        assert!(matches!(
            slot.next_refresh_packet_with_budget(1),
            Some(DccPacket::Speed128 { .. })
        ));
        assert_eq!(scheduler_invariant_recovery_count().wrapping_sub(before), 1);
    }
}

/// Manages active locomotive slots with round-robin scheduling.
#[cfg_attr(test, derive(Debug, Clone, PartialEq, Eq))]
pub struct SlotManager {
    slots: Vec<Slot, MAX_SLOTS>,
    consists: Vec<Consist, MAX_CONSISTS>,
    pending_broadcast_estop: bool,
    pending_estop_targets: Vec<DccAddress, MAX_SLOTS>,
    pending_pom: Vec<DccPacket, PENDING_POM_CAPACITY>,
    active_pom: Option<ActivePomRequest>,
    pending_railcom_telemetry: Option<DccPacket>,
    next_index: usize,
    mutation_sequence: u64,
    paused: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SlotAdmission {
    Inserted,
    Replaced { removed: DccAddress },
    Full,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ActivePomRequest {
    request_id: PomRequestId,
    target_address: DccAddress,
    cutout: CutoutMode,
}

enum DirtyFunctionSelection {
    NoPendingGroup,
    // `None` preserves fail-closed behavior for an invalid dirty group: the
    // current scheduler tick emits nothing instead of falling through to
    // lower-priority telemetry or refresh traffic.
    Selected(Option<DccPacket>),
}

impl Default for SlotManager {
    fn default() -> Self {
        Self::new()
    }
}

impl SlotManager {
    /// Create a new empty SlotManager.
    #[must_use]
    pub fn new() -> Self {
        Self {
            slots: Vec::new(),
            consists: Vec::new(),
            pending_broadcast_estop: false,
            pending_estop_targets: Vec::new(),
            pending_pom: Vec::new(),
            active_pom: None,
            pending_railcom_telemetry: None,
            next_index: 0,
            mutation_sequence: 0,
            paused: false,
        }
    }

    /// Return a copy of the scheduler-owned state for one locomotive.
    #[must_use]
    pub fn loco_snapshot(&self, address: DccAddress) -> Option<LocoSnapshot> {
        self.slots
            .iter()
            .find(|slot| slot.address == address)
            .map(Slot::snapshot)
    }

    fn advance_mutation_sequence(&mut self) -> u64 {
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

    fn admit_new_slot(&mut self, mut slot: Slot, replace_stopped: bool) -> SlotAdmission {
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
    pub(super) fn set_mutation_sequence_for_test(&mut self, sequence: u64) {
        self.mutation_sequence = sequence;
    }

    #[cfg(test)]
    pub(super) fn stopped_since_for_test(&self, address: DccAddress) -> Option<u64> {
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

    /// Create an empty consist with given ID.
    #[must_use]
    pub fn create_consist(&mut self, id: ConsistId) -> bool {
        if self.consists.iter().any(|c| c.id == id) {
            return true;
        }
        if self.consists.is_full() {
            return false;
        }
        let _ = self.consists.push(Consist {
            id,
            members: Vec::new(),
        });
        true
    }

    /// Add a locomotive to a consist.
    #[must_use]
    pub fn add_to_consist(
        &mut self,
        id: ConsistId,
        address: DccAddress,
        reverse_in_consist: bool,
    ) -> bool {
        let Some(consist) = self.consists.iter_mut().find(|c| c.id == id) else {
            return false;
        };

        if consist.members.iter().any(|m| m.address == address) {
            return true;
        }
        if consist.members.is_full() {
            return false;
        }

        let _ = consist.members.push(ConsistMember {
            address,
            reverse_in_consist,
        });
        true
    }

    /// Remove a locomotive from a consist.
    #[must_use]
    pub fn remove_from_consist(&mut self, id: ConsistId, address: DccAddress) -> bool {
        let Some(consist) = self.consists.iter_mut().find(|c| c.id == id) else {
            return false;
        };

        if let Some(pos) = consist.members.iter().position(|m| m.address == address) {
            consist.members.swap_remove(pos);
            true
        } else {
            false
        }
    }

    /// Apply speed command to all consist members.
    #[must_use]
    pub fn set_consist_speed(
        &mut self,
        id: ConsistId,
        speed: LogicalSpeed,
        direction: Direction,
    ) -> usize {
        let Some(consist) = self.consists.iter().find(|c| c.id == id) else {
            return 0;
        };

        let mut members: Vec<ConsistMember, MAX_CONSIST_MEMBERS> = Vec::new();
        for m in &consist.members {
            let _ = members.push(*m);
        }

        let mut updated = 0usize;
        for member in members {
            let member_direction = if member.reverse_in_consist {
                match direction {
                    Direction::Forward => Direction::Reverse,
                    Direction::Reverse => Direction::Forward,
                }
            } else {
                direction
            };

            if self.set_speed(member.address, speed, member_direction) {
                updated += 1;
            }
        }

        updated
    }

    /// Remove a locomotive slot.
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

    fn remove_address_from_consists(&mut self, address: DccAddress) {
        for consist in &mut self.consists {
            consist.members.retain(|member| member.address != address);
        }
    }

    #[cfg(test)]
    pub(super) fn has_pending_slot_transmissions_for_test(&self) -> bool {
        self.pending_broadcast_estop
            || !self.pending_estop_targets.is_empty()
            || self.slots.iter().any(Slot::has_pending_transmission)
    }

    /// Pause packet emission. All slot state is preserved.
    pub fn pause(&mut self) {
        self.paused = true;
    }

    /// Resume packet emission after pause.
    pub fn resume(&mut self) {
        self.paused = false;
    }

    /// Check if scheduler is currently paused.
    #[cfg(test)]
    #[must_use]
    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// Queue one explicit POM packet for single transmission.
    #[must_use]
    pub fn program_on_main(&mut self, request_id: PomRequestId, packet: DccPacket) -> bool {
        let (target_address, cutout) = match packet {
            DccPacket::PomReadByte { address, .. } => (address, CutoutMode::PomRead),
            DccPacket::PomWriteByte { address, .. } => (address, CutoutMode::PomWrite),
            _ => return false,
        };
        if let Some(active) = self.active_pom {
            if active.request_id == request_id {
                if active.target_address != target_address || active.cutout != cutout {
                    return false;
                }
            } else {
                self.pending_pom.clear();
            }
        }
        self.active_pom = Some(ActivePomRequest {
            request_id,
            target_address,
            cutout,
        });
        self.pending_pom.push(packet).is_ok()
    }

    pub fn close_program_on_main(&mut self, request_id: PomRequestId) -> bool {
        if !self
            .active_pom
            .is_some_and(|active| active.request_id == request_id)
        {
            return false;
        }
        self.active_pom = None;
        self.pending_pom.clear();
        true
    }

    #[cfg(any(test, target_arch = "riscv32"))]
    pub(super) fn pom_context_for_packet(
        &self,
        packet: DccPacket,
    ) -> Option<(PomRequestId, CutoutMode)> {
        let target_address = packet.railcom_target_address()?;
        self.active_pom
            .filter(|active| active.target_address == target_address)
            .map(|active| (active.request_id, active.cutout))
    }

    /// Ensure the scheduler has a non-dirty cyclic refresh slot for a decoder.
    #[must_use]
    pub fn ensure_railcom_refresh_slot(
        &mut self,
        address: DccAddress,
        format: SpeedFormat,
    ) -> bool {
        if self.slots.iter().any(|slot| slot.address == address) {
            return true;
        }
        let mut slot = Slot::new(address, LogicalSpeed::zero(format), Direction::Forward);
        slot.dirty_speed = false;
        slot.dirty_speed_retries = 0;
        matches!(self.admit_new_slot(slot, false), SlotAdmission::Inserted)
    }

    #[must_use]
    pub fn railcom_telemetry_packet(&mut self, packet: DccPacket) -> bool {
        if self.pending_railcom_telemetry.is_some() {
            return false;
        }
        self.pending_railcom_telemetry = Some(packet);
        true
    }

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

    fn take_pending_programming_packet(&mut self) -> Option<DccPacket> {
        (!self.pending_pom.is_empty()).then(|| self.pending_pom.remove(0))
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

    pub(super) fn build_next_packet_classified_with_function_budget(
        &mut self,
        max_slot_visits_without_function: u8,
    ) -> Option<(DccPacket, PacketClass)> {
        if self.paused {
            return None;
        }

        if let Some(packet) = self.take_pending_safety_packet() {
            return Some((packet, PacketClass::Safety));
        }
        if let Some(packet) = self.take_pending_programming_packet() {
            return Some((packet, PacketClass::Programming));
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

    /// Number of active slots.
    #[must_use]
    pub fn slot_count(&self) -> usize {
        self.slots.len()
    }

    /// Returns `true` if no slots are active.
    #[cfg(test)]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }

    /// Apply a scheduler command.
    #[must_use]
    pub fn apply_command(&mut self, command: SchedulerCommand) -> bool {
        match command {
            SchedulerCommand::EmergencyStopAll => {
                self.request_emergency_stop_all();
                true
            }
            SchedulerCommand::ProgramOnMain { request_id, packet } => {
                self.program_on_main(request_id, packet)
            }
            SchedulerCommand::CloseProgramOnMain { request_id } => {
                self.close_program_on_main(request_id)
            }
            SchedulerCommand::SuspendRailcomDiscovery
            | SchedulerCommand::RestartRailcomDiscovery => true,
            SchedulerCommand::RailcomTelemetry { packet } => self.railcom_telemetry_packet(packet),
            SchedulerCommand::CreateConsist { id } => self.create_consist(id),
            SchedulerCommand::AddToConsist {
                id,
                address,
                reverse_in_consist,
            } => self.add_to_consist(id, address, reverse_in_consist),
            SchedulerCommand::RemoveFromConsist { id, address } => {
                self.remove_from_consist(id, address)
            }
            SchedulerCommand::SetConsistSpeed {
                id,
                speed,
                direction,
            } => self.set_consist_speed(id, speed, direction) > 0,
            SchedulerCommand::RemoveSlot { address } => self.remove_slot(address),
            SchedulerCommand::Pause => {
                self.pause();
                true
            }
            SchedulerCommand::Resume => {
                self.resume();
                true
            }
        }
    }

    #[cfg(test)]
    pub(super) fn deadline_enforced_refresh_count(&self) -> u32 {
        self.slots
            .iter()
            .map(|s| s.deadline_enforced_refreshes)
            .sum()
    }
}

const fn speed_retry_count(speed: LogicalSpeed) -> u8 {
    if speed.value() == 0 {
        DIRTY_STOP_RETRY_COUNT
    } else {
        0
    }
}

pub(super) fn scheduler_tick_ms_for_slot_count(slot_count: usize) -> u64 {
    (TARGET_SLOT_PERIOD_MS / slot_count.max(1) as u64).max(MIN_TICK_MS)
}

pub(super) fn allowed_slot_visits_without_function(slot_count: usize) -> u8 {
    let slot_period_ms = scheduler_tick_ms_for_slot_count(slot_count) * slot_count.max(1) as u64;
    let visits = (MAX_FUNCTION_REFRESH_MS / slot_period_ms).max(1);
    visits.min(u8::MAX as u64) as u8
}

fn function_group(function: u8) -> u8 {
    match function {
        0..=4 => 0,
        5..=8 => 1,
        9..=12 => 2,
        13..=20 => 3,
        _ => 4,
    }
}
