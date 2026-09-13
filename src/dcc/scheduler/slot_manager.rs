use core::sync::atomic::{AtomicU32, Ordering};

use heapless::Vec;

use crate::authority::{LeaseEpoch, LeasePermit};
use crate::cutout::CutoutMode;
use crate::dcc::PomRequestId;
use crate::dcc::packet::{DccAddress, DccPacket, Direction};
use crate::dcc::speed28::logical_to_nmra_packet_speed;

use super::railcom_policy::PacketClass;
use super::{
    ConsistId, FunctionIndex, FunctionState, LOCO_SLOT_CAPACITY, LocoSnapshot, LogicalSpeed,
    SchedulerCommand, SpeedFormat,
};

mod consists;
mod emission;
mod locomotives;
mod pom;

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
    slots: Vec<Slot, LOCO_SLOT_CAPACITY>,
    consists: Vec<Consist, MAX_CONSISTS>,
    pending_broadcast_estop: bool,
    pending_estop_targets: Vec<DccAddress, LOCO_SLOT_CAPACITY>,
    pending_pom: Vec<PendingPomPacket, PENDING_POM_CAPACITY>,
    active_pom: Option<ActivePomRequest>,
    pending_railcom_telemetry: Option<DccPacket>,
    next_index: usize,
    mutation_sequence: u64,
    paused: bool,
    accepted_lease_epoch: Option<LeaseEpoch>,
    dequeued_authority: PacketAuthority,
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
    permit: Option<LeasePermit>,
    target_address: DccAddress,
    cutout: CutoutMode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PendingPomPacket {
    request_id: PomRequestId,
    permit: Option<LeasePermit>,
    packet: DccPacket,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PacketAuthority {
    None,
    Pom {
        request_id: PomRequestId,
        permit: LeasePermit,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ScheduledPacket {
    pub(super) packet: DccPacket,
    pub(super) class: PacketClass,
    pub(super) authority: PacketAuthority,
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
            accepted_lease_epoch: None,
            dequeued_authority: PacketAuthority::None,
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
            SchedulerCommand::ProgramOnMain {
                request_id,
                permit,
                packet,
            } => {
                self.accepts_lease(permit)
                    && self.program_on_main_authorized(request_id, permit, packet)
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

    pub(super) fn take_dequeued_authority(&mut self) -> PacketAuthority {
        core::mem::replace(&mut self.dequeued_authority, PacketAuthority::None)
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
