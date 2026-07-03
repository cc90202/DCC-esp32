use heapless::Vec;

use crate::dcc::packet::{DccAddress, DccPacket, Direction};
use crate::dcc::speed28::logical_to_nmra_packet_speed;

use super::railcom_policy::PacketClass;
use super::{FunctionIndex, LogicalSpeed, MAX_FUNCTION_INDEX, SchedulerCommand, SpeedFormat};

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

#[derive(Debug, Clone)]
struct Consist {
    id: u8,
    members: Vec<ConsistMember, MAX_CONSIST_MEMBERS>,
}

/// A locomotive slot holding the current command state.
#[derive(Debug, Clone)]
struct Slot {
    address: DccAddress,
    // Logical speed value kept in runtime state.
    // Speed28 uses protocol semantics: 0=stop, 1..=28=steps.
    // Conversion to NMRA packet semantics happens only when building DCC packets.
    speed: LogicalSpeed,
    direction: Direction,
    format: SpeedFormat,
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
    fn new(
        address: DccAddress,
        speed: LogicalSpeed,
        direction: Direction,
        format: SpeedFormat,
    ) -> Self {
        Self {
            address,
            speed,
            direction,
            format,
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

    fn has_any_functions(&self) -> bool {
        self.functions != 0 || self.known_groups != 0
    }

    fn set_function(&mut self, function: u8, enabled: bool) -> bool {
        if function > MAX_FUNCTION_INDEX {
            return false;
        }
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
        true
    }

    fn speed_packet(&self) -> Option<DccPacket> {
        match self.format {
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
            return Some(
                self.next_active_function_packet()
                    .expect("function-only slot has no active function group packet"),
            );
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
            unreachable!("slot has active functions but no active function group packet available");
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

/// Manages active locomotive slots with round-robin scheduling.
pub struct SlotManager {
    slots: Vec<Slot, MAX_SLOTS>,
    consists: Vec<Consist, MAX_CONSISTS>,
    pending_broadcast_estop: bool,
    pending_estop_targets: Vec<DccAddress, MAX_SLOTS>,
    pending_pom: Vec<DccPacket, PENDING_POM_CAPACITY>,
    pending_railcom_telemetry: Option<DccPacket>,
    next_index: usize,
    paused: bool,
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
            pending_railcom_telemetry: None,
            next_index: 0,
            paused: false,
        }
    }

    /// Set speed for a locomotive using Speed28 format.
    #[must_use]
    pub fn set_speed(
        &mut self,
        address: DccAddress,
        speed: LogicalSpeed,
        direction: Direction,
    ) -> bool {
        self.set_speed_with_format(address, speed, direction, SpeedFormat::Speed28)
    }

    /// Set speed with explicit format selection.
    #[must_use]
    pub fn set_speed_with_format(
        &mut self,
        address: DccAddress,
        speed: LogicalSpeed,
        direction: Direction,
        format: SpeedFormat,
    ) -> bool {
        for slot in self.slots.iter_mut() {
            if slot.address == address {
                slot.speed = speed;
                slot.direction = direction;
                slot.format = format;
                slot.dirty_speed = true;
                slot.dirty_speed_retries = speed_retry_count(speed);
                slot.speed_commanded = true;
                return true;
            }
        }

        if self.slots.is_full() {
            return false;
        }

        let _ = self
            .slots
            .push(Slot::new(address, speed, direction, format));
        true
    }

    /// Set function state for a locomotive.
    #[must_use]
    pub fn set_function(&mut self, address: DccAddress, function: u8, enabled: bool) -> bool {
        let Ok(function) = FunctionIndex::try_from(function) else {
            return false;
        };
        self.set_function_indexed(address, function, enabled)
    }

    /// Set function state for a locomotive using a validated function index.
    #[must_use]
    pub fn set_function_indexed(
        &mut self,
        address: DccAddress,
        function: FunctionIndex,
        enabled: bool,
    ) -> bool {
        let function = function.get();
        for slot in self.slots.iter_mut() {
            if slot.address == address {
                return slot.set_function(function, enabled);
            }
        }

        if self.slots.is_full() {
            return false;
        }

        let mut slot = Slot::new(
            address,
            LogicalSpeed::ZERO,
            Direction::Forward,
            SpeedFormat::Speed28,
        );
        slot.dirty_speed = false;
        slot.speed_commanded = false;
        if !slot.set_function(function, enabled) {
            return false;
        }
        let _ = self.slots.push(slot);
        true
    }

    /// Request global emergency stop.
    pub fn request_emergency_stop_all(&mut self) {
        for slot in self.slots.iter_mut() {
            slot.speed = LogicalSpeed::ZERO;
            slot.dirty_speed = true;
        }
        self.pending_broadcast_estop = true;
        self.pending_estop_targets.clear();
    }

    /// Request emergency stop for a single locomotive.
    #[must_use]
    pub fn request_emergency_stop(&mut self, address: DccAddress) -> bool {
        let mut found = false;
        for slot in self.slots.iter_mut() {
            if slot.address == address {
                slot.speed = LogicalSpeed::ZERO;
                slot.dirty_speed = true;
                slot.dirty_speed_retries = DIRTY_STOP_RETRY_COUNT;
                found = true;
                break;
            }
        }

        if !found {
            return false;
        }

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

    /// Backward-compatible alias for [`Self::request_emergency_stop_all`].
    pub fn emergency_stop_all(&mut self) {
        self.request_emergency_stop_all();
    }

    /// Backward-compatible alias for [`Self::request_emergency_stop`].
    #[must_use]
    pub fn emergency_stop(&mut self, address: DccAddress) -> bool {
        self.request_emergency_stop(address)
    }

    /// Create an empty consist with given ID.
    #[must_use]
    pub fn create_consist(&mut self, id: u8) -> bool {
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
        id: u8,
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
    pub fn remove_from_consist(&mut self, id: u8, address: DccAddress) -> bool {
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
        id: u8,
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
        if let Some(idx) = self.slots.iter().position(|s| s.address == address) {
            self.slots.swap_remove(idx);
            if self.next_index >= self.slots.len() {
                self.next_index = 0;
            }
            true
        } else {
            false
        }
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
    #[must_use]
    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// Queue one explicit POM packet for single transmission.
    #[must_use]
    pub fn program_on_main(&mut self, packet: DccPacket) -> bool {
        self.pending_pom.push(packet).is_ok()
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
        if self.slots.is_full() {
            return false;
        }

        let mut slot = Slot::new(address, LogicalSpeed::ZERO, Direction::Forward, format);
        slot.dirty_speed = false;
        slot.dirty_speed_retries = 0;
        let _ = self.slots.push(slot);
        true
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
    pub fn build_next_packet(&mut self) -> Option<DccPacket> {
        self.build_next_packet_with_function_budget(allowed_slot_visits_without_function(
            self.slots.len(),
        ))
    }

    /// Build next packet with an explicit function-refresh budget in slot visits.
    pub fn build_next_packet_with_function_budget(
        &mut self,
        max_slot_visits_without_function: u8,
    ) -> Option<DccPacket> {
        self.build_next_packet_classified_with_function_budget(max_slot_visits_without_function)
            .map(|(packet, _)| packet)
    }

    pub(super) fn build_next_packet_classified_with_function_budget(
        &mut self,
        max_slot_visits_without_function: u8,
    ) -> Option<(DccPacket, PacketClass)> {
        if self.paused {
            return None;
        }

        if self.pending_broadcast_estop {
            self.pending_broadcast_estop = false;
            return Some((DccPacket::BroadcastStop, PacketClass::Safety));
        }
        if !self.pending_estop_targets.is_empty() {
            let address = self.pending_estop_targets.remove(0);
            let direction = self
                .slots
                .iter()
                .find(|s| s.address == address)
                .map(|s| s.direction)
                .unwrap_or(Direction::Reverse);
            return Some((
                DccPacket::EmergencyStop { address, direction },
                PacketClass::Safety,
            ));
        }
        if !self.pending_pom.is_empty() {
            let packet = self.pending_pom.remove(0);
            return Some((packet, PacketClass::Programming));
        }

        if self.slots.is_empty() {
            if let Some(packet) = self.pending_railcom_telemetry.take() {
                return Some((packet, PacketClass::Telemetry));
            }
            return None;
        }

        for slot in self.slots.iter_mut() {
            if slot.dirty_speed {
                if slot.dirty_speed_retries == 0 {
                    slot.dirty_speed = false;
                } else {
                    slot.dirty_speed_retries -= 1;
                }
                let packet = slot.speed_packet();
                slot.last_sent = slot.last_sent.wrapping_add(1);
                if let Some(packet) = packet {
                    return Some((packet, PacketClass::Command));
                }
                #[cfg(target_arch = "riscv32")]
                defmt::warn!(
                    "invalid speed/format state for addr={}, dropping dirty speed",
                    slot.address.value()
                );
            }
        }

        for slot in self.slots.iter_mut() {
            if slot.dirty_function_groups != 0 {
                let packet = slot.next_dirty_function_packet();
                slot.last_sent = slot.last_sent.wrapping_add(1);
                return packet.map(|packet| (packet, PacketClass::Command));
            }
        }

        if let Some(packet) = self.pending_railcom_telemetry.take() {
            return Some((packet, PacketClass::Telemetry));
        }

        if self.next_index >= self.slots.len() {
            self.next_index = 0;
        }

        let packet = {
            let slot = &mut self.slots[self.next_index];
            let p = slot.next_refresh_packet_with_budget(max_slot_visits_without_function);
            slot.last_sent = slot.last_sent.wrapping_add(1);
            if p.is_none() {
                #[cfg(target_arch = "riscv32")]
                defmt::warn!(
                    "invalid refresh packet for addr={}, skipping",
                    slot.address.value()
                );
            }
            p
        };

        self.next_index = (self.next_index + 1) % self.slots.len();
        packet.map(|packet| (packet, PacketClass::Refresh))
    }

    /// Number of active slots.
    #[must_use]
    pub fn slot_count(&self) -> usize {
        self.slots.len()
    }

    /// Returns `true` if no slots are active.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }

    /// Apply a scheduler command.
    #[must_use]
    pub fn apply_command(&mut self, command: SchedulerCommand) -> bool {
        match command {
            SchedulerCommand::SetSpeed {
                address,
                speed,
                direction,
                format,
            } => self.set_speed_with_format(address, speed, direction, format),
            SchedulerCommand::SetFunction {
                address,
                function,
                enabled,
            } => self.set_function_indexed(address, function, enabled),
            SchedulerCommand::EmergencyStopAll => {
                self.request_emergency_stop_all();
                true
            }
            SchedulerCommand::EmergencyStop { address } => self.request_emergency_stop(address),
            SchedulerCommand::ProgramOnMain { packet } => self.program_on_main(packet),
            SchedulerCommand::SuspendRailcomDiscovery => true,
            SchedulerCommand::EnsureRailcomRefresh { address, format } => {
                self.ensure_railcom_refresh_slot(address, format)
            }
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
