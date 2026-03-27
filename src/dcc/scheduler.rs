//! DCC Packet Scheduler with Cyclic Refresh
//!
//! Manages active locomotive slots and retransmits commands cyclically to keep
//! decoders updated.
//!
//! # Overview
//!
//! The scheduler runs as an async Embassy actor task. It accepts `SchedulerCommand` messages
//! (SetSpeed, SetFunction, EmergencyStop, etc.) via a channel and generates a continuous
//! stream of DCC packets to refresh decoder state. Scheduling policy prioritizes:
//! - **Priority 1**: Dirty speed packets (just changed)
//! - **Priority 2**: Dirty function packets (just changed)
//! - **Priority 3**: Cyclic refresh (ensure all active locos get periodic updates)
//!
//! The scheduler supports:
//! - Up to 12 active locomotive slots
//! - Consist (multi-unit train) management
//! - Pause/Resume coordination with CV programming (main track stops during programming)
//!
//! # Examples
//!
//! **Send a speed command (simplified host example):**
//!
//! ```no_run
//! use dcc_esp32::dcc::{DccAddress, Direction, SpeedFormat, SchedulerCommand};
//!
//! // In real code, this would be sent via the SchedulerCommandChannel
//! let cmd = SchedulerCommand::SetSpeed {
//!     address: DccAddress::short(42).unwrap(),
//!     speed: 75,
//!     direction: Direction::Forward,
//!     format: SpeedFormat::Speed128,
//! };
//! // sender.send(cmd).await;
//! ```
//!
//! **Control a function (headlight or horn):**
//!
//! ```no_run
//! use dcc_esp32::dcc::{DccAddress, FunctionIndex, SchedulerCommand};
//!
//! let cmd = SchedulerCommand::SetFunction {
//!     address: DccAddress::short(10).unwrap(),
//!     function: FunctionIndex::new(0).unwrap(), // F0 = headlight
//!     enabled: true,
//! };
//! // sender.send(cmd).await;
//! ```
//!
//! **Emergency stop all locomotives:**
//!
//! ```no_run
//! use dcc_esp32::dcc::SchedulerCommand;
//!
//! let cmd = SchedulerCommand::EmergencyStopAll;
//! // sender.send(cmd).await;
//! ```
//!
//! **Create and control a consist (multi-unit train):**
//!
//! ```no_run
//! use dcc_esp32::dcc::{DccAddress, Direction, SchedulerCommand};
//!
//! // Create consist #1
//! // sender.send(SchedulerCommand::CreateConsist { id: 1 }).await;
//!
//! // Add locomotive address 50 as lead (normal orientation)
//! // sender.send(SchedulerCommand::AddToConsist {
//! //     id: 1,
//! //     address: DccAddress::short(50).unwrap(),
//! //     reverse_in_consist: false,
//! // }).await;
//!
//! // Add locomotive address 51 as helper (reversed in consist)
//! // sender.send(SchedulerCommand::AddToConsist {
//! //     id: 1,
//! //     address: DccAddress::short(51).unwrap(),
//! //     reverse_in_consist: true,
//! // }).await;
//!
//! // Control the whole consist (all members move together)
//! // sender.send(SchedulerCommand::SetConsistSpeed {
//! //     id: 1,
//! //     speed: 100,
//! //     direction: Direction::Forward,
//! // }).await;
//! ```
//!
//! # Function Index
//!
//! Functions are controlled via `FunctionIndex`, which validates the index 0-28:
//! - 0 = FL (headlight)
//! - 1-4 = F1-F4 (transmitted in Function Group 1)
//! - 5-8 = F5-F8 (Function Group 2A)
//! - 9-12 = F9-F12 (Function Group 2B)
//! - 13-20 = F13-F20 (Function Group 3)
//! - 21-28 = F21-F28 (Function Group 4)

use crate::dcc::packet::{DccAddress, DccPacket, Direction};
use crate::dcc::speed28::logical_to_nmra_packet_speed;
use heapless::Vec;

/// Maximum number of active locomotive slots
const MAX_SLOTS: usize = 12;
/// Maximum number of consists managed in software
const MAX_CONSISTS: usize = 8;
/// Maximum members per consist
const MAX_CONSIST_MEMBERS: usize = 8;
/// Scheduler target period to revisit one slot in normal conditions.
const TARGET_SLOT_PERIOD_MS: u64 = 120;
/// Minimum scheduler tick to avoid tight loops under high slot count.
const MIN_TICK_MS: u64 = 5;
/// Maximum allowed interval between refreshes of active function groups.
const MAX_FUNCTION_REFRESH_MS: u64 = 400;
/// Additional immediate retransmissions after a function state change.
const DIRTY_FUNCTION_RETRY_COUNT: u8 = 6;
/// Additional immediate retransmissions after a stop command.
const DIRTY_STOP_RETRY_COUNT: u8 = 4;

/// Speed format for a locomotive slot
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum SpeedFormat {
    Speed28,
    Speed128,
}

/// Validated function index (F0..F28).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FunctionIndex(u8);

/// Error returned when function index is out of range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidFunctionIndex;

impl FunctionIndex {
    /// Create a validated function index in range 0..=28.
    #[must_use]
    pub fn new(value: u8) -> Option<Self> {
        (value <= MAX_FUNCTION_INDEX).then_some(Self(value))
    }

    /// Return the raw function index value.
    #[must_use]
    pub const fn get(self) -> u8 {
        self.0
    }
}

impl TryFrom<u8> for FunctionIndex {
    type Error = InvalidFunctionIndex;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        FunctionIndex::new(value).ok_or(InvalidFunctionIndex)
    }
}

/// Command messages sent to the packet scheduler actor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerCommand {
    SetSpeed {
        address: DccAddress,
        speed: u8,
        direction: Direction,
        format: SpeedFormat,
    },
    SetFunction {
        address: DccAddress,
        function: FunctionIndex,
        enabled: bool,
    },
    EmergencyStopAll,
    EmergencyStop {
        address: DccAddress,
    },
    CreateConsist {
        id: u8,
    },
    AddToConsist {
        id: u8,
        address: DccAddress,
        reverse_in_consist: bool,
    },
    RemoveFromConsist {
        id: u8,
        address: DccAddress,
    },
    SetConsistSpeed {
        id: u8,
        speed: u8,
        direction: Direction,
    },
    RemoveSlot {
        address: DccAddress,
    },
    /// Pause packet emission (stop total, preserve state)
    /// Used by programming track service to halt main track output
    Pause,
    /// Resume packet emission (dirty packets first, then refresh)
    /// Used by programming track service to restart main track after CV operations
    Resume,
}

/// Function index mapping:
/// - 0 => FL
/// - 1..=28 => F1..F28
const MAX_FUNCTION_INDEX: u8 = 28;

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
    speed: u8,
    direction: Direction,
    format: SpeedFormat,
    // Bit 0 = FL, bits 1..28 = F1..F28
    functions: u32,
    dirty_speed: bool,
    dirty_speed_retries: u8,
    // True once a SetLocoDrive command has been received for this slot.
    // Speed packets are only included in the cyclic refresh when this is set —
    // a function-only slot must not flood the decoder with unsolicited Speed(0).
    speed_commanded: bool,
    // Dirty function groups bitmask: bit0=FG1, bit1=FG2A, bit2=FG2B, bit3=FG3, bit4=FG4
    dirty_function_groups: u8,
    // Remaining immediate retries for each dirty function group. Each group uses
    // one 3-bit lane inside this u16 (groups 0..4).
    dirty_function_retries: u16,
    // Tracks which function groups have ever been explicitly addressed (never cleared).
    // Ensures groups are still refreshed after all functions in them are turned off.
    known_groups: u8,
    // Tracks whether next refresh should prefer speed packet (unless function deadline requires function).
    refresh_speed_next: bool,
    // Counts slot refresh turns since the last function refresh packet.
    refresh_turns_since_function: u8,
    // Round-robin cursor for function groups (0..2)
    next_function_group: u8,
    // Monotonic send sequence number of the last emitted packet for this slot.
    // Used for observability and future scheduling policies.
    last_sent: u32,
    #[cfg(test)]
    // Test-only counter to prove deadline-enforced refresh path is exercised.
    deadline_enforced_refreshes: u32,
}

impl Slot {
    fn new(address: DccAddress, speed: u8, direction: Direction, format: SpeedFormat) -> Self {
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

        // Mark only the affected function group dirty to avoid unnecessary burst traffic.
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
                logical_to_nmra_packet_speed(self.speed)?,
                self.direction,
            ),
            SpeedFormat::Speed128 => {
                DccPacket::speed_128step(self.address, self.speed, self.direction)
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

        // Serve one dirty group at a time starting from cursor for fairness.
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
        // No speed command was ever sent: refresh function groups only.
        // Sending Speed(0) for a function-only slot would be unsolicited and
        // could confuse decoders that reset function state on speed packets.
        if !self.speed_commanded {
            if !self.has_any_functions() {
                return None;
            }
            return self.next_active_function_packet().or_else(|| {
                unreachable!("function-only slot has no active function group packet")
            });
        }

        // If no functions are active, refresh speed only.
        if !self.has_any_functions() {
            return self.speed_packet();
        }

        let must_send_function =
            self.refresh_turns_since_function + 1 >= max_slot_visits_without_function.max(1);

        // Enforce a bounded function refresh interval under load.
        if must_send_function && let Some(packet) = self.next_active_function_packet() {
            self.refresh_turns_since_function = 0;
            self.refresh_speed_next = true;
            #[cfg(test)]
            {
                self.deadline_enforced_refreshes = self.deadline_enforced_refreshes.wrapping_add(1);
            }
            return Some(packet);
        }

        // Nominal refresh pattern: speed/function alternation.
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
        // If no active groups, fallback to speed-only refresh.
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
        // Group 0: FL/F1..F4 => bits 0..=4
        if (self.functions & 0b0000_0000_0000_0000_0000_0000_0001_1111) != 0 {
            mask |= 1 << 0;
        }
        // Group 1: F5..F8 => bits 5..=8
        if (self.functions & 0b0000_0000_0000_0000_0000_0001_1110_0000) != 0 {
            mask |= 1 << 1;
        }
        // Group 2: F9..F12 => bits 9..=12
        if (self.functions & 0b0000_0000_0000_0000_0001_1110_0000_0000) != 0 {
            mask |= 1 << 2;
        }
        // Group 3: F13..F20 => bits 13..=20
        if (self.functions & 0b0000_0000_0001_1111_1110_0000_0000_0000) != 0 {
            mask |= 1 << 3;
        }
        // Group 4: F21..F28 => bits 21..=28
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

/// Manages active locomotive slots with round-robin scheduling
pub struct SlotManager {
    slots: Vec<Slot, MAX_SLOTS>,
    consists: Vec<Consist, MAX_CONSISTS>,
    pending_broadcast_estop: bool,
    pending_estop_targets: Vec<DccAddress, MAX_SLOTS>,
    next_index: usize,
    paused: bool,
}

impl Default for SlotManager {
    fn default() -> Self {
        Self::new()
    }
}

impl SlotManager {
    /// Create a new empty SlotManager
    pub fn new() -> Self {
        Self {
            slots: Vec::new(),
            consists: Vec::new(),
            pending_broadcast_estop: false,
            pending_estop_targets: Vec::new(),
            next_index: 0,
            paused: false,
        }
    }

    /// Set speed for a locomotive using Speed28 format.
    ///
    /// Creates a new slot if address is not present.
    /// Returns `false` if speed is out of range or the slot table is full.
    #[must_use]
    pub fn set_speed(&mut self, address: DccAddress, speed: u8, direction: Direction) -> bool {
        self.set_speed_with_format(address, speed, direction, SpeedFormat::Speed28)
    }

    /// Set speed with explicit format selection.
    ///
    /// Returns `false` if speed is out of range for selected format, or if
    /// the slot table is full and the address is new.
    #[must_use]
    pub fn set_speed_with_format(
        &mut self,
        address: DccAddress,
        speed: u8,
        direction: Direction,
        format: SpeedFormat,
    ) -> bool {
        if !speed_valid_for_format(speed, format) {
            return false;
        }

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
    ///
    /// Creates a default slot if needed.
    /// Returns `false` when function index is invalid or slot table is full.
    #[must_use]
    pub fn set_function(&mut self, address: DccAddress, function: u8, enabled: bool) -> bool {
        let Ok(function) = FunctionIndex::try_from(function) else {
            return false;
        };
        self.set_function_indexed(address, function, enabled)
    }

    /// Set function state for a locomotive using a validated function index.
    ///
    /// Creates a default slot if needed.
    /// Returns `false` if the slot table is full.
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

        let mut slot = Slot::new(address, 0, Direction::Forward, SpeedFormat::Speed28);
        // Slot created by a function-only command: no speed has been commanded.
        // Don't send Speed(0) now (dirty) or in the periodic refresh until the
        // user explicitly sets a speed via SetLocoDrive.
        slot.dirty_speed = false;
        slot.speed_commanded = false;
        if !slot.set_function(function, enabled) {
            return false;
        }
        let _ = self.slots.push(slot);
        true
    }

    /// Request global emergency stop.
    /// This has highest scheduling priority and emits `BroadcastStop`.
    pub fn request_emergency_stop_all(&mut self) {
        for slot in self.slots.iter_mut() {
            slot.speed = 0;
            slot.dirty_speed = true;
        }
        self.pending_broadcast_estop = true;
        self.pending_estop_targets.clear();
    }

    /// Request emergency stop for a single locomotive.
    /// This has highest scheduling priority after pending global e-stop.
    ///
    /// Returns `false` if target address is not currently active or if the
    /// pending e-stop queue is full.
    #[must_use]
    pub fn request_emergency_stop(&mut self, address: DccAddress) -> bool {
        let mut found = false;
        for slot in self.slots.iter_mut() {
            if slot.address == address {
                slot.speed = 0;
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
    ///
    /// Idempotent: returns `true` if consist already exists.
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
    /// Returns number of members updated.
    #[must_use]
    pub fn set_consist_speed(&mut self, id: u8, speed: u8, direction: Direction) -> usize {
        let Some(consist) = self.consists.iter().find(|c| c.id == id) else {
            return 0;
        };

        // Copy members to avoid borrow conflicts when mutating slots.
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

    /// Remove a locomotive slot. Returns `true` if found and removed.
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
    ///
    /// While paused:
    /// - `build_next_packet()` returns `None`
    /// - Commands like `set_speed()` still update slot state and mark dirty
    /// - Packets are NOT emitted until `resume()` is called
    ///
    /// Used by programming track service to stop main track output during CV operations.
    pub fn pause(&mut self) {
        self.paused = true;
    }

    /// Resume packet emission after pause.
    ///
    /// Dirty packets (speed/functions modified during pause) are emitted first,
    /// then normal cyclic refresh resumes.
    pub fn resume(&mut self) {
        self.paused = false;
    }

    /// Check if scheduler is currently paused.
    #[must_use]
    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// Build the next packet to transmit.
    ///
    /// Priority: dirty speed, dirty functions, then round-robin refresh.
    /// Returns `None` if no slots are active.
    pub fn build_next_packet(&mut self) -> Option<DccPacket> {
        self.build_next_packet_with_function_budget(allowed_slot_visits_without_function(
            self.slots.len(),
        ))
    }

    /// Build next packet with an explicit function-refresh budget in slot visits.
    ///
    /// `max_slot_visits_without_function` defines the upper bound between function refreshes
    /// for slots that have active function groups.
    pub fn build_next_packet_with_function_budget(
        &mut self,
        max_slot_visits_without_function: u8,
    ) -> Option<DccPacket> {
        // If paused, return None (no output) but preserve all slot state
        if self.paused {
            return None;
        }

        // Priority 0: requested emergency stop packets.
        if self.pending_broadcast_estop {
            self.pending_broadcast_estop = false;
            return Some(DccPacket::BroadcastStop);
        }
        if !self.pending_estop_targets.is_empty() {
            let address = self.pending_estop_targets.remove(0);
            let direction = self
                .slots
                .iter()
                .find(|s| s.address == address)
                .map(|s| s.direction)
                .unwrap_or(Direction::Reverse);
            return Some(DccPacket::EmergencyStop { address, direction });
        }

        if self.slots.is_empty() {
            return None;
        }

        // Priority 1: serve dirty speed first.
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
                    return Some(packet);
                }
                #[cfg(target_arch = "riscv32")]
                defmt::warn!(
                    "invalid speed/format state for addr={}, dropping dirty speed",
                    slot.address.value()
                );
            }
        }

        // Priority 2: serve dirty function groups.
        for slot in self.slots.iter_mut() {
            if slot.dirty_function_groups != 0 {
                let packet = slot.next_dirty_function_packet();
                slot.last_sent = slot.last_sent.wrapping_add(1);
                return packet;
            }
        }

        // Priority 3: cyclic refresh with round-robin fairness.
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
        packet
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
}

fn speed_valid_for_format(speed: u8, format: SpeedFormat) -> bool {
    match format {
        SpeedFormat::Speed28 => speed <= 28,
        SpeedFormat::Speed128 => speed <= 126,
    }
}

const fn speed_retry_count(speed: u8) -> u8 {
    if speed == 0 {
        DIRTY_STOP_RETRY_COUNT
    } else {
        0
    }
}

fn scheduler_tick_ms_for_slot_count(slot_count: usize) -> u64 {
    (TARGET_SLOT_PERIOD_MS / slot_count.max(1) as u64).max(MIN_TICK_MS)
}

fn allowed_slot_visits_without_function(slot_count: usize) -> u8 {
    let slot_period_ms = scheduler_tick_ms_for_slot_count(slot_count) * slot_count.max(1) as u64;
    let visits = (MAX_FUNCTION_REFRESH_MS / slot_period_ms).max(1);
    visits.min(u8::MAX as u64) as u8
}

fn function_group(function: u8) -> u8 {
    match function {
        0..=4 => 0,   // FG1: FL/F1-F4
        5..=8 => 1,   // FG2A: F5-F8
        9..=12 => 2,  // FG2B: F9-F12
        13..=20 => 3, // FG3: F13-F20
        _ => 4,       // FG4: F21-F28
    }
}

#[cfg(test)]
fn is_safety_critical_packet(packet: &DccPacket) -> bool {
    matches!(
        packet,
        DccPacket::BroadcastStop | DccPacket::EmergencyStop { .. }
    )
}

#[cfg(test)]
fn advance_safety_send_timeout_streak(streak: u8, timed_out: bool) -> u8 {
    if timed_out {
        streak.saturating_add(1)
    } else {
        0
    }
}

#[cfg(target_arch = "riscv32")]
fn emit_status_event_for_command(
    _command: SchedulerCommand,
    _accepted: bool,
    _status_sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        crate::system_status::SystemStatusEvent,
        16,
    >,
) {
    // No-op for now. Pause/Resume status is driven by the fault manager.
    // The status path now uses a queued Channel, so coalescing is no longer
    // a concern. Scheduler-originated Pause/Resume events can be added here
    // in the future if needed.
}

// --- Embedded-only scheduler task ---
#[cfg(target_arch = "riscv32")]
use embassy_futures::yield_now;

#[cfg(target_arch = "riscv32")]
pub type SchedulerCommandChannel = embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    SchedulerCommand,
    32,
>;

#[cfg(target_arch = "riscv32")]
pub async fn packet_scheduler_task(
    command_receiver: embassy_sync::channel::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        SchedulerCommand,
        32,
    >,
    sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        DccPacket,
        16,
    >,
    status_sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        crate::system_status::SystemStatusEvent,
        16,
    >,
    display_sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        crate::display::DisplayEvent,
        8,
    >,
) -> ! {
    let mut slot_manager = SlotManager::new();
    let mut prev_slot_count: usize = 0;

    loop {
        // Drain all pending commands before generating the next packet.
        // Commands are processed with at most one packet interval (~8 ms) of latency.
        while let Ok(cmd) = command_receiver.try_receive() {
            let accepted = slot_manager.apply_command(cmd);
            if !accepted {
                defmt::warn!("scheduler command rejected");
            }
            emit_status_event_for_command(cmd, accepted, status_sender);
        }

        let slot_count = slot_manager.slot_count();
        if slot_count != prev_slot_count {
            prev_slot_count = slot_count;
            let _ = display_sender.try_send(crate::display::DisplayEvent::ActiveLocoCount(
                slot_count as u8,
            ));
        }
        let to_send = slot_manager
            .build_next_packet_with_function_budget(allowed_slot_visits_without_function(
                slot_count.max(1),
            ))
            .unwrap_or(DccPacket::Idle);

        // send().await blocks until engine drains the channel — natural backpressure.
        // This paces the scheduler exactly to the engine's transmission rate (~8 ms/packet),
        // eliminating the old 120 ms tick that starved the channel with idle packets.
        sender.send(to_send).await;

        yield_now().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn addr(n: u8) -> DccAddress {
        DccAddress::new_short(n).unwrap()
    }

    #[test]
    fn test_new_slot_defaults() {
        let mut mgr = SlotManager::new();
        assert!(mgr.is_empty());
        assert_eq!(mgr.slot_count(), 0);

        let _ = mgr.set_speed(addr(3), 0, Direction::Forward);
        assert_eq!(mgr.slot_count(), 1);
    }

    #[test]
    fn test_set_speed_creates_or_updates() {
        let mut mgr = SlotManager::new();

        assert!(mgr.set_speed(addr(3), 10, Direction::Forward));
        assert_eq!(mgr.slot_count(), 1);

        assert!(mgr.set_speed(addr(3), 20, Direction::Reverse));
        assert_eq!(mgr.slot_count(), 1);

        assert!(mgr.set_speed(addr(5), 15, Direction::Forward));
        assert_eq!(mgr.slot_count(), 2);
    }

    #[test]
    fn test_set_speed_rejects_out_of_range() {
        let mut mgr = SlotManager::new();
        assert!(!mgr.set_speed_with_format(addr(3), 30, Direction::Forward, SpeedFormat::Speed28));
        assert!(!mgr.set_speed_with_format(
            addr(3),
            128,
            Direction::Forward,
            SpeedFormat::Speed128
        ));
        assert_eq!(mgr.slot_count(), 0);
    }

    #[test]
    fn test_speed28_step_one_maps_to_nmra_step_one_not_estop() {
        let mut mgr = SlotManager::new();
        assert!(mgr.set_speed_with_format(addr(3), 1, Direction::Forward, SpeedFormat::Speed28));

        let packet = mgr.build_next_packet().expect("expected speed packet");
        let DccPacket::Speed28 {
            address,
            direction,
            speed,
        } = packet
        else {
            panic!("expected Speed28, got {packet:?}");
        };

        assert_eq!(address, addr(3));
        assert_eq!(direction, Direction::Forward);
        assert_eq!(
            speed.value(),
            2,
            "logical step 1 must map to NMRA packet step 1"
        );
    }

    #[test]
    fn test_speed28_logical_max_maps_to_nmra_max() {
        let mut mgr = SlotManager::new();
        assert!(mgr.set_speed_with_format(addr(3), 28, Direction::Forward, SpeedFormat::Speed28));

        let packet = mgr.build_next_packet().expect("expected speed packet");
        let DccPacket::Speed28 { speed, .. } = packet else {
            panic!("expected Speed28, got {packet:?}");
        };

        assert_eq!(
            speed.value(),
            29,
            "logical step 28 must map to NMRA packet max step"
        );
    }

    #[test]
    fn test_is_safety_critical_packet_detection() {
        assert!(is_safety_critical_packet(&DccPacket::BroadcastStop));
        assert!(is_safety_critical_packet(&DccPacket::EmergencyStop {
            address: addr(3),
            direction: Direction::Forward,
        }));
        assert!(!is_safety_critical_packet(&DccPacket::Idle));
    }

    #[test]
    fn test_safety_send_timeout_streak_transitions() {
        let mut streak = 0;
        streak = advance_safety_send_timeout_streak(streak, true);
        assert_eq!(streak, 1);
        streak = advance_safety_send_timeout_streak(streak, true);
        assert_eq!(streak, 2);
        streak = advance_safety_send_timeout_streak(streak, false);
        assert_eq!(streak, 0);
    }

    #[test]
    fn test_capacity_limit() {
        let mut mgr = SlotManager::new();
        for i in 1..=12 {
            assert!(mgr.set_speed(addr(i), 0, Direction::Forward));
        }
        assert_eq!(mgr.slot_count(), 12);

        assert!(!mgr.set_speed(addr(13), 0, Direction::Forward));
        assert_eq!(mgr.slot_count(), 12);
    }

    #[test]
    fn test_emergency_stop_all() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(1), 20, Direction::Forward);
        let _ = mgr.set_speed(addr(2), 15, Direction::Reverse);

        mgr.build_next_packet();
        mgr.build_next_packet();

        mgr.emergency_stop_all();

        // Global e-stop must preempt queue and emit BroadcastStop first.
        let p0 = mgr.build_next_packet().unwrap();
        assert!(matches!(p0, DccPacket::BroadcastStop));

        let p1 = mgr.build_next_packet().unwrap();
        let p2 = mgr.build_next_packet().unwrap();

        let DccPacket::Speed28 { speed, .. } = p1 else {
            panic!("expected Speed28, got {p1:?}");
        };
        assert_eq!(speed.value(), 0);

        let DccPacket::Speed28 { speed, .. } = p2 else {
            panic!("expected Speed28, got {p2:?}");
        };
        assert_eq!(speed.value(), 0);
    }

    #[test]
    fn test_emergency_stop_single() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(1), 20, Direction::Forward);
        let _ = mgr.set_speed(addr(2), 15, Direction::Forward);

        mgr.build_next_packet();
        mgr.build_next_packet();

        assert!(mgr.emergency_stop(addr(1)));

        let p = mgr.build_next_packet().unwrap();
        let DccPacket::EmergencyStop { address, .. } = p else {
            panic!("expected EmergencyStop, got {p:?}");
        };
        assert_eq!(address, addr(1));
    }

    #[test]
    fn test_build_next_packet_empty() {
        let mut mgr = SlotManager::new();
        assert!(mgr.build_next_packet().is_none());
    }

    #[test]
    fn test_dirty_priority() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(1), 10, Direction::Forward);
        let _ = mgr.set_speed(addr(2), 20, Direction::Forward);

        for _ in 0..2 {
            let _ = mgr.build_next_packet();
        }

        let _ = mgr.set_speed(addr(2), 25, Direction::Forward);

        let p = mgr.build_next_packet().unwrap();
        let DccPacket::Speed28 { address, speed, .. } = p else {
            panic!("expected Speed28, got {p:?}");
        };
        assert_eq!(address, addr(2));
        assert_eq!(speed.value(), 26);
    }

    #[test]
    fn test_round_robin_fairness() {
        let mut mgr = SlotManager::new();
        let n = 4;
        for i in 1..=(n as u8) {
            let _ = mgr.set_speed(addr(i), i * 5, Direction::Forward);
        }

        for _ in 0..n {
            let _ = mgr.build_next_packet();
        }

        let mut seen = [0u8; 4];
        for _ in 0..n {
            let p = mgr.build_next_packet().unwrap();
            let DccPacket::Speed28 { address, .. } = p else {
                panic!("expected Speed28, got {p:?}");
            };
            let idx = (address.value() - 1) as usize;
            seen[idx] += 1;
        }

        for count in &seen {
            assert_eq!(
                *count, 1,
                "each slot should be visited exactly once per cycle"
            );
        }
    }

    #[test]
    fn test_remove_slot() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(1), 10, Direction::Forward);
        let _ = mgr.set_speed(addr(2), 20, Direction::Forward);

        assert!(mgr.remove_slot(addr(1)));
        assert_eq!(mgr.slot_count(), 1);
        assert!(!mgr.remove_slot(addr(1)));

        mgr.build_next_packet();
        let p = mgr.build_next_packet().unwrap();
        let DccPacket::Speed28 { address, .. } = p else {
            panic!("expected Speed28, got {p:?}");
        };
        assert_eq!(address, addr(2));
    }

    #[test]
    fn test_speed128_format() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed_with_format(addr(3), 100, Direction::Forward, SpeedFormat::Speed128);

        let p = mgr.build_next_packet().unwrap();
        let DccPacket::Speed128 {
            address,
            speed,
            direction,
        } = p
        else {
            panic!("expected Speed128, got {p:?}");
        };
        assert_eq!(address, addr(3));
        assert_eq!(speed.value(), 100);
        assert_eq!(direction, Direction::Forward);
    }

    #[test]
    fn test_dirty_cleared_after_build() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(1), 10, Direction::Forward);

        let p1 = mgr.build_next_packet().unwrap();
        let DccPacket::Speed28 { address, .. } = p1 else {
            panic!("expected Speed28, got {p1:?}");
        };
        assert_eq!(address, addr(1));

        let p2 = mgr.build_next_packet().unwrap();
        let DccPacket::Speed28 { address, .. } = p2 else {
            panic!("expected Speed28, got {p2:?}");
        };
        assert_eq!(address, addr(1));

        let _ = mgr.set_speed(addr(1), 20, Direction::Reverse);
        let p3 = mgr.build_next_packet().unwrap();
        let DccPacket::Speed28 {
            speed, direction, ..
        } = p3
        else {
            panic!("expected Speed28, got {p3:?}");
        };
        assert_eq!(speed.value(), 21);
        assert_eq!(direction, Direction::Reverse);
    }

    #[test]
    fn test_set_function_generates_function_packets() {
        let mut mgr = SlotManager::new();
        assert!(mgr.set_function(addr(3), 0, true)); // FL
        assert!(mgr.set_function(addr(3), 5, true)); // F5

        // Speed is dirty first (slot default).
        let _ = mgr.build_next_packet().unwrap();

        // Then function dirty groups should be emitted.
        let p = mgr.build_next_packet().unwrap();
        assert!(matches!(
            p,
            DccPacket::FunctionGroup1 { .. }
                | DccPacket::FunctionGroup2A { .. }
                | DccPacket::FunctionGroup2B { .. }
                | DccPacket::FunctionGroup3 { .. }
                | DccPacket::FunctionGroup4 { .. }
        ));
    }

    #[test]
    fn test_function_refresh_fairness_speed_then_function() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(1), 20, Direction::Forward);
        let _ = mgr.set_function(addr(1), 1, true);

        // Drain dirty speed + dirty function groups.
        for _ in 0..(1 + usize::from(DIRTY_FUNCTION_RETRY_COUNT)) {
            let _ = mgr.build_next_packet();
        }

        // Refresh should alternate speed/function for slot with active functions.
        let p1 = mgr.build_next_packet().unwrap();
        let p2 = mgr.build_next_packet().unwrap();
        assert!(matches!(
            p1,
            DccPacket::Speed28 { .. } | DccPacket::Speed128 { .. }
        ));
        assert!(matches!(
            p2,
            DccPacket::FunctionGroup1 { .. }
                | DccPacket::FunctionGroup2A { .. }
                | DccPacket::FunctionGroup2B { .. }
                | DccPacket::FunctionGroup3 { .. }
                | DccPacket::FunctionGroup4 { .. }
        ));
    }

    #[test]
    fn test_consist_base_direction_mapping() {
        let mut mgr = SlotManager::new();
        assert!(mgr.create_consist(1));
        assert!(mgr.add_to_consist(1, addr(3), false));
        assert!(mgr.add_to_consist(1, addr(4), true));

        let updated = mgr.set_consist_speed(1, 18, Direction::Forward);
        assert_eq!(updated, 2);

        let mut seen_fwd = false;
        let mut seen_rev = false;
        for _ in 0..2 {
            let p = mgr.build_next_packet().unwrap();
            let DccPacket::Speed28 {
                address,
                direction,
                speed,
            } = p
            else {
                panic!("expected Speed28, got {p:?}");
            };
            assert_eq!(speed.value(), 19);
            if address == addr(3) && direction == Direction::Forward {
                seen_fwd = true;
            }
            if address == addr(4) && direction == Direction::Reverse {
                seen_rev = true;
            }
        }

        assert!(
            seen_fwd && seen_rev,
            "consist direction mapping should be applied"
        );
    }

    #[test]
    fn test_emergency_stop_preempts_dirty_queue() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(1), 20, Direction::Forward);
        let _ = mgr.set_speed(addr(2), 15, Direction::Forward);
        let _ = mgr.set_function(addr(1), 1, true);

        // Queue a global e-stop after normal dirty traffic exists.
        mgr.request_emergency_stop_all();

        // Must preempt dirty queue.
        let p = mgr.build_next_packet().unwrap();
        assert!(matches!(p, DccPacket::BroadcastStop));
    }

    #[test]
    fn test_emergency_stop_single_does_not_target_others() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(1), 20, Direction::Forward);
        let _ = mgr.set_speed(addr(2), 15, Direction::Reverse);

        // Drain initial dirty speed packets.
        let _ = mgr.build_next_packet();
        let _ = mgr.build_next_packet();

        assert!(mgr.request_emergency_stop(addr(1)));

        let p = mgr.build_next_packet().unwrap();
        let DccPacket::EmergencyStop { address, .. } = p else {
            panic!("expected EmergencyStop, got {p:?}");
        };
        assert_eq!(address, addr(1));

        // Next packet must not be an e-stop for addr(2).
        let p2 = mgr.build_next_packet().unwrap();
        assert!(!matches!(
            p2,
            DccPacket::EmergencyStop { address, .. } if address == addr(2)
        ));
    }

    #[test]
    fn test_request_emergency_stop_unknown_address_is_rejected() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(1), 20, Direction::Forward);
        assert!(!mgr.request_emergency_stop(addr(2)));
    }

    #[test]
    fn test_recovery_after_emergency_stop() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(1), 20, Direction::Forward);
        let _ = mgr.build_next_packet(); // drain initial dirty

        assert!(mgr.request_emergency_stop(addr(1)));
        let p = mgr.build_next_packet().unwrap();
        assert!(matches!(p, DccPacket::EmergencyStop { .. }));

        // New command after e-stop should be emitted as dirty speed update.
        let _ = mgr.set_speed(addr(1), 12, Direction::Reverse);
        let p2 = mgr.build_next_packet().unwrap();
        let DccPacket::Speed28 {
            address,
            speed,
            direction,
        } = p2
        else {
            panic!("expected Speed28, got {p2:?}");
        };
        assert_eq!(address, addr(1));
        assert_eq!(speed.value(), 13);
        assert_eq!(direction, Direction::Reverse);
    }

    #[test]
    fn test_f13_f28_dirty_and_refresh() {
        let mut mgr = SlotManager::new();
        assert!(mgr.set_function(addr(3), 15, true)); // F15 in FG3
        assert!(mgr.set_function(addr(3), 25, true)); // F25 in FG4

        // No dirty speed when slot is created by function-only command.
        // First two packets should be dirty function groups FG3 and FG4.
        let p1 = mgr.build_next_packet().unwrap();
        let p2 = mgr.build_next_packet().unwrap();

        let mut seen_fg3 = false;
        let mut seen_fg4 = false;
        for p in [p1, p2] {
            match p {
                DccPacket::FunctionGroup3 { address, f15, .. } => {
                    assert_eq!(address, addr(3));
                    assert!(f15);
                    seen_fg3 = true;
                }
                DccPacket::FunctionGroup4 { address, f25, .. } => {
                    assert_eq!(address, addr(3));
                    assert!(f25);
                    seen_fg4 = true;
                }
                _ => {}
            }
        }
        assert!(seen_fg3 && seen_fg4, "Both FG3 and FG4 should be emitted");
    }

    #[test]
    fn test_function_index_validation() {
        assert!(FunctionIndex::new(0).is_some());
        assert!(FunctionIndex::new(28).is_some());
        assert!(FunctionIndex::new(29).is_none());
        assert!(FunctionIndex::try_from(12).is_ok());
        assert!(FunctionIndex::try_from(40).is_err());
    }

    #[test]
    fn test_function_refresh_bound_under_12_slots() {
        let mut mgr = SlotManager::new();
        for i in 1..=12 {
            assert!(mgr.set_speed(addr(i), 10, Direction::Forward));
            assert!(mgr.set_function(addr(i), 13, true));
            assert!(mgr.set_function(addr(i), 21, true));
        }

        // Drain initial dirty speed/function traffic.
        for _ in 0..200 {
            let _ = mgr.build_next_packet();
        }

        let slot_count = 12usize;
        let tick_ms = scheduler_tick_ms_for_slot_count(slot_count);
        // Tight budget to force deadline path under heavy slot pressure.
        let max_visits = 1;
        let mut last_function_packet_idx = [None; 12];

        for packet_idx in 0..1500usize {
            let packet = mgr
                .build_next_packet_with_function_budget(max_visits)
                .unwrap_or(DccPacket::Idle);
            let address = match packet {
                DccPacket::FunctionGroup1 { address, .. }
                | DccPacket::FunctionGroup2A { address, .. }
                | DccPacket::FunctionGroup2B { address, .. }
                | DccPacket::FunctionGroup3 { address, .. }
                | DccPacket::FunctionGroup4 { address, .. } => Some(address),
                _ => None,
            };

            if let Some(address) = address {
                let idx = (address.value() - 1) as usize;
                if let Some(previous) = last_function_packet_idx[idx] {
                    let delta_packets = packet_idx - previous;
                    let gap_ms = (delta_packets as u64) * tick_ms;
                    assert!(
                        gap_ms <= MAX_FUNCTION_REFRESH_MS,
                        "function refresh gap too high for addr {}: {}ms",
                        address.value(),
                        gap_ms
                    );
                }
                last_function_packet_idx[idx] = Some(packet_idx);
            }
        }

        for (idx, seen) in last_function_packet_idx.iter().enumerate() {
            assert!(
                seen.is_some(),
                "slot {} never received function refresh",
                idx + 1
            );
        }

        let deadline_hits: u32 = mgr
            .slots
            .iter()
            .map(|s| s.deadline_enforced_refreshes)
            .sum();
        assert!(
            deadline_hits > 0,
            "deadline enforcement path should be exercised under 12-slot pressure"
        );
    }

    #[test]
    fn test_pause_stops_emission() {
        let mut mgr = SlotManager::new();
        assert!(!mgr.is_paused());

        let _ = mgr.set_speed(addr(3), 10, Direction::Forward);

        // Normal emission works
        assert!(mgr.build_next_packet().is_some());

        // Pause stops emission
        mgr.pause();
        assert!(mgr.is_paused());
        assert!(mgr.build_next_packet().is_none());
        assert!(mgr.build_next_packet().is_none());
    }

    #[test]
    fn test_resume_restarts_emission() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(3), 10, Direction::Forward);

        mgr.pause();
        assert!(mgr.build_next_packet().is_none());

        mgr.resume();
        assert!(!mgr.is_paused());
        assert!(mgr.build_next_packet().is_some());
    }

    #[test]
    fn test_pause_preserves_slot_state() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(3), 10, Direction::Forward);
        let _ = mgr.build_next_packet(); // Drain dirty

        mgr.pause();

        // Update state during pause
        let _ = mgr.set_speed(addr(3), 20, Direction::Reverse);
        assert_eq!(mgr.slot_count(), 1);

        // No emission during pause
        assert!(mgr.build_next_packet().is_none());

        // Resume and verify updated state is emitted
        mgr.resume();
        let packet = mgr.build_next_packet().unwrap();

        let DccPacket::Speed28 {
            address,
            speed,
            direction,
        } = packet
        else {
            panic!("expected Speed28, got {packet:?}");
        };
        assert_eq!(address, addr(3));
        assert_eq!(speed.value(), 21);
        assert_eq!(direction, Direction::Reverse);
    }

    #[test]
    fn test_pause_resume_emits_dirty_first() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(3), 10, Direction::Forward);
        let _ = mgr.set_speed(addr(5), 15, Direction::Forward);
        let _ = mgr.build_next_packet(); // Drain one dirty
        let _ = mgr.build_next_packet(); // Drain second dirty

        mgr.pause();

        // Make addr(3) dirty during pause
        let _ = mgr.set_speed(addr(3), 20, Direction::Forward);

        mgr.resume();

        // Should emit dirty packet first (priority over refresh)
        let packet = mgr.build_next_packet().unwrap();
        let DccPacket::Speed28 { address, speed, .. } = packet else {
            panic!("expected Speed28, got {packet:?}");
        };
        assert_eq!(address, addr(3));
        assert_eq!(speed.value(), 21);
    }

    #[test]
    fn test_dirty_speed_retries_non_zero_speed() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(3), 10, Direction::Forward);

        let packet = mgr.build_next_packet().expect("expected speed packet");
        let DccPacket::Speed28 { address, speed, .. } = packet else {
            panic!("expected Speed28, got {packet:?}");
        };
        assert_eq!(address, addr(3));
        assert_eq!(speed.value(), 11);
    }

    #[test]
    fn test_dirty_speed_retries_stop_longer() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(3), 20, Direction::Forward);
        let _ = mgr.build_next_packet();

        let _ = mgr.set_speed(addr(3), 0, Direction::Forward);

        for i in 0..=DIRTY_STOP_RETRY_COUNT {
            let packet = mgr.build_next_packet().expect("expected stop speed packet");
            let DccPacket::Speed28 { address, speed, .. } = packet else {
                panic!("expected Speed28 stop on retry {i}, got {packet:?}");
            };
            assert_eq!(address, addr(3));
            assert_eq!(speed.value(), 0);
        }
    }

    #[test]
    fn test_pause_via_apply_command() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(3), 10, Direction::Forward);

        assert!(mgr.apply_command(SchedulerCommand::Pause));
        assert!(mgr.is_paused());
        assert!(mgr.build_next_packet().is_none());

        assert!(mgr.apply_command(SchedulerCommand::Resume));
        assert!(!mgr.is_paused());
        assert!(mgr.build_next_packet().is_some());
    }

    #[test]
    fn test_multiple_pause_resume_cycles() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_speed(addr(3), 10, Direction::Forward);
        let _ = mgr.build_next_packet(); // Drain dirty

        // Cycle 1
        mgr.pause();
        assert!(mgr.build_next_packet().is_none());
        mgr.resume();
        assert!(mgr.build_next_packet().is_some());

        // Cycle 2
        mgr.pause();
        assert!(mgr.build_next_packet().is_none());
        mgr.resume();
        assert!(mgr.build_next_packet().is_some());
    }

    #[test]
    fn test_function_group_refreshed_after_all_off() {
        let mut mgr = SlotManager::new();
        let _ = mgr.set_function_indexed(addr(3), FunctionIndex::new(0).unwrap(), true);
        // No dirty speed packet when slot is created by function-only command
        mgr.build_next_packet(); // dirty FunctionGroup1{fl:true}

        let _ = mgr.set_function_indexed(addr(3), FunctionIndex::new(0).unwrap(), false);
        mgr.build_next_packet(); // dirty FunctionGroup1{fl:false}

        // After: functions=0, known_groups=0b00001
        // The refresh loop MUST continue sending FunctionGroup1
        let mut got_fg1 = false;
        for _ in 0..20 {
            if let Some(DccPacket::FunctionGroup1 { fl, .. }) = mgr.build_next_packet() {
                assert!(!fl, "fl must be false after F0 OFF");
                got_fg1 = true;
                break;
            }
        }
        assert!(
            got_fg1,
            "FunctionGroup1 must appear in refresh even with all functions off"
        );
    }
}
