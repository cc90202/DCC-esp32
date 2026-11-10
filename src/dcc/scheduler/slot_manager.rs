use core::sync::atomic::{AtomicU32, Ordering};

use heapless::{Deque, Vec};

use crate::authority::{LeaseEpoch, LeasePermit};
use crate::cutout::CutoutMode;
use crate::dcc::PomRequestId;
use dcc_protocol::function_groups::{GROUP_LAYOUTS, group_bits, group_of};
use dcc_protocol::packet::{DccAddress, DccPacket, Direction};
use dcc_protocol::speed28::logical_to_nmra_packet_speed;

use super::railcom_policy::PacketClass;
use super::{
    FunctionIndex, FunctionState, LOCO_SLOT_CAPACITY, LocoSnapshot, LogicalSpeed, SchedulerCommand,
    SpeedFormat,
};

mod emission;
mod locomotives;
mod pom;

#[cfg(test)]
pub(super) use emission::same_address_different_packet;
pub(super) use pom::pom_cutout;

/// Scheduler target period to revisit one slot in normal conditions.
const TARGET_SLOT_PERIOD_MS: u64 = 120;
/// Minimum scheduler tick to avoid tight loops under high slot count.
const MIN_TICK_MS: u64 = 5;
static SCHEDULER_INVARIANT_RECOVERY_COUNT: AtomicU32 = AtomicU32::new(0);
/// Maximum allowed interval between refreshes of active function groups.
pub(super) const MAX_FUNCTION_REFRESH_MS: u64 = 400;
/// Additional immediate retransmissions after a function state change.
pub(super) const DIRTY_FUNCTION_RETRY_COUNT: u8 = 6;

/// Number of function groups, one per entry of `GROUP_LAYOUTS`.
const GROUP_COUNT: u8 = GROUP_LAYOUTS.len() as u8;

// Each group keeps a 3-bit retry counter packed into `dirty_function_retries`.
const _: () = assert!(GROUP_LAYOUTS.len() * 3 <= u16::BITS as usize);

/// Additional immediate retransmissions after a stop command.
pub(super) const DIRTY_STOP_RETRY_COUNT: u8 = 4;
/// Refresh turns between two due visits of a stopped locomotive while another
/// one is moving. Visits alternate a function group and the zero speed, so a
/// stopped locomotive hears a function packet within twice this many packets:
/// 320 ms at the 10 ms per packet of `scheduler_tick_ms_for_slot_count` with a
/// full table, inside `MAX_FUNCTION_REFRESH_MS`. With several active groups the
/// groups rotate across those packets, as they do for moving locomotives.
pub(super) const STOPPED_REFRESH_PERIOD_TURNS: u32 = 16;
/// Small bounded queue for programming-on-main packet sequences.
///
/// POM read needs at least two CV-access packets: the command itself and a
/// follow-up packet to the same address where decoders such as the ZIMO
/// reference implementation can emit app:pom in the RailCom cutout.
pub(crate) const PENDING_POM_CAPACITY: usize = 4;

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
    functions: FunctionState,
    dirty_speed: bool,
    dirty_speed_retries: u8,
    // True once a SetLocoDrive command has been received for this slot.
    // Speed packets are only included in the cyclic refresh when this is set.
    speed_commanded: bool,
    // True while the slot exists only to keep a POM read target refreshed.
    // Any loco command clears it; a cleared slot is never released.
    pom_refresh_only: bool,
    // Dirty function groups bitmask: bit0=FG1, bit1=FG2A, bit2=FG2B, bit3=FG3, bit4=FG4.
    dirty_function_groups: u8,
    // Remaining immediate retries for each dirty function group. Each group uses
    // one 3-bit lane inside this u16 (groups 0..4).
    dirty_function_retries: u16,
    // Tracks which function groups have ever been explicitly addressed.
    // Ensures groups are still refreshed after all functions in them are off.
    known_groups: u8,
    rotation: RefreshRotation,
    /// Refresh turn from which this slot, while stopped beside a moving one,
    /// is due for its next refresh visit.
    stopped_refresh_due: u32,
    last_sent: u32,
    #[cfg(test)]
    deadline_enforced_refreshes: u32,
}

/// Where a slot stands in its speed/function refresh cycle.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(test, derive(PartialEq, Eq))]
struct RefreshRotation {
    speed_next: bool,
    turns_since_function: u8,
    /// Group the next function packet, dirty or refresh, starts looking from.
    next_function_group: u8,
}

/// The refresh packet a slot would send next and the bookkeeping that goes
/// with it, computed without touching the slot so the manager can still
/// choose another slot.
struct RefreshPlan {
    packet: Option<DccPacket>,
    rotation: RefreshRotation,
    #[cfg(test)]
    deadline_enforced: bool,
    invariant_recovered: bool,
}

impl RefreshPlan {
    /// Plan the slot's next active function group as the packet, if it has
    /// one, and move the group rotation past it.
    fn take_function(&mut self, slot: &Slot) -> bool {
        let Some(group) = slot.next_group_in(slot.active_function_group_mask()) else {
            return false;
        };
        self.packet = Some(slot.function_packet_for_group(group));
        self.rotation.next_function_group = (group + 1) % GROUP_COUNT;
        self.rotation.turns_since_function = 0;
        true
    }
}

impl Slot {
    fn new(address: DccAddress, speed: LogicalSpeed, direction: Direction) -> Self {
        Self {
            address,
            speed,
            stopped_since: None,
            direction,
            functions: FunctionState::empty(),
            dirty_speed: true,
            dirty_speed_retries: speed_retry_count(speed),
            speed_commanded: true,
            pom_refresh_only: false,
            dirty_function_groups: 0,
            dirty_function_retries: 0,
            known_groups: 0,
            rotation: RefreshRotation {
                speed_next: true,
                turns_since_function: 0,
                next_function_group: 0,
            },
            stopped_refresh_due: 0,
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
            functions: self.functions,
        }
    }

    /// Whether any function group was ever addressed on this slot.
    ///
    /// `set_function` is the only writer of `functions` and it always records
    /// the group in `known_groups`, so a non-empty `functions` implies a
    /// non-zero `known_groups`; the groups alone decide.
    fn has_any_functions(&self) -> bool {
        self.known_groups != 0
    }

    fn set_function(&mut self, function: FunctionIndex, enabled: bool) {
        self.functions = self.functions.with(function, enabled);

        let group = group_of(function).index();
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
        let kind = GROUP_LAYOUTS[usize::from(group)].kind;
        DccPacket::function_group(self.address, kind, group_bits(self.functions, kind))
    }

    /// Whether the locomotive is currently moving (a non-zero speed was set).
    fn is_moving(&self) -> bool {
        self.stopped_since.is_none()
    }

    /// First group at or after the rotation cursor whose bit is set in `mask`.
    fn next_group_in(&self, mask: u8) -> Option<u8> {
        (0..GROUP_COUNT)
            .map(|offset| (self.rotation.next_function_group + offset) % GROUP_COUNT)
            .find(|&group| mask & (1u8 << group) != 0)
    }

    /// The dirty group whose packet goes out next, in rotation order.
    fn next_dirty_group(&self) -> Option<u8> {
        self.next_group_in(self.dirty_function_groups)
    }

    /// Count one transmission of a dirty group and move the rotation past it.
    fn consume_dirty_group(&mut self, group: u8) {
        let retries = self.dirty_retries(group);
        if retries <= 1 {
            self.dirty_function_groups &= !(1u8 << group);
            self.set_dirty_retries(group, 0);
        } else {
            self.set_dirty_retries(group, retries - 1);
        }
        self.rotation.next_function_group = (group + 1) % GROUP_COUNT;
    }

    #[cfg(test)]
    fn next_refresh_packet_with_budget(
        &mut self,
        max_slot_visits_without_function: u8,
    ) -> Option<DccPacket> {
        let plan = self.plan_refresh(max_slot_visits_without_function);
        self.commit_refresh(&plan);
        plan.packet
    }

    /// Decide the next refresh packet without changing the slot.
    fn plan_refresh(&self, max_slot_visits_without_function: u8) -> RefreshPlan {
        let mut plan = RefreshPlan {
            packet: None,
            rotation: self.rotation,
            #[cfg(test)]
            deadline_enforced: false,
            invariant_recovered: false,
        };

        if !self.speed_commanded {
            if self.has_any_functions() && !plan.take_function(self) {
                plan.invariant_recovered = true;
            }
            return plan;
        }

        if !self.has_any_functions() {
            plan.packet = self.speed_packet();
            return plan;
        }

        let must_send_function =
            self.rotation.turns_since_function + 1 >= max_slot_visits_without_function.max(1);
        if must_send_function && plan.take_function(self) {
            plan.rotation.speed_next = true;
            #[cfg(test)]
            {
                plan.deadline_enforced = true;
            }
            return plan;
        }

        if !self.rotation.speed_next {
            plan.rotation.speed_next = true;
            if plan.take_function(self) {
                return plan;
            }
            plan.invariant_recovered = true;
        } else {
            plan.rotation.speed_next = false;
        }
        plan.rotation.turns_since_function = self.rotation.turns_since_function.saturating_add(1);
        plan.packet = self.speed_packet();
        plan
    }

    /// Apply the bookkeeping of a refresh plan that is being transmitted.
    fn commit_refresh(&mut self, plan: &RefreshPlan) {
        self.rotation = plan.rotation;
        if plan.invariant_recovered {
            record_scheduler_invariant_recovery();
        }
        #[cfg(test)]
        if plan.deadline_enforced {
            self.deadline_enforced_refreshes = self.deadline_enforced_refreshes.wrapping_add(1);
        }
    }

    fn active_function_group_mask(&self) -> u8 {
        let active = GROUP_LAYOUTS
            .iter()
            .filter(|layout| group_bits(self.functions, layout.kind) != 0)
            .fold(0u8, |mask, layout| mask | (1 << layout.kind.index()));
        active | self.known_groups
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
        slot.functions = FunctionState::empty();
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
        slot.rotation.speed_next = false;

        assert!(matches!(
            slot.next_refresh_packet_with_budget(1),
            Some(DccPacket::Speed128 { .. })
        ));
        assert_eq!(scheduler_invariant_recovery_count().wrapping_sub(before), 1);
    }
}

/// Manages active locomotive slots: commands first, then a round-robin refresh.
/// Slot traffic (commands and refresh) never puts two different packets to one
/// address back to back; safety, POM and telemetry packets go out at once, as
/// in every surveyed station, and the slot traffic that follows them waits.
/// Stopped locomotives are visited less often while others are moving.
#[cfg_attr(test, derive(Debug, Clone, PartialEq, Eq))]
pub(crate) struct SlotManager {
    slots: Vec<Slot, LOCO_SLOT_CAPACITY>,
    pending_broadcast_estop: bool,
    pending_estop_targets: Deque<DccAddress, LOCO_SLOT_CAPACITY>,
    pending_pom: Deque<PendingPomPacket, PENDING_POM_CAPACITY>,
    active_pom: Option<ActivePomRequest>,
    pending_railcom_telemetry: Option<DccPacket>,
    next_index: usize,
    /// Slot index where the next dirty-packet scan starts.
    dirty_cursor: usize,
    /// Last packet really sent to the track, reported by the runtime through
    /// `note_emitted`; `None` before the first one.
    last_packet: Option<DccPacket>,
    /// Number of refresh turns taken so far; clocks the stopped slots' visits.
    refresh_turn: u32,
    /// Slot index where the next scan for a filler refresh starts, so the
    /// stopped slots share the turns no due slot can take.
    filler_cursor: usize,
    mutation_sequence: u64,
    paused: bool,
    accepted_lease_epoch: Option<LeaseEpoch>,
    dequeued_authority: PacketAuthority,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SlotAdmission {
    Inserted { index: usize },
    Replaced { index: usize, removed: DccAddress },
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
    pub(crate) fn new() -> Self {
        Self {
            slots: Vec::new(),
            pending_broadcast_estop: false,
            pending_estop_targets: Deque::new(),
            pending_pom: Deque::new(),
            active_pom: None,
            pending_railcom_telemetry: None,
            next_index: 0,
            dirty_cursor: 0,
            last_packet: None,
            refresh_turn: 0,
            filler_cursor: 0,
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
    pub(crate) fn pause(&mut self) {
        self.paused = true;
    }

    /// Resume packet emission after pause.
    pub(crate) fn resume(&mut self) {
        self.paused = false;
    }

    /// Check if scheduler is currently paused.
    #[cfg(any(test, feature = "dcc-a"))]
    #[must_use]
    pub(crate) fn is_paused(&self) -> bool {
        self.paused
    }

    /// Ensure the scheduler has a non-dirty cyclic refresh slot for a decoder.
    #[must_use]
    pub(crate) fn ensure_railcom_refresh_slot(
        &mut self,
        address: DccAddress,
        format: SpeedFormat,
    ) -> bool {
        if self.slot_index(address).is_some() {
            return true;
        }
        let mut slot = Slot::new(address, LogicalSpeed::zero(format), Direction::Forward);
        slot.dirty_speed = false;
        slot.dirty_speed_retries = 0;
        slot.pom_refresh_only = true;
        matches!(
            self.admit_new_slot(slot, false),
            SlotAdmission::Inserted { .. }
        )
    }

    /// Queue the one RailCom-related packet sent with a telemetry cutout;
    /// refused while another one waits.
    #[cfg(any(test, feature = "dcc-a"))]
    #[must_use]
    pub(crate) fn railcom_telemetry_packet(&mut self, packet: DccPacket) -> bool {
        if self.pending_railcom_telemetry.is_some() {
            return false;
        }
        self.pending_railcom_telemetry = Some(packet);
        true
    }

    /// Whether no telemetry packet waits for its turn.
    #[cfg(feature = "dcc-a")]
    #[must_use]
    pub(crate) fn railcom_telemetry_is_free(&self) -> bool {
        self.pending_railcom_telemetry.is_none()
    }

    /// Addresses of the active locomotives.
    #[cfg(feature = "dcc-a")]
    pub(crate) fn addresses(&self) -> impl Iterator<Item = DccAddress> + '_ {
        self.slots.iter().map(|slot| slot.address)
    }

    /// Whether a locomotive is active on `address`.
    #[cfg(feature = "dcc-a")]
    #[must_use]
    pub(crate) fn has_slot(&self, address: DccAddress) -> bool {
        self.slot_index(address).is_some()
    }

    /// Position of the slot holding `address`, if the locomotive is active.
    fn slot_index(&self, address: DccAddress) -> Option<usize> {
        self.slots.iter().position(|slot| slot.address == address)
    }

    /// Number of active slots.
    #[must_use]
    pub(crate) fn slot_count(&self) -> usize {
        self.slots.len()
    }

    /// Returns `true` if no slots are active.
    #[cfg(test)]
    #[must_use]
    pub(crate) fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }

    /// Apply a scheduler command.
    #[must_use]
    pub(crate) fn apply_command(&mut self, command: SchedulerCommand) -> bool {
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
            SchedulerCommand::RestartRailcomDiscovery => true,
            // Taken by the scheduler runtime for the decoder table.
            #[cfg(feature = "dcc-a")]
            SchedulerCommand::LogonAnswer(_) => false,
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
    visits.min(u64::from(u8::MAX)) as u8
}
