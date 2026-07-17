//! DCC Packet Scheduler with Cyclic Refresh
//!
//! Manages active locomotive slots and retransmits commands cyclically to keep
//! decoders updated.
//!
//! The scheduler runs as an async Embassy actor task. It accepts `SchedulerCommand` messages
//! (SetSpeed, SetFunction, EmergencyStop, etc.) via a channel and generates a continuous
//! stream of DCC packets to refresh decoder state.
//!
//! Speed packets that have just changed go out first, then changed function
//! packets, and whatever bandwidth is left goes to the cyclic refresh that keeps
//! every active locomotive updated. Sending a stale refresh ahead of a fresh
//! speed change is what makes a throttle feel laggy, so the ordering matters
//! more than it looks.
//!
//! The scheduler holds up to 12 active locomotive slots, manages consists for
//! multi-unit trains, and coordinates pause and resume with CV programming,
//! since the main track has to stop while programming runs.
//!
//! # Function index
//!
//! Functions are controlled via `FunctionIndex`, which validates the index 0-28:
//! - 0 = FL (headlight)
//! - 1-4 = F1-F4 (transmitted in Function Group 1)
//! - 5-8 = F5-F8 (Function Group 2A)
//! - 9-12 = F9-F12 (Function Group 2B)
//! - 13-20 = F13-F20 (Function Group 3)
//! - 21-28 = F21-F28 (Function Group 4)

use crate::dcc::PomRequestId;
use crate::dcc::packet::{DccAddress, DccPacket, Direction};

#[cfg(any(test, target_arch = "riscv32"))]
mod core;
#[cfg(target_arch = "riscv32")]
mod railcom_discovery;
mod railcom_policy;
#[cfg(target_arch = "riscv32")]
mod runtime;
#[cfg(any(test, target_arch = "riscv32"))]
mod slot_manager;

#[cfg(any(test, target_arch = "riscv32"))]
pub use railcom_policy::{RailcomSchedulerStats, railcom_scheduler_stats};
#[cfg(target_arch = "riscv32")]
pub use runtime::{
    LocoRequestChannel, LocoResponseChannel, SchedulerCommandChannel, packet_scheduler_task,
};
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) use slot_manager::PENDING_POM_CAPACITY;
#[cfg(any(test, target_arch = "riscv32"))]
pub use slot_manager::SlotManager;

/// Speed format for a locomotive slot
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum SpeedFormat {
    Speed28,
    Speed128,
}

/// Validated logical (runtime) speed value for a locomotive slot.
///
/// "Logical" speed is the scheduler's internal protocol-agnostic representation:
/// - `0` = stop
/// - Speed28  → `1..=28`  = speed steps
/// - Speed128 → `1..=126` = speed steps
///
/// The format is stored with the value so a speed validated for one format
/// cannot later be paired with another format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct LogicalSpeed {
    value: u8,
    format: SpeedFormat,
}

impl LogicalSpeed {
    /// Backward-compatible Speed28 stop value for internal scheduler tests.
    pub const ZERO: Self = Self::zero(SpeedFormat::Speed28);

    /// Validate `value` as a logical speed for `format`.
    #[must_use]
    pub fn new(value: u8, format: SpeedFormat) -> Option<Self> {
        let in_range = match format {
            SpeedFormat::Speed28 => value <= 28,
            SpeedFormat::Speed128 => value <= 126,
        };
        in_range.then_some(Self { value, format })
    }

    /// Build a stopped speed in the requested format.
    #[must_use]
    pub const fn zero(format: SpeedFormat) -> Self {
        Self { value: 0, format }
    }

    /// Return the raw logical speed value.
    #[must_use]
    pub const fn value(self) -> u8 {
        self.value
    }

    /// Return the format against which this speed was validated.
    #[must_use]
    pub const fn format(self) -> SpeedFormat {
        self.format
    }

    #[must_use]
    pub const fn is_zero(self) -> bool {
        self.value == 0
    }
}

/// Function index mapping:
/// - 0 => FL
/// - 1..=28 => F1..F28
const MAX_FUNCTION_INDEX: u8 = 28;

/// Validated function index (F0..F28).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
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

/// Correlates one network locomotive request with its scheduler response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
#[repr(transparent)]
pub struct LocoRequestId(u32);

impl LocoRequestId {
    #[must_use]
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    #[must_use]
    pub const fn value(self) -> u32 {
        self.0
    }
}

/// Monotonic scheduler tick after which a network request must not mutate
/// locomotive state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
#[repr(transparent)]
pub struct LocoRequestDeadline(u64);

impl LocoRequestDeadline {
    #[must_use]
    pub const fn from_ticks(ticks: u64) -> Self {
        Self(ticks)
    }

    #[must_use]
    pub const fn is_expired_at(self, now_ticks: u64) -> bool {
        now_ticks >= self.0
    }
}

/// Authoritative scheduler state exposed across the network boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct LocoSnapshot {
    pub address: DccAddress,
    pub speed: LogicalSpeed,
    pub direction: Direction,
    pub functions: u32,
}

/// Requested change to one locomotive function.
///
/// Keeping `Toggle` inside the scheduler boundary ensures that the operation is
/// resolved against authoritative state, not against a possibly stale network
/// projection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum FunctionChange {
    Enable,
    Disable,
    Toggle,
}

/// Locomotive operation requested by the network adapter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum LocoRequest {
    GetState {
        address: DccAddress,
    },
    EnsureRefresh {
        address: DccAddress,
        format: SpeedFormat,
    },
    SetSpeed {
        address: DccAddress,
        speed: LogicalSpeed,
        direction: Direction,
    },
    SetFunction {
        address: DccAddress,
        function: FunctionIndex,
        change: FunctionChange,
    },
    EmergencyStop {
        address: DccAddress,
    },
}

impl LocoRequest {
    #[must_use]
    pub const fn address(self) -> DccAddress {
        match self {
            Self::GetState { address }
            | Self::EnsureRefresh { address, .. }
            | Self::SetSpeed { address, .. }
            | Self::SetFunction { address, .. }
            | Self::EmergencyStop { address } => address,
        }
    }
}

/// One correlated request placed on the scheduler boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct LocoRequestMessage {
    pub request_id: LocoRequestId,
    pub request: LocoRequest,
    pub deadline: LocoRequestDeadline,
}

/// Result of applying a locomotive request to scheduler-owned state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum LocoRequestResult {
    Found(LocoSnapshot),
    Inserted(LocoSnapshot),
    Updated(LocoSnapshot),
    Replaced {
        removed: DccAddress,
        inserted: LocoSnapshot,
    },
    Full,
    NotFound,
    Rejected,
    Expired,
}

/// Scheduler response carrying the request identity unchanged.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct LocoResponse {
    pub request_id: LocoRequestId,
    pub result: LocoRequestResult,
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
    EmergencyStopAll,
    ProgramOnMain {
        request_id: PomRequestId,
        packet: DccPacket,
    },
    CloseProgramOnMain {
        request_id: PomRequestId,
    },
    /// Stop startup RailCom discovery traffic before latency-sensitive POM.
    ///
    /// Discovery is useful while idle, but during POM reads CH2 must be left
    /// free for app:pom instead of logon/search responses.
    SuspendRailcomDiscovery,
    /// Start a fresh discovery window after track power is physically enabled.
    RestartRailcomDiscovery,
    /// Ensure periodic packets exist for a decoder address so RailCom feedback
    /// that is emitted after a POM command can be attributed to that decoder.
    /// Send one RailCom-related packet with a telemetry cutout.
    RailcomTelemetry {
        packet: DccPacket,
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
        speed: LogicalSpeed,
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

#[cfg(test)]
mod tests;
