//! DCC Packet Scheduler with Cyclic Refresh
//!
//! Manages active locomotive slots and retransmits commands cyclically to keep
//! decoders updated.
//!
//! The scheduler runs as an async Embassy actor task. It takes locomotive
//! requests (`LocoRequest`: speed, functions, emergency stop) and control
//! messages (`SchedulerCommand`: stop-all, POM, pause, resume, RailCom discovery
//! and telemetry) over channels and generates a continuous stream of DCC packets
//! to refresh decoder state.
//!
//! Speed packets that have just changed go out first, then changed function
//! packets, and whatever bandwidth is left goes to the cyclic refresh that keeps
//! every active locomotive updated. Sending a stale refresh ahead of a fresh
//! speed change is what makes a throttle feel laggy, so the ordering matters
//! more than it looks.
//!
//! The scheduler holds up to 12 active locomotive slots. The fault manager
//! pauses packet emission on an emergency stop or a latched fault and resumes
//! it once the track may run again; slot state survives the pause.
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

use crate::authority::{LeaseEpoch, LeasePermit};
use crate::dcc::PomRequestId;
use dcc_protocol::packet::{DccAddress, DccPacket, Direction};

#[cfg(any(test, target_arch = "riscv32"))]
mod core;
#[cfg(target_arch = "riscv32")]
mod railcom_discovery;
mod railcom_policy;
#[cfg(target_arch = "riscv32")]
mod runtime;
#[cfg(any(test, target_arch = "riscv32"))]
mod slot_manager;

#[cfg(any(test, all(target_arch = "riscv32", feature = "bench-diag")))]
pub(crate) use railcom_policy::{RailcomSchedulerStats, railcom_scheduler_stats};
#[cfg(target_arch = "riscv32")]
pub use runtime::packet_scheduler_task;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) use slot_manager::PENDING_POM_CAPACITY;
#[cfg(test)]
pub(crate) use slot_manager::SlotManager;
// The speed and function types live in `dcc_protocol`; re-exported so the
// `crate::dcc::scheduler::` and `super::` paths keep resolving.
pub use dcc_protocol::function::{FunctionIndex, FunctionState, InvalidFunctionIndex};
pub use dcc_protocol::speed::{LogicalSpeed, SpeedFormat};

/// Maximum number of locomotive slots maintained by the command station.
pub const LOCO_SLOT_CAPACITY: usize = 12;

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
    pub functions: FunctionState,
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

    #[must_use]
    pub const fn requires_lease(self) -> bool {
        matches!(
            self,
            Self::EnsureRefresh { .. } | Self::SetSpeed { .. } | Self::SetFunction { .. }
        )
    }
}

/// Work admitted to the correlated scheduler request boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum SchedulerRequest {
    Loco {
        request: LocoRequest,
        permit: Option<LeasePermit>,
    },
    ResetForLease {
        permit: LeasePermit,
    },
}

/// One correlated request placed on the scheduler boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct LocoRequestMessage {
    pub request_id: LocoRequestId,
    pub request: SchedulerRequest,
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
    LeaseReset(LeaseEpoch),
}

/// Scheduler response carrying the request identity unchanged.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct LocoResponse {
    pub request_id: LocoRequestId,
    pub result: LocoRequestResult,
}

/// Command messages sent to the packet scheduler actor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerCommand {
    EmergencyStopAll,
    ProgramOnMain {
        request_id: PomRequestId,
        permit: LeasePermit,
        packet: DccPacket,
    },
    CloseProgramOnMain {
        request_id: PomRequestId,
    },
    /// Start a fresh discovery window after track power is physically enabled.
    RestartRailcomDiscovery,
    /// Send one RailCom-related packet with a telemetry cutout.
    RailcomTelemetry {
        packet: DccPacket,
    },
    /// Pause packet emission while keeping every slot's state.
    ///
    /// Sent by the fault relay when an emergency stop or a fault latches.
    Pause,
    /// Resume packet emission: pending changes go out first, then refresh.
    ///
    /// Sent by the fault relay when track power is armed or a latch clears.
    Resume,
}

#[cfg(test)]
mod tests;
