//! Framework-independent application policies and read projections.
//!
//! Network and hardware adapters depend on this module. Application code does
//! not import Z21, Embassy, ESP-HAL, logging, or peripheral drivers.

#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod loco_projection;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod locomotive;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod pom;
mod status;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod track_control;

/// Lease policy, kept beside the registry in `authority` and re-exported here
/// so the network adapters keep importing it from the application layer.
#[cfg(target_arch = "riscv32")]
pub(crate) use crate::authority::policy as client_safety;
pub use crate::authority::{LeaseEpoch, LeasePermit};
pub use status::StatusModel;

use crate::dcc::{DccAddress, Direction, FunctionState, LOCO_SLOT_CAPACITY, LogicalSpeed};

/// Scheduler-confirmed locomotive state exposed to interface adapters.
///
/// `speed` carries both its validated logical value and speed format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LocoState {
    pub address: DccAddress,
    pub speed: LogicalSpeed,
    pub direction: Direction,
    pub functions: FunctionState,
}

/// Fixed-capacity read projection used by application policies.
pub type LocoSlots = [Option<LocoState>; LOCO_SLOT_CAPACITY];
