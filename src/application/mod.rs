//! Framework-independent application policies and read projections.
//!
//! Network and hardware adapters depend on this module. Application code does
//! not import Z21, Embassy, ESP-HAL, logging, or peripheral drivers.

#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod client_safety;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod loco_projection;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod locomotive;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod pom;
pub mod status;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod track_control;

pub use status::StatusModel;

use crate::dcc::{DccAddress, Direction, FunctionState, LogicalSpeed};

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
pub type LocoSlots = [Option<LocoState>; 12];
