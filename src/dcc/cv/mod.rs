//! CV programming support.
//!
//! The production path is Programming on Main (POM) over the main DCC/RailCom
//! runtime. Service-mode packet encoding remains in `dcc_protocol::packet`; a
//! runtime programmer will be added only with a dedicated programming-track
//! transport.

mod pom;
#[cfg(target_arch = "riscv32")]
pub use pom::{
    PomOp, PomRailcomResult, PomRequest, PomResponse, PomRuntimeStats, PomTxStarted,
    pom_actor_task, pom_result_from_railcom_items, pom_runtime_stats,
};
