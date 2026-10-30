//! CV programming support.
//!
//! The production path is Programming on Main (POM) over the main DCC/RailCom
//! runtime. Service-mode packet encoding remains in `dcc_protocol::packet`; a
//! runtime programmer will be added only with a dedicated programming-track
//! transport.

mod pom;
#[cfg(all(target_arch = "riscv32", feature = "bench-diag"))]
pub(crate) use pom::pom_runtime_stats;
#[cfg(target_arch = "riscv32")]
pub(crate) use pom::{
    PomOp, PomRailcomResult, PomRequest, PomResponse, PomTxStarted, pom_actor_task,
    pom_result_from_railcom_items,
};
