//! CV programming support.
//!
//! The production path is Programming on Main (POM) over the main DCC/RailCom
//! runtime. Service-mode packet encoding remains in `dcc::packet`; a runtime
//! programmer will be added only with a dedicated programming-track transport.

mod pom;
pub use pom::PomRequestId;
#[cfg(target_arch = "riscv32")]
pub(crate) use pom::drain_channel;
#[cfg(target_arch = "riscv32")]
pub use pom::{
    PomRailcomResult, PomRailcomResultChannel, PomRequest, PomRequestChannel, PomResponse,
    PomResponseChannel, PomRuntimeStats, PomTxStarted, PomTxStartedChannel, pom_actor_task,
    pom_result_from_railcom_items, pom_runtime_stats,
};
