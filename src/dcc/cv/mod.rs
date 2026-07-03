//! CV programming support.
//!
//! The production path is Programming on Main (POM) over the main DCC/RailCom
//! runtime. Service-mode programming remains isolated in `service_mode` until
//! a dedicated programming-track packet transport exists.

#[cfg(any(test, target_arch = "riscv32"))]
mod pom;
// Service-mode scaffolding (`CvProgrammer` and friends) has no production
// caller yet - the programming-track packet transport isn't implemented, see
// `service_mode`'s module doc. Nothing outside this module needs its API, so
// it stays crate-private with no re-export here or from `dcc::mod`, and the
// currently-unreferenced parts are allowed to sit dead until real hardware
// integration starts calling them.
#[allow(dead_code)]
mod service_mode;

#[cfg(target_arch = "riscv32")]
pub(crate) use pom::drain_channel;
#[cfg(target_arch = "riscv32")]
pub use pom::{
    PomRailcomResult, PomRailcomResultChannel, PomRequest, PomRequestChannel, PomRequestId,
    PomResponse, PomResponseChannel, PomTxStartedChannel, pom_actor_task,
    pom_result_from_railcom_items,
};
