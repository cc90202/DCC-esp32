//! CV programming support.
//!
//! The production path is Programming on Main (POM) over the main DCC/RailCom
//! runtime. Service-mode programming remains isolated in `service_mode` until
//! a dedicated programming-track packet transport exists.

#[cfg(any(test, target_arch = "riscv32"))]
mod pom;
mod service_mode;

#[cfg(target_arch = "riscv32")]
pub(crate) use pom::drain_channel;
#[cfg(target_arch = "riscv32")]
pub use pom::{
    PomRailcomResult, PomRailcomResultChannel, PomRequest, PomRequestChannel, PomResponse,
    PomResponseChannel, PomTxStartedChannel, pom_actor_task, pom_result_from_railcom_items,
};
#[cfg(target_arch = "riscv32")]
pub use service_mode::CvProgrammer;
pub use service_mode::{
    AckDetector, AckError, CvReadError, CvWriteError, ProgrammingConfig, SessionError, SwitchError,
    TrackSwitch,
};
#[cfg(any(test, not(target_arch = "riscv32")))]
pub use service_mode::{MockAckDetector, MockTrackSwitch};
