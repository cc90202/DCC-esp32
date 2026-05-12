//! DCC (Digital Command Control) protocol implementation
//!
//! Based on NMRA Standards S-9.1 (Electrical) and S-9.2 (Communications)
//!
//! # Overview
//!
//! This module provides:
//! - DCC packet encoding (`packet` module)
//! - RMT pulse code generation (`encoder` module)
//! - Async DCC engine task (`engine` module)
//! - Timing constants (`timing` module)
//!
//! # Example
//!
//! ```no_run
//! use dcc_esp32::dcc::{DccPacket, DccAddress, Direction};
//!
//! let addr = DccAddress::new_short(3).unwrap();
//! let packet = DccPacket::speed_28step(addr, 15, Direction::Forward).expect("valid speed");
//! ```
//!
//! # Hardware Requirements
//!
//! - ESP32-C6 with RMT peripheral
//! - GPIO2 connected to the track-driver DCC input
//! - Track-power driver (currently DRV8874 in PH/EN wiring)
//!
//! # Safety
//!
//! This module generates electrical signals. Always use proper current limiting
//! and overcurrent protection when connecting to model railroad track.

pub mod cv;
pub mod encoder;
#[cfg(target_arch = "riscv32")]
pub mod engine;
pub mod packet;
#[cfg(target_arch = "riscv32")]
pub mod rmt_driver;
pub mod scheduler;
pub mod speed28;
pub mod timing;
pub mod validator;

/// RailCom cutout policy attached to a frame.
///
/// The three states encode the two orthogonal bits of information the
/// downstream RMT driver needs: whether to assert the cutout at all and, when
/// asserting, whether a RailCom response is expected for an in-flight POM
/// (programming-on-main) transaction. A POM frame without a cutout is
/// unreachable in the scheduler, so the bits are represented as one enum
/// rather than two independently set booleans.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CutoutMode {
    /// No cutout: normal command and safety traffic.
    #[default]
    None,
    /// Cutout asserted to collect RailCom telemetry on unrelated traffic
    /// (loco identification, background scanning).
    Telemetry,
    /// Cutout asserted and a POM response is expected for the current packet.
    Pom,
}

/// Precomputed transmission metadata for one DCC packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DccFrame {
    pub packet: DccPacket,
    pub cutout: CutoutMode,
    pub railcom_target_address: Option<DccAddress>,
    pub pom_read_requested: bool,
}

impl DccFrame {
    #[must_use]
    pub const fn new(packet: DccPacket, cutout: CutoutMode) -> Self {
        Self {
            packet,
            cutout,
            railcom_target_address: packet.railcom_target_address(),
            pom_read_requested: matches!(packet, DccPacket::PomReadByte { .. }),
        }
    }

    #[must_use]
    pub const fn cutout_allowed(&self) -> bool {
        !matches!(self.cutout, CutoutMode::None)
    }

    #[must_use]
    pub const fn pom_requested(&self) -> bool {
        matches!(self.cutout, CutoutMode::Pom)
    }
}

// Re-exports
#[cfg(target_arch = "riscv32")]
#[doc(inline)]
pub use cv::{
    PomRailcomResult, PomRailcomResultChannel, PomRequest, PomRequestChannel, PomResponse,
    PomResponseChannel, PomTxStartedChannel, pom_actor_task, pom_result_from_railcom_items,
};
#[doc(inline)]
pub use encoder::{EncodeError, PulseCode, dcc_bit_to_pulse, encode_dcc_packet};
#[cfg(target_arch = "riscv32")]
#[doc(inline)]
pub use engine::{IdleWaveformBuildError, build_idle_rmt_buffer, dcc_engine_task};
#[cfg(target_arch = "riscv32")]
pub(crate) use packet::PackedDccAddress;
#[doc(inline)]
pub use packet::{
    BinaryStateAddress, DccAddress, DccAddressKind, DccPacket, Direction, LogonGroup, NmraSpeed28,
    NmraSpeed128, PacketEncodeError, PomCv, ServiceModeCv,
};
#[cfg(target_arch = "riscv32")]
#[doc(inline)]
pub use scheduler::SchedulerCommandChannel;
#[cfg(target_arch = "riscv32")]
#[doc(inline)]
pub use scheduler::packet_scheduler_task;
#[doc(inline)]
pub use scheduler::{
    FunctionIndex, InvalidFunctionIndex, LogicalSpeed, SchedulerCommand, SlotManager, SpeedFormat,
};
#[doc(inline)]
pub use speed28::{encode_nmra_instruction_speed_bits, logical_to_nmra_packet_speed};
#[doc(inline)]
pub use timing::{
    DCC_MAX_PACKET_PULSES, DCC_ONE_HIGH_US, DCC_ONE_LOW_US, DCC_ZERO_HIGH_US, DCC_ZERO_LOW_US,
};
#[doc(inline)]
pub use validator::ValidationError;
