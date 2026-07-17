//! DCC (Digital Command Control) protocol implementation
//!
//! Based on NMRA Standards S-9.1 (Electrical) and S-9.2 (Communications)
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
//! # Hardware requirements
//!
//! - ESP32-C6 with RMT peripheral
//! - GPIO2 connected to the track-driver DCC input or external DCC logic
//! - Track-power driver with current limiting and fault handling
//!
//! # Safety
//!
//! This module generates electrical signals. Always use proper current limiting
//! and overcurrent protection when connecting to model railroad track.

pub(crate) mod cv;
pub(crate) mod encoder;
pub(crate) mod packet;
pub(crate) mod scheduler;
pub(crate) mod speed28;
pub(crate) mod timing;
#[cfg(any(test, feature = "dcc-validator"))]
pub(crate) mod validator;

pub use crate::cutout::CutoutMode;

/// Precomputed transmission metadata for one DCC packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DccFrame {
    pub packet: DccPacket,
    pub cutout: CutoutMode,
    pub railcom_target_address: Option<DccAddress>,
    pub pom_request_id: Option<PomRequestId>,
}

impl DccFrame {
    #[must_use]
    pub const fn new(packet: DccPacket, cutout: CutoutMode) -> Self {
        Self {
            packet,
            cutout,
            railcom_target_address: packet.railcom_target_address(),
            pom_request_id: None,
        }
    }

    #[must_use]
    #[cfg(any(test, target_arch = "riscv32"))]
    pub(crate) const fn with_pom_request_id(mut self, request_id: PomRequestId) -> Self {
        self.pom_request_id = Some(request_id);
        self
    }

    #[must_use]
    #[cfg(any(test, target_arch = "riscv32"))]
    pub(crate) const fn effective_cutout(self) -> CutoutMode {
        match (self.cutout, self.pom_request_id) {
            (CutoutMode::PomWrite | CutoutMode::PomRead, None) => CutoutMode::None,
            (cutout, _) => cutout,
        }
    }
}

// Re-exports
#[doc(inline)]
pub use cv::PomRequestId;
#[cfg(target_arch = "riscv32")]
#[doc(inline)]
pub use cv::{
    PomRailcomResult, PomRailcomResultChannel, PomRequest, PomRequestChannel, PomResponse,
    PomResponseChannel, PomRuntimeStats, PomTxStarted, PomTxStartedChannel, pom_actor_task,
    pom_result_from_railcom_items, pom_runtime_stats,
};
#[doc(inline)]
pub use encoder::{EncodeError, PulseCode, dcc_bit_to_pulse, encode_byte, encode_dcc_packet};
#[cfg(target_arch = "riscv32")]
pub(crate) use packet::PackedAddressFlags;
#[doc(inline)]
pub use packet::{
    BinaryStateAddress, DccAddress, DccAddressKind, DccPacket, Direction, LogonGroup, NmraSpeed28,
    NmraSpeed128, PacketEncodeError, PomCv, ServiceModeCv,
};
#[cfg(any(test, target_arch = "riscv32"))]
#[doc(inline)]
pub use scheduler::SlotManager;
#[cfg(target_arch = "riscv32")]
#[doc(inline)]
pub use scheduler::packet_scheduler_task;
#[doc(inline)]
pub use scheduler::{
    FunctionChange, FunctionIndex, InvalidFunctionIndex, LocoRequest, LocoRequestDeadline,
    LocoRequestId, LocoRequestMessage, LocoRequestResult, LocoResponse, LocoSnapshot, LogicalSpeed,
    SchedulerCommand, SpeedFormat,
};
#[cfg(target_arch = "riscv32")]
#[doc(inline)]
pub use scheduler::{LocoRequestChannel, LocoResponseChannel, SchedulerCommandChannel};
#[doc(inline)]
pub use speed28::{encode_nmra_instruction_speed_bits, logical_to_nmra_packet_speed};
#[doc(inline)]
pub use timing::{
    DCC_MAX_PACKET_PULSES, DCC_ONE_HIGH_US, DCC_ONE_LOW_US, DCC_ZERO_HIGH_US, DCC_ZERO_LOW_US,
};
#[cfg(any(test, feature = "dcc-validator"))]
#[doc(inline)]
pub use validator::{
    ValidationError, validate_checksum, validate_complete, validate_full, validate_nmra_compliance,
    validate_packet_structure, validate_timing,
};

#[cfg(test)]
mod frame_tests {
    use super::*;

    #[test]
    fn pom_request_id_zero_remains_present_in_frame_metadata() {
        let request_id = PomRequestId::new(0);
        let frame =
            DccFrame::new(DccPacket::Idle, CutoutMode::None).with_pom_request_id(request_id);

        assert_eq!(frame.pom_request_id, Some(request_id));
        assert_eq!(request_id.value(), 0);
    }

    #[test]
    fn pom_cutout_without_request_id_fails_closed() {
        let frame = DccFrame::new(DccPacket::Idle, CutoutMode::PomRead);

        assert_eq!(frame.effective_cutout(), CutoutMode::None);
    }
}
