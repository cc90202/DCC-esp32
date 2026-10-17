//! DCC (Digital Command Control) protocol implementation
//!
//! Based on NMRA Standards S-9.1 (Electrical) and S-9.2 (Communications)
//!
//! The HAL-free protocol modules `packet`, `encoder`, `function_groups` and
//! `speed28` are re-exported from the `dcc_protocol` crate, and `timing` adds
//! the RMT constants to the protocol timings it re-exports from there. This
//! module adds what needs the hardware or the firmware's tasks: the slot
//! scheduler, the POM actor and the channels between them.
//!
//! # Example
//!
//! ```ignore
//! use crate::dcc::{DccAddress, DccPacket, Direction};
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

#[cfg(target_arch = "riscv32")]
pub(crate) mod channels;
pub(crate) mod cv;
#[cfg(test)]
mod golden_tests;
#[cfg(any(test, target_arch = "riscv32"))]
mod packed;
pub(crate) mod scheduler;
#[cfg(target_arch = "riscv32")]
pub(crate) mod timing;

// The HAL-free protocol modules live in `dcc_protocol`; mounted here under
// their old names so `crate::dcc::<module>::` paths keep resolving.
#[cfg(test)]
pub(crate) use dcc_protocol::validator;
pub(crate) use dcc_protocol::{encoder, function_groups, packet, speed28};

pub use crate::cutout::CutoutMode;

/// Precomputed transmission metadata for one DCC packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DccFrame {
    packet: DccPacket,
    cutout: CutoutMode,
    railcom_target_address: Option<DccAddress>,
    pom_request_id: Option<PomRequestId>,
    fence_generation: Option<PowerGeneration>,
}

/// Monotonic identifier for a request to drain all earlier DCC frames before
/// the track output may be enabled again.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct PowerGeneration(u32);

impl PowerGeneration {
    #[must_use]
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    #[must_use]
    pub const fn get(self) -> u32 {
        self.0
    }
}

/// Identifies one POM request/response round-trip on the single-slot request
/// and response channels between the Z21 network task and the POM actor.
///
/// A newtype (rather than a bare `u32`) keeps request/response matching from
/// being confused with unrelated `u32` values (CV numbers, packet sequence
/// counters) flowing through the same call sites.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
#[repr(transparent)]
pub struct PomRequestId(u32);

impl PomRequestId {
    #[must_use]
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    #[must_use]
    pub const fn value(self) -> u32 {
        self.0
    }
}

impl DccFrame {
    #[must_use]
    pub const fn new(packet: DccPacket, cutout: CutoutMode) -> Self {
        Self {
            packet,
            cutout,
            railcom_target_address: packet.railcom_target_address(),
            pom_request_id: None,
            fence_generation: None,
        }
    }

    #[cfg(target_arch = "riscv32")]
    pub(crate) const fn fence(generation: PowerGeneration) -> Self {
        Self {
            packet: DccPacket::Idle,
            cutout: CutoutMode::None,
            railcom_target_address: None,
            pom_request_id: None,
            fence_generation: Some(generation),
        }
    }

    #[must_use]
    pub const fn fence_generation(self) -> Option<PowerGeneration> {
        self.fence_generation
    }

    #[must_use]
    pub const fn packet(self) -> DccPacket {
        self.packet
    }

    #[must_use]
    pub const fn cutout(self) -> CutoutMode {
        self.cutout
    }

    #[must_use]
    pub const fn railcom_target_address(self) -> Option<DccAddress> {
        self.railcom_target_address
    }

    #[must_use]
    pub const fn pom_request_id(self) -> Option<PomRequestId> {
        self.pom_request_id
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

#[cfg(target_arch = "riscv32")]
#[doc(inline)]
pub use cv::{
    PomOp, PomRailcomResult, PomRequest, PomResponse, PomRuntimeStats, PomTxStarted,
    pom_actor_task, pom_result_from_railcom_items, pom_runtime_stats,
};
#[doc(inline)]
pub use encoder::{EncodeError, PulseCode, dcc_bit_to_pulse, encode_byte, encode_dcc_packet};
#[doc(inline)]
pub use function_groups::FunctionGroupKind;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) use packed::PackedAddressFlags;
#[doc(inline)]
pub use packet::{
    BinaryStateAddress, DCC_MAX_PACKET_BYTES, DccAddress, DccAddressKind, DccPacket, Direction,
    LogonGroup, LogonSessionId, NmraSpeed28, NmraSpeed128, PomCv, ServiceModeCv,
};
#[cfg(any(test, target_arch = "riscv32"))]
#[doc(inline)]
pub use scheduler::SlotManager;
#[cfg(target_arch = "riscv32")]
#[doc(inline)]
pub use scheduler::packet_scheduler_task;
#[doc(inline)]
pub use scheduler::{
    FunctionChange, FunctionIndex, FunctionState, InvalidFunctionIndex, LOCO_SLOT_CAPACITY,
    LocoRequest, LocoRequestDeadline, LocoRequestId, LocoRequestMessage, LocoRequestResult,
    LocoResponse, LocoSnapshot, LogicalSpeed, SchedulerCommand, SchedulerRequest, SpeedFormat,
};
#[doc(inline)]
pub use speed28::{encode_nmra_instruction_speed_bits, logical_to_nmra_packet_speed};
#[cfg(test)]
mod frame_tests {
    use super::*;

    #[test]
    fn pom_request_id_zero_remains_present_in_frame_metadata() {
        let request_id = PomRequestId::new(0);
        let frame =
            DccFrame::new(DccPacket::Idle, CutoutMode::None).with_pom_request_id(request_id);

        assert_eq!(frame.pom_request_id(), Some(request_id));
        assert_eq!(request_id.value(), 0);
    }

    #[test]
    fn frame_carries_the_railcom_target_of_its_packet() {
        let packet = DccPacket::speed_128step(
            DccAddress::new_short(3).expect("valid test address"),
            4,
            Direction::Forward,
        )
        .expect("valid speed packet");
        let frame = DccFrame::new(packet, CutoutMode::Telemetry);

        assert_eq!(
            frame.railcom_target_address(),
            packet.railcom_target_address()
        );
        assert!(frame.railcom_target_address().is_some());
    }

    #[test]
    fn pom_cutout_without_request_id_fails_closed() {
        let frame = DccFrame::new(DccPacket::Idle, CutoutMode::PomRead);

        assert_eq!(frame.effective_cutout(), CutoutMode::None);
    }
}
