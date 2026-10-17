//! The DCC protocol of the command station, without a HAL.
//!
//! Everything here is pure data and arithmetic over NMRA S-9.1/S-9.2 and the
//! RCN RailCom and logon specifications; the firmware adds the waveform
//! peripheral, the scheduler tasks and the RailCom capture around it:
//!
//! - [`packet`] holds the validated packet fields and [`DccPacket`] with its
//!   byte encoding;
//! - [`encoder`] turns a packet into its pulse sequence, with the bit
//!   durations and the pulse capacities in [`timing`];
//! - [`speed28`] and [`function_groups`] map logical speeds and functions onto
//!   the packet fields that carry them;
//! - [`speed`] and [`function`] are the locomotive state as the command station
//!   holds it;
//! - [`railcom_data`] is the decoded RailCom feedback, and [`logon`] the
//!   RCN-218 logon checksum;
//! - `validator`, behind the `test-support` feature, checks encoded pulse
//!   streams against the NMRA timing and structure.
//!
//! ```
//! use dcc_protocol::{DccAddress, DccPacket, Direction};
//!
//! let addr = DccAddress::new_short(3).unwrap();
//! let packet = DccPacket::speed_28step(addr, 15, Direction::Forward).expect("valid speed");
//! // Address, instruction 01DCSSSS, XOR checksum.
//! assert_eq!(packet.to_bytes().as_slice(), &[0x03, 0x78, 0x7B]);
//! ```
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

// The test-only constructors and the validator must never reach the firmware.
#[cfg(all(feature = "test-support", target_os = "none"))]
compile_error!("the `test-support` feature of dcc-protocol is for host tests only");

pub mod encoder;
pub mod function;
pub mod function_groups;
#[cfg(test)]
mod golden_tests;
pub mod logon;
pub mod packet;
pub mod railcom_data;
pub mod speed;
pub mod speed28;
pub mod timing;
#[cfg(any(test, feature = "test-support"))]
pub mod validator;

#[doc(inline)]
pub use encoder::{EncodeError, PulseCode, dcc_bit_to_pulse, encode_byte, encode_dcc_packet};
#[doc(inline)]
pub use function::{FunctionIndex, FunctionState, InvalidFunctionIndex};
#[doc(inline)]
pub use function_groups::FunctionGroupKind;
#[doc(inline)]
pub use packet::{
    BinaryStateAddress, DCC_MAX_PACKET_BYTES, DccAddress, DccAddressKind, DccPacket, Direction,
    LogonGroup, LogonSessionId, NmraSpeed28, NmraSpeed128, PomCv, ServiceModeCv,
};
#[doc(inline)]
pub use speed::{LogicalSpeed, SpeedFormat};
#[doc(inline)]
pub use speed28::{encode_nmra_instruction_speed_bits, logical_to_nmra_packet_speed};
