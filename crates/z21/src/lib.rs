//! Z21 LAN Protocol v1.13: pure no_std parsing and encoding.
//!
//! This crate parses incoming UDP payloads into typed [`Z21Command`] values and
//! encodes the response and broadcast frames used by the network task. It has no
//! Embassy or ESP dependencies, so protocol behavior is covered by host tests;
//! the DCC types it carries come from `dcc_protocol`.
//!
//! ```
//! use z21::{Z21Command, encode_serial_number, parse_frame};
//!
//! // LAN_GET_SERIAL_NUMBER: DataLen 4, header 0x0010, both little-endian.
//! let request = [0x04, 0x00, 0x10, 0x00];
//! assert_eq!(parse_frame(&request), Ok(Z21Command::GetSerialNumber));
//!
//! let mut reply = [0u8; 8];
//! assert_eq!(encode_serial_number(0x1234_5678, &mut reply), Some(8));
//! assert_eq!(reply, [0x08, 0x00, 0x10, 0x00, 0x78, 0x56, 0x34, 0x12]);
//! ```
//!
//! # Command coverage
//!
//! Station identity and state:
//!
//! - `GetSerialNumber`: command station serial
//! - `GetCode`: firmware version/code
//! - `GetHwInfo`: hardware model and version
//! - `GetFirmwareVersion`: XBus firmware version
//! - `GetSystemState`: track power and running state
//! - `SetBroadcastFlags`: which events the client wants broadcast
//!
//! Track and locomotive control:
//!
//! - `SetTrackPowerOn` / `SetTrackPowerOff`: main track output
//! - `SetStop`: broadcast emergency stop
//! - `SetLocoEstop`: emergency stop for one locomotive
//! - `SetLocoDrive`: speed and direction
//! - `SetLocoFunction`: functions such as lights and horn
//! - `GetLocoInfo`: current speed and function state
//! - `GetLocoMode` / `SetLocoMode`: per-address output format
//!
//! Feedback:
//!
//! - `RailcomGetData`: poll the cached RailCom data
//! - `GetTurnoutInfo`: accessory decoder state, always answered as unknown
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod command;
mod encoding;
mod parsing;
mod state;
#[cfg(test)]
mod tests;
mod wire;

pub use command::{FrameKind, FunctionAction, ParseError, RailcomRequestType, Z21Command};
pub use encoding::{
    encode_bc_stopped, encode_bc_track_power, encode_bc_track_short_circuit, encode_code,
    encode_cv_nack, encode_cv_result, encode_firmware_version, encode_hwinfo, encode_loco_info,
    encode_loco_mode, encode_railcom_data, encode_serial_number, encode_status,
    encode_system_state, encode_turnout_info, encode_unknown_command, encode_xbus_version,
};
pub use parsing::{FrameBoundaryError, FrameIter, frame_kind, iter_frames, parse_frame};
pub use state::{CentralState, LocoInfo};
// The firmware's datagram handling reads these two headers; it reads
// `HEADER_SYSTEMSTATE_GETDATA` only in its bench diagnostics log.
pub use wire::{HEADER_SYSTEMSTATE_GETDATA, HEADER_XBUS};
