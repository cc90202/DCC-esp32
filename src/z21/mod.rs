//! Z21 LAN Protocol v1.13: pure no_std parsing and encoding.
//!
//! This module parses incoming UDP payloads into typed [`Z21Command`] values and
//! encodes the response and broadcast frames used by the network task. It has no
//! Embassy or ESP dependencies, so protocol behavior is covered by host tests.
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

use crate::dcc::{DccAddress, Direction, LogicalSpeed, SpeedFormat};

mod encoding;
mod parsing;
#[cfg(test)]
mod tests;
mod wire;

pub use encoding::{
    encode_bc_stopped, encode_bc_track_power, encode_code, encode_cv_nack, encode_cv_result,
    encode_firmware_version, encode_hwinfo, encode_loco_info, encode_loco_mode,
    encode_railcom_data, encode_serial_number, encode_status, encode_system_state,
    encode_turnout_info, encode_unknown_command, encode_xbus_version,
};
pub use parsing::{FrameIter, frame_kind, iter_frames, parse_frame};
#[cfg(target_arch = "riscv32")]
pub(crate) use wire::{HEADER_SYSTEMSTATE_GETDATA, HEADER_XBUS};

/// Locomotive state required to encode a `LAN_X_LOCO_INFO` frame.
///
/// This wire-facing view keeps the protocol contract independent from the
/// application's projection types. Network adapters are responsible for
/// mapping their state into this representation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct LocoInfo {
    pub address: DccAddress,
    pub speed: LogicalSpeed,
    pub direction: Direction,
    pub functions: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct FrameKind {
    pub header: u16,
    pub xheader: u8,
}

/// Parsed Z21 command from an incoming UDP frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum Z21Command {
    GetSerialNumber,
    GetCode,
    GetHwInfo,
    GetSystemState,
    Logoff,
    GetXBusVersion,
    GetFirmwareVersion,
    GetStatus,
    /// Accept a `LAN_SET_BROADCASTFLAGS` subscription request.
    ///
    /// Broadcast delivery is currently unconditional, so the validated
    /// payload is intentionally not retained.
    SetBroadcastFlags,
    SetTrackPowerOn,
    SetTrackPowerOff,
    SetStop,
    SetLocoEstop {
        address: DccAddress,
    },
    GetLocoMode {
        address: DccAddress,
    },
    SetLocoMode {
        address: DccAddress,
        mode: u8,
    },
    GetLocoInfo {
        address: DccAddress,
    },
    SetLocoDrive {
        address: DccAddress,
        speed: u8,
        direction: Direction,
        format: SpeedFormat,
    },
    SetLocoFunction {
        address: DccAddress,
        function: u8,
        action: FunctionAction,
    },
    CvPomWriteByte {
        address: DccAddress,
        cv: u16,
        value: u8,
    },
    CvPomReadByte {
        address: DccAddress,
        cv: u16,
    },
    /// LAN_X_GET_TURNOUT_INFO: accessory decoder state request.
    /// We have no accessory decoder support; respond with state=0 (unknown).
    GetTurnoutInfo {
        address: DccAddress,
    },
    RailcomGetData {
        request_type: u8,
        address: Option<DccAddress>,
    },
    /// `LAN_LOCONET_DETECTOR`: LocoNet track occupancy detector query.
    /// We have no LocoNet detector support; the parser only validates frame
    /// length, so no fields are extracted (dispatch is a 0-byte no-op).
    LoconetDetector,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum FunctionAction {
    On,
    Off,
    Toggle,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum ParseError {
    FrameTooShort,
    LenMismatch,
    BadXBusChecksum,
    InvalidAddress,
    InvalidFunction,
    InvalidFunctionAction,
    InvalidCvAddress,
}
