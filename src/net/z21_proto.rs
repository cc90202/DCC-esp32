//! Z21 LAN Protocol v1.13 - pure no_std parsing and encoding.
//!
//! This module parses incoming UDP payloads into typed [`Z21Command`] values and
//! encodes the response and broadcast frames used by the network task. It has no
//! Embassy or ESP dependencies, so protocol behavior is covered by host tests.
//!
//! # Command Coverage
//!
//! Implemented commands:
//! - **GetSerialNumber** - Returns command station serial
//! - **GetCode** - Returns firmware version/code
//! - **GetHwInfo** - Returns hardware info (model, version)
//! - **GetSystemState** - Returns track power, running state
//! - **GetFirmwareVersion** - Returns XBus firmware version
//! - **SetBroadcastFlags** - Configure which events to broadcast
//! - **GetLocoMode/SetLocoMode** - Query/store per-address output format
//! - **SetTrackPowerOn/Off** - Control main track output
//! - **SetStop** - Broadcast e-stop
//! - **SetLocoEstop** - Emergency stop specific loco
//! - **SetLocoDrive** - Control loco speed/direction
//! - **SetLocoFunction** - Control loco functions (lights, horn, etc.)
//! - **GetLocoInfo** - Query loco speed/function state
//! - **RailcomGetData** - Poll cached RailCom data
//! - **GetTurnoutInfo** - Query accessory decoder state (returns unknown)

use crate::dcc::{DccAddress, Direction, SpeedFormat};

mod encoding;
mod parsing;
mod slots;
#[cfg(test)]
mod tests;
mod wire;

pub use encoding::{
    encode_bc_stopped, encode_bc_track_power, encode_code, encode_cv_nack, encode_cv_result,
    encode_firmware_version, encode_hwinfo, encode_loco_info, encode_loco_mode,
    encode_railcom_data, encode_serial_number, encode_status, encode_system_state,
    encode_turnout_info, encode_unknown_command, encode_xbus_version,
};
pub use parsing::{FrameIter, iter_frames, parse_frame};
pub use slots::{find_or_insert, find_slot};
#[cfg(target_arch = "riscv32")]
pub(crate) use wire::{HEADER_SYSTEMSTATE_GETDATA, HEADER_XBUS};

/// `LAN_SET_BROADCASTFLAGS` payload - an OR-combination of Z21 broadcast
/// subscription bits (Z21 LAN Protocol Specification v1.13, §2.16).
///
/// We currently push all relevant broadcasts unconditionally rather than
/// gating on the subscribed set, so `SetBroadcastFlags` only needs to parse
/// and store the value (see `dispatch_command`'s handling of this variant).
/// The named accessors document the handful of bits apps are known to rely
/// on, without committing to modeling the full 32-bit set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct BroadcastFlags(u32);

impl BroadcastFlags {
    /// §2.16 `0x00000001` - driving/switching broadcasts (track power,
    /// programming mode, short circuit, stopped, loco info, turnout info).
    const BASIC_DRIVING_AND_SWITCHING: u32 = 0x0000_0001;
    /// §2.16 `0x00000100` - `LAN_SYSTEMSTATE_DATACHANGED` broadcasts.
    const SYSTEM_STATUS: u32 = 0x0000_0100;
    /// §2.16 `0x00000004` - `LAN_RAILCOM_DATACHANGED` for subscribed locos.
    const RAILCOM_DATA: u32 = 0x0000_0004;

    #[must_use]
    pub const fn new(raw: u32) -> Self {
        Self(raw)
    }

    #[must_use]
    pub const fn value(self) -> u32 {
        self.0
    }

    /// `0x00000001` - driving/switching broadcasts requested.
    #[must_use]
    pub const fn basic_driving_and_switching(self) -> bool {
        (self.0 & Self::BASIC_DRIVING_AND_SWITCHING) != 0
    }

    /// `0x00000100` - system status broadcasts requested.
    #[must_use]
    pub const fn system_status(self) -> bool {
        (self.0 & Self::SYSTEM_STATUS) != 0
    }

    /// `0x00000004` - RailCom data broadcasts requested.
    #[must_use]
    pub const fn railcom_data(self) -> bool {
        (self.0 & Self::RAILCOM_DATA) != 0
    }
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
    SetBroadcastFlags {
        flags: BroadcastFlags,
    },
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
    /// LAN_X_GET_TURNOUT_INFO - accessory decoder state request.
    /// We have no accessory decoder support; respond with state=0 (unknown).
    GetTurnoutInfo {
        address: u16,
    },
    RailcomGetData {
        request_type: u8,
        address: u16,
    },
    /// `LAN_LOCONET_DETECTOR` - LocoNet track occupancy detector query.
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
    InvalidCvAddress,
}
