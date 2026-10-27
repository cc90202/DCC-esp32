//! Commands parsed from Z21 frames, and the errors of parsing them.

use core::fmt;

use dcc_protocol::{DccAddress, Direction, FunctionIndex, LogicalSpeed, PomCv};

/// The header and X-Header of a frame, read without parsing the rest.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrameKind {
    /// Frame header; 0 when the frame is too short to carry one.
    pub header: u16,
    /// X-Header of an XBus frame; 0 for any other frame or when missing.
    pub xheader: u8,
}

/// Selector carried by `LAN_RAILCOM_GETDATA`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RailcomRequestType {
    /// Data of one locomotive, selector 0x01.
    Locomotive,
    /// Any other selector, kept as received.
    Unsupported(u8),
}

impl From<u8> for RailcomRequestType {
    fn from(value: u8) -> Self {
        match value {
            0x01 => Self::Locomotive,
            value => Self::Unsupported(value),
        }
    }
}

/// Parsed Z21 command from an incoming UDP frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Z21Command {
    /// `LAN_GET_SERIAL_NUMBER`.
    GetSerialNumber,
    /// `LAN_GET_CODE`, or the legacy header some apps send for it.
    GetCode,
    /// `LAN_GET_HWINFO`.
    GetHwInfo,
    /// `LAN_SYSTEMSTATE_GETDATA`.
    GetSystemState,
    /// `LAN_LOGOFF`: the client leaves.
    Logoff,
    /// `LAN_X_GET_VERSION`.
    GetXBusVersion,
    /// `LAN_X_GET_FIRMWARE_VERSION`.
    GetFirmwareVersion,
    /// `LAN_X_GET_STATUS`.
    GetStatus,
    /// Accept a `LAN_SET_BROADCASTFLAGS` subscription request.
    ///
    /// Broadcast delivery is currently unconditional, so the validated
    /// payload is intentionally not retained.
    SetBroadcastFlags,
    /// `LAN_X_SET_TRACK_POWER_ON`.
    SetTrackPowerOn,
    /// `LAN_X_SET_TRACK_POWER_OFF`.
    SetTrackPowerOff,
    /// `LAN_X_SET_STOP`: emergency stop of every locomotive.
    SetStop,
    /// `LAN_X_SET_LOCO_E_STOP`: emergency stop of one locomotive.
    SetLocoEstop {
        /// Locomotive to stop.
        address: DccAddress,
    },
    /// `LAN_GET_LOCOMODE`.
    GetLocoMode {
        /// Locomotive asked about.
        address: DccAddress,
    },
    /// `LAN_SET_LOCOMODE`.
    SetLocoMode {
        /// Locomotive to configure.
        address: DccAddress,
        /// Requested output format byte, as received.
        mode: u8,
    },
    /// `LAN_X_GET_LOCO_INFO`.
    GetLocoInfo {
        /// Locomotive asked about.
        address: DccAddress,
    },
    /// `LAN_X_SET_LOCO_DRIVE`: speed and direction.
    SetLocoDrive {
        /// Locomotive to drive.
        address: DccAddress,
        /// Logical speed in the format the client used; stop and a wire
        /// emergency stop both read as step 0.
        speed: LogicalSpeed,
        /// Direction of travel.
        direction: Direction,
    },
    /// `LAN_X_SET_LOCO_FUNCTION`.
    SetLocoFunction {
        /// Locomotive to change.
        address: DccAddress,
        /// Function index, F0..=F28.
        function: FunctionIndex,
        /// What to do with the function.
        action: FunctionAction,
    },
    /// `LAN_X_CV_POM_WRITE_BYTE`.
    CvPomWriteByte {
        /// Locomotive to program.
        address: DccAddress,
        /// CV to write.
        cv: PomCv,
        /// Value to write.
        value: u8,
    },
    /// `LAN_X_CV_POM_READ_BYTE`.
    CvPomReadByte {
        /// Locomotive to read from.
        address: DccAddress,
        /// CV to read.
        cv: PomCv,
    },
    /// LAN_X_GET_TURNOUT_INFO: accessory decoder state request.
    /// We have no accessory decoder support; respond with state=0 (unknown).
    GetTurnoutInfo {
        /// Accessory function address as received (`FAdr`, 0-based): not a
        /// locomotive address, so 0 is valid.
        address: u16,
    },
    /// `LAN_RAILCOM_GETDATA`.
    RailcomGetData {
        /// What the client asks for.
        request_type: RailcomRequestType,
        /// Locomotive asked about; `None` when the field is no valid address.
        address: Option<DccAddress>,
    },
    /// `LAN_LOCONET_DETECTOR`: LocoNet track occupancy detector query.
    /// We have no LocoNet detector support; the parser only validates frame
    /// length, so no fields are extracted (dispatch is a 0-byte no-op).
    LoconetDetector,
    /// A frame this command station does not handle.
    Unknown,
}

/// What a `LAN_X_SET_LOCO_FUNCTION` does with the function.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FunctionAction {
    /// Switch it on.
    On,
    /// Switch it off.
    Off,
    /// Invert its current state.
    Toggle,
}

/// Why a Z21 frame was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParseError {
    /// The frame is shorter than its message type needs.
    FrameTooShort,
    /// The `DataLen` field does not fit the frame.
    LenMismatch,
    /// The XBus checksum byte does not match the payload.
    BadXBusChecksum,
    /// The locomotive address is out of range.
    InvalidAddress,
    /// The speed step is above the top step of its format.
    InvalidSpeed,
    /// The function index is above F28.
    InvalidFunction,
    /// The function action bits name no action.
    InvalidFunctionAction,
    /// The CV number is out of range.
    InvalidCvAddress,
}

impl fmt::Display for ParseError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::FrameTooShort => "Z21 frame is too short",
            Self::LenMismatch => "Z21 frame length does not match its header",
            Self::BadXBusChecksum => "Z21 X-Bus checksum is invalid",
            Self::InvalidAddress => "Z21 locomotive address is invalid",
            Self::InvalidSpeed => "Z21 locomotive speed is invalid",
            Self::InvalidFunction => "Z21 locomotive function is invalid",
            Self::InvalidFunctionAction => "Z21 function action is invalid",
            Self::InvalidCvAddress => "Z21 CV address is invalid",
        };
        formatter.write_str(message)
    }
}

impl core::error::Error for ParseError {}
