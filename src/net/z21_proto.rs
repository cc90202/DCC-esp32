//! Z21 LAN Protocol v1.13 — pure no_std parsing and encoding.
//!
//! This module has no Embassy or ESP dependencies and compiles on host for testing.
//!
//! # Overview
//!
//! Z21 is a UDP-based remote control protocol for model railroad command stations.
//! This module provides parsing of incoming Z21 frames and generation of responses,
//! implementing the core commands from the Z21 LAN Protokoll v1.13 specification.
//!
//! # Examples
//!
//! **Parse an incoming Z21 UDP frame:**
//!
//! ```no_run
//! use dcc_esp32::net::z21_proto::{parse_z21_frame, Z21Command};
//!
//! // Raw UDP frame from Z21 app (e.g. SetLocoDrive command)
//! let frame = &[
//!     0x09, 0x00,       // Length: 9 bytes
//!     0x40, 0x00,       // Header: 0x0040 (XBus)
//!     0xE4,             // XBus header: SetLocoDrive
//!     // ... address, speed, direction bytes
//! ];
//!
//! match parse_z21_frame(frame) {
//!     Ok(Z21Command::SetLocoDrive { address, speed, direction, format }) => {
//!         println!("Loco {} speed {}", address.value(), speed);
//!     }
//!     Ok(cmd) => println!("Received: {:?}", cmd),
//!     Err(e) => println!("Parse error: {:?}", e),
//! }
//! ```
//!
//! **Generate a Z21 response frame:**
//!
//! ```no_run
//! use dcc_esp32::net::z21_proto::encode_system_state;
//!
//! // Encode system state response (track power on, running)
//! let response = encode_system_state(true, true);
//! // response is a Vec<u8> with Z21 framing
//! ```
//!
//! # Command Coverage
//!
//! Implemented commands:
//! - **GetSerialNumber** — Returns command station serial
//! - **GetCode** — Returns firmware version/code
//! - **GetHwInfo** — Returns hardware info (model, version)
//! - **GetSystemState** — Returns track power, running state
//! - **SetBroadcastFlags** — Configure which events to broadcast
//! - **SetTrackPowerOn/Off** — Control main track output
//! - **SetStop** — Broadcast e-stop
//! - **SetLocoEstop** — Emergency stop specific loco
//! - **SetLocoDrive** — Control loco speed/direction
//! - **SetLocoFunction** — Control loco functions (lights, horn, etc.)
//! - **GetLocoInfo** — Query loco speed/function state
//! - **GetTurnoutInfo** — Query accessory decoder state (returns unknown)
//!
//! # Error Handling
//!
//! [`parse_z21_frame`] returns [`ParseError`] for malformed frames:
//! - `FrameTooShort` — Less than 4 bytes
//! - `LenMismatch` — Header length doesn't match frame size
//! - `BadXBusChecksum` — XBus frame has wrong checksum
//! - `InvalidAddress` — DCC address out of valid range

use crate::dcc::{DccAddress, Direction, SpeedFormat, logical_to_z21_wire, z21_wire_to_logical};
use crate::net::{LocoSlots, LocoState};

// ── Z21 LAN header constants ─────────────────────────────────────────────────
// Source: Z21 LAN Protocol Specification v1.13 (docs/specs/z21-lan-protokoll-en.pdf)

const HEADER_GET_SERIAL_NUMBER: u16 = 0x0010;
// 0x0012 is not in the v1.13 spec but some app versions send it as GetCode.
const HEADER_GET_CODE_LEGACY: u16 = 0x0012; // app-compat alias
const HEADER_GET_CODE: u16 = 0x0018; // per spec §2.21
const HEADER_GET_HWINFO: u16 = 0x001A;
const HEADER_LOGOFF: u16 = 0x0030;
const HEADER_SET_BROADCAST_FLAGS: u16 = 0x0050;
pub(crate) const HEADER_XBUS: u16 = 0x0040;
pub(crate) const HEADER_SYSTEMSTATE_GETDATA: u16 = 0x0085;

// XBus X-Header values (used inside header=0x0040 frames)
const XHEADER_GET_VERSION: u8 = 0x21; // sub-commands dispatched by DB0 below
const XHEADER_TURNOUT_INFO: u8 = 0x43; // shared by LAN_X_GET_TURNOUT_INFO (request) and LAN_X_TURNOUT_INFO (response)
const XHEADER_SET_STOP: u8 = 0x80;
const XHEADER_SET_LOCO_ESTOP: u8 = 0x92;
const XHEADER_GET_LOCO_INFO: u8 = 0xE3;
const XHEADER_SET_LOCO_DRIVE: u8 = 0xE4;

// DB0 sub-commands under XHEADER_GET_VERSION (0x21)
const DB0_GET_XBUS_VERSION: u8 = 0x21;
const DB0_GET_STATUS: u8 = 0x24;
const DB0_SET_TRACK_POWER_OFF: u8 = 0x80;
const DB0_SET_TRACK_POWER_ON: u8 = 0x81;

// DB0 sub-commands under XHEADER_SET_LOCO_DRIVE (0xE4)
const DB0_LOCO_SPEED28: u8 = 0x12;
const DB0_LOCO_SPEED128: u8 = 0x13;
const DB0_LOCO_FUNCTION: u8 = 0xF8;

// DB0 discriminator under XHEADER_GET_LOCO_INFO (0xE3)
const DB0_GET_LOCO_INFO: u8 = 0xF0;

// Z21 response headers (always 0x0040 for XBus replies; others echo the request header)
const RESP_GET_SERIAL_NUMBER: u16 = 0x0010;
const RESP_GET_CODE: u16 = 0x0018; // per spec §2.21
const RESP_SYSTEMSTATE: u16 = 0x0084;
const RESP_LOCO_INFO: u16 = 0x0040;
const RESP_BC_XBUS: u16 = 0x0040; // shared header for all XBus broadcast/response frames

// Response XBus X-Header values
const RESP_XH_LOCO_INFO: u8 = 0xEF;
const RESP_XH_TRACK_POWER: u8 = 0x61; // BC_TRACK_POWER_ON/OFF and UNKNOWN_COMMAND
const RESP_XH_BC_STOPPED: u8 = 0x81;
const RESP_XH_STATUS_CHANGED: u8 = 0x62;
const RESP_XH_VERSION: u8 = 0x63;

// Response DB0 values
const RESP_DB0_POWER_ON: u8 = 0x01;
const RESP_DB0_POWER_OFF: u8 = 0x00;
const RESP_DB0_STATUS_CHANGED: u8 = 0x22;
const RESP_DB0_UNKNOWN_COMMAND: u8 = 0x82;

// CentralState bitmask (spec §2.2 — used in encode_status and encode_system_state)
const CS_EMERGENCY_STOP: u8 = 0x01;
const CS_TRACK_VOLTAGE_OFF: u8 = 0x02;
const CS_SHORT_CIRCUIT: u8 = 0x04;

// Device identity (reported to Z21 apps)
const XBUS_VERSION: u8 = 0x30; // V3.0
const CMDST_ID: u8 = 0x12; // Z21 device family
const HW_TYPE: u32 = 0x00000200; // Z21 black (full-featured)
const FW_VERSION: u32 = 0x00000140; // firmware 1.40

// ── Public types ─────────────────────────────────────────────────────────────

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
    GetStatus,
    SetBroadcastFlags {
        flags: u32,
    },
    SetTrackPowerOn,
    SetTrackPowerOff,
    SetStop,
    SetLocoEstop {
        address: DccAddress,
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
    /// LAN_X_GET_TURNOUT_INFO — accessory decoder state request.
    /// We have no accessory decoder support; respond with state=0 (unknown).
    GetTurnoutInfo {
        address: u16,
    },
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
}

// ── Parsing ──────────────────────────────────────────────────────────────────

/// Parse a single Z21 frame from `buf`.
///
/// `buf` must start at the first byte of the frame. Only the first `data_len`
/// bytes are consumed; any trailing bytes (additional frames in the same
/// UDP datagram) are ignored. The caller is responsible for iterating over
/// multiple frames using [`iter_frames`].
///
/// # Errors
/// Returns:
/// - [`ParseError::FrameTooShort`] when the frame is shorter than required
///   for the declared message type.
/// - [`ParseError::LenMismatch`] when the `DataLen` field is invalid for `buf`.
/// - [`ParseError::BadXBusChecksum`] for malformed XBus payload checksum.
/// - [`ParseError::InvalidAddress`] when a decoded loco address is out of range.
/// - [`ParseError::InvalidFunction`] when a loco function index is outside F0..F28.
pub fn parse_frame(buf: &[u8]) -> Result<Z21Command, ParseError> {
    if buf.len() < 4 {
        return Err(ParseError::FrameTooShort);
    }
    let data_len = u16::from_le_bytes([buf[0], buf[1]]) as usize;
    if data_len < 4 || data_len > buf.len() {
        return Err(ParseError::LenMismatch);
    }
    // Slice to exactly this frame's bytes so XBus checksum covers only its data.
    let frame = &buf[..data_len];
    let header = u16::from_le_bytes([frame[2], frame[3]]);

    match header {
        HEADER_GET_SERIAL_NUMBER => Ok(Z21Command::GetSerialNumber),
        HEADER_GET_CODE | HEADER_GET_CODE_LEGACY => Ok(Z21Command::GetCode),
        HEADER_GET_HWINFO => Ok(Z21Command::GetHwInfo),
        HEADER_LOGOFF => Ok(Z21Command::Logoff),
        HEADER_SYSTEMSTATE_GETDATA => Ok(Z21Command::GetSystemState),
        HEADER_SET_BROADCAST_FLAGS => {
            if frame.len() < 8 {
                return Err(ParseError::FrameTooShort);
            }
            let flags = u32::from_le_bytes([frame[4], frame[5], frame[6], frame[7]]);
            Ok(Z21Command::SetBroadcastFlags { flags })
        }
        HEADER_XBUS => parse_xbus(frame),
        _ => Ok(Z21Command::Unknown),
    }
}

/// Return the byte length of the first frame in `buf`, or `None` if the
/// buffer is too short or contains an invalid length field.
fn frame_len(buf: &[u8]) -> Option<usize> {
    if buf.len() < 4 {
        return None;
    }
    let len = u16::from_le_bytes([buf[0], buf[1]]) as usize;
    if len < 4 || len > buf.len() {
        None
    } else {
        Some(len)
    }
}

/// Iterate over all Z21 frames packed into a single UDP datagram.
///
/// The Z21 protocol allows multiple frames to be concatenated in one UDP
/// datagram. This iterator yields each frame's byte slice in order.
pub struct FrameIter<'a> {
    buf: &'a [u8],
}

impl<'a> Iterator for FrameIter<'a> {
    type Item = &'a [u8];
    fn next(&mut self) -> Option<Self::Item> {
        let len = frame_len(self.buf)?;
        let frame = &self.buf[..len];
        self.buf = &self.buf[len..];
        Some(frame)
    }
}

/// Build an iterator over all Z21 frames in a UDP datagram buffer.
pub fn iter_frames(buf: &[u8]) -> FrameIter<'_> {
    FrameIter { buf }
}

/// XOR checksum over bytes from x_header onwards (excluding the checksum byte).
fn xbus_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc ^ b)
}

fn parse_set_loco_drive_command(payload: &[u8], db0: u8) -> Result<Z21Command, ParseError> {
    let addr = parse_loco_address(payload[2], payload[3]).ok_or(ParseError::InvalidAddress)?;
    let speed_byte = payload[4];
    let direction = if (speed_byte & 0x80) != 0 {
        Direction::Forward
    } else {
        Direction::Reverse
    };
    let raw_speed = speed_byte & 0x7F;
    let (speed, format) = if db0 == DB0_LOCO_SPEED28 {
        (z21_wire_to_logical(raw_speed), SpeedFormat::Speed28)
    } else {
        let logical_speed = match raw_speed {
            0 | 1 => 0,
            _ => raw_speed - 1,
        };
        (logical_speed, SpeedFormat::Speed128)
    };

    Ok(Z21Command::SetLocoDrive {
        address: addr,
        speed,
        direction,
        format,
    })
}

fn parse_set_loco_function_command(payload: &[u8]) -> Result<Z21Command, ParseError> {
    // payload: [0xE4, 0xF8, AddrH, AddrL, FuncByte, checksum]
    let addr = parse_loco_address(payload[2], payload[3]).ok_or(ParseError::InvalidAddress)?;
    let func_byte = payload[4];
    let function = func_byte & 0x3F;
    if function > 28 {
        return Err(ParseError::InvalidFunction);
    }
    let action = match (func_byte >> 6) & 0x03 {
        0 => FunctionAction::Off,
        1 => FunctionAction::On,
        2 => FunctionAction::Toggle,
        _ => FunctionAction::Off,
    };
    Ok(Z21Command::SetLocoFunction {
        address: addr,
        function,
        action,
    })
}

fn parse_set_loco_drive_or_function(payload: &[u8]) -> Result<Z21Command, ParseError> {
    // payload: [0xE4, DB0, AddrH, AddrL, Data, checksum]
    if payload.len() < 6 {
        return Err(ParseError::FrameTooShort);
    }

    let db0 = payload[1];
    match db0 {
        DB0_LOCO_SPEED28 | DB0_LOCO_SPEED128 => parse_set_loco_drive_command(payload, db0),
        DB0_LOCO_FUNCTION => parse_set_loco_function_command(payload),
        _ => Ok(Z21Command::Unknown),
    }
}

fn parse_xbus(buf: &[u8]) -> Result<Z21Command, ParseError> {
    // XBus payload starts at buf[4]: X-Header, then DB0..DBn, checksum
    if buf.len() < 6 {
        return Err(ParseError::FrameTooShort);
    }
    let xheader = buf[4];
    let payload = &buf[4..]; // includes xheader, data bytes, checksum

    // Checksum: XOR of all bytes from xheader through last-before-checksum
    let expected_cs = xbus_checksum(&payload[..payload.len() - 1]);
    let actual_cs = payload[payload.len() - 1];
    if expected_cs != actual_cs {
        return Err(ParseError::BadXBusChecksum);
    }

    match xheader {
        XHEADER_GET_VERSION => {
            // Needs at least xheader + DB0 + checksum = 3 bytes in payload
            if payload.len() < 3 {
                return Err(ParseError::FrameTooShort);
            }
            let db0 = payload[1];
            match db0 {
                DB0_GET_XBUS_VERSION => Ok(Z21Command::GetXBusVersion),
                DB0_GET_STATUS => Ok(Z21Command::GetStatus),
                DB0_SET_TRACK_POWER_OFF => Ok(Z21Command::SetTrackPowerOff),
                DB0_SET_TRACK_POWER_ON => Ok(Z21Command::SetTrackPowerOn),
                _ => Ok(Z21Command::Unknown),
            }
        }
        XHEADER_TURNOUT_INFO => {
            // payload: [0x43, AddrH, AddrL, checksum]
            if payload.len() < 4 {
                return Err(ParseError::FrameTooShort);
            }
            let address = u16::from(payload[1]) << 8 | u16::from(payload[2]);
            Ok(Z21Command::GetTurnoutInfo { address })
        }
        XHEADER_SET_STOP => Ok(Z21Command::SetStop),
        XHEADER_SET_LOCO_ESTOP => {
            // payload: [0x92, AddrH, AddrL, checksum]
            if payload.len() < 4 {
                return Err(ParseError::FrameTooShort);
            }
            let addr =
                parse_loco_address(payload[1], payload[2]).ok_or(ParseError::InvalidAddress)?;
            Ok(Z21Command::SetLocoEstop { address: addr })
        }
        XHEADER_GET_LOCO_INFO => {
            // payload: [0xE3, 0xF0, AddrH, AddrL, checksum]
            if payload.len() < 5 {
                return Err(ParseError::FrameTooShort);
            }
            if payload[1] != DB0_GET_LOCO_INFO {
                return Ok(Z21Command::Unknown);
            }
            let addr =
                parse_loco_address(payload[2], payload[3]).ok_or(ParseError::InvalidAddress)?;
            Ok(Z21Command::GetLocoInfo { address: addr })
        }
        XHEADER_SET_LOCO_DRIVE => parse_set_loco_drive_or_function(payload),
        _ => Ok(Z21Command::Unknown),
    }
}

/// Decode a Z21 wire address (2-byte format: high byte, low byte).
///
/// - Short address: high byte = 0x00, low byte = 1-127
/// - Long address:  high byte has bit7..bit6 = 0b11, range 128-10239
fn parse_loco_address(high: u8, low: u8) -> Option<DccAddress> {
    if (high & 0xC0) == 0xC0 {
        // Long address: strip top 2 bits from high, combine with low
        let addr = (((high & 0x3F) as u16) << 8) | (low as u16);
        DccAddress::new_long(addr)
    } else {
        // Short address
        let addr = low;
        DccAddress::new_short(addr)
    }
}

// ── Encoding ─────────────────────────────────────────────────────────────────

/// Write a little-endian u16 into buf[pos..pos+2].
#[inline]
fn write_u16_le(buf: &mut [u8], pos: usize, val: u16) {
    buf[pos] = (val & 0xFF) as u8;
    buf[pos + 1] = (val >> 8) as u8;
}

/// Write a little-endian u32 into buf[pos..pos+4].
#[inline]
fn write_u32_le(buf: &mut [u8], pos: usize, val: u32) {
    buf[pos] = (val & 0xFF) as u8;
    buf[pos + 1] = ((val >> 8) & 0xFF) as u8;
    buf[pos + 2] = ((val >> 16) & 0xFF) as u8;
    buf[pos + 3] = ((val >> 24) & 0xFF) as u8;
}

/// Write a Z21 frame header: DataLen (LE u16) + Header (LE u16).
#[inline]
fn write_frame_header(buf: &mut [u8], data_len: u16, header: u16) {
    write_u16_le(buf, 0, data_len);
    write_u16_le(buf, 2, header);
}

/// Compute and write XBus checksum at the end of the XBus payload slice.
///
/// `xbus_payload` is the slice starting from X-Header byte up to but not
/// including the checksum byte. The checksum is written at `xbus_payload.len()`.
fn write_xbus_checksum(buf: &mut [u8], xbus_start: usize, xbus_payload_len: usize) {
    let cs = xbus_checksum(&buf[xbus_start..xbus_start + xbus_payload_len]);
    buf[xbus_start + xbus_payload_len] = cs;
}

/// Encode `LAN_X_LOCO_INFO` into `out`. Returns bytes written, or 0 if `out` is shorter than 14.
///
/// Wire format per spec §4.4:
/// ```
/// [DataLen:2][Header:0x0040:2][XH:0xEF]
/// [DB0:AdrH][DB1:AdrL][DB2:0000BKKK][DB3:RVVVVVVV]
/// [DB4:0DSLFGHJ][DB5:F5-F12][DB6:F13-F20][DB7:F21-F28][XCS]
/// ```
/// KKK: 0=14-step, 2=28-step, 4=128-step.  DB4: L=F0, F=F4, G=F3, H=F2, J=F1.
/// Total: 14 bytes.
pub fn encode_loco_info(state: &LocoState, out: &mut [u8]) -> usize {
    const LEN: usize = 14;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, RESP_LOCO_INFO);
    out[4] = RESP_XH_LOCO_INFO;

    // DB0, DB1: address
    let addr_val = state.address.value();
    if state.address.is_long() {
        out[5] = 0xC0 | ((addr_val >> 8) as u8);
        out[6] = (addr_val & 0xFF) as u8;
    } else {
        out[5] = 0x00;
        out[6] = (addr_val & 0xFF) as u8;
    }

    // DB2: 0000BKKK — B=0 (not busy), KKK=speed steps format
    let kkk: u8 = match state.format {
        SpeedFormat::Speed28 => 2,
        SpeedFormat::Speed128 => 4,
    };
    out[7] = kkk;

    // DB3: RVVVVVVV — direction + speed, encoded in the native format
    let wire_speed: u8 = match state.format {
        SpeedFormat::Speed128 => {
            if state.speed == 0 {
                0
            } else {
                state.speed.saturating_add(1) & 0x7F
            }
        }
        SpeedFormat::Speed28 => logical_to_z21_wire(state.speed).unwrap_or(0),
    };
    let dir_bit: u8 = if state.direction == Direction::Forward {
        0x80
    } else {
        0x00
    };
    out[8] = dir_bit | (wire_speed & 0x7F);

    // DB4: 0DSLFGHJ — D=double traction(0), S=smartsearch(0), L=F0, F=F4, G=F3, H=F2, J=F1
    let f = state.functions;
    let f0 = (f & 1) as u8;
    let f1 = ((f >> 1) & 1) as u8;
    let f2 = ((f >> 2) & 1) as u8;
    let f3 = ((f >> 3) & 1) as u8;
    let f4 = ((f >> 4) & 1) as u8;
    out[9] = (f0 << 4) | (f4 << 3) | (f3 << 2) | (f2 << 1) | f1;

    // DB5: F5(bit0)..F12(bit7)
    out[10] = ((f >> 5) & 0xFF) as u8;

    // DB6: F13(bit0)..F20(bit7)
    out[11] = ((f >> 13) & 0xFF) as u8;

    // DB7: F21(bit0)..F28(bit7)
    out[12] = ((f >> 21) & 0xFF) as u8;

    // XBus checksum: XOR of bytes [4..13)
    write_xbus_checksum(out, 4, 9);

    LEN
}

/// Encode `LAN_X_BC_TRACK_POWER_ON` or `LAN_X_BC_TRACK_POWER_OFF` into `out`.
/// Returns bytes written (7), or 0 if `out` is shorter than 7.
pub fn encode_bc_track_power(on: bool, out: &mut [u8]) -> usize {
    const LEN: usize = 7;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, RESP_BC_XBUS);
    out[4] = RESP_XH_TRACK_POWER;
    if on {
        out[5] = RESP_DB0_POWER_ON;
    } else {
        out[5] = RESP_DB0_POWER_OFF;
    }
    write_xbus_checksum(out, 4, 2);
    LEN
}

/// Encode `LAN_X_BC_STOPPED` into `out`. Returns bytes written (6), or 0 if `out` is shorter than 6.
pub fn encode_bc_stopped(out: &mut [u8]) -> usize {
    const LEN: usize = 6;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, RESP_BC_XBUS);
    out[4] = RESP_XH_BC_STOPPED;
    write_xbus_checksum(out, 4, 1);
    LEN
}

/// Encode `LAN_X_UNKNOWN_COMMAND` into `out`. Returns bytes written (7), or 0 if `out` is shorter than 7.
pub fn encode_unknown_command(out: &mut [u8]) -> usize {
    const LEN: usize = 7;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, RESP_BC_XBUS);
    out[4] = RESP_XH_TRACK_POWER;
    out[5] = RESP_DB0_UNKNOWN_COMMAND;
    write_xbus_checksum(out, 4, 2);
    LEN
}

/// Encode `LAN_X_TURNOUT_INFO` into `out`. Returns bytes written (9), or 0 if `out` is shorter than 9.
///
/// XBus reply, header=0x0040, X-Header=0x43 (same byte as request).
/// DB2=0x00 means state unknown — we have no accessory decoder support.
/// Spec section: Z21 LAN Protocol v1.13, accessory decoder chapter.
pub fn encode_turnout_info(address: u16, out: &mut [u8]) -> usize {
    // [DataLen:2][Header=0x0040:2][0x43][AddrH][AddrL][DB2=0x00][XCS]
    const LEN: usize = 9;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, RESP_BC_XBUS);
    out[4] = XHEADER_TURNOUT_INFO;
    out[5] = (address >> 8) as u8;
    out[6] = (address & 0xFF) as u8;
    out[7] = 0x00; // state: unknown
    write_xbus_checksum(out, 4, 4);
    LEN
}

/// Encode `LAN_SERIAL_NUMBER` into `out`. Returns bytes written (8), or 0 if `out` is shorter than 8.
pub fn encode_serial_number(serial: u32, out: &mut [u8]) -> usize {
    const LEN: usize = 8;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, RESP_GET_SERIAL_NUMBER);
    write_u32_le(out, 4, serial);
    LEN
}

/// Encode `LAN_X_GET_VERSION` reply into `out`. Returns bytes written (9), or 0 if `out` is shorter than 9.
///
/// Per spec §2.3: XBus reply, header=0x0040, X-Header=0x63.
/// DB0=0x21 (echo), DB1=XBUS_VER=0x30 (V3.0), DB2=CMDST_ID=0x12 (Z21 family).
pub fn encode_xbus_version(out: &mut [u8]) -> usize {
    // [DataLen:2][Header=0x0040:2][0x63][0x21][0x30][0x12][XCS]
    const LEN: usize = 9;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, RESP_BC_XBUS);
    out[4] = RESP_XH_VERSION;
    out[5] = DB0_GET_XBUS_VERSION; // DB0 echo
    out[6] = XBUS_VERSION;
    out[7] = CMDST_ID;
    write_xbus_checksum(out, 4, 4);
    LEN
}

/// Encode `LAN_GET_CODE` response into `out`. Returns bytes written (5), or 0 if `out` is shorter than 5.
///
/// Code=0x00 means no feature restrictions — all Z21 functionality available.
pub fn encode_code(out: &mut [u8]) -> usize {
    const LEN: usize = 5;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, RESP_GET_CODE);
    out[4] = 0x00; // no feature lock
    LEN
}

/// Encode `LAN_GET_HWINFO` response into `out`. Returns bytes written (12), or 0 if `out` is shorter than 12.
///
/// Reports HW_TYPE = 0x00000200 (Z21 black, full-featured) so the app enables
/// all controls. FW_VERSION = 1.40 (consistent with `encode_xbus_version`).
pub fn encode_hwinfo(out: &mut [u8]) -> usize {
    const LEN: usize = 12;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, HEADER_GET_HWINFO);
    write_u32_le(out, 4, HW_TYPE);
    write_u32_le(out, 8, FW_VERSION);
    LEN
}

/// Encode `LAN_X_STATUS_CHANGED` into `out`. Returns bytes written (8), or 0 if `out` is shorter than 8.
///
/// Per spec §2.12: XBus reply, header=0x0040, X-Header=0x62, DB0=0x22, DB1=Status, XCS.
/// Bitmask: bit0=EmergencyStop, bit1=TrackVoltageOff, bit2=ShortCircuit, bit5=ProgrammingMode.
pub fn encode_status(track_on: bool, estop: bool, out: &mut [u8]) -> usize {
    // [DataLen:2][Header=0x0040:2][0x62][0x22][Status][XCS]
    const LEN: usize = 8;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, RESP_BC_XBUS);
    out[4] = RESP_XH_STATUS_CHANGED;
    out[5] = RESP_DB0_STATUS_CHANGED;
    let mut status: u8 = 0;
    if estop {
        status |= CS_EMERGENCY_STOP;
    }
    if !track_on {
        status |= CS_TRACK_VOLTAGE_OFF;
    }
    out[6] = status;
    write_xbus_checksum(out, 4, 3);
    LEN
}

/// Encode `LAN_SYSTEMSTATE_DATACHANGED` (0x0084) into `out`. Returns bytes written (20), or 0 if `out` is shorter than 20.
///
/// Layout (Z21 spec §2.2):
///   [4-5]  MainCurrent         i16 mA  (we report 0 — no live ADC path here)
///   [6-7]  ProgCurrent         i16 mA
///   [8-9]  FilteredMainCurrent i16 mA
///   [10-11] Temperature        i16 °C  (nominal 25)
///   [12-13] SupplyVoltage      u16 mV  (nominal 18 V)
///   [14-15] VCCVoltage         u16 mV  (nominal 3.3 V)
///   [16]   CentralState: bit0=EmergencyStop, bit1=TrackVoltageOff, bit2=ShortCircuit
///   [17]   CentralStateEx (0)
///   [18-19] reserved
pub fn encode_system_state(
    track_on: bool,
    estop: bool,
    short_circuit: bool,
    out: &mut [u8],
) -> usize {
    const LEN: usize = 20;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, RESP_SYSTEMSTATE);
    // MainCurrent, ProgCurrent, FilteredMainCurrent — all 0 (no ADC)
    out[4] = 0;
    out[5] = 0;
    out[6] = 0;
    out[7] = 0;
    out[8] = 0;
    out[9] = 0;
    // Temperature: 25 °C
    out[10] = 25;
    out[11] = 0;
    // SupplyVoltage: 18000 mV
    write_u16_le(out, 12, 18000);
    // VCCVoltage: 3300 mV
    write_u16_le(out, 14, 3300);
    // CentralState: bit0=EmergencyStop, bit1=TrackVoltageOff, bit2=ShortCircuit
    let mut cs: u8 = 0;
    if estop {
        cs |= CS_EMERGENCY_STOP;
    }
    if !track_on {
        cs |= CS_TRACK_VOLTAGE_OFF;
    }
    if short_circuit {
        cs |= CS_SHORT_CIRCUIT;
    }
    out[16] = cs;
    out[17] = 0; // CentralStateEx
    out[18] = 0;
    out[19] = 0;
    LEN
}

// ── Slot management ───────────────────────────────────────────────────────────

/// Find an existing slot for `addr` or insert a new one.
///
/// Eviction policy: if all slots are full, evict the first slot with speed=0.
/// If all slots have speed>0, returns None (command dropped).
pub fn find_or_insert(slots: &mut LocoSlots, addr: DccAddress) -> Option<&mut LocoState> {
    // Pass 1: find existing slot.
    if let Some(i) = slots
        .iter()
        .position(|s| s.as_ref().is_some_and(|s| s.address == addr))
    {
        return slots[i].as_mut();
    }

    // Pass 2: find empty slot.
    if let Some(i) = slots.iter().position(|s| s.is_none()) {
        slots[i] = Some(LocoState {
            address: addr,
            speed: 0,
            direction: Direction::Forward,
            format: SpeedFormat::Speed128,
            functions: 0,
        });
        return slots[i].as_mut();
    }

    // Pass 3: evict first stopped slot (speed == 0).
    if let Some(i) = slots
        .iter()
        .position(|s| s.as_ref().is_some_and(|s| s.speed == 0))
    {
        slots[i] = Some(LocoState {
            address: addr,
            speed: 0,
            direction: Direction::Forward,
            format: SpeedFormat::Speed128,
            functions: 0,
        });
        return slots[i].as_mut();
    }

    None
}

/// Find an existing slot for `addr` (read-only).
pub fn find_slot(slots: &LocoSlots, addr: DccAddress) -> Option<&LocoState> {
    slots.iter().flatten().find(|s| s.address == addr)
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn addr_short(n: u8) -> DccAddress {
        DccAddress::new_short(n).unwrap()
    }

    fn addr_long(n: u16) -> DccAddress {
        DccAddress::new_long(n).unwrap()
    }

    // Helper: build a minimal valid 4-byte frame with just header
    fn frame_header_only(header: u16) -> [u8; 4] {
        let len = 4u16;
        [
            len as u8,
            (len >> 8) as u8,
            header as u8,
            (header >> 8) as u8,
        ]
    }

    // Helper: compute xbus checksum over a slice
    fn cs(data: &[u8]) -> u8 {
        data.iter().fold(0u8, |a, &b| a ^ b)
    }

    // Helper: build an XBus frame
    fn xbus_frame(xheader: u8, extra: &[u8]) -> heapless::Vec<u8, 64> {
        let mut payload: heapless::Vec<u8, 64> = heapless::Vec::new();
        let _ = payload.push(xheader);
        for &b in extra {
            let _ = payload.push(b);
        }
        let checksum = cs(&payload);
        let _ = payload.push(checksum);

        let data_len = (4 + payload.len()) as u16;
        let mut frame: heapless::Vec<u8, 64> = heapless::Vec::new();
        let _ = frame.push(data_len as u8);
        let _ = frame.push((data_len >> 8) as u8);
        let _ = frame.push(HEADER_XBUS as u8);
        let _ = frame.push((HEADER_XBUS >> 8) as u8);
        for b in &payload {
            let _ = frame.push(*b);
        }
        frame
    }

    // ── Parse tests ──────────────────────────────────────────────────────────

    #[test]
    fn test_parse_get_serial_number() {
        let frame = frame_header_only(HEADER_GET_SERIAL_NUMBER);
        assert_eq!(parse_frame(&frame), Ok(Z21Command::GetSerialNumber));
    }

    #[test]
    fn test_parse_get_hwinfo() {
        let frame = frame_header_only(HEADER_GET_HWINFO);
        assert_eq!(parse_frame(&frame), Ok(Z21Command::GetHwInfo));
    }

    #[test]
    fn test_parse_set_broadcast_flags() {
        // DataLen=8, header=0x0050, flags=0x00000001 (LE)
        let buf = [0x08, 0x00, 0x50, 0x00, 0x01, 0x00, 0x00, 0x00];
        let Ok(Z21Command::SetBroadcastFlags { flags }) = parse_frame(&buf) else {
            panic!("expected SetBroadcastFlags, got {:?}", parse_frame(&buf));
        };
        assert_eq!(flags, 1);
    }

    #[test]
    fn test_parse_set_broadcast_flags_too_short() {
        // Only 4 bytes — missing the flags payload
        let buf = [0x04, 0x00, 0x50, 0x00];
        assert_eq!(parse_frame(&buf), Err(ParseError::FrameTooShort));
    }

    #[test]
    fn test_encode_hwinfo() {
        let mut buf = [0u8; 32];
        let n = encode_hwinfo(&mut buf);
        assert_eq!(n, 12);
        assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 12);
        assert_eq!(u16::from_le_bytes([buf[2], buf[3]]), 0x001A);
        // HW_TYPE = Z21 black = 0x00000200
        assert_eq!(
            u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            0x00000200
        );
        // FW_VERSION = 1.40 = 0x00000140
        assert_eq!(
            u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            0x00000140
        );
    }

    #[test]
    fn test_parse_get_xbus_version() {
        let frame = xbus_frame(0x21, &[0x21]);
        assert_eq!(parse_frame(&frame), Ok(Z21Command::GetXBusVersion));
    }

    #[test]
    fn test_parse_set_track_power_on() {
        // Per spec §2.6: XBus X-Header=0x21, DB0=0x81, XCS=0xA0
        let frame = xbus_frame(0x21, &[0x81]);
        assert_eq!(parse_frame(&frame), Ok(Z21Command::SetTrackPowerOn));
    }

    #[test]
    fn test_parse_set_track_power_off() {
        // Per spec §2.5: XBus X-Header=0x21, DB0=0x80, XCS=0xA1
        let frame = xbus_frame(0x21, &[0x80]);
        assert_eq!(parse_frame(&frame), Ok(Z21Command::SetTrackPowerOff));
    }

    #[test]
    fn test_parse_set_loco_drive_28_short_addr() {
        // SetLocoDrive Speed28: addr=3, forward, speed step 14
        // NMRA encoding: N=14 (even) → v=((14+1)>>1)+1=8, |0x10=0x18; fwd=bit7 → 0x98
        let frame = xbus_frame(0xE4, &[0x12, 0x00, 0x03, 0x98]);
        let Ok(Z21Command::SetLocoDrive {
            address,
            speed,
            direction,
            format,
        }) = parse_frame(&frame)
        else {
            panic!("expected SetLocoDrive, got {:?}", parse_frame(&frame));
        };
        assert_eq!(address, addr_short(3));
        assert_eq!(direction, Direction::Forward);
        assert_eq!(format, SpeedFormat::Speed28);
        assert_eq!(speed, 14);
    }

    #[test]
    fn test_parse_set_loco_drive_28_long_addr() {
        // addr=1000 long: high = 0xC0 | (1000>>8) = 0xC3, low = 0xE8
        // speed=5 step28 (odd): N=5 → v=((5+1)>>1)+1=4, no mask → 0x04; reverse=bit7 clear → 0x04
        let frame = xbus_frame(0xE4, &[0x12, 0xC3, 0xE8, 0x04]);
        let Ok(Z21Command::SetLocoDrive {
            address,
            speed,
            direction,
            format,
        }) = parse_frame(&frame)
        else {
            panic!("expected SetLocoDrive, got {:?}", parse_frame(&frame));
        };
        assert_eq!(address, addr_long(1000));
        assert_eq!(direction, Direction::Reverse);
        assert_eq!(format, SpeedFormat::Speed28);
        assert_eq!(speed, 5);
    }

    #[test]
    fn test_parse_set_loco_drive_128() {
        // addr=5 short, Speed128, forward, wire speed=64 -> logical speed=63
        // SpeedByte = 0x80 | 64 = 0xC0
        let frame = xbus_frame(0xE4, &[0x13, 0x00, 0x05, 0xC0]);
        let Ok(Z21Command::SetLocoDrive {
            address,
            speed,
            direction,
            format,
        }) = parse_frame(&frame)
        else {
            panic!("expected SetLocoDrive, got {:?}", parse_frame(&frame));
        };
        assert_eq!(address, addr_short(5));
        assert_eq!(direction, Direction::Forward);
        assert_eq!(format, SpeedFormat::Speed128);
        assert_eq!(speed, 63);
    }

    #[test]
    fn test_parse_set_loco_function_on() {
        // F5 ON: func_byte = (1<<6) | 5 = 0x45
        let frame = xbus_frame(0xE4, &[0xF8, 0x00, 0x03, 0x45]);
        let Ok(Z21Command::SetLocoFunction {
            address,
            function,
            action,
        }) = parse_frame(&frame)
        else {
            panic!("expected SetLocoFunction, got {:?}", parse_frame(&frame));
        };
        assert_eq!(address, addr_short(3));
        assert_eq!(function, 5);
        assert_eq!(action, FunctionAction::On);
    }

    #[test]
    fn test_parse_set_loco_function_off() {
        // F0 OFF: func_byte = (0<<6) | 0 = 0x00
        let frame = xbus_frame(0xE4, &[0xF8, 0x00, 0x03, 0x00]);
        let Ok(Z21Command::SetLocoFunction {
            action, function, ..
        }) = parse_frame(&frame)
        else {
            panic!("expected SetLocoFunction, got {:?}", parse_frame(&frame));
        };
        assert_eq!(function, 0);
        assert_eq!(action, FunctionAction::Off);
    }

    #[test]
    fn test_parse_set_loco_function_toggle() {
        // F2 Toggle: func_byte = (2<<6) | 2 = 0x82
        let frame = xbus_frame(0xE4, &[0xF8, 0x00, 0x03, 0x82]);
        let Ok(Z21Command::SetLocoFunction {
            function, action, ..
        }) = parse_frame(&frame)
        else {
            panic!("expected SetLocoFunction, got {:?}", parse_frame(&frame));
        };
        assert_eq!(function, 2);
        assert_eq!(action, FunctionAction::Toggle);
    }

    #[test]
    fn test_parse_set_loco_function_rejects_out_of_range_index() {
        // F29 ON: low 6 bits encode 29, which is outside supported F0..F28 range.
        let frame = xbus_frame(0xE4, &[0xF8, 0x00, 0x03, 0x5D]);
        assert_eq!(parse_frame(&frame), Err(ParseError::InvalidFunction));
    }

    #[test]
    fn test_parse_set_stop() {
        let frame = xbus_frame(0x80, &[]);
        assert_eq!(parse_frame(&frame), Ok(Z21Command::SetStop));
    }

    #[test]
    fn test_parse_set_loco_estop() {
        // addr=3 short
        let frame = xbus_frame(0x92, &[0x00, 0x03]);
        let Ok(Z21Command::SetLocoEstop { address }) = parse_frame(&frame) else {
            panic!("expected SetLocoEstop, got {:?}", parse_frame(&frame));
        };
        assert_eq!(address, addr_short(3));
    }

    #[test]
    fn test_parse_get_loco_info() {
        // addr=3 short
        let frame = xbus_frame(0xE3, &[0xF0, 0x00, 0x03]);
        let Ok(Z21Command::GetLocoInfo { address }) = parse_frame(&frame) else {
            panic!("expected GetLocoInfo, got {:?}", parse_frame(&frame));
        };
        assert_eq!(address, addr_short(3));
    }

    // ── Encode tests ─────────────────────────────────────────────────────────

    #[test]
    fn test_encode_loco_info_checksum() {
        let state = LocoState {
            address: addr_short(3),
            speed: 14,
            direction: Direction::Forward,
            format: SpeedFormat::Speed28,
            functions: 0,
        };
        let mut buf = [0u8; 32];
        let n = encode_loco_info(&state, &mut buf);
        assert_eq!(n, 14);
        // DataLen = 14
        assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 14);
        // Header = 0x0040
        assert_eq!(u16::from_le_bytes([buf[2], buf[3]]), 0x0040);
        // X-Header = 0xEF
        assert_eq!(buf[4], 0xEF);
        // Checksum: XOR of buf[4..13)
        let expected_cs = xbus_checksum(&buf[4..13]);
        assert_eq!(buf[13], expected_cs);
    }

    #[test]
    fn test_encode_loco_info_function_layout() {
        // F0=1, F1=1, F4=1
        // functions bitmask: bit0=F0=1, bit1=F1=1, bit4=F4=1
        let funcs: u32 = (1 << 0) | (1 << 1) | (1 << 4);
        let state = LocoState {
            address: addr_short(3),
            speed: 0,
            direction: Direction::Forward,
            format: SpeedFormat::Speed128,
            functions: funcs,
        };
        let mut buf = [0u8; 32];
        encode_loco_info(&state, &mut buf);
        // buf[7] = DB2 = KKK=4 (128-step), B=0
        assert_eq!(buf[7], 4, "DB2 KKK should be 4 for Speed128");
        // buf[8] = DB3 = speed (0, forward) = 0x80
        assert_eq!(buf[8], 0x80, "DB3 should be forward+stop");
        // buf[9] = DB4 = F0(bit4)=1, F4(bit3)=1, F3(bit2)=0, F2(bit1)=0, F1(bit0)=1
        // = 0b00011001 = 0x19
        assert_eq!(buf[9], 0x19, "DB4 function byte mismatch");
    }

    #[test]
    fn test_encode_loco_info_speed28() {
        // NMRA intermediate-bit encoding (Z21 spec §4.2):
        // Step 14 (even): v=((14+1)>>1)+1=8, |0x10=0x18, fwd → 0x80|0x18=0x98
        let state = LocoState {
            address: addr_short(3),
            speed: 14,
            direction: Direction::Forward,
            format: SpeedFormat::Speed28,
            functions: 0,
        };
        let mut buf = [0u8; 32];
        encode_loco_info(&state, &mut buf);
        // DB2: KKK=2 (28-step)
        assert_eq!(buf[7], 2, "DB2 KKK should be 2 for Speed28");
        // DB3: forward(0x80) | 0x18 = 0x98
        assert_eq!(buf[8], 0x98, "Speed28 step14: NMRA wire=0x18, fwd=0x98");

        // Step 0 → stop → wire=0x00, forward = 0x80
        let mut state0 = state;
        state0.speed = 0;
        encode_loco_info(&state0, &mut buf);
        assert_eq!(buf[8], 0x80, "Speed28 stop: DB3=0x80 (fwd, stop)");

        // Step 1 (odd): v=((1+1)>>1)+1=2, no mask → 0x02, fwd → 0x82
        let mut state1 = state;
        state1.speed = 1;
        encode_loco_info(&state1, &mut buf);
        assert_eq!(buf[8], 0x82, "Speed28 step1: wire=0x02, fwd=0x82");

        // Step 28 (even): v=((28+1)>>1)+1=15, |0x10=0x1F, fwd → 0x9F
        let mut state28 = state;
        state28.speed = 28;
        encode_loco_info(&state28, &mut buf);
        assert_eq!(buf[8], 0x9F, "Speed28 step28: NMRA wire=0x1F, fwd=0x9F");
    }

    #[test]
    fn test_encode_bc_track_power_on_off() {
        let mut buf = [0u8; 32];

        let n = encode_bc_track_power(true, &mut buf);
        assert_eq!(n, 7);
        assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 7);
        assert_eq!(buf[4], 0x61);
        assert_eq!(buf[5], 0x01);
        let cs_on = xbus_checksum(&buf[4..6]);
        assert_eq!(buf[6], cs_on);

        let n = encode_bc_track_power(false, &mut buf);
        assert_eq!(n, 7);
        assert_eq!(buf[5], 0x00);
        let cs_off = xbus_checksum(&buf[4..6]);
        assert_eq!(buf[6], cs_off);
    }

    #[test]
    fn test_encode_system_state_central_state_bits() {
        let mut buf = [0u8; 32];

        // Normal: all bits clear
        let n = encode_system_state(true, false, false, &mut buf);
        assert_eq!(n, 20);
        assert_eq!(buf[16], 0x00);

        // E-stop: bit0
        encode_system_state(true, true, false, &mut buf);
        assert_eq!(buf[16], 0x01);

        // Track off: bit1
        encode_system_state(false, false, false, &mut buf);
        assert_eq!(buf[16], 0x02);

        // Short circuit: bit1 + bit2 (track off implied by short)
        encode_system_state(false, false, true, &mut buf);
        assert_eq!(buf[16], 0x06);

        // All flags: bit0 + bit1 + bit2
        encode_system_state(false, true, true, &mut buf);
        assert_eq!(buf[16], 0x07);
    }

    #[test]
    fn test_encode_bc_stopped() {
        let mut buf = [0u8; 32];
        let n = encode_bc_stopped(&mut buf);
        assert_eq!(n, 6);
        assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 6);
        assert_eq!(buf[4], 0x81);
        assert_eq!(buf[5], xbus_checksum(&[0x81]));
    }

    #[test]
    fn test_encode_unknown_command() {
        let mut buf = [0u8; 32];
        let n = encode_unknown_command(&mut buf);
        assert_eq!(n, 7);
        assert_eq!(buf[4], 0x61);
        assert_eq!(buf[5], 0x82);
        let cs = xbus_checksum(&[0x61, 0x82]);
        assert_eq!(buf[6], cs);
    }

    #[test]
    fn test_encode_serial_number() {
        let mut buf = [0u8; 32];
        let n = encode_serial_number(0xDEADBEEF, &mut buf);
        assert_eq!(n, 8);
        assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 8);
        // little-endian
        assert_eq!(buf[4], 0xEF);
        assert_eq!(buf[5], 0xBE);
        assert_eq!(buf[6], 0xAD);
        assert_eq!(buf[7], 0xDE);
    }

    #[test]
    fn test_encode_status_flags() {
        let mut buf = [0u8; 32];

        // XBus LAN_X_STATUS_CHANGED: header=0x0040, X-Header=0x62, DB0=0x22, DB1=Status, XCS
        // Status is at buf[6], X-Header at buf[4], DB0 at buf[5]

        // track on, no estop → status = 0x00
        encode_status(true, false, &mut buf);
        assert_eq!(buf[4], 0x62); // X-Header
        assert_eq!(buf[5], 0x22); // DB0
        assert_eq!(buf[6], 0x00); // status: no flags

        // track off, no estop → status = 0x02 (csTrackVoltageOff)
        encode_status(false, false, &mut buf);
        assert_eq!(buf[6], 0x02);

        // track on, estop → status = 0x01 (csEmergencyStop)
        encode_status(true, true, &mut buf);
        assert_eq!(buf[6], 0x01);

        // track off, estop → status = 0x03
        encode_status(false, true, &mut buf);
        assert_eq!(buf[6], 0x03);
    }

    // ── Error/fuzz tests ─────────────────────────────────────────────────────

    #[test]
    fn test_invalid_frame_too_short() {
        assert_eq!(
            parse_frame(&[0x04, 0x00, 0x10]),
            Err(ParseError::FrameTooShort)
        );
        assert_eq!(parse_frame(&[]), Err(ParseError::FrameTooShort));
    }

    #[test]
    fn test_invalid_frame_len_mismatch() {
        // Says len=8 but only 4 bytes
        let buf = [0x08, 0x00, 0x10, 0x00];
        assert_eq!(parse_frame(&buf), Err(ParseError::LenMismatch));
    }

    #[test]
    fn test_invalid_xbus_checksum() {
        // Build a valid frame then corrupt the checksum
        let mut frame = xbus_frame(0x21, &[0x21]);
        let last = frame.len() - 1;
        let last_byte = frame[last];
        frame[last] = last_byte ^ 0xFF; // corrupt checksum
        assert_eq!(parse_frame(&frame), Err(ParseError::BadXBusChecksum));
    }

    #[test]
    fn test_fuzz_random_bytes() {
        // Various random-ish byte sequences — must not panic, only return Err or Ok(Unknown)
        let cases: &[&[u8]] = &[
            &[0xFF, 0xFF, 0xFF, 0xFF],
            &[0x04, 0x00, 0xFF, 0xFF],
            &[0x06, 0x00, 0x40, 0x00, 0xAA, 0xBB],
            &[0x07, 0x00, 0x40, 0x00, 0x21, 0x21, 0xFF],
            &[0x04, 0x00, 0x00, 0x00],
        ];
        for &buf in cases {
            let _ = parse_frame(buf); // must not panic
        }
    }

    #[test]
    fn test_loco_address_short_encoding() {
        // Short address round-trip in drive frame
        let frame = xbus_frame(0xE4, &[0x13, 0x00, 0x7F, 0x80]);
        let Ok(Z21Command::SetLocoDrive { address, .. }) = parse_frame(&frame) else {
            panic!("expected SetLocoDrive, got {:?}", parse_frame(&frame));
        };
        assert!(address.is_short());
        assert_eq!(address.value(), 127);
    }

    #[test]
    fn test_loco_address_long_encoding() {
        // addr=200: high = 0xC0|(200>>8)=0xC0, low=200
        let frame = xbus_frame(0xE4, &[0x13, 0xC0, 200, 0x80]);
        let Ok(Z21Command::SetLocoDrive { address, .. }) = parse_frame(&frame) else {
            panic!("expected SetLocoDrive, got {:?}", parse_frame(&frame));
        };
        assert!(address.is_long());
        assert_eq!(address.value(), 200);
    }

    #[test]
    fn test_function_bitmask_f0_f28() {
        // Set F0 and F28, encode loco_info, verify DB4 (buf[9]) and DB7 (buf[12])
        let state = LocoState {
            address: addr_short(1),
            speed: 0,
            direction: Direction::Forward,
            format: SpeedFormat::Speed128,
            functions: (1 << 0) | (1 << 28), // F0 and F28
        };
        let mut buf = [0u8; 32];
        encode_loco_info(&state, &mut buf);
        // DB4 (buf[9]): F0 in bit4 → 0b00010000 = 0x10
        assert_eq!(buf[9] & 0x10, 0x10, "F0 should be in DB4 bit4");
        // DB7 (buf[12]): F28 in bit7 → 0x80
        assert_eq!(buf[12] & 0x80, 0x80, "F28 should be in DB7 bit7");
    }

    #[test]
    fn test_encode_loco_info_speed128_avoids_wire_estop_value() {
        let state = LocoState {
            address: addr_short(3),
            speed: 1,
            direction: Direction::Forward,
            format: SpeedFormat::Speed128,
            functions: 0,
        };
        let mut buf = [0u8; 32];
        encode_loco_info(&state, &mut buf);
        assert_eq!(buf[8], 0x82, "logical speed 1 must encode as wire speed 2");
    }

    // ── Slot management tests ─────────────────────────────────────────────────

    fn empty_slots() -> LocoSlots {
        [None; 12]
    }

    #[test]
    fn test_find_or_insert_new_loco() {
        let mut slots = empty_slots();
        let result = find_or_insert(&mut slots, addr_short(3));
        assert!(result.is_some());
        assert_eq!(result.unwrap().address, addr_short(3));
        // Slot should now be occupied
        assert!(find_slot(&slots, addr_short(3)).is_some());
    }

    #[test]
    fn test_find_or_insert_existing_loco() {
        let mut slots = empty_slots();
        // Insert once
        find_or_insert(&mut slots, addr_short(3)).unwrap().speed = 10;
        // Insert same address again
        let result = find_or_insert(&mut slots, addr_short(3));
        assert!(result.is_some());
        // Should return the same slot, speed preserved
        assert_eq!(result.unwrap().speed, 10);
        // Only one slot occupied
        let count = slots.iter().filter(|s| s.is_some()).count();
        assert_eq!(count, 1);
    }

    #[test]
    fn test_find_or_insert_full_evicts_stopped() {
        let mut slots = empty_slots();
        // Fill all 12 slots with different addresses, all speed=0
        for i in 1u8..=12 {
            let a = addr_short(i);
            find_or_insert(&mut slots, a).unwrap().speed = 0;
        }
        // Now insert addr=13 — should evict first stopped slot
        let result = find_or_insert(&mut slots, addr_short(13));
        assert!(result.is_some());
        assert_eq!(result.unwrap().address, addr_short(13));
    }

    // ── Turnout (0x43) tests ──────────────────────────────────────────────────

    #[test]
    fn test_parse_get_turnout_info_address_5() {
        // LAN_X_GET_TURNOUT_INFO: X-Header=0x43, AddrH=0x00, AddrL=0x05
        let frame = xbus_frame(0x43, &[0x00, 0x05]);
        let Ok(Z21Command::GetTurnoutInfo { address }) = parse_frame(&frame) else {
            panic!("expected GetTurnoutInfo, got {:?}", parse_frame(&frame));
        };
        assert_eq!(address, 5);
    }

    #[test]
    fn test_parse_get_turnout_info_too_short() {
        // xbus_frame generates a valid checksum, so the CRC check passes and we
        // reach the payload-length guard inside the XHEADER_TURNOUT_INFO arm.
        // payload = [0x43, 0x00, xcs] = 3 bytes < 4 required → FrameTooShort.
        let frame = xbus_frame(0x43, &[0x00]); // xheader + AddrH only, missing AddrL
        assert_eq!(parse_frame(&frame), Err(ParseError::FrameTooShort));
        // Raw buffer shorter than 6 bytes hits the early guard before checksum.
        let buf = [0x05, 0x00, 0x40, 0x00, 0x43];
        assert_eq!(parse_frame(&buf), Err(ParseError::FrameTooShort));
    }

    #[test]
    fn test_parse_get_turnout_info_high_address() {
        // address = 0x01_00 = 256
        let frame = xbus_frame(0x43, &[0x01, 0x00]);
        let Ok(Z21Command::GetTurnoutInfo { address }) = parse_frame(&frame) else {
            panic!("expected GetTurnoutInfo, got {:?}", parse_frame(&frame));
        };
        assert_eq!(address, 256);
    }

    #[test]
    fn test_encode_turnout_info_unknown_state() {
        // Response for address=5, state=0 (unknown/not-owned)
        // LEN=9, Header=0x0040, X-Header=0x43, AddrH=0x00, AddrL=0x05, DB2=0x00, XCS
        // XCS = 0x43^0x00^0x05^0x00 = 0x46
        let mut buf = [0u8; 16];
        let n = encode_turnout_info(5, &mut buf);
        assert_eq!(n, 9);
        assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 9); // DataLen
        assert_eq!(u16::from_le_bytes([buf[2], buf[3]]), 0x0040); // Header
        assert_eq!(buf[4], 0x43); // X-Header
        assert_eq!(buf[5], 0x00); // AddrH
        assert_eq!(buf[6], 0x05); // AddrL
        assert_eq!(buf[7], 0x00); // DB2: state unknown
        assert_eq!(buf[8], 0x43 ^ 0x05); // XCS
    }

    #[test]
    fn test_find_or_insert_full_all_running() {
        let mut slots = empty_slots();
        // Fill all 12 slots with speed > 0
        for i in 1u8..=12 {
            let a = addr_short(i);
            find_or_insert(&mut slots, a).unwrap().speed = 5;
        }
        // Insert 13th — all running → None
        let result = find_or_insert(&mut slots, addr_short(13));
        assert!(result.is_none());
    }
}
