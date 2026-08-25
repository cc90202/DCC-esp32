use crate::dcc::{DccAddress, Direction, SpeedFormat};

use super::wire::*;
use super::{FrameKind, FunctionAction, ParseError, Z21Command};

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
    let header = frame_kind(frame).header;

    match header {
        HEADER_GET_SERIAL_NUMBER => Ok(Z21Command::GetSerialNumber),
        HEADER_GET_CODE | HEADER_GET_CODE_LEGACY => Ok(Z21Command::GetCode),
        HEADER_GET_HWINFO => Ok(Z21Command::GetHwInfo),
        HEADER_LOGOFF => Ok(Z21Command::Logoff),
        HEADER_SYSTEMSTATE_GETDATA => Ok(Z21Command::GetSystemState),
        HEADER_GET_LOCOMODE => {
            if frame.len() < 6 {
                return Err(ParseError::FrameTooShort);
            }
            let address =
                parse_loco_address(frame[4], frame[5]).ok_or(ParseError::InvalidAddress)?;
            Ok(Z21Command::GetLocoMode { address })
        }
        HEADER_SET_LOCOMODE => {
            if frame.len() < 7 {
                return Err(ParseError::FrameTooShort);
            }
            let address =
                parse_loco_address(frame[4], frame[5]).ok_or(ParseError::InvalidAddress)?;
            Ok(Z21Command::SetLocoMode {
                address,
                mode: frame[6],
            })
        }
        HEADER_RAILCOM_GETDATA => {
            if frame.len() < 7 {
                return Err(ParseError::FrameTooShort);
            }
            let raw_address = u16::from_le_bytes([frame[5], frame[6]]);
            let address = DccAddress::from_magnitude(raw_address);
            Ok(Z21Command::RailcomGetData {
                request_type: frame[4].into(),
                address,
            })
        }
        HEADER_LOCONET_DETECTOR => {
            if frame.len() < 7 {
                return Err(ParseError::FrameTooShort);
            }
            Ok(Z21Command::LoconetDetector)
        }
        HEADER_SET_BROADCAST_FLAGS => {
            if frame.len() < 8 {
                return Err(ParseError::FrameTooShort);
            }
            Ok(Z21Command::SetBroadcastFlags)
        }
        HEADER_XBUS => parse_xbus(frame),
        _ => Ok(Z21Command::Unknown),
    }
}

#[must_use]
pub fn frame_kind(buf: &[u8]) -> FrameKind {
    let header = if buf.len() >= 4 {
        u16::from_le_bytes([buf[2], buf[3]])
    } else {
        0
    };
    let xheader = if header == HEADER_XBUS && buf.len() >= 5 {
        buf[4]
    } else {
        0
    };
    FrameKind { header, xheader }
}

/// Why a concatenated Z21 datagram cannot be split at its next frame boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum FrameBoundaryError {
    TruncatedHeader { remaining: usize },
    InvalidLength { declared: usize, remaining: usize },
}

/// Return the byte length of the first frame in `buf`.
fn frame_len(buf: &[u8]) -> Result<usize, FrameBoundaryError> {
    if buf.len() < 4 {
        return Err(FrameBoundaryError::TruncatedHeader {
            remaining: buf.len(),
        });
    }
    let len = u16::from_le_bytes([buf[0], buf[1]]) as usize;
    if len < 4 || len > buf.len() {
        Err(FrameBoundaryError::InvalidLength {
            declared: len,
            remaining: buf.len(),
        })
    } else {
        Ok(len)
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
    type Item = Result<&'a [u8], FrameBoundaryError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.buf.is_empty() {
            return None;
        }
        let len = match frame_len(self.buf) {
            Ok(len) => len,
            Err(error) => {
                self.buf = &[];
                return Some(Err(error));
            }
        };
        let frame = &self.buf[..len];
        self.buf = &self.buf[len..];
        Some(Ok(frame))
    }
}

/// Build an iterator over all Z21 frames in a UDP datagram buffer.
pub fn iter_frames(buf: &[u8]) -> FrameIter<'_> {
    FrameIter { buf }
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
        _ => return Err(ParseError::InvalidFunctionAction),
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

fn parse_cv_pom_command(payload: &[u8]) -> Result<Z21Command, ParseError> {
    // payload: [0xE6, 0x30, AddrH, AddrL, CVH/op, CVL, Value, checksum]
    if payload.len() < 8 {
        return Err(ParseError::FrameTooShort);
    }
    if payload[1] != DB0_POM_LOCO {
        return Ok(Z21Command::Unknown);
    }

    let address = parse_loco_address(payload[2], payload[3]).ok_or(ParseError::InvalidAddress)?;
    let op = payload[4] & 0xFC;
    let wire_cv = (u16::from(payload[4] & 0x03) << 8) | u16::from(payload[5]);
    let cv = cv_from_wire(wire_cv).ok_or(ParseError::InvalidCvAddress)?;

    match op {
        POM_OP_WRITE => Ok(Z21Command::CvPomWriteByte {
            address,
            cv,
            value: payload[6],
        }),
        POM_OP_READ => Ok(Z21Command::CvPomReadByte { address, cv }),
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
            let raw_address = u16::from(payload[1]) << 8 | u16::from(payload[2]);
            let address =
                DccAddress::from_magnitude(raw_address).ok_or(ParseError::InvalidAddress)?;
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
        XHEADER_CV_POM => parse_cv_pom_command(payload),
        XHEADER_GET_FIRMWARE_VERSION => {
            if payload.len() < 3 {
                return Err(ParseError::FrameTooShort);
            }
            if payload[1] == DB0_GET_FIRMWARE_VERSION {
                Ok(Z21Command::GetFirmwareVersion)
            } else {
                Ok(Z21Command::Unknown)
            }
        }
        _ => Ok(Z21Command::Unknown),
    }
}
