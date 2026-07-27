use crate::dcc::DccAddress;

// Source: Z21 LAN Protocol Specification v1.13 (docs/specs/z21-lan-protokoll-en.pdf)

pub(super) const HEADER_GET_SERIAL_NUMBER: u16 = 0x0010;
// 0x0012 is not in the v1.13 spec but some app versions send it as GetCode.
pub(super) const HEADER_GET_CODE_LEGACY: u16 = 0x0012; // app-compat alias
pub(super) const HEADER_GET_CODE: u16 = 0x0018; // per spec §2.21
pub(super) const HEADER_GET_HWINFO: u16 = 0x001A;
pub(super) const HEADER_LOGOFF: u16 = 0x0030;
pub(super) const HEADER_SET_BROADCAST_FLAGS: u16 = 0x0050;
pub(super) const HEADER_GET_LOCOMODE: u16 = 0x0060;
pub(super) const HEADER_SET_LOCOMODE: u16 = 0x0061;
pub(super) const HEADER_RAILCOM_GETDATA: u16 = 0x0089;
pub(super) const HEADER_LOCONET_DETECTOR: u16 = 0x00A4;
pub(crate) const HEADER_XBUS: u16 = 0x0040;
pub(crate) const HEADER_SYSTEMSTATE_GETDATA: u16 = 0x0085;

// XBus X-Header values (used inside header=0x0040 frames)
pub(super) const XHEADER_GET_VERSION: u8 = 0x21; // sub-commands dispatched by DB0 below
pub(super) const XHEADER_TURNOUT_INFO: u8 = 0x43; // shared by LAN_X_GET_TURNOUT_INFO (request) and LAN_X_TURNOUT_INFO (response)
pub(super) const XHEADER_SET_STOP: u8 = 0x80;
pub(super) const XHEADER_SET_LOCO_ESTOP: u8 = 0x92;
pub(super) const XHEADER_GET_LOCO_INFO: u8 = 0xE3;
pub(super) const XHEADER_SET_LOCO_DRIVE: u8 = 0xE4;
pub(super) const XHEADER_CV_POM: u8 = 0xE6;
pub(super) const XHEADER_GET_FIRMWARE_VERSION: u8 = 0xF1;

// DB0 sub-commands under XHEADER_GET_VERSION (0x21)
pub(super) const DB0_GET_XBUS_VERSION: u8 = 0x21;
pub(super) const DB0_GET_STATUS: u8 = 0x24;
pub(super) const DB0_SET_TRACK_POWER_OFF: u8 = 0x80;
pub(super) const DB0_SET_TRACK_POWER_ON: u8 = 0x81;

// DB0 sub-commands under XHEADER_SET_LOCO_DRIVE (0xE4)
pub(super) const DB0_LOCO_SPEED28: u8 = 0x12;
pub(super) const DB0_LOCO_SPEED128: u8 = 0x13;
pub(super) const DB0_LOCO_FUNCTION: u8 = 0xF8;

// DB0 discriminator under XHEADER_GET_LOCO_INFO (0xE3)
pub(super) const DB0_GET_LOCO_INFO: u8 = 0xF0;
pub(super) const DB0_GET_FIRMWARE_VERSION: u8 = 0x0A;

// Z21 response headers (always 0x0040 for XBus replies; others echo the request header)
pub(super) const RESP_GET_SERIAL_NUMBER: u16 = 0x0010;
pub(super) const RESP_GET_CODE: u16 = 0x0018; // per spec §2.21
pub(super) const RESP_SYSTEMSTATE: u16 = 0x0084;
pub(super) const RESP_LOCO_INFO: u16 = 0x0040;
pub(super) const RESP_LOCOMODE: u16 = 0x0060;
pub(super) const RESP_RAILCOM_DATACHANGED: u16 = 0x0088;
pub(super) const RESP_BC_XBUS: u16 = 0x0040; // shared header for all XBus broadcast/response frames

// Response XBus X-Header values
pub(super) const RESP_XH_LOCO_INFO: u8 = 0xEF;
pub(super) const RESP_XH_TRACK_POWER: u8 = 0x61; // BC_TRACK_POWER_ON/OFF and UNKNOWN_COMMAND
pub(super) const RESP_XH_BC_STOPPED: u8 = 0x81;
pub(super) const RESP_XH_STATUS_CHANGED: u8 = 0x62;
pub(super) const RESP_XH_VERSION: u8 = 0x63;
pub(super) const RESP_XH_CV_RESULT: u8 = 0x64;
pub(super) const RESP_XH_FIRMWARE_VERSION: u8 = 0xF3;

// Response DB0 values
pub(super) const RESP_DB0_POWER_ON: u8 = 0x01;
pub(super) const RESP_DB0_POWER_OFF: u8 = 0x00;
pub(super) const RESP_DB0_STATUS_CHANGED: u8 = 0x22;
pub(super) const RESP_DB0_UNKNOWN_COMMAND: u8 = 0x82;
pub(super) const RESP_DB0_CV_RESULT: u8 = 0x14;
pub(super) const RESP_DB0_CV_NACK: u8 = 0x13;

pub(super) const DB0_POM_LOCO: u8 = 0x30;
pub(super) const POM_OP_READ: u8 = 0xE4;
pub(super) const POM_OP_WRITE: u8 = 0xEC;

// CentralState bitmask (spec §2.2, used in encode_status and encode_system_state)
pub(super) const CS_EMERGENCY_STOP: u8 = 0x01;
pub(super) const CS_TRACK_VOLTAGE_OFF: u8 = 0x02;
pub(super) const CS_SHORT_CIRCUIT: u8 = 0x04;

// Device identity (reported to Z21 apps)
pub(super) const XBUS_VERSION: u8 = 0x30; // V3.0
pub(super) const CMDST_ID: u8 = 0x12; // Z21 device family
pub(super) const HW_TYPE: u32 = 0x00000200; // Z21 black (full-featured)
pub(super) const FW_VERSION: u32 = 0x00000140; // firmware 1.40
pub(super) const LOCO_MODE_DCC: u8 = 0x00;

/// XOR checksum over bytes from x_header onwards (excluding the checksum byte).
pub(super) fn xbus_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc ^ b)
}

/// Decode a Z21 wire address (2-byte format: high byte, low byte).
///
/// - Short address: high byte = 0x00, low byte = 1-127
/// - Long address:  high byte has bit7..bit6 = 0b11, range 128-10239
pub(super) fn parse_loco_address(high: u8, low: u8) -> Option<DccAddress> {
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

/// Write a little-endian u16 into buf[pos..pos+2].
#[inline]
pub(super) fn write_u16_le(buf: &mut [u8], pos: usize, val: u16) {
    buf[pos] = (val & 0xFF) as u8;
    buf[pos + 1] = (val >> 8) as u8;
}

/// Write a little-endian u32 into buf[pos..pos+4].
#[inline]
pub(super) fn write_u32_le(buf: &mut [u8], pos: usize, val: u32) {
    buf[pos] = (val & 0xFF) as u8;
    buf[pos + 1] = ((val >> 8) & 0xFF) as u8;
    buf[pos + 2] = ((val >> 16) & 0xFF) as u8;
    buf[pos + 3] = ((val >> 24) & 0xFF) as u8;
}

/// Write a Z21 frame header: DataLen (LE u16) + Header (LE u16).
#[inline]
pub(super) fn write_frame_header(buf: &mut [u8], data_len: u16, header: u16) {
    write_u16_le(buf, 0, data_len);
    write_u16_le(buf, 2, header);
}

/// Compute and write XBus checksum at the end of the XBus payload slice.
///
/// `xbus_payload` is the slice starting from X-Header byte up to but not
/// including the checksum byte. The checksum is written at `xbus_payload.len()`.
pub(super) fn write_xbus_checksum(buf: &mut [u8], xbus_start: usize, xbus_payload_len: usize) {
    let cs = xbus_checksum(&buf[xbus_start..xbus_start + xbus_payload_len]);
    buf[xbus_start + xbus_payload_len] = cs;
}

pub(super) fn write_loco_address_be(address: DccAddress, out: &mut [u8], pos: usize) {
    let raw = address.value();
    if address.is_long() {
        out[pos] = 0xC0 | ((raw >> 8) as u8);
        out[pos + 1] = (raw & 0xFF) as u8;
    } else {
        out[pos] = 0x00;
        out[pos + 1] = raw as u8;
    }
}

/// Convert a public 1-based CV number into its 10-bit wire representation.
#[must_use]
pub(super) const fn cv_to_wire(cv: u16) -> Option<u16> {
    match cv {
        1..=1024 => Some(cv - 1),
        _ => None,
    }
}

/// Convert a 10-bit wire CV value into its public 1-based representation.
#[must_use]
pub(super) const fn cv_from_wire(wire_cv: u16) -> Option<u16> {
    match wire_cv {
        0..=1023 => Some(wire_cv + 1),
        _ => None,
    }
}

/// Convert a logical runtime speed (0..=28) into the Z21 wire representation.
#[must_use]
pub(super) const fn logical_to_z21_wire(speed: u8) -> Option<u8> {
    let n @ 0..=28 = speed else {
        return None;
    };

    if n == 0 {
        Some(0)
    } else {
        let v = ((n + 1) >> 1) + 1;
        Some(if n.is_multiple_of(2) { v | 0x10 } else { v })
    }
}

/// Convert the Z21/NMRA intermediate wire representation into logical runtime speed.
///
/// Invalid wire values are mapped to stop, matching the parser's fail-safe policy.
#[must_use]
pub(super) const fn z21_wire_to_logical(raw_speed: u8) -> u8 {
    if raw_speed == 0 || raw_speed == 0x10 {
        return 0;
    }

    let v5 = (raw_speed >> 4) & 1;
    let v_low4 = raw_speed & 0x0F;
    if v_low4 <= 1 {
        0
    } else {
        let logical = (v_low4 - 1) * 2 + v5 - 1;
        if logical > 28 { 28 } else { logical }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_logical_to_z21_wire() {
        assert_eq!(logical_to_z21_wire(0), Some(0));
        assert_eq!(logical_to_z21_wire(1), Some(0x02));
        assert_eq!(logical_to_z21_wire(14), Some(0x18));
        assert_eq!(logical_to_z21_wire(28), Some(0x1F));
        assert_eq!(logical_to_z21_wire(29), None);
    }

    #[test]
    fn test_z21_wire_to_logical_roundtrip_examples() {
        assert_eq!(z21_wire_to_logical(0x00), 0);
        assert_eq!(z21_wire_to_logical(0x10), 0);
        assert_eq!(z21_wire_to_logical(0x02), 1);
        assert_eq!(z21_wire_to_logical(0x18), 14);
        assert_eq!(z21_wire_to_logical(0x1F), 28);
    }

    #[test]
    fn test_cv_wire_conversion_is_checked_and_reversible() {
        assert_eq!(cv_to_wire(0), None);
        assert_eq!(cv_to_wire(1), Some(0));
        assert_eq!(cv_to_wire(1024), Some(1023));
        assert_eq!(cv_to_wire(1025), None);
        assert_eq!(cv_from_wire(0), Some(1));
        assert_eq!(cv_from_wire(1023), Some(1024));
        assert_eq!(cv_from_wire(1024), None);
    }
}
