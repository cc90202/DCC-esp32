use crate::dcc::{DccAddress, Direction, SpeedFormat};

use super::LocoInfo;
use super::wire::*;

/// Encode `LAN_X_LOCO_INFO` into `out`. Returns bytes written, or 0 if `out` is shorter than 14.
///
/// Wire format per spec §4.4:
/// ```text
/// [DataLen:2][Header:0x0040:2][XH:0xEF]
/// [DB0:AdrH][DB1:AdrL][DB2:0000BKKK][DB3:RVVVVVVV]
/// [DB4:0DSLFGHJ][DB5:F5-F12][DB6:F13-F20][DB7:F21-F28][XCS]
/// ```
/// KKK: 0=14-step, 2=28-step, 4=128-step.  DB4: L=F0, F=F4, G=F3, H=F2, J=F1.
/// Total: 14 bytes.
pub fn encode_loco_info(state: &LocoInfo, out: &mut [u8]) -> usize {
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

    // DB2: 0000BKKK - B=0 (not busy), KKK=speed steps format
    let kkk: u8 = match state.speed.format() {
        SpeedFormat::Speed28 => 2,
        SpeedFormat::Speed128 => 4,
    };
    out[7] = kkk;

    // DB3: RVVVVVVV - direction + speed, encoded in the native format
    let wire_speed: u8 = match state.speed.format() {
        SpeedFormat::Speed128 => {
            if state.speed.is_zero() {
                0
            } else {
                state.speed.value().saturating_add(1) & 0x7F
            }
        }
        SpeedFormat::Speed28 => logical_to_z21_wire(state.speed.value()).unwrap_or(0),
    };
    let dir_bit: u8 = if state.direction == Direction::Forward {
        0x80
    } else {
        0x00
    };
    out[8] = dir_bit | (wire_speed & 0x7F);

    // DB4: 0DSLFGHJ - D=double traction(0), S=smartsearch(0), L=F0, F=F4, G=F3, H=F2, J=F1
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

/// Encode `LAN_X_CV_RESULT` into `out`. Returns bytes written (10), or 0 if `out` is too short.
pub fn encode_cv_result(cv: u16, value: u8, out: &mut [u8]) -> usize {
    const LEN: usize = 10;
    if out.len() < LEN || !(1..=1024).contains(&cv) {
        return 0;
    }

    let wire_cv = cv - 1;
    write_frame_header(out, LEN as u16, RESP_BC_XBUS);
    out[4] = RESP_XH_CV_RESULT;
    out[5] = RESP_DB0_CV_RESULT;
    out[6] = ((wire_cv >> 8) & 0x03) as u8;
    out[7] = (wire_cv & 0xFF) as u8;
    out[8] = value;
    write_xbus_checksum(out, 4, 5);
    LEN
}

/// Encode `LAN_X_CV_NACK` into `out`. Returns bytes written (7), or 0 if `out` is too short.
pub fn encode_cv_nack(out: &mut [u8]) -> usize {
    const LEN: usize = 7;
    if out.len() < LEN {
        return 0;
    }

    write_frame_header(out, LEN as u16, RESP_BC_XBUS);
    out[4] = RESP_XH_TRACK_POWER;
    out[5] = RESP_DB0_CV_NACK;
    write_xbus_checksum(out, 4, 2);
    LEN
}

/// Encode `LAN_X_TURNOUT_INFO` into `out`. Returns bytes written (9), or 0 if `out` is shorter than 9.
///
/// XBus reply, header=0x0040, X-Header=0x43 (same byte as request).
/// DB2=0x00 means state unknown - we have no accessory decoder support.
/// Spec section: Z21 LAN Protocol v1.13, accessory decoder chapter.
pub fn encode_turnout_info(address: DccAddress, out: &mut [u8]) -> usize {
    // [DataLen:2][Header=0x0040:2][0x43][AddrH][AddrL][DB2=0x00][XCS]
    const LEN: usize = 9;
    if out.len() < LEN {
        return 0;
    }
    let address = address.value();
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

/// Encode `LAN_X_GET_FIRMWARE_VERSION` reply into `out`. Returns bytes written (9), or 0.
///
/// Spec §2.15: XBus reply, header=0x0040, X-Header=0xF3, DB0=0x0A,
/// DB1/DB2 contain the BCD firmware version. `FW_VERSION=0x00000140` is reported as 1.40.
pub fn encode_firmware_version(out: &mut [u8]) -> usize {
    const LEN: usize = 9;
    if out.len() < LEN {
        return 0;
    }
    write_frame_header(out, LEN as u16, RESP_BC_XBUS);
    out[4] = RESP_XH_FIRMWARE_VERSION;
    out[5] = DB0_GET_FIRMWARE_VERSION;
    out[6] = ((FW_VERSION >> 8) & 0xFF) as u8;
    out[7] = (FW_VERSION & 0xFF) as u8;
    write_xbus_checksum(out, 4, 4);
    LEN
}

/// Encode `LAN_GET_CODE` response into `out`. Returns bytes written (5), or 0 if `out` is shorter than 5.
///
/// Code=0x00 means no feature restrictions - all Z21 functionality available.
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
///   [4-5]  MainCurrent         i16 mA  (we report 0 - no live ADC path here)
///   [6-7]  ProgCurrent         i16 mA
///   [8-9]  FilteredMainCurrent i16 mA
///   [10-11] Temperature        i16 °C  (nominal 25)
///   [12-13] SupplyVoltage      u16 mV  (nominal 18 V)
///   [14-15] VCCVoltage         u16 mV  (nominal 3.3 V)
///   \[16\]   CentralState: bit0=EmergencyStop, bit1=TrackVoltageOff, bit2=ShortCircuit
///   \[17\]   CentralStateEx (0)
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
    // MainCurrent, ProgCurrent, FilteredMainCurrent - all 0 (no ADC)
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

/// Encode `LAN_GET_LOCOMODE` reply. Mode 0 means DCC output format.
pub fn encode_loco_mode(address: DccAddress, out: &mut [u8]) -> usize {
    const LEN: usize = 7;
    if out.len() < LEN {
        return 0;
    }

    write_frame_header(out, LEN as u16, RESP_LOCOMODE);
    write_loco_address_be(address, out, 4);
    out[6] = LOCO_MODE_DCC;
    LEN
}

/// Encode `LAN_RAILCOM_DATACHANGED`.
///
/// This is the Z21 LAN-level RailCom cache response, not the NMRA RailCom UART payload.
/// The address and counters are enough for apps that poll whether a loco has RailCom data.
pub fn encode_railcom_data(
    address: DccAddress,
    receive_counter: u32,
    error_counter: u16,
    out: &mut [u8],
) -> usize {
    const LEN: usize = 17;
    if out.len() < LEN {
        return 0;
    }

    write_frame_header(out, LEN as u16, RESP_RAILCOM_DATACHANGED);
    write_u16_le(out, 4, address.value());
    write_u32_le(out, 6, receive_counter);
    write_u16_le(out, 10, error_counter);
    out[12] = 0; // reserved
    out[13] = 0; // no speed/QoS options available yet
    out[14] = 0; // speed
    out[15] = 0; // QoS
    out[16] = 0; // reserved
    LEN
}
