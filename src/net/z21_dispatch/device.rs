//! Z21 controller for static device and accessory information.

use z21 as z21_proto;

use crate::net::reply::encoded_len;

/// Arbitrary device serial reported to Z21 apps (no hardware identity exists).
const DEFAULT_Z21_SERIAL_NUMBER: u32 = 0xC0FF_EE01;

pub(super) fn encode_serial_number(out: &mut [u8]) -> usize {
    encoded_len(z21_proto::encode_serial_number(
        DEFAULT_Z21_SERIAL_NUMBER,
        out,
    ))
}

pub(super) fn encode_code(out: &mut [u8]) -> usize {
    encoded_len(z21_proto::encode_code(out))
}

pub(super) fn encode_hardware_info(out: &mut [u8]) -> usize {
    encoded_len(z21_proto::encode_hwinfo(out))
}

pub(super) fn encode_xbus_version(out: &mut [u8]) -> usize {
    encoded_len(z21_proto::encode_xbus_version(out))
}

pub(super) fn encode_firmware_version(out: &mut [u8]) -> usize {
    encoded_len(z21_proto::encode_firmware_version(out))
}

pub(super) fn encode_unknown_turnout(address: u16, out: &mut [u8]) -> usize {
    encoded_len(z21_proto::encode_turnout_info(address, out))
}
