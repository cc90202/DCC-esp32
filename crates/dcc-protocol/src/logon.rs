//! Shared RCN-218 automatic-logon primitives.

use crate::packet::DccAddress;

/// Annex D prefix (A13..A8) of a short locomotive address.
const PREFIX_SHORT: u8 = 0x38;
/// Highest annex D prefix of a long locomotive address.
const PREFIX_LONG_MAX: u8 = 0x27;
/// Bits of the first byte that hold A13..A8.
const PREFIX_MASK: u8 = 0x3f;

/// Reads a locomotive address in the 14-bit form of RCN-218 annex D.
///
/// `bytes` holds A13..A8 in the low six bits of the first byte and A7..A0 in
/// the second. The two top bits of the first byte are not part of the
/// address: the frame that carries it gives them a meaning of its own, and the
/// caller checks them. Prefix 0x38 is a short address in A6..A0, prefixes
/// 0x00..=0x27 a long address equal to A13..A0.
///
/// Returns `None` for accessory, reserved and firmware-update prefixes, for
/// short address 0 (which parks a decoder), and for a long address below 128.
/// Annex D allows the latter, but [`DccAddress`] cannot hold it, and the Z21
/// side reads 1..=127 as a short address, so no app could drive it.
#[must_use]
pub fn decode_address(bytes: [u8; 2]) -> Option<DccAddress> {
    let [high, low] = bytes;
    match high & PREFIX_MASK {
        PREFIX_SHORT => DccAddress::new_short(low),
        prefix @ 0..=PREFIX_LONG_MAX => DccAddress::new_long(u16::from_be_bytes([prefix, low])),
        _ => None,
    }
}

/// Computes the reflected Dallas/Maxim CRC-8 used by RCN-218 logon packets.
///
/// Parameters: polynomial `0x31` reflected as `0x8c`, initial value `0x00`,
/// no final XOR.
pub fn crc8_dallas_maxim(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in bytes {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x01 != 0 {
                (crc >> 1) ^ 0x8c
            } else {
                crc >> 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_short_address_with_its_annex_d_prefix() {
        assert_eq!(decode_address([0x38, 4]), DccAddress::new_short(4));
        assert_eq!(decode_address([0x38, 127]), DccAddress::new_short(127));
    }

    #[test]
    fn decodes_long_address_as_its_plain_value() {
        assert_eq!(decode_address([0x03, 0xE9]), DccAddress::new_long(1001));
        assert_eq!(decode_address([0x00, 0x80]), DccAddress::new_long(128));
        assert_eq!(decode_address([0x27, 0xFF]), DccAddress::new_long(10239));
    }

    #[test]
    fn leaves_the_two_top_bits_to_the_frame() {
        // The short info sets bit 7 as its frame marker.
        assert_eq!(decode_address([0xB8, 4]), DccAddress::new_short(4));
        assert_eq!(decode_address([0x83, 0xE9]), DccAddress::new_long(1001));
    }

    #[test]
    fn rejects_addresses_a_locomotive_cannot_drive_with() {
        // Short address 0 parks the decoder; only A6..A0 carry a short one.
        assert_eq!(decode_address([0x38, 0]), None);
        assert_eq!(decode_address([0x38, 0x80]), None);
        // A long address below 128: reading it as short would name another
        // decoder.
        assert_eq!(decode_address([0x00, 100]), None);
        // Extended and basic accessory decoders.
        assert_eq!(decode_address([0x28, 1]), None);
        assert_eq!(decode_address([0x30, 1]), None);
        // Reserved, and the firmware-update marker.
        assert_eq!(decode_address([0x39, 4]), None);
        assert_eq!(decode_address([0x3F, 0xFF]), None);
    }

    #[test]
    fn matches_zimo_logon_reference_vector() {
        let data = [
            0x0b, 0x0a, 0x00, 0x00, 0x8e, 0x40, 0x00, 0x0d, 0x67, 0x00, 0x01, 0x00,
        ];
        assert_eq!(crc8_dallas_maxim(&data), 0x4c);
    }
}
