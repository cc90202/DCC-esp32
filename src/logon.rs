//! Shared RCN-218 automatic-logon primitives.

/// Computes the reflected Dallas/Maxim CRC-8 used by RCN-218 logon packets.
///
/// Parameters: polynomial `0x31` reflected as `0x8c`, initial value `0x00`,
/// no final XOR.
pub(crate) fn crc8_dallas_maxim(bytes: &[u8]) -> u8 {
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
    fn matches_zimo_logon_reference_vector() {
        let data = [
            0x0b, 0x0a, 0x00, 0x00, 0x8e, 0x40, 0x00, 0x0d, 0x67, 0x00, 0x01, 0x00,
        ];
        assert_eq!(crc8_dallas_maxim(&data), 0x4c);
    }
}
