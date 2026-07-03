//! Shared 28-step speed semantics used across DCC packet and Z21 protocol code.

use crate::dcc::packet::PacketEncodeError;

/// Convert logical runtime speed (0..=28) into NMRA packet speed semantics.
///
/// Runtime state uses:
/// - `0` = stop
/// - `1..=28` = speed steps
///
/// NMRA packet encoding uses:
/// - `0` = stop
/// - `1` = emergency stop
/// - `2..=29` = speed steps 1..=28
#[must_use]
pub const fn logical_to_nmra_packet_speed(speed: u8) -> Option<u8> {
    match speed {
        0 => Some(0),
        1..=28 => Some(speed + 1),
        _ => None,
    }
}

/// Encode the 28-step NMRA instruction byte payload bits (`C` + `SSSS`).
pub const fn encode_nmra_instruction_speed_bits(speed: u8) -> Result<u8, PacketEncodeError> {
    match speed {
        0 => Ok(0b0_0000),
        1 => Ok(0b0_0001),
        2..=29 => {
            let step = speed - 1;
            let ssss = (step + 3) / 2;
            let c = (step - 1) & 1;
            Ok((ssss & 0x0F) | (c << 4))
        }
        _ => Err(PacketEncodeError::InvalidSpeed28 { speed }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_logical_to_nmra_packet_speed() {
        assert_eq!(logical_to_nmra_packet_speed(0), Some(0));
        assert_eq!(logical_to_nmra_packet_speed(1), Some(2));
        assert_eq!(logical_to_nmra_packet_speed(28), Some(29));
        assert_eq!(logical_to_nmra_packet_speed(29), None);
    }

    #[test]
    fn test_encode_nmra_instruction_speed_bits() {
        assert_eq!(encode_nmra_instruction_speed_bits(0), Ok(0x00));
        assert_eq!(encode_nmra_instruction_speed_bits(1), Ok(0x01));
        assert_eq!(encode_nmra_instruction_speed_bits(2), Ok(0x02));
        assert_eq!(encode_nmra_instruction_speed_bits(29), Ok(0x1F));
        assert_eq!(
            encode_nmra_instruction_speed_bits(30),
            Err(PacketEncodeError::InvalidSpeed28 { speed: 30 })
        );
    }
}
