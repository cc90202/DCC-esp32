//! Shared 28-step speed semantics used across DCC packet and Z21 protocol code.

/// Validated 28-step speed value per NMRA S-9.2 §2.3.2.3.
///
/// Range 0-29: 0=stop, 1=e-stop, 2-29=speed steps 1-28.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct NmraSpeed28(u8);

impl NmraSpeed28 {
    pub const fn new(speed: u8) -> Option<Self> {
        match speed {
            value @ 0..=29 => Some(Self(value)),
            _ => None,
        }
    }

    pub const fn value(self) -> u8 {
        self.0
    }
}

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
///
/// Stop and emergency stop map to `0b0_0000` and `0b0_0001`; steps 1..=28
/// interleave the intermediate bit `C` (bit 4) with `SSSS` (bits 0..=3).
#[must_use]
pub const fn encode_nmra_instruction_speed_bits(speed: NmraSpeed28) -> u8 {
    match speed.value() {
        0 => 0b0_0000,
        1 => 0b0_0001,
        raw => {
            let step = raw - 1;
            let ssss = u8::midpoint(step, 3);
            let c = (step - 1) & 1;
            (ssss & 0x0F) | (c << 4)
        }
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
        let bits = |raw| encode_nmra_instruction_speed_bits(NmraSpeed28::new(raw).unwrap());
        assert_eq!(bits(0), 0x00);
        assert_eq!(bits(1), 0x01);
        assert_eq!(bits(2), 0x02);
        assert_eq!(bits(29), 0x1F);
    }
}
