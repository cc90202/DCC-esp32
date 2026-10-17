//! Shared 28-step speed semantics used across DCC packet and Z21 protocol code.

use crate::speed::SpeedFormat;

/// Highest logical 28-step speed.
const MAX_LOGICAL: u8 = SpeedFormat::Speed28.max_step();

/// Highest packet speed value: the logical steps sit one above their number,
/// behind the emergency-stop value.
const MAX_WIRE: u8 = MAX_LOGICAL + 1;

/// Validated 28-step speed value per NMRA S-9.2 §2.3.2.3.
///
/// 0 is stop, 1 is emergency stop, and each logical step
/// `1..=`[`SpeedFormat::max_step`] of [`SpeedFormat::Speed28`] travels as the
/// value one above it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NmraSpeed28(u8);

impl NmraSpeed28 {
    /// Validate a packet speed value; `None` above the top step.
    #[must_use]
    pub const fn new(speed: u8) -> Option<Self> {
        match speed {
            value @ 0..=MAX_WIRE => Some(Self(value)),
            _ => None,
        }
    }

    /// The packet speed value: 0 stop, 1 emergency stop, then the steps.
    #[must_use]
    pub const fn value(self) -> u8 {
        self.0
    }
}

/// Convert a logical runtime speed into NMRA packet speed semantics; `None`
/// above the top logical step.
///
/// Runtime state uses:
/// - `0` = stop
/// - `1..=`[`SpeedFormat::max_step`] = speed steps
///
/// NMRA packet encoding uses:
/// - `0` = stop
/// - `1` = emergency stop
/// - from `2` = the speed steps, each one above its logical number
#[must_use]
pub const fn logical_to_nmra_packet_speed(speed: u8) -> Option<u8> {
    match speed {
        0 => Some(0),
        1..=MAX_LOGICAL => Some(speed + 1),
        _ => None,
    }
}

/// Encode the 28-step NMRA instruction byte payload bits (`C` + `SSSS`).
///
/// Stop and emergency stop map to `0b0_0000` and `0b0_0001`; the speed steps
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
        assert_eq!(logical_to_nmra_packet_speed(MAX_LOGICAL), Some(MAX_WIRE));
        assert_eq!(logical_to_nmra_packet_speed(MAX_LOGICAL + 1), None);
    }

    #[test]
    fn test_encode_nmra_instruction_speed_bits() {
        let bits = |raw| encode_nmra_instruction_speed_bits(NmraSpeed28::new(raw).unwrap());
        assert_eq!(bits(0), 0x00);
        assert_eq!(bits(1), 0x01);
        assert_eq!(bits(2), 0x02);
        assert_eq!(bits(MAX_WIRE), 0x1F);
    }

    #[test]
    fn nmra_speed28_rejects_values_above_the_top_wire_step() {
        assert_eq!(
            NmraSpeed28::new(MAX_WIRE).map(NmraSpeed28::value),
            Some(MAX_WIRE)
        );
        assert_eq!(NmraSpeed28::new(MAX_WIRE + 1), None);
    }
}
