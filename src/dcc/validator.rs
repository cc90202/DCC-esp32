//! NMRA DCC compliance validator
//!
//! Validates DCC packets and pulse sequences against NMRA standards S-9.1 and S-9.2.
//!
//! # Overview
//!
//! The validator provides several validation functions at different levels:
//! - [`DccValidator::validate_timing`] — Check pulse timing against NMRA acceptable ranges
//! - [`DccValidator::validate_packet_structure`] — Verify preamble, start/end bits, byte boundaries
//! - [`DccValidator::validate_nmra_compliance`] — Check address and speed ranges
//! - [`DccValidator::validate_checksum`] — Verify XOR checksum
//! - [`DccValidator::validate_full`] — Run all validations given packet + pulses
//! - [`DccValidator::validate_complete`] — Encode packet and run all validations (one call)
//!
//! All validations are based on NMRA S-9.1 (electrical) and S-9.2 (communications) standards.
//!
//! # Examples
//!
//! **Validate pulse timing (NMRA S-9.1):**
//!
//! ```
//! use dcc_esp32::dcc::encoder::{PulseCode, encode_dcc_packet};
//! use dcc_esp32::dcc::{DccValidator, DccPacket, DccAddress, Direction};
//!
//! // Create and encode a valid packet
//! let addr = DccAddress::short(5).unwrap();
//! let packet = DccPacket::speed_128step(addr, 50, Direction::Forward).unwrap();
//!
//! let pulses = encode_dcc_packet(&packet).unwrap();
//!
//! // Check that all pulses are within NMRA timing ranges
//! // "1" bit: 55-61μs, "0" bit: 95-9900μs
//! assert!(DccValidator::validate_timing(&pulses).is_ok());
//! ```
//!
//! **Validate packet structure:**
//!
//! ```
//! use dcc_esp32::dcc::encoder::encode_dcc_packet;
//! use dcc_esp32::dcc::{DccValidator, DccPacket, DccAddress, Direction};
//!
//! // Encoded pulses must have: preamble (14+ bits) + start (0) + bytes + separators/end (1)
//! let addr = DccAddress::short(10).unwrap();
//! let packet = DccPacket::speed_128step(addr, 100, Direction::Reverse).unwrap();
//!
//! let pulses = encode_dcc_packet(&packet).unwrap();
//! assert!(DccValidator::validate_packet_structure(&pulses).is_ok());
//! ```
//!
//! **Validate complete packet (structure + timing + compliance + checksum):**
//!
//! ```
//! use dcc_esp32::dcc::encoder::encode_dcc_packet;
//! use dcc_esp32::dcc::{DccValidator, DccPacket, DccAddress, Direction};
//!
//! let addr = DccAddress::short(42).unwrap();
//! let packet = DccPacket::speed_128step(addr, 75, Direction::Forward).unwrap();
//!
//! // Must encode first, then validate
//! let pulses = encode_dcc_packet(&packet).unwrap();
//!
//! // Comprehensive validation: timing + structure + address/speed ranges + checksum
//! assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
//! ```
//!
//! **Validate NMRA compliance (ranges only):**
//!
//! ```
//! use dcc_esp32::dcc::{DccValidator, DccPacket, DccAddress, Direction};
//!
//! let addr = DccAddress::short(5).unwrap();
//! let packet = DccPacket::speed_28step(addr, 15, Direction::Forward).unwrap();
//!
//! // Check that address and speed values are in valid NMRA ranges
//! assert!(DccValidator::validate_nmra_compliance(&packet).is_ok());
//! ```
//!
//! **Validate everything in one call (recommended):**
//!
//! ```
//! use dcc_esp32::dcc::{DccValidator, DccPacket, DccAddress, Direction};
//!
//! let addr = DccAddress::short(42).unwrap();
//! let packet = DccPacket::speed_128step(addr, 75, Direction::Forward).unwrap();
//!
//! // Single call: encode packet + validate timing + structure + ranges + checksum
//! assert!(DccValidator::validate_complete(&packet).is_ok());
//! ```
//!
//! # NMRA Ranges
//!
//! - **DCC "1" bit**: 55-61μs high and low (nominal 58μs ±3μs)
//! - **DCC "0" bit**: 95-9900μs (minimum 95μs; stretched "0" for decoders with limited resolution)
//! - **Minimum preamble**: 14 bits (per NMRA S-9.2, typically 16 bits transmitted)
//! - **Packet structure**: Preamble + Start Bit (0) + Address Byte + Data Byte(s) + Error Detection Byte + End Bit (1)

use crate::dcc::timing::{DCC_ONE_HIGH_US, DCC_ZERO_HIGH_US};
use crate::dcc::{DccPacket, encoder::PulseCode};

const ONE_BIT_MIN_US: u16 = 55;
const ONE_BIT_MAX_US: u16 = 61;
const ZERO_BIT_MIN_US: u16 = 95;
const ZERO_BIT_MAX_US: u16 = 9_900;

/// Validation errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum ValidationError {
    /// Pulse timing outside NMRA acceptable range
    TimingOutOfRange { expected_us: u16, actual_us: u16 },
    /// Preamble has fewer than 14 bits
    InsufficientPreamble { found: usize },
    /// Packet checksum doesn't match
    InvalidChecksum { expected: u8, actual: u8 },
    /// Speed value out of valid range
    /// Address out of valid range
    InvalidAddress,
    /// Missing start bit
    MissingStartBit,
    /// Missing end bit
    MissingEndBit,
    /// Packet too short
    PacketTooShort { min: usize, actual: usize },
    /// Invalid byte/bit structure in pulse sequence
    InvalidStructure,
    /// Packet could not be encoded to bytes
    EncodingError,
}

/// DCC packet and timing validator
pub struct DccValidator;

impl DccValidator {
    /// Validates pulse timing against NMRA S-9.1
    ///
    /// NMRA acceptable ranges:
    /// - "1" bit: 55-61μs high, 55-61μs low
    /// - "0" bit: 95-9900μs (min 95, max for stretching)
    ///
    /// V1: Rejects pulses in the gap range (62-94μs) that are neither valid "1" nor "0".
    pub fn validate_timing(pulses: &[PulseCode]) -> Result<(), ValidationError> {
        for pulse in pulses {
            if (ONE_BIT_MIN_US..=ONE_BIT_MAX_US).contains(&pulse.length1) {
                // Valid "1" bit high half — check low half
                if !(ONE_BIT_MIN_US..=ONE_BIT_MAX_US).contains(&pulse.length2) {
                    return Err(ValidationError::TimingOutOfRange {
                        expected_us: DCC_ONE_HIGH_US,
                        actual_us: pulse.length2,
                    });
                }
            } else if (ZERO_BIT_MIN_US..=ZERO_BIT_MAX_US).contains(&pulse.length1) {
                // Valid "0" bit high half — check low half
                if !(ZERO_BIT_MIN_US..=ZERO_BIT_MAX_US).contains(&pulse.length2) {
                    return Err(ValidationError::TimingOutOfRange {
                        expected_us: DCC_ZERO_HIGH_US,
                        actual_us: pulse.length2,
                    });
                }
            } else {
                // V1: Reject anything not recognizable as "1" or "0"
                return Err(ValidationError::TimingOutOfRange {
                    expected_us: 0,
                    actual_us: pulse.length1,
                });
            }
        }
        Ok(())
    }

    /// Returns true if a pulse represents a "1" bit
    fn is_one_bit(pulse: &PulseCode) -> bool {
        (ONE_BIT_MIN_US..=ONE_BIT_MAX_US).contains(&pulse.length1)
            && (ONE_BIT_MIN_US..=ONE_BIT_MAX_US).contains(&pulse.length2)
    }

    /// Returns true if a pulse represents a "0" bit
    fn is_zero_bit(pulse: &PulseCode) -> bool {
        (ZERO_BIT_MIN_US..=ZERO_BIT_MAX_US).contains(&pulse.length1)
            && (ZERO_BIT_MIN_US..=ZERO_BIT_MAX_US).contains(&pulse.length2)
    }

    /// Validates packet structure (preamble, start/end bits, byte boundaries)
    ///
    /// V2: After preamble and first start bit, validates that every 8 pulses
    /// is followed by a separator "0" (inter-byte) or end bit "1" (last byte).
    pub fn validate_packet_structure(pulses: &[PulseCode]) -> Result<(), ValidationError> {
        if pulses.len() < 15 {
            return Err(ValidationError::PacketTooShort {
                min: 15,
                actual: pulses.len(),
            });
        }

        // Count preamble "1" bits
        let preamble_count = pulses.iter().take_while(|p| Self::is_one_bit(p)).count();

        if preamble_count < 14 {
            return Err(ValidationError::InsufficientPreamble {
                found: preamble_count,
            });
        }

        // Check start bit after preamble
        let mut pos = preamble_count;
        if pos >= pulses.len() || !Self::is_zero_bit(&pulses[pos]) {
            return Err(ValidationError::MissingStartBit);
        }
        pos += 1; // skip start bit

        // V2: Validate byte structure — each byte is 8 pulses followed by separator/end
        loop {
            // Need at least 8 pulses for a data byte + 1 for separator/end
            if pos + 9 > pulses.len() {
                return Err(ValidationError::InvalidStructure);
            }

            pos += 8; // skip 8 data bit pulses

            // After 8 bits: expect "0" (inter-byte separator) or "1" (end bit)
            if Self::is_one_bit(&pulses[pos]) {
                // End bit — this should be the last pulse
                if pos != pulses.len() - 1 {
                    return Err(ValidationError::InvalidStructure);
                }
                break;
            } else if Self::is_zero_bit(&pulses[pos]) {
                // Inter-byte separator — continue to next byte
                pos += 1;
            } else {
                return Err(ValidationError::InvalidStructure);
            }
        }

        Ok(())
    }

    /// Validates NMRA S-9.2 packet compliance
    ///
    /// V3: Validates address ranges (defense in depth).
    /// V5: Validates Speed28 interleaved encoding.
    pub fn validate_nmra_compliance(packet: &DccPacket) -> Result<(), ValidationError> {
        match packet {
            DccPacket::Speed28 { address, .. } => {
                // Speed range guaranteed by NmraSpeed28 newtype (0-29)
                Self::validate_address(address)?;
            }
            DccPacket::Speed128 { address, .. } => {
                // Speed range guaranteed by NmraSpeed128 newtype (0-126)
                Self::validate_address(address)?;
            }
            DccPacket::FunctionGroup1 { address, .. }
            | DccPacket::FunctionGroup2A { address, .. }
            | DccPacket::FunctionGroup2B { address, .. }
            | DccPacket::FunctionGroup3 { address, .. }
            | DccPacket::FunctionGroup4 { address, .. }
            | DccPacket::EmergencyStop { address, .. } => {
                Self::validate_address(address)?;
            }
            DccPacket::ServiceModeVerifyByte { cv, .. }
            | DccPacket::ServiceModeWriteByte { cv, .. }
                // Service Mode: CV 1-256 only in V1 (Direct Mode)
                if !(1..=256).contains(cv) => {
                    return Err(ValidationError::InvalidAddress);
                }
            _ => {} // Idle, Reset, BroadcastStop have no address to validate
        }
        Ok(())
    }

    /// V3/V6: Validates address range (defense in depth)
    fn validate_address(address: &crate::dcc::DccAddress) -> Result<(), ValidationError> {
        let val = address.value();
        if address.is_short() {
            if val == 0 || val > 127 {
                return Err(ValidationError::InvalidAddress);
            }
        } else {
            // V6: Long address first byte must be in 0xC0-0xE7 range
            // meaning address range 128-10239
            if !(128..=10239).contains(&val) {
                return Err(ValidationError::InvalidAddress);
            }
        }
        Ok(())
    }

    /// Validates packet checksum
    pub fn validate_checksum(packet: &DccPacket) -> Result<(), ValidationError> {
        let bytes = packet
            .to_bytes()
            .map_err(|_| ValidationError::EncodingError)?;

        // Calculate expected checksum (XOR of all bytes except last)
        let mut expected = 0u8;
        for i in 0..(bytes.len() - 1) {
            expected ^= bytes[i];
        }

        let actual = bytes[bytes.len() - 1];

        if expected != actual {
            return Err(ValidationError::InvalidChecksum { expected, actual });
        }

        Ok(())
    }

    /// Full validation: timing + structure + compliance + checksum
    ///
    /// Validates both the logical packet and its physical pulse encoding.
    /// Runs all four validation stages in sequence.
    ///
    /// # When to use this vs `validate_complete()`
    ///
    /// - **Use `validate_full()`** (this function) when:
    ///   - You already have the encoded pulse sequence (from `encode_dcc_packet()`)
    ///   - You want to validate intermediate transmission state
    ///   - You need to inspect pulse timing separately
    ///   - Example: DCC engine validates pulses before transmission
    ///
    /// - **Use `validate_complete()` instead** when:
    ///   - You only have a packet and want automatic encoding
    ///   - You want the simplest one-call API
    ///   - Example: Z21 server validates received command
    pub fn validate_full(packet: &DccPacket, pulses: &[PulseCode]) -> Result<(), ValidationError> {
        Self::validate_timing(pulses)?;
        Self::validate_packet_structure(pulses)?;
        Self::validate_nmra_compliance(packet)?;
        Self::validate_checksum(packet)?;
        Ok(())
    }

    /// Complete validation in one step: encode then validate everything
    ///
    /// Convenience method that encodes the packet and validates timing, structure,
    /// NMRA compliance, and checksum in a single call.
    ///
    /// # When to use this vs `validate_full()`
    ///
    /// - **Use `validate_complete()`** (this function) when:
    ///   - You have a `DccPacket` and want to validate it end-to-end
    ///   - You don't already have the encoded pulse sequence
    ///   - You want the simplest API (one function call)
    ///   - Example: Z21 server receives a command, creates a packet, validates it immediately
    ///
    /// - **Use `validate_full(packet, pulses)`** when:
    ///   - You already have both the packet AND the pulse sequence
    ///   - You want to validate a pre-encoded transmission (diagnostic use)
    ///   - You need to inspect intermediate pulse data
    ///   - Example: DCC engine loop transmits pulses; validator checks they match spec
    ///
    /// # Errors
    ///
    /// Returns `ValidationError` if encoding fails, or any validation check fails
    /// (timing, structure, address/speed ranges, checksum).
    ///
    /// # Examples
    ///
    /// ```
    /// use dcc_esp32::dcc::{DccValidator, DccPacket, DccAddress, Direction};
    ///
    /// let addr = DccAddress::short(42).unwrap();
    /// let packet = DccPacket::speed_128step(addr, 75, Direction::Forward).unwrap();
    ///
    /// // Single call validates everything: encode + timing + structure + compliance + checksum
    /// assert!(DccValidator::validate_complete(&packet).is_ok());
    /// ```
    pub fn validate_complete(packet: &DccPacket) -> Result<(), ValidationError> {
        // Encode packet to pulses; return EncodingError if packet cannot be encoded
        let pulses =
            crate::dcc::encode_dcc_packet(packet).map_err(|_| ValidationError::EncodingError)?;

        // Run full validation pipeline
        Self::validate_full(packet, &pulses)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dcc::{DccAddress, Direction, NmraSpeed128, NmraSpeed28, encode_dcc_packet};

    // --- Positive tests ---

    #[test]
    fn test_validate_timing_correct() {
        let packet = DccPacket::idle();
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_timing(&pulses).is_ok());
    }

    #[test]
    fn test_validate_structure_correct() {
        let packet = DccPacket::idle();
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_packet_structure(&pulses).is_ok());
    }

    #[test]
    fn test_validate_checksum_correct() {
        let packet = DccPacket::idle();
        assert!(DccValidator::validate_checksum(&packet).is_ok());
    }

    #[test]
    fn test_validate_speed28_range() {
        let addr = DccAddress::short(3).unwrap();
        let valid = DccPacket::speed_28step(addr, 28, Direction::Forward).unwrap();
        assert!(DccValidator::validate_nmra_compliance(&valid).is_ok());
    }

    #[test]
    fn test_full_validation() {
        let addr = DccAddress::short(3).unwrap();
        let packet = DccPacket::speed_28step(addr, 15, Direction::Forward).unwrap();
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_full_validation_speed128() {
        let addr = DccAddress::long(1000).unwrap();
        let packet = DccPacket::speed_128step(addr, 64, Direction::Reverse).unwrap();
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_full_validation_function_group1() {
        let addr = DccAddress::short(5).unwrap();
        let packet = DccPacket::FunctionGroup1 {
            address: addr,
            fl: true,
            f1: true,
            f2: false,
            f3: false,
            f4: true,
        };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_full_validation_emergency_stop() {
        let addr = DccAddress::short(3).unwrap();
        let packet = DccPacket::EmergencyStop {
            address: addr,
            direction: Direction::Forward,
        };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_full_validation_broadcast_stop() {
        let packet = DccPacket::BroadcastStop;
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
    }

    // --- Negative tests (V7, T2) ---

    #[test]
    fn test_timing_invalid_50us() {
        let bad_pulse = PulseCode::new(true, 50, false, 50);
        let result = DccValidator::validate_timing(&[bad_pulse]);
        assert!(matches!(
            result,
            Err(ValidationError::TimingOutOfRange { .. })
        ));
    }

    #[test]
    fn test_timing_invalid_75us_gap() {
        // 75μs is in the gap between valid "1" (55-61) and valid "0" (95+)
        let bad_pulse = PulseCode::new(true, 75, false, 75);
        let result = DccValidator::validate_timing(&[bad_pulse]);
        assert!(matches!(
            result,
            Err(ValidationError::TimingOutOfRange { actual_us: 75, .. })
        ));
    }

    #[test]
    fn test_timing_invalid_80us_gap() {
        let bad_pulse = PulseCode::new(true, 80, false, 80);
        let result = DccValidator::validate_timing(&[bad_pulse]);
        assert!(matches!(
            result,
            Err(ValidationError::TimingOutOfRange { actual_us: 80, .. })
        ));
    }

    #[test]
    fn test_preamble_too_short() {
        // Build a pulse sequence with only 10 preamble bits
        let one = PulseCode::new(true, 58, false, 58);
        let zero = PulseCode::new(true, 100, false, 100);
        let mut pulses = heapless::Vec::<PulseCode, 64>::new();
        for _ in 0..10 {
            let _ = pulses.push(one);
        }
        let _ = pulses.push(zero); // start bit
        // 8 data bits
        for _ in 0..8 {
            let _ = pulses.push(one);
        }
        let _ = pulses.push(zero); // separator
        for _ in 0..8 {
            let _ = pulses.push(zero);
        }
        let _ = pulses.push(zero); // separator
        for _ in 0..8 {
            let _ = pulses.push(one);
        }
        let _ = pulses.push(one); // end bit

        let result = DccValidator::validate_packet_structure(&pulses);
        assert!(matches!(
            result,
            Err(ValidationError::InsufficientPreamble { found: 10 })
        ));
    }

    #[test]
    fn test_speed28_out_of_range_rejected_at_construction() {
        // Invalid speeds are now rejected by NmraSpeed28 newtype
        assert!(NmraSpeed28::new(30).is_none());
        assert!(NmraSpeed28::new(255).is_none());
    }

    #[test]
    fn test_speed128_out_of_range_rejected_at_construction() {
        // Invalid speeds are now rejected by NmraSpeed128 newtype
        assert!(NmraSpeed128::new(127).is_none());
        assert!(NmraSpeed128::new(255).is_none());
    }

    #[test]
    fn test_checksum_all_packet_types() {
        // Verify checksum is valid for every packet type
        let addr = DccAddress::short(3).unwrap();
        let packets: &[DccPacket] = &[
            DccPacket::Idle,
            DccPacket::Reset,
            DccPacket::speed_28step(addr, 15, Direction::Forward).unwrap(),
            DccPacket::speed_128step(addr, 64, Direction::Forward).unwrap(),
            DccPacket::BroadcastStop,
            DccPacket::EmergencyStop {
                address: addr,
                direction: Direction::Forward,
            },
            DccPacket::FunctionGroup1 {
                address: addr,
                fl: true,
                f1: false,
                f2: true,
                f3: false,
                f4: true,
            },
            DccPacket::FunctionGroup2A {
                address: addr,
                f5: true,
                f6: true,
                f7: false,
                f8: false,
            },
            DccPacket::FunctionGroup2B {
                address: addr,
                f9: false,
                f10: true,
                f11: false,
                f12: true,
            },
        ];
        for packet in packets {
            assert!(
                DccValidator::validate_checksum(packet).is_ok(),
                "Checksum failed for {:?}",
                packet
            );
        }
    }

    #[test]
    fn test_structure_validation_all_packet_types() {
        let addr = DccAddress::short(3).unwrap();
        let packets: &[DccPacket] = &[
            DccPacket::Idle,
            DccPacket::Reset,
            DccPacket::speed_28step(addr, 15, Direction::Forward).unwrap(),
            DccPacket::speed_128step(addr, 64, Direction::Forward).unwrap(),
            DccPacket::BroadcastStop,
            DccPacket::EmergencyStop {
                address: addr,
                direction: Direction::Forward,
            },
            DccPacket::FunctionGroup1 {
                address: addr,
                fl: true,
                f1: false,
                f2: true,
                f3: false,
                f4: true,
            },
        ];
        for packet in packets {
            let pulses = encode_dcc_packet(packet).unwrap();
            assert!(
                DccValidator::validate_packet_structure(&pulses).is_ok(),
                "Structure validation failed for {:?}",
                packet
            );
        }
    }

    #[test]
    fn test_packet_too_short() {
        let one = PulseCode::new(true, 58, false, 58);
        let pulses = [one; 10];
        let result = DccValidator::validate_packet_structure(&pulses);
        assert!(matches!(
            result,
            Err(ValidationError::PacketTooShort { .. })
        ));
    }

    #[test]
    fn test_full_validation_function_group3() {
        let addr = DccAddress::short(5).unwrap();
        let packet = DccPacket::FunctionGroup3 {
            address: addr,
            f13: true,
            f14: false,
            f15: true,
            f16: false,
            f17: false,
            f18: true,
            f19: false,
            f20: true,
        };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_full_validation_function_group4() {
        let addr = DccAddress::short(7).unwrap();
        let packet = DccPacket::FunctionGroup4 {
            address: addr,
            f21: false,
            f22: true,
            f23: false,
            f24: true,
            f25: true,
            f26: false,
            f27: false,
            f28: true,
        };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_full_validation_function_group3_long_address() {
        let addr = DccAddress::long(1000).unwrap();
        let packet = DccPacket::FunctionGroup3 {
            address: addr,
            f13: true,
            f14: false,
            f15: true,
            f16: false,
            f17: false,
            f18: true,
            f19: false,
            f20: true,
        };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_full_validation_function_group4_long_address() {
        let addr = DccAddress::long(1000).unwrap();
        let packet = DccPacket::FunctionGroup4 {
            address: addr,
            f21: false,
            f22: true,
            f23: false,
            f24: true,
            f25: true,
            f26: false,
            f27: false,
            f28: true,
        };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_service_mode_verify_byte_validation() {
        let packet = DccPacket::ServiceModeVerifyByte { cv: 1, value: 3 };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_service_mode_write_byte_validation() {
        let packet = DccPacket::ServiceModeWriteByte { cv: 29, value: 6 };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(DccValidator::validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_service_mode_cv_range() {
        // CV 1-256 are valid in V1
        let packet1 = DccPacket::ServiceModeVerifyByte { cv: 1, value: 0 };
        assert!(DccValidator::validate_nmra_compliance(&packet1).is_ok());

        let packet256 = DccPacket::ServiceModeVerifyByte { cv: 256, value: 0 };
        assert!(DccValidator::validate_nmra_compliance(&packet256).is_ok());

        // CV 0 is invalid
        let packet0 = DccPacket::ServiceModeVerifyByte { cv: 0, value: 0 };
        assert!(matches!(
            DccValidator::validate_nmra_compliance(&packet0),
            Err(ValidationError::InvalidAddress)
        ));

        // CV > 256 is invalid in V1
        let packet257 = DccPacket::ServiceModeVerifyByte { cv: 257, value: 0 };
        assert!(matches!(
            DccValidator::validate_nmra_compliance(&packet257),
            Err(ValidationError::InvalidAddress)
        ));
    }

    #[test]
    fn test_service_mode_checksum() {
        let packet_verify = DccPacket::ServiceModeVerifyByte { cv: 17, value: 192 };
        assert!(DccValidator::validate_checksum(&packet_verify).is_ok());

        let packet_write = DccPacket::ServiceModeWriteByte { cv: 29, value: 6 };
        assert!(DccValidator::validate_checksum(&packet_write).is_ok());
    }

    // --- validate_complete() tests (one-shot validation) ---

    #[test]
    fn test_validate_complete_speed128() {
        let addr = DccAddress::short(42).unwrap();
        let packet = DccPacket::speed_128step(addr, 75, Direction::Forward).unwrap();
        // Single call: encode + validate everything
        assert!(DccValidator::validate_complete(&packet).is_ok());
    }

    #[test]
    fn test_validate_complete_speed28() {
        let addr = DccAddress::long(5000).unwrap();
        let packet = DccPacket::speed_28step(addr, 20, Direction::Reverse).unwrap();
        assert!(DccValidator::validate_complete(&packet).is_ok());
    }

    #[test]
    fn test_validate_complete_idle() {
        let packet = DccPacket::Idle;
        assert!(DccValidator::validate_complete(&packet).is_ok());
    }

    #[test]
    fn test_validate_complete_emergency_stop() {
        let addr = DccAddress::short(10).unwrap();
        let packet = DccPacket::EmergencyStop {
            address: addr,
            direction: Direction::Forward,
        };
        assert!(DccValidator::validate_complete(&packet).is_ok());
    }

    #[test]
    fn test_validate_complete_function_group1() {
        let addr = DccAddress::short(7).unwrap();
        let packet = DccPacket::FunctionGroup1 {
            address: addr,
            fl: true,
            f1: true,
            f2: false,
            f3: true,
            f4: false,
        };
        assert!(DccValidator::validate_complete(&packet).is_ok());
    }

    #[test]
    fn test_validate_complete_service_mode() {
        let packet = DccPacket::ServiceModeVerifyByte { cv: 1, value: 42 };
        assert!(DccValidator::validate_complete(&packet).is_ok());
    }

    #[test]
    fn test_validate_complete_all_packet_types() {
        // Test validate_complete on a variety of packet types
        let addr = DccAddress::short(3).unwrap();
        let packets: &[DccPacket] = &[
            DccPacket::Idle,
            DccPacket::Reset,
            DccPacket::speed_28step(addr, 15, Direction::Forward).unwrap(),
            DccPacket::speed_128step(addr, 64, Direction::Reverse).unwrap(),
            DccPacket::BroadcastStop,
            DccPacket::EmergencyStop {
                address: addr,
                direction: Direction::Forward,
            },
            DccPacket::FunctionGroup1 {
                address: addr,
                fl: true,
                f1: true,
                f2: false,
                f3: true,
                f4: false,
            },
            DccPacket::ServiceModeVerifyByte { cv: 29, value: 192 },
            DccPacket::ServiceModeWriteByte { cv: 17, value: 100 },
        ];

        for packet in packets {
            assert!(
                DccValidator::validate_complete(packet).is_ok(),
                "validate_complete failed for {:?}",
                packet
            );
        }
    }
}
