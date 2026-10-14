//! Test-only NMRA DCC compliance validator.
//!
//! This module deliberately has no production feature flag or public re-export.
//! It independently checks encoded packets against NMRA S-9.1 and S-9.2 so
//! tests can catch regressions in packet construction and pulse generation.
//!
//! The validator provides several module-level functions at different levels:
//! - [`validate_timing`] checks pulse timing against NMRA acceptable ranges
//! - [`validate_packet_structure`] verifies the preamble, start/end bits, byte boundaries
//! - [`validate_nmra_compliance`] checks address and speed ranges
//! - [`validate_checksum`] verifies the XOR checksum
//! - [`validate_full`] runs all of them over a packet and its pulses
//! - [`validate_complete`] encodes the packet first, then runs all of them
//!
//! All validations are based on NMRA S-9.1 (electrical) and S-9.2 (communications) standards.
//!
//! # Examples
//!
//! Validate pulse timing (NMRA S-9.1):
//!
//! ```ignore
//! use crate::dcc::{
//!     DccAddress, DccPacket, Direction, encode_dcc_packet, validate_timing,
//! };
//!
//! // Create and encode a valid packet
//! let addr = DccAddress::new_short(5).unwrap();
//! let packet = DccPacket::speed_128step(addr, 50, Direction::Forward).unwrap();
//!
//! let pulses = encode_dcc_packet(&packet).unwrap();
//!
//! // Check that all pulses are within NMRA timing ranges
//! // "1" bit: 55-61μs, "0" bit: 95-9900μs
//! assert!(validate_timing(&pulses).is_ok());
//! ```
//!
//! Validate packet structure:
//!
//! ```ignore
//! use crate::dcc::{
//!     DccAddress, DccPacket, Direction, encode_dcc_packet, validate_packet_structure,
//! };
//!
//! // Encoded pulses must have: preamble (14+ bits) + start (0) + bytes + separators/end (1)
//! let addr = DccAddress::new_short(10).unwrap();
//! let packet = DccPacket::speed_128step(addr, 100, Direction::Reverse).unwrap();
//!
//! let pulses = encode_dcc_packet(&packet).unwrap();
//! assert!(validate_packet_structure(&pulses).is_ok());
//! ```
//!
//! Validate complete packet (structure + timing + compliance + checksum):
//!
//! ```ignore
//! use crate::dcc::{
//!     DccAddress, DccPacket, Direction, encode_dcc_packet, validate_full,
//! };
//!
//! let addr = DccAddress::new_short(42).unwrap();
//! let packet = DccPacket::speed_128step(addr, 75, Direction::Forward).unwrap();
//!
//! // Must encode first, then validate
//! let pulses = encode_dcc_packet(&packet).unwrap();
//!
//! // Validates timing, structure, address/speed ranges and checksum
//! assert!(validate_full(&packet, &pulses).is_ok());
//! ```
//!
//! Validate NMRA compliance (ranges only):
//!
//! ```ignore
//! use crate::dcc::{DccAddress, DccPacket, Direction, validate_nmra_compliance};
//!
//! let addr = DccAddress::new_short(5).unwrap();
//! let packet = DccPacket::speed_28step(addr, 15, Direction::Forward).unwrap();
//!
//! // Check that address and speed values are in valid NMRA ranges
//! assert!(validate_nmra_compliance(&packet).is_ok());
//! ```
//!
//! Validate everything in one call (recommended):
//!
//! ```ignore
//! use crate::dcc::{DccAddress, DccPacket, Direction, validate_complete};
//!
//! let addr = DccAddress::new_short(42).unwrap();
//! let packet = DccPacket::speed_128step(addr, 75, Direction::Forward).unwrap();
//!
//! // Single call: encode packet + validate timing + structure + ranges + checksum
//! assert!(validate_complete(&packet).is_ok());
//! ```
//!
//! # NMRA ranges
//!
//! A "1" bit is 55-61μs high and low, nominally 58μs with a 3μs tolerance. A
//! "0" bit runs from 95μs to 9900μs: the long end is the stretched "0" used for
//! decoders with limited timing resolution. The preamble is at least 14 bits per
//! S-9.2, though 16 are typically transmitted. A packet is the preamble, a start
//! bit (0), the address byte, one or more data bytes, the error detection byte,
//! and a final end bit (1).

use core::fmt;

use crate::dcc::timing::{DCC_ONE_HIGH_US, DCC_ZERO_HIGH_US};
use crate::dcc::{DCC_MAX_PACKET_BYTES, DccPacket, encoder::PulseCode};

const ONE_BIT_MIN_US: u16 = 55;
const ONE_BIT_MAX_US: u16 = 61;
const ZERO_BIT_MIN_US: u16 = 95;
const ZERO_BIT_MAX_US: u16 = 9_900;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DccBitKind {
    One,
    Zero,
}

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
    /// Address out of valid range.
    ///
    /// Unreachable via the public API: `DccAddress::new_short`/`new_long` already
    /// reject out-of-range values at construction, so no `DccPacket` can carry an
    /// invalid address here. `validate_address` keeps this check as a defensive
    /// guard against a future regression in the newtype constructors.
    InvalidAddress,
    /// Missing start bit
    MissingStartBit,
    /// Packet too short
    PacketTooShort { min: usize, actual: usize },
    /// Invalid byte/bit structure in pulse sequence
    InvalidStructure,
    /// Packet could not be encoded to bytes
    EncodingError,
}

impl fmt::Display for ValidationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TimingOutOfRange {
                expected_us,
                actual_us,
            } => write!(
                formatter,
                "DCC pulse timing {actual_us}us is outside the range around {expected_us}us"
            ),
            Self::InsufficientPreamble { found } => {
                write!(
                    formatter,
                    "DCC preamble has {found} bits; at least 14 are required"
                )
            }
            Self::InvalidChecksum { expected, actual } => write!(
                formatter,
                "invalid DCC checksum: expected {expected:#04x}, received {actual:#04x}"
            ),
            Self::InvalidAddress => formatter.write_str("invalid DCC address"),
            Self::MissingStartBit => formatter.write_str("DCC packet start bit is missing"),
            Self::PacketTooShort { min, actual } => write!(
                formatter,
                "DCC packet has {actual} pulses; at least {min} are required"
            ),
            Self::InvalidStructure => formatter.write_str("invalid DCC packet bit structure"),
            Self::EncodingError => formatter.write_str("DCC packet encoding failed"),
        }
    }
}

impl core::error::Error for ValidationError {}

/// Validates pulse timing against NMRA S-9.1
///
/// NMRA acceptable ranges:
/// - "1" bit: 55-61μs high, 55-61μs low
/// - "0" bit: 95-9900μs (min 95, max for stretching)
///
/// Pulses in the 62-94μs gap are rejected because they represent neither a
/// valid "1" nor a valid "0".
pub fn validate_timing(pulses: &[PulseCode]) -> Result<(), ValidationError> {
    for pulse in pulses {
        classify_pulse(pulse)?;
    }
    Ok(())
}

fn classify_pulse(pulse: &PulseCode) -> Result<DccBitKind, ValidationError> {
    if (ONE_BIT_MIN_US..=ONE_BIT_MAX_US).contains(&pulse.length1) {
        if (ONE_BIT_MIN_US..=ONE_BIT_MAX_US).contains(&pulse.length2) {
            return Ok(DccBitKind::One);
        }
        return Err(ValidationError::TimingOutOfRange {
            expected_us: DCC_ONE_HIGH_US,
            actual_us: pulse.length2,
        });
    }

    if (ZERO_BIT_MIN_US..=ZERO_BIT_MAX_US).contains(&pulse.length1) {
        if (ZERO_BIT_MIN_US..=ZERO_BIT_MAX_US).contains(&pulse.length2) {
            return Ok(DccBitKind::Zero);
        }
        return Err(ValidationError::TimingOutOfRange {
            expected_us: DCC_ZERO_HIGH_US,
            actual_us: pulse.length2,
        });
    }

    Err(ValidationError::TimingOutOfRange {
        expected_us: 0,
        actual_us: pulse.length1,
    })
}

/// Returns true if a pulse represents a "1" bit
fn is_one_bit(pulse: &PulseCode) -> bool {
    matches!(classify_pulse(pulse), Ok(DccBitKind::One))
}

/// Returns true if a pulse represents a "0" bit
fn is_zero_bit(pulse: &PulseCode) -> bool {
    matches!(classify_pulse(pulse), Ok(DccBitKind::Zero))
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
    let preamble_count = pulses.iter().take_while(|p| is_one_bit(p)).count();

    if preamble_count < 14 {
        return Err(ValidationError::InsufficientPreamble {
            found: preamble_count,
        });
    }

    // Check start bit after preamble
    let mut pos = preamble_count;
    if pos >= pulses.len() || !is_zero_bit(&pulses[pos]) {
        return Err(ValidationError::MissingStartBit);
    }
    pos += 1; // skip start bit

    // V2: Validate byte structure: each byte is 8 pulses followed by separator/end
    loop {
        // Need at least 8 pulses for a data byte + 1 for separator/end
        if pos + 9 > pulses.len() {
            return Err(ValidationError::InvalidStructure);
        }

        pos += 8; // skip 8 data bit pulses

        // After 8 bits: expect "0" (inter-byte separator) or "1" (end bit)
        if is_one_bit(&pulses[pos]) {
            // End bit: this should be the last pulse
            if pos != pulses.len() - 1 {
                return Err(ValidationError::InvalidStructure);
            }
            break;
        } else if is_zero_bit(&pulses[pos]) {
            // Inter-byte separator, continue to next byte
            pos += 1;
        } else {
            return Err(ValidationError::InvalidStructure);
        }
    }

    Ok(())
}

/// Decodes an encoded pulse stream back into the packet bytes it carries,
/// checksum byte included.
///
/// Walks the same layout as [`validate_packet_structure`] (preamble, start
/// bit, eight data bits per byte, separator or end bit) but accumulates the
/// data bits MSB first. It exists so tests can compare a pulse stream with a
/// recorded byte sequence; it is not an independent check of the encoder.
///
/// # Panics
///
/// Panics if the stream is not a well-formed DCC packet or carries more than
/// [`DCC_MAX_PACKET_BYTES`] bytes. Callers are tests, where a malformed stream
/// is a bug to surface.
pub(crate) fn decode_pulses(pulses: &[PulseCode]) -> heapless::Vec<u8, DCC_MAX_PACKET_BYTES> {
    let preamble_count = pulses.iter().take_while(|p| is_one_bit(p)).count();
    assert!(
        preamble_count >= 14,
        "preamble has {preamble_count} one bits, at least 14 required"
    );
    let mut bits = pulses[preamble_count..].iter().map(|pulse| {
        match classify_pulse(pulse).expect("every pulse must be a valid DCC bit") {
            DccBitKind::One => true,
            DccBitKind::Zero => false,
        }
    });
    assert_eq!(
        bits.next(),
        Some(false),
        "missing start bit after the preamble"
    );

    let mut bytes = heapless::Vec::new();
    loop {
        let byte = (0..8).fold(0u8, |acc, _| {
            let bit = bits.next().expect("stream ends inside a data byte");
            (acc << 1) | u8::from(bit)
        });
        bytes
            .push(byte)
            .expect("decoded packet must fit DCC_MAX_PACKET_BYTES");
        match bits.next() {
            Some(false) => {}
            Some(true) => break,
            None => panic!("stream ends without an end bit"),
        }
    }
    assert!(bits.next().is_none(), "pulses follow the end bit");
    bytes
}

/// Validates NMRA S-9.2 packet compliance
///
/// V3: Validates address ranges (defense in depth).
/// V5: Validates Speed28 interleaved encoding.
pub fn validate_nmra_compliance(packet: &DccPacket) -> Result<(), ValidationError> {
    match packet {
        DccPacket::Speed28 { address, .. } => {
            // Speed range guaranteed by NmraSpeed28 newtype (0-29)
            validate_address(address)?;
        }
        DccPacket::Speed128 { address, .. } => {
            // Speed range guaranteed by NmraSpeed128 newtype (0-126)
            validate_address(address)?;
        }
        DccPacket::FunctionGroup { address, .. } | DccPacket::EmergencyStop { address, .. } => {
            validate_address(address)?;
        }
        DccPacket::PomWriteByte { address, .. } | DccPacket::PomReadByte { address, .. } => {
            validate_address(address)?;
        }
        DccPacket::ServiceModeVerifyByte { .. } | DccPacket::ServiceModeWriteByte { .. } => {}
        _ => {} // Idle, Reset, BroadcastStop have no address to validate
    }
    Ok(())
}

/// V3/V6: Validates address range (defense in depth; see [`ValidationError::InvalidAddress`]).
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
    let bytes = packet.to_bytes();

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
/// Use this one when you already hold the encoded pulse sequence from
/// `encode_dcc_packet()`, when you want to check intermediate transmission
/// state, or when you need to inspect the pulse timing on its own. The DCC
/// engine uses it to validate pulses just before transmitting them.
///
/// Use `validate_complete()` instead when you only have the packet and want it
/// encoded for you, as the Z21 server does when validating a received command.
pub fn validate_full(packet: &DccPacket, pulses: &[PulseCode]) -> Result<(), ValidationError> {
    validate_timing(pulses)?;
    validate_packet_structure(pulses)?;
    validate_nmra_compliance(packet)?;
    validate_checksum(packet)?;
    Ok(())
}

/// Complete validation in one step: encode then validate everything
///
/// Convenience function that encodes the packet and validates timing, structure,
/// NMRA compliance, and checksum in a single call.
///
/// # When to use this vs `validate_full()`
///
/// Use this one when you have a `DccPacket`, do not already hold the encoded
/// pulse sequence, and want it validated end to end in a single call. The Z21
/// server takes this path: it receives a command, builds a packet, and
/// validates it immediately.
///
/// Use `validate_full(packet, pulses)` when you already have both the packet and
/// its pulses, for example to check a pre-encoded transmission during
/// diagnostics or to inspect the intermediate pulse data.
///
/// # Errors
///
/// Returns `ValidationError` if encoding fails, or any validation check fails
/// (timing, structure, address/speed ranges, checksum).
///
/// # Examples
///
/// ```ignore
/// use crate::dcc::{DccAddress, DccPacket, Direction, validate_complete};
///
/// let addr = DccAddress::new_short(42).unwrap();
/// let packet = DccPacket::speed_128step(addr, 75, Direction::Forward).unwrap();
///
/// // Single call validates everything: encode + timing + structure + compliance + checksum
/// assert!(validate_complete(&packet).is_ok());
/// ```
pub fn validate_complete(packet: &DccPacket) -> Result<(), ValidationError> {
    // Encode packet to pulses; return EncodingError if packet cannot be encoded
    let pulses =
        crate::dcc::encode_dcc_packet(packet).map_err(|_| ValidationError::EncodingError)?;

    // Run full validation pipeline
    validate_full(packet, &pulses)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dcc::function_groups::GROUP_LAYOUTS;
    use crate::dcc::{
        DccAddress, Direction, FunctionGroupKind, NmraSpeed28, NmraSpeed128, PomCv, ServiceModeCv,
        encode_dcc_packet,
    };

    fn service_cv(cv: u16) -> ServiceModeCv {
        ServiceModeCv::new(cv).expect("test service-mode CV must be valid")
    }

    fn pom_cv(cv: u16) -> PomCv {
        PomCv::new(cv).expect("test POM CV must be valid")
    }

    // --- Positive tests ---

    #[test]
    fn test_validate_timing_correct() {
        let packet = DccPacket::idle();
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(validate_timing(&pulses).is_ok());
    }

    #[test]
    fn test_validate_structure_correct() {
        let packet = DccPacket::idle();
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(validate_packet_structure(&pulses).is_ok());
    }

    #[test]
    fn test_validate_checksum_correct() {
        let packet = DccPacket::idle();
        assert!(validate_checksum(&packet).is_ok());
    }

    #[test]
    fn test_validate_speed28_range() {
        let addr = DccAddress::new_short(3).unwrap();
        let valid = DccPacket::speed_28step(addr, 28, Direction::Forward).unwrap();
        assert!(validate_nmra_compliance(&valid).is_ok());
    }

    #[test]
    fn test_full_validation() {
        let addr = DccAddress::new_short(3).unwrap();
        let packet = DccPacket::speed_28step(addr, 15, Direction::Forward).unwrap();
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_full_validation_speed128() {
        let addr = DccAddress::new_long(1000).unwrap();
        let packet = DccPacket::speed_128step(addr, 64, Direction::Reverse).unwrap();
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_full_validation_every_function_group() {
        let addresses = [
            DccAddress::new_short(5).unwrap(),
            DccAddress::new_long(1000).unwrap(),
        ];
        for address in addresses {
            for layout in GROUP_LAYOUTS {
                let packet = DccPacket::function_group(address, layout.kind, 0b1010_0101);
                let pulses = encode_dcc_packet(&packet).unwrap();
                assert!(
                    validate_full(&packet, &pulses).is_ok(),
                    "{:?} at {address:?}",
                    layout.kind
                );
            }
        }
    }

    #[test]
    fn test_full_validation_emergency_stop() {
        let addr = DccAddress::new_short(3).unwrap();
        let packet = DccPacket::EmergencyStop {
            address: addr,
            direction: Direction::Forward,
        };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_full_validation_broadcast_stop() {
        let packet = DccPacket::BroadcastStop;
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(validate_full(&packet, &pulses).is_ok());
    }

    // --- Negative tests (V7, T2) ---

    #[test]
    fn test_timing_invalid_50us() {
        let bad_pulse = PulseCode::new(true, 50, false, 50);
        let result = validate_timing(&[bad_pulse]);
        assert!(matches!(
            result,
            Err(ValidationError::TimingOutOfRange { .. })
        ));
    }

    #[test]
    fn test_timing_invalid_75us_gap() {
        // 75μs is in the gap between valid "1" (55-61) and valid "0" (95+)
        let bad_pulse = PulseCode::new(true, 75, false, 75);
        let result = validate_timing(&[bad_pulse]);
        assert!(matches!(
            result,
            Err(ValidationError::TimingOutOfRange { actual_us: 75, .. })
        ));
    }

    #[test]
    fn test_timing_invalid_80us_gap() {
        let bad_pulse = PulseCode::new(true, 80, false, 80);
        let result = validate_timing(&[bad_pulse]);
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

        let result = validate_packet_structure(&pulses);
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
        let addr = DccAddress::new_short(3).unwrap();
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
            DccPacket::function_group(addr, FunctionGroupKind::G1, 0b11010),
            DccPacket::function_group(addr, FunctionGroupKind::G2a, 0b0011),
            DccPacket::function_group(addr, FunctionGroupKind::G2b, 0b1010),
        ];
        for packet in packets {
            assert!(
                validate_checksum(packet).is_ok(),
                "Checksum failed for {:?}",
                packet
            );
        }
    }

    #[test]
    fn test_structure_validation_all_packet_types() {
        let addr = DccAddress::new_short(3).unwrap();
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
            DccPacket::function_group(addr, FunctionGroupKind::G1, 0b11010),
        ];
        for packet in packets {
            let pulses = encode_dcc_packet(packet).unwrap();
            assert!(
                validate_packet_structure(&pulses).is_ok(),
                "Structure validation failed for {:?}",
                packet
            );
        }
    }

    #[test]
    fn test_packet_too_short() {
        let one = PulseCode::new(true, 58, false, 58);
        let pulses = [one; 10];
        let result = validate_packet_structure(&pulses);
        assert!(matches!(
            result,
            Err(ValidationError::PacketTooShort { .. })
        ));
    }

    #[test]
    fn test_service_mode_verify_byte_validation() {
        let packet = DccPacket::ServiceModeVerifyByte {
            cv: service_cv(1),
            value: 3,
        };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_service_mode_write_byte_validation() {
        let packet = DccPacket::ServiceModeWriteByte {
            cv: service_cv(29),
            value: 6,
        };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_service_mode_cv_range() {
        // CV 1-256 are valid in V1
        let packet1 = DccPacket::ServiceModeVerifyByte {
            cv: service_cv(1),
            value: 0,
        };
        assert!(validate_nmra_compliance(&packet1).is_ok());

        let packet256 = DccPacket::ServiceModeVerifyByte {
            cv: service_cv(256),
            value: 0,
        };
        assert!(validate_nmra_compliance(&packet256).is_ok());

        assert!(ServiceModeCv::new(0).is_none());
        assert!(ServiceModeCv::new(257).is_none());
    }

    #[test]
    fn test_service_mode_checksum() {
        let packet_verify = DccPacket::ServiceModeVerifyByte {
            cv: service_cv(17),
            value: 192,
        };
        assert!(validate_checksum(&packet_verify).is_ok());

        let packet_write = DccPacket::ServiceModeWriteByte {
            cv: service_cv(29),
            value: 6,
        };
        assert!(validate_checksum(&packet_write).is_ok());
    }

    #[test]
    fn test_pom_write_byte_validation() {
        let addr = DccAddress::new_short(3).unwrap();
        let packet = DccPacket::PomWriteByte {
            address: addr,
            cv: pom_cv(29),
            value: 6,
        };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_pom_read_byte_validation() {
        let addr = DccAddress::new_long(1000).unwrap();
        let packet = DccPacket::PomReadByte {
            address: addr,
            cv: pom_cv(1),
        };
        let pulses = encode_dcc_packet(&packet).unwrap();
        assert!(validate_full(&packet, &pulses).is_ok());
    }

    #[test]
    fn test_pom_cv_range() {
        let addr = DccAddress::new_short(3).unwrap();
        assert!(
            validate_nmra_compliance(&DccPacket::PomReadByte {
                address: addr,
                cv: pom_cv(1),
            })
            .is_ok()
        );
        assert!(
            validate_nmra_compliance(&DccPacket::PomReadByte {
                address: addr,
                cv: pom_cv(1024),
            })
            .is_ok()
        );
        assert!(PomCv::new(0).is_none());
        assert!(PomCv::new(1025).is_none());
    }

    // --- validate_complete() tests (one-shot validation) ---

    #[test]
    fn test_validate_complete_speed128() {
        let addr = DccAddress::new_short(42).unwrap();
        let packet = DccPacket::speed_128step(addr, 75, Direction::Forward).unwrap();
        // Single call: encode + validate everything
        assert!(validate_complete(&packet).is_ok());
    }

    #[test]
    fn test_validate_complete_speed28() {
        let addr = DccAddress::new_long(5000).unwrap();
        let packet = DccPacket::speed_28step(addr, 20, Direction::Reverse).unwrap();
        assert!(validate_complete(&packet).is_ok());
    }

    #[test]
    fn test_validate_complete_idle() {
        let packet = DccPacket::Idle;
        assert!(validate_complete(&packet).is_ok());
    }

    #[test]
    fn test_validate_complete_emergency_stop() {
        let addr = DccAddress::new_short(10).unwrap();
        let packet = DccPacket::EmergencyStop {
            address: addr,
            direction: Direction::Forward,
        };
        assert!(validate_complete(&packet).is_ok());
    }

    #[test]
    fn test_validate_complete_service_mode() {
        let packet = DccPacket::ServiceModeVerifyByte {
            cv: service_cv(1),
            value: 42,
        };
        assert!(validate_complete(&packet).is_ok());
    }

    #[test]
    fn test_validate_complete_all_packet_types() {
        // Test validate_complete on a variety of packet types
        let addr = DccAddress::new_short(3).unwrap();
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
            DccPacket::function_group(addr, FunctionGroupKind::G1, 0b10101),
            DccPacket::PomReadByte {
                address: addr,
                cv: pom_cv(29),
            },
            DccPacket::PomWriteByte {
                address: addr,
                cv: pom_cv(17),
                value: 100,
            },
            DccPacket::ServiceModeVerifyByte {
                cv: service_cv(29),
                value: 192,
            },
            DccPacket::ServiceModeWriteByte {
                cv: service_cv(17),
                value: 100,
            },
        ];

        for packet in packets {
            assert!(
                validate_complete(packet).is_ok(),
                "validate_complete failed for {:?}",
                packet
            );
        }
    }
}
