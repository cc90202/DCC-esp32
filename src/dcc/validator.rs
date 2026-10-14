//! Test-only NMRA DCC compliance validator.
//!
//! This module deliberately has no production feature flag or public re-export.
//! It checks encoded pulse streams against NMRA S-9.1 and S-9.2 so tests can
//! catch regressions in packet construction and pulse generation. Every check
//! reads the pulse stream alone; none of them asks the packet for its bytes, so
//! a fault in the encoder cannot hide itself from the validator.
//!
//! - [`validate_timing`] checks pulse timing against NMRA acceptable ranges
//! - [`validate_packet_structure`] verifies the preamble, start/end bits, byte boundaries
//! - [`validate_checksum`] decodes the bytes on the wire and verifies the XOR checksum
//! - [`validate_full`] runs all of them over a pulse stream
//! - [`validate_complete`] encodes a packet first, then runs [`validate_full`]
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
    /// Missing start bit
    MissingStartBit,
    /// Packet too short
    PacketTooShort { min: usize, actual: usize },
    /// Packet carries more bytes than a DCC packet can hold
    PacketTooLong { max_bytes: usize },
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
            Self::MissingStartBit => formatter.write_str("DCC packet start bit is missing"),
            Self::PacketTooShort { min, actual } => write!(
                formatter,
                "DCC packet has {actual} pulses; at least {min} are required"
            ),
            Self::PacketTooLong { max_bytes } => {
                write!(formatter, "DCC packet carries more than {max_bytes} bytes")
            }
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
/// After preamble and first start bit, validates that every 8 pulses
/// is followed by a separator "0" (inter-byte) or end bit "1" (last byte),
/// and that the packet holds at most [`DCC_MAX_PACKET_BYTES`] bytes.
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

    // Validate byte structure: each byte is 8 pulses followed by separator/end
    let mut byte_count = 0;
    loop {
        if byte_count == DCC_MAX_PACKET_BYTES {
            return Err(ValidationError::PacketTooLong {
                max_bytes: DCC_MAX_PACKET_BYTES,
            });
        }
        byte_count += 1;

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

/// Validates the checksum carried by a pulse stream.
///
/// Decodes the bytes on the wire and checks that the last one is the XOR of
/// all the others (NMRA S-9.2 error detection byte).
///
/// # Errors
///
/// Returns [`ValidationError::InvalidChecksum`] when the error detection byte
/// does not match the XOR of the preceding bytes.
///
/// # Panics
///
/// Panics on a stream that [`validate_timing`] or
/// [`validate_packet_structure`] would reject, including one with more than
/// [`DCC_MAX_PACKET_BYTES`] bytes. [`validate_full`] runs both checks first,
/// so it returns their error instead of panicking.
pub fn validate_checksum(pulses: &[PulseCode]) -> Result<(), ValidationError> {
    let bytes = decode_pulses(pulses);
    let (&actual, payload) = bytes
        .split_last()
        .expect("a decoded packet holds at least one byte");
    let expected = payload.iter().fold(0u8, |acc, byte| acc ^ byte);

    if expected != actual {
        return Err(ValidationError::InvalidChecksum { expected, actual });
    }
    Ok(())
}

/// Validates a pulse stream end to end: timing, structure, then checksum.
///
/// # Errors
///
/// Returns the first [`ValidationError`] raised by [`validate_timing`],
/// [`validate_packet_structure`] or [`validate_checksum`].
pub fn validate_full(pulses: &[PulseCode]) -> Result<(), ValidationError> {
    validate_timing(pulses)?;
    validate_packet_structure(pulses)?;
    validate_checksum(pulses)
}

/// Encodes a packet and validates the resulting pulse stream with
/// [`validate_full`].
///
/// # Errors
///
/// Returns [`ValidationError::EncodingError`] if the packet cannot be encoded,
/// otherwise the first error raised by [`validate_full`].
pub fn validate_complete(packet: &DccPacket) -> Result<(), ValidationError> {
    let pulses =
        crate::dcc::encode_dcc_packet(packet).map_err(|_| ValidationError::EncodingError)?;
    validate_full(&pulses)
}

#[cfg(test)]
mod tests;
