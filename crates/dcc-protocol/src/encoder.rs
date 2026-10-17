//! DCC packet to pulse sequence encoder
//!
//! Converts logical DCC packets into pulse sequences for a waveform generator.
//! Each DCC bit becomes one [`PulseCode`]: a high half followed by a low half,
//! with NMRA-compliant timing.
//!
//! The encoder implements NMRA S-9.1 timing:
//! - DCC "1" bit: 58μs ±3μs high, 58μs ±3μs low
//! - DCC "0" bit: ≥100μs high, ≥100μs low
//!
//! Packets are prepended with a preamble of [`PREAMBLE_BITS`] "1" bits (NMRA S-9.1 asks
//! for at least 14), followed by start bit (0), payload bytes, and end bit (1). All in
//! MSB-first order.
//!
//! [`encode_dcc_packet`] returns the whole packet. [`encode_dcc_data_portion`]
//! returns only the variable tail after the preamble, for a double-buffered
//! waveform generator that keeps the fixed preamble in place and rewrites the
//! tail of each packet.
//!
//! # Examples
//!
//! Encode a complete packet:
//!
//! ```
//! use dcc_protocol::{DccAddress, DccPacket, Direction, encode_dcc_packet};
//!
//! let addr = DccAddress::new_short(10).unwrap();
//! let packet = DccPacket::speed_128step(addr, 100, Direction::Forward).unwrap();
//!
//! // Encode to a pulse sequence (Vec<PulseCode>)
//! let pulses = encode_dcc_packet(&packet).expect("valid packet");
//! // Preamble, start bit, 8 bits per packet byte with a separator between
//! // bytes, end bit; this packet has 4 bytes (address, 0x3F, speed, checksum).
//! assert_eq!(pulses.len(), dcc_protocol::timing::PREAMBLE_BITS + 1 + 4 * 8 + 3 + 1);
//! ```
//!
//! Convert a single bit:
//!
//! ```
//! use dcc_protocol::dcc_bit_to_pulse;
//!
//! let one_bit = dcc_bit_to_pulse(true);
//! assert_eq!(one_bit.length1, 58); // 58μs high
//! assert_eq!(one_bit.length2, 58); // 58μs low
//!
//! let zero_bit = dcc_bit_to_pulse(false);
//! assert_eq!(zero_bit.length1, 100); // 100μs high (minimum)
//! ```
//!
//! Encode a byte:
//!
//! ```
//! use dcc_protocol::encode_byte;
//!
//! let pulses = encode_byte(0x3F); // 8 pulses (one per bit, MSB first)
//! assert_eq!(pulses.len(), 8);
//! ```
//!
//! # Error handling
//!
//! [`encode_dcc_packet`] returns [`EncodeError::PulseBufferOverflow`] if the pulse
//! sequence exceeds capacity (should not occur with valid input). Byte encoding
//! itself cannot fail: public packet constructors use validated newtypes for
//! address, speed, and CV ranges.

use core::fmt;

use crate::packet::DccPacket;
use crate::timing::{
    DCC_MAX_PACKET_PULSES, DCC_ONE_HIGH_US, DCC_ONE_LOW_US, DCC_ZERO_HIGH_US, DCC_ZERO_LOW_US,
    PREAMBLE_BITS,
};
use crate::timing::{MAX_DATA_PULSES, PREAMBLE_DURATION_US};
use heapless::Vec;

/// Errors produced while encoding packets into pulse sequences.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EncodeError {
    /// `PulseBufferOverflow` means the pulse output buffer capacity was exceeded.
    PulseBufferOverflow,
}

impl fmt::Display for EncodeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PulseBufferOverflow => formatter.write_str("DCC pulse buffer capacity exceeded"),
        }
    }
}

impl core::error::Error for EncodeError {}

/// One DCC bit as two timed levels (high, then low).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PulseCode {
    /// First level (typically high)
    pub level1: bool,
    /// First duration in microseconds
    pub length1: u16,
    /// Second level (typically low)
    pub level2: bool,
    /// Second duration in microseconds
    pub length2: u16,
}

/// Variable packet tail together with the duration of the full DCC packet.
#[derive(Debug)]
pub struct EncodedDccData {
    /// The start bit, the data bytes with their separators, and the end bit.
    pub pulses: Vec<PulseCode, MAX_DATA_PULSES>,
    /// Duration of the whole packet in microseconds, preamble included.
    pub dcc_duration_us: u32,
}

impl PulseCode {
    /// Create a pulse code from its two levels and their durations.
    #[must_use]
    pub const fn new(level1: bool, length1: u16, level2: bool, length2: u16) -> Self {
        Self {
            level1,
            length1,
            level2,
            length2,
        }
    }
}

/// Convert a DCC bit (true=1, false=0) to its pulse.
#[must_use]
pub fn dcc_bit_to_pulse(bit: bool) -> PulseCode {
    if bit {
        // DCC "1": 58μs high, 58μs low
        PulseCode::new(true, DCC_ONE_HIGH_US, false, DCC_ONE_LOW_US)
    } else {
        // DCC "0": 100μs high, 100μs low
        PulseCode::new(true, DCC_ZERO_HIGH_US, false, DCC_ZERO_LOW_US)
    }
}

/// Encode a byte as 8 DCC bits (MSB first)
#[must_use]
pub fn encode_byte(byte: u8) -> [PulseCode; 8] {
    let mut pulses = [dcc_bit_to_pulse(false); 8];

    for (i, pulse) in pulses.iter_mut().enumerate() {
        let bit = (byte >> (7 - i)) & 1 == 1;
        *pulse = dcc_bit_to_pulse(bit);
    }

    pulses
}

/// Encode a complete DCC packet to a pulse sequence.
///
/// Format: \[preamble\]\[start_bit\]\[data_bytes\]\[end_bit\]
///
/// [`DCC_MAX_PACKET_PULSES`] holds the longest packet; the timing module
/// checks that at compile time.
///
/// # Errors
/// Returns [`EncodeError::PulseBufferOverflow`] if the pulse buffer overflows.
pub fn encode_dcc_packet(
    packet: &DccPacket,
) -> Result<Vec<PulseCode, DCC_MAX_PACKET_PULSES>, EncodeError> {
    let mut pulses = Vec::new();

    // Preamble: PREAMBLE_BITS "1" bits
    for _ in 0..PREAMBLE_BITS {
        push_pulse(&mut pulses, dcc_bit_to_pulse(true))?;
    }

    encode_packet_tail(*packet, &mut pulses, push_pulse)?;

    Ok(pulses)
}

/// Encode only the variable packet tail, for generators that keep the fixed
/// preamble in place.
///
/// Format: \[start_bit\]\[data_bytes\]\[end_bit\]
///
/// # Errors
/// Returns [`EncodeError::PulseBufferOverflow`] if the pulse buffer overflows.
pub fn encode_dcc_data_portion(packet: &DccPacket) -> Result<EncodedDccData, EncodeError> {
    let mut pulses = Vec::new();
    let mut dcc_duration_us = PREAMBLE_DURATION_US;

    encode_packet_tail(*packet, &mut pulses, |pulses, pulse| {
        push_timed_pulse(pulses, &mut dcc_duration_us, pulse)
    })?;

    Ok(EncodedDccData {
        pulses,
        dcc_duration_us,
    })
}

fn encode_packet_tail<const N: usize>(
    packet: DccPacket,
    pulses: &mut Vec<PulseCode, N>,
    mut push: impl FnMut(&mut Vec<PulseCode, N>, PulseCode) -> Result<(), EncodeError>,
) -> Result<(), EncodeError> {
    push(pulses, dcc_bit_to_pulse(false))?;

    let bytes = packet.to_bytes();
    for (index, &byte) in bytes.iter().enumerate() {
        for pulse in encode_byte(byte) {
            push(pulses, pulse)?;
        }
        if index + 1 < bytes.len() {
            push(pulses, dcc_bit_to_pulse(false))?;
        }
    }

    push(pulses, dcc_bit_to_pulse(true))
}

fn push_timed_pulse(
    pulses: &mut Vec<PulseCode, MAX_DATA_PULSES>,
    dcc_duration_us: &mut u32,
    pulse: PulseCode,
) -> Result<(), EncodeError> {
    push_pulse(pulses, pulse)?;
    *dcc_duration_us += u32::from(pulse.length1) + u32::from(pulse.length2);
    Ok(())
}

/// Push a pulse into a fixed-capacity pulse buffer of any capacity `N`.
///
/// Shared by [`encode_dcc_packet`] (capacity `DCC_MAX_PACKET_PULSES`) and
/// [`encode_dcc_data_portion`] (capacity `MAX_DATA_PULSES`).
fn push_pulse<const N: usize>(
    pulses: &mut Vec<PulseCode, N>,
    pulse: PulseCode,
) -> Result<(), EncodeError> {
    pulses
        .push(pulse)
        .map_err(|_| EncodeError::PulseBufferOverflow)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FunctionGroupKind;
    use crate::packet::*;

    fn service_cv(cv: u16) -> ServiceModeCv {
        ServiceModeCv::new(cv).expect("test service-mode CV must be valid")
    }

    #[test]
    fn a_speed128_packet_encodes_to_the_documented_pulse_count() {
        let address = DccAddress::new_short(10).expect("10 is a valid short address");
        let packet =
            DccPacket::speed_128step(address, 100, Direction::Forward).expect("speed 100 is valid");

        let pulses = encode_dcc_packet(&packet).expect("valid packet");

        // Preamble, start bit, 4 bytes of 8 bits with 3 separators, end bit.
        assert_eq!(pulses.len(), PREAMBLE_BITS + 1 + 4 * 8 + 3 + 1);
    }

    #[test]
    fn a_one_bit_and_a_zero_bit_carry_the_documented_durations() {
        let one = dcc_bit_to_pulse(true);
        assert!(one.level1);
        assert_eq!(one.length1, 58);
        assert!(!one.level2);
        assert_eq!(one.length2, 58);

        let zero = dcc_bit_to_pulse(false);
        assert!(zero.level1);
        assert_eq!(zero.length1, 100);
        assert!(!zero.level2);
        assert_eq!(zero.length2, 100);
    }

    #[test]
    fn a_byte_encodes_to_one_pulse_per_bit() {
        let pulses = encode_byte(0b1010_1010);

        // MSB first: 1,0,1,0,1,0,1,0
        assert_eq!(pulses.len(), 8);
        assert_eq!(pulses[0].length1, 58); // 1
        assert_eq!(pulses[1].length1, 100); // 0
        assert_eq!(pulses[2].length1, 58); // 1
        assert_eq!(pulses[3].length1, 100); // 0
    }

    #[test]
    fn test_idle_packet_encoding() {
        let packet = DccPacket::idle();
        let pulses = encode_dcc_packet(&packet).unwrap();

        // Preamble, then a start bit before each of the 3 bytes, then the end bit.
        assert_eq!(pulses.len(), PREAMBLE_BITS + 3 * (1 + 8) + 1);

        // The preamble is all "1"s
        for i in 0..PREAMBLE_BITS {
            assert_eq!(pulses[i].length1, 58);
        }

        // First bit after preamble should be start bit "0"
        assert_eq!(pulses[PREAMBLE_BITS].length1, 100);
    }

    #[test]
    fn test_speed_packet_structure() {
        let addr = DccAddress::new_short(3).unwrap();
        let packet = DccPacket::speed_28step(addr, 10, Direction::Forward).unwrap();
        let pulses = encode_dcc_packet(&packet).unwrap();

        // Verify it's a valid length packet
        assert!(pulses.len() > 14); // At least preamble + some data

        // Preamble should be all "1"s (58μs)
        for i in 0..PREAMBLE_BITS {
            assert_eq!(pulses[i].length1, DCC_ONE_HIGH_US);
        }
    }

    #[test]
    fn test_encode_dcc_data_portion_matches_full_packet_tail() {
        let addr = DccAddress::new_short(3).unwrap();
        let packet = DccPacket::speed_28step(addr, 10, Direction::Forward).unwrap();

        let full = encode_dcc_packet(&packet).unwrap();
        let tail = encode_dcc_data_portion(&packet).unwrap();

        assert_eq!(&full[PREAMBLE_BITS..], tail.pulses.as_slice());
        assert_eq!(
            tail.dcc_duration_us,
            full.iter()
                .map(|pulse| u32::from(pulse.length1) + u32::from(pulse.length2))
                .sum::<u32>()
        );
    }

    #[test]
    fn every_packet_kind_encodes_to_one_pulse_per_bit() {
        let addr_short = DccAddress::new_short(3).unwrap();
        let addr_long = DccAddress::new_long(9999).unwrap();

        let packets = [
            DccPacket::Idle,
            DccPacket::Reset,
            DccPacket::speed_28step(addr_short, 28, Direction::Forward).unwrap(),
            DccPacket::speed_128step(addr_long, 126, Direction::Reverse).unwrap(),
            DccPacket::function_group(addr_long, FunctionGroupKind::G4, u8::MAX),
            DccPacket::ServiceModeVerifyByte {
                cv: service_cv(256),
                value: 255,
            },
            DccPacket::ServiceModeWriteByte {
                cv: service_cv(256),
                value: 255,
            },
            DccPacket::EmergencyStop {
                address: addr_long,
                direction: Direction::Forward,
            },
            DccPacket::LogonSelect {
                manufacturer_id: 0x0FFF,
                decoder_id: u32::MAX,
                subcommand: u8::MAX,
            },
        ];

        for packet in &packets {
            let pulses = encode_dcc_packet(packet)
                .unwrap_or_else(|e| panic!("encode failed for {packet:?}: {e:?}"));
            // Preamble, start bit, 8 bits per byte, a separator between
            // bytes, end bit.
            let bytes = packet.to_bytes().len();
            assert_eq!(
                pulses.len(),
                PREAMBLE_BITS + 1 + bytes * 8 + (bytes - 1) + 1,
                "{packet:?}"
            );
        }
    }
}
