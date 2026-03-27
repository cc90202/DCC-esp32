//! DCC packet to RMT PulseCode encoder
//!
//! Converts logical DCC packets into RMT pulse sequences for transmission via ESP32 RMT peripheral.
//! Each DCC bit becomes a pair of transitions (high→low) with NMRA-compliant timing.
//!
//! # Overview
//!
//! The encoder implements NMRA S-9.1 timing:
//! - DCC "1" bit: 58μs ±3μs high, 58μs ±3μs low
//! - DCC "0" bit: ≥100μs high, ≥100μs low
//!
//! Packets are prepended with a 14-bit preamble (sixteen "1" bits minus 2), followed by
//! start bit (0), payload bytes, and end bit (1). All in MSB-first order.
//!
//! # Examples
//!
//! **Encode a complete packet:**
//!
//! ```
//! use dcc_esp32::dcc::{DccPacket, DccAddress, Direction, encode_dcc_packet};
//!
//! let addr = DccAddress::short(10).unwrap();
//! let packet = DccPacket::speed_128step(addr, 100, Direction::Forward).unwrap();
//!
//! // Encode to RMT pulse sequence (Vec<PulseCode>)
//! let pulses = encode_dcc_packet(&packet).expect("valid packet");
//! println!("Generated {} pulses", pulses.len());
//! // Output: 14 (preamble) + 1 (start) + 16 (2 bytes × 8 bits) + 1 (end) = 32 pulses
//! ```
//!
//! **Convert a single bit:**
//!
//! ```
//! use dcc_esp32::dcc::dcc_bit_to_pulse;
//!
//! let one_bit = dcc_bit_to_pulse(true);
//! assert_eq!(one_bit.length1, 58); // 58μs high
//! assert_eq!(one_bit.length2, 58); // 58μs low
//!
//! let zero_bit = dcc_bit_to_pulse(false);
//! assert_eq!(zero_bit.length1, 100); // 100μs high (minimum)
//! ```
//!
//! **Encode a byte:**
//!
//! ```
//! use dcc_esp32::dcc::encoder::encode_byte;
//!
//! let pulses = encode_byte(0x3F); // 8 pulses (one per bit, MSB first)
//! assert_eq!(pulses.len(), 8);
//! ```
//!
//! # Error Handling
//!
//! [`encode_dcc_packet`] returns [`EncodeError::Packet`] if the packet cannot be encoded
//! to bytes (invalid CV address in Service Mode), or [`EncodeError::PulseBufferOverflow`]
//! if the pulse sequence exceeds capacity (should not occur with valid input).

use crate::dcc::packet::DccPacket;
use crate::dcc::timing::{
    DCC_MAX_PACKET_PULSES, DCC_ONE_HIGH_US, DCC_ONE_LOW_US, DCC_ZERO_HIGH_US, DCC_ZERO_LOW_US,
    PREAMBLE_BITS,
};
use heapless::Vec;

/// Errors produced while encoding packets into pulse sequences.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum EncodeError {
    /// `Packet` wraps errors returned by packet byte encoding.
    Packet(crate::dcc::packet::PacketEncodeError),
    /// `PulseBufferOverflow` means the pulse output buffer capacity was exceeded.
    PulseBufferOverflow,
}

impl From<crate::dcc::packet::PacketEncodeError> for EncodeError {
    fn from(value: crate::dcc::packet::PacketEncodeError) -> Self {
        Self::Packet(value)
    }
}

/// RMT pulse representation
///
/// Represents one DCC bit as two transitions (high→low)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
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

impl PulseCode {
    /// Create a new pulse code
    pub const fn new(level1: bool, length1: u16, level2: bool, length2: u16) -> Self {
        Self {
            level1,
            length1,
            level2,
            length2,
        }
    }
}

/// Convert a DCC bit (true=1, false=0) to RMT pulse
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
pub fn encode_byte(byte: u8) -> [PulseCode; 8] {
    let mut pulses = [dcc_bit_to_pulse(false); 8];

    for (i, pulse) in pulses.iter_mut().enumerate() {
        let bit = (byte >> (7 - i)) & 1 == 1;
        *pulse = dcc_bit_to_pulse(bit);
    }

    pulses
}

/// Encode a complete DCC packet to RMT pulse sequence.
///
/// Format: \[preamble\]\[start_bit\]\[data_bytes\]\[end_bit\]
///
/// Capacity: `DCC_MAX_PACKET_PULSES` pulses is sufficient for all standard DCC packets:
/// - Idle packet: 42 pulses
/// - Speed packet (long addr): ~60 pulses
/// - Max realistic packet: < `DCC_MAX_PACKET_PULSES`
///
/// # Errors
/// Returns [`EncodeError::Packet`] if packet byte encoding fails,
/// or [`EncodeError::PulseBufferOverflow`] if the pulse buffer overflows.
pub fn encode_dcc_packet(
    packet: &DccPacket,
) -> Result<Vec<PulseCode, DCC_MAX_PACKET_PULSES>, EncodeError> {
    let mut pulses = Vec::new();

    // Preamble: 14+ "1" bits
    for _ in 0..PREAMBLE_BITS {
        push_pulse(&mut pulses, dcc_bit_to_pulse(true))?;
    }

    // Packet start bit: "0"
    push_pulse(&mut pulses, dcc_bit_to_pulse(false))?;

    // Data bytes
    let bytes = packet.to_bytes()?;
    for (i, &byte) in bytes.iter().enumerate() {
        let byte_pulses = encode_byte(byte);
        for pulse in byte_pulses {
            push_pulse(&mut pulses, pulse)?;
        }

        // Inter-byte start bit: "0" (except after last byte)
        if i < bytes.len() - 1 {
            push_pulse(&mut pulses, dcc_bit_to_pulse(false))?;
        }
    }

    // Packet end bit: "1"
    push_pulse(&mut pulses, dcc_bit_to_pulse(true))?;

    Ok(pulses)
}

/// Push a pulse into the fixed-capacity pulse buffer.
fn push_pulse(
    pulses: &mut Vec<PulseCode, DCC_MAX_PACKET_PULSES>,
    pulse: PulseCode,
) -> Result<(), EncodeError> {
    pulses
        .push(pulse)
        .map_err(|_| EncodeError::PulseBufferOverflow)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dcc::packet::*;

    #[test]
    fn test_dcc_bit_encoding() {
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
    fn test_byte_encoding() {
        let pulses = encode_byte(0b10101010);

        // Should be 8 pulses: 1,0,1,0,1,0,1,0
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

        // Preamble (20) + start (1) + byte1 (8) + start (1) + byte2 (8) + start (1) + byte3 (8) + end (1) = 48
        assert_eq!(pulses.len(), 48);

        // First 20 should be preamble "1"s
        for i in 0..PREAMBLE_BITS {
            assert_eq!(pulses[i].length1, 58);
        }

        // First bit after preamble should be start bit "0"
        assert_eq!(pulses[PREAMBLE_BITS].length1, 100);
    }

    #[test]
    fn test_speed_packet_structure() {
        let addr = DccAddress::short(3).unwrap();
        let packet = DccPacket::speed_28step(addr, 10, Direction::Forward).unwrap();
        let pulses = encode_dcc_packet(&packet).unwrap();

        // Verify it's a valid length packet
        assert!(pulses.len() > 14); // At least preamble + some data

        // Preamble should be all "1"s (58μs)
        for i in 0..PREAMBLE_BITS {
            assert_eq!(pulses[i].length1, DCC_ONE_HIGH_US);
        }
    }
}
