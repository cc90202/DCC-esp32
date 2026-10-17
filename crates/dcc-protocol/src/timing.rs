//! DCC bit timings per NMRA S-9.1, and the pulse capacities that follow from
//! the longest packet.
//!
//! All durations are in microseconds (μs). One pulse is one DCC bit: a high
//! half followed by a low half.

use crate::packet::DCC_MAX_PACKET_BYTES;

/// DCC "1" bit: high duration (58μs nominal, 55-61μs acceptable)
pub const DCC_ONE_HIGH_US: u16 = 58;

/// DCC "1" bit: low duration (58μs nominal, 55-61μs acceptable)
pub const DCC_ONE_LOW_US: u16 = 58;

/// DCC "0" bit: high duration (100μs nominal, 95-9900μs acceptable for stretching)
pub const DCC_ZERO_HIGH_US: u16 = 100;

/// DCC "0" bit: low duration (100μs nominal, 95-9900μs acceptable for stretching)
pub const DCC_ZERO_LOW_US: u16 = 100;

/// Preamble bits transmitted before each packet.
/// NMRA S-9.1 requires 14+ but modern decoders (especially ESU LokSound)
/// synchronise more reliably with a longer preamble. DCC-EX uses 20-22.
pub const PREAMBLE_BITS: usize = 20;

/// Duration of the fixed packet preamble in microseconds.
pub const PREAMBLE_DURATION_US: u32 =
    PREAMBLE_BITS as u32 * (DCC_ONE_HIGH_US as u32 + DCC_ONE_LOW_US as u32);

/// Pulses in the tail of the longest packet: the start bit, eight data bits
/// per byte, a separator bit between consecutive bytes, and the end bit.
const LONGEST_TAIL_PULSES: usize = 1 + DCC_MAX_PACKET_BYTES * 8 + (DCC_MAX_PACKET_BYTES - 1) + 1;

/// Pulses of the longest packet with its preamble.
const LONGEST_PACKET_PULSES: usize = PREAMBLE_BITS + LONGEST_TAIL_PULSES;

/// Capacity of the pulse buffer that holds a whole packet, preamble included.
///
/// At least the pulses of the longest packet ([`DCC_MAX_PACKET_BYTES`] bytes
/// after [`PREAMBLE_BITS`] preamble bits); the margin above that is spare.
pub const DCC_MAX_PACKET_PULSES: usize = 128;

/// Capacity of the pulse buffer that holds the variable packet tail alone:
/// the start bit, the data bytes with their separators, and the end bit.
///
/// At least the tail of the longest packet ([`DCC_MAX_PACKET_BYTES`] bytes);
/// the margin above that is spare.
pub const MAX_DATA_PULSES: usize = 96;

const _: () = assert!(
    DCC_MAX_PACKET_PULSES >= LONGEST_PACKET_PULSES,
    "the whole-packet pulse buffer must hold the longest packet"
);
const _: () = assert!(
    MAX_DATA_PULSES >= LONGEST_TAIL_PULSES,
    "the packet-tail pulse buffer must hold the tail of the longest packet"
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_longest_packet_minimums_count_every_bit() {
        // Each byte is 8 data bits plus the separator or start bit before
        // it, and the end bit closes the packet.
        assert_eq!(LONGEST_TAIL_PULSES, 9 * DCC_MAX_PACKET_BYTES + 1);
        assert_eq!(
            LONGEST_PACKET_PULSES,
            PREAMBLE_BITS + 9 * DCC_MAX_PACKET_BYTES + 1
        );
    }
}
