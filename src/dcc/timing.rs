//! DCC timing constants per NMRA S-9.1
//!
//! All values in microseconds (μs)

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

/// RMT clock frequency (1 MHz = 1μs resolution)
pub const RMT_CLOCK_HZ: u32 = 1_000_000;

/// Maximum number of DCC data pulses encodable in one RMT TX submission (single-shot).
///
/// Used for real-packet (non-idle) single-shot blocking transmissions.
/// 128 comfortably fits 5–6 byte DCC packets (preamble + data + checksum) plus end marker.
pub const DCC_MAX_PACKET_PULSES: usize = 128;

/// RMT buffer capacity for the pre-built idle packet used by the hardware continuous loop.
///
/// Idle packet: PREAMBLE_BITS(20) + 1 start + 3×(8 data + 1 sep) + 1 end-bit = 48 DCC pulses
/// + 1 RMT end marker = 49 entries total.
///
/// The engine uses `transmit_continuously(LoopMode::Infinite)` on a two-block RMT channel
/// (96 slots) so the hardware loops the idle waveform with zero inter-iteration gap.
/// This eliminates the ~64μs Embassy interrupt-to-task latency gap that would otherwise
/// extend the last LOW half-period and create an invalid ~122μs DCC bit at every boundary.
pub const IDLE_RMT_SIZE: usize = 49; // 48 DCC pulses + 1 end marker
