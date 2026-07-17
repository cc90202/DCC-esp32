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

/// Duration of the fixed packet preamble in microseconds.
#[cfg(any(test, target_arch = "riscv32"))]
pub const PREAMBLE_DURATION_US: u32 =
    PREAMBLE_BITS as u32 * (DCC_ONE_HIGH_US as u32 + DCC_ONE_LOW_US as u32);

/// RMT clock frequency (1 MHz = 1μs resolution)
#[cfg(target_arch = "riscv32")]
pub const RMT_CLOCK_HZ: u32 = 1_000_000;

/// Maximum number of DCC data pulses encodable in one RMT TX submission.
///
/// 128 comfortably fits 10-byte DCC packets (preamble + data + checksum).
/// The actual RMT buffer includes +1 for the end marker.
/// Must not exceed RMT RAM capacity (memsize: 3 = 144 slots) since
/// `transmit_continuously` requires all data in RAM at once.
pub const DCC_MAX_PACKET_PULSES: usize = 128;

/// Number of fixed preamble entries written at the start of the RMT RAM window.
#[cfg(target_arch = "riscv32")]
pub const PREAMBLE_RMT_OFFSET: usize = PREAMBLE_BITS;

/// Maximum number of RMT entries in the variable packet tail (start bit, data,
/// separators, end bit). The ISR appends the end marker itself.
///
/// RCN-218 LOGON_SELECT is 10 bytes:
/// 1 start + 10*8 data bits + 9 separators + 1 end = 91 entries.
#[cfg(any(test, target_arch = "riscv32"))]
pub const MAX_DATA_PULSES: usize = 96;

/// RMT buffer capacity for the pre-built idle packet.
///
/// Idle packet: PREAMBLE_BITS(20) + 1 start + 3×(8 data + 1 sep) + 1 end-bit = 48 DCC pulses
/// + 1 RMT end marker = 49 entries total.
///
/// The engine uses this pre-encoded buffer to avoid re-encoding DccPacket::Idle
/// on every loop iteration.  All packets (idle and real) are transmitted via
/// `transmit_continuously(LoopMode::Infinite)` - the hardware loops the buffer
/// with zero inter-packet gap.  RMT channel memsize must be ≥ 2 blocks (96 slots)
/// to fit idle (49 entries). Three blocks leave enough tail RAM for RCN-218
/// LOGON_SELECT packets.
#[cfg(target_arch = "riscv32")]
pub const IDLE_RMT_SIZE: usize = 49; // 48 DCC pulses + 1 end marker
