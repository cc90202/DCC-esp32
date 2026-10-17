//! DCC timing constants per NMRA S-9.1, plus the RMT buffer sizes of this
//! firmware.
//!
//! The protocol timings live in `dcc_protocol::timing` and are re-exported here
//! so existing `crate::dcc::timing::` paths keep resolving. The constants below
//! describe the ESP32-C6 RMT peripheral and stay with the firmware.

pub use dcc_protocol::timing::*;

/// RMT clock frequency (1 MHz = 1μs resolution)
#[cfg(target_arch = "riscv32")]
pub const RMT_CLOCK_HZ: u32 = 1_000_000;

/// Number of fixed preamble entries written at the start of the RMT RAM window.
#[cfg(target_arch = "riscv32")]
pub const PREAMBLE_RMT_OFFSET: usize = PREAMBLE_BITS;

/// RMT buffer capacity for the pre-built idle packet.
///
/// Idle packet: PREAMBLE_BITS(20) + 1 start + 3×(8 data + 1 sep) + 1 end-bit = 48 DCC pulses
/// + 1 RMT end marker = 49 entries total.
///
/// The engine uses this pre-encoded buffer to avoid re-encoding DccPacket::Idle
/// on every loop iteration.  All packets (idle and real) are transmitted via
/// `transmit_continuously(LoopMode::Infinite)`, so the hardware loops the buffer
/// with zero inter-packet gap.  RMT channel memsize must be ≥ 2 blocks (96 slots)
/// to fit idle (49 entries). Three blocks leave enough tail RAM for RCN-218
/// LOGON_SELECT packets.
#[cfg(target_arch = "riscv32")]
pub const IDLE_RMT_SIZE: usize = 49; // 48 DCC pulses + 1 end marker
