//! DCC timing constants per NMRA S-9.1, plus the RMT buffer sizes of this
//! firmware.
//!
//! The protocol timings and pulse capacities live in `dcc_protocol::timing`.
//! The constants here describe the ESP32-C6 RMT peripheral and stay with the
//! firmware.
//!
//! The RMT channel RAM must hold the preamble ([`PREAMBLE_RMT_OFFSET`]
//! entries), a packet tail of up to
//! [`MAX_DATA_PULSES`](dcc_protocol::timing::MAX_DATA_PULSES) pulses and the
//! end marker, because `transmit_continuously` needs all of it in RAM at once.
//! `rmt_driver.rs` checks that at compile time against
//! the channel RAM it configures.

use dcc_protocol::timing::PREAMBLE_BITS;

/// RMT clock frequency (1 MHz = 1μs resolution)
pub(crate) const RMT_CLOCK_HZ: u32 = 1_000_000;

/// Number of fixed preamble entries written at the start of the RMT RAM window.
pub(crate) const PREAMBLE_RMT_OFFSET: usize = PREAMBLE_BITS;

/// RMT buffer capacity for the pre-built idle packet.
///
/// The idle packet is [`PREAMBLE_BITS`] preamble pulses, then a start bit and
/// 8 data bits for each of its 3 bytes, then the end bit; the RMT end marker
/// adds one entry.
///
/// The engine uses this pre-encoded buffer to avoid re-encoding DccPacket::Idle
/// on every loop iteration.  All packets (idle and real) are transmitted via
/// `transmit_continuously(LoopMode::Infinite)`, so the hardware loops the buffer
/// with zero inter-packet gap.  RMT channel memsize must be ≥ 2 blocks (96 slots)
/// to fit the idle buffer. Three blocks leave enough tail RAM for RCN-218
/// LOGON_SELECT packets.
pub(crate) const IDLE_RMT_SIZE: usize = 49;

const _: () = assert!(
    IDLE_RMT_SIZE == PREAMBLE_BITS + 3 * (1 + 8) + 1 + 1,
    "IDLE_RMT_SIZE must hold the idle packet and the RMT end marker"
);
