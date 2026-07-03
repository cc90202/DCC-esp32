use core::sync::atomic::{AtomicU32, Ordering};

use heapless::Vec;

use crate::railcom::parser::{
    ParseError, RailcomDatagram, RailcomItem, RailcomParseResult, RailcomParseStatus,
    parse_channel1, parse_channel2,
};

// Per-outcome counters. Every window lands in exactly one bucket:
// empty, parsed, parse-error, or oversized (FIFO snapshot exceeded the max
// captured byte count — this is an overflow condition, not detected UART
// framing/glitch corruption).
static RX_EMPTY_WINDOW_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_PARSE_OK_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_PARSE_ERR_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_OVERSIZED_WINDOW_COUNT: AtomicU32 = AtomicU32::new(0);
// Within-parsed counters (not mutually exclusive; derived from items).
static RX_ACK_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_NACK_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_ADR_HIGH_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_ADR_LOW_COUNT: AtomicU32 = AtomicU32::new(0);
// Capture-level overflow counter (dropped ring entries or windows too long).
static RX_OVERFLOW_COUNT: AtomicU32 = AtomicU32::new(0);

/// Free-running, wraparound-safe packet-boundary counter shared across the
/// RailCom pipeline (POM dispatch attribution, loco identification, ring
/// drains).
///
/// This counter is a `u32` that overflows during normal long-running
/// operation, so "is this newer/older/within N packets" comparisons must use
/// wrapping arithmetic rather than plain `<`/`>`/subtraction. Centralising
/// that arithmetic here avoids re-deriving the same `wrapping_sub` +
/// threshold comparison at each call site.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketSequence(u32);

impl PacketSequence {
    #[must_use]
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    #[must_use]
    pub const fn value(self) -> u32 {
        self.0
    }

    /// Number of packet boundaries between `earlier` and `self`, correct
    /// across `u32` wraparound (i.e. `self` is assumed to be at or after
    /// `earlier` on the wrapping counter).
    #[must_use]
    pub const fn age_since(self, earlier: PacketSequence) -> u32 {
        self.0.wrapping_sub(earlier.0)
    }

    /// True when `self` is within `window` packet boundaries of `earlier`
    /// (inclusive), accounting for wraparound.
    #[must_use]
    pub const fn is_within(self, earlier: PacketSequence, window: u32) -> bool {
        self.age_since(earlier) <= window
    }
}

impl From<u32> for PacketSequence {
    fn from(value: u32) -> Self {
        Self::new(value)
    }
}

impl From<PacketSequence> for u32 {
    fn from(sequence: PacketSequence) -> Self {
        sequence.value()
    }
}

const RAILCOM_CHANNEL_COUNT: usize = 2;
static RX_CHANNEL_WINDOW_COUNTS: [AtomicU32; RAILCOM_CHANNEL_COUNT] =
    [const { AtomicU32::new(0) }; RAILCOM_CHANNEL_COUNT];
static RX_CHANNEL_EMPTY_COUNTS: [AtomicU32; RAILCOM_CHANNEL_COUNT] =
    [const { AtomicU32::new(0) }; RAILCOM_CHANNEL_COUNT];

const MAX_CHANNEL2_WINDOW_BYTES: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomChannel {
    Channel1,
    Channel2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomRxWindowError {
    WindowTooLong { provided_len: usize, max_len: usize },
}

/// One logical RailCom receive window.
///
/// At most 6 raw bytes: the largest supported CH2 datagram spans 6 encoded
/// 4/8 symbols.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomRxWindow {
    pub packet_sequence: u32,
    pub channel: RailcomChannel,
    pub raw_len: u8,
    pub raw_bytes: [u8; 6],
}

impl RailcomRxWindow {
    pub fn try_new(
        packet_sequence: u32,
        channel: RailcomChannel,
        raw_bytes: &[u8],
    ) -> Result<Self, RailcomRxWindowError> {
        if raw_bytes.len() > MAX_CHANNEL2_WINDOW_BYTES {
            return Err(RailcomRxWindowError::WindowTooLong {
                provided_len: raw_bytes.len(),
                max_len: MAX_CHANNEL2_WINDOW_BYTES,
            });
        }

        let mut stored = [0u8; MAX_CHANNEL2_WINDOW_BYTES];
        let len = raw_bytes.len();
        stored[..len].copy_from_slice(raw_bytes);

        Ok(Self {
            packet_sequence,
            channel,
            raw_len: len as u8,
            raw_bytes: stored,
        })
    }

    #[must_use]
    pub fn raw_slice(&self) -> &[u8] {
        &self.raw_bytes[..self.raw_len as usize]
    }

    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.raw_len == 0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomRxOutcome {
    Empty,
    Parsed,
    ParseError(ParseError),
    PartialUnsupportedDatagram(u8),
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomRxResult {
    pub window: RailcomRxWindow,
    pub outcome: RailcomRxOutcome,
    pub items: Vec<RailcomItem, 6>,
}

/// Snapshot of RX pipeline counters.
///
/// The persisted atomics cover only orthogonal outcomes; `rx_window_count`
/// and `rx_windows_with_bytes_count` are derived at snapshot time so the
/// totals stay exactly consistent with their parts regardless of ISR racing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomRxStats {
    pub rx_empty_window_count: u32,
    pub rx_parse_ok_count: u32,
    pub rx_parse_err_count: u32,
    pub rx_oversized_window_count: u32,
    pub rx_ack_count: u32,
    pub rx_nack_count: u32,
    pub rx_adr_high_count: u32,
    pub rx_adr_low_count: u32,
    pub rx_overflow_count: u32,
    pub ch1_window_count: u32,
    pub ch1_empty_count: u32,
    pub ch2_window_count: u32,
    pub ch2_empty_count: u32,
}

impl RailcomRxStats {
    #[must_use]
    pub fn rx_windows_with_bytes_count(&self) -> u32 {
        self.rx_parse_ok_count + self.rx_parse_err_count + self.rx_oversized_window_count
    }

    #[must_use]
    pub fn rx_window_count(&self) -> u32 {
        self.rx_empty_window_count + self.rx_windows_with_bytes_count()
    }
}

#[must_use]
pub fn railcom_rx_stats() -> RailcomRxStats {
    RailcomRxStats {
        rx_empty_window_count: RX_EMPTY_WINDOW_COUNT.load(Ordering::Acquire),
        rx_parse_ok_count: RX_PARSE_OK_COUNT.load(Ordering::Acquire),
        rx_parse_err_count: RX_PARSE_ERR_COUNT.load(Ordering::Acquire),
        rx_oversized_window_count: RX_OVERSIZED_WINDOW_COUNT.load(Ordering::Acquire),
        rx_ack_count: RX_ACK_COUNT.load(Ordering::Acquire),
        rx_nack_count: RX_NACK_COUNT.load(Ordering::Acquire),
        rx_adr_high_count: RX_ADR_HIGH_COUNT.load(Ordering::Acquire),
        rx_adr_low_count: RX_ADR_LOW_COUNT.load(Ordering::Acquire),
        rx_overflow_count: RX_OVERFLOW_COUNT.load(Ordering::Acquire),
        ch1_window_count: RX_CHANNEL_WINDOW_COUNTS[RailcomChannel::Channel1.index()]
            .load(Ordering::Acquire),
        ch1_empty_count: RX_CHANNEL_EMPTY_COUNTS[RailcomChannel::Channel1.index()]
            .load(Ordering::Acquire),
        ch2_window_count: RX_CHANNEL_WINDOW_COUNTS[RailcomChannel::Channel2.index()]
            .load(Ordering::Acquire),
        ch2_empty_count: RX_CHANNEL_EMPTY_COUNTS[RailcomChannel::Channel2.index()]
            .load(Ordering::Acquire),
    }
}

#[cfg(test)]
pub fn reset_railcom_rx_stats() {
    RX_EMPTY_WINDOW_COUNT.store(0, Ordering::Release);
    RX_PARSE_OK_COUNT.store(0, Ordering::Release);
    RX_PARSE_ERR_COUNT.store(0, Ordering::Release);
    RX_OVERSIZED_WINDOW_COUNT.store(0, Ordering::Release);
    RX_ACK_COUNT.store(0, Ordering::Release);
    RX_NACK_COUNT.store(0, Ordering::Release);
    RX_ADR_HIGH_COUNT.store(0, Ordering::Release);
    RX_ADR_LOW_COUNT.store(0, Ordering::Release);
    RX_OVERFLOW_COUNT.store(0, Ordering::Release);
    for index in 0..RAILCOM_CHANNEL_COUNT {
        RX_CHANNEL_WINDOW_COUNTS[index].store(0, Ordering::Release);
        RX_CHANNEL_EMPTY_COUNTS[index].store(0, Ordering::Release);
    }
}

pub fn record_oversized_window() {
    RX_OVERSIZED_WINDOW_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn record_rx_overflows(count: u32) {
    RX_OVERFLOW_COUNT.fetch_add(count, Ordering::Relaxed);
}

impl RailcomChannel {
    const fn index(self) -> usize {
        match self {
            RailcomChannel::Channel1 => 0,
            RailcomChannel::Channel2 => 1,
        }
    }
}

/// Wire/ISR-facing numeric id for a RailCom channel: 1 = CH1, 2 = CH2.
///
/// `0` is reserved as an "unset" sentinel by ISR-side code and is therefore
/// never produced here; see `TryFrom<u8> for RailcomChannel`.
impl From<RailcomChannel> for u8 {
    /// Called from ISR-resident code (`track_output`, `isr_capture`); the
    /// inline guarantee keeps the conversion out of flash on non-LTO builds.
    #[inline(always)]
    fn from(channel: RailcomChannel) -> Self {
        match channel {
            RailcomChannel::Channel1 => 1,
            RailcomChannel::Channel2 => 2,
        }
    }
}

impl TryFrom<u8> for RailcomChannel {
    type Error = ();

    #[inline]
    fn try_from(id: u8) -> Result<Self, Self::Error> {
        match id {
            1 => Ok(RailcomChannel::Channel1),
            2 => Ok(RailcomChannel::Channel2),
            _ => Err(()),
        }
    }
}

fn record_channel_window(channel: RailcomChannel) {
    RX_CHANNEL_WINDOW_COUNTS[channel.index()].fetch_add(1, Ordering::Relaxed);
}

fn record_channel_empty(channel: RailcomChannel) {
    RX_CHANNEL_EMPTY_COUNTS[channel.index()].fetch_add(1, Ordering::Relaxed);
}

fn parsed_result(window: RailcomRxWindow, parsed: RailcomParseResult) -> RailcomRxResult {
    RX_PARSE_OK_COUNT.fetch_add(1, Ordering::Relaxed);
    record_parsed_items(parsed.items.as_slice());

    RailcomRxResult {
        window,
        outcome: match parsed.status {
            RailcomParseStatus::Complete => RailcomRxOutcome::Parsed,
            RailcomParseStatus::PartialUnsupportedDatagram(id) => {
                RailcomRxOutcome::PartialUnsupportedDatagram(id)
            }
        },
        items: parsed.items,
    }
}

#[must_use]
pub fn process_rx_window(window: RailcomRxWindow) -> RailcomRxResult {
    record_channel_window(window.channel);
    if window.is_empty() {
        RX_EMPTY_WINDOW_COUNT.fetch_add(1, Ordering::Relaxed);
        record_channel_empty(window.channel);
        return RailcomRxResult {
            window,
            outcome: RailcomRxOutcome::Empty,
            items: Vec::new(),
        };
    }

    let parse_fn = match window.channel {
        RailcomChannel::Channel1 => parse_channel1,
        RailcomChannel::Channel2 => parse_channel2,
    };

    match parse_fn(window.raw_slice()) {
        Ok(parsed) => parsed_result(window, parsed),
        Err(err) => {
            RX_PARSE_ERR_COUNT.fetch_add(1, Ordering::Relaxed);
            RailcomRxResult {
                window,
                outcome: RailcomRxOutcome::ParseError(err),
                items: Vec::new(),
            }
        }
    }
}

fn record_parsed_items(items: &[RailcomItem]) {
    for item in items {
        match item {
            RailcomItem::Ack => {
                RX_ACK_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            RailcomItem::Nack => {
                RX_NACK_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            RailcomItem::Datagram(RailcomDatagram::AdrHigh(_)) => {
                RX_ADR_HIGH_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            RailcomItem::Datagram(RailcomDatagram::AdrLow(_)) => {
                RX_ADR_LOW_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            RailcomItem::Datagram(_) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::railcom::parser::{ACK_1_CODE, ACK_2_CODE, RailcomDatagram};

    const TEST_ID0_CODE_0X42: [u8; 2] = [0b1010_1010, 0b1010_1001];

    #[test]
    fn test_process_rx_window_empty_window() {
        reset_railcom_rx_stats();
        let window = RailcomRxWindow::try_new(7, RailcomChannel::Channel2, &[])
            .expect("empty window must fit");
        let result = process_rx_window(window);

        assert_eq!(result.outcome, RailcomRxOutcome::Empty);
        assert!(result.items.is_empty());
        let stats = railcom_rx_stats();
        assert_eq!(stats.rx_window_count(), 1);
        assert_eq!(stats.rx_empty_window_count, 1);
        assert_eq!(stats.rx_windows_with_bytes_count(), 0);
    }

    #[test]
    fn test_process_rx_window_channel2_parsed() {
        reset_railcom_rx_stats();
        let raw = [ACK_1_CODE, TEST_ID0_CODE_0X42[0], TEST_ID0_CODE_0X42[1]];
        let window = RailcomRxWindow::try_new(11, RailcomChannel::Channel2, &raw)
            .expect("raw window must fit");
        let result = process_rx_window(window);

        assert_eq!(result.outcome, RailcomRxOutcome::Parsed);
        assert_eq!(
            result.items.as_slice(),
            &[
                RailcomItem::Ack,
                RailcomItem::Datagram(RailcomDatagram::CvData(0x42)),
            ]
        );
        let stats = railcom_rx_stats();
        assert_eq!(stats.rx_window_count(), 1);
        assert_eq!(stats.rx_windows_with_bytes_count(), 1);
        assert_eq!(stats.rx_parse_ok_count, 1);
        assert_eq!(stats.rx_ack_count, 1);
    }

    #[test]
    fn test_process_rx_window_partial_unsupported_datagram_keeps_prefix() {
        reset_railcom_rx_stats();
        let raw = [ACK_1_CODE, 0b1011_0010, 0b1010_1100];
        let window = RailcomRxWindow::try_new(21, RailcomChannel::Channel2, &raw)
            .expect("raw window must fit");
        let result = process_rx_window(window);

        assert_eq!(
            result.outcome,
            RailcomRxOutcome::PartialUnsupportedDatagram(4)
        );
        assert_eq!(result.items.as_slice(), &[RailcomItem::Ack]);
        assert_eq!(railcom_rx_stats().rx_parse_ok_count, 1);
        assert_eq!(railcom_rx_stats().rx_parse_err_count, 0);
    }

    #[test]
    fn test_process_rx_window_channel2_parse_error() {
        reset_railcom_rx_stats();
        let window = RailcomRxWindow::try_new(15, RailcomChannel::Channel2, &[0xff])
            .expect("single-byte window must fit");
        let result = process_rx_window(window);

        assert_eq!(
            result.outcome,
            RailcomRxOutcome::ParseError(ParseError::Invalid4Of8Code(0xff))
        );
        assert!(result.items.is_empty());
        assert_eq!(railcom_rx_stats().rx_parse_err_count, 1);
    }

    #[test]
    fn test_try_new_rejects_too_long_window() {
        let err = RailcomRxWindow::try_new(23, RailcomChannel::Channel2, &[0, 1, 2, 3, 4, 5, 6])
            .expect_err("window longer than 6 bytes must be rejected");

        assert_eq!(
            err,
            RailcomRxWindowError::WindowTooLong {
                provided_len: 7,
                max_len: 6,
            }
        );
    }

    #[test]
    fn test_process_rx_window_channel1_parses_address_datagram() {
        reset_railcom_rx_stats();
        let raw = [0b1001_1001, 0b1100_1001];
        let window = RailcomRxWindow::try_new(19, RailcomChannel::Channel1, &raw)
            .expect("CH1 address window must fit");
        let result = process_rx_window(window);

        assert_eq!(result.outcome, RailcomRxOutcome::Parsed);
        assert_eq!(
            result.items.as_slice(),
            &[RailcomItem::Datagram(RailcomDatagram::AdrLow(42))]
        );
        let stats = railcom_rx_stats();
        assert_eq!(stats.rx_windows_with_bytes_count(), 1);
        assert_eq!(stats.rx_parse_ok_count, 1);
        assert_eq!(stats.rx_parse_err_count, 0);
        assert_eq!(stats.ch1_window_count, 1);
        assert_eq!(stats.rx_adr_low_count, 1);
    }

    #[test]
    fn test_process_rx_window_channel1_accepts_ack() {
        reset_railcom_rx_stats();
        let window = RailcomRxWindow::try_new(19, RailcomChannel::Channel1, &[ACK_2_CODE])
            .expect("single-byte CH1 window must fit");
        let result = process_rx_window(window);

        assert_eq!(result.outcome, RailcomRxOutcome::Parsed);
        assert_eq!(result.items.as_slice(), &[RailcomItem::Ack]);
        let stats = railcom_rx_stats();
        assert_eq!(stats.rx_parse_ok_count, 1);
        assert_eq!(stats.rx_parse_err_count, 0);
        assert_eq!(stats.ch1_window_count, 1);
        assert_eq!(stats.rx_ack_count, 1);
    }

    #[test]
    fn test_packet_sequence_age_since_wraps() {
        let earlier = PacketSequence::new(u32::MAX - 1);
        let later = PacketSequence::new(1);

        assert_eq!(later.age_since(earlier), 3);
    }

    #[test]
    fn test_packet_sequence_is_within_inclusive_window() {
        let earlier = PacketSequence::new(10);

        assert!(PacketSequence::new(18).is_within(earlier, 8));
        assert!(!PacketSequence::new(19).is_within(earlier, 8));
    }

    #[test]
    fn test_record_oversized_window_updates_aggregate_stats() {
        reset_railcom_rx_stats();
        record_oversized_window();

        let stats = railcom_rx_stats();
        assert_eq!(stats.rx_window_count(), 1);
        assert_eq!(stats.rx_windows_with_bytes_count(), 1);
        assert_eq!(stats.rx_oversized_window_count, 1);
    }
}
