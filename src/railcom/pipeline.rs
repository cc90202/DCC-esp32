use core::sync::atomic::{AtomicU32, Ordering};

use heapless::Vec;

use crate::railcom::parser::{
    Channel2ParseStatus, ParseError, RailcomChannel2Item, parse_channel2,
};

static RX_WINDOW_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_EMPTY_WINDOW_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_PARSE_OK_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_PARSE_ERR_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_ACK_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_NACK_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_UNSUPPORTED_CHANNEL_COUNT: AtomicU32 = AtomicU32::new(0);

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
/// In the current CH2-first phase we store at most 6 raw bytes because the
/// largest supported CH2 datagram spans 6 encoded symbols.
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
    UnsupportedChannel,
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomRxResult {
    pub window: RailcomRxWindow,
    pub outcome: RailcomRxOutcome,
    pub items: Vec<RailcomChannel2Item, 6>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomRxStats {
    pub rx_window_count: u32,
    pub rx_empty_window_count: u32,
    pub rx_parse_ok_count: u32,
    pub rx_parse_err_count: u32,
    pub rx_ack_count: u32,
    pub rx_nack_count: u32,
    pub rx_unsupported_channel_count: u32,
}

#[must_use]
pub fn railcom_rx_stats() -> RailcomRxStats {
    RailcomRxStats {
        rx_window_count: RX_WINDOW_COUNT.load(Ordering::Acquire),
        rx_empty_window_count: RX_EMPTY_WINDOW_COUNT.load(Ordering::Acquire),
        rx_parse_ok_count: RX_PARSE_OK_COUNT.load(Ordering::Acquire),
        rx_parse_err_count: RX_PARSE_ERR_COUNT.load(Ordering::Acquire),
        rx_ack_count: RX_ACK_COUNT.load(Ordering::Acquire),
        rx_nack_count: RX_NACK_COUNT.load(Ordering::Acquire),
        rx_unsupported_channel_count: RX_UNSUPPORTED_CHANNEL_COUNT.load(Ordering::Acquire),
    }
}

#[cfg(test)]
pub fn reset_railcom_rx_stats() {
    RX_WINDOW_COUNT.store(0, Ordering::Release);
    RX_EMPTY_WINDOW_COUNT.store(0, Ordering::Release);
    RX_PARSE_OK_COUNT.store(0, Ordering::Release);
    RX_PARSE_ERR_COUNT.store(0, Ordering::Release);
    RX_ACK_COUNT.store(0, Ordering::Release);
    RX_NACK_COUNT.store(0, Ordering::Release);
    RX_UNSUPPORTED_CHANNEL_COUNT.store(0, Ordering::Release);
}

#[must_use]
pub fn process_rx_window(window: RailcomRxWindow) -> RailcomRxResult {
    RX_WINDOW_COUNT.fetch_add(1, Ordering::Relaxed);

    if window.is_empty() {
        RX_EMPTY_WINDOW_COUNT.fetch_add(1, Ordering::Relaxed);
        return RailcomRxResult {
            window,
            outcome: RailcomRxOutcome::Empty,
            items: Vec::new(),
        };
    }

    match window.channel {
        RailcomChannel::Channel1 => {
            RX_UNSUPPORTED_CHANNEL_COUNT.fetch_add(1, Ordering::Relaxed);
            RailcomRxResult {
                window,
                outcome: RailcomRxOutcome::UnsupportedChannel,
                items: Vec::new(),
            }
        }
        RailcomChannel::Channel2 => match parse_channel2(window.raw_slice()) {
            Ok(parsed) => {
                RX_PARSE_OK_COUNT.fetch_add(1, Ordering::Relaxed);
                for item in parsed.items.iter() {
                    match item {
                        RailcomChannel2Item::Ack => {
                            RX_ACK_COUNT.fetch_add(1, Ordering::Relaxed);
                        }
                        RailcomChannel2Item::Nack => {
                            RX_NACK_COUNT.fetch_add(1, Ordering::Relaxed);
                        }
                        RailcomChannel2Item::Datagram(_) => {}
                    }
                }

                RailcomRxResult {
                    window,
                    outcome: match parsed.status {
                        Channel2ParseStatus::Complete => RailcomRxOutcome::Parsed,
                        Channel2ParseStatus::PartialUnsupportedDatagram(id) => {
                            RailcomRxOutcome::PartialUnsupportedDatagram(id)
                        }
                    },
                    items: parsed.items,
                }
            }
            Err(err) => {
                RX_PARSE_ERR_COUNT.fetch_add(1, Ordering::Relaxed);
                RailcomRxResult {
                    window,
                    outcome: RailcomRxOutcome::ParseError(err),
                    items: Vec::new(),
                }
            }
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::railcom::parser::{ACK_1_CODE, RailcomDatagram};

    const TEST_ID0_CODE_0X42: [u8; 2] = [0b1010_1010, 0b1010_1001];

    #[test]
    fn test_process_rx_window_empty_window() {
        reset_railcom_rx_stats();
        let window = RailcomRxWindow::try_new(7, RailcomChannel::Channel2, &[])
            .expect("empty window must fit");
        let result = process_rx_window(window);

        assert_eq!(result.outcome, RailcomRxOutcome::Empty);
        assert!(result.items.is_empty());
        assert_eq!(
            railcom_rx_stats(),
            RailcomRxStats {
                rx_window_count: 1,
                rx_empty_window_count: 1,
                rx_parse_ok_count: 0,
                rx_parse_err_count: 0,
                rx_ack_count: 0,
                rx_nack_count: 0,
                rx_unsupported_channel_count: 0,
            }
        );
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
                RailcomChannel2Item::Ack,
                RailcomChannel2Item::Datagram(RailcomDatagram::CvData(0x42)),
            ]
        );
        assert_eq!(
            railcom_rx_stats(),
            RailcomRxStats {
                rx_window_count: 1,
                rx_empty_window_count: 0,
                rx_parse_ok_count: 1,
                rx_parse_err_count: 0,
                rx_ack_count: 1,
                rx_nack_count: 0,
                rx_unsupported_channel_count: 0,
            }
        );
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
        assert_eq!(result.items.as_slice(), &[RailcomChannel2Item::Ack]);
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
    fn test_process_rx_window_channel1_is_explicitly_unsupported_for_now() {
        reset_railcom_rx_stats();
        let window = RailcomRxWindow::try_new(19, RailcomChannel::Channel1, &[ACK_1_CODE])
            .expect("single-byte CH1 window must fit");
        let result = process_rx_window(window);

        assert_eq!(result.outcome, RailcomRxOutcome::UnsupportedChannel);
        assert!(result.items.is_empty());
        let stats = railcom_rx_stats();
        assert_eq!(stats.rx_parse_ok_count, 0);
        assert_eq!(stats.rx_parse_err_count, 0);
        assert_eq!(stats.rx_unsupported_channel_count, 1);
    }
}
