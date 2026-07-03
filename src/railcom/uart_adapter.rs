use crate::railcom::pipeline::{
    RailcomChannel, RailcomRxResult, RailcomRxWindowError, process_rx_window,
};

#[cfg(target_arch = "riscv32")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(target_arch = "riscv32")]
use embassy_sync::channel::{Receiver, Sender};

const MAX_WINDOW_BYTES: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomUartEvent {
    BeginWindow {
        packet_sequence: u32,
        channel: RailcomChannel,
    },
    Byte(u8),
    EndWindow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomUartAdapterError {
    WindowAlreadyOpen {
        packet_sequence: u32,
        channel: RailcomChannel,
    },
    ByteOutsideWindow,
    EndWithoutWindow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomUartWindowError {
    WindowTooLong {
        packet_sequence: u32,
        channel: RailcomChannel,
        provided_len: usize,
        max_len: usize,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomUartAdapterOutput {
    WindowProcessed(RailcomRxResult),
    WindowError(RailcomUartWindowError),
    AdapterError(RailcomUartAdapterError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
struct PendingWindow {
    packet_sequence: u32,
    channel: RailcomChannel,
    raw_len: usize,
    raw_bytes: [u8; MAX_WINDOW_BYTES],
}

impl PendingWindow {
    const fn new(packet_sequence: u32, channel: RailcomChannel) -> Self {
        Self {
            packet_sequence,
            channel,
            raw_len: 0,
            raw_bytes: [0; MAX_WINDOW_BYTES],
        }
    }

    fn push_byte(&mut self, byte: u8) {
        if self.raw_len < MAX_WINDOW_BYTES {
            self.raw_bytes[self.raw_len] = byte;
        }
        self.raw_len += 1;
    }

    fn raw_slice(&self) -> &[u8] {
        &self.raw_bytes[..self.raw_len.min(MAX_WINDOW_BYTES)]
    }
}

#[derive(Debug, Default)]
pub struct RailcomUartAdapter {
    pending: Option<PendingWindow>,
}

impl RailcomUartAdapter {
    #[must_use]
    pub const fn new() -> Self {
        Self { pending: None }
    }

    pub fn handle_event(&mut self, event: RailcomUartEvent) -> Option<RailcomUartAdapterOutput> {
        match event {
            RailcomUartEvent::BeginWindow {
                packet_sequence,
                channel,
            } => {
                if let Some(active) = self.pending {
                    Some(RailcomUartAdapterOutput::AdapterError(
                        RailcomUartAdapterError::WindowAlreadyOpen {
                            packet_sequence: active.packet_sequence,
                            channel: active.channel,
                        },
                    ))
                } else {
                    self.pending = Some(PendingWindow::new(packet_sequence, channel));
                    None
                }
            }
            RailcomUartEvent::Byte(byte) => {
                let Some(pending) = self.pending.as_mut() else {
                    return Some(RailcomUartAdapterOutput::AdapterError(
                        RailcomUartAdapterError::ByteOutsideWindow,
                    ));
                };
                pending.push_byte(byte);
                None
            }
            RailcomUartEvent::EndWindow => {
                let Some(pending) = self.pending.take() else {
                    return Some(RailcomUartAdapterOutput::AdapterError(
                        RailcomUartAdapterError::EndWithoutWindow,
                    ));
                };

                if pending.raw_len > MAX_WINDOW_BYTES {
                    return Some(RailcomUartAdapterOutput::WindowError(
                        RailcomUartWindowError::WindowTooLong {
                            packet_sequence: pending.packet_sequence,
                            channel: pending.channel,
                            provided_len: pending.raw_len,
                            max_len: MAX_WINDOW_BYTES,
                        },
                    ));
                }

                match crate::railcom::pipeline::RailcomRxWindow::try_new(
                    pending.packet_sequence,
                    pending.channel,
                    pending.raw_slice(),
                ) {
                    Ok(window) => Some(RailcomUartAdapterOutput::WindowProcessed(
                        process_rx_window(window),
                    )),
                    Err(RailcomRxWindowError::WindowTooLong {
                        provided_len,
                        max_len,
                    }) => Some(RailcomUartAdapterOutput::WindowError(
                        RailcomUartWindowError::WindowTooLong {
                            packet_sequence: pending.packet_sequence,
                            channel: pending.channel,
                            provided_len,
                            max_len,
                        },
                    )),
                }
            }
        }
    }
}

#[cfg(target_arch = "riscv32")]
pub type RailcomUartEventChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, RailcomUartEvent, 16>;

#[cfg(target_arch = "riscv32")]
pub type RailcomUartResultChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, RailcomUartAdapterOutput, 8>;

/// Software-side UART adapter task.
///
/// This task is intentionally outside the DCC/RMT ISR path. A future UART ISR
/// or UART async reader should emit `BeginWindow/Byte/EndWindow` events into
/// `event_receiver`; this task assembles one logical RailCom window at a time
/// and forwards parsed results or adapter errors to normal runtime consumers.
#[cfg(target_arch = "riscv32")]
pub async fn railcom_uart_adapter_task(
    event_receiver: Receiver<'static, CriticalSectionRawMutex, RailcomUartEvent, 16>,
    result_sender: Sender<'static, CriticalSectionRawMutex, RailcomUartAdapterOutput, 8>,
) -> ! {
    let mut adapter = RailcomUartAdapter::new();

    loop {
        let event = event_receiver.receive().await;
        if let Some(output) = adapter.handle_event(event) {
            result_sender.send(output).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::railcom::parser::{ACK_1_CODE, RailcomChannel2Item, RailcomDatagram};
    use crate::railcom::pipeline::{RailcomRxOutcome, railcom_rx_stats, reset_railcom_rx_stats};

    const TEST_ID0_CODE_0X42: [u8; 2] = [0b1010_1010, 0b1010_1001];

    #[test]
    fn test_adapter_assembles_window_and_processes_it() {
        reset_railcom_rx_stats();
        let mut adapter = RailcomUartAdapter::new();

        assert_eq!(
            adapter.handle_event(RailcomUartEvent::BeginWindow {
                packet_sequence: 7,
                channel: RailcomChannel::Channel2,
            }),
            None
        );
        assert_eq!(
            adapter.handle_event(RailcomUartEvent::Byte(ACK_1_CODE)),
            None
        );
        assert_eq!(
            adapter.handle_event(RailcomUartEvent::Byte(TEST_ID0_CODE_0X42[0])),
            None
        );
        assert_eq!(
            adapter.handle_event(RailcomUartEvent::Byte(TEST_ID0_CODE_0X42[1])),
            None
        );

        let Some(RailcomUartAdapterOutput::WindowProcessed(result)) =
            adapter.handle_event(RailcomUartEvent::EndWindow)
        else {
            panic!("end window should produce a processed result");
        };

        assert_eq!(result.outcome, RailcomRxOutcome::Parsed);
        assert_eq!(
            result.items.as_slice(),
            &[
                RailcomChannel2Item::Ack,
                RailcomChannel2Item::Datagram(RailcomDatagram::CvData(0x42)),
            ]
        );
        assert_eq!(railcom_rx_stats().rx_window_count, 1);
    }

    #[test]
    fn test_adapter_reports_byte_outside_window() {
        let mut adapter = RailcomUartAdapter::new();

        assert_eq!(
            adapter.handle_event(RailcomUartEvent::Byte(0x12)),
            Some(RailcomUartAdapterOutput::AdapterError(
                RailcomUartAdapterError::ByteOutsideWindow,
            ))
        );
    }

    #[test]
    fn test_adapter_reports_end_without_window() {
        let mut adapter = RailcomUartAdapter::new();

        assert_eq!(
            adapter.handle_event(RailcomUartEvent::EndWindow),
            Some(RailcomUartAdapterOutput::AdapterError(
                RailcomUartAdapterError::EndWithoutWindow,
            ))
        );
    }

    #[test]
    fn test_adapter_reports_window_already_open() {
        let mut adapter = RailcomUartAdapter::new();

        assert_eq!(
            adapter.handle_event(RailcomUartEvent::BeginWindow {
                packet_sequence: 9,
                channel: RailcomChannel::Channel2,
            }),
            None
        );
        assert_eq!(
            adapter.handle_event(RailcomUartEvent::BeginWindow {
                packet_sequence: 10,
                channel: RailcomChannel::Channel1,
            }),
            Some(RailcomUartAdapterOutput::AdapterError(
                RailcomUartAdapterError::WindowAlreadyOpen {
                    packet_sequence: 9,
                    channel: RailcomChannel::Channel2,
                },
            ))
        );
    }

    #[test]
    fn test_adapter_reports_too_long_window_without_silent_truncation() {
        let mut adapter = RailcomUartAdapter::new();

        assert_eq!(
            adapter.handle_event(RailcomUartEvent::BeginWindow {
                packet_sequence: 12,
                channel: RailcomChannel::Channel2,
            }),
            None
        );
        for byte in [0, 1, 2, 3, 4, 5, 6] {
            assert_eq!(adapter.handle_event(RailcomUartEvent::Byte(byte)), None);
        }

        assert_eq!(
            adapter.handle_event(RailcomUartEvent::EndWindow),
            Some(RailcomUartAdapterOutput::WindowError(
                RailcomUartWindowError::WindowTooLong {
                    packet_sequence: 12,
                    channel: RailcomChannel::Channel2,
                    provided_len: 7,
                    max_len: 6,
                },
            ))
        );
    }
}
