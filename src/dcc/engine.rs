//! DCC engine feeder for the ISR-driven RMT backend.

use crate::dcc::encoder::{PulseCode as DccPulseCode, encode_dcc_data_portion};
use crate::dcc::rmt_driver;
use crate::dcc::timing::{IDLE_RMT_SIZE, MAX_DATA_PULSES, RMT_CLOCK_HZ};
use crate::dcc::{DccPacket, encode_dcc_packet};
use crate::fault_manager::FaultEvent;
use crate::system_status::FaultCause;

use embassy_futures::yield_now;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::gpio::Level;
use esp_hal::rmt::PulseCode;
use heapless::Vec;

const _: () = assert!(
    RMT_CLOCK_HZ == 1_000_000,
    "RMT_CLOCK_HZ must be 1MHz for direct microsecond-to-tick conversion"
);

const ISR_WATCHDOG_TIMEOUT: Duration = Duration::from_millis(50);
const ISR_RESET_GRACE_PERIOD: Duration = Duration::from_millis(100);

/// DCC packet channel type for sending packets to the engine.
pub type DccPacketChannel = embassy_sync::channel::Channel<CriticalSectionRawMutex, DccPacket, 16>;

/// Pre-encoded idle waveform used to bootstrap continuous RMT loop mode.
pub type IdleRmtBuffer = Vec<PulseCode, IDLE_RMT_SIZE>;

/// Errors while preparing the pre-encoded idle waveform.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum IdleWaveformBuildError {
    PacketEncoding,
    BufferOverflow,
}

/// Build the pre-encoded idle waveform used to bootstrap the continuous loop.
pub fn build_idle_rmt_buffer() -> Result<IdleRmtBuffer, IdleWaveformBuildError> {
    let idle_packet = DccPacket::Idle;
    let idle_pulses =
        encode_dcc_packet(&idle_packet).map_err(|_| IdleWaveformBuildError::PacketEncoding)?;
    let mut buf: IdleRmtBuffer = Vec::new();
    for pulse in &idle_pulses {
        let rmt_pulse = convert_pulse_to_rmt(pulse);
        buf.push(rmt_pulse)
            .map_err(|_| IdleWaveformBuildError::BufferOverflow)?;
    }
    buf.push(PulseCode::end_marker())
        .map_err(|_| IdleWaveformBuildError::BufferOverflow)?;
    Ok(buf)
}

/// Pure async feeder paced by the ISR ACK.
pub async fn dcc_engine_task(
    receiver: Receiver<'static, CriticalSectionRawMutex, DccPacket, 16>,
    fault_sender: Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>,
) -> ! {
    defmt::info!("DCC engine feeder started");

    let mut last_heartbeat = rmt_driver::isr_heartbeat();
    let mut last_progress = Instant::now();

    loop {
        while !rmt_driver::is_consumed() {
            let heartbeat = rmt_driver::isr_heartbeat();
            if heartbeat != last_heartbeat {
                last_heartbeat = heartbeat;
                last_progress = Instant::now();
            } else if Instant::now().duration_since(last_progress) > ISR_WATCHDOG_TIMEOUT {
                defmt::error!(
                    "RMT ISR heartbeat stalled: heartbeat={}, idle_timeout_ms={}",
                    heartbeat,
                    ISR_WATCHDOG_TIMEOUT.as_millis()
                );
                let _ = fault_sender.try_send(FaultEvent::FaultLatched(FaultCause::Internal));
                Timer::after(ISR_RESET_GRACE_PERIOD).await;
                esp_hal::system::software_reset();
            }
            yield_now().await;
        }

        let packet = receiver.receive().await;
        let next_rmt = match encode_packet_to_rmt_data(&packet) {
            Some(buf) => buf,
            None => {
                defmt::warn!("packet encoding failed, skipping");
                continue;
            }
        };

        rmt_driver::submit_packet(next_rmt.as_slice());
    }
}

fn encode_packet_to_rmt_data(packet: &DccPacket) -> Option<Vec<PulseCode, MAX_DATA_PULSES>> {
    let pulses = match encode_dcc_data_portion(packet) {
        Ok(p) => p,
        Err(e) => {
            defmt::warn!("packet encoding failed: {:?}", e);
            return None;
        }
    };
    convert_to_rmt(&pulses).ok()
}

fn convert_pulse_to_rmt(pulse: &DccPulseCode) -> PulseCode {
    let level1 = if pulse.level1 {
        Level::High
    } else {
        Level::Low
    };
    let level2 = if pulse.level2 {
        Level::High
    } else {
        Level::Low
    };
    PulseCode::new(level1, pulse.length1, level2, pulse.length2)
}

fn convert_to_rmt(pulses: &[DccPulseCode]) -> Result<Vec<PulseCode, MAX_DATA_PULSES>, ()> {
    let mut buf = Vec::new();
    for pulse in pulses {
        buf.push(convert_pulse_to_rmt(pulse)).map_err(|_| ())?;
    }
    Ok(buf)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dcc::timing::PREAMBLE_BITS;

    #[test]
    fn test_idle_waveform_layout_matches_fixed_preamble_design() {
        let idle = build_idle_rmt_buffer().unwrap();
        assert_eq!(idle.len(), IDLE_RMT_SIZE);
        assert_eq!(PREAMBLE_BITS, 20);
        assert_eq!(idle[IDLE_RMT_SIZE - 1], PulseCode::end_marker());
    }
}
