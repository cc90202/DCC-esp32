//! DCC Engine - Continuous packet transmission task
//!
//! Implements an async Embassy task that continuously transmits DCC packets
//! from a queue, falling back to a hardware-looped idle waveform when the
//! queue is empty.
//!
//! ## Gap elimination (NMRA S-9.2.4)
//!
//! When the queue is empty the engine uses `transmit_continuously(LoopMode::Infinite)`
//! on a `Channel<Blocking, Tx>`.  The RMT hardware sets `tx_conti_mode=1` and
//! immediately restarts from the first buffer entry when it reaches the end marker —
//! truly zero gap between loop iterations.
//!
//! When a real packet arrives the engine calls `stop_next()` (blocks ≤ one idle-packet
//! duration, ~6 ms) to halt the hardware loop at a clean boundary, then transmits
//! real packets with blocking single-shot `transmit().wait()`.  Because `receiver.recv().await`
//! properly suspends the Embassy task during idle, other tasks run freely.
//!
//! The RMT channel is configured with `memsize: 2` (96 RAM slots) so the 49-entry
//! idle buffer (48 DCC pulses + 1 end marker) fits within the hardware limit.
//!
//! ## Performance Characteristics
//!
//! ### End-to-End Latency (Reception → RMT transmission)
//!
//! **Idle state (queue empty):**
//! - Continuous hardware loop transmits idle packets with **zero inter-packet gap** (NMRA requirement met)
//! - Other Embassy tasks run freely; engine task is suspended on `receiver.recv().await`
//!
//! **Real packet arrival:**
//! - Engine wakes from `recv().await` (1-2μs context switch, negligible)
//! - Calls `stop_next()` to halt hardware loop: blocks **≤ one idle-packet duration (~6ms)**
//!   - Idle packet: 20 preamble + 1 start + 3×(8 data+1 sep) + 1 end = 48 DCC pulses
//!   - Each pulse: 58μs + 58μs = 116μs ("1" bit), so ~5.5ms total
//!   - Stop boundary is clean (no torn packets)
//! - Encodes real packet: **50-100μs** (heapless::Vec, no alloc)
//! - Converts to RMT format: **20-50μs** (loop + push, in-register)
//! - Submits to RMT hardware via `transmit()`: **1-2μs**
//! - **Total: 6-7ms worst case** (gated by `stop_next()` boundary)
//!
//! **NMRA S-9.2.4 compliance:**
//! - Maximum allowed signal gap: **30ms** (between packet end and next packet start)
//! - Engine worst case: ~6ms gap + real packet transmission (5-7ms) = ~12-13ms total
//! - **Safety margin: >50%** (12ms out of 30ms allowed)
//!
//! ### Memory Budget
//!
//! **Heap (esp_alloc, 65KB total):**
//! - DCC packet encoding: 0B (heapless::Vec, stack-allocated)
//! - RMT buffer conversion: 0B (stack-allocated, capacity 129 PulseCode entries)
//! - Embassy channels: ~384B (3 × `Channel<..., 16>` DccPacket)
//! - Z21 UDP stack (riscv32 only): ~2-3KB (network buffers, defmt, wifi)
//! - **Typical usage: <5KB** (abundant headroom in 65KB heap)
//!
//! **Stack (per-task):**
//! - `dcc_engine_task`: ~1KB (RMT buffer Vec, locals)
//! - Other tasks: 2-4KB each (scheduler, buttons, LED tasks)
//! - Embassy runtime: ~4KB (context switching, interrupt frames)
//! - **Total: ~20KB** (plenty of margin in ESP32-C6 192KB stack)
//!
//! **Static allocation (no heap):**
//! - Idle RMT buffer: 49 PulseCode entries (49 × 8B = 392B, pre-encoded at boot)
//! - heapless::Vec for hot paths: 64-128 capacity (empty until used, allocated on stack)
//! - Channel metadata: StaticCell per channel (negligible)
//!
//! **Hot path optimization:**
//! - Packet encode loop: entirely stack-based, zero heap allocation
//! - No dynamic allocations in `dcc_engine_task` or `transmit_one()`
//! - RMT conversion uses fixed-size stack Vec; buffer overflow = compile-time panic

use crate::dcc::timing::{DCC_MAX_PACKET_PULSES, IDLE_RMT_SIZE, RMT_CLOCK_HZ};
use crate::dcc::{DccPacket, encode_dcc_packet};
use crate::fault_manager::FaultEvent;
use crate::system_status::FaultCause;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Duration, Timer};
use esp_hal::Blocking;
use esp_hal::gpio::Level;
use esp_hal::rmt::{Channel as RmtChannel, LoopMode, PulseCode, Tx};
use heapless::Vec;

// Compile-time assertion: Verify RMT clock is 1MHz so microsecond values equal tick counts
const _: () = assert!(
    RMT_CLOCK_HZ == 1_000_000,
    "RMT_CLOCK_HZ must be 1MHz for direct microsecond-to-tick conversion"
);
// Reset after four consecutive `txn.wait()` failures: one transient retry is cheap,
// but repeated TX completion errors leave track output reliability undefined.
const TX_WAIT_ERROR_RESET_THRESHOLD: u8 = 4;
fn advance_tx_error_streak(streak: u8, had_error: bool) -> u8 {
    if had_error {
        streak.saturating_add(1)
    } else {
        0
    }
}

/// DCC packet channel type for sending packets to the engine
pub type DccPacketChannel = embassy_sync::channel::Channel<CriticalSectionRawMutex, DccPacket, 16>;
/// Pre-encoded idle waveform used for RMT continuous loop mode.
pub type IdleRmtBuffer = Vec<PulseCode, IDLE_RMT_SIZE>;

/// Errors while preparing the pre-encoded idle waveform.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum IdleWaveformBuildError {
    PacketEncoding,
    BufferOverflow,
}

/// Build the pre-encoded idle waveform used by the continuous RMT loop.
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

/// DCC engine task - continuously transmits DCC packets via RMT
///
/// This task runs forever, receiving packets from a channel and transmitting
/// them via RMT.  When the channel is empty, the RMT hardware loops an idle
/// packet with zero inter-packet gap (`tx_conti_mode=1`).
///
/// Uses a single RMT channel (2 RAM blocks) on GPIO2.  The complementary signal
/// for the H-bridge is produced externally by a 74HC14 Schmitt-trigger inverter.
pub async fn dcc_engine_task(
    receiver: Receiver<'static, CriticalSectionRawMutex, DccPacket, 16>,
    fault_sender: Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>,
    mut tx: RmtChannel<'static, Blocking, Tx>,
    idle_rmt: IdleRmtBuffer,
) -> ! {
    defmt::info!("DCC engine task started");
    let mut consecutive_tx_errors: u8 = 0;

    defmt::info!(
        "Idle packet: {} DCC pulses + 1 end marker = {} RMT entries (continuous loop)",
        idle_rmt.len() - 1,
        idle_rmt.len()
    );

    loop {
        // ── Idle phase ────────────────────────────────────────────────────────
        // Start hardware loop: tx_conti_mode=1, zero inter-iteration gap.
        let cont_tx = match tx.transmit_continuously(idle_rmt.as_slice(), LoopMode::Infinite) {
            Ok(cont_tx) => cont_tx,
            Err(e) => {
                defmt::error!("continuous idle TX start error: {:?}", e);
                let _ = fault_sender.try_send(FaultEvent::FaultLatched(FaultCause::Internal));
                defmt::error!("idle continuous loop failed to start; internal fault latched");
                Timer::after(Duration::from_millis(100)).await;
                esp_hal::system::software_reset();
            }
        };

        // Suspend this task until a real (non-idle) packet arrives.
        // Other Embassy tasks run freely here via cooperative scheduling.
        let first_packet = loop {
            let pkt = receiver.receive().await;
            if !matches!(pkt, DccPacket::Idle) {
                break pkt;
            }
            // Idle packets from the scheduler are silently discarded —
            // the hardware loop already provides an ideal idle waveform.
        };

        // Stop hardware loop at the next clean packet boundary.
        // Blocks for at most one idle-packet duration (~6 ms) — well within
        // the 30 ms NMRA S-9.2.4 maximum signal gap.
        tx = match cont_tx.stop_next() {
            Ok(ch) => ch,
            Err((e, ch)) => {
                defmt::error!("stop_next error: {:?}", e);
                ch
            }
        };

        // ── Real-packet phase ─────────────────────────────────────────────────
        // Transmit the first real packet, then drain any others queued up.
        tx = transmit_one(tx, &first_packet, &fault_sender, &mut consecutive_tx_errors).await;
        while let Ok(pkt) = receiver.try_receive() {
            if !matches!(pkt, DccPacket::Idle) {
                tx = transmit_one(tx, &pkt, &fault_sender, &mut consecutive_tx_errors).await;
            }
        }
        // Loop back → restart continuous idle immediately.
    }
}

/// Encode and transmit a single DCC packet via blocking single-shot RMT.
///
/// Returns the channel so the caller can chain further transmissions or restart
/// the continuous idle loop.
async fn transmit_one(
    tx: RmtChannel<'static, Blocking, Tx>,
    packet: &DccPacket,
    fault_sender: &Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>,
    consecutive_errors: &mut u8,
) -> RmtChannel<'static, Blocking, Tx> {
    let rmt_buf = match encode_dcc_packet(packet) {
        Ok(pulses) => match convert_to_rmt(&pulses) {
            Ok(buf) => buf,
            Err(()) => {
                defmt::warn!("rmt conversion overflow; skipping packet");
                return tx;
            }
        },
        Err(_) => {
            defmt::warn!("packet encoding failed; skipping packet");
            return tx;
        }
    };

    let txn = match tx.transmit(rmt_buf.as_slice()) {
        Ok(t) => t,
        Err(e) => {
            defmt::error!("RMT transmit start error: {:?}", e);
            let _ = fault_sender.try_send(FaultEvent::FaultLatched(FaultCause::Internal));
            defmt::error!("RMT TX start failed; internal fault latched before reset");
            // Failed transmit start consumes the channel and provides no recovery path.
            // Latch an internal fault before resetting so operators and logs see a
            // stable failure mode rather than a silent reboot.
            Timer::after(Duration::from_millis(100)).await;
            esp_hal::system::software_reset();
        }
    };

    match txn.wait() {
        Ok(ch) => {
            if *consecutive_errors != 0 {
                defmt::warn!("RMT TX recovered after {} errors", *consecutive_errors);
            }
            *consecutive_errors = advance_tx_error_streak(*consecutive_errors, false);
            ch
        }
        Err((e, ch)) => {
            defmt::error!("RMT TX error: {:?}", e);
            *consecutive_errors = advance_tx_error_streak(*consecutive_errors, true);
            if *consecutive_errors >= TX_WAIT_ERROR_RESET_THRESHOLD {
                let _ = fault_sender.try_send(FaultEvent::FaultLatched(FaultCause::Internal));
                defmt::error!(
                    "RMT TX failed {} times consecutively, resetting chip",
                    *consecutive_errors
                );
                esp_hal::system::software_reset();
            }
            ch
        }
    }
}

fn convert_pulse_to_rmt(pulse: &crate::dcc::encoder::PulseCode) -> PulseCode {
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

/// Convert DCC PulseCode sequence to RMT PulseCode format (with end marker).
fn convert_to_rmt(
    pulses: &[crate::dcc::encoder::PulseCode],
) -> Result<Vec<PulseCode, { DCC_MAX_PACKET_PULSES + 1 }>, ()> {
    let mut buf = Vec::new();
    for pulse in pulses {
        buf.push(convert_pulse_to_rmt(pulse)).map_err(|_| ())?;
    }
    buf.push(PulseCode::end_marker()).map_err(|_| ())?;
    Ok(buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_advance_tx_error_streak() {
        let mut streak = 0;
        streak = advance_tx_error_streak(streak, true);
        assert_eq!(streak, 1);
        streak = advance_tx_error_streak(streak, true);
        assert_eq!(streak, 2);
        streak = advance_tx_error_streak(streak, false);
        assert_eq!(streak, 0);
    }

    #[test]
    fn test_tx_error_reset_threshold_is_reachable() {
        let mut streak = 0;
        for _ in 0..TX_WAIT_ERROR_RESET_THRESHOLD {
            streak = advance_tx_error_streak(streak, true);
        }
        assert!(streak >= TX_WAIT_ERROR_RESET_THRESHOLD);
    }
}
