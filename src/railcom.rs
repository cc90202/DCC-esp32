//! RailCom capture, decoding, attribution, and runtime dispatch.
//!
//! The pure parser and pipeline turn UART windows into typed datagrams, the
//! locomotive tracker correlates decoder sightings, and target-only adapters
//! route logon and POM feedback. Cutout authorisation lives in the scheduler
//! and physical timing in `track_output`; this module also exposes aggregate
//! diagnostics across those paths.

pub mod loco_tracker;
pub mod parser;
pub mod pipeline;
#[cfg(target_arch = "riscv32")]
pub(crate) mod pom_dispatch;
#[cfg(target_arch = "riscv32")]
pub(crate) mod runtime_dispatch;
#[cfg(target_arch = "riscv32")]
pub(crate) mod uart_reader;

use core::sync::atomic::{AtomicU32, Ordering};

#[cfg(target_arch = "riscv32")]
use crate::dcc::scheduler::{RailcomSchedulerStats, railcom_scheduler_stats};
#[cfg(target_arch = "riscv32")]
use crate::railcom::loco_tracker::{RailcomLocoTrackerStats, railcom_loco_tracker_stats};
#[cfg(target_arch = "riscv32")]
use crate::railcom::pipeline::{
    RAILCOM_CHANNEL_COUNT, RailcomChannelStats, RailcomRxStats, railcom_rx_channel_stats,
    railcom_rx_stats,
};
#[cfg(target_arch = "riscv32")]
use crate::track_output::{TrackOutputStats, stats as track_output_stats};

// A global atomic is the only viable mechanism for ISR ↔ task communication on
// this target: the ISR cannot receive injected dependencies, and
// `embassy_sync` primitives are not safe to use from a hardware interrupt
// context. Relaxed ordering is sufficient: the boundary counter is advisory
// telemetry, not a synchronisation point.
static PACKET_BOUNDARY_COUNT: AtomicU32 = AtomicU32::new(0);

/// Snapshot of RMT boundary telemetry counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomStats {
    pub packet_boundary_count: u32,
}

#[cfg(test)]
pub fn reset_scaffold_state() {
    PACKET_BOUNDARY_COUNT.store(0, Ordering::Release);
}

/// Called at each DCC packet boundary by the RMT ISR.
///
/// The function is intentionally tiny, a single relaxed atomic increment, so
/// the ISR footprint remains effectively unchanged.
///
/// # Safety (link_section)
///
/// `unsafe(link_section = ".rwtext")` forces placement in RAM on riscv32
/// targets. The function is called from the `#[ram]` RMT ISR; flash-resident
/// code can stall under WiFi / cache pressure, causing timing violations or
/// watchdog resets. Removing the attribute may cause intermittent failures
/// under radio load.
#[inline(always)]
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".rwtext"))]
pub fn on_dcc_packet_boundary() {
    PACKET_BOUNDARY_COUNT.fetch_add(1, Ordering::Relaxed);
}

#[must_use]
pub fn stats() -> RailcomStats {
    RailcomStats {
        packet_boundary_count: PACKET_BOUNDARY_COUNT.load(Ordering::Acquire),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomDiagnostics {
    pub boundary: RailcomStats,
    #[cfg(target_arch = "riscv32")]
    pub scheduler: RailcomSchedulerStats,
    #[cfg(target_arch = "riscv32")]
    pub track_output: TrackOutputStats,
    #[cfg(target_arch = "riscv32")]
    pub rx: RailcomRxStats,
    /// Per-channel receive counters, indexed by `RailcomChannel::index`.
    #[cfg(target_arch = "riscv32")]
    pub rx_channels: [RailcomChannelStats; RAILCOM_CHANNEL_COUNT],
    #[cfg(target_arch = "riscv32")]
    pub loco: RailcomLocoTrackerStats,
}

/// Returns a unified RailCom diagnostic snapshot across boundary telemetry,
/// scheduler policy decisions, and physical cutout execution.
#[cfg(target_arch = "riscv32")]
#[must_use]
pub fn diagnostics() -> RailcomDiagnostics {
    RailcomDiagnostics {
        boundary: stats(),
        scheduler: railcom_scheduler_stats(),
        track_output: track_output_stats(),
        rx: railcom_rx_stats(),
        rx_channels: railcom_rx_channel_stats(),
        loco: railcom_loco_tracker_stats(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_boundary_increments() {
        reset_scaffold_state();

        on_dcc_packet_boundary();
        on_dcc_packet_boundary();

        assert_eq!(
            stats(),
            RailcomStats {
                packet_boundary_count: 2,
            }
        );
    }
}
