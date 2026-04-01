//! RMT packet-boundary telemetry for RailCom integration.
//!
//! This module does not control physical cutout hardware. It only counts packet-boundary events seen by the RMT ISR.
//! The real cutout decision lives in the scheduler and the physical cutout execution lives in `track_output`. These counters are therefore boundary telemetry only, not proof that a cutout was authorized or started.

pub mod parser;
pub mod pipeline;
pub mod uart_adapter;
#[cfg(target_arch = "riscv32")]
pub mod uart_reader;
pub mod window_bridge;

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

#[cfg(target_arch = "riscv32")]
use crate::dcc::scheduler::{RailcomSchedulerStats, railcom_scheduler_stats};
#[cfg(target_arch = "riscv32")]
use crate::railcom::pipeline::{RailcomRxStats, railcom_rx_stats};
#[cfg(target_arch = "riscv32")]
use crate::track_output::{TrackOutputStats, stats as track_output_stats};

// Global statics are the only viable mechanism for ISR ↔ task communication
// on this target: the ISR cannot receive injected dependencies, and
// embassy_sync primitives are not safe to use from a hardware interrupt
// context. All fields are atomic, so no critical section is needed.
static CUTOUT_ENABLED: AtomicBool = AtomicBool::new(false);
static PACKET_BOUNDARY_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_ENABLED_BOUNDARY_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_DISABLED_BOUNDARY_COUNT: AtomicU32 = AtomicU32::new(0);

/// Snapshot of RMT boundary telemetry counters used during RailCom bring-up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomStats {
    pub packet_boundary_count: u32,
    pub boundary_with_cutout_enabled_count: u32,
    pub boundary_with_cutout_disabled_count: u32,
    pub cutout_enabled: bool,
}

/// Enable or disable the telemetry flag that marks whether cutout would be allowed at the boundary hook.
///
/// In the current software-only phase this flag only changes counters and does
/// not alter any hardware behavior.
pub fn set_cutout_enabled(enabled: bool) {
    CUTOUT_ENABLED.store(enabled, Ordering::Release);
}

/// Returns whether the boundary telemetry currently marks cutout as enabled.
#[must_use]
pub fn cutout_enabled() -> bool {
    CUTOUT_ENABLED.load(Ordering::Acquire)
}

/// Clears all counters and restores the default disabled state.
/// Test-only: runtime counters are monotonic by design.
#[cfg(test)]
pub fn reset_scaffold_state() {
    CUTOUT_ENABLED.store(false, Ordering::Release);
    PACKET_BOUNDARY_COUNT.store(0, Ordering::Release);
    CUTOUT_ENABLED_BOUNDARY_COUNT.store(0, Ordering::Release);
    CUTOUT_DISABLED_BOUNDARY_COUNT.store(0, Ordering::Release);
}

/// Called at each DCC packet boundary by the RMT ISR when the `railcom`
/// feature is enabled.
///
/// This function is intentionally tiny: it performs only atomic operations so
/// the ISR timing remains effectively unchanged in this phase.
///
/// # Safety (link_section)
///
/// The `unsafe(link_section = ".rwtext")` attribute forces placement in RAM
/// on riscv32 targets. This is required because the function is called from
/// the `#[ram]` RMT ISR — flash-resident code can stall under WiFi / cache
/// pressure, causing timing violations or watchdog resets. Removing the
/// attribute may cause intermittent failures under radio load.
#[inline(always)]
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".rwtext"))]
pub fn on_dcc_packet_boundary() {
    PACKET_BOUNDARY_COUNT.fetch_add(1, Ordering::Relaxed);

    if CUTOUT_ENABLED.load(Ordering::Relaxed) {
        CUTOUT_ENABLED_BOUNDARY_COUNT.fetch_add(1, Ordering::Relaxed);
    } else {
        CUTOUT_DISABLED_BOUNDARY_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

/// Returns a copy of the current boundary telemetry counters.
#[must_use]
pub fn stats() -> RailcomStats {
    RailcomStats {
        packet_boundary_count: PACKET_BOUNDARY_COUNT.load(Ordering::Acquire),
        boundary_with_cutout_enabled_count: CUTOUT_ENABLED_BOUNDARY_COUNT.load(Ordering::Acquire),
        boundary_with_cutout_disabled_count: CUTOUT_DISABLED_BOUNDARY_COUNT.load(Ordering::Acquire),
        cutout_enabled: CUTOUT_ENABLED.load(Ordering::Acquire),
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
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_packet_boundary_counts_when_cutout_disabled() {
        reset_scaffold_state();

        on_dcc_packet_boundary();
        on_dcc_packet_boundary();

        assert_eq!(
            stats(),
            RailcomStats {
                packet_boundary_count: 2,
                boundary_with_cutout_enabled_count: 0,
                boundary_with_cutout_disabled_count: 2,
                cutout_enabled: false,
            }
        );
    }

    #[test]
    fn test_enabled_cutout_stops_incrementing_skip_counter() {
        reset_scaffold_state();
        set_cutout_enabled(true);

        on_dcc_packet_boundary();

        assert_eq!(
            stats(),
            RailcomStats {
                packet_boundary_count: 1,
                boundary_with_cutout_enabled_count: 1,
                boundary_with_cutout_disabled_count: 0,
                cutout_enabled: true,
            }
        );
    }
}
