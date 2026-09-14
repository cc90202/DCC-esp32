//! RailCom capture, decoding, attribution, and runtime dispatch.
//!
//! The pure parser and pipeline turn UART windows into typed datagrams, the
//! locomotive tracker correlates decoder sightings, and target-only adapters
//! route logon and POM feedback. Cutout authorisation lives in the scheduler
//! and physical timing in `track_output`. Each of those publishes its own
//! counters; assembling a view across them is the composition root's job, not
//! this module's — reaching out for their snapshots was the only reason this
//! module depended on `track_output`, and that dependency closed a cycle.

pub mod loco_tracker;
pub mod parser;
pub mod pipeline;
#[cfg(target_arch = "riscv32")]
pub(crate) mod pom_dispatch;
#[cfg(target_arch = "riscv32")]
pub(crate) mod runtime_dispatch;
#[cfg(target_arch = "riscv32")]
pub(crate) mod uart_reader;

pub use crate::cutout::{PacketSequence, RailcomChannel};
pub use crate::railcom_data::{RailcomDatagram, RailcomItem};

use core::sync::atomic::{AtomicU32, Ordering};

// A global atomic is the smallest sufficient mechanism for this single counter
// shared between the ISR and tasks. Relaxed ordering is sufficient: the value
// is advisory telemetry, not a synchronisation point.
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
