//! DCC Packet Scheduler with Cyclic Refresh
//!
//! Manages active locomotive slots and retransmits commands cyclically to keep
//! decoders updated.
//!
//! # Overview
//!
//! The scheduler runs as an async Embassy actor task. It accepts `SchedulerCommand` messages
//! (SetSpeed, SetFunction, EmergencyStop, etc.) via a channel and generates a continuous
//! stream of DCC packets to refresh decoder state. Scheduling policy prioritizes:
//! - **Priority 1**: Dirty speed packets (just changed)
//! - **Priority 2**: Dirty function packets (just changed)
//! - **Priority 3**: Cyclic refresh (ensure all active locos get periodic updates)
//!
//! The scheduler supports:
//! - Up to 12 active locomotive slots
//! - Consist (multi-unit train) management
//! - Pause/Resume coordination with CV programming (main track stops during programming)
//!
//! # Function Index
//!
//! Functions are controlled via `FunctionIndex`, which validates the index 0-28:
//! - 0 = FL (headlight)
//! - 1-4 = F1-F4 (transmitted in Function Group 1)
//! - 5-8 = F5-F8 (Function Group 2A)
//! - 9-12 = F9-F12 (Function Group 2B)
//! - 13-20 = F13-F20 (Function Group 3)
//! - 21-28 = F21-F28 (Function Group 4)

use crate::dcc::packet::{DccAddress, DccPacket, Direction};
use crate::dcc::speed28::logical_to_nmra_packet_speed;
#[cfg(target_arch = "riscv32")]
use crate::dcc::{CutoutMode, DccFrame};
use core::sync::atomic::{AtomicU32, Ordering};
use heapless::Vec;

/// Maximum number of active locomotive slots
const MAX_SLOTS: usize = 12;
/// Maximum number of consists managed in software
const MAX_CONSISTS: usize = 8;
/// Maximum members per consist
const MAX_CONSIST_MEMBERS: usize = 8;
/// Scheduler target period to revisit one slot in normal conditions.
const TARGET_SLOT_PERIOD_MS: u64 = 120;
/// Minimum scheduler tick to avoid tight loops under high slot count.
const MIN_TICK_MS: u64 = 5;
/// Maximum allowed interval between refreshes of active function groups.
const MAX_FUNCTION_REFRESH_MS: u64 = 400;
/// Additional immediate retransmissions after a function state change.
const DIRTY_FUNCTION_RETRY_COUNT: u8 = 6;
/// Additional immediate retransmissions after a stop command.
const DIRTY_STOP_RETRY_COUNT: u8 = 4;
/// Minimum number of transmitted packets between two RailCom cutouts on
/// non-Programming traffic. Caps telemetry cutouts at roughly 1 every 45 ms
/// (6 × ~7.5 ms/packet) so the detector has time to settle between windows
/// and refresh/idle packets still make measurable forward progress.
#[cfg_attr(not(target_arch = "riscv32"), allow(dead_code))]
const RAILCOM_MIN_PACKET_GAP: u8 = 6;

#[cfg(target_arch = "riscv32")]
mod railcom_discovery;
mod railcom_policy;
mod slot_manager;

pub use railcom_policy::{RailcomSchedulerStats, railcom_scheduler_stats};
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) use slot_manager::PENDING_POM_CAPACITY;
pub use slot_manager::SlotManager;

/// Speed format for a locomotive slot
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum SpeedFormat {
    Speed28,
    Speed128,
}

/// Function index mapping:
/// - 0 => FL
/// - 1..=28 => F1..F28
const MAX_FUNCTION_INDEX: u8 = 28;

/// Validated function index (F0..F28).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FunctionIndex(u8);

/// Error returned when function index is out of range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidFunctionIndex;

impl FunctionIndex {
    /// Create a validated function index in range 0..=28.
    #[must_use]
    pub fn new(value: u8) -> Option<Self> {
        (value <= MAX_FUNCTION_INDEX).then_some(Self(value))
    }

    /// Return the raw function index value.
    #[must_use]
    pub const fn get(self) -> u8 {
        self.0
    }
}

impl TryFrom<u8> for FunctionIndex {
    type Error = InvalidFunctionIndex;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        FunctionIndex::new(value).ok_or(InvalidFunctionIndex)
    }
}

/// Command messages sent to the packet scheduler actor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerCommand {
    SetSpeed {
        address: DccAddress,
        speed: LogicalSpeed,
        direction: Direction,
        format: SpeedFormat,
    },
    SetFunction {
        address: DccAddress,
        function: FunctionIndex,
        enabled: bool,
    },
    EmergencyStopAll,
    EmergencyStop {
        address: DccAddress,
    },
    ProgramOnMain {
        packet: DccPacket,
    },
    /// Stop startup RailCom discovery traffic before latency-sensitive POM.
    ///
    /// Discovery is useful while idle, but during POM reads CH2 must be left
    /// free for app:pom instead of logon/search responses.
    SuspendRailcomDiscovery,
    /// Ensure periodic packets exist for a decoder address so RailCom feedback
    /// that is emitted after a POM command can be attributed to that decoder.
    EnsureRailcomRefresh {
        address: DccAddress,
        format: SpeedFormat,
    },
    /// Send one RailCom-related packet with a telemetry cutout.
    RailcomTelemetry {
        packet: DccPacket,
    },
    CreateConsist {
        id: u8,
    },
    AddToConsist {
        id: u8,
        address: DccAddress,
        reverse_in_consist: bool,
    },
    RemoveFromConsist {
        id: u8,
        address: DccAddress,
    },
    SetConsistSpeed {
        id: u8,
        speed: LogicalSpeed,
        direction: Direction,
    },
    RemoveSlot {
        address: DccAddress,
    },
    /// Pause packet emission (stop total, preserve state)
    /// Used by programming track service to halt main track output
    Pause,
    /// Resume packet emission (dirty packets first, then refresh)
    /// Used by programming track service to restart main track after CV operations
    Resume,
}

#[cfg(test)]
fn is_safety_critical_packet(packet: &DccPacket) -> bool {
    matches!(
        packet,
        DccPacket::BroadcastStop | DccPacket::EmergencyStop { .. }
    )
}

#[cfg(test)]
fn advance_safety_send_timeout_streak(streak: u8, timed_out: bool) -> u8 {
    if timed_out {
        streak.saturating_add(1)
    } else {
        0
    }
}

// --- Embedded-only scheduler task ---
#[cfg(target_arch = "riscv32")]
use embassy_futures::yield_now;

#[cfg(target_arch = "riscv32")]
pub type SchedulerCommandChannel = embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    SchedulerCommand,
    32,
>;

#[cfg(target_arch = "riscv32")]
pub async fn packet_scheduler_task(
    command_receiver: embassy_sync::channel::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        SchedulerCommand,
        32,
    >,
    sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        DccPacket,
        16,
    >,
    display_sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        crate::system_status::DisplayEvent,
        8,
    >,
) -> ! {
    let mut slot_manager = SlotManager::new();
    let mut prev_slot_count: usize = 0;
    let mut railcom_budget = RailcomCutoutBudget::new();
    let mut discovery = RailcomDiscovery::new(Instant::now());

    loop {
        // Drain all pending commands before generating the next packet.
        // Commands are processed with at most one packet interval (~8 ms) of latency.
        while let Ok(cmd) = command_receiver.try_receive() {
            let accepted = if matches!(cmd, SchedulerCommand::SuspendRailcomDiscovery) {
                discovery.suspend(Instant::now());
                true
            } else {
                slot_manager.apply_command(cmd)
            };
            if accepted && matches!(cmd, SchedulerCommand::Resume) {
                discovery.restart(Instant::now());
                defmt::info!("railcom discovery: restarted");
            }
            if !accepted {
                defmt::warn!("scheduler command rejected");
            }
        }

        let slot_count = slot_manager.slot_count();
        if slot_count != prev_slot_count {
            prev_slot_count = slot_count;
            let _ = display_sender.try_send(crate::system_status::DisplayEvent::ActiveLocoCount(
                slot_count as u8,
            ));
        }
        let (mut packet, class) = slot_manager
            .build_next_packet_classified_with_function_budget(
                allowed_slot_visits_without_function(slot_count.max(1)),
            )
            .unwrap_or((DccPacket::Idle, PacketClass::Idle));
        let cutout = match (railcom_budget.allow_cutout_for(class), class) {
            (true, PacketClass::Programming) => CutoutMode::Pom,
            (true, _) => CutoutMode::Telemetry,
            (false, _) => CutoutMode::None,
        };
        let to_send = DccFrame::new(packet, cutout);

        // send().await blocks until engine drains the channel — natural backpressure.
        // This paces the scheduler exactly to the engine's transmission rate (~8 ms/packet),
        // eliminating the old 120 ms tick that starved the channel with idle packets.
        sender.send(to_send).await;

        yield_now().await;
    }
}

#[cfg(test)]
mod tests;
