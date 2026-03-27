//! Central fault and e-stop state manager.
//!
//! Pure state-machine [`FaultManagerState::reduce`] drives all transitions;
//! the async [`fault_manager_task`] only performs I/O on its output.
//!
//! # Overview
//!
//! The fault manager is a state machine with three states:
//! - **Normal** — Track powered, motion commands accepted, H-bridge enabled
//! - **EstopLatched** — Operator pressed stop button; resume clears; H-bridge disabled
//! - **FaultLatched** — Hardware fault (track short, CV error); long resume or service action clears; H-bridge disabled
//!
//! # Examples
//!
//! **Host-side state machine test:**
//!
//! ```
//! use dcc_esp32::fault_manager::{FaultManagerState, FaultEvent, FaultCause};
//!
//! // Start in Normal state
//! let mut state = FaultManagerState::Normal;
//!
//! // Operator presses stop button
//! let event = FaultEvent::StopPressed;
//! state = FaultManagerState::reduce(state, event);
//! assert_eq!(state, FaultManagerState::EstopLatched);
//!
//! // Operator presses resume (short press, <2s)
//! state = FaultManagerState::reduce(state, FaultEvent::ResumeShortPressed);
//! assert_eq!(state, FaultManagerState::Normal);
//! ```
//!
//! **Fault latching (e.g., track short):**
//!
//! ```
//! use dcc_esp32::fault_manager::{FaultManagerState, FaultEvent, FaultCause};
//!
//! let mut state = FaultManagerState::Normal;
//!
//! // Short detector triggers; sends FaultEvent::FaultLatched(TrackShort)
//! state = FaultManagerState::reduce(
//!     state,
//!     FaultEvent::FaultLatched(FaultCause::TrackShort),
//! );
//! assert_eq!(state, FaultManagerState::FaultLatched(FaultCause::TrackShort));
//!
//! // Only long resume or service clear can exit
//! state = FaultManagerState::reduce(state, FaultEvent::ResumeLongPressed);
//! assert_eq!(state, FaultManagerState::Normal);
//! ```
//!
//! ```text
//!                    StopPressed
//!            ┌───────────────────────┐
//!            │                       ▼
//!        ┌────────┐           ┌─────────────┐
//!        │ Normal │           │ EstopLatched │
//!        └────────┘           └─────────────┘
//!            ▲  │                  │  ▲  │
//!  Resume*   │  │ FaultLatched     │  │  │ FaultLatched
//!  FaultClr  │  │ (cause)         │  │  │ (cause)
//!            │  ▼                  │  │  ▼
//!        ┌──────────────┐ ◄───────┘  │
//!        │ FaultLatched │    (estop   │
//!        │   (cause)    │  replaced)  │
//!        └──────────────┘             │
//!              │  ResumeShort ─── ignored (fault stays)
//!              │  ResumeLong ──── clears fault ──► Normal
//!              │  FaultClearedByService ──────────► Normal
//!              └──────────────────────────────────────┘
//! ```
//!
//! **Side-effects per transition** (emitted by `Transition`):
//! - Scheduler commands: `EmergencyStopAll`, `Pause`, `Resume`
//! - Status events: `EstopActive`, `EstopCleared`, `FaultLatched`, `FaultCleared`
//! - H-bridge GPIO: HIGH in Normal, LOW otherwise

#[cfg(target_arch = "riscv32")]
use core::sync::atomic::{AtomicBool, Ordering};

use crate::system_status::FaultCause;
#[cfg(target_arch = "riscv32")]
use crate::{dcc::SchedulerCommand, system_status::SystemStatusEvent};
#[cfg(target_arch = "riscv32")]
use embassy_sync::channel::Sender;
#[cfg(target_arch = "riscv32")]
use embassy_sync::watch;

#[cfg(target_arch = "riscv32")]
static MOTION_COMMANDS_ENABLED: AtomicBool = AtomicBool::new(true);
#[cfg(target_arch = "riscv32")]
static TRACK_POWER_ARMED: AtomicBool = AtomicBool::new(false);

/// External events consumed by the fault state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum FaultEvent {
    /// Boot sequence has finished; track power may be enabled if state permits.
    TrackPowerArmed,
    /// Physical stop button pressed (GPIO22).
    StopPressed,
    /// Resume button short-press (<2 s) — clears e-stop, ignored during fault.
    ResumeShortPressed,
    /// Resume button long-press (≥2 s) — force-clears any latched state.
    ResumeLongPressed,
    /// Hardware or service fault detected (e.g. track short, CV error).
    FaultLatched(FaultCause),
    /// Fault condition resolved by automated service logic.
    FaultClearedByService,
}

/// Tri-state machine governing track safety.
///
/// Only [`Normal`](Self::Normal) allows motion commands and H-bridge output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum FaultManagerState {
    /// Track powered, motion commands accepted.
    Normal,
    /// Operator-initiated emergency stop; resume button clears.
    EstopLatched,
    /// Hardware/service fault; requires long-press or service clear.
    FaultLatched(FaultCause),
}

/// Returns `true` when the system is in [`Normal`](FaultManagerState::Normal) state.
///
/// Checked by the network layer before forwarding speed/function commands to the scheduler.
#[cfg(target_arch = "riscv32")]
#[must_use]
pub fn motion_commands_enabled() -> bool {
    MOTION_COMMANDS_ENABLED.load(Ordering::Acquire)
}

/// Allow the fault manager to energize the track when state returns to `Normal`.
#[cfg(target_arch = "riscv32")]
pub fn arm_track_power_output() {
    TRACK_POWER_ARMED.store(true, Ordering::Release);
}

#[cfg(target_arch = "riscv32")]
fn set_motion_commands_enabled_for_state(state: FaultManagerState) {
    let enabled = matches!(state, FaultManagerState::Normal);
    MOTION_COMMANDS_ENABLED.store(enabled, Ordering::Release);
}

/// Complete description of a state transition's side-effects.
///
/// Produced by [`FaultManagerState::reduce`]; the async task applies each field.
#[cfg(any(target_arch = "riscv32", test))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Transition {
    next: FaultManagerState,
    emit_estop_active: bool,
    emit_estop_cleared: bool,
    emit_fault_latched: Option<FaultCause>,
    emit_fault_cleared: bool,
    send_emergency_stop_all: bool,
    send_pause: bool,
    send_resume: bool,
}

#[cfg(any(target_arch = "riscv32", test))]
impl FaultManagerState {
    /// Pure state reducer — no I/O, fully testable on host.
    ///
    /// Every `(state, event)` pair is handled; unrecognised combinations are no-ops
    /// (the `next` field stays equal to `self`).
    const fn reduce(self, event: FaultEvent) -> Transition {
        let mut t = Transition {
            next: self,
            emit_estop_active: false,
            emit_estop_cleared: false,
            emit_fault_latched: None,
            emit_fault_cleared: false,
            send_emergency_stop_all: false,
            send_pause: false,
            send_resume: false,
        };

        match (self, event) {
            (FaultManagerState::Normal, FaultEvent::TrackPowerArmed)
            | (FaultManagerState::EstopLatched, FaultEvent::TrackPowerArmed)
            | (FaultManagerState::FaultLatched(_), FaultEvent::TrackPowerArmed) => {}
            (FaultManagerState::Normal, FaultEvent::StopPressed) => {
                t.next = FaultManagerState::EstopLatched;
                t.send_emergency_stop_all = true;
                t.send_pause = true;
                t.emit_estop_active = true;
            }
            (FaultManagerState::EstopLatched, FaultEvent::ResumeShortPressed)
            | (FaultManagerState::EstopLatched, FaultEvent::ResumeLongPressed) => {
                t.next = FaultManagerState::Normal;
                t.send_resume = true;
                t.emit_estop_cleared = true;
            }
            (FaultManagerState::FaultLatched(_), FaultEvent::ResumeShortPressed) => {
                // Fault remains latched until service confirms clear.
            }
            (FaultManagerState::FaultLatched(_), FaultEvent::ResumeLongPressed) => {
                t.next = FaultManagerState::Normal;
                t.send_resume = true;
                t.emit_fault_cleared = true;
            }
            (FaultManagerState::Normal, FaultEvent::FaultLatched(cause)) => {
                t.next = FaultManagerState::FaultLatched(cause);
                t.send_pause = true;
                t.emit_fault_latched = Some(cause);
            }
            (FaultManagerState::EstopLatched, FaultEvent::FaultLatched(cause)) => {
                t.next = FaultManagerState::FaultLatched(cause);
                t.emit_estop_cleared = true;
                t.emit_fault_latched = Some(cause);
            }
            (FaultManagerState::FaultLatched(_), FaultEvent::FaultLatched(cause)) => {
                t.next = FaultManagerState::FaultLatched(cause);
                t.emit_fault_latched = Some(cause);
            }
            (FaultManagerState::FaultLatched(_), FaultEvent::FaultClearedByService) => {
                t.next = FaultManagerState::Normal;
                t.send_resume = true;
                t.emit_fault_cleared = true;
            }
            (FaultManagerState::Normal, FaultEvent::ResumeShortPressed)
            | (FaultManagerState::Normal, FaultEvent::ResumeLongPressed)
            | (FaultManagerState::Normal, FaultEvent::FaultClearedByService)
            | (FaultManagerState::EstopLatched, FaultEvent::StopPressed)
            | (FaultManagerState::EstopLatched, FaultEvent::FaultClearedByService)
            | (FaultManagerState::FaultLatched(_), FaultEvent::StopPressed) => {}
        }

        t
    }
}

#[cfg(target_arch = "riscv32")]
pub type FaultEventChannel = embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    FaultEvent,
    16,
>;
#[cfg(target_arch = "riscv32")]
pub type FaultStateWatch =
    watch::Watch<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, FaultManagerState, 1>;

#[cfg(target_arch = "riscv32")]
pub struct FaultManagerTaskContext {
    pub receiver: embassy_sync::channel::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        FaultEvent,
        16,
    >,
    pub scheduler_sender: Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        SchedulerCommand,
        32,
    >,
    pub status_sender: Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        SystemStatusEvent,
        16,
    >,
    pub net_status_sender: Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        SystemStatusEvent,
        8,
    >,
    pub hbridge_enable: esp_hal::gpio::Output<'static>,
    pub display_sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        crate::display::DisplayEvent,
        8,
    >,
    pub state_sender: watch::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        FaultManagerState,
        1,
    >,
    pub ready_sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        crate::boot::BootReadyEvent,
        9,
    >,
}

/// Async event loop that applies [`FaultManagerState::reduce`] and dispatches side-effects.
///
/// Owns the H-bridge enable GPIO and forwards scheduler/status commands via channels.
#[cfg(target_arch = "riscv32")]
#[embassy_executor::task]
pub async fn fault_manager_task(context: FaultManagerTaskContext) -> ! {
    #[inline]
    fn apply_hbridge_enable_for_state(
        state: FaultManagerState,
        hbridge_enable: &mut esp_hal::gpio::Output<'static>,
    ) {
        let armed = TRACK_POWER_ARMED.load(Ordering::Acquire);
        if armed && matches!(state, FaultManagerState::Normal) {
            hbridge_enable.set_high();
        } else {
            hbridge_enable.set_low();
        }
    }

    let FaultManagerTaskContext {
        receiver,
        scheduler_sender,
        status_sender,
        net_status_sender,
        mut hbridge_enable,
        display_sender,
        state_sender,
        ready_sender,
    } = context;

    let mut state = FaultManagerState::Normal;
    set_motion_commands_enabled_for_state(state);
    apply_hbridge_enable_for_state(state, &mut hbridge_enable);
    state_sender.send(state);
    ready_sender
        .send(crate::boot::BootReadyEvent::FaultManager)
        .await;

    loop {
        let event = receiver.receive().await;
        defmt::info!("fault_manager: event={:?} state_before={:?}", event, state);
        let t = state.reduce(event);
        let prev_state = state;
        state = t.next;
        set_motion_commands_enabled_for_state(state);
        apply_hbridge_enable_for_state(state, &mut hbridge_enable);
        if state != prev_state {
            state_sender.send(state);
            defmt::warn!(
                "fault_manager: state {:?} -> {:?}, track_enabled={}, motion_enabled={}",
                prev_state,
                state,
                matches!(state, FaultManagerState::Normal),
                motion_commands_enabled()
            );
        } else {
            defmt::info!(
                "fault_manager: state unchanged {:?}, track_enabled={}, motion_enabled={}",
                state,
                matches!(state, FaultManagerState::Normal),
                motion_commands_enabled()
            );
        }

        if t.send_emergency_stop_all {
            scheduler_sender
                .send(SchedulerCommand::EmergencyStopAll)
                .await;
        }
        if t.send_pause {
            scheduler_sender.send(SchedulerCommand::Pause).await;
        }
        if t.send_resume {
            scheduler_sender.send(SchedulerCommand::Resume).await;
        }

        if t.emit_estop_active {
            let event = SystemStatusEvent::EstopActive;
            status_sender.send(event).await;
            net_status_sender.send(event).await;
            let _ = display_sender
                .try_send(crate::display::DisplayEvent::Fault(FaultCause::Estop));
        }
        if t.emit_estop_cleared {
            let event = SystemStatusEvent::EstopCleared;
            status_sender.send(event).await;
            net_status_sender.send(event).await;
            let _ = display_sender.try_send(crate::display::DisplayEvent::FaultCleared);
        }
        if let Some(cause) = t.emit_fault_latched {
            let event = SystemStatusEvent::FaultLatched(cause);
            status_sender.send(event).await;
            net_status_sender.send(event).await;
            let _ = display_sender.try_send(crate::display::DisplayEvent::Fault(cause));
        }
        if t.emit_fault_cleared {
            let event = SystemStatusEvent::FaultCleared;
            status_sender.send(event).await;
            net_status_sender.send(event).await;
            let _ = display_sender.try_send(crate::display::DisplayEvent::FaultCleared);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_track_power_armed_is_state_machine_noop() {
        let t = FaultManagerState::Normal.reduce(FaultEvent::TrackPowerArmed);
        assert_eq!(t.next, FaultManagerState::Normal);
        assert!(!t.send_resume);
        assert!(!t.send_pause);
        assert!(!t.send_emergency_stop_all);
    }

    #[test]
    fn test_stop_latches_estop() {
        let t = FaultManagerState::Normal.reduce(FaultEvent::StopPressed);
        assert_eq!(t.next, FaultManagerState::EstopLatched);
        assert!(t.send_emergency_stop_all);
        assert!(t.send_pause);
        assert!(t.emit_estop_active);
    }

    #[test]
    fn test_resume_short_from_estop_clears_estop() {
        let t = FaultManagerState::EstopLatched.reduce(FaultEvent::ResumeShortPressed);
        assert_eq!(t.next, FaultManagerState::Normal);
        assert!(t.send_resume);
        assert!(t.emit_estop_cleared);
    }

    #[test]
    fn test_resume_in_normal_is_ignored() {
        let t = FaultManagerState::Normal.reduce(FaultEvent::ResumeShortPressed);
        assert_eq!(t.next, FaultManagerState::Normal);
        assert!(!t.send_resume);
    }

    #[test]
    fn test_resume_ignored_while_fault_latched() {
        let s = FaultManagerState::FaultLatched(FaultCause::CvService);
        let t = s.reduce(FaultEvent::ResumeShortPressed);
        assert_eq!(t.next, s);
        assert!(!t.send_resume);
    }

    #[test]
    fn test_resume_long_clears_fault_latched() {
        let s = FaultManagerState::FaultLatched(FaultCause::TrackShort);
        let t = s.reduce(FaultEvent::ResumeLongPressed);
        assert_eq!(t.next, FaultManagerState::Normal);
        assert!(t.send_resume);
        assert!(t.emit_fault_cleared);
        assert!(!t.emit_estop_cleared);
    }

    #[test]
    fn test_fault_cleared_by_service_returns_to_normal() {
        let s = FaultManagerState::FaultLatched(FaultCause::TrackShort);
        let t = s.reduce(FaultEvent::FaultClearedByService);
        assert_eq!(t.next, FaultManagerState::Normal);
        assert!(t.send_resume);
        assert!(t.emit_fault_cleared);
    }

    #[test]
    fn test_fault_from_normal_pauses_scheduler() {
        let t = FaultManagerState::Normal.reduce(FaultEvent::FaultLatched(FaultCause::TrackShort));
        assert_eq!(
            t.next,
            FaultManagerState::FaultLatched(FaultCause::TrackShort)
        );
        assert!(t.send_pause);
    }

    #[test]
    fn test_fault_replaces_estop_state() {
        let t = FaultManagerState::EstopLatched
            .reduce(FaultEvent::FaultLatched(FaultCause::TrackShort));
        assert_eq!(
            t.next,
            FaultManagerState::FaultLatched(FaultCause::TrackShort)
        );
        assert!(t.emit_estop_cleared);
        assert_eq!(t.emit_fault_latched, Some(FaultCause::TrackShort));
    }
}
