//! Central fault and e-stop state manager.
//!
//! The pure `policy` module drives all transitions; the async firmware fault-manager task
//! is an adapter that applies the requested effects to Embassy channels and GPIO.
//!
//! # Overview
//!
//! The fault manager is a state machine with three states:
//! - **Normal** — Track powered, motion commands accepted, track driver enabled
//! - **EstopLatched** — Operator pressed stop button; resume clears; track driver disabled
//! - **FaultLatched** — Hardware fault (track short, CV error); long resume or service action clears; track driver disabled
//!
//! The runtime state is exposed as [`FaultManagerState`] for observers. State
//! transitions and their required effects remain private to the pure policy so
//! callers cannot bypass the fault manager's application rules.
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
//! **Side-effects per transition** (emitted by the fault policy):
//! - Scheduler commands: `EmergencyStopAll`, `Pause`, `Resume`
//! - Status events: `EstopActive`, `EstopCleared`, `FaultLatched`, `FaultCleared`
//! - Track-driver enable GPIO: HIGH in Normal, LOW otherwise

mod policy;

pub use policy::FaultManagerState;

#[cfg(target_arch = "riscv32")]
use crate::track_output::TrackOutput;
#[cfg(target_arch = "riscv32")]
use crate::{
    dcc::SchedulerCommand,
    system_status::{DisplayEvent, FaultCause, FaultEvent, SystemStatusEvent},
};
#[cfg(target_arch = "riscv32")]
use embassy_sync::channel::Sender;
#[cfg(target_arch = "riscv32")]
use embassy_sync::watch;
#[cfg(target_arch = "riscv32")]
use policy::{FaultPolicy, SchedulerEffect, StatusEffect};

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
    pub track_output: TrackOutput,
    pub display_sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        crate::system_status::DisplayEvent,
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
        crate::system_status::BootReadyEvent,
        9,
    >,
}

/// Async adapter that applies pure fault-policy decisions to runtime outputs.
///
/// Owns the track-driver enable GPIO and forwards scheduler/status commands via channels.
#[cfg(target_arch = "riscv32")]
#[embassy_executor::task]
pub async fn fault_manager_task(context: FaultManagerTaskContext) -> ! {
    let FaultManagerTaskContext {
        receiver,
        scheduler_sender,
        status_sender,
        net_status_sender,
        mut track_output,
        display_sender,
        state_sender,
        ready_sender,
    } = context;

    let mut policy = FaultPolicy::new();
    track_output.set_track_enabled(policy.track_enabled());
    state_sender.send(policy.state());
    ready_sender
        .send(crate::system_status::BootReadyEvent::FaultManager)
        .await;

    loop {
        let event = receiver.receive().await;
        defmt::info!(
            "fault_manager: event={:?} state_before={:?}",
            event,
            policy.state()
        );
        let decision = policy.handle(event);
        track_output.set_track_enabled(decision.track_enabled);

        if decision.state != decision.previous_state {
            state_sender.send(decision.state);
            defmt::warn!(
                "fault_manager: state {:?} -> {:?}, track_enabled={}, motion_enabled={}",
                decision.previous_state,
                decision.state,
                decision.track_enabled,
                decision.motion_enabled
            );
        } else {
            defmt::info!(
                "fault_manager: state unchanged {:?}, track_enabled={}, motion_enabled={}",
                decision.state,
                decision.track_enabled,
                decision.motion_enabled
            );
        }

        for effect in decision.scheduler_effects.into_iter().flatten() {
            let command = match effect {
                SchedulerEffect::EmergencyStopAll => SchedulerCommand::EmergencyStopAll,
                SchedulerEffect::Pause => SchedulerCommand::Pause,
                SchedulerEffect::Resume => SchedulerCommand::Resume,
                SchedulerEffect::RestartRailcomDiscovery => {
                    SchedulerCommand::RestartRailcomDiscovery
                }
            };
            scheduler_sender.send(command).await;
        }

        for effect in decision.status_effects.into_iter().flatten() {
            let (event, display_event) = match effect {
                StatusEffect::EstopActive => (
                    SystemStatusEvent::EstopActive,
                    DisplayEvent::Fault(FaultCause::Estop),
                ),
                StatusEffect::EstopCleared => {
                    (SystemStatusEvent::EstopCleared, DisplayEvent::FaultCleared)
                }
                StatusEffect::FaultLatched(cause) => (
                    SystemStatusEvent::FaultLatched(cause),
                    DisplayEvent::Fault(cause),
                ),
                StatusEffect::FaultCleared => {
                    (SystemStatusEvent::FaultCleared, DisplayEvent::FaultCleared)
                }
            };
            status_sender.send(event).await;
            net_status_sender.send(event).await;
            if display_sender.try_send(display_event).is_err() {
                defmt::warn!("fault_manager: display event dropped");
            }
        }
    }
}
