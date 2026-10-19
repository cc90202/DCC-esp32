//! Central fault and e-stop state manager.
//!
//! The pure `policy` module drives all transitions. The firmware fault-manager
//! task applies GPIO and state-watch changes immediately, while a separate
//! relay task forwards scheduler and presentation effects without
//! backpressuring the safety loop.
//!
//! The fault manager is a state machine with three states. In `Normal` the
//! track is powered, motion commands are accepted and the track driver is
//! enabled. `EstopLatched` is reached when the operator presses the stop
//! button, and a resume press leaves it. `FaultLatched` is reached on a
//! hardware fault such as a track short or a CV error, and only a long resume
//! press or a service action clears it. In both latched states the track driver
//! is disabled.
//!
//! A fault can be latched from either of the other two states, and it replaces
//! a pending e-stop rather than queueing behind it. A short resume press while
//! a fault is latched is deliberately ignored, so an operator cannot clear a
//! hardware fault by reflex.
//!
//! The runtime state is exposed as [`crate::station_status::FaultManagerState`]
//! for observers. State transitions and their required effects remain private
//! to the pure policy so callers cannot bypass the fault manager's application
//! rules.
#[cfg(any(test, target_arch = "riscv32"))]
mod policy;
#[cfg(any(test, target_arch = "riscv32"))]
mod power_fence;
#[cfg(any(test, target_arch = "riscv32"))]
mod relay;
#[cfg(any(test, target_arch = "riscv32"))]
mod step;
#[cfg(target_arch = "riscv32")]
mod task;

#[cfg(target_arch = "riscv32")]
pub(crate) use relay::{FaultEffectsSignal, FaultEffectsTaskContext, fault_effects_task};
#[cfg(target_arch = "riscv32")]
pub(crate) use task::{FaultManagerTaskContext, fault_manager_task};

#[cfg(target_arch = "riscv32")]
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

/// Signalled once a stop requested while the track interlock is engaged has
/// taken effect and every frame queued before it has left the engine.
#[cfg(target_arch = "riscv32")]
pub(crate) static INTERLOCK_STOP_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signalled after every `TrackPowerArmed` the fault manager applied.
///
/// Like [`INTERLOCK_STOP_DONE`], a `Signal` wakes one waiter only. One wait
/// at a time is guaranteed by the update lifecycle: a confirmation waits
/// only during a trial, a boot arming only before the track is armed, an
/// update or setup only while it holds the update lock.
#[cfg(target_arch = "riscv32")]
pub(crate) static TRACK_POWER_ARMED: Signal<CriticalSectionRawMutex, ()> = Signal::new();
