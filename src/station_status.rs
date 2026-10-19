//! Station status and fault types: plain data shared by the application
//! policies, the fault manager and the runtime tasks.
//!
//! These types carry no channel, peripheral or display dependency, so the
//! application layer can name them without pulling in `system_status`, which
//! also holds the Embassy channels and the display events. `system_status`
//! re-exports them for the task modules.

use crate::authority::LeasePermit;

/// Fault origin used for observability and future policy routing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum FaultCause {
    TrackShort,
    Estop,
    Internal,
}

/// Mutually exclusive safety state governing track power and motion.
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

/// Runtime status events emitted by tasks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum SystemStatusEvent {
    BootStarted,
    BootCompleted,
    WifiConnecting,
    WifiConnected,
    WifiDisconnected,
    EstopActive,
    EstopCleared,
    FaultLatched(FaultCause),
    FaultCleared,
}

/// LED representation of the current system state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum LedState {
    Booting,
    WifiConnecting,
    Running,
    EstopActive,
    FaultLatched,
}

/// External events consumed by the fault state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum FaultEvent {
    /// Boot sequence has finished; track power may be enabled if state permits.
    TrackPowerArmed,
    /// Physical stop button pressed (GPIO22).
    StopPressed,
    /// Resume button short-press (<2 s): clears e-stop, ignored during fault.
    ResumeShortPressed,
    /// Resume button long-press (≥2 s): force-clears any latched state.
    ResumeLongPressed,
    /// Network power-on request carrying the lease that must still be current
    /// when the physical output is enabled.
    NetworkResume(LeasePermit),
    /// Hardware or service fault detected (e.g. track short, CV error).
    FaultLatched(FaultCause),
}
