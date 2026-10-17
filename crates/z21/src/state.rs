//! State the command station reports to Z21 clients.

use dcc_protocol::{DccAddress, Direction, FunctionState, LogicalSpeed};

/// Locomotive state required to encode a `LAN_X_LOCO_INFO` frame.
///
/// This wire-facing view keeps the protocol contract independent from the
/// application's projection types. Network adapters are responsible for
/// mapping their state into this representation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LocoInfo {
    /// Locomotive reported.
    pub address: DccAddress,
    /// Current speed, with the speed steps it is held in.
    pub speed: LogicalSpeed,
    /// Current direction of travel.
    pub direction: Direction,
    /// Functions F0..=F28 currently on.
    pub functions: FunctionState,
}

/// Command station state reported in the Z21 `CentralState` byte.
///
/// `LAN_X_STATUS_CHANGED` reports only `estop` and `track_on`;
/// `LAN_SYSTEMSTATE_DATACHANGED` also reports `short_circuit`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CentralState {
    /// Track output is powered; `false` sets the track-voltage-off bit.
    pub track_on: bool,
    /// Emergency stop is active.
    pub estop: bool,
    /// The track output is in a short-circuit fault.
    pub short_circuit: bool,
}
