//! Safety helpers for intentional track shutdown paths.
//!
//! This module keeps the "operator asked us to remove track power" sequence in
//! one place. Those paths must suppress the expected GPIO3 transition before
//! disabling the DRV8874 bridge, otherwise a normal stop can be misreported as
//! a real short circuit.

use embassy_time::Duration;

const INTENTIONAL_TRACK_OFF_SHORT_SUPPRESSION: Duration = Duration::from_millis(250);

/// Disable track output immediately for an intentional stop/power-off action.
///
/// This is for commanded stops only. Real short detection must still call
/// `track_output::emergency_disable()` directly so it never masks an actual
/// fault edge before cutting power.
pub(crate) fn disable_track_intentionally() {
    crate::short_detector::suppress_short_edges_for(INTENTIONAL_TRACK_OFF_SHORT_SUPPRESSION);
    if !crate::track_output::emergency_disable() {
        defmt::warn!("track disable requested before hardware initialization");
    }
}
