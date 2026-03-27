//! Network layer — WiFi + Z21 UDP control.

#[cfg(target_arch = "riscv32")]
pub mod udp_control;
pub mod z21_proto;

// esp-radio 0.17.0 defines these stubs only for Xtensa chips (#[cfg(xtensa)]).
// On RISC-V (ESP32-C6) the precompiled WiFi library still references them via
// EXTERN/PROVIDE in the linker script; without them the release build fails.
#[cfg(target_arch = "riscv32")]
mod esp_radio_stubs {
    #[unsafe(no_mangle)]
    unsafe extern "C" fn __esp_radio_misc_nvs_deinit() {}

    #[unsafe(no_mangle)]
    unsafe extern "C" fn __esp_radio_misc_nvs_init() -> i32 {
        0
    }
}

use crate::dcc::{DccAddress, Direction, SpeedFormat};

/// Mirror of the commanded loco state kept by the net task.
///
/// `speed` is the logical protocol speed for the current format:
/// - Speed28  → 0-28  (0=stop, 1-28=speed steps)
/// - Speed128 → 0-126 (0=stop, 1-126=speed steps; wire value 1 is reserved for e-stop)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LocoState {
    pub address: DccAddress,
    pub speed: u8,
    pub direction: Direction,
    pub format: SpeedFormat,
    /// F0-F28 bitmask, bit N = F(N), F0=bit0
    pub functions: u32,
}

/// Fixed-size loco slot array — no heap alloc.
pub type LocoSlots = [Option<LocoState>; 12];

/// Returns whether a logical loco speed represents motion for its DCC format.
#[must_use]
pub const fn loco_is_moving(speed: u8, format: SpeedFormat) -> bool {
    match format {
        SpeedFormat::Speed28 => speed > 0,
        SpeedFormat::Speed128 => speed > 0,
    }
}

/// Returns whether loco motion/function commands may mutate runtime state.
#[must_use]
pub const fn loco_commands_allowed(track_power_on: bool) -> bool {
    track_power_on
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_speed28_motion_threshold_includes_step_one() {
        assert!(!loco_is_moving(0, SpeedFormat::Speed28));
        assert!(loco_is_moving(1, SpeedFormat::Speed28));
    }

    #[test]
    fn test_speed128_motion_threshold_includes_step_one() {
        assert!(!loco_is_moving(0, SpeedFormat::Speed128));
        assert!(loco_is_moving(1, SpeedFormat::Speed128));
    }

    #[test]
    fn test_loco_commands_blocked_when_track_power_off() {
        assert!(!loco_commands_allowed(false));
        assert!(loco_commands_allowed(true));
    }
}
