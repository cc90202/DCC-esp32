//! Network infrastructure - WiFi, provisioning, and UDP transport adapters.

#[cfg(any(test, target_arch = "riscv32"))]
mod loco_client;
#[cfg(target_arch = "riscv32")]
mod pom_client;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod provisioning;
#[cfg(target_arch = "riscv32")]
mod radio;
#[cfg(target_arch = "riscv32")]
pub use radio::{RadioInitError, WifiBringupError};
#[cfg(target_arch = "riscv32")]
mod railcom_lookup;
#[cfg(target_arch = "riscv32")]
pub mod udp_control;
#[cfg(target_arch = "riscv32")]
mod wifi;
pub mod wifi_config;
#[cfg(target_arch = "riscv32")]
mod z21_context;
#[cfg(target_arch = "riscv32")]
mod z21_dispatch;
#[cfg(target_arch = "riscv32")]
pub(crate) use loco_client::loco_response_timeout_count;
#[cfg(target_arch = "riscv32")]
pub(crate) use udp_control::{status_broadcast_send_failure_count, udp_receive_failure_count};
#[cfg(target_arch = "riscv32")]
pub(crate) use z21_dispatch::{loco_command_rejected_count, railcom_getdata_no_data_count};
// Compatibility alias for callers using the former protocol path.
pub use crate::z21 as z21_proto;

// esp-radio 0.17.0 defines these stubs only for Xtensa chips (#[cfg(xtensa)]).
// On RISC-V (ESP32-C6) the precompiled WiFi library still references them via
// EXTERN/PROVIDE in the linker script; without them the release build fails.
#[cfg(target_arch = "riscv32")]
mod esp_radio_stubs {
    // SAFETY: The WiFi library expects this exact unmangled symbol at link time.
    // The stub intentionally performs no deinit work on ESP32-C6.
    #[unsafe(no_mangle)]
    unsafe extern "C" fn __esp_radio_misc_nvs_deinit() {}

    // SAFETY: The WiFi library expects this exact unmangled symbol at link time.
    // Returning 0 preserves the "success" contract used by the precompiled blob.
    #[unsafe(no_mangle)]
    unsafe extern "C" fn __esp_radio_misc_nvs_init() -> i32 {
        0
    }
}
