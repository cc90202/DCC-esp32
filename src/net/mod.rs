//! Network infrastructure: WiFi, provisioning, and UDP transport adapters.

#[cfg(target_arch = "riscv32")]
pub(crate) mod client_watchdog;
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
pub(crate) mod udp_control;
#[cfg(target_arch = "riscv32")]
mod wifi;
#[cfg(target_arch = "riscv32")]
pub use wifi::{NetInitError, UdpBindError};
pub mod wifi_config;
#[cfg(target_arch = "riscv32")]
mod z21_context;
#[cfg(target_arch = "riscv32")]
mod z21_dispatch;
// Counter readers for the bench diagnostics dump; see the `bench-diag`
// feature in Cargo.toml. Both conditions are needed: the modules themselves
// only exist on the firmware target.
#[cfg(all(target_arch = "riscv32", feature = "bench-diag"))]
pub(crate) use {
    loco_client::loco_response_timeout_count,
    udp_control::{status_broadcast_send_failure_count, udp_receive_failure_count},
    z21_dispatch::{loco_command_rejected_count, railcom_getdata_no_data_count},
};
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
