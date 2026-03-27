#![cfg_attr(not(test), no_std)]
#![deny(clippy::mem_forget)]

#[cfg(target_arch = "riscv32")]
pub mod boot;
#[cfg(target_arch = "riscv32")]
pub mod control_buttons;
pub mod control_logic;
// net module is available on all targets: z21_proto is pure no_std and testable on host.
// The riscv32-only submodules (udp_control) are gated inside net/mod.rs.
pub mod config;
pub mod dcc;
pub mod display;
pub mod fault_manager;
pub mod net;
pub mod short_detector;
#[cfg(target_arch = "riscv32")]
pub mod status_led;
pub mod system_status;
