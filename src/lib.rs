//! `no_std` DCC (Digital Command Control) command station for ESP32-C6.
//!
//! Generates NMRA-compliant DCC track signals via the RMT peripheral and
//! exposes a Z21 UDP network interface for control by JMRI, Z21 apps, and
//! other LAN-based throttles.
//!
//! - [`dcc`] — packet encoding, RMT pulse generation, scheduler, CV programming.
//! - [`net`] — Z21 protocol parsing/dispatch (host-testable) and UDP transport.
//! - [`system_status`], [`fault_manager`] — event-driven runtime state (boot, e-stop, faults).
//! - [`display`] — `DisplayModel` reducer is host-testable; the I2C rendering task is ESP-only.
//! - `status_led`, `control_buttons`, `short_detector` — peripheral tasks (ESP-only).
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
pub mod railcom;
pub mod short_detector;
#[cfg(target_arch = "riscv32")]
pub mod status_led;
pub mod system_status;
#[cfg(target_arch = "riscv32")]
pub mod track_output;
