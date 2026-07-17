//! `no_std` DCC (Digital Command Control) command station for ESP32-C6.
//!
//! Generates NMRA-compliant DCC track signals via the RMT peripheral and
//! exposes a Z21 UDP network interface for control by JMRI, Z21 apps, and
//! other LAN-based throttles.
//!
//! - [`application`] - framework-independent use cases and read projections.
//! - [`dcc`] - packet encoding, RMT pulse generation, scheduler, CV programming.
//! - [`z21`] - pure Z21 protocol contract, parsing, and encoding.
//! - [`net`] - Z21 adapters, WiFi, and UDP transport.
//! - [`system_status`], [`fault_manager`] - event-driven runtime state (boot, e-stop, faults).
//! - [`display`] - `DisplayModel` reducer is host-testable; the I2C rendering task is ESP-only.
//! - `status_led`, `control_buttons`, `short_detector` - peripheral tasks (ESP-only).
#![cfg_attr(not(test), no_std)]
#![deny(clippy::mem_forget)]

pub mod application;
#[cfg(target_arch = "riscv32")]
pub mod boot;
pub mod config;
#[cfg(target_arch = "riscv32")]
pub(crate) mod control_buttons;
pub mod control_logic;
pub mod cutout;
pub mod dcc;
// ESP runtime adapters live beside their DCC/RailCom feature code on disk, but
// are mounted at the crate root to keep hardware and tasking out of the pure
// protocol modules and their host builds.
#[cfg(target_arch = "riscv32")]
#[path = "dcc/engine.rs"]
pub(crate) mod dcc_runtime;
pub mod display;
pub mod fault_manager;
pub mod net;
pub mod railcom;
#[cfg(target_arch = "riscv32")]
#[path = "railcom/isr_capture.rs"]
pub(crate) mod railcom_capture;
pub mod railcom_data;
#[cfg(target_arch = "riscv32")]
#[path = "dcc/rmt_driver.rs"]
pub(crate) mod rmt_dcc;
pub(crate) mod short_detector;
#[cfg(target_arch = "riscv32")]
pub(crate) mod status_led;
pub mod system_status;
#[cfg(target_arch = "riscv32")]
pub(crate) mod track_output;
#[cfg(target_arch = "riscv32")]
pub(crate) mod track_safety;
pub mod z21;
