//! `no_std` DCC (Digital Command Control) command station for ESP32-C6.
//!
//! Generates NMRA-compliant DCC track signals via the RMT peripheral and
//! exposes a Z21 UDP network interface for control by JMRI, Z21 apps, and
//! other LAN-based throttles.
//!
//! [`application`] holds the framework-independent use cases and read
//! projections. [`dcc`] covers packet encoding, RMT pulse generation, the
//! scheduler and CV programming (the packet types and their encoding come from
//! the HAL-free `dcc_protocol` crate and are re-exported there), while
//! [`z21`] is the pure protocol contract (re-exported from the HAL-free `z21`
//! crate) and [`net`] the adapters around it: Z21 wiring, WiFi and UDP
//! transport.
//! Runtime state lives in [`system_status`] and `fault_manager`, which react
//! to boot, e-stop and fault events.
//!
//! `display` is split: the `DisplayModel` reducer is host-testable, the I2C
//! rendering task is not. `status_led`, `control_buttons` and `short_detector`
//! are peripheral tasks and only build for the ESP32 target.
//!
//! Over-the-air updates live in the HAL-free `dcc_ota` crate; on the device,
//! `ota` adds tasks, locks and hardware around it, with `flash` (the shared
//! flash handle), `liveness` (task heartbeats), `clock` (time conversions)
//! and `restart` (a restart that also resets the radio).
#![cfg_attr(not(test), no_std)]
#![deny(clippy::mem_forget)]

pub mod application;
mod authority;
#[cfg(target_arch = "riscv32")]
pub mod boot;
#[cfg(target_arch = "riscv32")]
pub(crate) mod clock;
#[cfg(target_arch = "riscv32")]
pub(crate) mod control_buttons;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod control_logic;
pub(crate) mod cutout;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod cutout_metadata;
pub mod dcc;
// ESP runtime adapters live beside their DCC/RailCom feature code on disk, but
// are mounted at the crate root to keep hardware and tasking out of the pure
// protocol modules and their host builds.
#[cfg(target_arch = "riscv32")]
#[path = "dcc/engine.rs"]
pub(crate) mod dcc_runtime;
pub(crate) mod diagnostics;
pub mod display;
pub(crate) mod fault_manager;
#[cfg(target_arch = "riscv32")]
pub(crate) mod flash;
#[cfg(target_arch = "riscv32")]
pub(crate) mod liveness;
pub(crate) use dcc_protocol::logon;
mod macros;
pub mod net;
#[cfg(target_arch = "riscv32")]
pub(crate) mod ota;
pub mod railcom;
#[cfg(target_arch = "riscv32")]
#[path = "railcom/isr_capture.rs"]
pub(crate) mod railcom_capture;
use dcc_protocol::railcom_data;
#[cfg(target_arch = "riscv32")]
pub mod restart;
#[cfg(target_arch = "riscv32")]
#[path = "dcc/rmt_driver.rs"]
pub(crate) mod rmt_dcc;
#[cfg(target_arch = "riscv32")]
pub(crate) mod runtime_channels;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod seqlock;
pub(crate) mod short_detector;
#[cfg(target_arch = "riscv32")]
pub(crate) mod status_led;
pub mod system_status;
#[cfg(target_arch = "riscv32")]
pub(crate) mod track_authority;
#[cfg(target_arch = "riscv32")]
pub(crate) mod track_output;
#[cfg(target_arch = "riscv32")]
pub(crate) mod track_safety;
pub mod z21;
