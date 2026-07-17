//! Embassy channel types owned by the firmware composition root.
//!
//! Keeping these transport details out of `system_status` leaves the status
//! events and reducer independent from the async runtime used to deliver them.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use crate::system_status::{BootReadyEvent, DisplayEvent, FaultEvent, SystemStatusEvent};

pub(crate) type SystemStatusChannel = Channel<CriticalSectionRawMutex, SystemStatusEvent, 16>;

/// Dedicated status channel for the net task (fan-out from system status).
pub(crate) type NetStatusChannel = Channel<CriticalSectionRawMutex, SystemStatusEvent, 8>;

pub(crate) type BootReadyChannel = Channel<CriticalSectionRawMutex, BootReadyEvent, 9>;

pub(crate) type FaultEventChannel = Channel<CriticalSectionRawMutex, FaultEvent, 16>;

pub(crate) type DisplayChannel = Channel<CriticalSectionRawMutex, DisplayEvent, 8>;
