//! Dependencies used by the Z21 application handlers.
//!
//! This lives between the UDP transport and command dispatch so neither layer
//! needs to own the other's context type.

use core::cell::Cell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};

use crate::application::StatusModel;
use crate::dcc::{LocoRequestMessage, LocoResponse, PomRequest, PomResponse, SchedulerCommand};
use crate::system_status::FaultEvent;

/// Shared dependencies threaded through the Z21 command handlers.
///
/// Per-call buffers and locomotive slots remain explicit because handlers
/// mutate them for each incoming frame.
pub(super) struct Z21Ctx<'a> {
    pub(super) scheduler_sender: &'a Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
    pub(super) fault_sender: &'a Sender<'static, CriticalSectionRawMutex, FaultEvent, 16>,
    pub(super) status_model: &'a StatusModel,
    pub(super) pom_request_sender: &'a Sender<'static, CriticalSectionRawMutex, PomRequest, 1>,
    pub(super) pom_response_receiver:
        &'a Receiver<'static, CriticalSectionRawMutex, PomResponse, 1>,
    /// Local counter: the network task is the sole POM request producer.
    pub(super) next_pom_request_id: &'a Cell<u32>,
    pub(super) loco_request_sender:
        &'a Sender<'static, CriticalSectionRawMutex, LocoRequestMessage, 1>,
    pub(super) loco_response_receiver:
        &'a Receiver<'static, CriticalSectionRawMutex, LocoResponse, 1>,
    /// Local counter: the network task is the sole locomotive request producer.
    pub(super) next_loco_request_id: &'a Cell<u32>,
}
