//! Dependencies used by the Z21 application handlers.
//!
//! This lives between the UDP transport and command dispatch so neither layer
//! needs to own the other's context type.

use core::cell::Cell;

use crate::application::StatusModel;
use crate::runtime_channels::{
    FaultEventSender, LocoRequestSender, LocoResponseReceiver, PomRequestSender,
    PomResponseReceiver, SchedulerCommandSender,
};

/// Dependencies used only by track-power and status handlers.
pub(super) struct TrackCtx<'a> {
    pub(super) fault_sender: &'a FaultEventSender,
    pub(super) status_model: &'a StatusModel,
}

/// Dependencies used only by locomotive handlers.
pub(super) struct LocoCtx<'a> {
    pub(super) status_model: &'a StatusModel,
    pub(super) request_sender: &'a LocoRequestSender,
    pub(super) response_receiver: &'a LocoResponseReceiver,
    /// Local counter: the network task is the sole locomotive request producer.
    pub(super) next_request_id: &'a Cell<u32>,
}

/// Dependencies used only by Programming-on-Main handlers.
pub(super) struct PomCtx<'a> {
    pub(super) scheduler_sender: &'a SchedulerCommandSender,
    pub(super) status_model: &'a StatusModel,
    pub(super) request_sender: &'a PomRequestSender,
    pub(super) response_receiver: &'a PomResponseReceiver,
    /// Local counter: the network task is the sole POM request producer.
    pub(super) next_request_id: &'a Cell<u32>,
    pub(super) loco: LocoCtx<'a>,
}

/// Family-specific dependencies used by the Z21 router.
pub(super) struct Z21Ctx<'a> {
    pub(super) track: TrackCtx<'a>,
    pub(super) loco: LocoCtx<'a>,
    pub(super) pom: PomCtx<'a>,
}
