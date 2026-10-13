//! Dependencies used by the Z21 application handlers.
//!
//! This lives between the UDP transport and command dispatch so neither layer
//! needs to own the other's context type.

use core::cell::Cell;

use crate::application::LeasePermit;
use crate::application::StatusModel;
use crate::dcc::channels::{
    LocoRequestSender, LocoResponseReceiver, PomRequestSender, PomResponseReceiver,
};
use crate::system_status::channels::FaultEventSender;

/// Dependencies shared by every Z21 handler, built once per UDP frame.
pub(super) struct Z21Ctx<'a> {
    pub status_model: &'a StatusModel,
    pub lease_permit: Option<LeasePermit>,
    pub fault_sender: &'a FaultEventSender,
    pub loco: LocoChannels<'a>,
    pub pom: PomChannels<'a>,
}

/// Locomotive request/response channels and their request-id counter.
pub(super) struct LocoChannels<'a> {
    pub request_sender: &'a LocoRequestSender,
    pub response_receiver: &'a LocoResponseReceiver,
    /// Local counter: the network task is the sole locomotive request producer.
    pub next_request_id: &'a Cell<u32>,
}

/// Programming-on-Main request/response channels and their request-id counter.
pub(super) struct PomChannels<'a> {
    pub request_sender: &'a PomRequestSender,
    pub response_receiver: &'a PomResponseReceiver,
    /// Local counter: the network task is the sole POM request producer.
    pub next_request_id: &'a Cell<u32>,
}
