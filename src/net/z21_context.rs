//! Dependencies used by the Z21 application handlers.
//!
//! This lives between the UDP transport and command dispatch so neither layer
//! needs to own the other's context type.

use core::cell::Cell;

use embassy_net::{IpAddress, IpEndpoint};

use crate::application::LeasePermit;
use crate::application::StatusModel;
use crate::application::pom::PomSession;
use crate::dcc::channels::{LocoRequestSender, LocoResponseReceiver, PomRequestSender};
use crate::system_status::channels::FaultEventSender;

/// POM requests keyed by client host address; replies go to the full endpoint
/// the request came from, since mobile apps rotate their UDP source port.
pub(super) type Z21PomSession = PomSession<IpAddress, IpEndpoint>;

/// Dependencies shared by every Z21 handler, built once per UDP frame.
pub(super) struct Z21Ctx<'a> {
    /// Sender of the frame being handled.
    pub endpoint: IpEndpoint,
    pub status_model: &'a StatusModel,
    pub lease_permit: Option<LeasePermit>,
    pub fault_sender: &'a FaultEventSender,
    pub loco: LocoChannels<'a>,
    pub pom_request_sender: &'a PomRequestSender,
}

/// Locomotive request/response channels and their request-id counter.
pub(super) struct LocoChannels<'a> {
    pub request_sender: &'a LocoRequestSender,
    pub response_receiver: &'a LocoResponseReceiver,
    /// Local counter: the network task is the sole locomotive request producer.
    pub next_request_id: &'a Cell<u32>,
}
