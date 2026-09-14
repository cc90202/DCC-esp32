//! Embassy transport aliases for DCC messages.
//!
//! Every channel carrying a DCC message is declared here, together with the
//! two ends and the one constant that gives all three their depth. Every
//! triplet is complete: earlier only the ends in use were declared, and
//! completing them is what let the hand-written signatures elsewhere be
//! replaced.
//! Keeping the triplet in one place is the point: the depth is a concurrency
//! invariant, because it decides where back-pressure applies and when a send
//! blocks a task, and it used to be written once for the channel and again for
//! each end.
//!
//! Two groups still spell their Embassy types out by hand, both deliberately.
//! The lease channels in `net` are self-contained, with no constant here to
//! diverge from. `net::loco_client`, `net::provisioning::ap` and
//! `fault_manager::power_fence` compile for the host as well, while
//! `runtime_channels` is target-only, so using these aliases there would mean
//! widening that gate — a decision with its own consequences, not a detail of
//! this module.
//!
//! They sit in a submodule rather than at the root of `dcc` because they name
//! Embassy types, and the protocol module's namespace is kept free of
//! async-runtime detail.

use crate::dcc::{
    DccFrame, LocoRequestMessage, LocoResponse, PomRailcomResult, PomRequest, PomResponse,
    PomTxStarted, PowerGeneration, SchedulerCommand,
};
use crate::runtime_channels::{RuntimeChannel, RuntimeReceiver, RuntimeSender};

/// Encoded frames buffered ahead of the RMT engine, so the scheduler can run
/// far enough ahead that a scheduling hiccup never starves the waveform.
const DCC_FRAME_DEPTH: usize = 16;
/// Commands queued for the packet scheduler. Sized for the burst the network
/// task can produce in one Z21 datagram without being made to wait.
const SCHEDULER_COMMAND_DEPTH: usize = 32;
/// Slot for a correlated exchange: one message in flight by construction, so
/// the sender and the receiver meet rather than queue.
const SINGLE_FLIGHT_DEPTH: usize = 1;
/// Programming-on-main notifications. A read burst is four packets, and each
/// can produce one notification before the actor drains them.
const POM_NOTIFICATION_DEPTH: usize = 4;
/// The power fence is a one-way rendezvous, not a correlated exchange: the ack
/// must be deliverable while the requester is already waiting for it.
const POWER_FENCE_DEPTH: usize = 1;

pub(crate) type DccPacketChannel = RuntimeChannel<DccFrame, DCC_FRAME_DEPTH>;
pub(crate) type DccFrameSender = RuntimeSender<DccFrame, DCC_FRAME_DEPTH>;
pub(crate) type DccFrameReceiver = RuntimeReceiver<DccFrame, DCC_FRAME_DEPTH>;

pub(crate) type SchedulerCommandChannel = RuntimeChannel<SchedulerCommand, SCHEDULER_COMMAND_DEPTH>;
pub(crate) type SchedulerCommandSender = RuntimeSender<SchedulerCommand, SCHEDULER_COMMAND_DEPTH>;
pub(crate) type SchedulerCommandReceiver =
    RuntimeReceiver<SchedulerCommand, SCHEDULER_COMMAND_DEPTH>;

pub(crate) type LocoRequestChannel = RuntimeChannel<LocoRequestMessage, SINGLE_FLIGHT_DEPTH>;
pub(crate) type LocoRequestSender = RuntimeSender<LocoRequestMessage, SINGLE_FLIGHT_DEPTH>;
pub(crate) type LocoRequestReceiver = RuntimeReceiver<LocoRequestMessage, SINGLE_FLIGHT_DEPTH>;

pub(crate) type LocoResponseChannel = RuntimeChannel<LocoResponse, SINGLE_FLIGHT_DEPTH>;
pub(crate) type LocoResponseSender = RuntimeSender<LocoResponse, SINGLE_FLIGHT_DEPTH>;
pub(crate) type LocoResponseReceiver = RuntimeReceiver<LocoResponse, SINGLE_FLIGHT_DEPTH>;

pub(crate) type PomRequestChannel = RuntimeChannel<PomRequest, SINGLE_FLIGHT_DEPTH>;
pub(crate) type PomRequestSender = RuntimeSender<PomRequest, SINGLE_FLIGHT_DEPTH>;
pub(crate) type PomRequestReceiver = RuntimeReceiver<PomRequest, SINGLE_FLIGHT_DEPTH>;

pub(crate) type PomResponseChannel = RuntimeChannel<PomResponse, SINGLE_FLIGHT_DEPTH>;
pub(crate) type PomResponseSender = RuntimeSender<PomResponse, SINGLE_FLIGHT_DEPTH>;
pub(crate) type PomResponseReceiver = RuntimeReceiver<PomResponse, SINGLE_FLIGHT_DEPTH>;

pub(crate) type PomTxStartedChannel = RuntimeChannel<PomTxStarted, POM_NOTIFICATION_DEPTH>;
pub(crate) type PomTxStartedSender = RuntimeSender<PomTxStarted, POM_NOTIFICATION_DEPTH>;
pub(crate) type PomTxStartedReceiver = RuntimeReceiver<PomTxStarted, POM_NOTIFICATION_DEPTH>;

pub(crate) type PomRailcomResultChannel = RuntimeChannel<PomRailcomResult, POM_NOTIFICATION_DEPTH>;
pub(crate) type PomRailcomResultSender = RuntimeSender<PomRailcomResult, POM_NOTIFICATION_DEPTH>;
pub(crate) type PomRailcomResultReceiver =
    RuntimeReceiver<PomRailcomResult, POM_NOTIFICATION_DEPTH>;

/// Power generation fence: the engine acknowledges, the scheduler waits.
pub(crate) type PowerFenceAckChannel = RuntimeChannel<PowerGeneration, POWER_FENCE_DEPTH>;
pub(crate) type PowerFenceAckSender = RuntimeSender<PowerGeneration, POWER_FENCE_DEPTH>;
pub(crate) type PowerFenceAckReceiver = RuntimeReceiver<PowerGeneration, POWER_FENCE_DEPTH>;
pub(crate) type PowerQuiesceChannel = RuntimeChannel<PowerGeneration, POWER_FENCE_DEPTH>;
pub(crate) type PowerQuiesceSender = RuntimeSender<PowerGeneration, POWER_FENCE_DEPTH>;
pub(crate) type PowerQuiesceReceiver = RuntimeReceiver<PowerGeneration, POWER_FENCE_DEPTH>;
