//! Embassy transport aliases for the messages defined at the root of `dcc`,
//! and the depth shared by every single-flight exchange.
//!
//! Every channel carrying a DCC message is declared together with its two ends
//! and the one constant that gives all three their depth. The depth is a
//! concurrency invariant, because it decides where back-pressure applies and
//! when a send blocks a task, so it is written once per channel and never
//! again for each end.
//!
//! Each owner of a message type declares the channels of its messages beside
//! it: the packet scheduler in [`crate::dcc::scheduler::channels`], the POM
//! actor in [`crate::dcc::cv::channels`]. Declaring them all here made this
//! module depend on the scheduler and the POM actor while both send on its
//! channels, a cycle between the three. What stays here is the frame queue
//! and the power fence, whose messages are defined at the root of `dcc`.
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

use crate::dcc::{DccFrame, PowerGeneration};
use crate::runtime_channels::{RuntimeChannel, RuntimeReceiver, RuntimeSender};

/// Encoded frames buffered ahead of the RMT engine, so the scheduler can run
/// far enough ahead that a scheduling hiccup never starves the waveform.
const DCC_FRAME_DEPTH: usize = 16;
/// Slot for a correlated exchange: one message in flight by construction, so
/// the sender and the receiver meet rather than queue.
pub(crate) const SINGLE_FLIGHT_DEPTH: usize = 1;
/// The power fence is a one-way rendezvous, not a correlated exchange: the ack
/// must be deliverable while the requester is already waiting for it.
const POWER_FENCE_DEPTH: usize = 1;

pub(crate) type DccPacketChannel = RuntimeChannel<DccFrame, DCC_FRAME_DEPTH>;
pub(crate) type DccFrameSender = RuntimeSender<DccFrame, DCC_FRAME_DEPTH>;
pub(crate) type DccFrameReceiver = RuntimeReceiver<DccFrame, DCC_FRAME_DEPTH>;

/// Power generation fence: the engine acknowledges, the scheduler waits.
pub(crate) type PowerFenceAckChannel = RuntimeChannel<PowerGeneration, POWER_FENCE_DEPTH>;
pub(crate) type PowerFenceAckSender = RuntimeSender<PowerGeneration, POWER_FENCE_DEPTH>;
pub(crate) type PowerFenceAckReceiver = RuntimeReceiver<PowerGeneration, POWER_FENCE_DEPTH>;
pub(crate) type PowerQuiesceChannel = RuntimeChannel<PowerGeneration, POWER_FENCE_DEPTH>;
pub(crate) type PowerQuiesceSender = RuntimeSender<PowerGeneration, POWER_FENCE_DEPTH>;
pub(crate) type PowerQuiesceReceiver = RuntimeReceiver<PowerGeneration, POWER_FENCE_DEPTH>;
