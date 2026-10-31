//! Embassy transport aliases for the messages the packet scheduler receives
//! and answers.
//!
//! Each channel is declared with its two ends and the constant that gives all
//! three their depth, as in [`crate::dcc::channels`].

use crate::dcc::channels::SINGLE_FLIGHT_DEPTH;
use crate::runtime_channels::{RuntimeChannel, RuntimeReceiver, RuntimeSender};

use super::{LocoRequestMessage, LocoResponse, SchedulerCommand};

/// Commands queued for the packet scheduler. Sized for the burst the network
/// task can produce in one Z21 datagram without being made to wait.
const SCHEDULER_COMMAND_DEPTH: usize = 32;

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
