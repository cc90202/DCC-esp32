//! Embassy transport aliases for the messages of the POM actor.
//!
//! Each channel is declared with its two ends and the constant that gives all
//! three their depth, as in [`crate::dcc::channels`].

use crate::dcc::channels::SINGLE_FLIGHT_DEPTH;
use crate::runtime_channels::{RuntimeChannel, RuntimeReceiver, RuntimeSender};

use super::{PomRailcomResult, PomRequest, PomResponse, PomTxStarted};

/// Programming-on-main notifications. A read burst is four packets, and each
/// can produce one notification before the actor drains them.
const POM_NOTIFICATION_DEPTH: usize = 4;

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
