//! Embassy transport aliases shared by firmware task adapters.
//!
//! Channel capacities live here so task signatures cannot silently diverge
//! from the composition root that allocates them.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};

use crate::dcc::{
    DccFrame, LocoRequestMessage, LocoResponse, PomRequest, PomResponse, SchedulerCommand,
};
use crate::railcom::uart_reader::RailcomRxOutput;
use crate::system_status::{BootReadyEvent, DisplayEvent, FaultEvent, SystemStatusEvent};

pub(crate) type RuntimeChannel<T, const N: usize> = Channel<CriticalSectionRawMutex, T, N>;
pub(crate) type RuntimeSender<T, const N: usize> = Sender<'static, CriticalSectionRawMutex, T, N>;
pub(crate) type RuntimeReceiver<T, const N: usize> =
    Receiver<'static, CriticalSectionRawMutex, T, N>;

pub(crate) type SystemStatusChannel = RuntimeChannel<SystemStatusEvent, 16>;
pub(crate) type SystemStatusSender = RuntimeSender<SystemStatusEvent, 16>;
pub(crate) type SystemStatusReceiver = RuntimeReceiver<SystemStatusEvent, 16>;

/// Dedicated status channel for the net task (fan-out from system status).
pub(crate) type NetStatusChannel = RuntimeChannel<SystemStatusEvent, 8>;
pub(crate) type NetStatusReceiver = RuntimeReceiver<SystemStatusEvent, 8>;

pub(crate) type BootReadyChannel = RuntimeChannel<BootReadyEvent, 9>;
pub(crate) type BootReadySender = RuntimeSender<BootReadyEvent, 9>;
pub(crate) type BootReadyReceiver = RuntimeReceiver<BootReadyEvent, 9>;

pub(crate) type FaultEventChannel = RuntimeChannel<FaultEvent, 16>;
pub(crate) type FaultEventSender = RuntimeSender<FaultEvent, 16>;
pub(crate) type FaultEventReceiver = RuntimeReceiver<FaultEvent, 16>;

pub(crate) type DisplayChannel = RuntimeChannel<DisplayEvent, 8>;
pub(crate) type DisplaySender = RuntimeSender<DisplayEvent, 8>;
pub(crate) type DisplayReceiver = RuntimeReceiver<DisplayEvent, 8>;

pub(crate) type DccFrameSender = RuntimeSender<DccFrame, 16>;
pub(crate) type DccFrameReceiver = RuntimeReceiver<DccFrame, 16>;
pub(crate) type SchedulerCommandSender = RuntimeSender<SchedulerCommand, 32>;
pub(crate) type SchedulerCommandReceiver = RuntimeReceiver<SchedulerCommand, 32>;
pub(crate) type LocoRequestSender = RuntimeSender<LocoRequestMessage, 1>;
pub(crate) type LocoRequestReceiver = RuntimeReceiver<LocoRequestMessage, 1>;
pub(crate) type LocoResponseSender = RuntimeSender<LocoResponse, 1>;
pub(crate) type LocoResponseReceiver = RuntimeReceiver<LocoResponse, 1>;
pub(crate) type PomRequestSender = RuntimeSender<PomRequest, 1>;
pub(crate) type PomResponseReceiver = RuntimeReceiver<PomResponse, 1>;
pub(crate) type RailcomRxOutputSender = RuntimeSender<RailcomRxOutput, 8>;

/// Publishes one readiness acknowledgement and records it consistently.
pub(crate) async fn announce_ready(sender: BootReadySender, event: BootReadyEvent) {
    sender.send(event).await;
    defmt::info!("boot: ready ack from {:?}", event);
}
