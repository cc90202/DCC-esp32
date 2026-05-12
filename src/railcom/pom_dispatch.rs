//! RailCom-to-POM dispatch glue.
//!
//! Keeps app:pom attribution state next to the RailCom receive path instead of
//! in boot orchestration. The DCC CV actor owns request/response policy; this
//! module only decides whether a RailCom CH2 window belongs to the in-flight
//! POM exchange and forwards matching results.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;

use crate::dcc::{DccAddress, PackedDccAddress, PomRailcomResult, pom_result_from_railcom_items};
use crate::railcom::parser::{RailcomDatagram, RailcomItem};
use crate::railcom::pipeline::RailcomChannel;
use crate::track_output::{CutoutRuntimeEvent, RailcomPacketMetadata};

static PENDING_POM_RAILCOM_PACKET_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static PENDING_POM_RAILCOM_METADATA: AtomicU32 = AtomicU32::new(0);
static PENDING_POM_RAILCOM_ACTIVE: AtomicBool = AtomicBool::new(false);

const PENDING_POM_SEQUENCE_WINDOW: u32 = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PendingPomRailcomMetadata(u32);

impl PendingPomRailcomMetadata {
    const ADDRESS_MASK: u32 = 0xffff;
    const READ_REQUESTED: u32 = 1 << 16;

    fn new(target_address: Option<DccAddress>, read_requested: bool) -> Self {
        let mut raw = u32::from(PackedDccAddress::from_address(target_address).raw());
        if read_requested {
            raw |= Self::READ_REQUESTED;
        }
        Self(raw)
    }

    const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }

    const fn raw(self) -> u32 {
        self.0
    }

    fn target_address(self) -> Option<DccAddress> {
        PackedDccAddress::from_raw((self.0 & Self::ADDRESS_MASK) as u16).address()
    }

    const fn read_requested(self) -> bool {
        (self.0 & Self::READ_REQUESTED) != 0
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct PomWindowEvaluation {
    pub metadata_pom: bool,
    pub pom_read_candidate: bool,
    pub target_address: Option<DccAddress>,
    pub highlight_result: bool,
    pub result: Option<PomRailcomResult>,
}

fn set_pending_pom_railcom(
    packet_sequence: u32,
    target_address: Option<DccAddress>,
    read_requested: bool,
) {
    PENDING_POM_RAILCOM_PACKET_SEQUENCE.store(packet_sequence, Ordering::Release);
    PENDING_POM_RAILCOM_METADATA.store(
        PendingPomRailcomMetadata::new(target_address, read_requested).raw(),
        Ordering::Release,
    );
    PENDING_POM_RAILCOM_ACTIVE.store(true, Ordering::Release);
}

fn pending_pom_target_address() -> Option<DccAddress> {
    PendingPomRailcomMetadata::from_raw(PENDING_POM_RAILCOM_METADATA.load(Ordering::Acquire))
        .target_address()
}

fn pending_pom_read_requested() -> bool {
    PENDING_POM_RAILCOM_ACTIVE.load(Ordering::Acquire)
        && PendingPomRailcomMetadata::from_raw(PENDING_POM_RAILCOM_METADATA.load(Ordering::Acquire))
            .read_requested()
}

fn clear_pending_pom_railcom_before(packet_sequence: u32) {
    if PENDING_POM_RAILCOM_ACTIVE.load(Ordering::Acquire) {
        let pending = PENDING_POM_RAILCOM_PACKET_SEQUENCE.load(Ordering::Acquire);
        if packet_sequence.wrapping_sub(pending) > PENDING_POM_SEQUENCE_WINDOW {
            PENDING_POM_RAILCOM_ACTIVE.store(false, Ordering::Release);
        }
    }
}

fn pending_pom_railcom_accepts(packet_sequence: u32, target_address: Option<DccAddress>) -> bool {
    if !PENDING_POM_RAILCOM_ACTIVE.load(Ordering::Acquire) {
        return false;
    }
    let pending = PENDING_POM_RAILCOM_PACKET_SEQUENCE.load(Ordering::Acquire);
    let age = packet_sequence.wrapping_sub(pending);
    age <= PENDING_POM_SEQUENCE_WINDOW
        && (packet_sequence == pending
            || (target_address.is_some() && target_address == pending_pom_target_address()))
}

fn railcom_items_contain_pom_response(items: &[RailcomItem], include_ack: bool) -> bool {
    items.iter().any(|item| match item {
        RailcomItem::Ack => include_ack,
        RailcomItem::Nack => true,
        RailcomItem::Datagram(RailcomDatagram::CvData(_)) => true,
        RailcomItem::Datagram(_) => false,
    })
}

fn should_highlight_result(result: &PomRailcomResult) -> bool {
    matches!(
        *result,
        PomRailcomResult::Window {
            value: Some(_),
            ack: false,
            nack: false,
            ..
        }
    )
}

#[must_use]
pub fn evaluate_pom_window(
    packet_sequence: u32,
    channel: RailcomChannel,
    packet_metadata: Option<RailcomPacketMetadata>,
    items: &[RailcomItem],
) -> Option<PomWindowEvaluation> {
    if !matches!(channel, RailcomChannel::Channel2) {
        return None;
    }

    let target_address = packet_metadata.and_then(|metadata| metadata.target_address);
    let metadata_pom = packet_metadata.is_some_and(|metadata| metadata.pom_requested);
    let pom_read_candidate = packet_metadata
        .map(|metadata| metadata.pom_read_requested)
        .unwrap_or(false)
        || pending_pom_read_requested();
    let include_ack = !pom_read_candidate;
    let is_pending_pom_response = pending_pom_railcom_accepts(packet_sequence, target_address)
        && railcom_items_contain_pom_response(items, include_ack);

    if !metadata_pom && !is_pending_pom_response {
        return None;
    }

    let result = pom_result_from_railcom_items(packet_sequence, target_address, include_ack, items);
    let highlight_result = result.as_ref().is_some_and(should_highlight_result);

    Some(PomWindowEvaluation {
        metadata_pom,
        pom_read_candidate,
        target_address,
        highlight_result,
        result,
    })
}

#[embassy_executor::task]
pub async fn pom_cutout_monitor_task(
    pom_tx_started_sender: Sender<'static, CriticalSectionRawMutex, u32, 4>,
) -> ! {
    let mut next_cutout_sequence_read = 0u32;

    loop {
        crate::track_output::cutout_event_notify_receiver()
            .receive()
            .await;

        loop {
            let mut events = heapless::Vec::<
                CutoutRuntimeEvent,
                { crate::track_output::CUTOUT_EVENT_RING_CAPACITY },
            >::new();
            crate::track_output::drain_cutout_runtime_events(
                &mut next_cutout_sequence_read,
                &mut events,
            );
            if events.is_empty() {
                break;
            }

            for event in events {
                match event {
                    CutoutRuntimeEvent::Opened {
                        packet_sequence,
                        pom_requested,
                        pom_read_requested,
                        target_address,
                    } => {
                        if pom_requested {
                            set_pending_pom_railcom(
                                packet_sequence,
                                target_address,
                                pom_read_requested,
                            );
                            if pom_tx_started_sender.try_send(packet_sequence).is_err() {
                                defmt::warn!(
                                    "railcom pom tx-start notify dropped: packet={}",
                                    packet_sequence,
                                );
                            }
                        } else {
                            clear_pending_pom_railcom_before(packet_sequence);
                        }
                    }
                }
            }
        }
    }
}
