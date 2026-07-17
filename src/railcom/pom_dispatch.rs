//! RailCom-to-POM dispatch glue.
//!
//! Keeps app:pom attribution state next to the RailCom receive path instead of
//! in boot orchestration. The DCC CV actor owns request/response policy; this
//! module only decides whether a RailCom CH2 window belongs to the in-flight
//! POM exchange and forwards matching results.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;

use crate::cutout::CutoutMode;
use crate::dcc::{PomRailcomResult, PomTxStarted, pom_result_from_railcom_items};
use crate::railcom::pipeline::{PacketSequence, RailcomChannel};
use crate::railcom_data::RailcomItem;
use crate::track_output::{CutoutRuntimeEvent, RailcomPacketMetadata};

#[must_use]
pub fn evaluate_pom_window(
    packet_sequence: PacketSequence,
    channel: RailcomChannel,
    packet_metadata: Option<RailcomPacketMetadata>,
    items: &[RailcomItem],
) -> Option<PomRailcomResult> {
    if !matches!(channel, RailcomChannel::Channel2) {
        return None;
    }

    let metadata = packet_metadata?;
    let request_id = metadata.pom_request_id?;
    if matches!(metadata.cutout, CutoutMode::None | CutoutMode::Telemetry) {
        return None;
    }
    let include_ack = matches!(metadata.cutout, CutoutMode::PomWrite);
    pom_result_from_railcom_items(
        request_id,
        packet_sequence,
        metadata.target_address,
        include_ack,
        items,
    )
}

#[embassy_executor::task]
pub async fn pom_cutout_monitor_task(
    pom_tx_started_sender: Sender<'static, CriticalSectionRawMutex, PomTxStarted, 4>,
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
                        cutout,
                        pom_request_id,
                        ..
                    } => {
                        if !matches!(cutout, CutoutMode::PomWrite | CutoutMode::PomRead) {
                            continue;
                        }
                        let Some(request_id) = pom_request_id else {
                            continue;
                        };
                        if pom_tx_started_sender
                            .try_send(PomTxStarted {
                                request_id,
                                packet_sequence,
                            })
                            .is_err()
                        {
                            defmt::warn!(
                                "railcom pom tx-start notify dropped: packet={}",
                                packet_sequence.value(),
                            );
                        }
                    }
                }
            }
        }
    }
}
