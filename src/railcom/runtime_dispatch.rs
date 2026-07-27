//! Runtime routing for parsed RailCom UART windows.
//!
//! The boot layer wires UART hardware and channels; this module owns the
//! application-level routing of decoded RailCom windows to loco tracking,
//! logon handling, and POM response attribution.

use defmt::{info, warn};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};

use crate::dcc::{DccPacket, PomRailcomResult, SchedulerCommand};
use crate::railcom::loco_tracker::RailcomLocoSighting;
use crate::railcom::parser::{RailcomLogonResponse, parse_logon_response_48};
use crate::railcom::pipeline::{
    PacketSequence, RailcomChannel, RailcomRxResult, record_pom_result_dropped,
    record_pom_result_forwarded,
};
use crate::railcom::pom_dispatch::evaluate_pom_window;
use crate::railcom::uart_reader::RailcomRxOutput;
use crate::railcom_data::RailcomItem;
use crate::track_output::RailcomPacketMetadata;

#[derive(Default)]
struct LogonDispatchState {
    pending_channel1: Option<(PacketSequence, [u8; 2])>,
    select_sent_for: Option<(u16, u32)>,
}

impl LogonDispatchState {
    fn observe_window(&mut self, result: &RailcomRxResult) -> Option<RailcomLogonResponse> {
        match result.window.channel {
            RailcomChannel::Channel1 if result.window.raw_len() == 2 => {
                let raw = result.window.raw_slice();
                self.pending_channel1 = Some((result.window.packet_sequence, [raw[0], raw[1]]));
                None
            }
            RailcomChannel::Channel2 => {
                let (packet_sequence, channel1_raw) = self.pending_channel1?;
                if packet_sequence != result.window.packet_sequence {
                    return None;
                }

                self.pending_channel1 = None;
                parse_logon_response_48(&channel1_raw, result.window.raw_slice()).ok()
            }
            RailcomChannel::Channel1 => None,
        }
    }

    fn dispatch_response(
        &mut self,
        packet_sequence: PacketSequence,
        response: RailcomLogonResponse,
        scheduler_sender: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
    ) {
        match response {
            RailcomLogonResponse::DecoderId(logon_id) => {
                let selected = (logon_id.manufacturer_id, logon_id.decoder_id);
                if self.select_sent_for == Some(selected) {
                    return;
                }

                info!(
                    "railcom logon id: manufacturer={} decoder_id={} packet={}",
                    logon_id.manufacturer_id, logon_id.decoder_id, packet_sequence,
                );
                let packet = DccPacket::LogonSelect {
                    manufacturer_id: logon_id.manufacturer_id,
                    decoder_id: logon_id.decoder_id,
                    subcommand: 0xff,
                };
                if scheduler_sender
                    .try_send(SchedulerCommand::RailcomTelemetry { packet })
                    .is_ok()
                {
                    self.select_sent_for = Some(selected);
                    info!(
                        "railcom logon select queued: manufacturer={} decoder_id={}",
                        logon_id.manufacturer_id, logon_id.decoder_id,
                    );
                } else {
                    warn!("railcom logon select: scheduler channel full");
                }
            }
            RailcomLogonResponse::Select(logon_select) => {
                let sighting = crate::railcom::loco_tracker::record_identified_address(
                    packet_sequence,
                    logon_select.address,
                );
                log_first_sighting("railcom logon address", sighting);
            }
        }
    }
}

fn log_first_sighting(prefix: &str, sighting: RailcomLocoSighting) {
    // Repeats are hot-path telemetry covered by aggregate counters; only the
    // first sighting is a state transition worth logging.
    if sighting.seen_count == 1 {
        info!(
            "{}: addr={} kind={:?} packet={}",
            prefix,
            sighting.address.value(),
            sighting.address.kind(),
            sighting.packet_sequence,
        );
    }
}

fn record_loco_sighting(
    result: &RailcomRxResult,
    packet_metadata: Option<RailcomPacketMetadata>,
    complete_items: &[RailcomItem],
) {
    let direct_sighting = crate::railcom::loco_tracker::record_loco_identification(
        result.window.packet_sequence,
        complete_items,
    );
    let target_sighting = direct_sighting.or_else(|| {
        packet_metadata
            .and_then(|metadata| metadata.target_address)
            .and_then(|target| {
                crate::railcom::loco_tracker::record_target_from_matching_address_fragment(
                    result.window.packet_sequence,
                    target,
                    complete_items,
                )
            })
    });

    if let Some(sighting) = target_sighting {
        log_first_sighting("railcom loco identified", sighting);
    }
}

fn forward_pom_result(
    result: &RailcomRxResult,
    packet_metadata: Option<RailcomPacketMetadata>,
    complete_items: &[RailcomItem],
    pom_result_sender: Sender<'static, CriticalSectionRawMutex, PomRailcomResult, 4>,
) {
    let Some(pom_result) = evaluate_pom_window(
        result.window.packet_sequence,
        result.window.channel,
        packet_metadata,
        complete_items,
    ) else {
        return;
    };

    if pom_result_sender.try_send(pom_result).is_ok() {
        record_pom_result_forwarded();
    } else {
        record_pom_result_dropped();
    }
}

fn dispatch_processed_window(
    state: &mut LogonDispatchState,
    result: &RailcomRxResult,
    pom_result_sender: Sender<'static, CriticalSectionRawMutex, PomRailcomResult, 4>,
    scheduler_sender: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
) {
    let complete_items = result.complete_items();
    let packet_metadata =
        crate::track_output::railcom_packet_metadata(result.window.packet_sequence);

    record_loco_sighting(result, packet_metadata, complete_items);

    if let Some(response) = state.observe_window(result) {
        state.dispatch_response(result.window.packet_sequence, response, scheduler_sender);
    }

    forward_pom_result(result, packet_metadata, complete_items, pom_result_sender);
}

#[embassy_executor::task]
pub async fn railcom_uart_runtime_dispatch_task(
    receiver: Receiver<'static, CriticalSectionRawMutex, RailcomRxOutput, 8>,
    pom_result_sender: Sender<'static, CriticalSectionRawMutex, PomRailcomResult, 4>,
    scheduler_sender: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
) -> ! {
    let mut state = LogonDispatchState::default();

    loop {
        match receiver.receive().await {
            RailcomRxOutput::WindowProcessed(result) => {
                dispatch_processed_window(&mut state, &result, pom_result_sender, scheduler_sender);
            }
            RailcomRxOutput::WindowError(err) => {
                warn!("railcom rx window assembly: {:?}", err);
            }
        }
    }
}
