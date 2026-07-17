//! Runtime routing for parsed RailCom UART windows.
//!
//! The boot layer wires UART hardware and channels; this module owns the
//! application-level routing of decoded RailCom windows to loco tracking,
//! logon handling, and POM response attribution.

use defmt::{info, warn};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};

use crate::dcc::{DccPacket, PomRailcomResult, SchedulerCommand};
use crate::railcom::parser::{RailcomLogonResponse, parse_logon_response_48};
use crate::railcom::pipeline::{
    PacketSequence, RailcomChannel, record_pom_result_dropped, record_pom_result_forwarded,
};
use crate::railcom::pom_dispatch::evaluate_pom_window;
use crate::railcom::uart_reader::RailcomRxOutput;

#[embassy_executor::task]
pub async fn railcom_uart_runtime_dispatch_task(
    receiver: Receiver<'static, CriticalSectionRawMutex, RailcomRxOutput, 8>,
    pom_result_sender: Sender<'static, CriticalSectionRawMutex, PomRailcomResult, 4>,
    scheduler_sender: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
) -> ! {
    let mut pending_logon_ch1: Option<(PacketSequence, [u8; 2])> = None;
    let mut logon_select_sent_for: Option<(u16, u32)> = None;

    loop {
        let output = receiver.receive().await;
        match output {
            RailcomRxOutput::WindowProcessed(result) => {
                let complete_items = result.complete_items();

                let direct_sighting = crate::railcom::loco_tracker::record_loco_identification(
                    result.window.packet_sequence,
                    complete_items,
                );
                let packet_metadata =
                    crate::track_output::railcom_packet_metadata(result.window.packet_sequence);
                let target_address = packet_metadata.and_then(|metadata| metadata.target_address);
                let target_sighting = direct_sighting.or_else(|| {
                    target_address.and_then(|target| {
                        crate::railcom::loco_tracker::record_target_from_matching_address_fragment(
                            result.window.packet_sequence,
                            target,
                            complete_items,
                        )
                    })
                });
                if let Some(sighting) = target_sighting {
                    // Only the first sighting is a state transition worth logging;
                    // repeats are hot-path telemetry covered by aggregate counters.
                    if sighting.seen_count == 1 {
                        info!(
                            "railcom loco identified: addr={} kind={:?} packet={}",
                            sighting.address.value(),
                            sighting.address.kind(),
                            sighting.packet_sequence,
                        );
                    }
                }

                match result.window.channel {
                    RailcomChannel::Channel1 if result.window.raw_len() == 2 => {
                        let raw = result.window.raw_slice();
                        pending_logon_ch1 = Some((result.window.packet_sequence, [raw[0], raw[1]]));
                    }
                    RailcomChannel::Channel2 => {
                        if let Some((packet_sequence, channel1_raw)) = pending_logon_ch1
                            && packet_sequence == result.window.packet_sequence
                        {
                            let response =
                                parse_logon_response_48(&channel1_raw, result.window.raw_slice());
                            pending_logon_ch1 = None;
                            if let Ok(response) = response {
                                match response {
                                    RailcomLogonResponse::DecoderId(logon_id) => {
                                        let selected =
                                            (logon_id.manufacturer_id, logon_id.decoder_id);
                                        if logon_select_sent_for != Some(selected) {
                                            info!(
                                                "railcom logon id: manufacturer={} decoder_id={} packet={}",
                                                logon_id.manufacturer_id,
                                                logon_id.decoder_id,
                                                result.window.packet_sequence,
                                            );
                                            let packet = DccPacket::LogonSelect {
                                                manufacturer_id: logon_id.manufacturer_id,
                                                decoder_id: logon_id.decoder_id,
                                                subcommand: 0xff,
                                            };
                                            if scheduler_sender
                                                .try_send(SchedulerCommand::RailcomTelemetry {
                                                    packet,
                                                })
                                                .is_ok()
                                            {
                                                logon_select_sent_for = Some(selected);
                                                info!(
                                                    "railcom logon select queued: manufacturer={} decoder_id={}",
                                                    logon_id.manufacturer_id, logon_id.decoder_id,
                                                );
                                            } else {
                                                warn!(
                                                    "railcom logon select: scheduler channel full"
                                                );
                                            }
                                        }
                                    }
                                    RailcomLogonResponse::Select(logon_select) => {
                                        let sighting =
                                            crate::railcom::loco_tracker::record_identified_address(
                                                result.window.packet_sequence,
                                                logon_select.address,
                                            );
                                        if sighting.seen_count == 1 {
                                            info!(
                                                "railcom logon address: addr={} kind={:?} packet={}",
                                                sighting.address.value(),
                                                sighting.address.kind(),
                                                sighting.packet_sequence,
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    }
                    RailcomChannel::Channel1 => {}
                }

                if let Some(pom_result) = evaluate_pom_window(
                    result.window.packet_sequence,
                    result.window.channel,
                    packet_metadata,
                    complete_items,
                ) {
                    if pom_result_sender.try_send(pom_result).is_ok() {
                        record_pom_result_forwarded();
                    } else {
                        record_pom_result_dropped();
                    }
                }
            }
            RailcomRxOutput::WindowError(err) => {
                warn!("railcom rx window assembly: {:?}", err);
            }
        }
    }
}
