//! Single-flight POM request orchestration and timeout policy.

use core::sync::atomic::Ordering;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Duration, Instant, with_timeout};

use crate::dcc::packet::DccPacket;
use crate::dcc::scheduler::SchedulerCommand;

use super::{
    POM, PomRailcomResult, PomRequest, PomResponse, PomTxStarted, drain_channel, match_pom_result,
};

// The scheduler-to-engine path is naturally backpressured by a small frame
// queue. Under app traffic the POM command can be accepted immediately but its
// RailCom cutout event may be observed after more than one queued DCC frame.
const POM_TX_START_TIMEOUT: Duration = Duration::from_millis(500);
// Keep one app:pom attribution window open long enough for decoders that emit
// the RailCom CV data on a later same-address packet. Returning NACK too early
// lets the Z21 app send the next CV request, which can clear the decoder's
// pending app:pom response.
const POM_RESPONSE_TIMEOUT: Duration = Duration::from_millis(1_500);
const POM_READ_PACKET_REPETITIONS: u8 = 4;
// Some decoders answer a POM read with ACK in the first cutout and deliver the
// CV value only after a later read packet. Re-send the burst until the value
// arrives or the overall response deadline expires.
const POM_READ_BURST_RESEND_INTERVAL: Duration = Duration::from_millis(150);
const POM_MINIMUM_TX_STARTS: u8 = 2;
const _: () = assert!(
    POM_READ_PACKET_REPETITIONS as usize <= crate::dcc::scheduler::PENDING_POM_CAPACITY,
    "POM_READ_PACKET_REPETITIONS must fit in scheduler::pending_pom"
);

fn pom_packet_from_request(request: PomRequest) -> DccPacket {
    match request {
        PomRequest::Read { address, cv, .. } => DccPacket::PomReadByte { address, cv },
        PomRequest::Write {
            address, cv, value, ..
        } => DccPacket::PomWriteByte { address, cv, value },
    }
}

async fn await_matching_pom_result(
    request: PomRequest,
    earliest_sequence: crate::cutout::PacketSequence,
    railcom_results: &Receiver<'static, CriticalSectionRawMutex, PomRailcomResult, 4>,
) -> PomResponse {
    loop {
        let result = railcom_results.receive().await;
        if result.request_id() != request.request_id()
            || !result.packet_sequence().is_at_or_after(earliest_sequence)
        {
            POM.stale_result_count.fetch_add(1, Ordering::Relaxed);
            continue;
        }
        if result.target_address() != Some(request.address()) {
            POM.wrong_target_result_count
                .fetch_add(1, Ordering::Relaxed);
            continue;
        }
        if let Some(response) = match_pom_result(request, earliest_sequence, result) {
            return response;
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PomAttemptOutcome {
    Response(PomResponse),
    TxTimeout,
    ResponseTimeout,
}

async fn run_pom_attempt(
    request: PomRequest,
    tx_started_receiver: &Receiver<'static, CriticalSectionRawMutex, PomTxStarted, 4>,
    railcom_result_receiver: &Receiver<'static, CriticalSectionRawMutex, PomRailcomResult, 4>,
    scheduler_sender: &Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
) -> PomAttemptOutcome {
    drain_channel(tx_started_receiver);
    drain_channel(railcom_result_receiver);

    let packet = pom_packet_from_request(request);
    // Enqueue the complete burst before waiting so the scheduler cannot
    // interleave cyclic refresh packets between NMRA CV-access repetitions.
    let repetitions = match request {
        PomRequest::Read { .. } => POM_READ_PACKET_REPETITIONS,
        PomRequest::Write { .. } => 2,
    };
    enqueue_pom_burst(scheduler_sender, request, packet, repetitions).await;

    // Ignore the first matching cutout: it can still contain a delayed answer
    // to the previous request for the same decoder.
    let earliest_response_sequence = match with_timeout(POM_TX_START_TIMEOUT, async {
        let mut matching_starts = 0u8;
        loop {
            let started = tx_started_receiver.receive().await;
            if started.request_id == request.request_id() {
                matching_starts += 1;
                if matching_starts == POM_MINIMUM_TX_STARTS {
                    return started.packet_sequence;
                }
            }
        }
    })
    .await
    {
        Ok(sequence) => sequence,
        Err(_) => return PomAttemptOutcome::TxTimeout,
    };

    let is_read = matches!(request, PomRequest::Read { .. });
    let deadline = Instant::now() + POM_RESPONSE_TIMEOUT;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.as_ticks() == 0 {
            return PomAttemptOutcome::ResponseTimeout;
        }
        let wait = if is_read {
            remaining.min(POM_READ_BURST_RESEND_INTERVAL)
        } else {
            remaining
        };
        if let Ok(response) = with_timeout(
            wait,
            await_matching_pom_result(request, earliest_response_sequence, railcom_result_receiver),
        )
        .await
        {
            return PomAttemptOutcome::Response(response);
        }
        if !is_read {
            return PomAttemptOutcome::ResponseTimeout;
        }
        drain_channel(tx_started_receiver);
        enqueue_pom_burst(scheduler_sender, request, packet, repetitions).await;
    }
}

async fn enqueue_pom_burst(
    scheduler_sender: &Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
    request: PomRequest,
    packet: DccPacket,
    repetitions: u8,
) {
    for _ in 0..repetitions {
        scheduler_sender
            .send(SchedulerCommand::ProgramOnMain {
                request_id: request.request_id(),
                permit: request.permit(),
                packet,
            })
            .await;
    }
}

/// Runs one POM transaction at a time and correlates its RailCom response.
#[embassy_executor::task]
pub async fn pom_actor_task(
    request_receiver: Receiver<'static, CriticalSectionRawMutex, PomRequest, 1>,
    response_sender: Sender<'static, CriticalSectionRawMutex, PomResponse, 1>,
    tx_started_receiver: Receiver<'static, CriticalSectionRawMutex, PomTxStarted, 4>,
    railcom_result_receiver: Receiver<'static, CriticalSectionRawMutex, PomRailcomResult, 4>,
    scheduler_sender: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
) -> ! {
    loop {
        let request = request_receiver.receive().await;
        let request_id = request.request_id();
        if !crate::track_authority::accepts(request.permit(), Instant::now().as_millis()) {
            response_sender.send(PomResponse::Nack { request_id }).await;
            continue;
        }
        let final_response = match run_pom_attempt(
            request,
            &tx_started_receiver,
            &railcom_result_receiver,
            &scheduler_sender,
        )
        .await
        {
            PomAttemptOutcome::Response(response) => response,
            PomAttemptOutcome::TxTimeout => {
                POM.tx_start_timeout_count.fetch_add(1, Ordering::Relaxed);
                defmt::warn!("POM request timed out before tx-start");
                PomResponse::Nack { request_id }
            }
            PomAttemptOutcome::ResponseTimeout => {
                POM.response_timeout_count.fetch_add(1, Ordering::Relaxed);
                defmt::warn!("POM request timed out waiting for RailCom CV data");
                PomResponse::Nack { request_id }
            }
        };

        scheduler_sender
            .send(SchedulerCommand::CloseProgramOnMain { request_id })
            .await;
        response_sender.send(final_response).await;
    }
}
