//! Programming on Main runtime transport.
//!
//! This module owns the single-flight POM actor and RailCom response matching
//! used by the Z21 network path.

#[cfg(target_arch = "riscv32")]
use crate::dcc::packet::DccPacket;
use crate::dcc::packet::{DccAddress, PomCv};
#[cfg(target_arch = "riscv32")]
use crate::dcc::scheduler::SchedulerCommand;
use crate::railcom::parser::{RailcomDatagram, RailcomItem};

#[cfg(target_arch = "riscv32")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(target_arch = "riscv32")]
use embassy_sync::channel::{Receiver, Sender};
#[cfg(target_arch = "riscv32")]
use embassy_time::{Duration, with_timeout};

#[cfg(target_arch = "riscv32")]
// The scheduler-to-engine path is naturally backpressured by a small frame
// queue. Under app traffic the POM command can be accepted immediately but its
// RailCom cutout event may be observed after more than one queued DCC frame.
const POM_TX_START_TIMEOUT: Duration = Duration::from_millis(500);
#[cfg(target_arch = "riscv32")]
// Keep one app:pom attribution window open long enough for decoders that emit
// the RailCom CV data on a later same-address packet. Returning NACK too early
// lets the Z21 app send the next CV request, which can clear the decoder's
// pending app:pom response.
const POM_RESPONSE_TIMEOUT: Duration = Duration::from_millis(1_500);
#[cfg(target_arch = "riscv32")]
const POM_READ_PACKET_REPETITIONS: u8 = 4;
// All repetitions are enqueued in one burst (see `run_pom_attempt`) so they
// reach the wire consecutively per NMRA S-9.2.1. The scheduler's
// `pending_pom` queue must be large enough to hold the full burst, otherwise
// the tail packets get dropped silently.
#[cfg(target_arch = "riscv32")]
const _: () = assert!(
    POM_READ_PACKET_REPETITIONS as usize <= crate::dcc::scheduler::PENDING_POM_CAPACITY,
    "POM_READ_PACKET_REPETITIONS must fit in scheduler::pending_pom"
);
/// Accept app:pom feedback that appears on packets shortly after the POM
/// command. ZIMO's reference decoder may return app:pom on a following packet
/// to the same address, not only on the exact CV-access packet cutout.
#[cfg(any(test, target_arch = "riscv32"))]
const POM_RESPONSE_SEQUENCE_WINDOW: u32 = 64;

/// Identifies one POM request/response round-trip on the single-slot request
/// and response channels between the Z21 network task and the POM actor.
///
/// A newtype (rather than a bare `u32`) keeps request/response matching from
/// being confused with unrelated `u32` values (CV numbers, packet sequence
/// counters) flowing through the same call sites.
#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct PomRequestId(u32);

#[cfg(any(test, target_arch = "riscv32"))]
impl PomRequestId {
    #[must_use]
    pub const fn new(value: u32) -> Self {
        Self(value)
    }
}

#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum PomRequest {
    Read {
        request_id: PomRequestId,
        address: DccAddress,
        cv: PomCv,
    },
    Write {
        request_id: PomRequestId,
        address: DccAddress,
        cv: PomCv,
        value: u8,
    },
}

#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum PomResponse {
    Value { request_id: PomRequestId, value: u8 },
    Ack { request_id: PomRequestId },
    Nack { request_id: PomRequestId },
}

#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum PomRailcomResult {
    Window {
        packet_sequence: u32,
        target_address: Option<DccAddress>,
        value: Option<u8>,
        ack: bool,
        nack: bool,
    },
}

#[cfg(target_arch = "riscv32")]
pub type PomRequestChannel = embassy_sync::channel::Channel<CriticalSectionRawMutex, PomRequest, 1>;
#[cfg(target_arch = "riscv32")]
pub type PomResponseChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, PomResponse, 1>;
#[cfg(target_arch = "riscv32")]
pub type PomTxStartedChannel = embassy_sync::channel::Channel<CriticalSectionRawMutex, u32, 4>;
#[cfg(target_arch = "riscv32")]
pub type PomRailcomResultChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, PomRailcomResult, 4>;

#[cfg(target_arch = "riscv32")]
fn pom_packet_from_request(request: PomRequest) -> DccPacket {
    match request {
        PomRequest::Read { address, cv, .. } => DccPacket::PomReadByte { address, cv },
        PomRequest::Write {
            address, cv, value, ..
        } => DccPacket::PomWriteByte { address, cv, value },
    }
}

/// Drain any pending items from a `Receiver` without awaiting.
///
/// Used before dispatching a new request on a single-slot channel to make sure
/// the next response the caller observes is actually for its new request, not
/// a stale leftover. `Copy` keeps the helper allocation-free for the small POM
/// types that flow through these channels.
#[cfg(target_arch = "riscv32")]
pub(crate) fn drain_channel<T: Copy, const N: usize>(
    receiver: &Receiver<'static, CriticalSectionRawMutex, T, N>,
) {
    while receiver.try_receive().is_ok() {}
}

#[cfg(any(test, target_arch = "riscv32"))]
fn match_pom_result(
    request: PomRequest,
    packet_sequence: u32,
    result: PomRailcomResult,
) -> Option<PomResponse> {
    let PomRailcomResult::Window {
        packet_sequence: seq,
        target_address,
        value,
        ack,
        nack,
        ..
    } = result;
    let same_packet = seq == packet_sequence;
    let same_target = match request {
        PomRequest::Read { address, .. } | PomRequest::Write { address, .. } => {
            target_address == Some(address)
        }
    };
    if !same_packet
        && (!same_target || seq.wrapping_sub(packet_sequence) > POM_RESPONSE_SEQUENCE_WINDOW)
    {
        return None;
    }
    match request {
        PomRequest::Read { request_id, .. } => {
            if let Some(value) = value {
                Some(PomResponse::Value { request_id, value })
            } else if nack {
                Some(PomResponse::Nack { request_id })
            } else {
                None
            }
        }
        PomRequest::Write { request_id, .. } => {
            if ack {
                Some(PomResponse::Ack { request_id })
            } else if nack {
                Some(PomResponse::Nack { request_id })
            } else {
                None
            }
        }
    }
}

#[cfg(target_arch = "riscv32")]
async fn await_matching_pom_result(
    request: PomRequest,
    packet_sequence: u32,
    railcom_results: &Receiver<'static, CriticalSectionRawMutex, PomRailcomResult, 4>,
) -> PomResponse {
    loop {
        let result = railcom_results.receive().await;
        if let Some(response) = match_pom_result(request, packet_sequence, result) {
            return response;
        }
    }
}

#[cfg(target_arch = "riscv32")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PomAttemptOutcome {
    Response(PomResponse),
    TxTimeout,
    ResponseTimeout,
}

#[cfg(target_arch = "riscv32")]
async fn run_pom_attempt(
    request: PomRequest,
    tx_started_receiver: &Receiver<'static, CriticalSectionRawMutex, u32, 4>,
    railcom_result_receiver: &Receiver<'static, CriticalSectionRawMutex, PomRailcomResult, 4>,
    scheduler_sender: &Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
) -> PomAttemptOutcome {
    drain_channel(tx_started_receiver);
    drain_channel(railcom_result_receiver);

    let packet = pom_packet_from_request(request);

    // Burst-enqueue all CV-access repetitions before awaiting any tx-start.
    //
    // NMRA S-9.2.1 §3.5.1 requires CV access instructions to be sent as a
    // sequence of identical packets so the decoder commits the access only
    // after seeing the same instruction repeated back-to-back. Awaiting the
    // first cutout between sends would let the scheduler interleave cyclic
    // refresh packets (the RMT feeder queue holds ~16 frames), pushing the
    // repetitions ~150 ms apart and breaking the consecutive-pair rule -
    // observed in practice with ESU/Trix/Roco decoders that then never
    // emit `app:pom` on CH2.
    //
    // `pending_pom` capacity is `POM_READ_PACKET_REPETITIONS`, so the burst
    // fits without dropping. The scheduler serves `pending_pom` ahead of any
    // refresh (priority `Programming`), keeping the packets consecutive on
    // the wire.
    let repetitions = match request {
        PomRequest::Read { .. } => POM_READ_PACKET_REPETITIONS,
        // NMRA S-9.2.1 §3.5.1 also calls for repeated write packets; ZIMO's
        // reference command station emits two. Keep that minimum.
        PomRequest::Write { .. } => 2,
    };
    for _ in 0..repetitions {
        scheduler_sender
            .send(SchedulerCommand::ProgramOnMain { packet })
            .await;
    }

    // Anchor on the first tx-start (the burst's first cutout). Subsequent
    // tx-starts arrive in the channel and will be drained by the next attempt.
    let packet_sequence =
        match with_timeout(POM_TX_START_TIMEOUT, tx_started_receiver.receive()).await {
            Ok(packet_sequence) => packet_sequence,
            Err(_) => return PomAttemptOutcome::TxTimeout,
        };

    match with_timeout(
        POM_RESPONSE_TIMEOUT,
        await_matching_pom_result(request, packet_sequence, railcom_result_receiver),
    )
    .await
    {
        Ok(response) => PomAttemptOutcome::Response(response),
        Err(_) => PomAttemptOutcome::ResponseTimeout,
    }
}

/// Single-flight POM actor.
///
/// This actor keeps retry/timeout orchestration outside the scheduler and RMT path.
#[cfg(target_arch = "riscv32")]
#[embassy_executor::task]
pub async fn pom_actor_task(
    request_receiver: Receiver<'static, CriticalSectionRawMutex, PomRequest, 1>,
    response_sender: Sender<'static, CriticalSectionRawMutex, PomResponse, 1>,
    tx_started_receiver: Receiver<'static, CriticalSectionRawMutex, u32, 4>,
    railcom_result_receiver: Receiver<'static, CriticalSectionRawMutex, PomRailcomResult, 4>,
    scheduler_sender: Sender<'static, CriticalSectionRawMutex, SchedulerCommand, 32>,
) -> ! {
    loop {
        let request = request_receiver.receive().await;
        let request_id = match request {
            PomRequest::Read { request_id, .. } | PomRequest::Write { request_id, .. } => {
                request_id
            }
        };
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
                defmt::warn!("POM request timed out before tx-start");
                PomResponse::Nack { request_id }
            }
            PomAttemptOutcome::ResponseTimeout => {
                defmt::warn!("POM request timed out waiting for RailCom CV data");
                PomResponse::Nack { request_id }
            }
        };

        response_sender.send(final_response).await;
    }
}

#[cfg(any(test, target_arch = "riscv32"))]
pub fn pom_result_from_railcom_items(
    packet_sequence: u32,
    target_address: Option<DccAddress>,
    include_ack: bool,
    items: &[RailcomItem],
) -> Option<PomRailcomResult> {
    let mut value = None;
    let mut ack = false;
    let mut nack = false;

    for item in items {
        match item {
            RailcomItem::Ack => ack = include_ack,
            RailcomItem::Nack => nack = true,
            RailcomItem::Datagram(RailcomDatagram::CvData(cv_value)) => {
                value.get_or_insert(*cv_value);
            }
            RailcomItem::Datagram(_) => {}
        }
    }

    (value.is_some() || ack || nack).then_some(PomRailcomResult::Window {
        packet_sequence,
        target_address,
        value,
        ack,
        nack,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pom_cv(cv: u16) -> PomCv {
        PomCv::new(cv).expect("test POM CV must be valid")
    }

    fn pom_id(value: u32) -> PomRequestId {
        PomRequestId::new(value)
    }

    #[test]
    fn test_pom_result_prefers_value_when_ack_and_cv_data_are_both_present() {
        let items = [
            RailcomItem::Ack,
            RailcomItem::Datagram(RailcomDatagram::CvData(0x42)),
        ];

        assert_eq!(
            pom_result_from_railcom_items(17, DccAddress::new_short(3), true, &items),
            Some(PomRailcomResult::Window {
                packet_sequence: 17,
                target_address: DccAddress::new_short(3),
                value: Some(0x42),
                ack: true,
                nack: false,
            })
        );
    }

    #[test]
    fn test_pom_result_maps_cv_data_to_value() {
        let items = [RailcomItem::Datagram(RailcomDatagram::CvData(0x55))];

        assert_eq!(
            pom_result_from_railcom_items(23, DccAddress::new_short(3), false, &items),
            Some(PomRailcomResult::Window {
                packet_sequence: 23,
                target_address: DccAddress::new_short(3),
                value: Some(0x55),
                ack: false,
                nack: false,
            })
        );
    }

    #[test]
    fn test_pom_result_ignores_unrelated_datagrams() {
        let items = [RailcomItem::Datagram(RailcomDatagram::Dyn {
            value: 0x12,
            sub_index: 0,
        })];

        assert_eq!(
            pom_result_from_railcom_items(99, DccAddress::new_short(3), true, &items),
            None
        );
    }

    #[test]
    fn test_pom_read_ignores_ack_only_feedback() {
        let items = [RailcomItem::Ack];

        assert_eq!(
            pom_result_from_railcom_items(101, DccAddress::new_short(3), false, &items),
            None
        );
    }

    #[test]
    fn test_pom_read_accepts_followup_packet_value() {
        let request = PomRequest::Read {
            request_id: pom_id(7),
            address: DccAddress::new_short(3).unwrap(),
            cv: pom_cv(8),
        };
        let result = PomRailcomResult::Window {
            packet_sequence: 42 + 3,
            target_address: DccAddress::new_short(3),
            value: Some(151),
            ack: false,
            nack: false,
        };

        assert_eq!(
            match_pom_result(request, 42, result),
            Some(PomResponse::Value {
                request_id: pom_id(7),
                value: 151,
            })
        );
    }

    #[test]
    fn test_pom_write_accepts_matching_ack() {
        let request = PomRequest::Write {
            request_id: pom_id(11),
            address: DccAddress::new_short(3).unwrap(),
            cv: pom_cv(29),
            value: 6,
        };
        let result = PomRailcomResult::Window {
            packet_sequence: 50,
            target_address: DccAddress::new_short(3),
            value: None,
            ack: true,
            nack: false,
        };

        assert_eq!(
            match_pom_result(request, 50, result),
            Some(PomResponse::Ack {
                request_id: pom_id(11)
            })
        );
    }

    #[test]
    fn test_pom_read_rejects_stale_followup_packet_value() {
        let request = PomRequest::Read {
            request_id: pom_id(7),
            address: DccAddress::new_short(3).unwrap(),
            cv: pom_cv(8),
        };
        let result = PomRailcomResult::Window {
            packet_sequence: 42 + POM_RESPONSE_SEQUENCE_WINDOW + 1,
            target_address: DccAddress::new_short(3),
            value: Some(151),
            ack: false,
            nack: false,
        };

        assert_eq!(match_pom_result(request, 42, result), None);
    }

    #[test]
    fn test_pom_read_rejects_followup_packet_from_other_loco() {
        let request = PomRequest::Read {
            request_id: pom_id(7),
            address: DccAddress::new_short(3).unwrap(),
            cv: pom_cv(8),
        };
        let result = PomRailcomResult::Window {
            packet_sequence: 42 + 3,
            target_address: DccAddress::new_short(4),
            value: Some(151),
            ack: false,
            nack: false,
        };

        assert_eq!(match_pom_result(request, 42, result), None);
    }
}
