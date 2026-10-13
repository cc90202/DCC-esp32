//! Programming on Main runtime transport.
//!
//! This module owns the single-flight POM actor and RailCom response matching
//! used by the Z21 network path.

#[cfg(any(test, target_arch = "riscv32"))]
use crate::cutout::PacketSequence;
#[cfg(any(test, target_arch = "riscv32"))]
use crate::dcc::packet::{DccAddress, PomCv};
#[cfg(any(test, target_arch = "riscv32"))]
use crate::railcom_data::{RailcomDatagram, RailcomItem};

use crate::diagnostics::diagnostic_counters;

#[cfg(target_arch = "riscv32")]
mod actor;
#[cfg(target_arch = "riscv32")]
pub use actor::pom_actor_task;

diagnostic_counters! {
    #[cfg(target_arch = "riscv32")]
    static POM: PomRuntimeCounters;

    /// Snapshot of POM actor timeout and attribution counters.
    ///
    pub snapshot PomRuntimeStats;

    tx_start_timeout_count,
    response_timeout_count,
    stale_result_count,
    wrong_target_result_count,
}

#[cfg(target_arch = "riscv32")]
#[must_use]
pub fn pom_runtime_stats() -> PomRuntimeStats {
    POM.snapshot()
}

/// Identifies one POM request/response round-trip on the single-slot request
/// and response channels between the Z21 network task and the POM actor.
///
/// A newtype (rather than a bare `u32`) keeps request/response matching from
/// being confused with unrelated `u32` values (CV numbers, packet sequence
/// counters) flowing through the same call sites.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
#[repr(transparent)]
pub struct PomRequestId(u32);

impl PomRequestId {
    #[must_use]
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    #[must_use]
    pub const fn value(self) -> u32 {
        self.0
    }
}

#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum PomRequest {
    Read {
        request_id: PomRequestId,
        permit: crate::authority::LeasePermit,
        address: DccAddress,
        cv: PomCv,
    },
    Write {
        request_id: PomRequestId,
        permit: crate::authority::LeasePermit,
        address: DccAddress,
        cv: PomCv,
        value: u8,
    },
}

#[cfg(target_arch = "riscv32")]
impl PomRequest {
    #[must_use]
    pub const fn request_id(self) -> PomRequestId {
        match self {
            Self::Read { request_id, .. } | Self::Write { request_id, .. } => request_id,
        }
    }

    #[must_use]
    pub const fn address(self) -> DccAddress {
        match self {
            Self::Read { address, .. } | Self::Write { address, .. } => address,
        }
    }

    #[must_use]
    pub const fn permit(self) -> crate::authority::LeasePermit {
        match self {
            Self::Read { permit, .. } | Self::Write { permit, .. } => permit,
        }
    }
}

#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum PomResponse {
    Value { request_id: PomRequestId, value: u8 },
    Ack { request_id: PomRequestId },
    Nack { request_id: PomRequestId },
}

#[cfg(target_arch = "riscv32")]
impl PomResponse {
    /// Id of the request this response answers.
    #[must_use]
    pub(crate) const fn request_id(&self) -> PomRequestId {
        match *self {
            Self::Value { request_id, .. }
            | Self::Ack { request_id }
            | Self::Nack { request_id } => request_id,
        }
    }
}

#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum PomRailcomResult {
    Window {
        request_id: PomRequestId,
        packet_sequence: PacketSequence,
        target_address: Option<DccAddress>,
        value: Option<u8>,
        ack: bool,
        nack: bool,
    },
}

#[cfg(target_arch = "riscv32")]
impl PomRailcomResult {
    const fn request_id(self) -> PomRequestId {
        match self {
            Self::Window { request_id, .. } => request_id,
        }
    }

    const fn target_address(self) -> Option<DccAddress> {
        match self {
            Self::Window { target_address, .. } => target_address,
        }
    }

    const fn packet_sequence(self) -> PacketSequence {
        match self {
            Self::Window {
                packet_sequence, ..
            } => packet_sequence,
        }
    }
}

#[cfg(target_arch = "riscv32")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct PomTxStarted {
    pub request_id: PomRequestId,
    pub packet_sequence: PacketSequence,
}

/// Drain any pending items from a `Receiver` without awaiting.
///
/// Used before dispatching a new request on a single-slot channel to make sure
/// the next response the caller observes is actually for its new request, not
/// a stale leftover. `Copy` keeps the helper allocation-free for the small POM
/// types that flow through these channels.
#[cfg(target_arch = "riscv32")]
pub(crate) fn drain_channel<T: Copy, const N: usize>(
    receiver: &crate::runtime_channels::RuntimeReceiver<T, N>,
) {
    while receiver.try_receive().is_ok() {}
}

#[cfg(any(test, target_arch = "riscv32"))]
fn match_pom_result(
    request: PomRequest,
    earliest_sequence: PacketSequence,
    result: PomRailcomResult,
) -> Option<PomResponse> {
    let PomRailcomResult::Window {
        request_id: result_request_id,
        packet_sequence,
        target_address,
        value,
        ack,
        nack,
        ..
    } = result;
    let (request_id, address) = match request {
        PomRequest::Read {
            request_id,
            address,
            ..
        }
        | PomRequest::Write {
            request_id,
            address,
            ..
        } => (request_id, address),
    };
    if result_request_id != request_id
        || target_address != Some(address)
        || !packet_sequence.is_at_or_after(earliest_sequence)
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
            // RCN-217 §§2.5, 5.2: an unsupported CV is reported as ACK
            // followed by NACK in the same CH2 response. NACK must win.
            if nack {
                Some(PomResponse::Nack { request_id })
            } else if ack {
                Some(PomResponse::Ack { request_id })
            } else {
                None
            }
        }
    }
}

/// Turns one channel-2 receive window into a POM outcome, if it carries one.
///
/// `items` must come from a window that parsed whole — in the runtime, from
/// [`RailcomRxResult::complete_items`], which yields nothing for a partial
/// parse. The rules below read the window as a complete statement: a prefix cut
/// short at an unsupported identifier could hide the very symbol that decides
/// the outcome, and would let an ACK standing before the cut speak for a
/// decoder that went on to say something else.
///
/// [`RailcomRxResult::complete_items`]: crate::railcom::pipeline::RailcomRxResult::complete_items
#[cfg(any(test, target_arch = "riscv32"))]
pub fn pom_result_from_railcom_items(
    request_id: PomRequestId,
    packet_sequence: PacketSequence,
    target_address: Option<DccAddress>,
    include_ack: bool,
    items: &[RailcomItem],
) -> Option<PomRailcomResult> {
    // RCN-217 §5.2: the app:pom answer is the first datagram of channel 2.
    // A CV datagram anywhere else is almost always a misaligned tail of a
    // longer datagram after a lost byte (bench, 2026-09-10: a dynamic-data
    // window missing one symbol decoded as "CV1 = 0"), so it is not trusted.
    let value = match items.first() {
        Some(RailcomItem::Datagram(RailcomDatagram::CvData(cv_value))) => Some(*cv_value),
        _ => None,
    };
    let mut ack = false;
    let mut nack = false;
    let mut busy = false;

    for item in items {
        match item {
            RailcomItem::Ack => ack = include_ack,
            RailcomItem::Nack => nack = true,
            RailcomItem::Busy => busy = true,
            RailcomItem::Datagram(_) => {}
        }
    }

    // NMRA S-9.3.2: a busy decoder has not answered, so it cannot have agreed
    // or refused either. Busy therefore suppresses both verdicts and leaves the
    // request unresolved: a read repeats its burst until the deadline, a write
    // waits the deadline out. Reporting a write as accepted on the strength of
    // an ACK standing beside a BUSY would claim an answer the decoder said it
    // could not give. A value is different: a CV datagram is data that did
    // arrive, and it stands whatever follows it.
    if busy {
        ack = false;
        nack = false;
    }

    (value.is_some() || ack || nack).then_some(PomRailcomResult::Window {
        request_id,
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
    use crate::railcom::parser::{ACK_1_CODE, BUSY_CODE, NACK_CODE, parse_channel2};

    fn permit() -> crate::authority::LeasePermit {
        crate::authority::LeasePermit::new(crate::authority::LeaseEpoch::new(1), u64::MAX)
    }

    fn pom_cv(cv: u16) -> PomCv {
        PomCv::new(cv).expect("test POM CV must be valid")
    }

    fn pom_id(value: u32) -> PomRequestId {
        PomRequestId::new(value)
    }

    fn seq(value: u32) -> PacketSequence {
        PacketSequence::new(value)
    }

    #[test]
    fn test_pom_result_prefers_value_when_ack_and_cv_data_are_both_present() {
        let items = [
            RailcomItem::Datagram(RailcomDatagram::CvData(0x42)),
            RailcomItem::Ack,
        ];

        assert_eq!(
            pom_result_from_railcom_items(
                pom_id(1),
                seq(17),
                DccAddress::new_short(3),
                true,
                &items,
            ),
            Some(PomRailcomResult::Window {
                request_id: pom_id(1),
                packet_sequence: seq(17),
                target_address: DccAddress::new_short(3),
                value: Some(0x42),
                ack: true,
                nack: false,
            })
        );
    }

    #[test]
    fn test_pom_result_ignores_cv_data_that_does_not_open_the_window() {
        // A dynamic-data window that lost one symbol re-aligns into a bogus
        // trailing CV datagram; it must not become a value.
        let items = [
            RailcomItem::Datagram(RailcomDatagram::Dyn {
                value: 0,
                sub_index: 0,
            }),
            RailcomItem::Datagram(RailcomDatagram::CvData(0x00)),
        ];

        assert_eq!(
            pom_result_from_railcom_items(
                pom_id(1),
                seq(17),
                DccAddress::new_short(3),
                true,
                &items,
            ),
            None
        );
    }

    #[test]
    fn test_pom_result_maps_cv_data_to_value() {
        let items = [RailcomItem::Datagram(RailcomDatagram::CvData(0x55))];

        assert_eq!(
            pom_result_from_railcom_items(
                pom_id(2),
                seq(23),
                DccAddress::new_short(3),
                false,
                &items,
            ),
            Some(PomRailcomResult::Window {
                request_id: pom_id(2),
                packet_sequence: seq(23),
                target_address: DccAddress::new_short(3),
                value: Some(0x55),
                ack: false,
                nack: false,
            })
        );
    }

    #[test]
    fn test_pom_result_survives_a_short_ext_behind_the_cv_answer() {
        // End to end from the wire, because this is where the short `app:ext`
        // costs data: the whole window used to fail as a truncated datagram,
        // taking the CV answer in front of it down with it.
        let window = [
            0xa9, 0xc5, // app:pom, CV = 0xab
            0x8d, 0x66, // app:ext in two symbols instead of three
            0x0f, // the ACK padding RCN-217 allows
        ];
        let parsed =
            parse_channel2(&window).expect("the window must parse despite the short app:ext");

        assert_eq!(
            pom_result_from_railcom_items(
                pom_id(7),
                seq(42),
                DccAddress::new_short(3),
                true,
                &parsed.items,
            ),
            Some(PomRailcomResult::Window {
                request_id: pom_id(7),
                packet_sequence: seq(42),
                target_address: DccAddress::new_short(3),
                value: Some(0xab),
                ack: true,
                nack: false,
            })
        );
    }

    #[test]
    fn test_pom_result_leaves_a_read_pending_when_the_decoder_is_busy() {
        // A busy decoder has not refused, so the read must stay open for the
        // retry rather than resolve as a NACK. End to end from the wire,
        // because the window used to fail as a reserved code.
        let parsed =
            parse_channel2(&[BUSY_CODE]).expect("busy must parse as a reply, not as corruption");

        assert_eq!(parsed.items.as_slice(), &[RailcomItem::Busy]);
        assert_eq!(
            pom_result_from_railcom_items(
                pom_id(9),
                seq(11),
                DccAddress::new_short(3),
                false,
                &parsed.items,
            ),
            None
        );
    }

    #[test]
    fn test_pom_result_survives_a_busy_behind_the_cv_answer() {
        // The window the fix is for: the busy reply used to read as a reserved
        // code and take the CV answer in front of it down with it.
        let window = [
            0xa9, 0xc5, // app:pom, CV = 0xab
            BUSY_CODE,
        ];
        let parsed =
            parse_channel2(&window).expect("the cv answer must survive a busy reply behind it");

        assert_eq!(
            pom_result_from_railcom_items(
                pom_id(9),
                seq(11),
                DccAddress::new_short(3),
                true,
                &parsed.items,
            ),
            Some(PomRailcomResult::Window {
                request_id: pom_id(9),
                packet_sequence: seq(11),
                target_address: DccAddress::new_short(3),
                value: Some(0xab),
                ack: false,
                nack: false,
            })
        );
    }

    #[test]
    fn test_pom_result_lets_busy_suppress_a_verdict_beside_it() {
        // A decoder that says it cannot answer has neither agreed nor refused,
        // so the window must carry no verdict at all: nothing then reaches the
        // POM actor, and the request waits out its deadline instead of being
        // reported as accepted, or closed as a refusal never given. Before busy
        // was decoded this window failed as a reserved code and was retried;
        // the outcome must not get worse.
        for window in [
            [BUSY_CODE, ACK_1_CODE],
            [ACK_1_CODE, BUSY_CODE],
            [BUSY_CODE, NACK_CODE],
            [NACK_CODE, BUSY_CODE],
        ] {
            let parsed = parse_channel2(&window)
                .expect("busy beside a verdict is a valid control-only response");

            assert_eq!(
                pom_result_from_railcom_items(
                    pom_id(9),
                    seq(11),
                    DccAddress::new_short(3),
                    true,
                    &parsed.items,
                ),
                None,
                "window {window:#04x?}"
            );
        }
    }

    #[test]
    fn test_pom_result_ignores_unrelated_datagrams() {
        let items = [RailcomItem::Datagram(RailcomDatagram::Dyn {
            value: 0x12,
            sub_index: 0,
        })];

        assert_eq!(
            pom_result_from_railcom_items(
                pom_id(3),
                seq(99),
                DccAddress::new_short(3),
                true,
                &items,
            ),
            None
        );
    }

    #[test]
    fn test_pom_read_ignores_ack_only_feedback() {
        let items = [RailcomItem::Ack];

        assert_eq!(
            pom_result_from_railcom_items(
                pom_id(4),
                seq(101),
                DccAddress::new_short(3),
                false,
                &items,
            ),
            None
        );
    }

    #[test]
    fn test_pom_read_accepts_followup_packet_value() {
        let request = PomRequest::Read {
            request_id: pom_id(7),
            permit: permit(),
            address: DccAddress::new_short(3).unwrap(),
            cv: pom_cv(8),
        };
        let result = PomRailcomResult::Window {
            request_id: pom_id(7),
            packet_sequence: seq(45),
            target_address: DccAddress::new_short(3),
            value: Some(151),
            ack: false,
            nack: false,
        };

        assert_eq!(
            match_pom_result(request, seq(43), result),
            Some(PomResponse::Value {
                request_id: pom_id(7),
                value: 151,
            })
        );
    }

    #[test]
    fn test_pom_write_wire_ack_nack_reports_unsupported_cv() {
        let request = PomRequest::Write {
            request_id: pom_id(11),
            permit: permit(),
            address: DccAddress::new_short(3).unwrap(),
            cv: pom_cv(29),
            value: 6,
        };
        for ack in [0x0f, 0xf0] {
            let parsed = crate::railcom::parser::parse_channel2(&[ack, 0x3c]).unwrap();
            let result = pom_result_from_railcom_items(
                pom_id(11),
                seq(50),
                DccAddress::new_short(3),
                true,
                &parsed.items,
            )
            .unwrap();
            assert_eq!(
                match_pom_result(request, seq(50), result),
                Some(PomResponse::Nack {
                    request_id: pom_id(11)
                })
            );
        }
    }

    #[test]
    fn test_pom_write_accepts_matching_ack() {
        let request = PomRequest::Write {
            request_id: pom_id(11),
            permit: permit(),
            address: DccAddress::new_short(3).unwrap(),
            cv: pom_cv(29),
            value: 6,
        };
        let result = PomRailcomResult::Window {
            request_id: pom_id(11),
            packet_sequence: seq(50),
            target_address: DccAddress::new_short(3),
            value: None,
            ack: true,
            nack: false,
        };

        assert_eq!(
            match_pom_result(request, seq(50), result),
            Some(PomResponse::Ack {
                request_id: pom_id(11)
            })
        );
    }

    #[test]
    fn test_pom_read_rejects_result_from_previous_request() {
        let request = PomRequest::Read {
            request_id: pom_id(7),
            permit: permit(),
            address: DccAddress::new_short(3).unwrap(),
            cv: pom_cv(8),
        };
        let result = PomRailcomResult::Window {
            request_id: pom_id(6),
            packet_sequence: seq(45),
            target_address: DccAddress::new_short(3),
            value: Some(151),
            ack: false,
            nack: false,
        };

        assert_eq!(match_pom_result(request, seq(42), result), None);
    }

    #[test]
    fn test_pom_read_rejects_followup_packet_from_other_loco() {
        let request = PomRequest::Read {
            request_id: pom_id(7),
            permit: permit(),
            address: DccAddress::new_short(3).unwrap(),
            cv: pom_cv(8),
        };
        let result = PomRailcomResult::Window {
            request_id: pom_id(7),
            packet_sequence: seq(45),
            target_address: DccAddress::new_short(4),
            value: Some(151),
            ack: false,
            nack: false,
        };

        assert_eq!(match_pom_result(request, seq(43), result), None);
    }

    #[test]
    fn test_pom_rejects_delayed_previous_value_in_first_new_request_window() {
        let request = PomRequest::Read {
            request_id: pom_id(8),
            permit: permit(),
            address: DccAddress::new_short(3).unwrap(),
            cv: pom_cv(9),
        };
        let relabelled_previous_result = PomRailcomResult::Window {
            request_id: pom_id(8),
            packet_sequence: seq(100),
            target_address: DccAddress::new_short(3),
            value: Some(77),
            ack: false,
            nack: false,
        };

        assert_eq!(
            match_pom_result(request, seq(101), relabelled_previous_result),
            None,
        );
    }

    #[test]
    fn test_pom_sequence_gate_accepts_response_after_u32_wrap() {
        let request = PomRequest::Read {
            request_id: pom_id(9),
            permit: permit(),
            address: DccAddress::new_short(3).unwrap(),
            cv: pom_cv(10),
        };
        let result = PomRailcomResult::Window {
            request_id: pom_id(9),
            packet_sequence: seq(0),
            target_address: DccAddress::new_short(3),
            value: Some(88),
            ack: false,
            nack: false,
        };

        assert_eq!(
            match_pom_result(request, seq(u32::MAX), result),
            Some(PomResponse::Value {
                request_id: pom_id(9),
                value: 88,
            }),
        );
    }
}
