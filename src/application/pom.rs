//! Framework-independent Programming on Main application policy.

use crate::application::loco_projection::projected_slot_index;
use crate::application::{LeasePermit, LocoSlots};
use crate::dcc::{DccAddress, LocoRequest, PomCv, PomRequestId, SpeedFormat};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) enum PomAdmissionError {
    TrackPowerOff,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PomReadPlan {
    pub(crate) address: DccAddress,
    pub(crate) cv: PomCv,
    pub(crate) refresh_request: LocoRequest,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PomWritePlan {
    pub(crate) address: DccAddress,
    pub(crate) cv: PomCv,
    pub(crate) value: u8,
}

pub(crate) fn prepare_read(
    slots: &LocoSlots,
    pom_allowed: bool,
    address: DccAddress,
    cv: PomCv,
) -> Result<PomReadPlan, PomAdmissionError> {
    if !pom_allowed {
        return Err(PomAdmissionError::TrackPowerOff);
    }
    let format = projected_format(slots, address);
    Ok(PomReadPlan {
        address,
        cv,
        refresh_request: LocoRequest::EnsureRefresh { address, format },
    })
}

pub(crate) fn prepare_write(
    pom_allowed: bool,
    address: DccAddress,
    cv: PomCv,
    value: u8,
) -> Result<PomWritePlan, PomAdmissionError> {
    if !pom_allowed {
        return Err(PomAdmissionError::TrackPowerOff);
    }
    Ok(PomWritePlan { address, cv, value })
}

fn projected_format(slots: &LocoSlots, address: DccAddress) -> SpeedFormat {
    projected_slot_index(slots, address)
        .and_then(|index| slots[index])
        .map_or(SpeedFormat::DEFAULT, |slot| slot.speed.format())
}

/// An admitted POM operation, ready to be handed to the actor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PomJob {
    pub(crate) permit: LeasePermit,
    pub(crate) plan: PomPlan,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PomPlan {
    Read(PomReadPlan),
    Write(PomWritePlan),
}

impl PomPlan {
    /// Decoder the operation is addressed to.
    #[must_use]
    pub(crate) const fn address(self) -> DccAddress {
        match self {
            Self::Read(plan) => plan.address,
            Self::Write(plan) => plan.address,
        }
    }
}

/// How a POM request ended, as reported to its client.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PomOutcome {
    Value(u8),
    Ack,
    Nack,
    /// No response: the request could not be handed to the actor, or its
    /// deadline passed first.
    Unavailable,
    /// Replaced in the queue by a newer request from its client before it
    /// could start.
    Displaced,
    /// A read answered at once because its client has since asked about
    /// another decoder; whatever the decoder answers is dropped.
    Superseded,
    /// Refused before it reached the actor, for a reason already reported.
    #[cfg_attr(
        not(target_arch = "riscv32"),
        expect(
            dead_code,
            reason = "only the target refresh step refuses a started read"
        )
    )]
    Refused,
}

/// A request the caller must now hand to the actor under `request_id`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PomStart {
    pub(crate) request_id: PomRequestId,
    pub(crate) job: PomJob,
}

/// The reply owed to the client that sent `job`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PomDelivery<Reply> {
    pub(crate) reply: Reply,
    pub(crate) job: PomJob,
    pub(crate) outcome: PomOutcome,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PomSubmission<Reply> {
    /// The slot was free: start this request now.
    Start(PomStart),
    /// Waits behind the request in flight from the same client. `superseded`
    /// is the reply owed now to a read in flight for another decoder,
    /// `displaced` the one owed to the request pushed out of the queue.
    Queued {
        superseded: Option<PomDelivery<Reply>>,
        displaced: Option<PomDelivery<Reply>>,
    },
    /// Another client's request is in flight; refuse at once.
    Busy,
}

/// What follows the end of the request in flight: the reply to send, then
/// the queued successor to start, if any.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PomCompletion<Reply> {
    pub(crate) delivery: Option<PomDelivery<Reply>>,
    pub(crate) next: Option<PomStart>,
}

impl<Reply> PomCompletion<Reply> {
    #[must_use]
    pub(crate) const fn idle() -> Self {
        Self {
            delivery: None,
            next: None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct PomTicket<Client, Reply> {
    client: Client,
    reply: Reply,
    job: PomJob,
}

#[derive(Debug, Clone, Copy)]
struct InFlight<Client, Reply> {
    ticket: PomTicket<Client, Reply>,
    request_id: PomRequestId,
    /// `None` until the request has been handed to the actor.
    deadline_ms: Option<u64>,
    /// Already answered as [`PomOutcome::Superseded`]: its outcome is dropped.
    answered: bool,
}

/// POM requests of every Z21 client: one in flight at the single-flight actor,
/// and at most one successor from the same client waiting behind it.
///
/// Every request gets exactly one reply, and a client reads one decoder at a
/// time. The request in flight is answered even when its client has sent a
/// newer one for the same decoder meanwhile: a client polling CVs keeps its
/// rhythm only if every request it sent gets its reply, and the Z21 result
/// carries the CV number, so an older value cannot land in the field of a
/// newer read. It carries no decoder address, though: once the client asks
/// about another decoder, a read in flight is answered at once as
/// [`PomOutcome::Superseded`] and what the decoder answers is dropped, or the
/// value would show up as the other decoder's (bench, 2026-10-26: "CV 8 =
/// 151" of loco 4 sent 1.9 s after the app had moved to loco 5). A write in
/// flight is still answered with its outcome. A newer request from the client
/// in flight replaces the waiting successor, which is answered at once as
/// [`PomOutcome::Displaced`]: left without a reply, the Z21 app repeats it,
/// and each repeat displaced the next (same bench: 1496 reads sent, 412
/// answered, the app never settled). A queued write is never replaced: while
/// one waits, a newer request from its client is refused. The actor attempt
/// is never cancelled; the successor starts when the attempt ends.
pub(crate) struct PomSession<Client, Reply> {
    next_request_id: u32,
    in_flight: Option<InFlight<Client, Reply>>,
    queued: Option<PomTicket<Client, Reply>>,
}

impl<Client: Copy + Eq, Reply: Copy> PomSession<Client, Reply> {
    #[must_use]
    pub(crate) const fn new() -> Self {
        Self {
            next_request_id: 1,
            in_flight: None,
            queued: None,
        }
    }

    /// Admits one request. On [`PomSubmission::Start`] the caller hands it to
    /// the actor, then calls [`Self::arm_deadline`], or [`Self::complete`] with
    /// the failure if the hand-off fails.
    #[must_use]
    pub(crate) fn submit(
        &mut self,
        client: Client,
        reply: Reply,
        job: PomJob,
    ) -> PomSubmission<Reply> {
        let ticket = PomTicket { client, reply, job };
        let Some(in_flight) = self.in_flight.as_mut() else {
            return PomSubmission::Start(self.launch(ticket));
        };
        let queued_write = self
            .queued
            .is_some_and(|queued| matches!(queued.job.plan, PomPlan::Write(_)));
        if in_flight.ticket.client != client || queued_write {
            return PomSubmission::Busy;
        }
        let superseded = (!in_flight.answered
            && matches!(in_flight.ticket.job.plan, PomPlan::Read(_))
            && in_flight.ticket.job.plan.address() != job.plan.address())
        .then(|| {
            in_flight.answered = true;
            PomDelivery {
                reply: in_flight.ticket.reply,
                job: in_flight.ticket.job,
                outcome: PomOutcome::Superseded,
            }
        });
        let displaced = self.queued.replace(ticket).map(|old| PomDelivery {
            reply: old.reply,
            job: old.job,
            outcome: PomOutcome::Displaced,
        });
        PomSubmission::Queued {
            superseded,
            displaced,
        }
    }

    /// Starts the deadline of the request in flight, now handed to the actor.
    ///
    /// A successor handed over while the actor is still on an expired attempt
    /// counts its deadline from the hand-off, so it can expire before the
    /// actor picks it up.
    pub(crate) fn arm_deadline(&mut self, deadline_ms: u64) {
        if let Some(in_flight) = self.in_flight.as_mut() {
            in_flight.deadline_ms = Some(deadline_ms);
        }
    }

    /// Ends the request in flight if `request_id` is its id; a response to any
    /// other request is ignored.
    #[must_use]
    pub(crate) fn complete(
        &mut self,
        request_id: PomRequestId,
        outcome: PomOutcome,
    ) -> PomCompletion<Reply> {
        let Some(in_flight) = self
            .in_flight
            .take_if(|in_flight| in_flight.request_id == request_id)
        else {
            return PomCompletion::idle();
        };
        PomCompletion {
            delivery: (!in_flight.answered).then_some(PomDelivery {
                reply: in_flight.ticket.reply,
                job: in_flight.ticket.job,
                outcome,
            }),
            next: self.queued.take().map(|ticket| self.launch(ticket)),
        }
    }

    /// Ends the request in flight as [`PomOutcome::Unavailable`] once its
    /// deadline has passed.
    #[must_use]
    pub(crate) fn expire(&mut self, now_ms: u64) -> PomCompletion<Reply> {
        match self.in_flight {
            Some(InFlight {
                request_id,
                deadline_ms: Some(deadline_ms),
                ..
            }) if now_ms >= deadline_ms => self.complete(request_id, PomOutcome::Unavailable),
            _ => PomCompletion::idle(),
        }
    }

    fn launch(&mut self, ticket: PomTicket<Client, Reply>) -> PomStart {
        let request_id = PomRequestId::new(self.next_request_id);
        self.next_request_id = self.next_request_id.wrapping_add(1);
        self.in_flight = Some(InFlight {
            ticket,
            request_id,
            deadline_ms: None,
            answered: false,
        });
        PomStart {
            request_id,
            job: ticket.job,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::application::{LeaseEpoch, LocoState};
    use crate::dcc::{Direction, FunctionState, LogicalSpeed};

    fn address() -> DccAddress {
        DccAddress::new_short(3).unwrap()
    }

    fn state(format: SpeedFormat) -> LocoState {
        LocoState {
            address: address(),
            speed: LogicalSpeed::zero(format),
            direction: Direction::Forward,
            functions: FunctionState::empty(),
        }
    }

    #[test]
    fn write_is_admitted_only_while_pom_is_allowed() {
        let cv = PomCv::new(29).unwrap();
        assert_eq!(
            prepare_write(true, address(), cv, 7),
            Ok(PomWritePlan {
                address: address(),
                cv,
                value: 7,
            })
        );
        assert_eq!(
            prepare_write(false, address(), PomCv::new(29).unwrap(), 7),
            Err(PomAdmissionError::TrackPowerOff)
        );
    }

    #[test]
    fn read_plan_uses_confirmed_loco_format() {
        let mut slots = [None; 12];
        slots[0] = Some(state(SpeedFormat::Speed28));

        let plan = prepare_read(&slots, true, address(), PomCv::new(17).unwrap()).unwrap();

        assert_eq!(
            plan.refresh_request,
            LocoRequest::EnsureRefresh {
                address: address(),
                format: SpeedFormat::Speed28,
            }
        );
    }

    type Session = PomSession<u8, u16>;

    const OWNER: u8 = 1;
    const OTHER: u8 = 2;

    fn read(cv: u16) -> PomJob {
        PomJob {
            permit: LeasePermit::new(LeaseEpoch::new(1), u64::MAX),
            plan: PomPlan::Read(
                prepare_read(&[None; 12], true, address(), PomCv::new(cv).unwrap()).unwrap(),
            ),
        }
    }

    fn write(cv: u16, value: u8) -> PomJob {
        PomJob {
            permit: LeasePermit::new(LeaseEpoch::new(1), u64::MAX),
            plan: PomPlan::Write(
                prepare_write(true, address(), PomCv::new(cv).unwrap(), value).unwrap(),
            ),
        }
    }

    /// Starts `job` from a free slot and arms its deadline at 1 000 ms.
    fn start(session: &mut Session, client: u8, reply: u16, job: PomJob) -> PomStart {
        let PomSubmission::Start(start) = session.submit(client, reply, job) else {
            panic!("the slot must be free");
        };
        assert_eq!(start.job, job);
        session.arm_deadline(1_000);
        start
    }

    /// A request queued without displacing or superseding another.
    fn queued() -> PomSubmission<u16> {
        PomSubmission::Queued {
            superseded: None,
            displaced: None,
        }
    }

    fn delivery(reply: u16, job: PomJob, outcome: PomOutcome) -> Option<PomDelivery<u16>> {
        Some(PomDelivery {
            reply,
            job,
            outcome,
        })
    }

    #[test]
    fn read_in_flight_is_answered_before_the_newer_read_starts() {
        let mut session = Session::new();
        let first = start(&mut session, OWNER, 10, read(8));
        assert_eq!(session.submit(OWNER, 11, read(29)), queued());

        let completion = session.complete(first.request_id, PomOutcome::Value(5));
        assert_eq!(
            completion.delivery,
            delivery(10, read(8), PomOutcome::Value(5))
        );
        let next = completion.next.expect("the newest read must start");
        assert_eq!(next.job, read(29));
        session.arm_deadline(2_000);

        let completion = session.complete(next.request_id, PomOutcome::Value(3));
        assert_eq!(
            completion,
            PomCompletion {
                delivery: delivery(11, read(29), PomOutcome::Value(3)),
                next: None,
            }
        );
    }

    fn read_of(address: u8, cv: u16) -> PomJob {
        let address = DccAddress::new_short(address).unwrap();
        PomJob {
            permit: LeasePermit::new(LeaseEpoch::new(1), u64::MAX),
            plan: PomPlan::Read(
                prepare_read(&[None; 12], true, address, PomCv::new(cv).unwrap()).unwrap(),
            ),
        }
    }

    #[test]
    fn read_in_flight_is_answered_at_once_when_the_client_moves_to_another_loco() {
        // Bench, 2026-10-26: the Z21 app had moved to loco 5 and the value of
        // loco 4 still in flight landed in its fields.
        let mut session = Session::new();
        let first = start(&mut session, OWNER, 10, read_of(4, 8));

        assert_eq!(
            session.submit(OWNER, 11, read_of(5, 8)),
            PomSubmission::Queued {
                superseded: delivery(10, read_of(4, 8), PomOutcome::Superseded),
                displaced: None
            }
        );

        let completion = session.complete(first.request_id, PomOutcome::Value(151));
        assert_eq!(completion.delivery, None);
        assert_eq!(completion.next.map(|next| next.job), Some(read_of(5, 8)));
    }

    #[test]
    fn superseded_read_is_answered_once_even_if_the_client_keeps_switching() {
        let mut session = Session::new();
        start(&mut session, OWNER, 10, read_of(4, 8));
        assert!(matches!(
            session.submit(OWNER, 11, read_of(5, 7)),
            PomSubmission::Queued {
                superseded: Some(_),
                ..
            }
        ));

        assert_eq!(
            session.submit(OWNER, 12, read_of(4, 7)),
            PomSubmission::Queued {
                superseded: None,
                displaced: delivery(11, read_of(5, 7), PomOutcome::Displaced)
            }
        );

        let completion = session.expire(1_000);
        assert_eq!(completion.delivery, None);
        assert_eq!(completion.next.map(|next| next.job), Some(read_of(4, 7)));
    }

    #[test]
    fn write_in_flight_keeps_its_outcome_when_the_client_moves_to_another_loco() {
        let mut session = Session::new();
        let first = start(&mut session, OWNER, 10, write(29, 6));

        assert_eq!(session.submit(OWNER, 11, read_of(5, 8)), queued());

        assert_eq!(
            session.complete(first.request_id, PomOutcome::Ack).delivery,
            delivery(10, write(29, 6), PomOutcome::Ack)
        );
    }

    #[test]
    fn another_clients_request_is_refused_while_the_slot_is_busy() {
        let mut session = Session::new();
        let first = start(&mut session, OWNER, 10, read(8));

        assert_eq!(session.submit(OTHER, 20, read(7)), PomSubmission::Busy);

        let completion = session.complete(first.request_id, PomOutcome::Value(5));
        assert_eq!(
            completion,
            PomCompletion {
                delivery: delivery(10, read(8), PomOutcome::Value(5)),
                next: None,
            }
        );
    }

    #[test]
    fn pending_write_is_delivered_before_the_next_read_starts() {
        let mut session = Session::new();
        let first = start(&mut session, OWNER, 10, write(29, 6));
        assert_eq!(session.submit(OWNER, 11, read(29)), queued());

        let completion = session.complete(first.request_id, PomOutcome::Ack);

        assert_eq!(
            completion.delivery,
            delivery(10, write(29, 6), PomOutcome::Ack)
        );
        assert_eq!(completion.next.map(|next| next.job), Some(read(29)));
    }

    #[test]
    fn response_with_an_unknown_request_id_is_ignored() {
        let mut session = Session::new();
        let first = start(&mut session, OWNER, 10, read(8));
        let stale = PomRequestId::new(first.request_id.value().wrapping_sub(1));

        assert_eq!(
            session.complete(stale, PomOutcome::Nack),
            PomCompletion::idle()
        );
        assert_eq!(
            session
                .complete(first.request_id, PomOutcome::Value(5))
                .delivery,
            delivery(10, read(8), PomOutcome::Value(5))
        );
    }

    #[test]
    fn deadline_answers_the_pending_request_and_starts_the_successor() {
        let mut session = Session::new();
        start(&mut session, OWNER, 10, write(29, 6));
        assert_eq!(session.submit(OWNER, 11, read(29)), queued());

        assert_eq!(session.expire(999), PomCompletion::idle());
        let completion = session.expire(1_000);

        assert_eq!(
            completion.delivery,
            delivery(10, write(29, 6), PomOutcome::Unavailable)
        );
        assert_eq!(completion.next.map(|next| next.job), Some(read(29)));
    }

    #[test]
    fn newer_queued_request_replaces_the_older_one_which_is_answered_at_once() {
        let mut session = Session::new();
        let first = start(&mut session, OWNER, 10, write(29, 6));
        assert_eq!(session.submit(OWNER, 11, read(7)), queued());
        assert_eq!(
            session.submit(OWNER, 12, read(8)),
            PomSubmission::Queued {
                superseded: None,
                displaced: delivery(11, read(7), PomOutcome::Displaced)
            }
        );

        let next = session
            .complete(first.request_id, PomOutcome::Ack)
            .next
            .expect("the newest queued read must start");
        assert_eq!(next.job, read(8));
        session.arm_deadline(2_000);

        assert_eq!(
            session.complete(next.request_id, PomOutcome::Value(4)),
            PomCompletion {
                delivery: delivery(12, read(8), PomOutcome::Value(4)),
                next: None,
            }
        );
    }

    #[test]
    fn queued_write_is_not_replaced_by_a_later_request() {
        let mut session = Session::new();
        let first = start(&mut session, OWNER, 10, read(8));
        assert_eq!(session.submit(OWNER, 11, write(29, 6)), queued());

        assert_eq!(session.submit(OWNER, 12, read(7)), PomSubmission::Busy);

        let next = session
            .complete(first.request_id, PomOutcome::Value(5))
            .next
            .expect("the queued write must start");
        assert_eq!(next.job, write(29, 6));
    }

    #[test]
    fn unknown_loco_defaults_read_refresh_to_speed128() {
        let plan = prepare_read(&[None; 12], true, address(), PomCv::new(17).unwrap()).unwrap();
        assert_eq!(
            plan.refresh_request,
            LocoRequest::EnsureRefresh {
                address: address(),
                format: SpeedFormat::Speed128,
            }
        );
    }
}
