//! Z21-side POM (Programming on Main) request/response client.
//!
//! Admits Z21 POM commands into the loop's [`Z21PomSession`] and hands the
//! started requests to the POM actor task (`dcc::cv::pom_actor_task`). The
//! actor's responses are received by the Z21 loop, which settles them through
//! the session and sends the replies encoded here.
//!
//! ESP32-C6 only, gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

use defmt::{info, warn};

use crate::application::locomotive::{
    LocoCommandFailure, LocoCommandOutcome, LocoLookupIssue, resolve_loco_command,
    resolve_loco_lookup,
};
use crate::application::pom::{
    PomAdmissionError, PomCompletion, PomDelivery, PomJob, PomOutcome, PomPlan, PomReadPlan,
    PomStart, PomSubmission, prepare_read, prepare_write,
};
use crate::application::{LeasePermit, LocoSlots};
use crate::dcc::PomCv;
use crate::dcc::channels::PomRequestSender;
use crate::dcc::{LocoRequest, PomOp, PomRequest, PomResponse};
use crate::net::loco_client::request_loco;
use crate::net::z21_context::{LocoChannels, Z21Ctx, Z21PomSession};
use crate::net::z21_dispatch::encoded_len;
use z21 as z21_proto;

// Covers the single bounded POM actor attempt: TX-start waits + the
// RailCom app:pom attribution window, with margin for task scheduling.
const POM_CLIENT_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(2_700);

#[derive(Clone, Copy, defmt::Format)]
enum PomOperation {
    Read,
    Write,
}

pub(super) async fn handle_cv_pom_write(
    loco_slots: &mut LocoSlots,
    session: &mut Z21PomSession,
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
    address: crate::dcc::DccAddress,
    cv: PomCv,
    value: u8,
) -> usize {
    let plan = match prepare_write(ctx.status_model.pom_allowed(), address, cv, value) {
        Ok(plan) => plan,
        Err(error) => return reject_admission(out, ctx, address, cv, PomOperation::Write, error),
    };
    let Some(permit) = ctx.lease_permit else {
        return encode_nack(out);
    };
    let job = PomJob {
        permit,
        plan: PomPlan::Write(plan),
    };
    submit(loco_slots, session, out, ctx, job).await
}

pub(super) async fn handle_cv_pom_read(
    loco_slots: &mut LocoSlots,
    session: &mut Z21PomSession,
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
    address: crate::dcc::DccAddress,
    cv: PomCv,
) -> usize {
    let plan = match prepare_read(loco_slots, ctx.status_model.pom_allowed(), address, cv) {
        Ok(plan) => plan,
        Err(error) => return reject_admission(out, ctx, address, cv, PomOperation::Read, error),
    };
    let Some(permit) = ctx.lease_permit else {
        return encode_nack(out);
    };
    let job = PomJob {
        permit,
        plan: PomPlan::Read(plan),
    };
    submit(loco_slots, session, out, ctx, job).await
}

/// Answers at once only when the request is refused or fails to start; a
/// pending or queued request is answered later from the Z21 loop.
async fn submit(
    loco_slots: &mut LocoSlots,
    session: &mut Z21PomSession,
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
    job: PomJob,
) -> usize {
    match session.submit(ctx.endpoint.addr, ctx.endpoint, job) {
        PomSubmission::Start(start) => {
            let completion = start_request(
                start,
                loco_slots,
                session,
                &ctx.loco,
                ctx.pom_request_sender,
            )
            .await;
            debug_assert!(
                completion.next.is_none(),
                "a request started from a free slot has no successor"
            );
            if let Some(release) = completion.release {
                release_target(loco_slots, &ctx.loco, release).await;
            }
            completion
                .delivery
                .map_or(0, |delivery| encode_delivery(delivery, out))
        }
        PomSubmission::Queued {
            superseded,
            displaced,
        } => {
            // Both replies go to the sender of this datagram, as one datagram.
            let mut len = 0;
            for delivery in [superseded, displaced].into_iter().flatten() {
                len += encode_delivery(delivery, &mut out[len..]);
            }
            len
        }
        PomSubmission::Busy => {
            let (operation, address, cv) = match job.plan {
                PomPlan::Read(plan) => (PomOperation::Read, plan.address, plan.cv),
                PomPlan::Write(plan) => (PomOperation::Write, plan.address, plan.cv),
            };
            warn!(
                "POM {:?} addr={} cv={} refused: another request is pending",
                operation,
                address.value(),
                cv.value()
            );
            encode_nack(out)
        }
    }
}

/// Hands `start` to the actor and arms its deadline.
///
/// A read first makes sure the target loco is in the refresh cycle. When the
/// request cannot be handed over, returns the session's completion for that
/// failure. Returns [`PomCompletion::idle`] once the actor has the request:
/// the reply then follows from its response or its deadline.
pub(super) async fn start_request(
    start: PomStart,
    loco_slots: &mut LocoSlots,
    session: &mut Z21PomSession,
    loco: &LocoChannels<'_>,
    pom_request_sender: &PomRequestSender,
) -> PomCompletion<embassy_net::IpEndpoint> {
    let PomStart { request_id, job } = start;
    let (address, cv, op) = match job.plan {
        PomPlan::Read(plan) => {
            if !refresh_target(loco_slots, loco, job.permit, plan).await {
                return session.complete(request_id, PomOutcome::Refused);
            }
            (plan.address, plan.cv, PomOp::Read)
        }
        PomPlan::Write(plan) => (plan.address, plan.cv, PomOp::Write(plan.value)),
    };
    let request = PomRequest {
        request_id,
        permit: job.permit,
        address,
        cv,
        op,
    };
    if pom_request_sender.try_send(request).is_err() {
        return session.complete(request_id, PomOutcome::Unavailable);
    }
    session.arm_deadline(crate::clock::now_ms() + POM_CLIENT_TIMEOUT.as_millis());
    PomCompletion::idle()
}

async fn refresh_target(
    loco_slots: &mut LocoSlots,
    loco: &LocoChannels<'_>,
    permit: LeasePermit,
    plan: PomReadPlan,
) -> bool {
    let address = plan.address;
    let cv = plan.cv.value();
    let refresh = request_loco(
        loco.request_sender,
        loco.response_receiver,
        loco.next_request_id,
        Some(permit),
        plan.refresh_request,
    )
    .await;
    match resolve_loco_command(loco_slots, address, refresh.map(|response| response.result)) {
        LocoCommandOutcome::Applied(_) => true,
        LocoCommandOutcome::Rejected(LocoCommandFailure::SchedulerTimeout) => {
            warn!(
                "POM read addr={} cv={} rejected: scheduler response timed out",
                address.value(),
                cv
            );
            false
        }
        LocoCommandOutcome::Rejected(LocoCommandFailure::SchedulerRejected(result)) => {
            warn!(
                "POM read addr={} cv={} rejected by scheduler: {:?}",
                address.value(),
                cv,
                result
            );
            false
        }
        LocoCommandOutcome::Rejected(LocoCommandFailure::Projection(error)) => {
            warn!(
                "POM read addr={} cv={} projection failed: {:?}",
                address.value(),
                cv,
                error
            );
            false
        }
    }
}

/// Sends `request`, the release a finished read owes its refresh slot, and
/// drops the address from `loco_slots` when the scheduler removed the slot.
pub(super) async fn release_target(
    loco_slots: &mut LocoSlots,
    loco: &LocoChannels<'_>,
    request: LocoRequest,
) {
    let address = request.address();
    let response = request_loco(
        loco.request_sender,
        loco.response_receiver,
        loco.next_request_id,
        None,
        request,
    )
    .await;
    let outcome = resolve_loco_lookup(
        loco_slots,
        address,
        response.map(|response| response.result),
    );
    match outcome.issue {
        None => {}
        Some(LocoLookupIssue::SchedulerTimeout) => warn!(
            "POM read addr={} refresh release timed out",
            address.value()
        ),
        Some(LocoLookupIssue::UnexpectedResult(result)) => warn!(
            "POM read addr={} refresh release result={:?}",
            address.value(),
            result
        ),
        Some(LocoLookupIssue::Projection(error)) => warn!(
            "POM read addr={} refresh release projection failed: {:?}",
            address.value(),
            error
        ),
    }
}

/// Outcome carried by an actor response.
#[must_use]
pub(super) const fn response_outcome(response: PomResponse) -> PomOutcome {
    match response {
        PomResponse::Value { value, .. } => PomOutcome::Value(value),
        PomResponse::Ack { .. } => PomOutcome::Ack,
        PomResponse::Nack { .. } => PomOutcome::Nack,
    }
}

/// Encodes the Z21 reply for a finished request into `out`.
pub(super) fn encode_delivery(
    delivery: PomDelivery<embassy_net::IpEndpoint>,
    out: &mut [u8],
) -> usize {
    match (delivery.job.plan, delivery.outcome) {
        (PomPlan::Read(plan), PomOutcome::Value(value)) => {
            info!(
                "POM read addr={} cv={} value={}",
                plan.address.value(),
                plan.cv.value(),
                value
            );
            encoded_len(z21_proto::encode_cv_result(plan.cv, value, out))
        }
        (PomPlan::Read(plan), PomOutcome::Ack | PomOutcome::Nack | PomOutcome::Unavailable) => {
            warn!(
                "POM read addr={} cv={} completed without value",
                plan.address.value(),
                plan.cv.value()
            );
            encode_nack(out)
        }
        (PomPlan::Write(plan), PomOutcome::Ack) => {
            encoded_len(z21_proto::encode_cv_result(plan.cv, plan.value, out))
        }
        (PomPlan::Write(_), PomOutcome::Nack)
        | (_, PomOutcome::Refused | PomOutcome::Displaced | PomOutcome::Superseded) => {
            encode_nack(out)
        }
        (PomPlan::Write(plan), PomOutcome::Value(_) | PomOutcome::Unavailable) => {
            warn!(
                "POM write addr={} cv={} completed without ACK",
                plan.address.value(),
                plan.cv.value()
            );
            encode_nack(out)
        }
    }
}

fn encode_nack(out: &mut [u8]) -> usize {
    encoded_len(z21_proto::encode_cv_nack(out))
}

fn reject_admission(
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
    address: crate::dcc::DccAddress,
    cv: PomCv,
    operation: PomOperation,
    error: PomAdmissionError,
) -> usize {
    match error {
        PomAdmissionError::TrackPowerOff => warn!(
            "POM {:?} addr={} cv={} rejected (track_on={} estop={} fault={:?})",
            operation,
            address.value(),
            cv.value(),
            ctx.status_model.track_power_on(),
            ctx.status_model.estop_active(),
            ctx.status_model.fault_cause()
        ),
    }
    encode_nack(out)
}
