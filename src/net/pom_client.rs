//! Z21-side POM (Programming on Main) request/response client.
//!
//! Talks to the POM actor task (`dcc::cv::pom_actor_task`) over the
//! single-slot request/response channels threaded through [`Z21Ctx`].
//!
//! ESP32-C6 only, gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

use core::cell::Cell;

use defmt::warn;

use super::correlated::{SendMode, correlated_request};
use crate::application::LocoSlots;
use crate::application::locomotive::{
    LocoCommandFailure, LocoCommandOutcome, resolve_loco_command,
};
use crate::application::pom::{PomAdmissionError, prepare_read, prepare_write};
use crate::dcc::channels::{PomRequestSender, PomResponseReceiver};
use crate::dcc::{PomRequest, PomRequestId, PomResponse};
use crate::net::loco_client::request_loco;
use crate::net::z21_context::Z21Ctx;
use crate::net::z21_dispatch::encoded_len;
use crate::z21 as z21_proto;

// Covers the single bounded POM actor attempt: TX-start waits + the
// RailCom app:pom attribution window, with margin for task scheduling.
const POM_CLIENT_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(2_700);

#[derive(Clone, Copy, defmt::Format)]
enum PomOperation {
    Read,
    Write,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PomOutcome {
    Value(u8),
    Ack,
    Nack,
    Unavailable,
}

/// Sends one POM request to the actor and waits for the reply carrying its id.
///
/// The id comes from the net task's local counter: it only needs to be unique
/// within this task's single-flight request/response channel.
async fn request_pom(
    pom_request_sender: &PomRequestSender,
    pom_response_receiver: &PomResponseReceiver,
    next_request_id: &Cell<u32>,
    build_request: impl FnOnce(PomRequestId) -> PomRequest,
) -> Option<PomResponse> {
    correlated_request(
        pom_request_sender,
        pom_response_receiver,
        next_request_id,
        PomRequestId::new,
        |request_id, _now| build_request(request_id),
        PomResponse::request_id,
        SendMode::TryOnce,
        POM_CLIENT_TIMEOUT,
    )
    .await
}

pub(super) async fn handle_cv_pom_write(
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
    address: crate::dcc::DccAddress,
    cv: u16,
    value: u8,
) -> usize {
    let plan = match prepare_write(ctx.status_model.pom_allowed(), address, cv, value) {
        Ok(plan) => plan,
        Err(error) => return reject_admission(out, ctx, address, cv, PomOperation::Write, error),
    };

    let Some(permit) = ctx.lease_permit else {
        return encode_nack(out);
    };

    let reply = classify_reply(
        request_pom(
            ctx.pom.request_sender,
            ctx.pom.response_receiver,
            ctx.pom.next_request_id,
            |request_id| PomRequest::Write {
                request_id,
                permit,
                address: plan.address,
                cv: plan.cv,
                value: plan.value,
            },
        )
        .await,
    );
    match reply {
        PomOutcome::Ack => encoded_len(z21_proto::encode_cv_result(cv, value, out)),
        PomOutcome::Nack => encode_nack(out),
        PomOutcome::Value(_) | PomOutcome::Unavailable => {
            warn!(
                "POM write addr={} cv={} completed without ACK",
                address.value(),
                cv
            );
            encode_nack(out)
        }
    }
}

pub(super) async fn handle_cv_pom_read(
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
    address: crate::dcc::DccAddress,
    cv: u16,
) -> usize {
    let plan = match prepare_read(loco_slots, ctx.status_model.pom_allowed(), address, cv) {
        Ok(plan) => plan,
        Err(error) => return reject_admission(out, ctx, address, cv, PomOperation::Read, error),
    };
    let Some(permit) = ctx.lease_permit else {
        return encode_nack(out);
    };
    let refresh = request_loco(
        ctx.loco.request_sender,
        ctx.loco.response_receiver,
        ctx.loco.next_request_id,
        Some(permit),
        plan.refresh_request,
    )
    .await;
    match resolve_loco_command(loco_slots, address, refresh.map(|response| response.result)) {
        LocoCommandOutcome::Applied(_) => {}
        LocoCommandOutcome::Rejected(LocoCommandFailure::SchedulerTimeout) => {
            warn!(
                "POM read addr={} cv={} rejected: scheduler response timed out",
                address.value(),
                cv
            );
            return encode_nack(out);
        }
        LocoCommandOutcome::Rejected(LocoCommandFailure::SchedulerRejected(result)) => {
            warn!(
                "POM read addr={} cv={} rejected by scheduler: {:?}",
                address.value(),
                cv,
                result
            );
            return encode_nack(out);
        }
        LocoCommandOutcome::Rejected(LocoCommandFailure::Projection(error)) => {
            warn!(
                "POM read addr={} cv={} projection failed: {:?}",
                address.value(),
                cv,
                error
            );
            return encode_nack(out);
        }
    }

    let reply = classify_reply(
        request_pom(
            ctx.pom.request_sender,
            ctx.pom.response_receiver,
            ctx.pom.next_request_id,
            |request_id| PomRequest::Read {
                request_id,
                permit,
                address: plan.address,
                cv: plan.cv,
            },
        )
        .await,
    );
    match reply {
        PomOutcome::Value(value) => encoded_len(z21_proto::encode_cv_result(cv, value, out)),
        PomOutcome::Ack | PomOutcome::Nack | PomOutcome::Unavailable => {
            warn!(
                "POM read addr={} cv={} completed without value",
                address.value(),
                cv
            );
            encode_nack(out)
        }
    }
}

fn classify_reply(response: Option<PomResponse>) -> PomOutcome {
    match response {
        Some(PomResponse::Value { value, .. }) => PomOutcome::Value(value),
        Some(PomResponse::Ack { .. }) => PomOutcome::Ack,
        Some(PomResponse::Nack { .. }) => PomOutcome::Nack,
        None => PomOutcome::Unavailable,
    }
}

fn encode_nack(out: &mut [u8]) -> usize {
    encoded_len(z21_proto::encode_cv_nack(out))
}

fn reject_admission(
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
    address: crate::dcc::DccAddress,
    cv: u16,
    operation: PomOperation,
    error: PomAdmissionError,
) -> usize {
    match error {
        PomAdmissionError::InvalidCv(_) => {
            warn!(
                "POM {:?} addr={} invalid cv={}",
                operation,
                address.value(),
                cv
            );
        }
        PomAdmissionError::TrackPowerOff => warn!(
            "POM {:?} addr={} cv={} rejected (track_on={} estop={} fault={:?})",
            operation,
            address.value(),
            cv,
            ctx.status_model.track_power_on(),
            ctx.status_model.estop_active(),
            ctx.status_model.fault_cause()
        ),
    }
    encode_nack(out)
}
