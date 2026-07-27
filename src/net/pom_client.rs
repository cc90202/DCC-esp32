//! Z21-side POM (Programming on Main) request/response client.
//!
//! Talks to the POM actor task (`dcc::cv::pom_actor_task`) over the
//! single-slot request/response channels threaded through [`PomCtx`].
//!
//! ESP32-C6 only, gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

use core::cell::Cell;

use defmt::warn;
use embassy_time::{Instant, with_timeout};

use crate::application::LocoSlots;
use crate::application::pom::{
    PomAdmissionError, PomOutcome, PomRefreshFailure, PomReply, apply_refresh_result, prepare_read,
    prepare_write, resolve_reply,
};
use crate::dcc::cv::drain_channel;
use crate::dcc::{PomRequest, PomRequestId, PomResponse};
use crate::net::loco_client::request_loco;
use crate::net::z21_context::PomCtx;
use crate::runtime_channels::{PomRequestSender, PomResponseReceiver};
use crate::z21 as z21_proto;

// Covers the single bounded POM actor attempt: TX-start waits + the
// RailCom app:pom attribution window, with margin for task scheduling.
const POM_CLIENT_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(2_700);

/// Allocate the next [`PomRequestId`] from the net task's local counter.
///
/// Replaces a global `AtomicU32`: the counter only needs to be unique within
/// this task's single-flight request/response channel, so a plain `Cell`
/// owned by `net_task` is enough.
fn next_pom_request_id(counter: &Cell<u32>) -> PomRequestId {
    let id = counter.get();
    counter.set(id.wrapping_add(1));
    PomRequestId::new(id)
}

pub(super) async fn request_pom(
    pom_request_sender: &PomRequestSender,
    pom_response_receiver: &PomResponseReceiver,
    next_request_id: &Cell<u32>,
    build_request: impl FnOnce(PomRequestId) -> PomRequest,
) -> Option<PomResponse> {
    let request_id = next_pom_request_id(next_request_id);
    drain_channel(pom_response_receiver);

    if pom_request_sender
        .try_send(build_request(request_id))
        .is_err()
    {
        return None;
    }

    let deadline = Instant::now() + POM_CLIENT_TIMEOUT;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        let response = with_timeout(remaining, pom_response_receiver.receive())
            .await
            .ok()?;
        match response {
            PomResponse::Ack {
                request_id: response_request_id,
            }
            | PomResponse::Nack {
                request_id: response_request_id,
            } if response_request_id == request_id => {
                return Some(response);
            }
            PomResponse::Value {
                request_id: response_request_id,
                ..
            } if response_request_id == request_id => return Some(response),
            _ => {}
        }
    }
}

pub(super) async fn handle_cv_pom_write(
    out: &mut [u8],
    ctx: &PomCtx<'_>,
    address: crate::dcc::DccAddress,
    cv: u16,
    value: u8,
) -> usize {
    let plan = match prepare_write(ctx.status_model.pom_allowed(), address, cv, value) {
        Ok(plan) => plan,
        Err(error) => return reject_admission(out, ctx, address, cv, "write", error),
    };

    ctx.scheduler_sender
        .send(crate::dcc::SchedulerCommand::SuspendRailcomDiscovery)
        .await;

    let reply = classify_reply(
        request_pom(
            ctx.request_sender,
            ctx.response_receiver,
            ctx.next_request_id,
            |request_id| PomRequest::Write {
                request_id,
                address: plan.address,
                cv: plan.cv,
                value: plan.value,
            },
        )
        .await,
    );
    match resolve_reply(reply) {
        PomOutcome::Ack => z21_proto::encode_cv_result(cv, value, out),
        PomOutcome::Nack => z21_proto::encode_cv_nack(out),
        PomOutcome::Value(_) | PomOutcome::Unavailable => {
            warn!(
                "POM write addr={} cv={} completed without ACK",
                address.value(),
                cv
            );
            z21_proto::encode_cv_nack(out)
        }
    }
}

pub(super) async fn handle_cv_pom_read(
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &PomCtx<'_>,
    address: crate::dcc::DccAddress,
    cv: u16,
) -> usize {
    let plan = match prepare_read(loco_slots, ctx.status_model.pom_allowed(), address, cv) {
        Ok(plan) => plan,
        Err(error) => return reject_admission(out, ctx, address, cv, "read", error),
    };
    let refresh = request_loco(
        ctx.loco.request_sender,
        ctx.loco.response_receiver,
        ctx.loco.next_request_id,
        plan.refresh_request,
    )
    .await;
    match apply_refresh_result(loco_slots, refresh.map(|response| response.result)) {
        Ok(()) => {}
        Err(PomRefreshFailure::SchedulerTimeout) => {
            warn!(
                "POM read addr={} cv={} rejected: scheduler response timed out",
                address.value(),
                cv
            );
            return z21_proto::encode_cv_nack(out);
        }
        Err(PomRefreshFailure::SchedulerRejected(result)) => {
            warn!(
                "POM read addr={} cv={} rejected by scheduler: {:?}",
                address.value(),
                cv,
                result
            );
            return z21_proto::encode_cv_nack(out);
        }
        Err(PomRefreshFailure::Projection(error)) => {
            warn!(
                "POM read addr={} cv={} projection failed: {:?}",
                address.value(),
                cv,
                error
            );
            return z21_proto::encode_cv_nack(out);
        }
    }

    ctx.scheduler_sender
        .send(crate::dcc::SchedulerCommand::SuspendRailcomDiscovery)
        .await;

    let reply = classify_reply(
        request_pom(
            ctx.request_sender,
            ctx.response_receiver,
            ctx.next_request_id,
            |request_id| PomRequest::Read {
                request_id,
                address: plan.address,
                cv: plan.cv,
            },
        )
        .await,
    );
    match resolve_reply(reply) {
        PomOutcome::Value(value) => z21_proto::encode_cv_result(cv, value, out),
        PomOutcome::Ack | PomOutcome::Nack | PomOutcome::Unavailable => {
            warn!(
                "POM read addr={} cv={} completed without value",
                address.value(),
                cv
            );
            z21_proto::encode_cv_nack(out)
        }
    }
}

fn classify_reply(response: Option<PomResponse>) -> PomReply {
    match response {
        Some(PomResponse::Value { value, .. }) => PomReply::Value(value),
        Some(PomResponse::Ack { .. }) => PomReply::Ack,
        Some(PomResponse::Nack { .. }) => PomReply::Nack,
        None => PomReply::Unavailable,
    }
}

fn reject_admission(
    out: &mut [u8],
    ctx: &PomCtx<'_>,
    address: crate::dcc::DccAddress,
    cv: u16,
    operation: &str,
    error: PomAdmissionError,
) -> usize {
    match error {
        PomAdmissionError::InvalidCv(_) => {
            warn!(
                "POM {} addr={} invalid cv={}",
                operation,
                address.value(),
                cv
            );
        }
        PomAdmissionError::TrackPowerOff => warn!(
            "POM {} addr={} cv={} rejected (track_on={} estop={} fault={:?})",
            operation,
            address.value(),
            cv,
            ctx.status_model.track_power_on(),
            ctx.status_model.estop_active(),
            ctx.status_model.fault_cause()
        ),
    }
    z21_proto::encode_cv_nack(out)
}
