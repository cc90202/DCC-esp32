//! Z21-side POM (Programming on Main) request/response client.
//!
//! Talks to the POM actor task (`dcc::cv::pom_actor_task`) over the
//! single-slot request/response channels threaded through [`Z21Ctx`].
//!
//! ESP32-C6 only — gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

use core::cell::Cell;

use defmt::warn;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Instant, with_timeout};

use crate::dcc::cv::drain_channel;
use crate::dcc::{PomCv, PomRequest, PomRequestId, PomResponse};
use crate::net::LocoSlots;
use crate::net::udp_control::Z21Ctx;
use crate::net::z21_proto;

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
    pom_request_sender: &Sender<'static, CriticalSectionRawMutex, PomRequest, 1>,
    pom_response_receiver: &Receiver<'static, CriticalSectionRawMutex, PomResponse, 1>,
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
    ctx: &Z21Ctx<'_>,
    address: crate::dcc::DccAddress,
    cv: u16,
    value: u8,
) -> usize {
    let Some(cv_addr) = PomCv::new(cv) else {
        warn!("POM write addr={} invalid cv={}", address.value(), cv);
        return z21_proto::encode_cv_nack(out);
    };

    if !ctx.status_model.pom_allowed() {
        warn!(
            "POM write addr={} cv={} rejected (track_on={} estop={} fault={:?})",
            address.value(),
            cv,
            ctx.status_model.track_power_on(),
            ctx.status_model.estop_active(),
            ctx.status_model.fault_cause()
        );
        return z21_proto::encode_cv_nack(out);
    }

    ctx.scheduler_sender
        .send(crate::dcc::SchedulerCommand::SuspendRailcomDiscovery)
        .await;

    match request_pom(
        ctx.pom_request_sender,
        ctx.pom_response_receiver,
        ctx.next_pom_request_id,
        |request_id| PomRequest::Write {
            request_id,
            address,
            cv: cv_addr,
            value,
        },
    )
    .await
    {
        Some(PomResponse::Ack { .. }) => z21_proto::encode_cv_result(cv, value, out),
        Some(PomResponse::Nack { .. }) => z21_proto::encode_cv_nack(out),
        Some(PomResponse::Value { .. }) | None => {
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
    ctx: &Z21Ctx<'_>,
    address: crate::dcc::DccAddress,
    cv: u16,
) -> usize {
    let Some(cv_addr) = PomCv::new(cv) else {
        warn!("POM read addr={} invalid cv={}", address.value(), cv);
        return z21_proto::encode_cv_nack(out);
    };

    if !ctx.status_model.pom_allowed() {
        warn!(
            "POM read addr={} cv={} rejected (track_on={} estop={} fault={:?})",
            address.value(),
            cv,
            ctx.status_model.track_power_on(),
            ctx.status_model.estop_active(),
            ctx.status_model.fault_cause()
        );
        return z21_proto::encode_cv_nack(out);
    }

    ctx.scheduler_sender
        .send(crate::dcc::SchedulerCommand::SuspendRailcomDiscovery)
        .await;

    let format = z21_proto::find_or_insert(loco_slots, address)
        .map(|slot| slot.format)
        .unwrap_or(crate::dcc::SpeedFormat::Speed128);
    ctx.scheduler_sender
        .send(crate::dcc::SchedulerCommand::EnsureRailcomRefresh { address, format })
        .await;

    match request_pom(
        ctx.pom_request_sender,
        ctx.pom_response_receiver,
        ctx.next_pom_request_id,
        |request_id| PomRequest::Read {
            request_id,
            address,
            cv: cv_addr,
        },
    )
    .await
    {
        Some(PomResponse::Value { value, .. }) => z21_proto::encode_cv_result(cv, value, out),
        Some(PomResponse::Ack { .. }) | Some(PomResponse::Nack { .. }) | None => {
            warn!(
                "POM read addr={} cv={} completed without value",
                address.value(),
                cv
            );
            z21_proto::encode_cv_nack(out)
        }
    }
}
