//! Z21 adapter for track-power commands and status responses.

use crate::application::StatusModel;
use crate::application::track_control::{
    LeaseCheck, RefusalReply, RefuseReason, TrackPowerFeedback, TrackPowerRequest,
    TrackPowerVerdict, TrackStatus, judge_track_power, refusal_reply,
};
use crate::net::z21_context::Z21Ctx;
use z21 as z21_proto;

use super::encoded_len;

pub(super) fn encode_current_system_state(status_model: StatusModel, out: &mut [u8]) -> usize {
    encode_system_state(TrackStatus::from_model(status_model), out)
}

fn central_state(status: TrackStatus) -> z21_proto::CentralState {
    z21_proto::CentralState {
        track_on: status.power_on,
        estop: status.estop_active,
        short_circuit: status.short_circuit,
    }
}

pub(in crate::net) fn encode_system_state(status: TrackStatus, out: &mut [u8]) -> usize {
    encoded_len(z21_proto::encode_system_state(central_state(status), out))
}

pub(super) fn encode_current_status(status_model: StatusModel, out: &mut [u8]) -> usize {
    let status = TrackStatus::from_model(status_model);
    encoded_len(z21_proto::encode_status(central_state(status), out))
}

fn lease_check(ctx: &Z21Ctx<'_>) -> LeaseCheck {
    match ctx.lease_permit {
        None => LeaseCheck::None,
        Some(permit) if crate::track_authority::accepts(permit) => LeaseCheck::Valid(permit),
        Some(_) => LeaseCheck::Expired,
    }
}

fn warn_refused(reason: RefuseReason) {
    match reason {
        RefuseReason::NoLease => {
            defmt::warn!("Z21 power-on rejected without a controller lease");
        }
        RefuseReason::ExpiredLease => {
            defmt::warn!("Z21 power-on rejected with an expired controller lease");
        }
        RefuseReason::Inhibited => {
            defmt::warn!("Z21 power-on refused: an update holds the track off");
        }
    }
}

pub(super) async fn apply_power_request(
    request: TrackPowerRequest,
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
) -> usize {
    let (lease, inhibited) = if matches!(request, TrackPowerRequest::Enable) {
        (lease_check(ctx), crate::track_output::inhibited())
    } else {
        (LeaseCheck::None, false)
    };
    let decision = match judge_track_power(request, lease, inhibited) {
        TrackPowerVerdict::Refuse(reason) => {
            warn_refused(reason);
            return match refusal_reply(reason) {
                RefusalReply::Silent => 0,
                RefusalReply::TrackPowerOff => {
                    encoded_len(z21_proto::encode_bc_track_power(false, out))
                }
            };
        }
        TrackPowerVerdict::Apply(decision) => decision,
    };
    if decision.disable_output_immediately {
        crate::track_safety::disable_track_intentionally();
    }
    ctx.fault_sender.send(decision.fault_event).await;
    match decision.feedback {
        TrackPowerFeedback::None => 0,
        TrackPowerFeedback::Stopped => encoded_len(z21_proto::encode_bc_stopped(out)),
    }
}
