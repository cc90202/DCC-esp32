//! Z21 adapter for track-power commands and status responses.

use crate::application::StatusModel;
use crate::application::track_control::{
    TrackPowerFeedback, TrackPowerRequest, TrackStatus, decide_track_power,
};
use crate::net::z21_context::TrackCtx;
use crate::z21 as z21_proto;

use super::encoded_len;

pub(super) fn encode_current_system_state(status_model: &StatusModel, out: &mut [u8]) -> usize {
    encode_system_state(TrackStatus::from_model(*status_model), out)
}

pub(in crate::net) fn encode_system_state(status: TrackStatus, out: &mut [u8]) -> usize {
    encoded_len(z21_proto::encode_system_state(
        status.power_on,
        status.estop_active,
        status.short_circuit,
        out,
    ))
}

pub(super) fn encode_current_status(status_model: &StatusModel, out: &mut [u8]) -> usize {
    let status = TrackStatus::from_model(*status_model);
    encoded_len(z21_proto::encode_status(
        status.power_on,
        status.estop_active,
        out,
    ))
}

pub(super) async fn apply_power_request(
    request: TrackPowerRequest,
    out: &mut [u8],
    ctx: &TrackCtx<'_>,
) -> usize {
    let decision = decide_track_power(request);
    if decision.disable_output_immediately {
        crate::track_safety::disable_track_intentionally();
    }
    ctx.fault_sender.send(decision.fault_event).await;
    match decision.feedback {
        TrackPowerFeedback::None => 0,
        TrackPowerFeedback::Stopped => encoded_len(z21_proto::encode_bc_stopped(out)),
    }
}
