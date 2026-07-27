//! Z21 controller for scheduler-backed locomotive commands.

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::{info, warn};

use crate::application::locomotive::{
    LocoCommandFailure, LocoCommandOutcome, LocoLookupIssue, LocoRequestError,
    prepare_drive_request, prepare_function_request, resolve_loco_command, resolve_loco_lookup,
};
use crate::application::{LocoSlots, LocoState};
use crate::dcc::{DccAddress, Direction, FunctionChange, LocoRequest, SpeedFormat};
use crate::net::loco_client::request_loco;
use crate::net::z21_context::LocoCtx;
use crate::z21::{self as z21_proto, FunctionAction};

use super::encoded_len;

static LOCO_COMMAND_REJECTED_COUNT: AtomicU32 = AtomicU32::new(0);

#[must_use]
pub(crate) fn loco_command_rejected_count() -> u32 {
    LOCO_COMMAND_REJECTED_COUNT.load(Ordering::Acquire)
}

pub(super) fn encode_mode(address: DccAddress, out: &mut [u8]) -> usize {
    encoded_len(z21_proto::encode_loco_mode(address, out))
}

pub(super) fn set_mode(address: DccAddress, mode: u8) -> usize {
    if mode != 0 {
        warn!(
            "loco mode addr={} unsupported mode={} (DCC only)",
            address.value(),
            mode
        );
    }
    0
}

pub(super) async fn get_info(
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &LocoCtx<'_>,
    address: DccAddress,
) -> usize {
    let response = request_loco(
        ctx.request_sender,
        ctx.response_receiver,
        ctx.next_request_id,
        LocoRequest::GetState { address },
    )
    .await;
    let outcome = resolve_loco_lookup(
        loco_slots,
        address,
        response.map(|response| response.result),
    );
    if let Some(issue) = outcome.issue {
        match issue {
            LocoLookupIssue::SchedulerTimeout => warn!(
                "loco info addr={} scheduler response timed out",
                address.value()
            ),
            LocoLookupIssue::UnexpectedResult(result) => warn!(
                "loco info addr={} unexpected scheduler result={:?}",
                address.value(),
                result
            ),
            LocoLookupIssue::Projection(error) => warn!(
                "loco info addr={} projection failed: {:?}",
                address.value(),
                error
            ),
        }
    }
    encoded_len(z21_proto::encode_loco_info(&loco_info(outcome.state), out))
}

pub(super) async fn emergency_stop(
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &LocoCtx<'_>,
    address: DccAddress,
) -> usize {
    let outcome = apply_requested_change(
        loco_slots,
        ctx,
        LocoRequest::EmergencyStop { address },
        "emergency stop",
    )
    .await;
    encode_confirmed_feedback(outcome, out)
}

pub(super) async fn set_drive(
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &LocoCtx<'_>,
    address: DccAddress,
    speed: u8,
    direction: Direction,
    format: SpeedFormat,
) -> usize {
    let request = match prepare_drive_request(
        ctx.status_model.track_power_on(),
        address,
        speed,
        direction,
        format,
    ) {
        Ok(request) => request,
        Err(error) => {
            report_request_error(ctx, address, "drive", error);
            return 0;
        }
    };

    let outcome = apply_requested_change(loco_slots, ctx, request, "drive").await;
    if outcome.confirmed_state().is_some() {
        info!(
            "loco drive addr={} fmt={:?} dir={:?} speed={}",
            address.value(),
            format,
            direction,
            speed
        );
    }
    encode_confirmed_feedback(outcome, out)
}

pub(super) async fn set_function(
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &LocoCtx<'_>,
    address: DccAddress,
    function: u8,
    action: FunctionAction,
) -> usize {
    let change = match action {
        FunctionAction::On => FunctionChange::Enable,
        FunctionAction::Off => FunctionChange::Disable,
        FunctionAction::Toggle => FunctionChange::Toggle,
    };
    let request = match prepare_function_request(
        ctx.status_model.track_power_on(),
        address,
        function,
        change,
    ) {
        Ok(request) => request,
        Err(error) => {
            report_request_error(ctx, address, "function", error);
            return 0;
        }
    };

    let outcome = apply_requested_change(loco_slots, ctx, request, "function").await;
    if let Some(state) = outcome.confirmed_state() {
        info!(
            "loco fn addr={} f{} action={:?} enabled={} speed={} fmt={:?}",
            address.value(),
            function,
            action,
            (state.functions & (1u32 << function)) != 0,
            state.speed.value(),
            state.speed.format()
        );
    }
    encode_confirmed_feedback(outcome, out)
}

fn report_request_error(
    ctx: &LocoCtx<'_>,
    address: DccAddress,
    command_kind: &str,
    error: LocoRequestError,
) {
    match error {
        LocoRequestError::TrackPowerOff => warn!(
            "loco {} addr={} rejected while track power is OFF (estop={} short={} track_on={})",
            command_kind,
            address.value(),
            ctx.status_model.estop_active(),
            ctx.status_model.short_circuit(),
            ctx.status_model.track_power_on()
        ),
        LocoRequestError::InvalidSpeed { speed, format } => warn!(
            "loco drive addr={} speed={} invalid for format={:?}, command dropped",
            address.value(),
            speed,
            format
        ),
        LocoRequestError::InvalidFunction { function } => warn!(
            "loco function addr={} rejected: unsupported function index {}",
            address.value(),
            function
        ),
    }
    LOCO_COMMAND_REJECTED_COUNT.fetch_add(1, Ordering::Relaxed);
}

fn encode_confirmed_feedback(outcome: LocoCommandOutcome, out: &mut [u8]) -> usize {
    outcome.confirmed_state().map_or(0, |state: LocoState| {
        encoded_len(z21_proto::encode_loco_info(&loco_info(state), out))
    })
}

fn loco_info(state: LocoState) -> z21_proto::LocoInfo {
    z21_proto::LocoInfo {
        address: state.address,
        speed: state.speed,
        direction: state.direction,
        functions: state.functions,
    }
}

async fn apply_requested_change(
    loco_slots: &mut LocoSlots,
    ctx: &LocoCtx<'_>,
    request: LocoRequest,
    command_kind: &str,
) -> LocoCommandOutcome {
    let address = request.address();
    let response = request_loco(
        ctx.request_sender,
        ctx.response_receiver,
        ctx.next_request_id,
        request,
    )
    .await;

    let outcome = resolve_loco_command(
        loco_slots,
        address,
        response.map(|response| response.result),
    );
    match outcome {
        LocoCommandOutcome::Applied(_) => outcome,
        LocoCommandOutcome::Rejected(failure) => {
            LOCO_COMMAND_REJECTED_COUNT.fetch_add(1, Ordering::Relaxed);
            match failure {
                LocoCommandFailure::SchedulerTimeout => warn!(
                    "loco {} addr={} rejected: scheduler response timed out",
                    command_kind,
                    address.value()
                ),
                LocoCommandFailure::SchedulerRejected(result) => warn!(
                    "loco {} addr={} rejected by scheduler: {:?}",
                    command_kind,
                    address.value(),
                    result
                ),
                LocoCommandFailure::Projection(error) => warn!(
                    "loco {} addr={} accepted but projection failed: {:?}",
                    command_kind,
                    address.value(),
                    error
                ),
            }
            outcome
        }
    }
}
