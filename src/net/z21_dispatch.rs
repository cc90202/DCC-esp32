//! Z21 command dispatch: frame handling and per-command handlers.
//!
//! ESP32-C6 only - gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

use defmt::{info, warn};

use crate::dcc::{
    DccAddress, Direction, FunctionIndex, LogicalSpeed, SchedulerCommand, SpeedFormat,
};
use crate::net::udp_control::Z21Ctx;
use crate::net::z21_proto::{self, FunctionAction, HEADER_XBUS, Z21Command};
use crate::net::{LocoSlots, LocoState, loco_commands_allowed};
use crate::railcom::loco_tracker::RailcomLocoSighting;
use crate::system_status::{FaultEvent, StatusModel, SystemStatusEvent};

use super::pom_client::{handle_cv_pom_read, handle_cv_pom_write};
use super::railcom_lookup::{
    latest_railcom_sighting, railcom_request_address, railcom_sighting_for_address,
};

/// The one RailCom GETDATA request type we support - a poll for the cached
/// sighting of a single (or, if address is 0, the most recently seen) loco.
const RAILCOM_GETDATA_TYPE_LOCO: u8 = 0x01;

/// Arbitrary device serial reported to Z21 apps (no real meaning).
const DEFAULT_Z21_SERIAL_NUMBER: u32 = 0xC0FFEE01;

/// Process one incoming Z21 frame; dispatch commands and build response.
pub(super) async fn handle_packet(
    buf: &[u8],
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
) -> usize {
    let cmd = match z21_proto::parse_frame(buf) {
        Ok(c) => c,
        Err(e) => {
            warn!("Z21 parse error: {:?}", e);
            return z21_proto::encode_unknown_command(out);
        }
    };

    log_command(cmd);
    dispatch_command(cmd, buf, loco_slots, out, ctx).await
}

fn log_command(cmd: Z21Command) {
    // Polling commands arrive every ~1s - avoid log noise.
    if !matches!(cmd, Z21Command::GetSystemState | Z21Command::GetStatus) {
        info!("Z21 cmd: {:?}", cmd);
    }
}

pub(super) fn encode_current_system_state(status_model: &StatusModel, out: &mut [u8]) -> usize {
    z21_proto::encode_system_state(
        status_model.track_power_on(),
        status_model.estop_active(),
        status_model.short_circuit(),
        out,
    )
}

fn encode_current_status(status_model: &StatusModel, out: &mut [u8]) -> usize {
    z21_proto::encode_status(
        status_model.track_power_on(),
        status_model.estop_active(),
        out,
    )
}

async fn dispatch_command(
    cmd: Z21Command,
    raw_frame: &[u8],
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
) -> usize {
    match cmd {
        Z21Command::GetSerialNumber => {
            z21_proto::encode_serial_number(DEFAULT_Z21_SERIAL_NUMBER, out)
        }
        Z21Command::GetCode => z21_proto::encode_code(out),
        Z21Command::GetHwInfo => z21_proto::encode_hwinfo(out),
        Z21Command::GetSystemState => encode_current_system_state(ctx.status_model, out),
        Z21Command::Logoff => 0,
        // After SetBroadcastFlags the app expects an immediate unsolicited
        // LAN_SYSTEMSTATE_DATACHANGED push to learn current track state.
        // Without it the app waits indefinitely and never enters main UI.
        Z21Command::SetBroadcastFlags { .. } => encode_current_system_state(ctx.status_model, out),
        Z21Command::GetXBusVersion => z21_proto::encode_xbus_version(out),
        Z21Command::GetFirmwareVersion => z21_proto::encode_firmware_version(out),
        Z21Command::GetStatus => encode_current_status(ctx.status_model, out),
        Z21Command::SetTrackPowerOn => {
            ctx.fault_sender.send(FaultEvent::ResumeShortPressed).await;
            0
        }
        Z21Command::SetTrackPowerOff => {
            ctx.fault_sender.send(FaultEvent::StopPressed).await;
            0
        }
        Z21Command::SetStop => {
            ctx.fault_sender.send(FaultEvent::StopPressed).await;
            z21_proto::encode_bc_stopped(out)
        }
        Z21Command::SetLocoEstop { address } => {
            ctx.scheduler_sender
                .send(SchedulerCommand::EmergencyStop { address })
                .await;
            z21_proto::encode_bc_stopped(out)
        }
        Z21Command::GetLocoMode { address } => z21_proto::encode_loco_mode(address, out),
        Z21Command::SetLocoMode { address, mode } => {
            if mode != 0 {
                warn!(
                    "loco mode addr={} unsupported mode={} (DCC only)",
                    address.value(),
                    mode
                );
            }
            0
        }
        Z21Command::GetLocoInfo { address } => {
            let known_to_net = z21_proto::find_slot(loco_slots, address).is_some();
            if let Some(state) = z21_proto::find_or_insert(loco_slots, address) {
                let state = *state;
                let len = z21_proto::encode_loco_info(&state, out);
                if !known_to_net {
                    info!(
                        "loco info addr={} created DCC refresh slot",
                        address.value()
                    );
                    if let Some(speed) = LogicalSpeed::new(state.speed, state.format) {
                        ctx.scheduler_sender
                            .send(SchedulerCommand::SetSpeed {
                                address,
                                speed,
                                direction: state.direction,
                                format: state.format,
                            })
                            .await;
                    } else {
                        warn!(
                            "loco info addr={} speed={} invalid for format={:?}, refresh slot not seeded",
                            address.value(),
                            state.speed,
                            state.format
                        );
                    }
                }
                len
            } else {
                z21_proto::encode_unknown_command(out)
            }
        }
        Z21Command::SetLocoDrive {
            address,
            speed,
            direction,
            format,
        } => handle_set_loco_drive(loco_slots, out, ctx, address, speed, direction, format).await,
        Z21Command::SetLocoFunction {
            address,
            function,
            action,
        } => handle_set_loco_function(loco_slots, out, ctx, address, function, action).await,
        Z21Command::CvPomWriteByte { address, cv, value } => {
            handle_cv_pom_write(out, ctx, address, cv, value).await
        }
        Z21Command::CvPomReadByte { address, cv } => {
            handle_cv_pom_read(loco_slots, out, ctx, address, cv).await
        }
        // Turnout info - no accessory decoder support; reply with state=unknown to keep
        // the app in sync and suppress repeated warn logs.
        Z21Command::GetTurnoutInfo { address } => z21_proto::encode_turnout_info(address, out),
        Z21Command::RailcomGetData {
            request_type,
            address,
        } => encode_railcom_getdata_response(request_type, address, out),
        Z21Command::LoconetDetector => 0,
        Z21Command::Unknown => {
            log_unknown_command(raw_frame);
            0
        }
    }
}

fn encode_railcom_getdata_response(request_type: u8, raw_address: u16, out: &mut [u8]) -> usize {
    if request_type != RAILCOM_GETDATA_TYPE_LOCO {
        warn!("RailCom GETDATA unsupported type={}", request_type);
        return 0;
    }

    let sighting: Option<RailcomLocoSighting> = match railcom_request_address(raw_address) {
        Some(address) => railcom_sighting_for_address(address),
        None => latest_railcom_sighting(),
    };

    if let Some(sighting) = sighting {
        z21_proto::encode_railcom_data(sighting.address, sighting.seen_count, 0, out)
    } else {
        0
    }
}

fn encode_rejected_loco_info(loco_slots: &LocoSlots, out: &mut [u8], address: DccAddress) -> usize {
    if let Some(state) = z21_proto::find_slot(loco_slots, address) {
        return z21_proto::encode_loco_info(state, out);
    }

    let state = LocoState {
        address,
        speed: 0,
        direction: Direction::Forward,
        format: SpeedFormat::Speed128,
        functions: 0,
    };
    z21_proto::encode_loco_info(&state, out)
}

/// Reject a loco drive/function command while track power is off, logging why.
///
/// Shared by `handle_set_loco_drive` and `handle_set_loco_function`, which
/// both need to reject identically (same warn shape, same rejected-info
/// response) - only the "drive"/"function" word in the log differs.
fn reject_if_track_off(
    ctx: &Z21Ctx<'_>,
    loco_slots: &LocoSlots,
    out: &mut [u8],
    address: DccAddress,
    command_kind: &str,
) -> Option<usize> {
    if loco_commands_allowed(ctx.status_model.track_power_on()) {
        return None;
    }
    warn!(
        "loco {} addr={} rejected while track power is OFF (estop={} short={} track_on={})",
        command_kind,
        address.value(),
        ctx.status_model.estop_active(),
        ctx.status_model.short_circuit(),
        ctx.status_model.track_power_on()
    );
    Some(encode_rejected_loco_info(loco_slots, out, address))
}

async fn handle_set_loco_drive(
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
    address: DccAddress,
    speed: u8,
    direction: Direction,
    format: SpeedFormat,
) -> usize {
    if let Some(resp) = reject_if_track_off(ctx, loco_slots, out, address, "drive") {
        return resp;
    }

    let Some(logical_speed) = LogicalSpeed::new(speed, format) else {
        warn!(
            "loco drive addr={} speed={} invalid for format={:?}, command dropped",
            address.value(),
            speed,
            format
        );
        return encode_rejected_loco_info(loco_slots, out, address);
    };

    let Some(slot) = z21_proto::find_or_insert(loco_slots, address) else {
        warn!("SetLocoDrive: all 12 slots full and running, command dropped");
        return 0;
    };

    slot.speed = logical_speed.value();
    slot.direction = direction;
    slot.format = format;

    info!(
        "loco drive addr={} fmt={:?} dir={:?} speed={}",
        address.value(),
        format,
        direction,
        speed
    );
    ctx.scheduler_sender
        .send(SchedulerCommand::SetSpeed {
            address,
            speed: logical_speed,
            direction,
            format,
        })
        .await;

    z21_proto::encode_loco_info(slot, out)
}

// Log raw header/xheader to help identify protocol gaps.
fn log_unknown_command(buf: &[u8]) {
    let header = if buf.len() >= 4 {
        u16::from_le_bytes([buf[2], buf[3]])
    } else {
        0
    };
    let xheader = if buf.len() >= 5 && header == HEADER_XBUS {
        buf[4]
    } else {
        0
    };

    if header == HEADER_XBUS {
        warn!(
            "Z21 unknown XBus: xheader=0x{:02X} db0=0x{:02X} len={}",
            xheader,
            if buf.len() >= 6 { buf[5] } else { 0 },
            buf.len()
        );
    } else {
        warn!(
            "Z21 unknown top-level: header=0x{:04X} len={}",
            header,
            buf.len()
        );
    }
}

async fn handle_set_loco_function(
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
    address: DccAddress,
    function: u8,
    action: FunctionAction,
) -> usize {
    if let Some(resp) = reject_if_track_off(ctx, loco_slots, out, address, "function") {
        return resp;
    }

    let Some(fi) = FunctionIndex::new(function) else {
        warn!(
            "loco function addr={} rejected: unsupported function index {}",
            address.value(),
            function
        );
        return encode_rejected_loco_info(loco_slots, out, address);
    };

    let Some(slot) = z21_proto::find_or_insert(loco_slots, address) else {
        return 0;
    };

    let bit = 1u32 << function;
    let enabled = match action {
        FunctionAction::On => {
            slot.functions |= bit;
            true
        }
        FunctionAction::Off => {
            slot.functions &= !bit;
            false
        }
        FunctionAction::Toggle => {
            let was_enabled = (slot.functions & bit) != 0;
            if was_enabled {
                slot.functions &= !bit;
            } else {
                slot.functions |= bit;
            }
            !was_enabled
        }
    };

    info!(
        "loco fn addr={} f{} action={:?} enabled={} speed={} fmt={:?}",
        address.value(),
        function,
        action,
        enabled,
        slot.speed,
        slot.format
    );
    ctx.scheduler_sender
        .send(SchedulerCommand::SetFunction {
            address,
            function: fi,
            enabled,
        })
        .await;

    z21_proto::encode_loco_info(slot, out)
}

/// Encode a broadcast response for a SystemStatusEvent.
///
/// `FaultLatched` and `FaultCleared` are handled inline in the event loop
/// (they require multiple frames including `LAN_SYSTEMSTATE_DATACHANGED`).
pub(super) fn handle_status_event(event: SystemStatusEvent, out: &mut [u8]) -> usize {
    match event {
        SystemStatusEvent::EstopActive => z21_proto::encode_bc_stopped(out),
        SystemStatusEvent::EstopCleared => z21_proto::encode_bc_track_power(true, out),
        _ => 0,
    }
}
