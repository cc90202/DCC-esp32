//! Z21 frame parsing and command-family routing.
//!
//! ESP32-C6 only; command behavior lives in focused controllers below this
//! module, while POM delegates to its existing scheduler/RailCom adapter.

use defmt::{info, warn};

use crate::application::LocoSlots;
use crate::application::track_control::TrackPowerRequest;
use crate::net::z21_context::Z21Ctx;
use crate::z21::{self as z21_proto, HEADER_XBUS, Z21Command};

use super::pom_client::{handle_cv_pom_read, handle_cv_pom_write};

mod device;
mod locomotive;
mod railcom;
mod track;

pub(crate) use locomotive::loco_command_rejected_count;
pub(crate) use railcom::railcom_getdata_no_data_count;
pub(super) use track::encode_system_state;

/// Parse one incoming Z21 frame, route it, and build any immediate response.
pub(super) async fn handle_packet(
    buf: &[u8],
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
) -> usize {
    let command = match z21_proto::parse_frame(buf) {
        Ok(command) => command,
        Err(error) => {
            warn!("Z21 parse error: {:?}", error);
            return z21_proto::encode_unknown_command(out);
        }
    };

    log_command(command);
    route_command(command, buf, loco_slots, out, ctx).await
}

fn log_command(command: Z21Command) {
    if !matches!(command, Z21Command::GetSystemState | Z21Command::GetStatus) {
        info!("Z21 cmd: {:?}", command);
    }
}

async fn route_command(
    command: Z21Command,
    raw_frame: &[u8],
    loco_slots: &mut LocoSlots,
    out: &mut [u8],
    ctx: &Z21Ctx<'_>,
) -> usize {
    match command {
        Z21Command::GetSerialNumber => device::encode_serial_number(out),
        Z21Command::GetCode => device::encode_code(out),
        Z21Command::GetHwInfo => device::encode_hardware_info(out),
        Z21Command::GetXBusVersion => device::encode_xbus_version(out),
        Z21Command::GetFirmwareVersion => device::encode_firmware_version(out),
        Z21Command::GetTurnoutInfo { address } => device::encode_unknown_turnout(address, out),

        Z21Command::GetSystemState => track::encode_current_system_state(ctx.status_model, out),
        // Z21 apps expect an immediate state push after subscription.
        Z21Command::SetBroadcastFlags => track::encode_current_system_state(ctx.status_model, out),
        Z21Command::GetStatus => track::encode_current_status(ctx.status_model, out),
        Z21Command::SetTrackPowerOn => {
            track::apply_power_request(TrackPowerRequest::Enable, out, ctx).await
        }
        Z21Command::SetTrackPowerOff => {
            track::apply_power_request(TrackPowerRequest::Disable, out, ctx).await
        }
        Z21Command::SetStop => {
            track::apply_power_request(TrackPowerRequest::EmergencyStop, out, ctx).await
        }

        Z21Command::SetLocoEstop { address } => {
            locomotive::emergency_stop(loco_slots, out, ctx, address).await
        }
        Z21Command::GetLocoMode { address } => locomotive::encode_mode(address, out),
        Z21Command::SetLocoMode { address, mode } => locomotive::set_mode(address, mode),
        Z21Command::GetLocoInfo { address } => {
            locomotive::get_info(loco_slots, out, ctx, address).await
        }
        Z21Command::SetLocoDrive {
            address,
            speed,
            direction,
            format,
        } => locomotive::set_drive(loco_slots, out, ctx, address, speed, direction, format).await,
        Z21Command::SetLocoFunction {
            address,
            function,
            action,
        } => locomotive::set_function(loco_slots, out, ctx, address, function, action).await,

        Z21Command::CvPomWriteByte { address, cv, value } => {
            handle_cv_pom_write(out, ctx, address, cv, value).await
        }
        Z21Command::CvPomReadByte { address, cv } => {
            handle_cv_pom_read(loco_slots, out, ctx, address, cv).await
        }

        Z21Command::RailcomGetData {
            request_type,
            address,
        } => railcom::encode_getdata_response(request_type, address, out),

        Z21Command::Logoff | Z21Command::LoconetDetector => 0,
        Z21Command::Unknown => {
            log_unknown_command(raw_frame);
            0
        }
    }
}

fn log_unknown_command(buf: &[u8]) {
    let kind = z21_proto::frame_kind(buf);

    if kind.header == HEADER_XBUS {
        warn!(
            "Z21 unknown XBus: xheader=0x{:02X} db0=0x{:02X} len={}",
            kind.xheader,
            if buf.len() >= 6 { buf[5] } else { 0 },
            buf.len()
        );
    } else {
        warn!(
            "Z21 unknown top-level: header=0x{:04X} len={}",
            kind.header,
            buf.len()
        );
    }
}
