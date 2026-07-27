//! Z21 controller for cached RailCom locomotive data.

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::warn;

use crate::dcc::DccAddress;
use crate::net::railcom_lookup::{latest_railcom_sighting, railcom_sighting_for_address};
use crate::railcom::loco_tracker::RailcomLocoSighting;
use crate::z21::{self as z21_proto, RailcomRequestType};

use super::encoded_len;

static RAILCOM_GETDATA_NO_DATA_COUNT: AtomicU32 = AtomicU32::new(0);

#[must_use]
pub(crate) fn railcom_getdata_no_data_count() -> u32 {
    RAILCOM_GETDATA_NO_DATA_COUNT.load(Ordering::Acquire)
}

pub(super) fn encode_getdata_response(
    request_type: RailcomRequestType,
    address: Option<DccAddress>,
    out: &mut [u8],
) -> usize {
    if let RailcomRequestType::Unsupported(raw) = request_type {
        warn!("RailCom GETDATA unsupported type={}", raw);
        return 0;
    }

    let sighting: Option<RailcomLocoSighting> = match address {
        Some(address) => railcom_sighting_for_address(address),
        None => latest_railcom_sighting(),
    };

    if let Some(sighting) = sighting {
        encoded_len(z21_proto::encode_railcom_data(
            sighting.address,
            sighting.seen_count,
            0,
            out,
        ))
    } else {
        RAILCOM_GETDATA_NO_DATA_COUNT.fetch_add(1, Ordering::Relaxed);
        0
    }
}
