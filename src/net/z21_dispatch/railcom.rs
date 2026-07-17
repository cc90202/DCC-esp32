//! Z21 controller for cached RailCom locomotive data.

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::warn;

use crate::dcc::DccAddress;
use crate::net::railcom_lookup::{latest_railcom_sighting, railcom_sighting_for_address};
use crate::railcom::loco_tracker::RailcomLocoSighting;
use crate::z21 as z21_proto;

const RAILCOM_GETDATA_TYPE_LOCO: u8 = 0x01;
static RAILCOM_GETDATA_NO_DATA_COUNT: AtomicU32 = AtomicU32::new(0);

#[must_use]
pub(crate) fn railcom_getdata_no_data_count() -> u32 {
    RAILCOM_GETDATA_NO_DATA_COUNT.load(Ordering::Acquire)
}

pub(super) fn encode_getdata_response(
    request_type: u8,
    address: Option<DccAddress>,
    out: &mut [u8],
) -> usize {
    if request_type != RAILCOM_GETDATA_TYPE_LOCO {
        warn!("RailCom GETDATA unsupported type={}", request_type);
        return 0;
    }

    let sighting: Option<RailcomLocoSighting> = match address {
        Some(address) => railcom_sighting_for_address(address),
        None => latest_railcom_sighting(),
    };

    if let Some(sighting) = sighting {
        z21_proto::encode_railcom_data(sighting.address, sighting.seen_count, 0, out)
    } else {
        RAILCOM_GETDATA_NO_DATA_COUNT.fetch_add(1, Ordering::Relaxed);
        0
    }
}
