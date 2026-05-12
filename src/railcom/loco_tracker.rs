use core::cell::RefCell;

use critical_section::Mutex;

use crate::dcc::DccAddress;
use crate::railcom::parser::{RailcomDatagram, RailcomItem};

// Number of most-recently-identified locos kept for display/diagnostics.
//
// TODO(tuning): no recorded derivation for `4`. Observable constraint: this
// is a rolling "who is on the track right now" display list, not a full
// roster — a handful of entries is enough to show recent activity on a
// hobby-scale layout without growing the stack-resident stats struct.
const RECENT_SIGHTING_CAPACITY: usize = 4;
const PENDING_ADR_HIGH_MAX_PACKET_GAP: u32 = 8;

// Maximum packet-boundary gap allowed between an ADR-HIGH fragment (channel 1)
// and the ADR-LOW fragment that completes a long-address identification.
//
// TODO(tuning): no recorded derivation for `8`. Observable constraint: the
// two fragments are expected to arrive on the very next RailCom-eligible
// packet from the same decoder; 8 packet boundaries gives slack for a couple
// of skipped/collided cutouts while still discarding genuinely stale
// fragments before they could be paired with an unrelated decoder's ADR-LOW.
const PENDING_ADR_HIGH_MAX_PACKET_GAP: u32 = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomLocoSighting {
    pub address: DccAddress,
    pub packet_sequence: u32,
    pub seen_count: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomLocoTrackerStats {
    pub identification_window_count: u32,
    pub identified_loco_count: u32,
    pub invalid_identification_count: u32,
    pub recent_sightings: [Option<RailcomLocoSighting>; RECENT_SIGHTING_CAPACITY],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct TrackerState {
    identification_window_count: u32,
    identified_loco_count: u32,
    invalid_identification_count: u32,
    recent_sightings: [Option<RailcomLocoSighting>; RECENT_SIGHTING_CAPACITY],
    next_insert_index: usize,
    pending_adr_high: Option<PendingAdrHigh>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PendingAdrHigh {
    value: u8,
    packet_sequence: u32,
}

impl TrackerState {
    const fn new() -> Self {
        Self {
            identification_window_count: 0,
            identified_loco_count: 0,
            invalid_identification_count: 0,
            recent_sightings: [None; RECENT_SIGHTING_CAPACITY],
            next_insert_index: 0,
            pending_adr_high: None,
        }
    }

    fn snapshot(&self) -> RailcomLocoTrackerStats {
        RailcomLocoTrackerStats {
            identification_window_count: self.identification_window_count,
            identified_loco_count: self.identified_loco_count,
            invalid_identification_count: self.invalid_identification_count,
            recent_sightings: self.recent_sightings,
        }
    }

    fn record_identified(
        &mut self,
        packet_sequence: u32,
        address: DccAddress,
    ) -> RailcomLocoSighting {
        self.identification_window_count = self.identification_window_count.wrapping_add(1);
        self.identified_loco_count = self.identified_loco_count.wrapping_add(1);

        for sighting in self.recent_sightings.iter_mut().flatten() {
            if sighting.address == address {
                sighting.packet_sequence = packet_sequence;
                sighting.seen_count = sighting.seen_count.wrapping_add(1);
                return *sighting;
            }
        }

        let sighting = RailcomLocoSighting {
            address,
            packet_sequence,
            seen_count: 1,
        };
        self.recent_sightings[self.next_insert_index] = Some(sighting);
        self.next_insert_index = (self.next_insert_index + 1) % RECENT_SIGHTING_CAPACITY;
        sighting
    }

    fn record_invalid(&mut self) {
        self.identification_window_count = self.identification_window_count.wrapping_add(1);
        self.invalid_identification_count = self.invalid_identification_count.wrapping_add(1);
    }

    fn record_pending_high(&mut self, packet_sequence: u32, value: u8) {
        self.identification_window_count = self.identification_window_count.wrapping_add(1);
        self.pending_adr_high = Some(PendingAdrHigh {
            value,
            packet_sequence,
        });
    }

    fn take_recent_pending_high(&mut self, packet_sequence: u32) -> Option<u8> {
        let pending = self.pending_adr_high?;
        let age = packet_sequence.wrapping_sub(pending.packet_sequence);
        if age <= PENDING_ADR_HIGH_MAX_PACKET_GAP {
            self.pending_adr_high = None;
            Some(pending.value)
        } else {
            self.pending_adr_high = None;
            None
        }
    }
}

static TRACKER_STATE: Mutex<RefCell<TrackerState>> = Mutex::new(RefCell::new(TrackerState::new()));

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AddressCandidate {
    None,
    PresentButInvalid,
    Identified(DccAddress),
    PendingHigh(u8),
}

fn decode_railcom_address(adr_high: Option<u8>, adr_low: Option<u8>) -> Option<DccAddress> {
    match (adr_high, adr_low) {
        (Some(high), Some(low)) => {
            let raw = ((u16::from(high) & 0x3f) << 8) | u16::from(low);
            if raw <= 127 {
                DccAddress::new_short(raw as u8)
            } else {
                DccAddress::new_long(raw)
            }
        }
        (None, Some(low)) => DccAddress::new_short(low),
        _ => None,
    }
}

fn address_fragment_matches_target(
    target_address: DccAddress,
    adr_high: Option<u8>,
    adr_low: Option<u8>,
) -> Option<bool> {
    let target = target_address.value();
    let expected_high = ((target >> 8) & 0x3f) as u8;
    let expected_low = (target & 0xff) as u8;

    match (adr_high, adr_low) {
        (Some(high), Some(low)) => Some(high == expected_high && low == expected_low),
        (None, Some(low)) if target_address.is_short() => Some(low == expected_low),
        (None, Some(low)) => {
            if low == expected_low {
                None
            } else {
                Some(false)
            }
        }
        (Some(high), None) => {
            if high == expected_high {
                None
            } else {
                Some(false)
            }
        }
        (None, None) => None,
    }
}

#[must_use]
pub fn identify_loco_from_items(items: &[RailcomItem]) -> Option<DccAddress> {
    match classify_address_candidate(items) {
        AddressCandidate::Identified(address) => Some(address),
        AddressCandidate::None
        | AddressCandidate::PresentButInvalid
        | AddressCandidate::PendingHigh(_) => None,
    }
}

fn address_parts(items: &[RailcomItem]) -> (Option<u8>, Option<u8>, bool) {
    let mut adr_high = None;
    let mut adr_low = None;
    let mut saw_address_datagram = false;

    for item in items {
        let RailcomItem::Datagram(datagram) = item else {
            continue;
        };

        match datagram {
            RailcomDatagram::AdrHigh(high) => {
                adr_high = Some(*high);
                saw_address_datagram = true;
            }
            RailcomDatagram::AdrLow(low) => {
                adr_low = Some(*low);
                saw_address_datagram = true;
            }
            RailcomDatagram::Ext(_)
            | RailcomDatagram::Dyn { .. }
            | RailcomDatagram::Xpom { .. }
            | RailcomDatagram::CvData(_)
            | RailcomDatagram::Search(_) => {}
        }
    }

    (adr_high, adr_low, saw_address_datagram)
}

fn classify_address_candidate(items: &[RailcomItem]) -> AddressCandidate {
    let (adr_high, adr_low, saw_address_datagram) = address_parts(items);

    if !saw_address_datagram {
        return AddressCandidate::None;
    }

    match (adr_high, adr_low) {
        (Some(high), None) => AddressCandidate::PendingHigh(high),
        _ => match decode_railcom_address(adr_high, adr_low) {
            Some(address) => AddressCandidate::Identified(address),
            None => AddressCandidate::PresentButInvalid,
        },
    }
}

pub fn record_target_from_matching_address_fragment(
    packet_sequence: u32,
    target_address: DccAddress,
    items: &[RailcomItem],
) -> Option<RailcomLocoSighting> {
    let (adr_high, adr_low, saw_address_datagram) = address_parts(items);
    if !saw_address_datagram {
        return None;
    }

    critical_section::with(|cs| {
        let mut state = TRACKER_STATE.borrow_ref_mut(cs);
        match address_fragment_matches_target(target_address, adr_high, adr_low) {
            Some(true) => Some(state.record_identified(packet_sequence, target_address)),
            Some(false) => {
                state.record_invalid();
                None
            }
            None => None,
        }
    })
}

pub fn record_loco_identification(
    packet_sequence: u32,
    items: &[RailcomItem],
) -> Option<RailcomLocoSighting> {
    critical_section::with(|cs| {
        let mut state = TRACKER_STATE.borrow_ref_mut(cs);
        let (adr_high, adr_low, saw_address_datagram) = address_parts(items);
        if !saw_address_datagram {
            return None;
        }

        if let Some(low) = adr_low {
            let high = adr_high.or_else(|| state.take_recent_pending_high(packet_sequence));
            return match decode_railcom_address(high, Some(low)) {
                Some(address) => Some(state.record_identified(packet_sequence, address)),
                None => {
                    state.record_invalid();
                    None
                }
            };
        }

        match classify_address_candidate(items) {
            AddressCandidate::PresentButInvalid => {
                state.record_invalid();
                None
            }
            AddressCandidate::PendingHigh(high) => {
                state.record_pending_high(packet_sequence, high);
                None
            }
            AddressCandidate::None | AddressCandidate::Identified(_) => None,
        }
    })
}

pub fn record_identified_address(packet_sequence: u32, address: DccAddress) -> RailcomLocoSighting {
    critical_section::with(|cs| {
        TRACKER_STATE
            .borrow_ref_mut(cs)
            .record_identified(packet_sequence, address)
    })
}

#[must_use]
pub fn railcom_loco_tracker_stats() -> RailcomLocoTrackerStats {
    critical_section::with(|cs| TRACKER_STATE.borrow_ref(cs).snapshot())
}

#[cfg(test)]
pub fn reset_railcom_loco_tracker() {
    critical_section::with(|cs| {
        *TRACKER_STATE.borrow_ref_mut(cs) = TrackerState::new();
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn short_addr(address: u8) -> DccAddress {
        DccAddress::new_short(address).expect("valid short address")
    }

    fn long_addr(address: u16) -> DccAddress {
        DccAddress::new_long(address).expect("valid long address")
    }

    #[test]
    fn test_identify_short_address_from_adr_low_only() {
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(42))];

        assert_eq!(
            classify_address_candidate(&items),
            AddressCandidate::Identified(short_addr(42))
        );
    }

    #[test]
    fn test_identify_long_address_from_adr_high_and_low() {
        let items = [
            RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x03)),
            RailcomItem::Datagram(RailcomDatagram::AdrLow(0xe8)),
        ];

        assert_eq!(
            classify_address_candidate(&items),
            AddressCandidate::Identified(long_addr(1000))
        );
    }

    #[test]
    fn test_record_keeps_pending_adr_high_without_marking_invalid() {
        reset_railcom_loco_tracker();
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x01))];

        assert_eq!(record_loco_identification(7, &items), None);
        assert_eq!(
            railcom_loco_tracker_stats(),
            RailcomLocoTrackerStats {
                identification_window_count: 1,
                identified_loco_count: 0,
                invalid_identification_count: 0,
                recent_sightings: [None; RECENT_SIGHTING_CAPACITY],
            }
        );
    }

    #[test]
    fn test_record_identifies_long_address_from_pending_adr_high_and_next_low() {
        reset_railcom_loco_tracker();
        let high_items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x03))];
        let low_items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(0xe8))];

        assert_eq!(record_loco_identification(10, &high_items), None);
        let sighting = record_loco_identification(11, &low_items).expect("must identify");

        assert_eq!(
            sighting,
            RailcomLocoSighting {
                address: long_addr(1000),
                packet_sequence: 11,
                seen_count: 1,
            }
        );
        assert_eq!(
            railcom_loco_tracker_stats(),
            RailcomLocoTrackerStats {
                identification_window_count: 2,
                identified_loco_count: 1,
                invalid_identification_count: 0,
                recent_sightings: [
                    Some(RailcomLocoSighting {
                        address: long_addr(1000),
                        packet_sequence: 11,
                        seen_count: 1,
                    }),
                    None,
                    None,
                    None,
                ],
            }
        );
    }

    #[test]
    fn test_record_ignores_stale_pending_adr_high() {
        reset_railcom_loco_tracker();
        let high_items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x03))];
        let low_items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(42))];

        assert_eq!(record_loco_identification(10, &high_items), None);
        let sighting =
            record_loco_identification(10 + PENDING_ADR_HIGH_MAX_PACKET_GAP + 1, &low_items)
                .expect("stale high must fall back to short address");

        assert_eq!(
            sighting,
            RailcomLocoSighting {
                address: short_addr(42),
                packet_sequence: 19,
                seen_count: 1,
            }
        );
        assert_eq!(railcom_loco_tracker_stats().invalid_identification_count, 0);
    }

    #[test]
    fn test_record_updates_existing_sighting_seen_count() {
        reset_railcom_loco_tracker();
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(7))];

        let first = record_loco_identification(11, &items).expect("must identify");
        let second = record_loco_identification(19, &items).expect("must identify");

        assert_eq!(
            first,
            RailcomLocoSighting {
                address: short_addr(7),
                packet_sequence: 11,
                seen_count: 1,
            }
        );
        assert_eq!(
            second,
            RailcomLocoSighting {
                address: short_addr(7),
                packet_sequence: 19,
                seen_count: 2,
            }
        );
    }

    #[test]
    fn test_record_identified_address_from_logon_response() {
        reset_railcom_loco_tracker();

        let sighting = record_identified_address(20, short_addr(3));

        assert_eq!(
            sighting,
            RailcomLocoSighting {
                address: short_addr(3),
                packet_sequence: 20,
                seen_count: 1,
            }
        );
        assert_eq!(railcom_loco_tracker_stats().identified_loco_count, 1);
    }

    #[test]
    fn test_record_target_from_matching_short_address_low_fragment() {
        reset_railcom_loco_tracker();
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(3))];

        let sighting = record_target_from_matching_address_fragment(42, short_addr(3), &items)
            .expect("matching low fragment should confirm short target address");

        assert_eq!(
            sighting,
            RailcomLocoSighting {
                address: short_addr(3),
                packet_sequence: 42,
                seen_count: 1,
            }
        );
    }

    #[test]
    fn test_record_target_ignores_non_unique_high_fragment() {
        reset_railcom_loco_tracker();
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0))];

        assert_eq!(
            record_target_from_matching_address_fragment(42, short_addr(3), &items),
            None
        );
        assert_eq!(railcom_loco_tracker_stats().identified_loco_count, 0);
        assert_eq!(railcom_loco_tracker_stats().invalid_identification_count, 0);
    }

    #[test]
    fn test_record_target_rejects_mismatched_address_fragment() {
        reset_railcom_loco_tracker();
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(4))];

        assert_eq!(
            record_target_from_matching_address_fragment(42, short_addr(3), &items),
            None
        );
        assert_eq!(railcom_loco_tracker_stats().invalid_identification_count, 1);
    }

    #[test]
    fn test_ack_only_feedback_does_not_identify_loco() {
        reset_railcom_loco_tracker();
        let items = [RailcomItem::Ack];

        assert_eq!(record_loco_identification(50, &items), None);
        assert_eq!(railcom_loco_tracker_stats().identified_loco_count, 0);
    }
}
