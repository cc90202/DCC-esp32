use core::cell::RefCell;

use critical_section::Mutex;

use crate::dcc::DccAddress;
use crate::railcom::PacketSequence;
use dcc_protocol::packet::HighFragment;
use dcc_protocol::railcom_data::{RailcomDatagram, RailcomItem};

// Packet boundaries after which a held high fragment is dropped even though
// no lost window was seen: about one second at the usual packet rate. Command
// packets get no cutout, so windows can lie far apart; a fragment this old
// predates a pause of the track, after which decoders restart their
// alternation.
const PENDING_ADR_HIGH_MAX_PACKET_GAP: u32 = 128;

// Fixed-size recent-activity cache used by diagnostics and Z21 RailCom lookup.
// It is deliberately not a locomotive roster: retaining four addresses keeps
// the stack-resident diagnostics snapshot bounded while serving recent polls.
const RECENT_SIGHTING_CAPACITY: usize = 4;

// How long a sighting is still served as current, in packet boundaries.
//
// The boundary counter advances about 154 times a second, so this is a little
// over three minutes. Under programming traffic a locomotive on the track is
// confirmed about every thirty seconds (bench, 2026-09-14: two confirmations in
// sixty seconds), which this window clears several times over. Without it the
// answer was "yes, I have data" for as long as the firmware ran, because only
// four addresses share the list and a second address practically never turns up
// to take a slot: the same evening, fourteen minutes with the track idle
// produced no address other than the one already known.
//
// The window errs long on purpose. Too short is the damaging direction — a
// locomotive standing on the track reported absent between two confirmations is
// something the operator sees, while a stale answer surviving a few minutes
// longer is not. And those thirty seconds were measured while the Z21 app was
// polling CVs, which is what produces most of the cutouts: with the track idle
// the interval is unmeasured and could be far longer than this window. That
// measurement is what would let this value be set rather than guessed.
const SIGHTING_MAX_AGE_PACKETS: u32 = 30_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomLocoSighting {
    pub address: DccAddress,
    pub packet_sequence: PacketSequence,
    pub seen_count: u32,
}

impl RailcomLocoSighting {
    /// Whether this sighting still describes the track as it is at `now`.
    #[must_use]
    pub(crate) fn is_current_at(self, now: PacketSequence) -> bool {
        now.is_within(self.packet_sequence, SIGHTING_MAX_AGE_PACKETS)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomLocoTrackerStats {
    pub identification_window_count: u32,
    pub identified_loco_count: u32,
    pub invalid_identification_count: u32,
    /// Windows stating a long address between 1 and 127: legal on the wire, and
    /// outside what [`DccAddress`] can hold. Counted apart from the corrupt
    /// ones, because it says a decoder is configured in a way this firmware
    /// does not support, not that the line is noisy.
    pub unrepresentable_address_count: u32,
    pub recent_sightings: [Option<RailcomLocoSighting>; RECENT_SIGHTING_CAPACITY],
}

impl RailcomLocoTrackerStats {
    /// The current sighting of `address`, if the track still says it is there.
    ///
    /// A sighting older than the freshness window stays in the list so that a
    /// locomotive which comes back keeps counting from where it left off. It is
    /// simply no longer answered for, and its slot is taken before that of any
    /// locomotive the track still confirms.
    #[must_use]
    pub fn current_sighting_for(
        &self,
        address: DccAddress,
        now: PacketSequence,
    ) -> Option<RailcomLocoSighting> {
        self.current_sightings(now)
            .find(|sighting| sighting.address == address)
    }

    /// Every sighting the track still confirms, in slot order.
    fn current_sightings(&self, now: PacketSequence) -> impl Iterator<Item = RailcomLocoSighting> {
        self.recent_sightings
            .iter()
            .flatten()
            .copied()
            .filter(move |sighting| sighting.is_current_at(now))
    }

    /// The most recently seen locomotive that is still current.
    #[must_use]
    pub fn latest_current_sighting(&self, now: PacketSequence) -> Option<RailcomLocoSighting> {
        self.current_sightings(now)
            .min_by_key(|sighting| now.age_since(sighting.packet_sequence))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct TrackerState {
    identification_window_count: u32,
    identified_loco_count: u32,
    invalid_identification_count: u32,
    unrepresentable_address_count: u32,
    recent_sightings: [Option<RailcomLocoSighting>; RECENT_SIGHTING_CAPACITY],
    /// A high fragment waiting for its low one.
    ///
    /// RCN-217 §5.2: after every packet sent to a locomotive decoder, each
    /// decoder sends one of its two fragments in channel 1, alternating, so a
    /// high fragment pairs only with the low one of the next channel-1 window
    /// after a locomotive packet, however many packets without a window come
    /// in between. A window that should have carried a fragment and did not,
    /// a window lost on the way, or [`PENDING_ADR_HIGH_MAX_PACKET_GAP`]
    /// packets drop it.
    pending_adr_high: Option<PendingAdrHigh>,
    /// The last window the identification pass judged, so that the target pass
    /// cannot judge it again. The two passes do not see the same state — the
    /// first consumes a high fragment held from an earlier window, the second
    /// never learns of it — so the second would decide on less than the first
    /// had, and on a window already accounted for.
    judged_window: Option<PacketSequence>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PendingAdrHigh {
    value: u8,
    packet_sequence: PacketSequence,
}

impl TrackerState {
    const fn new() -> Self {
        Self {
            identification_window_count: 0,
            identified_loco_count: 0,
            invalid_identification_count: 0,
            unrepresentable_address_count: 0,
            recent_sightings: [None; RECENT_SIGHTING_CAPACITY],
            pending_adr_high: None,
            judged_window: None,
        }
    }

    fn snapshot(&self) -> RailcomLocoTrackerStats {
        RailcomLocoTrackerStats {
            identification_window_count: self.identification_window_count,
            identified_loco_count: self.identified_loco_count,
            invalid_identification_count: self.invalid_identification_count,
            unrepresentable_address_count: self.unrepresentable_address_count,
            recent_sightings: self.recent_sightings,
        }
    }

    /// Counts one window that carried an address datagram.
    ///
    /// Called by the identification path alone, which runs on every window
    /// before the target path does: counting in both would score one window
    /// twice whenever identification declines and the target path then speaks.
    fn count_window(&mut self) {
        self.identification_window_count = self.identification_window_count.wrapping_add(1);
    }

    fn record_identified(
        &mut self,
        packet_sequence: PacketSequence,
        address: DccAddress,
    ) -> RailcomLocoSighting {
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
        self.recent_sightings[self.slot_for_new_sighting(packet_sequence)] = Some(sighting);
        sighting
    }

    /// Marks a window as decided, so the target pass leaves it alone.
    fn judge(&mut self, packet_sequence: PacketSequence) {
        self.judged_window = Some(packet_sequence);
    }

    /// Picks the slot a newly seen address should take: the one the track has
    /// gone longest without confirming, with an empty slot counting as older
    /// than any of them.
    ///
    /// Taking slots in turn instead would let a new address evict the one
    /// locomotive still being confirmed while sightings nobody confirms any
    /// more kept their places — losing exactly the count this list is kept for.
    /// Ties go to the lowest slot, so the choice never depends on the order the
    /// array happens to be walked in.
    fn slot_for_new_sighting(&self, now: PacketSequence) -> usize {
        let staleness = |slot: &Option<RailcomLocoSighting>| {
            slot.map_or(u32::MAX, |sighting| now.age_since(sighting.packet_sequence))
        };
        let oldest = self
            .recent_sightings
            .iter()
            .map(staleness)
            .max()
            .unwrap_or_default();

        self.recent_sightings
            .iter()
            .position(|slot| staleness(slot) == oldest)
            .unwrap_or_default()
    }

    fn record_unreadable(&mut self, fragments: UnreadableFragments) {
        match fragments {
            UnreadableFragments::UnrepresentableLong => {
                self.unrepresentable_address_count =
                    self.unrepresentable_address_count.wrapping_add(1);
            }
            UnreadableFragments::Invalid => {
                self.invalid_identification_count =
                    self.invalid_identification_count.wrapping_add(1);
            }
        }
    }

    fn record_decoded(
        &mut self,
        packet_sequence: PacketSequence,
        high: u8,
        low: u8,
    ) -> Option<RailcomLocoSighting> {
        match decode_railcom_address(Some(high), Some(low)) {
            Ok(address) => Some(self.record_identified(packet_sequence, address)),
            Err(fragments) => {
                self.record_unreadable(fragments);
                None
            }
        }
    }

    fn take_pending_high(&mut self, packet_sequence: PacketSequence) -> Option<u8> {
        let pending = self.pending_adr_high.take()?;
        packet_sequence
            .is_within(pending.packet_sequence, PENDING_ADR_HIGH_MAX_PACKET_GAP)
            .then_some(pending.value)
    }

    /// Judges a window against the address a programming request was sent to.
    ///
    /// In the runtime only a lone fragment reaches this: the identification
    /// pass runs first on every window and judges any that completes an
    /// address, so the cases below that need both are there for the
    /// predicate's sake, not because this path can meet them.
    fn record_target_from_matching_address_fragment(
        &mut self,
        packet_sequence: PacketSequence,
        target_address: DccAddress,
        items: &[RailcomItem],
    ) -> Option<RailcomLocoSighting> {
        let (adr_high, adr_low, saw_address_datagram) = address_parts(items);
        if !saw_address_datagram || self.judged_window == Some(packet_sequence) {
            return None;
        }

        match address_fragment_matches_target(target_address, adr_high, adr_low) {
            Some(true) => {
                self.judge(packet_sequence);
                Some(self.record_identified(packet_sequence, target_address))
            }
            Some(false) => {
                self.judge(packet_sequence);
                self.record_unreadable(classify_unreadable(adr_high, adr_low));
                None
            }
            None => None,
        }
    }

    fn record_loco_identification(
        &mut self,
        packet_sequence: PacketSequence,
        window: FragmentWindow,
        items: &[RailcomItem],
    ) -> Option<RailcomLocoSighting> {
        let (adr_high, adr_low, saw_address_datagram) = address_parts(items);
        match window {
            FragmentWindow::LocoChannel1 => {}
            // No telling what this window followed, so whether a fragment
            // was owed: the run of consecutive windows is broken.
            FragmentWindow::UnknownChannel1 => {
                self.pending_adr_high = None;
                return None;
            }
            // No decoder sends its address after an idle, broadcast or logon
            // packet (RCN-217 §5.2); whatever parses as a fragment here is not
            // one, and the held fragment keeps its place.
            FragmentWindow::OtherChannel1 => return None,
            // A track-search answer carries both fragments in one window; a
            // lone one is not part of the channel-1 alternation.
            FragmentWindow::Channel2 => {
                let (Some(high), Some(low)) = (adr_high, adr_low) else {
                    return None;
                };
                self.count_window();
                self.judge(packet_sequence);
                return self.record_decoded(packet_sequence, high, low);
            }
        }
        if !saw_address_datagram {
            // Every decoder had to send a fragment here (RCN-217 §4): this
            // window was lost, so a held high fragment no longer has its low
            // one next. Pairing it later could join parts of two decoders'
            // addresses, which the standard forbids.
            self.pending_adr_high = None;
            return None;
        }
        self.count_window();

        if let Some(low) = adr_low {
            // RCN-217 §5.2 has every decoder alternate ADR1 and ADR2, short
            // addresses included, so a low fragment alone means its high
            // fragment was lost. Reading it as a short address would invent a
            // locomotive; the window is left to the target pass instead.
            let high = adr_high.or_else(|| self.take_pending_high(packet_sequence))?;
            self.judge(packet_sequence);
            return self.record_decoded(packet_sequence, high, low);
        }

        // Only a lone high fragment can reach here: the low fragment returns
        // above, and nothing else sets `saw_address_datagram`.
        if let Some(high) = adr_high {
            self.pending_adr_high = Some(PendingAdrHigh {
                value: high,
                packet_sequence,
            });
        }
        None
    }
}

static TRACKER_STATE: Mutex<RefCell<TrackerState>> = Mutex::new(RefCell::new(TrackerState::new()));

/// Why a pair of fragments named no address we could use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum UnreadableFragments {
    /// A long address below 128. The wire allows it; [`DccAddress`] cannot hold
    /// it, and reading it as the short address of the same number would name a
    /// different decoder.
    UnrepresentableLong,
    /// Anything else: a high fragment of no known form, or a value out of
    /// range.
    Invalid,
}

fn decode_railcom_address(
    adr_high: Option<u8>,
    adr_low: Option<u8>,
) -> Result<DccAddress, UnreadableFragments> {
    let (Some(high), Some(low)) = (adr_high, adr_low) else {
        return Err(UnreadableFragments::Invalid);
    };
    DccAddress::from_address_fragments(high, low)
        .ok_or_else(|| classify_unreadable(adr_high, adr_low))
}

/// Tells the one address the wire can state and this crate cannot hold apart
/// from everything else that fails to read.
fn classify_unreadable(adr_high: Option<u8>, adr_low: Option<u8>) -> UnreadableFragments {
    let (Some(high), Some(low)) = (adr_high, adr_low) else {
        return UnreadableFragments::Invalid;
    };
    let fragment = HighFragment::read(high);
    match (fragment, fragment.address_value(low)) {
        (HighFragment::Long(_), Some(1..=127)) => UnreadableFragments::UnrepresentableLong,
        _ => UnreadableFragments::Invalid,
    }
}

/// Whether a high fragment can belong to `target`.
///
/// The form of the fragment decides, on the same rule as
/// [`DccAddress::from_address_fragments`]: an extended-address fragment
/// answers only for a long target with the same top bits, a base or consist
/// marker only for a short target, and a malformed byte for nothing.
fn high_fragment_matches(high: u8, target: DccAddress) -> bool {
    match HighFragment::read(high) {
        HighFragment::Long(value) => {
            target.is_long() && value == HighFragment::value_bits_of(target)
        }
        HighFragment::Short | HighFragment::Consist => target.is_short(),
        HighFragment::Malformed => false,
    }
}

fn address_fragment_matches_target(
    target_address: DccAddress,
    adr_high: Option<u8>,
    adr_low: Option<u8>,
) -> Option<bool> {
    match (adr_high, adr_low) {
        (Some(high), Some(low)) => {
            Some(DccAddress::from_address_fragments(high, low) == Some(target_address))
        }
        // A lone low fragment names no address (RCN-217 §5.2), short or long,
        // and rules out none either: with several decoders on the track it
        // may be another one's.
        (None, Some(_)) => None,
        (Some(high), None) => {
            if high_fragment_matches(high, target_address) {
                None
            } else {
                Some(false)
            }
        }
        (None, None) => None,
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
            | RailcomDatagram::Info1(_)
            | RailcomDatagram::Dyn { .. }
            | RailcomDatagram::Xpom { .. }
            | RailcomDatagram::CvData(_)
            | RailcomDatagram::Search(_) => {}
        }
    }

    (adr_high, adr_low, saw_address_datagram)
}

pub fn record_target_from_matching_address_fragment(
    packet_sequence: impl Into<PacketSequence>,
    target_address: DccAddress,
    items: &[RailcomItem],
) -> Option<RailcomLocoSighting> {
    let packet_sequence = packet_sequence.into();
    critical_section::with(|cs| {
        TRACKER_STATE
            .borrow_ref_mut(cs)
            .record_target_from_matching_address_fragment(packet_sequence, target_address, items)
    })
}

/// Which window the fragments came from, which decides what they may pair with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FragmentWindow {
    /// Channel 1 after a packet sent to a locomotive decoder: every decoder
    /// must send one fragment here (RCN-217 §4, §5.2).
    LocoChannel1,
    /// Channel 1 whose packet is no longer known.
    UnknownChannel1,
    /// Channel 1 after any other packet: idle, broadcast or logon.
    OtherChannel1,
    /// Channel 2.
    Channel2,
}

/// Reads the address fragments of one window.
pub fn record_loco_identification(
    packet_sequence: impl Into<PacketSequence>,
    window: FragmentWindow,
    items: &[RailcomItem],
) -> Option<RailcomLocoSighting> {
    let packet_sequence = packet_sequence.into();
    critical_section::with(|cs| {
        TRACKER_STATE
            .borrow_ref_mut(cs)
            .record_loco_identification(packet_sequence, window, items)
    })
}

/// Drops a held high fragment: a window was lost between capture and
/// dispatch, so the next low fragment may not follow it.
pub fn forget_pending_fragment() {
    critical_section::with(|cs| TRACKER_STATE.borrow_ref_mut(cs).pending_adr_high = None);
}

pub fn record_identified_address(
    packet_sequence: impl Into<PacketSequence>,
    address: DccAddress,
) -> RailcomLocoSighting {
    let packet_sequence = packet_sequence.into();
    critical_section::with(|cs| {
        let mut state = TRACKER_STATE.borrow_ref_mut(cs);
        // The logon answer is normally a window of its own, which no
        // identification pass has counted. Nothing enforces that: a channel-2
        // logon window whose bytes also parse as an address fragment would be
        // counted twice. Rare enough to accept, not rare enough to claim it
        // cannot happen.
        state.count_window();
        state.record_identified(packet_sequence, address)
    })
}

#[must_use]
pub fn railcom_loco_tracker_stats() -> RailcomLocoTrackerStats {
    critical_section::with(|cs| TRACKER_STATE.borrow_ref(cs).snapshot())
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
    fn test_record_identifies_a_flagged_long_address_across_two_windows() {
        let mut state = TrackerState::new();
        let high_items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x83))];
        let low_items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(0xe8))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &high_items
            ),
            None
        );
        let sighting = state
            .record_loco_identification(
                PacketSequence::new(11),
                FragmentWindow::LocoChannel1,
                &low_items,
            )
            .expect("must identify");

        assert_eq!(sighting.address, long_addr(1000));
    }

    #[test]
    fn test_a_flagged_fragment_matches_the_long_target_it_names() {
        // RCN-217 table 11 sends 1000 as ADR1 0x83, ADR2 0xe8. Asserted on the
        // predicate, which is where the rule lives — the window itself never
        // reaches the target pass, since one carrying both fragments is judged
        // by the identification pass first.
        assert_eq!(
            address_fragment_matches_target(long_addr(1000), Some(0x83), Some(0xe8)),
            Some(true)
        );
        assert_eq!(
            address_fragment_matches_target(short_addr(3), Some(0x80), Some(3)),
            Some(false)
        );
        assert_eq!(
            address_fragment_matches_target(short_addr(3), Some(0x00), Some(3)),
            Some(true)
        );
    }

    #[test]
    fn test_record_target_rejects_every_flagged_high_fragment_for_a_short_target() {
        // The low byte agreeing is not enough. `0x80` says long where the
        // target is short; `0x40` and `0xc0` (the DCC packet prefix) are no
        // ADR1 form at all. Driven in the order the runtime uses. The
        // identification pass judges the window and produces the outcome; the
        // target pass that follows finds it already judged and adds nothing.
        // One window, one outcome — and the reason is carried through, so
        // `0x80` states a long address the firmware cannot hold while the
        // other two are of no known form.
        let cases = [(0x40, 1, 0), (0xc0, 1, 0), (0x80, 0, 1)];
        for (high, expected_invalid, expected_unrepresentable) in cases {
            let mut state = TrackerState::new();
            let items = [
                RailcomItem::Datagram(RailcomDatagram::AdrHigh(high)),
                RailcomItem::Datagram(RailcomDatagram::AdrLow(3)),
            ];

            assert_eq!(
                state.record_loco_identification(
                    PacketSequence::new(42),
                    FragmentWindow::LocoChannel1,
                    &items
                ),
                None,
                "high fragment {high:#04x}"
            );
            assert_eq!(
                state.record_target_from_matching_address_fragment(
                    PacketSequence::new(42),
                    short_addr(3),
                    &items,
                ),
                None,
                "high fragment {high:#04x}"
            );

            let stats = state.snapshot();
            assert_eq!(
                stats.identification_window_count, 1,
                "high fragment {high:#04x}"
            );
            assert_eq!(
                stats.invalid_identification_count, expected_invalid,
                "high fragment {high:#04x}"
            );
            assert_eq!(
                stats.unrepresentable_address_count, expected_unrepresentable,
                "high fragment {high:#04x}"
            );
        }
    }

    #[test]
    fn test_decode_reads_each_form_of_high_fragment() {
        // The whole rule in one place, after RCN-217 table 11: base, consist
        // and extended forms, malformed bytes, and the long address the wire
        // allows and `DccAddress` cannot hold.
        let cases = [
            (0x00, 0x03, Ok(short_addr(3))),
            (0x00, 0x2a, Ok(short_addr(42))),
            (0x60, 0x05, Ok(short_addr(5))),
            (0x83, 0xe8, Ok(long_addr(1000))),
            (0x84, 0x03, Ok(long_addr(1027))),
            (0x81, 0x2c, Ok(long_addr(300))),
            (0x80, 0x03, Err(UnreadableFragments::UnrepresentableLong)),
            // Zero is no address in any reading, so it is not the configuration
            // the unrepresentable counter is meant to name.
            (0x80, 0x00, Err(UnreadableFragments::Invalid)),
            (0xc3, 0xe8, Err(UnreadableFragments::Invalid)),
            (0x03, 0xe8, Err(UnreadableFragments::Invalid)),
            (0x40, 0x03, Err(UnreadableFragments::Invalid)),
            (0x00, 0x00, Err(UnreadableFragments::Invalid)),
            (0xff, 0xff, Err(UnreadableFragments::Invalid)),
        ];

        for (high, low, expected) in cases {
            assert_eq!(
                decode_railcom_address(Some(high), Some(low)),
                expected,
                "fragments {high:#04x} {low:#04x}"
            );
        }
    }

    #[test]
    fn test_record_counts_an_unrepresentable_address_apart_from_a_corrupt_one() {
        // Across two windows, which is how the fragments really arrive. The
        // decoder is configured in a way this firmware cannot represent; that
        // must not read as line noise in the diagnostics.
        let mut state = TrackerState::new();
        let high_items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x80))];
        let low_items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(3))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(30),
                FragmentWindow::LocoChannel1,
                &high_items
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(31),
                FragmentWindow::LocoChannel1,
                &low_items
            ),
            None
        );

        let stats = state.snapshot();
        assert_eq!(stats.identification_window_count, 2);
        assert_eq!(stats.identified_loco_count, 0);
        assert_eq!(stats.unrepresentable_address_count, 1);
        assert_eq!(stats.invalid_identification_count, 0);
        assert_eq!(stats.recent_sightings, [None; RECENT_SIGHTING_CAPACITY]);
    }

    #[test]
    fn test_record_target_waits_on_a_flagged_high_fragment_alone() {
        // A lone high fragment that agrees with a long target decides nothing:
        // it is neither a match nor a mismatch until the low fragment arrives.
        let mut state = TrackerState::new();
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x83))];

        assert_eq!(
            state.record_target_from_matching_address_fragment(
                PacketSequence::new(42),
                long_addr(1000),
                &items,
            ),
            None
        );
        assert_eq!(state.snapshot().invalid_identification_count, 0);
    }

    #[test]
    fn test_record_keeps_pending_adr_high_without_marking_invalid() {
        let mut state = TrackerState::new();
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x01))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(7),
                FragmentWindow::LocoChannel1,
                &items
            ),
            None
        );
        assert_eq!(
            state.snapshot(),
            RailcomLocoTrackerStats {
                identification_window_count: 1,
                identified_loco_count: 0,
                invalid_identification_count: 0,
                unrepresentable_address_count: 0,
                recent_sightings: [None; RECENT_SIGHTING_CAPACITY],
            }
        );
    }

    #[test]
    fn test_record_identifies_long_address_from_pending_adr_high_and_next_low() {
        let mut state = TrackerState::new();
        let high_items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x83))];
        let low_items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(0xe8))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &high_items
            ),
            None
        );
        let sighting = state
            .record_loco_identification(
                PacketSequence::new(11),
                FragmentWindow::LocoChannel1,
                &low_items,
            )
            .expect("must identify");

        assert_eq!(
            sighting,
            RailcomLocoSighting {
                address: long_addr(1000),
                packet_sequence: PacketSequence::new(11),
                seen_count: 1,
            }
        );
        assert_eq!(
            state.snapshot(),
            RailcomLocoTrackerStats {
                identification_window_count: 2,
                identified_loco_count: 1,
                invalid_identification_count: 0,
                unrepresentable_address_count: 0,
                recent_sightings: listed(&[RailcomLocoSighting {
                    address: long_addr(1000),
                    packet_sequence: PacketSequence::new(11),
                    seen_count: 1,
                }]),
            }
        );
    }

    #[test]
    fn test_a_high_fragment_waits_through_packets_without_a_window() {
        // Command packets get no cutout, so the next window can come many
        // packets later; the pair still forms there.
        let mut state = TrackerState::new();
        let high_items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x84))];
        let low_items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(0x03))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &high_items
            ),
            None
        );
        let sighting = state
            .record_loco_identification(
                PacketSequence::new(110),
                FragmentWindow::LocoChannel1,
                &low_items,
            )
            .expect("1027 is ADR1 0x84, ADR2 0x03");
        assert_eq!(sighting.address, long_addr(1027));
    }

    #[test]
    fn test_a_high_fragment_expires_after_a_long_silence() {
        let mut state = TrackerState::new();
        let high_items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x84))];
        let low_items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(0x03))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &high_items
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10 + PENDING_ADR_HIGH_MAX_PACKET_GAP + 1),
                FragmentWindow::LocoChannel1,
                &low_items,
            ),
            None
        );
        assert_eq!(state.snapshot().identified_loco_count, 0);
    }

    #[test]
    fn test_a_lost_window_drops_the_held_fragment() {
        critical_section::with(|cs| *TRACKER_STATE.borrow_ref_mut(cs) = TrackerState::new());
        let high_items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x83))];
        let low_items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(5))];

        assert_eq!(
            record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &high_items
            ),
            None
        );
        forget_pending_fragment();
        assert_eq!(
            record_loco_identification(
                PacketSequence::new(17),
                FragmentWindow::LocoChannel1,
                &low_items
            ),
            None
        );
    }

    #[test]
    fn test_a_lone_low_fragment_names_no_locomotive() {
        // RCN-217 §5.2: decoders always alternate ADR1 and ADR2, so a lone
        // ADR2 is an ADR1 that was lost, never a short address by omission.
        let mut state = TrackerState::new();
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(0x2c))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &items
            ),
            None
        );
        assert_eq!(state.snapshot().identified_loco_count, 0);
        assert_eq!(state.snapshot().invalid_identification_count, 0);
    }

    #[test]
    fn test_a_short_address_is_read_from_its_two_fragments_in_consecutive_windows() {
        // Loco 4, as the bench decoder sends it: ADR1 0x00, then ADR2 4 one
        // window (seven packets) later.
        let mut state = TrackerState::new();
        let high_items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x00))];
        let low_items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(4))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &high_items
            ),
            None
        );
        let sighting = state
            .record_loco_identification(
                PacketSequence::new(17),
                FragmentWindow::LocoChannel1,
                &low_items,
            )
            .expect("the pair names loco 4");
        assert_eq!(sighting.address, short_addr(4));
    }

    #[test]
    fn test_record_updates_existing_sighting_seen_count() {
        let mut state = TrackerState::new();
        let items = [
            RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x00)),
            RailcomItem::Datagram(RailcomDatagram::AdrLow(7)),
        ];

        let first = state
            .record_loco_identification(
                PacketSequence::new(11),
                FragmentWindow::LocoChannel1,
                &items,
            )
            .expect("must identify");
        let second = state
            .record_loco_identification(
                PacketSequence::new(19),
                FragmentWindow::LocoChannel1,
                &items,
            )
            .expect("must identify");

        assert_eq!(
            first,
            RailcomLocoSighting {
                address: short_addr(7),
                packet_sequence: PacketSequence::new(11),
                seen_count: 1,
            }
        );
        assert_eq!(
            second,
            RailcomLocoSighting {
                address: short_addr(7),
                packet_sequence: PacketSequence::new(19),
                seen_count: 2,
            }
        );
    }

    #[test]
    fn test_record_identified_address_from_logon_response() {
        let mut state = TrackerState::new();

        let sighting = state.record_identified(PacketSequence::new(20), short_addr(3));

        assert_eq!(
            sighting,
            RailcomLocoSighting {
                address: short_addr(3),
                packet_sequence: PacketSequence::new(20),
                seen_count: 1,
            }
        );
        assert_eq!(state.snapshot().identified_loco_count, 1);
    }

    #[test]
    fn test_record_target_waits_on_a_lone_matching_low_fragment() {
        // RCN-217 §5.2: a lone ADR2 names no address. A long decoder such as
        // 1028 sends ADR2 4 too, so it cannot confirm short target 4.
        let mut state = TrackerState::new();
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(4))];

        assert_eq!(
            state.record_target_from_matching_address_fragment(
                PacketSequence::new(42),
                short_addr(4),
                &items,
            ),
            None
        );
        assert_eq!(state.snapshot().identified_loco_count, 0);
    }

    #[test]
    fn test_a_lost_window_after_a_loco_packet_breaks_the_pair() {
        // ADR1 of one decoder, then a window every decoder had to answer and
        // nothing came, then an ADR2: it may belong to another decoder, and
        // RCN-217 §4 forbids joining parts of different addresses.
        let mut state = TrackerState::new();
        let high = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x83))];
        let low = [RailcomItem::Datagram(RailcomDatagram::AdrLow(5))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &high
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(17),
                FragmentWindow::LocoChannel1,
                &[]
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(24),
                FragmentWindow::LocoChannel1,
                &low
            ),
            None
        );
        assert_eq!(state.snapshot().identified_loco_count, 0);
    }

    #[test]
    fn test_the_channel_2_window_of_the_same_cutout_keeps_the_pair() {
        // Each cutout is reported as a channel-1 window, then a channel-2
        // window with the same packet sequence; channel 2 carries the
        // addressed decoder's reply, never an address fragment it owed.
        let mut state = TrackerState::new();
        let high = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x00))];
        let low = [RailcomItem::Datagram(RailcomDatagram::AdrLow(4))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &high
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::Channel2,
                &[RailcomItem::Ack]
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(17),
                FragmentWindow::LocoChannel1,
                &[]
            ),
            None
        );
        assert_eq!(
            state.snapshot().identified_loco_count,
            0,
            "a lost channel-1 window breaks it"
        );

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(24),
                FragmentWindow::LocoChannel1,
                &high
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(24),
                FragmentWindow::Channel2,
                &[]
            ),
            None
        );
        let sighting = state
            .record_loco_identification(PacketSequence::new(31), FragmentWindow::LocoChannel1, &low)
            .expect("the pair names loco 4");
        assert_eq!(sighting.address, short_addr(4));
    }

    #[test]
    fn test_channel_2_and_non_loco_windows_never_join_the_alternation() {
        // A logon answer or a stray channel-2 byte that parses as a fragment
        // must neither complete a held high fragment nor become one.
        let mut state = TrackerState::new();
        let high = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x83))];
        let low = [RailcomItem::Datagram(RailcomDatagram::AdrLow(0x2c))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &high
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(12),
                FragmentWindow::Channel2,
                &low
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(14),
                FragmentWindow::OtherChannel1,
                &low
            ),
            None
        );
        assert_eq!(state.snapshot().identified_loco_count, 0);

        let mut state = TrackerState::new();
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::Channel2,
                &high
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(12),
                FragmentWindow::OtherChannel1,
                &high
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(17),
                FragmentWindow::LocoChannel1,
                &low
            ),
            None
        );
        assert_eq!(state.snapshot().identified_loco_count, 0);
    }

    #[test]
    fn test_a_track_search_answer_names_its_address_in_one_window() {
        let mut state = TrackerState::new();
        let both = [
            RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x00)),
            RailcomItem::Datagram(RailcomDatagram::AdrLow(4)),
        ];
        let sighting = state
            .record_loco_identification(PacketSequence::new(10), FragmentWindow::Channel2, &both)
            .expect("ADR1 and ADR2 together name loco 4");
        assert_eq!(sighting.address, short_addr(4));
    }

    #[test]
    fn test_a_window_of_unknown_origin_breaks_the_pair() {
        let mut state = TrackerState::new();
        let high = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x00))];
        let low = [RailcomItem::Datagram(RailcomDatagram::AdrLow(4))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &high
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(17),
                FragmentWindow::UnknownChannel1,
                &low
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(24),
                FragmentWindow::LocoChannel1,
                &low
            ),
            None
        );
        assert_eq!(state.snapshot().identified_loco_count, 0);
    }

    #[test]
    fn test_a_window_after_an_idle_packet_keeps_the_pair() {
        // No decoder sends after an idle packet, so its empty window is no
        // lost answer.
        let mut state = TrackerState::new();
        let high = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x00))];
        let low = [RailcomItem::Datagram(RailcomDatagram::AdrLow(4))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &high
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(12),
                FragmentWindow::OtherChannel1,
                &[]
            ),
            None
        );
        let sighting = state
            .record_loco_identification(PacketSequence::new(17), FragmentWindow::LocoChannel1, &low)
            .expect("the pair names loco 4");
        assert_eq!(sighting.address, short_addr(4));
    }

    #[test]
    fn test_record_target_ignores_non_unique_high_fragment() {
        let mut state = TrackerState::new();
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0))];

        assert_eq!(
            state.record_target_from_matching_address_fragment(
                PacketSequence::new(42),
                short_addr(3),
                &items,
            ),
            None
        );
        assert_eq!(state.snapshot().identified_loco_count, 0);
        assert_eq!(state.snapshot().invalid_identification_count, 0);
    }

    #[test]
    fn test_a_window_judged_by_identification_is_not_judged_again_by_the_target() {
        // The fragments arrive in separate windows, which is the normal case.
        // The identification pass consumes the high fragment it had been
        // holding and finds an address this firmware cannot represent; the
        // second window then carries only the low fragment, and the target pass
        // would see a plain short address matching its target. Judging it again
        // would put back exactly the aliasing this whole change removes — the
        // wrong locomotive recorded, under a third outcome on two windows.
        let mut state = TrackerState::new();
        let high_items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x80))];
        let low_items = [RailcomItem::Datagram(RailcomDatagram::AdrLow(3))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(30),
                FragmentWindow::LocoChannel1,
                &high_items
            ),
            None
        );
        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(31),
                FragmentWindow::LocoChannel1,
                &low_items
            ),
            None
        );
        assert_eq!(
            state.record_target_from_matching_address_fragment(
                PacketSequence::new(31),
                short_addr(3),
                &low_items,
            ),
            None
        );

        let stats = state.snapshot();
        assert_eq!(stats.identification_window_count, 2);
        assert_eq!(stats.identified_loco_count, 0);
        assert_eq!(stats.invalid_identification_count, 0);
        assert_eq!(stats.unrepresentable_address_count, 1);
        assert_eq!(stats.recent_sightings, [None; RECENT_SIGHTING_CAPACITY]);
    }

    #[test]
    fn test_record_target_rejects_a_mismatched_high_fragment() {
        // The lone high fragment is the only case this path still judges, and
        // it is driven the way the runtime does: identification first, which
        // only sets the fragment aside, then the target pass.
        let mut state = TrackerState::new();
        let items = [RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x85))];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(42),
                FragmentWindow::LocoChannel1,
                &items
            ),
            None
        );
        assert_eq!(
            state.record_target_from_matching_address_fragment(
                PacketSequence::new(42),
                long_addr(1000),
                &items,
            ),
            None
        );

        let stats = state.snapshot();
        assert_eq!(stats.identification_window_count, 1);
        assert_eq!(stats.invalid_identification_count, 1);
    }

    fn sighting_at(address: DccAddress, sequence: u32) -> RailcomLocoSighting {
        RailcomLocoSighting {
            address,
            packet_sequence: PacketSequence::new(sequence),
            seen_count: 1,
        }
    }

    /// A sighting list holding `sightings` in its first slots.
    fn listed(
        sightings: &[RailcomLocoSighting],
    ) -> [Option<RailcomLocoSighting>; RECENT_SIGHTING_CAPACITY] {
        let mut list = [None; RECENT_SIGHTING_CAPACITY];
        for (slot, sighting) in list.iter_mut().zip(sightings) {
            *slot = Some(*sighting);
        }
        list
    }

    fn stats_with(
        sightings: [Option<RailcomLocoSighting>; RECENT_SIGHTING_CAPACITY],
    ) -> RailcomLocoTrackerStats {
        RailcomLocoTrackerStats {
            recent_sightings: sightings,
            ..RailcomLocoTrackerStats::default()
        }
    }

    #[test]
    fn test_a_sighting_stays_current_to_the_edge_of_its_window() {
        let sighting = sighting_at(short_addr(3), 1_000);

        assert!(sighting.is_current_at(PacketSequence::new(1_000)));
        assert!(sighting.is_current_at(PacketSequence::new(1_000 + SIGHTING_MAX_AGE_PACKETS)));
        assert!(!sighting.is_current_at(PacketSequence::new(1_000 + SIGHTING_MAX_AGE_PACKETS + 1)));
        // A date in the future counts as stale: silence beats a wrong answer.
        assert!(!sighting.is_current_at(PacketSequence::new(999)));
    }

    #[test]
    fn test_freshness_survives_the_counter_wrapping() {
        // The boundary counter is 32 bits and advances about 154 times a
        // second, so it wraps after about eleven months of running. A sighting taken
        // just before the wrap must still read as current just after it.
        let sighting = sighting_at(short_addr(3), u32::MAX - 10);

        assert!(sighting.is_current_at(PacketSequence::new(20)));
        assert!(sighting.is_current_at(PacketSequence::new(SIGHTING_MAX_AGE_PACKETS - 11)));
        assert!(!sighting.is_current_at(PacketSequence::new(SIGHTING_MAX_AGE_PACKETS)));
    }

    #[test]
    fn test_a_stale_sighting_is_no_longer_answered_for() {
        // The locomotive was lifted off the track. Without the window the
        // lookup kept saying it was there for as long as the firmware ran,
        // because only four addresses share the list and a second address
        // practically never appears.
        let stats = stats_with(listed(&[sighting_at(short_addr(3), 1_000)]));
        let within = PacketSequence::new(1_000 + SIGHTING_MAX_AGE_PACKETS);
        let past = PacketSequence::new(1_000 + SIGHTING_MAX_AGE_PACKETS + 1);

        assert_eq!(
            stats.current_sighting_for(short_addr(3), within),
            Some(sighting_at(short_addr(3), 1_000))
        );
        assert_eq!(stats.current_sighting_for(short_addr(3), past), None);
        assert_eq!(stats.latest_current_sighting(past), None);
    }

    #[test]
    fn test_the_latest_sighting_skips_the_stale_ones() {
        let stale = sighting_at(short_addr(7), 1_000);
        let fresh = sighting_at(short_addr(3), 1_000 + SIGHTING_MAX_AGE_PACKETS);
        let stats = stats_with(listed(&[stale, fresh]));
        let now = PacketSequence::new(1_000 + SIGHTING_MAX_AGE_PACKETS + 1);

        assert_eq!(stats.latest_current_sighting(now), Some(fresh));
        assert_eq!(stats.current_sighting_for(short_addr(7), now), None);
    }

    #[test]
    fn test_the_latest_sighting_is_the_most_recent_of_the_current_ones() {
        // With the freshness filter doing nothing, the ordering is what decides.
        let older = sighting_at(short_addr(7), 1_000);
        let newer = sighting_at(short_addr(3), 1_500);
        let stats = stats_with(listed(&[older, newer]));

        assert_eq!(
            stats.latest_current_sighting(PacketSequence::new(2_000)),
            Some(newer)
        );
    }

    #[test]
    fn test_a_stale_sighting_stops_being_answered_for_but_keeps_its_count() {
        // The two halves of the decision, in one test: silent while stale, and
        // counting from where it left off when the locomotive comes back.
        let mut state = TrackerState::new();
        let items = [
            RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x00)),
            RailcomItem::Datagram(RailcomDatagram::AdrLow(3)),
        ];

        state
            .record_loco_identification(
                PacketSequence::new(10),
                FragmentWindow::LocoChannel1,
                &items,
            )
            .expect("must identify");

        let gone = PacketSequence::new(10 + SIGHTING_MAX_AGE_PACKETS + 1);
        let stats = state.snapshot();
        assert_eq!(stats.current_sighting_for(short_addr(3), gone), None);
        assert!(stats.recent_sightings[0].is_some());

        let back = state
            .record_loco_identification(gone, FragmentWindow::LocoChannel1, &items)
            .expect("must identify again");

        assert_eq!(back.seen_count, 2);
        assert_eq!(
            state.snapshot().current_sighting_for(short_addr(3), gone),
            Some(back)
        );
    }

    /// One distinct short address per slot, with address 3 in the first.
    fn full_list_addresses() -> [u8; RECENT_SIGHTING_CAPACITY] {
        core::array::from_fn(|slot| 3 + 4 * slot as u8)
    }

    #[test]
    fn test_a_new_address_evicts_the_one_seen_longest_ago() {
        // A full list, every locomotive still confirmed, and one more address
        // turns up. The list has to give something up: it gives up the one the
        // track has gone longest without confirming.
        let mut state = TrackerState::new();
        let addresses = full_list_addresses();
        for (offset, address) in addresses.into_iter().enumerate() {
            state.record_identified(PacketSequence::new(10 + offset as u32), short_addr(address));
        }

        let now = PacketSequence::new(100);
        state.record_identified(now, short_addr(100));

        let stats = state.snapshot();
        assert_eq!(
            stats.current_sighting_for(short_addr(addresses[0]), now),
            None,
            "the oldest must be the one to go"
        );
        for address in addresses[1..].iter().copied().chain([100]) {
            assert!(
                stats
                    .current_sighting_for(short_addr(address), now)
                    .is_some(),
                "address {address} must still be listed"
            );
        }
    }

    #[test]
    fn test_a_new_address_evicts_a_stale_slot_before_a_current_one() {
        // The list is full, only one locomotive is still being seen, and a new
        // address turns up. Rotating blindly would evict the live one and keep
        // the dead entries.
        let mut state = TrackerState::new();
        for address in full_list_addresses() {
            state.record_identified(PacketSequence::new(10), short_addr(address));
        }
        // The list is full and the rotation points back at the first slot,
        // which is where address 3 sits. Keep confirming exactly that one.
        let refreshed = PacketSequence::new(10 + SIGHTING_MAX_AGE_PACKETS);
        state.record_identified(refreshed, short_addr(3));

        let later = PacketSequence::new(10 + SIGHTING_MAX_AGE_PACKETS + 1);
        state.record_identified(later, short_addr(42));

        let stats = state.snapshot();
        assert!(
            stats.current_sighting_for(short_addr(3), later).is_some(),
            "the locomotive still being confirmed must survive the new address"
        );
        assert!(
            stats.current_sighting_for(short_addr(42), later).is_some(),
            "the new address must have taken a slot"
        );
    }

    #[test]
    fn test_ack_only_feedback_does_not_identify_loco() {
        let mut state = TrackerState::new();
        let items = [RailcomItem::Ack];

        assert_eq!(
            state.record_loco_identification(
                PacketSequence::new(50),
                FragmentWindow::LocoChannel1,
                &items
            ),
            None
        );
        assert_eq!(state.snapshot().identified_loco_count, 0);
    }
}
