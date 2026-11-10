//! Track search broadcasts sent in idle turns after each track enable, so
//! decoders answer in the cutouts while no locomotive is driven yet.

use dcc_protocol::packet::{BinaryStateAddress, DccPacket};
use embassy_time::{Duration, Instant};

use super::railcom_policy::record_track_search_sent;

/// RailCom track search trigger used when the command station has no active
/// slots yet but still wants decoders to answer during cutouts.
const RAILCOM_TRACK_SEARCH_BIN_ADDR: BinaryStateAddress = match BinaryStateAddress::new(2) {
    Some(address) => address,
    None => panic!("track-search binary-state address out of range"),
};
const RAILCOM_DISCOVERY_INTERVAL_MS: u64 = 1_000;
/// ZIMO's reference decoder only answers app:search shortly after signal
/// acquisition/rerail. Keep command-station discovery bounded to the same
/// 30-second startup window instead of sending search broadcasts forever.
const RAILCOM_TRACK_SEARCH_WINDOW_SECS: u64 = 30;

pub(super) struct RailcomDiscovery {
    active_until: Instant,
    next_due: Instant,
}

impl RailcomDiscovery {
    pub(super) fn new(now: Instant) -> Self {
        Self {
            active_until: now,
            next_due: now,
        }
    }

    pub(super) fn restart(&mut self, now: Instant) {
        self.active_until = now + Duration::from_secs(RAILCOM_TRACK_SEARCH_WINDOW_SECS);
        self.next_due = now;
    }

    pub(super) fn is_due(&self, now: Instant) -> bool {
        now < self.active_until && now >= self.next_due
    }

    pub(super) fn take_next_idle_packet(&mut self, now: Instant) -> DccPacket {
        self.next_due = now + Duration::from_millis(RAILCOM_DISCOVERY_INTERVAL_MS);
        record_track_search_sent();
        DccPacket::BroadcastBinaryStateShort {
            bin_addr: RAILCOM_TRACK_SEARCH_BIN_ADDR,
            state: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Idle packets of one whole discovery window after a track enable.
    fn discovery_window_packets() -> impl Iterator<Item = DccPacket> {
        let start = Instant::from_secs(0);
        let mut discovery = RailcomDiscovery::new(start);
        discovery.restart(start);
        (0..RAILCOM_TRACK_SEARCH_WINDOW_SECS).filter_map(move |second| {
            let now = start + Duration::from_secs(second);
            discovery
                .is_due(now)
                .then(|| discovery.take_next_idle_packet(now))
        })
    }

    #[test]
    fn every_idle_turn_of_the_window_is_a_track_search() {
        let packets: Vec<_> = discovery_window_packets().collect();

        assert_eq!(packets.len(), RAILCOM_TRACK_SEARCH_WINDOW_SECS as usize);
        assert!(
            packets
                .iter()
                .all(|packet| matches!(packet, DccPacket::BroadcastBinaryStateShort { .. }))
        );
    }
}
