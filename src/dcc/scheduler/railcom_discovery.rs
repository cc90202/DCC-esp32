use crate::dcc::packet::{BinaryStateAddress, DccPacket, LogonGroup, LogonSessionId};
use embassy_time::{Duration, Instant};

use super::railcom_policy::{record_track_logon_sent, record_track_search_sent};

/// DCC-A/RailCom track search trigger used when the command station has no
/// active slots yet but still wants decoders to answer during cutouts.
const RAILCOM_TRACK_SEARCH_BIN_ADDR: BinaryStateAddress = match BinaryStateAddress::new(2) {
    Some(address) => address,
    None => panic!("track-search binary-state address out of range"),
};
const RAILCOM_LOGON_GROUP_NOW: LogonGroup = match LogonGroup::new(3) {
    Some(group) => group,
    None => panic!("logon group out of range"),
};
const RAILCOM_LOGON_COMMAND_STATION_ID: u16 = 0x0DCC;
const RAILCOM_LOGON_SESSION_ID: LogonSessionId = LogonSessionId::new(1);
const RAILCOM_LOGON_ENABLE_STARTUP_REPEATS: u8 = 3;
const RAILCOM_DISCOVERY_INTERVAL_MS: u64 = 1_000;
/// ZIMO's reference decoder only answers app:search shortly after signal
/// acquisition/rerail. Keep command-station discovery bounded to the same
/// 30-second startup window instead of sending search broadcasts forever.
const RAILCOM_TRACK_SEARCH_WINDOW_SECS: u64 = 30;

pub(super) struct RailcomDiscovery {
    active_until: Instant,
    next_due: Instant,
    logon_enable_remaining: u8,
}

impl RailcomDiscovery {
    pub(super) fn new(now: Instant) -> Self {
        Self {
            active_until: now,
            next_due: now,
            logon_enable_remaining: 0,
        }
    }

    pub(super) fn restart(&mut self, now: Instant) {
        self.active_until = now + Duration::from_secs(RAILCOM_TRACK_SEARCH_WINDOW_SECS);
        self.next_due = now;
        self.logon_enable_remaining = RAILCOM_LOGON_ENABLE_STARTUP_REPEATS;
    }

    pub(super) fn suspend(&mut self, now: Instant) {
        self.active_until = now;
        self.next_due = now;
        self.logon_enable_remaining = 0;
    }

    pub(super) fn is_due(&self, now: Instant) -> bool {
        now < self.active_until && now >= self.next_due
    }

    pub(super) fn take_next_idle_packet(&mut self, now: Instant) -> DccPacket {
        self.next_due = now + Duration::from_millis(RAILCOM_DISCOVERY_INTERVAL_MS);

        if self.logon_enable_remaining > 0 {
            self.logon_enable_remaining -= 1;
            record_track_logon_sent();
            DccPacket::LogonEnable {
                group: RAILCOM_LOGON_GROUP_NOW,
                command_station_id: RAILCOM_LOGON_COMMAND_STATION_ID,
                session_id: RAILCOM_LOGON_SESSION_ID,
            }
        } else {
            record_track_search_sent();
            DccPacket::BroadcastBinaryStateShort {
                bin_addr: RAILCOM_TRACK_SEARCH_BIN_ADDR,
                state: false,
            }
        }
    }
}
