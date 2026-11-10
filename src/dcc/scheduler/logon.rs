//! Decoder table and logon sequence of RCN-218 (DCC-A).
//!
//! While the track has power a logon enable goes out a few times a second,
//! inviting the decoders that are not registered yet to announce themselves.
//! The invitation asks for an answer at once; when two decoders answer
//! together it switches to the form that lets each one wait a random number
//! of invitations, until the track is quiet again (RCN-218 §6.1). Every
//! track power-on starts a new session, so all decoders log on again.
//!
//! A decoder that announces itself after a logon enable is selected, asked
//! for the address it wants, and assigned that address; the decoder confirms
//! with its state. One select or assign is on the track at a time, so the
//! answer in the cutout that follows it belongs to the decoder it named. A
//! command that gets no answer is sent again a few times, then reported and
//! tried again after a pause, for as long as it takes: a decoder that was
//! selected no longer announces itself and obeys nothing but broadcasts until
//! it is assigned, so it is never given up for good.
//!
//! The table is pure state: the scheduler core feeds it answers and time and
//! takes the packets it wants on the track.

use dcc_protocol::packet::{DccAddress, DccPacket, LogonCommand, LogonGroup, LogonSessionId};
use embassy_time::{Duration, Instant};
use heapless::Vec;

use super::{DecoderUid, LogonAnswer};

/// Decoders the table follows. Above the 12 locomotive slots, so a decoder
/// that failed its logon does not take the place of a working one.
pub(super) const LOGON_TABLE_CAPACITY: usize = 16;

/// Times a select or an assign goes out in a row before its decoder is
/// reported as failed and left alone for [`RETRY_PAUSE`].
/// RCN-218 gives no number; OpenMRN sends each command twice.
const ATTEMPTS: u8 = 3;

/// Rest between two runs of attempts on a decoder that does not answer.
/// Long enough that a decoder taken off the track costs one packet in
/// hundreds; short enough that one put back answers the app within seconds.
const RETRY_PAUSE: Duration = Duration::from_secs(2);

/// How long an answer may take to come back from the RailCom side once its
/// command is on the track. The answer of a cutout normally arrives within a
/// few packets; this only covers a window lost on the way.
const ANSWER_TIMEOUT: Duration = Duration::from_millis(300);

/// First address tried for a decoder whose own address is taken; the search
/// goes down from here. The highest address the Z21 apps let the user dial.
const TEMPORARY_ADDRESS_TOP: u16 = 9_999;

/// Period of the logon enable. RCN-218 §3.2 asks for one at least every
/// 300 ms; the margin covers the safety and programming packets that go out
/// ahead of it when its turn comes.
const ENABLE_INTERVAL: Duration = Duration::from_millis(250);

/// Step the session moves at each track power-on. A decoder keeps its
/// registration across a step of one (RCN-218 §7.1), so two makes every
/// decoder log on again.
const SESSION_STEP: u8 = 2;

/// Invitations a decoder may let pass after its first collision; the range
/// doubles at each further one up to [`BACKOFF_RANGE_MAX`] (RCN-218 §7.2).
const BACKOFF_RANGE_FIRST: u8 = 8;
const BACKOFF_RANGE_MAX: u8 = 64;

/// Logon enable every decoder not registered answers at once.
const GROUP_NOW: LogonGroup = match LogonGroup::new(3) {
    Some(group) => group,
    None => panic!("logon group out of range"),
};

/// Logon enable a decoder answers after the wait its collisions earned it.
const GROUP_ALL: LogonGroup = match LogonGroup::new(0) {
    Some(group) => group,
    None => panic!("logon group out of range"),
};

/// Command-station id for the logon enable, derived from the board's unique
/// address so it stays the same across boots and differs between boards
/// (RCN-218 annex E): FNV-1a over the six bytes, folded to 16 bits.
pub(super) fn station_id_from(unique: [u8; 6]) -> u16 {
    const OFFSET_BASIS: u32 = 0x811c_9dc5;
    const PRIME: u32 = 0x0100_0193;
    let hash = unique.iter().fold(OFFSET_BASIS, |hash, &byte| {
        (hash ^ u32::from(byte)).wrapping_mul(PRIME)
    });
    let [a, b, c, d] = hash.to_be_bytes();
    u16::from_be_bytes([a ^ c, b ^ d])
}

/// RCN-218 §3.4 select subcommand that asks for the short info.
const SELECT_SHORT_INFO: u8 = 0xff;

/// A change in the decoder table worth telling the owner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum LogonEvent {
    /// The decoder now answers to `address`, which differs from `wanted`
    /// when that one was taken.
    Registered {
        uid: DecoderUid,
        address: DccAddress,
        wanted: DccAddress,
    },
    /// The decoder gave no answer to `command`. It is tried again after a
    /// pause; this is told once, not at every failed run.
    Failed {
        uid: DecoderUid,
        command: LogonCommand,
    },
    /// The decoder announced itself and the table has no room for it.
    TableFull(DecoderUid),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    /// Announced; its short info is still to be read.
    Selecting,
    /// Its address is chosen and still to be confirmed.
    Assigning,
    Assigned,
    /// Known from an earlier session and not announced in this one. Its
    /// address is remembered for when it comes back, without holding it.
    Away,
}

#[derive(Debug, Clone, Copy)]
struct Entry {
    uid: DecoderUid,
    stage: Stage,
    /// Address the decoder asked for in its last short info.
    wanted: Option<DccAddress>,
    /// Address chosen for the decoder. Kept when it logs on again, so the
    /// address it already answers to is not taken for somebody else's.
    address: Option<DccAddress>,
    attempts_left: u8,
    /// Set after a failed run of attempts: no command before this instant.
    rest_until: Option<Instant>,
    /// Whether the failure was already told.
    failed: bool,
}

impl Entry {
    /// Whether a select or an assign is still owed to the decoder.
    fn is_pending(&self) -> bool {
        matches!(self.stage, Stage::Selecting | Stage::Assigning)
    }

    /// Whether the decoder may be sent a command now.
    fn is_due(&self, now: Instant) -> bool {
        self.is_pending() && self.rest_until.is_none_or(|rest_until| now >= rest_until)
    }
}

/// The run of logon enables of one session.
#[derive(Debug, Clone, Copy)]
struct Invitation {
    next_due: Instant,
    /// Handed out and not on the track yet.
    handed: bool,
    /// Collisions seen since the track was last quiet; zero while the
    /// invitation asks for an answer at once.
    collisions: u8,
    /// Invitations in a row that brought neither a collision nor a decoder.
    quiet: u8,
    /// Whether the last invitation brought a collision or a decoder.
    answered: bool,
}

impl Invitation {
    fn starting_at(now: Instant) -> Self {
        Self {
            next_due: now,
            handed: false,
            collisions: 0,
            quiet: 0,
            answered: false,
        }
    }

    /// Invitations the slowest decoder may still let pass.
    fn backoff_range(&self) -> u8 {
        BACKOFF_RANGE_FIRST
            .checked_shl(u32::from(self.collisions.saturating_sub(1)))
            .map_or(BACKOFF_RANGE_MAX, |range| range.min(BACKOFF_RANGE_MAX))
    }

    /// Counts the outcome of the last invitation and picks the next group:
    /// back to answering at once after a whole backoff range of quiet.
    fn next_group(&mut self) -> LogonGroup {
        if self.collisions > 0 {
            self.quiet = if self.answered { 0 } else { self.quiet + 1 };
            if self.quiet >= self.backoff_range() {
                self.collisions = 0;
                self.quiet = 0;
            }
        }
        self.answered = false;
        if self.collisions == 0 {
            GROUP_NOW
        } else {
            GROUP_ALL
        }
    }

    fn collided(&mut self) {
        self.collisions = self.collisions.saturating_add(1);
        self.answered = true;
    }
}

/// The one select or assign whose answer is awaited.
#[derive(Debug, Clone, Copy)]
struct InFlight {
    entry: usize,
    command: LogonCommand,
    packet: DccPacket,
    /// When the answer is given up; none until the packet is on the track.
    deadline: Option<Instant>,
}

pub(super) struct LogonTable {
    /// Entries are never removed, only reused while none of their commands
    /// is awaited, so an index stays valid.
    entries: Vec<Entry, LOGON_TABLE_CAPACITY>,
    in_flight: Option<InFlight>,
    station_id: u16,
    session: u8,
    /// None until the track is first powered.
    invitation: Option<Invitation>,
}

impl LogonTable {
    /// Sets the station's identity; the first session follows
    /// `session_seed`. Call before the track is first powered.
    pub(super) fn set_identity(&mut self, station_id: u16, session_seed: u8) {
        self.station_id = station_id;
        self.session = session_seed;
    }

    /// A table for the station `station_id`, whose first session follows
    /// `session_seed`.
    pub(super) const fn new(station_id: u16, session_seed: u8) -> Self {
        Self {
            entries: Vec::new(),
            in_flight: None,
            station_id,
            session: session_seed,
            invitation: None,
        }
    }

    /// Starts a new session: the track has just been powered, and every
    /// decoder on it logs on again.
    pub(super) fn start_session(&mut self, now: Instant) {
        self.session = self.session.wrapping_add(SESSION_STEP);
        self.in_flight = None;
        self.invitation = Some(Invitation::starting_at(now));
        for entry in &mut self.entries {
            entry.stage = Stage::Away;
            entry.attempts_left = ATTEMPTS;
            entry.rest_until = None;
            entry.failed = false;
        }
    }

    /// Takes one answer from the RailCom side.
    ///
    /// `in_use` tells whether a locomotive is being driven on an address.
    pub(super) fn on_answer(
        &mut self,
        answer: LogonAnswer,
        now: Instant,
        in_use: impl Fn(DccAddress) -> bool,
    ) -> Option<LogonEvent> {
        match answer {
            LogonAnswer::Announced(uid) => {
                if let Some(invitation) = &mut self.invitation {
                    invitation.answered = true;
                }
                self.announce(uid)
            }
            LogonAnswer::Collided => {
                if let Some(invitation) = &mut self.invitation {
                    invitation.collided();
                }
                None
            }
            LogonAnswer::WantsAddress(wanted) => {
                let entry = self.take_answered(LogonCommand::Select)?;
                let address = self.address_for(entry, wanted, in_use);
                let entry = &mut self.entries[entry];
                entry.wanted = Some(wanted);
                entry.address = Some(address);
                entry.stage = Stage::Assigning;
                entry.attempts_left = ATTEMPTS;
                None
            }
            LogonAnswer::Confirmed => {
                let entry = self.take_answered(LogonCommand::Assign)?;
                let entry = &mut self.entries[entry];
                entry.stage = Stage::Assigned;
                entry.failed = false;
                Some(LogonEvent::Registered {
                    uid: entry.uid,
                    address: entry.address?,
                    wanted: entry.wanted?,
                })
            }
            LogonAnswer::Unanswered(command) => {
                let entry = self.take_answered(command)?;
                self.attempt_failed(entry, command, now)
            }
        }
    }

    /// Gives up an answer that did not come back in time.
    pub(super) fn expire(&mut self, now: Instant) -> Option<LogonEvent> {
        let in_flight = self.in_flight?;
        if in_flight.deadline.is_none_or(|deadline| now < deadline) {
            return None;
        }
        self.in_flight = None;
        self.attempt_failed(in_flight.entry, in_flight.command, now)
    }

    /// Next logon packet to send: the invitation when it is due, else a
    /// select or assign if none is awaiting its answer.
    pub(super) fn next_packet(&mut self, now: Instant) -> Option<DccPacket> {
        if let Some(invitation) = &mut self.invitation
            && !invitation.handed
            && now >= invitation.next_due
        {
            invitation.handed = true;
            return Some(DccPacket::LogonEnable {
                group: invitation.next_group(),
                command_station_id: self.station_id,
                session_id: LogonSessionId::new(self.session),
            });
        }
        if self.in_flight.is_some() {
            return None;
        }
        let (index, entry) = self
            .entries
            .iter()
            .enumerate()
            .find(|(_, entry)| entry.is_due(now))?;
        let DecoderUid {
            manufacturer_id,
            decoder_id,
        } = entry.uid;
        let (command, packet) = match (entry.stage, entry.address) {
            (Stage::Assigning, Some(address)) => (
                LogonCommand::Assign,
                DccPacket::LogonAssign {
                    manufacturer_id,
                    decoder_id,
                    address,
                },
            ),
            _ => (
                LogonCommand::Select,
                DccPacket::LogonSelect {
                    manufacturer_id,
                    decoder_id,
                    subcommand: SELECT_SHORT_INFO,
                },
            ),
        };
        self.in_flight = Some(InFlight {
            entry: index,
            command,
            packet,
            deadline: None,
        });
        Some(packet)
    }

    /// Whether the last packet handed out has not reached the track yet.
    fn awaits_emission(&self) -> bool {
        self.in_flight
            .is_some_and(|in_flight| in_flight.deadline.is_none())
    }

    /// Forgets a packet that was handed out and dropped before the track, so
    /// it is handed out again. No attempt is spent.
    pub(super) fn requeue(&mut self) {
        if self.awaits_emission() {
            self.in_flight = None;
        }
        if let Some(invitation) = &mut self.invitation {
            invitation.handed = false;
        }
    }

    /// Stops waiting for the answer to a command that is on the track, so
    /// it is sent again. No attempt is spent.
    ///
    /// For a track that lost power under the decoder that was answering.
    pub(super) fn suspend(&mut self) {
        if !self.awaits_emission() {
            self.in_flight = None;
        }
    }

    /// Starts the wait for the answer when the awaited packet is on the track.
    pub(super) fn packet_emitted(&mut self, packet: DccPacket, now: Instant) {
        if matches!(packet, DccPacket::LogonEnable { .. }) {
            if let Some(invitation) = &mut self.invitation {
                invitation.handed = false;
                // Counted from when it was due, so a late one does not push
                // back all that follow.
                invitation.next_due = (invitation.next_due + ENABLE_INTERVAL).max(now);
            }
            return;
        }
        if let Some(in_flight) = &mut self.in_flight
            && in_flight.deadline.is_none()
            && in_flight.packet == packet
        {
            in_flight.deadline = Some(now + ANSWER_TIMEOUT);
        }
    }

    fn announce(&mut self, uid: DecoderUid) -> Option<LogonEvent> {
        let fresh = Entry {
            uid,
            stage: Stage::Selecting,
            wanted: None,
            address: None,
            attempts_left: ATTEMPTS,
            rest_until: None,
            failed: false,
        };
        if let Some(entry) = self.entries.iter_mut().find(|entry| entry.uid == uid) {
            // A decoder in the middle of its logon repeats its id until it is
            // selected. One that was assigned, now or in an earlier session,
            // starts over, and one left to rest is there to answer: it is
            // tried at once.
            if matches!(entry.stage, Stage::Assigned | Stage::Away) {
                entry.stage = Stage::Selecting;
                entry.attempts_left = ATTEMPTS;
            }
            entry.rest_until = None;
            return None;
        }
        if self.entries.push(fresh).is_ok() {
            return None;
        }
        // A decoder not seen in this session gives way first, then one that
        // does not answer.
        let reusable = self
            .entries
            .iter()
            .position(|entry| entry.stage == Stage::Away)
            .or_else(|| {
                self.entries
                    .iter()
                    .position(|entry| entry.failed && entry.is_pending())
            });
        match reusable.map(|index| &mut self.entries[index]) {
            Some(silent) => {
                *silent = fresh;
                None
            }
            None => Some(LogonEvent::TableFull(uid)),
        }
    }

    /// Closes the wait for the answer to `command`, returning its entry.
    ///
    /// None when no such command is on the track: the answer is a late one,
    /// already given up.
    fn take_answered(&mut self, command: LogonCommand) -> Option<usize> {
        let in_flight = self.in_flight?;
        if in_flight.command != command || in_flight.deadline.is_none() {
            return None;
        }
        self.in_flight = None;
        Some(in_flight.entry)
    }

    fn attempt_failed(
        &mut self,
        entry: usize,
        command: LogonCommand,
        now: Instant,
    ) -> Option<LogonEvent> {
        let entry = &mut self.entries[entry];
        entry.attempts_left = entry.attempts_left.saturating_sub(1);
        if entry.attempts_left > 0 {
            return None;
        }
        entry.attempts_left = ATTEMPTS;
        entry.rest_until = Some(now + RETRY_PAUSE);
        if entry.failed {
            return None;
        }
        entry.failed = true;
        Some(LogonEvent::Failed {
            uid: entry.uid,
            command,
        })
    }

    /// Address to assign to a decoder that asks for `wanted`.
    ///
    /// Its own when free. A decoder that logs on again asking for the same
    /// address keeps the one it was given, unless another decoder took it
    /// meanwhile: the locomotive the app drives on it is this very one.
    /// Otherwise the highest free address from [`TEMPORARY_ADDRESS_TOP`] down.
    fn address_for(
        &self,
        entry: usize,
        wanted: DccAddress,
        in_use: impl Fn(DccAddress) -> bool,
    ) -> DccAddress {
        let held_by_another = |address: DccAddress| {
            self.entries.iter().enumerate().any(|(index, other)| {
                index != entry && other.stage != Stage::Away && other.address == Some(address)
            })
        };
        let own = &self.entries[entry];
        if own.wanted == Some(wanted)
            && let Some(held) = own.address
            && !held_by_another(held)
        {
            return held;
        }
        let taken = |address: DccAddress| held_by_another(address) || in_use(address);
        if !taken(wanted) {
            return wanted;
        }
        (DccAddress::LONG_MIN..=TEMPORARY_ADDRESS_TOP)
            .rev()
            .filter_map(DccAddress::new_long)
            .find(|&address| !taken(address))
            // Unreachable: the table and the slots hold a few dozen addresses.
            .unwrap_or(wanted)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const START: Instant = Instant::from_secs(10);
    const STATION: u16 = 0x0abc;

    fn uid(decoder_id: u32) -> DecoderUid {
        DecoderUid {
            manufacturer_id: 145,
            decoder_id,
        }
    }

    fn short(address: u8) -> DccAddress {
        DccAddress::new_short(address).unwrap()
    }

    fn long(address: u16) -> DccAddress {
        DccAddress::new_long(address).unwrap()
    }

    fn nothing_driven(_: DccAddress) -> bool {
        false
    }

    fn select(decoder_id: u32) -> DccPacket {
        DccPacket::LogonSelect {
            manufacturer_id: 145,
            decoder_id,
            subcommand: 0xff,
        }
    }

    fn assign(decoder_id: u32, address: DccAddress) -> DccPacket {
        DccPacket::LogonAssign {
            manufacturer_id: 145,
            decoder_id,
            address,
        }
    }

    /// Hands out the next packet and puts it on the track.
    fn send(table: &mut LogonTable) -> Option<DccPacket> {
        let packet = table.next_packet(START)?;
        table.packet_emitted(packet, START);
        Some(packet)
    }

    /// Runs a whole logon of `decoder_id` asking for `wanted`.
    fn log_on(
        table: &mut LogonTable,
        decoder_id: u32,
        wanted: DccAddress,
        in_use: impl Fn(DccAddress) -> bool + Copy,
    ) -> Option<LogonEvent> {
        table.on_answer(LogonAnswer::Announced(uid(decoder_id)), START, in_use);
        assert_eq!(send(table), Some(select(decoder_id)));
        table.on_answer(LogonAnswer::WantsAddress(wanted), START, in_use);
        assert!(matches!(send(table), Some(DccPacket::LogonAssign { .. })));
        table.on_answer(LogonAnswer::Confirmed, START, in_use)
    }

    #[test]
    fn a_decoder_is_selected_then_assigned_the_address_it_wants() {
        let mut table = LogonTable::new(STATION, 0);

        assert_eq!(
            table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven),
            None
        );
        assert_eq!(send(&mut table), Some(select(7)));
        assert_eq!(
            table.on_answer(LogonAnswer::WantsAddress(short(3)), START, nothing_driven),
            None
        );
        assert_eq!(send(&mut table), Some(assign(7, short(3))));
        assert_eq!(
            table.on_answer(LogonAnswer::Confirmed, START, nothing_driven),
            Some(LogonEvent::Registered {
                uid: uid(7),
                address: short(3),
                wanted: short(3),
            })
        );
        assert_eq!(table.next_packet(START), None);
    }

    #[test]
    fn one_command_waits_for_its_answer_before_the_next_goes_out() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(1)), START, nothing_driven);
        table.on_answer(LogonAnswer::Announced(uid(2)), START, nothing_driven);

        assert_eq!(send(&mut table), Some(select(1)));
        assert_eq!(table.next_packet(START), None);
        table.on_answer(LogonAnswer::WantsAddress(short(3)), START, nothing_driven);
        assert_eq!(send(&mut table), Some(assign(1, short(3))));
        table.on_answer(LogonAnswer::Confirmed, START, nothing_driven);
        assert_eq!(send(&mut table), Some(select(2)));
    }

    #[test]
    fn an_unanswered_select_is_sent_again_then_reported() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);

        for _ in 0..2 {
            assert_eq!(send(&mut table), Some(select(7)));
            assert_eq!(
                table.on_answer(
                    LogonAnswer::Unanswered(LogonCommand::Select),
                    START,
                    nothing_driven
                ),
                None
            );
        }
        assert_eq!(send(&mut table), Some(select(7)));
        assert_eq!(
            table.on_answer(
                LogonAnswer::Unanswered(LogonCommand::Select),
                START,
                nothing_driven
            ),
            Some(LogonEvent::Failed {
                uid: uid(7),
                command: LogonCommand::Select,
            })
        );
        assert_eq!(table.next_packet(START), None);
    }

    #[test]
    fn an_unanswered_assign_is_sent_again_and_can_still_succeed() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);
        send(&mut table);
        table.on_answer(
            LogonAnswer::WantsAddress(long(1_000)),
            START,
            nothing_driven,
        );

        assert_eq!(send(&mut table), Some(assign(7, long(1_000))));
        table.on_answer(
            LogonAnswer::Unanswered(LogonCommand::Assign),
            START,
            nothing_driven,
        );
        assert_eq!(send(&mut table), Some(assign(7, long(1_000))));
        assert!(matches!(
            table.on_answer(LogonAnswer::Confirmed, START, nothing_driven),
            Some(LogonEvent::Registered { .. })
        ));
    }

    #[test]
    fn a_resting_decoder_is_tried_at_once_when_it_announces_itself() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);
        for _ in 0..ATTEMPTS {
            send(&mut table);
            table.on_answer(
                LogonAnswer::Unanswered(LogonCommand::Select),
                START,
                nothing_driven,
            );
        }
        assert_eq!(table.next_packet(START), None);

        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);

        assert_eq!(send(&mut table), Some(select(7)));
    }

    #[test]
    fn a_silent_decoder_is_tried_again_after_a_pause_and_reported_once() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);
        let mut reports = 0;

        for run in 0..3u32 {
            let now = START + RETRY_PAUSE * run;
            for _ in 0..ATTEMPTS {
                let packet = table.next_packet(now).expect("a select is due");
                assert_eq!(packet, select(7));
                table.packet_emitted(packet, now);
                let event = table.on_answer(
                    LogonAnswer::Unanswered(LogonCommand::Select),
                    now,
                    nothing_driven,
                );
                reports += usize::from(event.is_some());
            }
            assert_eq!(table.next_packet(now + RETRY_PAUSE / 2), None);
        }

        assert_eq!(reports, 1);
    }

    #[test]
    fn a_decoder_selected_and_not_confirmed_is_assigned_again_after_a_pause() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);
        send(&mut table);
        table.on_answer(LogonAnswer::WantsAddress(short(3)), START, nothing_driven);
        for _ in 0..ATTEMPTS {
            send(&mut table);
            table.on_answer(
                LogonAnswer::Unanswered(LogonCommand::Assign),
                START,
                nothing_driven,
            );
        }
        assert_eq!(table.next_packet(START), None);

        assert_eq!(
            table.next_packet(START + RETRY_PAUSE),
            Some(assign(7, short(3)))
        );
    }

    #[test]
    fn a_repeated_announcement_does_not_restart_a_logon_under_way() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);
        send(&mut table);
        table.on_answer(LogonAnswer::WantsAddress(short(3)), START, nothing_driven);

        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);

        assert_eq!(send(&mut table), Some(assign(7, short(3))));
    }

    #[test]
    fn an_unanswered_assign_is_reported_after_the_last_attempt() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);
        send(&mut table);
        table.on_answer(LogonAnswer::WantsAddress(short(3)), START, nothing_driven);

        let events: std::vec::Vec<_> = (0..ATTEMPTS)
            .map(|_| {
                assert_eq!(send(&mut table), Some(assign(7, short(3))));
                table.on_answer(
                    LogonAnswer::Unanswered(LogonCommand::Assign),
                    START,
                    nothing_driven,
                )
            })
            .collect();

        assert_eq!(
            events,
            [
                None,
                None,
                Some(LogonEvent::Failed {
                    uid: uid(7),
                    command: LogonCommand::Assign,
                })
            ]
        );
        assert_eq!(table.next_packet(START), None);
    }

    #[test]
    fn a_suspended_wait_sends_the_command_again_without_spending_an_attempt() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);

        for _ in 0..10 {
            assert_eq!(send(&mut table), Some(select(7)));
            table.suspend();
            assert_eq!(table.expire(START + ANSWER_TIMEOUT), None);
        }

        assert_eq!(send(&mut table), Some(select(7)));
    }

    #[test]
    fn a_lost_answer_stops_being_awaited_after_the_timeout() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);
        assert_eq!(send(&mut table), Some(select(7)));

        assert_eq!(table.expire(START + ANSWER_TIMEOUT / 2), None);
        assert_eq!(table.next_packet(START), None);
        assert_eq!(table.expire(START + ANSWER_TIMEOUT), None);

        assert_eq!(send(&mut table), Some(select(7)));
    }

    #[test]
    fn an_answer_that_arrives_after_its_command_was_given_up_is_ignored() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(1)), START, nothing_driven);
        send(&mut table);
        table.expire(START + ANSWER_TIMEOUT);

        // The select is handed out again but is not on the track yet.
        assert_eq!(table.next_packet(START), Some(select(1)));
        table.on_answer(LogonAnswer::WantsAddress(short(3)), START, nothing_driven);
        table.packet_emitted(select(1), START);
        // An assign confirmation does not answer a select either.
        table.on_answer(LogonAnswer::Confirmed, START, nothing_driven);

        table.on_answer(LogonAnswer::WantsAddress(short(5)), START, nothing_driven);
        assert_eq!(send(&mut table), Some(assign(1, short(5))));
    }

    #[test]
    fn a_packet_dropped_before_the_track_is_handed_out_again_at_no_cost() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);

        for _ in 0..10 {
            assert_eq!(table.next_packet(START), Some(select(7)));
            assert!(table.awaits_emission());
            table.requeue();
        }

        assert_eq!(send(&mut table), Some(select(7)));
        assert!(!table.awaits_emission());
    }

    #[test]
    fn two_decoders_asking_for_one_address_get_different_ones() {
        let mut table = LogonTable::new(STATION, 0);

        log_on(&mut table, 1, short(3), nothing_driven);
        let second = log_on(&mut table, 2, short(3), nothing_driven);

        assert_eq!(
            second,
            Some(LogonEvent::Registered {
                uid: uid(2),
                address: long(9_999),
                wanted: short(3),
            })
        );
    }

    #[test]
    fn an_address_driven_by_the_app_is_not_given_to_a_new_decoder() {
        let mut table = LogonTable::new(STATION, 0);
        let driven = |address| address == short(3) || address == long(9_999);

        let event = log_on(&mut table, 1, short(3), driven);

        assert_eq!(
            event,
            Some(LogonEvent::Registered {
                uid: uid(1),
                address: long(9_998),
                wanted: short(3),
            })
        );
    }

    #[test]
    fn temporary_addresses_go_down_from_the_top_without_repeating() {
        let mut table = LogonTable::new(STATION, 0);
        log_on(&mut table, 1, short(3), nothing_driven);

        for (decoder_id, expected) in [(2, 9_999), (3, 9_998), (4, 9_997)] {
            assert_eq!(
                log_on(&mut table, decoder_id, short(3), nothing_driven),
                Some(LogonEvent::Registered {
                    uid: uid(decoder_id),
                    address: long(expected),
                    wanted: short(3),
                })
            );
        }
    }

    #[test]
    fn a_decoder_logging_on_again_keeps_its_address_while_the_app_drives_it() {
        let mut table = LogonTable::new(STATION, 0);
        log_on(&mut table, 1, short(3), nothing_driven);
        log_on(&mut table, 2, short(3), nothing_driven);
        let driven = |address| address == short(3) || address == long(9_999);

        assert_eq!(
            log_on(&mut table, 1, short(3), driven),
            Some(LogonEvent::Registered {
                uid: uid(1),
                address: short(3),
                wanted: short(3),
            })
        );
        assert_eq!(
            log_on(&mut table, 2, short(3), driven),
            Some(LogonEvent::Registered {
                uid: uid(2),
                address: long(9_999),
                wanted: short(3),
            })
        );
    }

    #[test]
    fn a_decoder_whose_address_was_changed_leaves_its_temporary_one() {
        let mut table = LogonTable::new(STATION, 0);
        log_on(&mut table, 1, short(3), nothing_driven);
        log_on(&mut table, 2, short(3), nothing_driven);

        assert_eq!(
            log_on(&mut table, 2, short(5), nothing_driven),
            Some(LogonEvent::Registered {
                uid: uid(2),
                address: short(5),
                wanted: short(5),
            })
        );
    }

    #[test]
    fn twelve_decoders_log_on_one_after_the_other() {
        let mut table = LogonTable::new(STATION, 0);
        for decoder_id in 1..=12 {
            table.on_answer(
                LogonAnswer::Announced(uid(decoder_id)),
                START,
                nothing_driven,
            );
        }

        for decoder_id in 1..=12u8 {
            assert_eq!(send(&mut table), Some(select(u32::from(decoder_id))));
            table.on_answer(
                LogonAnswer::WantsAddress(short(decoder_id)),
                START,
                nothing_driven,
            );
            assert_eq!(
                send(&mut table),
                Some(assign(u32::from(decoder_id), short(decoder_id)))
            );
            assert!(
                table
                    .on_answer(LogonAnswer::Confirmed, START, nothing_driven)
                    .is_some()
            );
        }
        assert_eq!(table.next_packet(START), None);
    }

    #[test]
    fn a_full_table_reuses_a_silent_decoder_and_reports_when_none_is_left() {
        let mut table = LogonTable::new(STATION, 0);
        for decoder_id in 0..LOGON_TABLE_CAPACITY as u32 {
            table.on_answer(
                LogonAnswer::Announced(uid(decoder_id)),
                START,
                nothing_driven,
            );
        }

        assert_eq!(
            table.on_answer(LogonAnswer::Announced(uid(100)), START, nothing_driven),
            Some(LogonEvent::TableFull(uid(100)))
        );

        for _ in 0..ATTEMPTS {
            send(&mut table);
            table.on_answer(
                LogonAnswer::Unanswered(LogonCommand::Select),
                START,
                nothing_driven,
            );
        }
        assert_eq!(
            table.on_answer(LogonAnswer::Announced(uid(100)), START, nothing_driven),
            None
        );
    }

    /// Sends the invitation due at `now`, returning its group and session.
    fn invite(table: &mut LogonTable, now: Instant) -> Option<(LogonGroup, u8)> {
        let packet = table.next_packet(now)?;
        let DccPacket::LogonEnable {
            group,
            command_station_id,
            session_id,
        } = packet
        else {
            panic!("expected a logon enable, got {packet:?}");
        };
        assert_eq!(command_station_id, STATION);
        table.packet_emitted(packet, now);
        Some((group, session_id.value()))
    }

    /// Groups of the next `count` invitations, one per interval from `from`.
    fn invitation_groups(
        table: &mut LogonTable,
        from: Instant,
        count: u32,
    ) -> std::vec::Vec<LogonGroup> {
        (0..count)
            .map(|turn| {
                invite(table, from + ENABLE_INTERVAL * turn)
                    .expect("an invitation is due")
                    .0
            })
            .collect()
    }

    #[test]
    fn no_invitation_goes_out_before_the_track_is_first_powered() {
        let mut table = LogonTable::new(STATION, 0);

        assert_eq!(table.next_packet(START), None);
    }

    #[test]
    fn the_invitation_goes_out_at_power_on_and_then_at_every_interval() {
        let mut table = LogonTable::new(STATION, 40);
        table.start_session(START);

        assert_eq!(invite(&mut table, START), Some((GROUP_NOW, 42)));
        assert_eq!(
            table.next_packet(START + ENABLE_INTERVAL - Duration::from_millis(1)),
            None
        );
        assert_eq!(
            invite(&mut table, START + ENABLE_INTERVAL),
            Some((GROUP_NOW, 42))
        );
    }

    #[test]
    fn a_late_invitation_does_not_push_back_the_one_after_it() {
        let mut table = LogonTable::new(STATION, 0);
        table.start_session(START);
        invite(&mut table, START);
        let late = Duration::from_millis(40);

        assert!(invite(&mut table, START + ENABLE_INTERVAL + late).is_some());

        assert!(invite(&mut table, START + ENABLE_INTERVAL * 2).is_some());
    }

    #[test]
    fn a_collision_before_the_first_session_is_ignored() {
        let mut table = LogonTable::new(STATION, 0);

        assert_eq!(
            table.on_answer(LogonAnswer::Collided, START, nothing_driven),
            None
        );
        table.start_session(START);
        assert_eq!(
            invite(&mut table, START).map(|(group, _)| group),
            Some(GROUP_NOW)
        );
    }

    #[test]
    fn every_power_on_moves_the_session_two_steps_and_wraps() {
        let mut table = LogonTable::new(STATION, 251);

        let sessions: std::vec::Vec<_> = (0..4)
            .map(|_| {
                table.start_session(START);
                invite(&mut table, START).expect("an invitation is due").1
            })
            .collect();

        assert_eq!(sessions, [253, 255, 1, 3]);
    }

    #[test]
    fn an_invitation_dropped_before_the_track_is_handed_out_again() {
        let mut table = LogonTable::new(STATION, 0);
        table.start_session(START);
        assert!(table.next_packet(START).is_some());
        assert_eq!(table.next_packet(START), None);

        table.requeue();

        assert!(invite(&mut table, START).is_some());
    }

    #[test]
    fn the_invitation_goes_out_while_a_select_awaits_its_answer() {
        let mut table = LogonTable::new(STATION, 0);
        table.start_session(START);
        invite(&mut table, START);
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);
        assert_eq!(send(&mut table), Some(select(7)));

        assert!(invite(&mut table, START + ENABLE_INTERVAL).is_some());
    }

    #[test]
    fn a_collision_lets_the_decoders_back_off_until_a_whole_range_is_quiet() {
        let mut table = LogonTable::new(STATION, 0);
        table.start_session(START);
        invite(&mut table, START);
        table.on_answer(LogonAnswer::Collided, START, nothing_driven);

        let groups = invitation_groups(&mut table, START + ENABLE_INTERVAL, 10);

        let mut expected = vec![GROUP_ALL; usize::from(BACKOFF_RANGE_FIRST)];
        expected.extend([GROUP_NOW, GROUP_NOW]);
        assert_eq!(groups, expected);
    }

    #[test]
    fn a_decoder_announcing_itself_restarts_the_count_of_quiet_invitations() {
        let mut table = LogonTable::new(STATION, 0);
        table.start_session(START);
        invite(&mut table, START);
        table.on_answer(LogonAnswer::Collided, START, nothing_driven);
        let later = START + ENABLE_INTERVAL;
        invitation_groups(&mut table, later, 5);

        table.on_answer(LogonAnswer::Announced(uid(1)), later, nothing_driven);
        let from = later + ENABLE_INTERVAL * 5;
        let groups = invitation_groups(&mut table, from, u32::from(BACKOFF_RANGE_FIRST) + 1);

        // The select of the announced decoder takes no invitation's place.
        assert_eq!(
            groups.iter().filter(|&&group| group == GROUP_ALL).count(),
            usize::from(BACKOFF_RANGE_FIRST)
        );
        assert_eq!(groups.last(), Some(&GROUP_NOW));
    }

    #[test]
    fn each_further_collision_doubles_the_quiet_range_up_to_the_longest_backoff() {
        for (collisions, range) in [(1, 8), (2, 16), (3, 32), (4, 64), (9, 64)] {
            let mut table = LogonTable::new(STATION, 0);
            table.start_session(START);
            invite(&mut table, START);
            for _ in 0..collisions {
                table.on_answer(LogonAnswer::Collided, START, nothing_driven);
            }

            let groups = invitation_groups(&mut table, START + ENABLE_INTERVAL, range + 1);

            assert_eq!(
                groups.iter().filter(|&&group| group == GROUP_ALL).count(),
                range as usize,
                "after {collisions} collisions"
            );
            assert_eq!(groups.last(), Some(&GROUP_NOW));
        }
    }

    #[test]
    fn a_new_session_makes_a_registered_decoder_log_on_again_on_its_address() {
        let mut table = LogonTable::new(STATION, 0);
        log_on(&mut table, 1, short(3), nothing_driven);
        let driven = |address| address == short(3);

        table.start_session(START);
        invite(&mut table, START);

        assert_eq!(table.next_packet(START), None);
        assert_eq!(
            log_on(&mut table, 1, short(3), driven),
            Some(LogonEvent::Registered {
                uid: uid(1),
                address: short(3),
                wanted: short(3),
            })
        );
    }

    #[test]
    fn a_new_session_drops_the_command_that_was_awaited() {
        let mut table = LogonTable::new(STATION, 0);
        table.on_answer(LogonAnswer::Announced(uid(1)), START, nothing_driven);
        send(&mut table);

        table.start_session(START);
        invite(&mut table, START);

        assert_eq!(table.expire(START + ANSWER_TIMEOUT), None);
        table.on_answer(LogonAnswer::WantsAddress(short(3)), START, nothing_driven);
        assert_eq!(table.next_packet(START), None);
    }

    #[test]
    fn the_address_of_a_decoder_gone_from_the_track_is_free_for_another() {
        let mut table = LogonTable::new(STATION, 0);
        log_on(&mut table, 1, short(3), nothing_driven);
        table.start_session(START);
        invite(&mut table, START);

        assert_eq!(
            log_on(&mut table, 2, short(3), nothing_driven),
            Some(LogonEvent::Registered {
                uid: uid(2),
                address: short(3),
                wanted: short(3),
            })
        );
        // The first one comes back and finds its address taken.
        assert_eq!(
            log_on(&mut table, 1, short(3), nothing_driven),
            Some(LogonEvent::Registered {
                uid: uid(1),
                address: long(9_999),
                wanted: short(3),
            })
        );
    }

    #[test]
    fn a_full_table_gives_the_place_of_a_decoder_gone_from_the_track() {
        let mut table = LogonTable::new(STATION, 0);
        for decoder_id in 0..LOGON_TABLE_CAPACITY as u32 {
            table.on_answer(
                LogonAnswer::Announced(uid(decoder_id)),
                START,
                nothing_driven,
            );
        }
        table.start_session(START);

        assert_eq!(
            table.on_answer(LogonAnswer::Announced(uid(100)), START, nothing_driven),
            None
        );
    }

    #[test]
    fn the_station_id_is_stable_for_a_board_and_differs_between_boards() {
        let board = [0x40, 0x4c, 0xca, 0x12, 0x34, 0x56];
        let sibling = [0x40, 0x4c, 0xca, 0x12, 0x34, 0x57];

        assert_eq!(station_id_from(board), station_id_from(board));
        assert_ne!(station_id_from(board), station_id_from(sibling));
    }
}
