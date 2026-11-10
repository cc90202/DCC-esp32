//! Decoder table and logon sequence of RCN-218 (DCC-A).
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

use dcc_protocol::packet::{DccAddress, DccPacket, LogonCommand};
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
}

impl LogonTable {
    pub(super) const fn new() -> Self {
        Self {
            entries: Vec::new(),
            in_flight: None,
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
            LogonAnswer::Announced(uid) => self.announce(uid),
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

    /// Next select or assign to send, if none is awaiting its answer.
    pub(super) fn next_packet(&mut self, now: Instant) -> Option<DccPacket> {
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
            // selected. One that was assigned starts over, and one left to
            // rest is there to answer: it is tried at once.
            if entry.stage == Stage::Assigned {
                entry.stage = Stage::Selecting;
                entry.attempts_left = ATTEMPTS;
            }
            entry.rest_until = None;
            return None;
        }
        if self.entries.push(fresh).is_ok() {
            return None;
        }
        match self
            .entries
            .iter_mut()
            .find(|entry| entry.failed && entry.is_pending())
        {
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
    /// address keeps the one it was given. Otherwise the highest free address
    /// from [`TEMPORARY_ADDRESS_TOP`] down.
    fn address_for(
        &self,
        entry: usize,
        wanted: DccAddress,
        in_use: impl Fn(DccAddress) -> bool,
    ) -> DccAddress {
        let own = &self.entries[entry];
        if own.wanted == Some(wanted)
            && let Some(held) = own.address
        {
            return held;
        }
        let held_by_another = |address: DccAddress| {
            self.entries
                .iter()
                .enumerate()
                .any(|(index, other)| index != entry && other.address == Some(address))
        };
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
        let mut table = LogonTable::new();

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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);
        send(&mut table);
        table.on_answer(LogonAnswer::WantsAddress(short(3)), START, nothing_driven);

        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);

        assert_eq!(send(&mut table), Some(assign(7, short(3))));
    }

    #[test]
    fn an_unanswered_assign_is_reported_after_the_last_attempt() {
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
        table.on_answer(LogonAnswer::Announced(uid(7)), START, nothing_driven);
        assert_eq!(send(&mut table), Some(select(7)));

        assert_eq!(table.expire(START + ANSWER_TIMEOUT / 2), None);
        assert_eq!(table.next_packet(START), None);
        assert_eq!(table.expire(START + ANSWER_TIMEOUT), None);

        assert_eq!(send(&mut table), Some(select(7)));
    }

    #[test]
    fn an_answer_that_arrives_after_its_command_was_given_up_is_ignored() {
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();

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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
        let mut table = LogonTable::new();
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
}
