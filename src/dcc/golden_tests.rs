//! Reference encodings of every DCC packet kind.
//!
//! Pins the on-wire bytes of each packet kind, so that changes to the packet
//! encoder, the function-group representation and the validator can prove
//! the wire bytes did not change.
//!
//! The corpus has two independent builders, so a change to one cannot
//! rewrite the proof:
//!
//! - [`golden_packets_a`] builds [`DccPacket`] values directly and pins the
//!   byte layout of each variant.
//! - [`golden_packets_b`] drives a fresh [`SlotManager`] through
//!   `handle_loco_request` (or, for POM, `apply_command` with
//!   `SchedulerCommand::ProgramOnMain` under an accepted lease) and records
//!   the first packet it emits whose instruction byte has the expected kind.
//!   It pins the mapping from locomotive state to packets, and it selects
//!   packets by their bytes only, never by [`DccPacket`] variant.
//!
//! The expected-bytes column is fixed: never edit it to make a test pass;
//! only the builder column of A may change.

use std::vec::Vec;

use crate::authority::{LeaseEpoch, LeasePermit};
use crate::dcc::scheduler::{
    FunctionChange, FunctionIndex, LocoRequest, LocoRequestResult, LogicalSpeed, SchedulerCommand,
    SpeedFormat,
};
use crate::dcc::validator::decode_pulses;
use crate::dcc::{
    BinaryStateAddress, DccAddress, DccPacket, Direction, LogonGroup, LogonSessionId, NmraSpeed28,
    NmraSpeed128, PomCv, PomRequestId, ServiceModeCv, SlotManager, encode_dcc_packet,
};

/// Upper bound on packets pulled from a [`SlotManager`] while looking for the
/// expected one; reaching it means the scheduler stopped emitting that kind.
const MAX_PULLS: usize = 16;

fn short(address: u8) -> DccAddress {
    DccAddress::new_short(address).expect("golden short address must be valid")
}

fn long(address: u16) -> DccAddress {
    DccAddress::new_long(address).expect("golden long address must be valid")
}

fn speed28(value: u8) -> NmraSpeed28 {
    NmraSpeed28::new(value).expect("golden 28-step speed must be valid")
}

fn speed128(value: u8) -> NmraSpeed128 {
    NmraSpeed128::new(value).expect("golden 128-step speed must be valid")
}

fn pom_cv(cv: u16) -> PomCv {
    PomCv::new(cv).expect("golden POM CV must be valid")
}

fn service_cv(cv: u16) -> ServiceModeCv {
    ServiceModeCv::new(cv).expect("golden service-mode CV must be valid")
}

fn logon_group(group: u8) -> LogonGroup {
    LogonGroup::new(group).expect("golden logon group must be valid")
}

fn binary_state(address: u8) -> BinaryStateAddress {
    BinaryStateAddress::new(address).expect("golden binary-state address must be valid")
}

/// Builder A: packet literals and the bytes `to_bytes()` produced for them.
#[rustfmt::skip]
fn golden_packets_a() -> Vec<(&'static str, DccPacket, &'static [u8])> {
    std::vec![
        ("idle", DccPacket::Idle, &[0xFF, 0x00, 0xFF]),
        ("reset", DccPacket::Reset, &[0x00, 0x00, 0x00]),
        ("broadcast stop", DccPacket::BroadcastStop, &[0x00, 0x61, 0x61]),
        ("speed28 short 3 forward 0", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(0) }, &[0x03, 0x60, 0x63]),
        ("speed28 short 3 forward 1", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(1) }, &[0x03, 0x61, 0x62]),
        ("speed28 short 3 forward 2", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(2) }, &[0x03, 0x62, 0x61]),
        ("speed28 short 3 forward 3", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(3) }, &[0x03, 0x72, 0x71]),
        ("speed28 short 3 forward 4", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(4) }, &[0x03, 0x63, 0x60]),
        ("speed28 short 3 forward 5", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(5) }, &[0x03, 0x73, 0x70]),
        ("speed28 short 3 forward 6", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(6) }, &[0x03, 0x64, 0x67]),
        ("speed28 short 3 forward 7", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(7) }, &[0x03, 0x74, 0x77]),
        ("speed28 short 3 forward 8", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(8) }, &[0x03, 0x65, 0x66]),
        ("speed28 short 3 forward 9", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(9) }, &[0x03, 0x75, 0x76]),
        ("speed28 short 3 forward 10", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(10) }, &[0x03, 0x66, 0x65]),
        ("speed28 short 3 forward 11", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(11) }, &[0x03, 0x76, 0x75]),
        ("speed28 short 3 forward 12", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(12) }, &[0x03, 0x67, 0x64]),
        ("speed28 short 3 forward 13", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(13) }, &[0x03, 0x77, 0x74]),
        ("speed28 short 3 forward 14", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(14) }, &[0x03, 0x68, 0x6B]),
        ("speed28 short 3 forward 15", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(15) }, &[0x03, 0x78, 0x7B]),
        ("speed28 short 3 forward 16", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(16) }, &[0x03, 0x69, 0x6A]),
        ("speed28 short 3 forward 17", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(17) }, &[0x03, 0x79, 0x7A]),
        ("speed28 short 3 forward 18", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(18) }, &[0x03, 0x6A, 0x69]),
        ("speed28 short 3 forward 19", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(19) }, &[0x03, 0x7A, 0x79]),
        ("speed28 short 3 forward 20", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(20) }, &[0x03, 0x6B, 0x68]),
        ("speed28 short 3 forward 21", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(21) }, &[0x03, 0x7B, 0x78]),
        ("speed28 short 3 forward 22", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(22) }, &[0x03, 0x6C, 0x6F]),
        ("speed28 short 3 forward 23", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(23) }, &[0x03, 0x7C, 0x7F]),
        ("speed28 short 3 forward 24", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(24) }, &[0x03, 0x6D, 0x6E]),
        ("speed28 short 3 forward 25", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(25) }, &[0x03, 0x7D, 0x7E]),
        ("speed28 short 3 forward 26", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(26) }, &[0x03, 0x6E, 0x6D]),
        ("speed28 short 3 forward 27", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(27) }, &[0x03, 0x7E, 0x7D]),
        ("speed28 short 3 forward 28", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(28) }, &[0x03, 0x6F, 0x6C]),
        ("speed28 short 3 forward 29", DccPacket::Speed28 { address: short(3), direction: Direction::Forward, speed: speed28(29) }, &[0x03, 0x7F, 0x7C]),
        ("speed28 short 3 reverse 0", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(0) }, &[0x03, 0x40, 0x43]),
        ("speed28 short 3 reverse 1", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(1) }, &[0x03, 0x41, 0x42]),
        ("speed28 short 3 reverse 2", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(2) }, &[0x03, 0x42, 0x41]),
        ("speed28 short 3 reverse 3", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(3) }, &[0x03, 0x52, 0x51]),
        ("speed28 short 3 reverse 4", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(4) }, &[0x03, 0x43, 0x40]),
        ("speed28 short 3 reverse 5", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(5) }, &[0x03, 0x53, 0x50]),
        ("speed28 short 3 reverse 6", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(6) }, &[0x03, 0x44, 0x47]),
        ("speed28 short 3 reverse 7", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(7) }, &[0x03, 0x54, 0x57]),
        ("speed28 short 3 reverse 8", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(8) }, &[0x03, 0x45, 0x46]),
        ("speed28 short 3 reverse 9", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(9) }, &[0x03, 0x55, 0x56]),
        ("speed28 short 3 reverse 10", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(10) }, &[0x03, 0x46, 0x45]),
        ("speed28 short 3 reverse 11", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(11) }, &[0x03, 0x56, 0x55]),
        ("speed28 short 3 reverse 12", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(12) }, &[0x03, 0x47, 0x44]),
        ("speed28 short 3 reverse 13", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(13) }, &[0x03, 0x57, 0x54]),
        ("speed28 short 3 reverse 14", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(14) }, &[0x03, 0x48, 0x4B]),
        ("speed28 short 3 reverse 15", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(15) }, &[0x03, 0x58, 0x5B]),
        ("speed28 short 3 reverse 16", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(16) }, &[0x03, 0x49, 0x4A]),
        ("speed28 short 3 reverse 17", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(17) }, &[0x03, 0x59, 0x5A]),
        ("speed28 short 3 reverse 18", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(18) }, &[0x03, 0x4A, 0x49]),
        ("speed28 short 3 reverse 19", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(19) }, &[0x03, 0x5A, 0x59]),
        ("speed28 short 3 reverse 20", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(20) }, &[0x03, 0x4B, 0x48]),
        ("speed28 short 3 reverse 21", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(21) }, &[0x03, 0x5B, 0x58]),
        ("speed28 short 3 reverse 22", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(22) }, &[0x03, 0x4C, 0x4F]),
        ("speed28 short 3 reverse 23", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(23) }, &[0x03, 0x5C, 0x5F]),
        ("speed28 short 3 reverse 24", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(24) }, &[0x03, 0x4D, 0x4E]),
        ("speed28 short 3 reverse 25", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(25) }, &[0x03, 0x5D, 0x5E]),
        ("speed28 short 3 reverse 26", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(26) }, &[0x03, 0x4E, 0x4D]),
        ("speed28 short 3 reverse 27", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(27) }, &[0x03, 0x5E, 0x5D]),
        ("speed28 short 3 reverse 28", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(28) }, &[0x03, 0x4F, 0x4C]),
        ("speed28 short 3 reverse 29", DccPacket::Speed28 { address: short(3), direction: Direction::Reverse, speed: speed28(29) }, &[0x03, 0x5F, 0x5C]),
        ("speed28 long 1234 forward 0", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(0) }, &[0xC4, 0xD2, 0x60, 0x76]),
        ("speed28 long 1234 forward 1", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(1) }, &[0xC4, 0xD2, 0x61, 0x77]),
        ("speed28 long 1234 forward 2", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(2) }, &[0xC4, 0xD2, 0x62, 0x74]),
        ("speed28 long 1234 forward 3", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(3) }, &[0xC4, 0xD2, 0x72, 0x64]),
        ("speed28 long 1234 forward 4", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(4) }, &[0xC4, 0xD2, 0x63, 0x75]),
        ("speed28 long 1234 forward 5", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(5) }, &[0xC4, 0xD2, 0x73, 0x65]),
        ("speed28 long 1234 forward 6", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(6) }, &[0xC4, 0xD2, 0x64, 0x72]),
        ("speed28 long 1234 forward 7", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(7) }, &[0xC4, 0xD2, 0x74, 0x62]),
        ("speed28 long 1234 forward 8", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(8) }, &[0xC4, 0xD2, 0x65, 0x73]),
        ("speed28 long 1234 forward 9", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(9) }, &[0xC4, 0xD2, 0x75, 0x63]),
        ("speed28 long 1234 forward 10", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(10) }, &[0xC4, 0xD2, 0x66, 0x70]),
        ("speed28 long 1234 forward 11", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(11) }, &[0xC4, 0xD2, 0x76, 0x60]),
        ("speed28 long 1234 forward 12", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(12) }, &[0xC4, 0xD2, 0x67, 0x71]),
        ("speed28 long 1234 forward 13", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(13) }, &[0xC4, 0xD2, 0x77, 0x61]),
        ("speed28 long 1234 forward 14", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(14) }, &[0xC4, 0xD2, 0x68, 0x7E]),
        ("speed28 long 1234 forward 15", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(15) }, &[0xC4, 0xD2, 0x78, 0x6E]),
        ("speed28 long 1234 forward 16", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(16) }, &[0xC4, 0xD2, 0x69, 0x7F]),
        ("speed28 long 1234 forward 17", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(17) }, &[0xC4, 0xD2, 0x79, 0x6F]),
        ("speed28 long 1234 forward 18", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(18) }, &[0xC4, 0xD2, 0x6A, 0x7C]),
        ("speed28 long 1234 forward 19", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(19) }, &[0xC4, 0xD2, 0x7A, 0x6C]),
        ("speed28 long 1234 forward 20", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(20) }, &[0xC4, 0xD2, 0x6B, 0x7D]),
        ("speed28 long 1234 forward 21", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(21) }, &[0xC4, 0xD2, 0x7B, 0x6D]),
        ("speed28 long 1234 forward 22", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(22) }, &[0xC4, 0xD2, 0x6C, 0x7A]),
        ("speed28 long 1234 forward 23", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(23) }, &[0xC4, 0xD2, 0x7C, 0x6A]),
        ("speed28 long 1234 forward 24", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(24) }, &[0xC4, 0xD2, 0x6D, 0x7B]),
        ("speed28 long 1234 forward 25", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(25) }, &[0xC4, 0xD2, 0x7D, 0x6B]),
        ("speed28 long 1234 forward 26", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(26) }, &[0xC4, 0xD2, 0x6E, 0x78]),
        ("speed28 long 1234 forward 27", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(27) }, &[0xC4, 0xD2, 0x7E, 0x68]),
        ("speed28 long 1234 forward 28", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(28) }, &[0xC4, 0xD2, 0x6F, 0x79]),
        ("speed28 long 1234 forward 29", DccPacket::Speed28 { address: long(1234), direction: Direction::Forward, speed: speed28(29) }, &[0xC4, 0xD2, 0x7F, 0x69]),
        ("speed28 long 1234 reverse 0", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(0) }, &[0xC4, 0xD2, 0x40, 0x56]),
        ("speed28 long 1234 reverse 1", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(1) }, &[0xC4, 0xD2, 0x41, 0x57]),
        ("speed28 long 1234 reverse 2", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(2) }, &[0xC4, 0xD2, 0x42, 0x54]),
        ("speed28 long 1234 reverse 3", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(3) }, &[0xC4, 0xD2, 0x52, 0x44]),
        ("speed28 long 1234 reverse 4", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(4) }, &[0xC4, 0xD2, 0x43, 0x55]),
        ("speed28 long 1234 reverse 5", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(5) }, &[0xC4, 0xD2, 0x53, 0x45]),
        ("speed28 long 1234 reverse 6", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(6) }, &[0xC4, 0xD2, 0x44, 0x52]),
        ("speed28 long 1234 reverse 7", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(7) }, &[0xC4, 0xD2, 0x54, 0x42]),
        ("speed28 long 1234 reverse 8", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(8) }, &[0xC4, 0xD2, 0x45, 0x53]),
        ("speed28 long 1234 reverse 9", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(9) }, &[0xC4, 0xD2, 0x55, 0x43]),
        ("speed28 long 1234 reverse 10", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(10) }, &[0xC4, 0xD2, 0x46, 0x50]),
        ("speed28 long 1234 reverse 11", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(11) }, &[0xC4, 0xD2, 0x56, 0x40]),
        ("speed28 long 1234 reverse 12", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(12) }, &[0xC4, 0xD2, 0x47, 0x51]),
        ("speed28 long 1234 reverse 13", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(13) }, &[0xC4, 0xD2, 0x57, 0x41]),
        ("speed28 long 1234 reverse 14", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(14) }, &[0xC4, 0xD2, 0x48, 0x5E]),
        ("speed28 long 1234 reverse 15", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(15) }, &[0xC4, 0xD2, 0x58, 0x4E]),
        ("speed28 long 1234 reverse 16", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(16) }, &[0xC4, 0xD2, 0x49, 0x5F]),
        ("speed28 long 1234 reverse 17", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(17) }, &[0xC4, 0xD2, 0x59, 0x4F]),
        ("speed28 long 1234 reverse 18", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(18) }, &[0xC4, 0xD2, 0x4A, 0x5C]),
        ("speed28 long 1234 reverse 19", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(19) }, &[0xC4, 0xD2, 0x5A, 0x4C]),
        ("speed28 long 1234 reverse 20", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(20) }, &[0xC4, 0xD2, 0x4B, 0x5D]),
        ("speed28 long 1234 reverse 21", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(21) }, &[0xC4, 0xD2, 0x5B, 0x4D]),
        ("speed28 long 1234 reverse 22", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(22) }, &[0xC4, 0xD2, 0x4C, 0x5A]),
        ("speed28 long 1234 reverse 23", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(23) }, &[0xC4, 0xD2, 0x5C, 0x4A]),
        ("speed28 long 1234 reverse 24", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(24) }, &[0xC4, 0xD2, 0x4D, 0x5B]),
        ("speed28 long 1234 reverse 25", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(25) }, &[0xC4, 0xD2, 0x5D, 0x4B]),
        ("speed28 long 1234 reverse 26", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(26) }, &[0xC4, 0xD2, 0x4E, 0x58]),
        ("speed28 long 1234 reverse 27", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(27) }, &[0xC4, 0xD2, 0x5E, 0x48]),
        ("speed28 long 1234 reverse 28", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(28) }, &[0xC4, 0xD2, 0x4F, 0x59]),
        ("speed28 long 1234 reverse 29", DccPacket::Speed28 { address: long(1234), direction: Direction::Reverse, speed: speed28(29) }, &[0xC4, 0xD2, 0x5F, 0x49]),
        ("speed128 short 3 forward 0", DccPacket::Speed128 { address: short(3), direction: Direction::Forward, speed: speed128(0) }, &[0x03, 0x3F, 0x80, 0xBC]),
        ("speed128 short 3 forward 1", DccPacket::Speed128 { address: short(3), direction: Direction::Forward, speed: speed128(1) }, &[0x03, 0x3F, 0x82, 0xBE]),
        ("speed128 short 3 forward 2", DccPacket::Speed128 { address: short(3), direction: Direction::Forward, speed: speed128(2) }, &[0x03, 0x3F, 0x83, 0xBF]),
        ("speed128 short 3 forward 126", DccPacket::Speed128 { address: short(3), direction: Direction::Forward, speed: speed128(126) }, &[0x03, 0x3F, 0xFF, 0xC3]),
        ("speed128 short 3 reverse 0", DccPacket::Speed128 { address: short(3), direction: Direction::Reverse, speed: speed128(0) }, &[0x03, 0x3F, 0x00, 0x3C]),
        ("speed128 short 3 reverse 1", DccPacket::Speed128 { address: short(3), direction: Direction::Reverse, speed: speed128(1) }, &[0x03, 0x3F, 0x02, 0x3E]),
        ("speed128 short 3 reverse 2", DccPacket::Speed128 { address: short(3), direction: Direction::Reverse, speed: speed128(2) }, &[0x03, 0x3F, 0x03, 0x3F]),
        ("speed128 short 3 reverse 126", DccPacket::Speed128 { address: short(3), direction: Direction::Reverse, speed: speed128(126) }, &[0x03, 0x3F, 0x7F, 0x43]),
        ("speed128 long 1234 forward 0", DccPacket::Speed128 { address: long(1234), direction: Direction::Forward, speed: speed128(0) }, &[0xC4, 0xD2, 0x3F, 0x80, 0xA9]),
        ("speed128 long 1234 forward 1", DccPacket::Speed128 { address: long(1234), direction: Direction::Forward, speed: speed128(1) }, &[0xC4, 0xD2, 0x3F, 0x82, 0xAB]),
        ("speed128 long 1234 forward 2", DccPacket::Speed128 { address: long(1234), direction: Direction::Forward, speed: speed128(2) }, &[0xC4, 0xD2, 0x3F, 0x83, 0xAA]),
        ("speed128 long 1234 forward 126", DccPacket::Speed128 { address: long(1234), direction: Direction::Forward, speed: speed128(126) }, &[0xC4, 0xD2, 0x3F, 0xFF, 0xD6]),
        ("speed128 long 1234 reverse 0", DccPacket::Speed128 { address: long(1234), direction: Direction::Reverse, speed: speed128(0) }, &[0xC4, 0xD2, 0x3F, 0x00, 0x29]),
        ("speed128 long 1234 reverse 1", DccPacket::Speed128 { address: long(1234), direction: Direction::Reverse, speed: speed128(1) }, &[0xC4, 0xD2, 0x3F, 0x02, 0x2B]),
        ("speed128 long 1234 reverse 2", DccPacket::Speed128 { address: long(1234), direction: Direction::Reverse, speed: speed128(2) }, &[0xC4, 0xD2, 0x3F, 0x03, 0x2A]),
        ("speed128 long 1234 reverse 126", DccPacket::Speed128 { address: long(1234), direction: Direction::Reverse, speed: speed128(126) }, &[0xC4, 0xD2, 0x3F, 0x7F, 0x56]),
        ("emergency stop short 3 forward", DccPacket::EmergencyStop { address: short(3), direction: Direction::Forward }, &[0x03, 0x61, 0x62]),
        ("emergency stop short 3 reverse", DccPacket::EmergencyStop { address: short(3), direction: Direction::Reverse }, &[0x03, 0x41, 0x42]),
        ("emergency stop long 1234 forward", DccPacket::EmergencyStop { address: long(1234), direction: Direction::Forward }, &[0xC4, 0xD2, 0x61, 0x77]),
        ("emergency stop long 1234 reverse", DccPacket::EmergencyStop { address: long(1234), direction: Direction::Reverse }, &[0xC4, 0xD2, 0x41, 0x57]),
        ("only F0 set short 3", DccPacket::FunctionGroup1 { address: short(3), fl: true, f1: false, f2: false, f3: false, f4: false }, &[0x03, 0x90, 0x93]),
        ("only F1 set short 3", DccPacket::FunctionGroup1 { address: short(3), fl: false, f1: true, f2: false, f3: false, f4: false }, &[0x03, 0x81, 0x82]),
        ("only F2 set short 3", DccPacket::FunctionGroup1 { address: short(3), fl: false, f1: false, f2: true, f3: false, f4: false }, &[0x03, 0x82, 0x81]),
        ("only F3 set short 3", DccPacket::FunctionGroup1 { address: short(3), fl: false, f1: false, f2: false, f3: true, f4: false }, &[0x03, 0x84, 0x87]),
        ("only F4 set short 3", DccPacket::FunctionGroup1 { address: short(3), fl: false, f1: false, f2: false, f3: false, f4: true }, &[0x03, 0x88, 0x8B]),
        ("only F5 set short 3", DccPacket::FunctionGroup2A { address: short(3), f5: true, f6: false, f7: false, f8: false }, &[0x03, 0xB1, 0xB2]),
        ("only F6 set short 3", DccPacket::FunctionGroup2A { address: short(3), f5: false, f6: true, f7: false, f8: false }, &[0x03, 0xB2, 0xB1]),
        ("only F7 set short 3", DccPacket::FunctionGroup2A { address: short(3), f5: false, f6: false, f7: true, f8: false }, &[0x03, 0xB4, 0xB7]),
        ("only F8 set short 3", DccPacket::FunctionGroup2A { address: short(3), f5: false, f6: false, f7: false, f8: true }, &[0x03, 0xB8, 0xBB]),
        ("only F9 set short 3", DccPacket::FunctionGroup2B { address: short(3), f9: true, f10: false, f11: false, f12: false }, &[0x03, 0xA1, 0xA2]),
        ("only F10 set short 3", DccPacket::FunctionGroup2B { address: short(3), f9: false, f10: true, f11: false, f12: false }, &[0x03, 0xA2, 0xA1]),
        ("only F11 set short 3", DccPacket::FunctionGroup2B { address: short(3), f9: false, f10: false, f11: true, f12: false }, &[0x03, 0xA4, 0xA7]),
        ("only F12 set short 3", DccPacket::FunctionGroup2B { address: short(3), f9: false, f10: false, f11: false, f12: true }, &[0x03, 0xA8, 0xAB]),
        ("only F13 set short 3", DccPacket::FunctionGroup3 { address: short(3), f13: true, f14: false, f15: false, f16: false, f17: false, f18: false, f19: false, f20: false }, &[0x03, 0xDE, 0x01, 0xDC]),
        ("only F14 set short 3", DccPacket::FunctionGroup3 { address: short(3), f13: false, f14: true, f15: false, f16: false, f17: false, f18: false, f19: false, f20: false }, &[0x03, 0xDE, 0x02, 0xDF]),
        ("only F15 set short 3", DccPacket::FunctionGroup3 { address: short(3), f13: false, f14: false, f15: true, f16: false, f17: false, f18: false, f19: false, f20: false }, &[0x03, 0xDE, 0x04, 0xD9]),
        ("only F16 set short 3", DccPacket::FunctionGroup3 { address: short(3), f13: false, f14: false, f15: false, f16: true, f17: false, f18: false, f19: false, f20: false }, &[0x03, 0xDE, 0x08, 0xD5]),
        ("only F17 set short 3", DccPacket::FunctionGroup3 { address: short(3), f13: false, f14: false, f15: false, f16: false, f17: true, f18: false, f19: false, f20: false }, &[0x03, 0xDE, 0x10, 0xCD]),
        ("only F18 set short 3", DccPacket::FunctionGroup3 { address: short(3), f13: false, f14: false, f15: false, f16: false, f17: false, f18: true, f19: false, f20: false }, &[0x03, 0xDE, 0x20, 0xFD]),
        ("only F19 set short 3", DccPacket::FunctionGroup3 { address: short(3), f13: false, f14: false, f15: false, f16: false, f17: false, f18: false, f19: true, f20: false }, &[0x03, 0xDE, 0x40, 0x9D]),
        ("only F20 set short 3", DccPacket::FunctionGroup3 { address: short(3), f13: false, f14: false, f15: false, f16: false, f17: false, f18: false, f19: false, f20: true }, &[0x03, 0xDE, 0x80, 0x5D]),
        ("only F21 set short 3", DccPacket::FunctionGroup4 { address: short(3), f21: true, f22: false, f23: false, f24: false, f25: false, f26: false, f27: false, f28: false }, &[0x03, 0xDF, 0x01, 0xDD]),
        ("only F22 set short 3", DccPacket::FunctionGroup4 { address: short(3), f21: false, f22: true, f23: false, f24: false, f25: false, f26: false, f27: false, f28: false }, &[0x03, 0xDF, 0x02, 0xDE]),
        ("only F23 set short 3", DccPacket::FunctionGroup4 { address: short(3), f21: false, f22: false, f23: true, f24: false, f25: false, f26: false, f27: false, f28: false }, &[0x03, 0xDF, 0x04, 0xD8]),
        ("only F24 set short 3", DccPacket::FunctionGroup4 { address: short(3), f21: false, f22: false, f23: false, f24: true, f25: false, f26: false, f27: false, f28: false }, &[0x03, 0xDF, 0x08, 0xD4]),
        ("only F25 set short 3", DccPacket::FunctionGroup4 { address: short(3), f21: false, f22: false, f23: false, f24: false, f25: true, f26: false, f27: false, f28: false }, &[0x03, 0xDF, 0x10, 0xCC]),
        ("only F26 set short 3", DccPacket::FunctionGroup4 { address: short(3), f21: false, f22: false, f23: false, f24: false, f25: false, f26: true, f27: false, f28: false }, &[0x03, 0xDF, 0x20, 0xFC]),
        ("only F27 set short 3", DccPacket::FunctionGroup4 { address: short(3), f21: false, f22: false, f23: false, f24: false, f25: false, f26: false, f27: true, f28: false }, &[0x03, 0xDF, 0x40, 0x9C]),
        ("only F28 set short 3", DccPacket::FunctionGroup4 { address: short(3), f21: false, f22: false, f23: false, f24: false, f25: false, f26: false, f27: false, f28: true }, &[0x03, 0xDF, 0x80, 0x5C]),
        ("FunctionGroup1 all set short 3", DccPacket::FunctionGroup1 { address: short(3), fl: true, f1: true, f2: true, f3: true, f4: true }, &[0x03, 0x9F, 0x9C]),
        ("FunctionGroup2A all set short 3", DccPacket::FunctionGroup2A { address: short(3), f5: true, f6: true, f7: true, f8: true }, &[0x03, 0xBF, 0xBC]),
        ("FunctionGroup2B all set short 3", DccPacket::FunctionGroup2B { address: short(3), f9: true, f10: true, f11: true, f12: true }, &[0x03, 0xAF, 0xAC]),
        ("FunctionGroup3 all set short 3", DccPacket::FunctionGroup3 { address: short(3), f13: true, f14: true, f15: true, f16: true, f17: true, f18: true, f19: true, f20: true }, &[0x03, 0xDE, 0xFF, 0x22]),
        ("FunctionGroup4 all set short 3", DccPacket::FunctionGroup4 { address: short(3), f21: true, f22: true, f23: true, f24: true, f25: true, f26: true, f27: true, f28: true }, &[0x03, 0xDF, 0xFF, 0x23]),
        ("FunctionGroup1 all set long 1234", DccPacket::FunctionGroup1 { address: long(1234), fl: true, f1: true, f2: true, f3: true, f4: true }, &[0xC4, 0xD2, 0x9F, 0x89]),
        ("FunctionGroup2A all set long 1234", DccPacket::FunctionGroup2A { address: long(1234), f5: true, f6: true, f7: true, f8: true }, &[0xC4, 0xD2, 0xBF, 0xA9]),
        ("FunctionGroup2B all set long 1234", DccPacket::FunctionGroup2B { address: long(1234), f9: true, f10: true, f11: true, f12: true }, &[0xC4, 0xD2, 0xAF, 0xB9]),
        ("FunctionGroup3 all set long 1234", DccPacket::FunctionGroup3 { address: long(1234), f13: true, f14: true, f15: true, f16: true, f17: true, f18: true, f19: true, f20: true }, &[0xC4, 0xD2, 0xDE, 0xFF, 0x37]),
        ("FunctionGroup4 all set long 1234", DccPacket::FunctionGroup4 { address: long(1234), f21: true, f22: true, f23: true, f24: true, f25: true, f26: true, f27: true, f28: true }, &[0xC4, 0xD2, 0xDF, 0xFF, 0x36]),
        ("pom write short 3 cv 29 0x22", DccPacket::PomWriteByte { address: short(3), cv: pom_cv(29), value: 0x22 }, &[0x03, 0xEC, 0x1C, 0x22, 0xD1]),
        ("pom write long 1234 cv 1024 0xFF", DccPacket::PomWriteByte { address: long(1234), cv: pom_cv(1024), value: 0xFF }, &[0xC4, 0xD2, 0xEF, 0xFF, 0xFF, 0xF9]),
        ("pom read short 3 cv 1", DccPacket::PomReadByte { address: short(3), cv: pom_cv(1) }, &[0x03, 0xE4, 0x00, 0x00, 0xE7]),
        ("pom read long 1234 cv 513", DccPacket::PomReadByte { address: long(1234), cv: pom_cv(513) }, &[0xC4, 0xD2, 0xE6, 0x00, 0x00, 0xF0]),
        ("service write cv 1 3", DccPacket::ServiceModeWriteByte { cv: service_cv(1), value: 3 }, &[0x7C, 0x00, 0x03, 0x7F]),
        ("service write cv 256 0xAA", DccPacket::ServiceModeWriteByte { cv: service_cv(256), value: 0xAA }, &[0x7C, 0xFF, 0xAA, 0x29]),
        ("service verify cv 1 3", DccPacket::ServiceModeVerifyByte { cv: service_cv(1), value: 3 }, &[0x74, 0x00, 0x03, 0x77]),
        ("service verify cv 256 0x55", DccPacket::ServiceModeVerifyByte { cv: service_cv(256), value: 0x55 }, &[0x74, 0xFF, 0x55, 0xDE]),
        ("logon enable all 0x1234 session 5", DccPacket::LogonEnable { group: logon_group(0), command_station_id: 0x1234, session_id: LogonSessionId::new(5) }, &[0xFE, 0xFC, 0x12, 0x34, 0x05, 0x21]),
        ("logon enable now 0xFFFF session 255", DccPacket::LogonEnable { group: logon_group(3), command_station_id: 0xFFFF, session_id: LogonSessionId::new(255) }, &[0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]),
        ("logon select 0x097 0x12345678 0xFF", DccPacket::LogonSelect { manufacturer_id: 0x097, decoder_id: 0x1234_5678, subcommand: 0xFF }, &[0xFE, 0xD0, 0x97, 0x12, 0x34, 0x56, 0x78, 0xFF, 0xE7, 0xA9]),
        ("logon select 0xFFF 0 0x00", DccPacket::LogonSelect { manufacturer_id: 0xFFF, decoder_id: 0, subcommand: 0x00 }, &[0xFE, 0xDF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0xFF]),
        ("binary state 2 off", DccPacket::BroadcastBinaryStateShort { bin_addr: binary_state(2), state: false }, &[0x00, 0xDD, 0x02, 0xDF]),
        ("binary state 127 on", DccPacket::BroadcastBinaryStateShort { bin_addr: binary_state(127), state: true }, &[0x00, 0xDD, 0xFF, 0x22]),
    ]
}

/// One way of driving a fresh [`SlotManager`] to emit a packet.
#[derive(Debug, Clone, Copy)]
enum Scenario {
    /// Enable each listed function, in order, through `SetFunction` requests.
    /// All functions belong to the same NMRA function group.
    Functions {
        address: DccAddress,
        functions: &'static [u8],
    },
    /// Admit the locomotive through one `SetSpeed` request.
    Speed {
        address: DccAddress,
        speed: LogicalSpeed,
        direction: Direction,
    },
    /// Admit the locomotive moving at 28-step speed 10, let its speed packet
    /// out, then request a per-locomotive emergency stop.
    EmergencyStop {
        address: DccAddress,
        direction: Direction,
    },
    /// Accept a lease, then queue `packet` (a POM read or write addressed to
    /// `address`) through `SchedulerCommand::ProgramOnMain`.
    ProgramOnMain {
        address: DccAddress,
        packet: DccPacket,
    },
}

fn functions(address: DccAddress, functions: &'static [u8]) -> Scenario {
    Scenario::Functions { address, functions }
}

fn set_speed(
    address: DccAddress,
    format: SpeedFormat,
    value: u8,
    direction: Direction,
) -> Scenario {
    let speed = LogicalSpeed::new(value, format).expect("golden logical speed must be valid");
    Scenario::Speed {
        address,
        speed,
        direction,
    }
}

fn emergency_stop(address: DccAddress, direction: Direction) -> Scenario {
    Scenario::EmergencyStop { address, direction }
}

fn pom_read(address: DccAddress, cv: u16) -> Scenario {
    let packet = DccPacket::PomReadByte {
        address,
        cv: pom_cv(cv),
    };
    Scenario::ProgramOnMain { address, packet }
}

fn pom_write(address: DccAddress, cv: u16, value: u8) -> Scenario {
    let packet = DccPacket::PomWriteByte {
        address,
        cv: pom_cv(cv),
        value,
    };
    Scenario::ProgramOnMain { address, packet }
}

/// Builder B: scheduler scenarios and the bytes of the packet they emitted.
#[rustfmt::skip]
fn golden_packets_b() -> Vec<(&'static str, Scenario, &'static [u8])> {
    std::vec![
        ("F0 short 3", functions(short(3), &[0]), &[0x03, 0x90, 0x93]),
        ("F1 short 3", functions(short(3), &[1]), &[0x03, 0x81, 0x82]),
        ("F2 short 3", functions(short(3), &[2]), &[0x03, 0x82, 0x81]),
        ("F3 short 3", functions(short(3), &[3]), &[0x03, 0x84, 0x87]),
        ("F4 short 3", functions(short(3), &[4]), &[0x03, 0x88, 0x8B]),
        ("F5 short 3", functions(short(3), &[5]), &[0x03, 0xB1, 0xB2]),
        ("F6 short 3", functions(short(3), &[6]), &[0x03, 0xB2, 0xB1]),
        ("F7 short 3", functions(short(3), &[7]), &[0x03, 0xB4, 0xB7]),
        ("F8 short 3", functions(short(3), &[8]), &[0x03, 0xB8, 0xBB]),
        ("F9 short 3", functions(short(3), &[9]), &[0x03, 0xA1, 0xA2]),
        ("F10 short 3", functions(short(3), &[10]), &[0x03, 0xA2, 0xA1]),
        ("F11 short 3", functions(short(3), &[11]), &[0x03, 0xA4, 0xA7]),
        ("F12 short 3", functions(short(3), &[12]), &[0x03, 0xA8, 0xAB]),
        ("F13 short 3", functions(short(3), &[13]), &[0x03, 0xDE, 0x01, 0xDC]),
        ("F14 short 3", functions(short(3), &[14]), &[0x03, 0xDE, 0x02, 0xDF]),
        ("F15 short 3", functions(short(3), &[15]), &[0x03, 0xDE, 0x04, 0xD9]),
        ("F16 short 3", functions(short(3), &[16]), &[0x03, 0xDE, 0x08, 0xD5]),
        ("F17 short 3", functions(short(3), &[17]), &[0x03, 0xDE, 0x10, 0xCD]),
        ("F18 short 3", functions(short(3), &[18]), &[0x03, 0xDE, 0x20, 0xFD]),
        ("F19 short 3", functions(short(3), &[19]), &[0x03, 0xDE, 0x40, 0x9D]),
        ("F20 short 3", functions(short(3), &[20]), &[0x03, 0xDE, 0x80, 0x5D]),
        ("F21 short 3", functions(short(3), &[21]), &[0x03, 0xDF, 0x01, 0xDD]),
        ("F22 short 3", functions(short(3), &[22]), &[0x03, 0xDF, 0x02, 0xDE]),
        ("F23 short 3", functions(short(3), &[23]), &[0x03, 0xDF, 0x04, 0xD8]),
        ("F24 short 3", functions(short(3), &[24]), &[0x03, 0xDF, 0x08, 0xD4]),
        ("F25 short 3", functions(short(3), &[25]), &[0x03, 0xDF, 0x10, 0xCC]),
        ("F26 short 3", functions(short(3), &[26]), &[0x03, 0xDF, 0x20, 0xFC]),
        ("F27 short 3", functions(short(3), &[27]), &[0x03, 0xDF, 0x40, 0x9C]),
        ("F28 short 3", functions(short(3), &[28]), &[0x03, 0xDF, 0x80, 0x5C]),
        ("F0 long 1234", functions(long(1234), &[0]), &[0xC4, 0xD2, 0x90, 0x86]),
        ("F28 long 1234", functions(long(1234), &[28]), &[0xC4, 0xD2, 0xDF, 0x80, 0x49]),
        ("F0+F2+F4 short 3", functions(short(3), &[0, 2, 4]), &[0x03, 0x9A, 0x99]),
        ("F5+F8 short 3", functions(short(3), &[5, 8]), &[0x03, 0xB9, 0xBA]),
        ("F13+F20 short 3", functions(short(3), &[13, 20]), &[0x03, 0xDE, 0x81, 0x5C]),
        ("F21..=F28 minus F25 short 3", functions(short(3), &[21, 22, 23, 24, 26, 27, 28]), &[0x03, 0xDF, 0xEF, 0x33]),
        ("set speed28 short 3 forward 0", set_speed(short(3), SpeedFormat::Speed28, 0, Direction::Forward), &[0x03, 0x60, 0x63]),
        ("set speed28 short 3 forward 1", set_speed(short(3), SpeedFormat::Speed28, 1, Direction::Forward), &[0x03, 0x62, 0x61]),
        ("set speed28 short 3 forward 14", set_speed(short(3), SpeedFormat::Speed28, 14, Direction::Forward), &[0x03, 0x78, 0x7B]),
        ("set speed28 short 3 forward 28", set_speed(short(3), SpeedFormat::Speed28, 28, Direction::Forward), &[0x03, 0x7F, 0x7C]),
        ("set speed28 short 3 reverse 14", set_speed(short(3), SpeedFormat::Speed28, 14, Direction::Reverse), &[0x03, 0x58, 0x5B]),
        ("set speed28 long 1234 forward 14", set_speed(long(1234), SpeedFormat::Speed28, 14, Direction::Forward), &[0xC4, 0xD2, 0x78, 0x6E]),
        ("set speed128 short 3 forward 0", set_speed(short(3), SpeedFormat::Speed128, 0, Direction::Forward), &[0x03, 0x3F, 0x80, 0xBC]),
        ("set speed128 short 3 forward 1", set_speed(short(3), SpeedFormat::Speed128, 1, Direction::Forward), &[0x03, 0x3F, 0x82, 0xBE]),
        ("set speed128 short 3 forward 126", set_speed(short(3), SpeedFormat::Speed128, 126, Direction::Forward), &[0x03, 0x3F, 0xFF, 0xC3]),
        ("set speed128 short 3 reverse 64", set_speed(short(3), SpeedFormat::Speed128, 64, Direction::Reverse), &[0x03, 0x3F, 0x41, 0x7D]),
        ("emergency stop short 3 forward", emergency_stop(short(3), Direction::Forward), &[0x03, 0x61, 0x62]),
        ("emergency stop short 3 reverse", emergency_stop(short(3), Direction::Reverse), &[0x03, 0x41, 0x42]),
        ("emergency stop long 1234 forward", emergency_stop(long(1234), Direction::Forward), &[0xC4, 0xD2, 0x61, 0x77]),
        ("emergency stop long 1234 reverse", emergency_stop(long(1234), Direction::Reverse), &[0xC4, 0xD2, 0x41, 0x57]),
        ("pom read short 3 cv 1", pom_read(short(3), 1), &[0x03, 0xE4, 0x00, 0x00, 0xE7]),
        ("pom read long 1234 cv 513", pom_read(long(1234), 513), &[0xC4, 0xD2, 0xE6, 0x00, 0x00, 0xF0]),
        ("pom write short 3 cv 29 0x22", pom_write(short(3), 29, 0x22), &[0x03, 0xEC, 0x1C, 0x22, 0xD1]),
        ("pom write long 1234 cv 1024 0xFF", pom_write(long(1234), 1024, 0xFF), &[0xC4, 0xD2, 0xEF, 0xFF, 0xFF, 0xF9]),
    ]
}

/// NMRA function group (0 = F0..F4, 1 = F5..F8, 2 = F9..F12, 3 = F13..F20,
/// 4 = F21..F28) that carries `function`.
fn nmra_group_of(function: u8) -> u8 {
    match function {
        0..=4 => 0,
        5..=8 => 1,
        9..=12 => 2,
        13..=20 => 3,
        21..=28 => 4,
        _ => panic!("F{function} is outside F0..=F28"),
    }
}

/// Function group encoded by an instruction byte, read from the bytes alone.
fn function_group_of_instruction(instruction: u8) -> Option<u8> {
    match instruction {
        0b1000_0000..=0b1001_1111 => Some(0),
        0b1011_0000..=0b1011_1111 => Some(1),
        0b1010_0000..=0b1010_1111 => Some(2),
        0b1101_1110 => Some(3),
        0b1101_1111 => Some(4),
        _ => None,
    }
}

/// Instruction byte of a multi-function decoder packet: byte 1 after a short
/// address, byte 2 after a long one.
fn instruction_byte(address: DccAddress, bytes: &[u8]) -> u8 {
    let index = if address.is_long() { 2 } else { 1 };
    bytes[index]
}

/// Leading address bytes of a multi-function decoder packet: one byte for a
/// short address, `11AAAAAA AAAAAAAA` for a long one (NMRA S-9.2.1).
fn address_prefix(address: DccAddress) -> Vec<u8> {
    let [high, low] = address.value().to_be_bytes();
    if address.is_long() {
        std::vec![0b1100_0000 | high, low]
    } else {
        std::vec![low]
    }
}

fn is_speed_instruction(instruction: u8) -> bool {
    // 0x3F = advanced operations (128 steps); 01DCSSSS = 28-step speed.
    instruction == 0x3F || instruction & 0b1100_0000 == 0b0100_0000
}

fn is_emergency_stop_instruction(instruction: u8) -> bool {
    // 01DC0001 with C = 0.
    instruction & 0b1101_1111 == 0b0100_0001
}

fn is_pom_instruction(instruction: u8) -> bool {
    // 1110CCVV: long-form CV access, CC = 01 verify (read) or 11 write.
    instruction & 0b1111_0000 == 0b1110_0000
}

fn assert_admitted(result: LocoRequestResult) {
    assert!(
        matches!(
            result,
            LocoRequestResult::Inserted(_) | LocoRequestResult::Updated(_)
        ),
        "request was not admitted: {result:?}"
    );
}

/// Pulls packets until one addressed to `address` has an instruction byte
/// accepted by `wanted`, and returns it.
fn pull_until(
    manager: &mut SlotManager,
    address: DccAddress,
    wanted: impl Fn(u8) -> bool,
) -> DccPacket {
    for _ in 0..MAX_PULLS {
        let packet = manager
            .build_next_packet()
            .expect("an admitted locomotive must keep emitting packets");
        let bytes = packet.to_bytes().expect("emitted packet must encode");
        if bytes.starts_with(&address_prefix(address)) && wanted(instruction_byte(address, &bytes))
        {
            return packet;
        }
    }
    panic!("no matching packet within {MAX_PULLS} pulls");
}

/// Runs `scenario` on a fresh [`SlotManager`] and returns the packet it pins.
fn run_scenario(scenario: Scenario) -> DccPacket {
    let mut manager = SlotManager::new();
    match scenario {
        Scenario::Functions { address, functions } => {
            let group = nmra_group_of(functions[0]);
            for &function in functions {
                assert_eq!(nmra_group_of(function), group, "mixed groups in one entry");
                assert_admitted(manager.handle_loco_request(LocoRequest::SetFunction {
                    address,
                    function: FunctionIndex::new(function).expect("valid function index"),
                    change: FunctionChange::Enable,
                }));
            }
            // A slot admitted through SetFunction starts with no dirty speed
            // (locomotives.rs clears dirty_speed and speed_commanded), so the
            // first packet is expected to be the function packet. The loop
            // filters by instruction byte only as a defence against future
            // scheduler ordering changes.
            let packet = pull_until(&mut manager, address, |instruction| {
                function_group_of_instruction(instruction).is_some()
            });
            let bytes = packet.to_bytes().expect("emitted packet must encode");
            assert_eq!(
                function_group_of_instruction(instruction_byte(address, &bytes)),
                Some(group),
                "emitted function packet is in the wrong group"
            );
            packet
        }
        Scenario::Speed {
            address,
            speed,
            direction,
        } => {
            assert_admitted(manager.handle_loco_request(LocoRequest::SetSpeed {
                address,
                speed,
                direction,
            }));
            pull_until(&mut manager, address, is_speed_instruction)
        }
        Scenario::EmergencyStop { address, direction } => {
            let speed = LogicalSpeed::new(10, SpeedFormat::Speed28).expect("valid speed");
            assert_admitted(manager.handle_loco_request(LocoRequest::SetSpeed {
                address,
                speed,
                direction,
            }));
            pull_until(&mut manager, address, is_speed_instruction);
            assert_admitted(manager.handle_loco_request(LocoRequest::EmergencyStop { address }));
            pull_until(&mut manager, address, is_emergency_stop_instruction)
        }
        Scenario::ProgramOnMain { address, packet } => {
            // Same lease shape as the scheduler tests: epoch 1, never expires.
            let permit = LeasePermit::new(LeaseEpoch::new(1), u64::MAX);
            manager.reset_for_lease(permit);
            assert!(
                manager.apply_command(SchedulerCommand::ProgramOnMain {
                    request_id: PomRequestId::new(1),
                    permit,
                    packet,
                }),
                "POM packet was not queued"
            );
            pull_until(&mut manager, address, is_pom_instruction)
        }
    }
}

#[test]
fn every_golden_packet_encodes_to_its_recorded_bytes() {
    for (label, packet, expected) in golden_packets_a() {
        let bytes = packet.to_bytes().expect("golden packet must encode");
        assert_eq!(bytes.as_slice(), expected, "A: {label}");
    }
    for (label, scenario, expected) in golden_packets_b() {
        let bytes = run_scenario(scenario)
            .to_bytes()
            .expect("emitted packet must encode");
        assert_eq!(bytes.as_slice(), expected, "B: {label}");
    }
}

#[test]
fn every_golden_packet_pulse_stream_decodes_to_its_bytes() {
    let a = golden_packets_a();
    let b = golden_packets_b()
        .into_iter()
        .map(|(label, scenario, expected)| (label, run_scenario(scenario), expected));
    for (label, packet, expected) in a.into_iter().chain(b) {
        let pulses = encode_dcc_packet(&packet).expect("golden packet must encode to pulses");
        assert_eq!(decode_pulses(&pulses).as_slice(), expected, "{label}");
    }
}
