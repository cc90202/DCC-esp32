//! Reference encodings of every DCC packet kind, built from packet literals.
//!
//! Pins the on-wire bytes of each packet kind, so that changes to the packet
//! encoder, the function-group representation and the validator can prove
//! the wire bytes did not change.
//!
//! [`golden_packets_a`] builds [`DccPacket`] values directly and pins the
//! byte layout of each variant. The firmware keeps a second, independent
//! corpus that drives its slot scheduler and pins the packets it emits, so a
//! change to one builder cannot rewrite the proof.
//!
//! The expected-bytes column is fixed: never edit it to make a test pass;
//! only the builder column may change.

use std::vec::Vec;

use crate::function_groups::{group_bits, group_of};
use crate::validator::decode_pulses;
use crate::{
    BinaryStateAddress, DccAddress, DccPacket, Direction, FunctionGroupKind, FunctionIndex,
    FunctionState, LogonGroup, LogonSessionId, NmraSpeed28, NmraSpeed128, PomCv, ServiceModeCv,
    encode_dcc_packet,
};

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
        ("only F0 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G1, 0b1_0000), &[0x03, 0x90, 0x93]),
        ("only F1 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G1, 0b0_0001), &[0x03, 0x81, 0x82]),
        ("only F2 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G1, 0b0_0010), &[0x03, 0x82, 0x81]),
        ("only F3 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G1, 0b0_0100), &[0x03, 0x84, 0x87]),
        ("only F4 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G1, 0b0_1000), &[0x03, 0x88, 0x8B]),
        ("only F5 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G2a, 0b0001), &[0x03, 0xB1, 0xB2]),
        ("only F6 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G2a, 0b0010), &[0x03, 0xB2, 0xB1]),
        ("only F7 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G2a, 0b0100), &[0x03, 0xB4, 0xB7]),
        ("only F8 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G2a, 0b1000), &[0x03, 0xB8, 0xBB]),
        ("only F9 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G2b, 0b0001), &[0x03, 0xA1, 0xA2]),
        ("only F10 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G2b, 0b0010), &[0x03, 0xA2, 0xA1]),
        ("only F11 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G2b, 0b0100), &[0x03, 0xA4, 0xA7]),
        ("only F12 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G2b, 0b1000), &[0x03, 0xA8, 0xAB]),
        ("only F13 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G3, 0b0000_0001), &[0x03, 0xDE, 0x01, 0xDC]),
        ("only F14 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G3, 0b0000_0010), &[0x03, 0xDE, 0x02, 0xDF]),
        ("only F15 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G3, 0b0000_0100), &[0x03, 0xDE, 0x04, 0xD9]),
        ("only F16 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G3, 0b0000_1000), &[0x03, 0xDE, 0x08, 0xD5]),
        ("only F17 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G3, 0b0001_0000), &[0x03, 0xDE, 0x10, 0xCD]),
        ("only F18 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G3, 0b0010_0000), &[0x03, 0xDE, 0x20, 0xFD]),
        ("only F19 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G3, 0b0100_0000), &[0x03, 0xDE, 0x40, 0x9D]),
        ("only F20 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G3, 0b1000_0000), &[0x03, 0xDE, 0x80, 0x5D]),
        ("only F21 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G4, 0b0000_0001), &[0x03, 0xDF, 0x01, 0xDD]),
        ("only F22 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G4, 0b0000_0010), &[0x03, 0xDF, 0x02, 0xDE]),
        ("only F23 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G4, 0b0000_0100), &[0x03, 0xDF, 0x04, 0xD8]),
        ("only F24 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G4, 0b0000_1000), &[0x03, 0xDF, 0x08, 0xD4]),
        ("only F25 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G4, 0b0001_0000), &[0x03, 0xDF, 0x10, 0xCC]),
        ("only F26 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G4, 0b0010_0000), &[0x03, 0xDF, 0x20, 0xFC]),
        ("only F27 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G4, 0b0100_0000), &[0x03, 0xDF, 0x40, 0x9C]),
        ("only F28 set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G4, 0b1000_0000), &[0x03, 0xDF, 0x80, 0x5C]),
        ("G1 all set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G1, 0b1_1111), &[0x03, 0x9F, 0x9C]),
        ("G2a all set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G2a, 0b1111), &[0x03, 0xBF, 0xBC]),
        ("G2b all set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G2b, 0b1111), &[0x03, 0xAF, 0xAC]),
        ("G3 all set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G3, 0b1111_1111), &[0x03, 0xDE, 0xFF, 0x22]),
        ("G4 all set short 3", DccPacket::function_group(short(3), FunctionGroupKind::G4, 0b1111_1111), &[0x03, 0xDF, 0xFF, 0x23]),
        ("G1 all set long 1234", DccPacket::function_group(long(1234), FunctionGroupKind::G1, 0b1_1111), &[0xC4, 0xD2, 0x9F, 0x89]),
        ("G2a all set long 1234", DccPacket::function_group(long(1234), FunctionGroupKind::G2a, 0b1111), &[0xC4, 0xD2, 0xBF, 0xA9]),
        ("G2b all set long 1234", DccPacket::function_group(long(1234), FunctionGroupKind::G2b, 0b1111), &[0xC4, 0xD2, 0xAF, 0xB9]),
        ("G3 all set long 1234", DccPacket::function_group(long(1234), FunctionGroupKind::G3, 0b1111_1111), &[0xC4, 0xD2, 0xDE, 0xFF, 0x37]),
        ("G4 all set long 1234", DccPacket::function_group(long(1234), FunctionGroupKind::G4, 0b1111_1111), &[0xC4, 0xD2, 0xDF, 0xFF, 0x36]),
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

#[test]
fn every_golden_packet_encodes_to_its_recorded_bytes() {
    for (label, packet, expected) in golden_packets_a() {
        let bytes = packet.to_bytes();
        assert_eq!(bytes.as_slice(), expected, "{label}");
    }
}

#[test]
fn every_single_function_encodes_to_its_golden_bytes() {
    let corpus = golden_packets_a();
    let address = short(3);
    for function in 0..=28u8 {
        let label = std::format!("only F{function} set short 3");
        let expected = corpus
            .iter()
            .find(|(entry, _, _)| *entry == label)
            .map_or_else(
                || panic!("no golden entry {label:?}"),
                |(_, _, bytes)| *bytes,
            );
        let index = FunctionIndex::new(function).expect("F0..=F28 are valid");
        let kind = group_of(index);
        let functions = FunctionState::from_bits_truncate(1 << function);
        let packet = DccPacket::function_group(address, kind, group_bits(functions, kind));
        assert_eq!(packet.to_bytes().as_slice(), expected, "F{function}");
    }
}

#[test]
fn every_golden_packet_pulse_stream_decodes_to_its_bytes() {
    for (label, packet, expected) in golden_packets_a() {
        let pulses = encode_dcc_packet(&packet).expect("golden packet must encode to pulses");
        assert_eq!(decode_pulses(&pulses).as_slice(), expected, "{label}");
    }
}
