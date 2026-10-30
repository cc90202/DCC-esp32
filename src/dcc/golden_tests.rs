//! Reference encodings of the packets the slot scheduler emits.
//!
//! [`golden_packets_b`] drives a fresh [`SlotManager`] through
//! `handle_loco_request` (or, for POM, `apply_command` with
//! `SchedulerCommand::ProgramOnMain` under an accepted lease) and records the
//! first packet it emits whose instruction byte has the expected kind. It pins
//! the mapping from locomotive state to packets, and it selects packets by
//! their bytes only, never by [`DccPacket`] variant.
//!
//! It is the second of two independent corpora: the first, built from packet
//! literals, lives in the `dcc_protocol` crate, so a change to one builder
//! cannot rewrite the proof.
//!
//! The expected-bytes column is fixed: never edit it to make a test pass.

use crate::authority::{LeaseEpoch, LeasePermit};
use crate::dcc::scheduler::{
    FunctionChange, FunctionIndex, LocoRequest, LocoRequestResult, LogicalSpeed, SchedulerCommand,
    SpeedFormat,
};
use crate::dcc::validator::decode_pulses;
use crate::dcc::{
    DccAddress, DccPacket, Direction, PomCv, PomRequestId, SlotManager, encode_dcc_packet,
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

fn pom_cv(cv: u16) -> PomCv {
    PomCv::new(cv).expect("golden POM CV must be valid")
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
        let bytes = packet.to_bytes();
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
            let bytes = packet.to_bytes();
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
    for (label, scenario, expected) in golden_packets_b() {
        let bytes = run_scenario(scenario).to_bytes();
        assert_eq!(bytes.as_slice(), expected, "{label}");
    }
}

#[test]
fn every_golden_packet_pulse_stream_decodes_to_its_bytes() {
    for (label, scenario, expected) in golden_packets_b() {
        let packet = run_scenario(scenario);
        let pulses = encode_dcc_packet(&packet).expect("golden packet must encode to pulses");
        assert_eq!(decode_pulses(&pulses).as_slice(), expected, "{label}");
    }
}
