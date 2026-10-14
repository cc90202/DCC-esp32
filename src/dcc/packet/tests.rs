//! Encoding and validation tests for DCC packets and addresses.

use super::*;

#[test]
fn test_high_fragment_reads_each_form_of_the_byte() {
    assert_eq!(HighFragment::read(0x00), HighFragment::Plain(0x00));
    assert_eq!(HighFragment::read(0x03), HighFragment::Plain(0x03));
    assert_eq!(HighFragment::read(0xc3), HighFragment::Long(0x03));
    assert_eq!(HighFragment::read(0x40), HighFragment::Malformed);
    assert_eq!(HighFragment::read(0x80), HighFragment::Malformed);
    assert_eq!(HighFragment::Malformed.address_value(0x12), None);
    assert_eq!(HighFragment::Long(0x03).address_value(0xe8), Some(1000));
    assert_eq!(HighFragment::Plain(0x03).address_value(0xe8), Some(1000));
}

#[test]
fn test_from_address_fragments_reads_every_form_of_high_byte() {
    // A stated long address is taken at its word, a plain fragment leaves
    // magnitude to decide, and half a marker names nothing. The refusal at
    // the end is the one the wire allows and this type cannot hold.
    let cases = [
        (0x00, 0x03, DccAddress::new_short(3)),
        (0x00, 0xff, DccAddress::new_long(255)),
        (0x03, 0xe8, DccAddress::new_long(1000)),
        (0xc3, 0xe8, DccAddress::new_long(1000)),
        (0xc0, 0x80, DccAddress::new_long(128)),
        (0xc0, 0x03, None),
        (0x40, 0x03, None),
        (0x80, 0x03, None),
        (0x00, 0x00, None),
        (0xff, 0xff, None),
    ];

    for (high, low, expected) in cases {
        assert_eq!(
            DccAddress::from_address_fragments(high, low),
            expected,
            "fragments {high:#04x} {low:#04x}"
        );
    }
}

fn variant_name(packet: DccPacket) -> &'static str {
    match packet {
        DccPacket::Idle => "Idle",
        DccPacket::Reset => "Reset",
        DccPacket::Speed28 { .. } => "Speed28",
        DccPacket::Speed128 { .. } => "Speed128",
        DccPacket::FunctionGroup { .. } => "FunctionGroup",
        DccPacket::EmergencyStop { .. } => "EmergencyStop",
        DccPacket::BroadcastStop => "BroadcastStop",
        DccPacket::BroadcastBinaryStateShort { .. } => "BroadcastBinaryStateShort",
        DccPacket::LogonEnable { .. } => "LogonEnable",
        DccPacket::LogonSelect { .. } => "LogonSelect",
        DccPacket::ServiceModeVerifyByte { .. } => "ServiceModeVerifyByte",
        DccPacket::ServiceModeWriteByte { .. } => "ServiceModeWriteByte",
        DccPacket::PomWriteByte { .. } => "PomWriteByte",
        DccPacket::PomReadByte { .. } => "PomReadByte",
    }
}

fn service_cv(cv: u16) -> ServiceModeCv {
    ServiceModeCv::new(cv).expect("test service-mode CV must be valid")
}

fn pom_cv(cv: u16) -> PomCv {
    PomCv::new(cv).expect("test POM CV must be valid")
}

fn binary_state_addr(address: u8) -> BinaryStateAddress {
    BinaryStateAddress::new(address).expect("test binary-state address must be valid")
}

fn logon_group(group: u8) -> LogonGroup {
    LogonGroup::new(group).expect("test logon group must be valid")
}

#[test]
fn every_packet_variant_fits_the_declared_byte_capacity() {
    let long_address = DccAddress::new_long(10_239).unwrap();
    let all_variants = [
        DccPacket::Idle,
        DccPacket::Reset,
        DccPacket::Speed28 {
            address: long_address,
            direction: Direction::Forward,
            speed: NmraSpeed28::new(29).unwrap(),
        },
        DccPacket::Speed128 {
            address: long_address,
            direction: Direction::Reverse,
            speed: NmraSpeed128::new(126).unwrap(),
        },
        // G4 is the longest function group form (instruction + data byte).
        DccPacket::function_group(long_address, FunctionGroupKind::G4, u8::MAX),
        DccPacket::EmergencyStop {
            address: long_address,
            direction: Direction::Forward,
        },
        DccPacket::BroadcastStop,
        DccPacket::BroadcastBinaryStateShort {
            bin_addr: binary_state_addr(127),
            state: true,
        },
        DccPacket::LogonEnable {
            group: logon_group(3),
            command_station_id: u16::MAX,
            session_id: LogonSessionId::new(u8::MAX),
        },
        DccPacket::LogonSelect {
            manufacturer_id: u16::MAX,
            decoder_id: u32::MAX,
            subcommand: u8::MAX,
        },
        DccPacket::ServiceModeVerifyByte {
            cv: service_cv(256),
            value: u8::MAX,
        },
        DccPacket::ServiceModeWriteByte {
            cv: service_cv(256),
            value: u8::MAX,
        },
        DccPacket::PomWriteByte {
            address: long_address,
            cv: pom_cv(1024),
            value: u8::MAX,
        },
        DccPacket::PomReadByte {
            address: long_address,
            cv: pom_cv(1024),
        },
    ];

    for packet in all_variants {
        let encoded = packet.to_bytes();
        assert!(
            encoded.len() <= DCC_MAX_PACKET_BYTES,
            "{} encoded to {} bytes, exceeding DCC_MAX_PACKET_BYTES",
            variant_name(packet),
            encoded.len()
        );
    }

    let longest = DccPacket::LogonSelect {
        manufacturer_id: u16::MAX,
        decoder_id: u32::MAX,
        subcommand: u8::MAX,
    };
    assert_eq!(
        longest.to_bytes().len(),
        DCC_MAX_PACKET_BYTES,
        "the declared capacity should remain tight"
    );
}

#[test]
fn test_from_magnitude_picks_short_or_long_by_range() {
    assert_eq!(DccAddress::from_magnitude(0), None);
    assert_eq!(DccAddress::from_magnitude(3), DccAddress::new_short(3));
    assert_eq!(DccAddress::from_magnitude(127), DccAddress::new_short(127));
    assert_eq!(DccAddress::from_magnitude(128), DccAddress::new_long(128));
    assert_eq!(
        DccAddress::from_magnitude(10_239),
        DccAddress::new_long(10_239)
    );
    assert_eq!(DccAddress::from_magnitude(10_240), None);
}

#[test]
fn test_packed_dcc_address_roundtrip() {
    let short = DccAddress::new_short(3).unwrap();
    let long = DccAddress::new_long(1000).unwrap();

    assert_eq!(PackedDccAddress::from_address(None).raw(), 0);
    assert_eq!(
        PackedDccAddress::from_address(Some(short)).address(),
        Some(short)
    );
    assert_eq!(
        PackedDccAddress::from_address(Some(long)).address(),
        Some(long)
    );
    assert_eq!(PackedDccAddress::from_raw(0).address(), None);
}

#[test]
fn test_packed_dcc_address_rejects_invalid_raw_values() {
    assert_eq!(PackedDccAddress::from_raw(128).address(), None);
    assert_eq!(PackedDccAddress::from_raw(0x8000 | 127).address(), None);
    assert_eq!(PackedDccAddress::from_raw(0x8000 | 10_240).address(), None);
}

#[test]
fn test_packed_address_flags_roundtrip() {
    let short = DccAddress::new_short(3).unwrap();

    let packed = PackedAddressFlags::new(Some(short))
        .with_flag(0, true)
        .with_flag(0, true)
        .with_flag(1, false);

    assert_eq!(packed.address(), Some(short));
    assert!(packed.flag(0));
    assert!(!packed.flag(1));
    assert_eq!(PackedAddressFlags::from_raw(packed.raw()), packed);
}

#[test]
fn test_packed_address_flags_no_flags_set_by_default() {
    let packed = PackedAddressFlags::new(None);

    assert_eq!(packed.address(), None);
    assert!(!packed.flag(0));
    assert!(!packed.flag(1));
}

#[test]
fn test_packed_address_flags_field_replaces_previous_value() {
    let short = DccAddress::new_short(3).unwrap();
    let packed = PackedAddressFlags::new(Some(short))
        .with_field(2, 2, 3)
        .with_field(2, 2, 1);

    assert_eq!(packed.field(2, 2), 1);
    assert_eq!(packed.address(), Some(short));
    assert_eq!(
        [0, 1, 2, 3].map(|index| packed.flag(index)),
        [false, false, true, false]
    );
}

#[test]
fn test_short_address_validation() {
    assert_eq!(DccAddress::new_short(0), None);
    assert_eq!(DccAddress::new_short(128), None);
    assert!(DccAddress::new_short(3).is_some());
    assert!(DccAddress::new_short(127).is_some());
    let addr = DccAddress::new_short(3).unwrap();
    assert!(addr.is_short());
    assert!(!addr.is_long());
    assert_eq!(addr.value(), 3);
}

#[test]
fn test_long_address_validation() {
    assert_eq!(DccAddress::new_long(127), None);
    assert_eq!(DccAddress::new_long(10240), None);
    assert!(DccAddress::new_long(128).is_some());
    assert!(DccAddress::new_long(10239).is_some());
    let addr = DccAddress::new_long(1000).unwrap();
    assert!(addr.is_long());
    assert!(!addr.is_short());
    assert_eq!(addr.value(), 1000);
}

#[test]
fn test_idle_packet_encoding() {
    let packet = DccPacket::Idle;
    let bytes = packet.to_bytes();
    // Idle packet: [0xFF, 0x00, checksum]
    // checksum = 0xFF XOR 0x00 = 0xFF
    assert_eq!(bytes.as_slice(), &[0xFF, 0x00, 0xFF]);
}

#[test]
fn test_reset_packet_encoding() {
    let packet = DccPacket::Reset;
    let bytes = packet.to_bytes();
    assert_eq!(bytes.as_slice(), &[0x00, 0x00, 0x00]);
}

#[test]
fn test_speed28_packet_encoding() {
    let addr = DccAddress::new_short(3).unwrap();
    let packet = DccPacket::speed_28step(addr, 14, Direction::Forward).unwrap();
    let bytes = packet.to_bytes();

    // address = 3
    // instruction = 01DCSSSS per NMRA S-9.2
    // speed 14: step=13, SSSS=(13+3)/2=8, C=(13-1)&1=0, CSSSS=0b0_1000
    // instruction = 0b0100_0000 | 0b0010_0000 (fwd) | 0b0000_1000 = 0b0110_1000 = 0x68
    // checksum = 3 ^ 0x68 = 0x6B
    assert_eq!(bytes.len(), 3);
    assert_eq!(bytes[0], 3);
    assert_eq!(bytes[1], 0x68);
    assert_eq!(bytes[2], 0x6B);
}

#[test]
fn test_speed128_packet_encoding() {
    let packet =
        DccPacket::speed_128step(DccAddress::new_long(1000).unwrap(), 64, Direction::Reverse)
            .unwrap();
    let bytes = packet.to_bytes();

    // Expected: [addr_high, addr_low, 0x3F, speed_byte, checksum]
    // Long address 1000 = 0x03E8
    // high = 0xC0 | (0x03E8 >> 8) = 0xC0 | 0x03 = 0xC3
    // low = 0xE8
    // logical speed 64 -> wire 65 (reverse, so bit 7 = 0) = 0x41
    // checksum = 0xC3 ^ 0xE8 ^ 0x3F ^ 0x41
    let expected_checksum = 0xC3 ^ 0xE8 ^ 0x3F ^ 0x41;

    assert_eq!(bytes.len(), 5);
    assert_eq!(bytes[0], 0xC3);
    assert_eq!(bytes[1], 0xE8);
    assert_eq!(bytes[2], 0x3F);
    assert_eq!(bytes[3], 0x41);
    assert_eq!(bytes[4], expected_checksum);
}

#[test]
fn function_group_bits_never_reach_the_opcode() {
    let addr = DccAddress::new_short(3).unwrap();
    // The constructor drops bits outside the group's data field...
    assert_eq!(
        DccPacket::function_group(addr, FunctionGroupKind::G2a, 0xFF),
        DccPacket::function_group(addr, FunctionGroupKind::G2a, 0x0F)
    );
    // ...and so does the encoder for a packet built without it.
    let raw = DccPacket::FunctionGroup {
        address: addr,
        kind: FunctionGroupKind::G1,
        bits: 0xFF,
    };
    assert_eq!(raw.to_bytes().as_slice(), &[0x03, 0x9F, 0x9C]);
}

#[test]
fn test_emergency_stop_encoding() {
    let addr = DccAddress::new_short(3).unwrap();
    let packet = DccPacket::EmergencyStop {
        address: addr,
        direction: Direction::Forward,
    };
    let bytes = packet.to_bytes();
    // instruction = 01DC0001 (C=0): 0b0100_0001 | 0b0010_0000 (fwd) = 0b0110_0001 = 0x61
    assert_eq!(bytes.len(), 3);
    assert_eq!(bytes[0], 3);
    assert_eq!(bytes[1], 0b0110_0001);
    assert_eq!(bytes[2], 3 ^ 0b0110_0001);
}

#[test]
fn test_broadcast_stop_encoding() {
    let packet = DccPacket::BroadcastStop;
    let bytes = packet.to_bytes();
    // [0x00, 0b0110_0001, checksum]
    // 0b0110_0001 = 01DC0001, D=1 (fwd), C=0, SSSS=0001 (e-stop)
    // checksum = 0x00 ^ 0b0110_0001 = 0b0110_0001
    assert_eq!(bytes.len(), 3);
    assert_eq!(bytes[0], 0x00);
    assert_eq!(bytes[1], 0b0110_0001);
    assert_eq!(bytes[2], 0b0110_0001);
}

#[test]
fn test_broadcast_binary_state_short_encoding() {
    let packet = DccPacket::BroadcastBinaryStateShort {
        bin_addr: binary_state_addr(2),
        state: false,
    };
    let bytes = packet.to_bytes();

    assert_eq!(bytes.as_slice(), &[0x00, 0xDD, 0x02, 0xDF]);
}

#[test]
fn test_binary_state_address_validation() {
    assert!(BinaryStateAddress::new(0).is_some());
    assert!(BinaryStateAddress::new(127).is_some());
    assert!(BinaryStateAddress::new(128).is_none());
}

#[test]
fn test_logon_enable_now_encoding() {
    let packet = DccPacket::LogonEnable {
        group: logon_group(3),
        command_station_id: 0x0DCC,
        session_id: LogonSessionId::new(0x42),
    };
    let bytes = packet.to_bytes();

    assert_eq!(bytes.as_slice(), &[254, 0xFF, 0x0D, 0xCC, 0x42, 0x82]);
}

#[test]
fn test_logon_group_validation() {
    assert!(LogonGroup::new(0).is_some());
    assert!(LogonGroup::new(3).is_some());
    assert!(LogonGroup::new(4).is_none());
}

#[test]
fn test_logon_select_encoding_uses_dallas_maxim_crc() {
    let packet = DccPacket::LogonSelect {
        manufacturer_id: 0x000D,
        decoder_id: 0x1234_5678,
        subcommand: 0xFF,
    };
    let bytes = packet.to_bytes();

    assert_eq!(
        bytes.as_slice(),
        &[0xFE, 0xD0, 0x0D, 0x12, 0x34, 0x56, 0x78, 0xFF, 0x99, 0x4D]
    );
}

#[test]
fn test_dallas_maxim_crc_matches_zimo_reference_vector() {
    let data = [
        0x0B, 0x0A, 0x00, 0x00, 0x8E, 0x40, 0x00, 0x0D, 0x67, 0x00, 0x01, 0x00,
    ];

    assert_eq!(crate::logon::crc8_dallas_maxim(&data), 0x4C);
}

#[test]
fn test_broadcast_stop_differs_from_reset() {
    let stop = DccPacket::BroadcastStop.to_bytes();
    let reset = DccPacket::Reset.to_bytes();
    // BroadcastStop has instruction byte 0x61, Reset has 0x00
    assert_ne!(stop.as_slice(), reset.as_slice());
}

#[test]
fn test_speed28_constructor_validates_range() {
    let addr = DccAddress::new_short(3).unwrap();
    assert!(DccPacket::speed_28step(addr, 0, Direction::Forward).is_some());
    assert!(DccPacket::speed_28step(addr, 29, Direction::Forward).is_some());
    assert!(DccPacket::speed_28step(addr, 30, Direction::Forward).is_none());
}

/// Verify Speed28 interleaved encoding against NMRA S-9.2 §2.3.2.3.
///
/// Instruction byte: 01DCSSSS
/// - D = direction (1=forward, 0=reverse)
/// - For speed 0: CSSSS=00000 (stop)
/// - For speed 1: CSSSS=00001 (e-stop)
/// - For speed 2-29: step=speed-1, SSSS=(step+3)/2, C=(step-1)&1
#[test]
fn test_speed28_nmra_table() {
    let addr = DccAddress::new_short(1).unwrap();

    // Verify stop (speed=0): CSSSS = 00000
    let pkt = DccPacket::speed_28step(addr, 0, Direction::Forward).unwrap();
    let bytes = pkt.to_bytes();
    assert_eq!(bytes[1] & 0b0001_1111, 0b0_0000, "speed=0 should be stop");

    // Verify e-stop (speed=1): CSSSS = 00001
    let pkt = DccPacket::speed_28step(addr, 1, Direction::Forward).unwrap();
    let bytes = pkt.to_bytes();
    assert_eq!(bytes[1] & 0b0001_1111, 0b0_0001, "speed=1 should be e-stop");

    // Verify direction bit: forward=1, reverse=0
    let fwd = DccPacket::speed_28step(addr, 10, Direction::Forward).unwrap();
    let rev = DccPacket::speed_28step(addr, 10, Direction::Reverse).unwrap();
    assert_eq!(fwd.to_bytes()[1] & 0b0010_0000, 0b0010_0000, "forward D=1");
    assert_eq!(rev.to_bytes()[1] & 0b0010_0000, 0b0000_0000, "reverse D=0");

    // Verify NMRA S-9.2 interleaved encoding for speed values:
    // step=speed-1, SSSS=(step+3)/2, C=(step-1)&1
    let test_cases: &[(u8, u8)] = &[
        (2, 0b0_0010),  // step=1:  SSSS=2,  C=0
        (3, 0b1_0010),  // step=2:  SSSS=2,  C=1
        (4, 0b0_0011),  // step=3:  SSSS=3,  C=0
        (5, 0b1_0011),  // step=4:  SSSS=3,  C=1
        (14, 0b0_1000), // step=13: SSSS=8,  C=0
        (28, 0b0_1111), // step=27: SSSS=15, C=0
        (29, 0b1_1111), // step=28: SSSS=15, C=1
    ];

    for &(speed, expected_cssss) in test_cases {
        let pkt = DccPacket::speed_28step(addr, speed, Direction::Forward).unwrap();
        let bytes = pkt.to_bytes();
        assert_eq!(
            bytes[1] & 0b0001_1111,
            expected_cssss,
            "speed={speed}: instruction byte CSSSS={:#07b}, expected {expected_cssss:#07b}",
            bytes[1] & 0b0001_1111
        );
    }
}

#[test]
fn test_speed128_constructor_validates_range() {
    let addr = DccAddress::new_short(3).unwrap();
    assert!(DccPacket::speed_128step(addr, 0, Direction::Forward).is_some());
    assert!(DccPacket::speed_128step(addr, 126, Direction::Forward).is_some());
    assert!(DccPacket::speed_128step(addr, 127, Direction::Forward).is_none());
}

#[test]
fn test_service_mode_verify_byte_encoding() {
    // Test CV1 verify value 3 (common decoder address verification)
    let packet = DccPacket::ServiceModeVerifyByte {
        cv: service_cv(1),
        value: 3,
    };
    let bytes = packet.to_bytes();

    // Expected: [0b0111_0100, 0x00, 0x03, checksum]
    // CV high byte: 0b0111_0100 (0111 01 00 - Verify instruction, CV bits 9-8 = 00)
    // CV low byte: 0x00 (wire address = CV-1)
    // Value: 0x03
    // Checksum: 0b0111_0100 ^ 0x00 ^ 0x03
    assert_eq!(bytes.len(), 4);
    assert_eq!(bytes[0], 0b0111_0100);
    assert_eq!(bytes[1], 0x00);
    assert_eq!(bytes[2], 0x03);
    assert_eq!(bytes[3], 0b0111_0100 ^ 0x03);
}

#[test]
fn test_service_mode_write_byte_encoding() {
    // Test CV29 write value 6 (common configuration CV)
    let packet = DccPacket::ServiceModeWriteByte {
        cv: service_cv(29),
        value: 6,
    };
    let bytes = packet.to_bytes();

    // Expected: [0b0111_1100, 0x1C, 0x06, checksum]
    // CV high byte: 0b0111_1100 (0111 11 00 - Write instruction, CV bits 9-8 = 00)
    // CV low byte: 0x1C (wire address = CV-1)
    // Value: 0x06
    // Checksum: 0b0111_1100 ^ 0x1C ^ 0x06
    assert_eq!(bytes.len(), 4);
    assert_eq!(bytes[0], 0b0111_1100);
    assert_eq!(bytes[1], 28);
    assert_eq!(bytes[2], 6);
    assert_eq!(bytes[3], 0b0111_1100 ^ 28 ^ 6);
}

#[test]
fn test_service_mode_verify_vs_write_instruction_bits() {
    let verify = DccPacket::ServiceModeVerifyByte {
        cv: service_cv(1),
        value: 0,
    };
    let write = DccPacket::ServiceModeWriteByte {
        cv: service_cv(1),
        value: 0,
    };

    let verify_bytes = verify.to_bytes();
    let write_bytes = write.to_bytes();

    // Instruction bytes differ only in bits 3-2: Verify=01, Write=11
    // Verify: 0b0111_0100 (0x74)
    // Write:  0b0111_1100 (0x7C)
    assert_eq!(verify_bytes[0], 0b0111_0100);
    assert_eq!(write_bytes[0], 0b0111_1100);
    assert_eq!(verify_bytes[0] & 0b1111_0011, write_bytes[0] & 0b1111_0011); // Other bits match
}

#[test]
fn test_service_mode_cv_address_range() {
    // Test boundary cases for CV 1-256
    let cv1 = DccPacket::ServiceModeVerifyByte {
        cv: service_cv(1),
        value: 0,
    };
    let cv256 = DccPacket::ServiceModeVerifyByte {
        cv: service_cv(256),
        value: 0,
    };

    let bytes1 = cv1.to_bytes();
    let bytes256 = cv256.to_bytes();

    // CV1: high byte = 0b0111_0100, low byte = 0x00
    assert_eq!(bytes1[0], 0b0111_0100);
    assert_eq!(bytes1[1], 0);

    // CV256: high byte = 0b0111_0100, low byte = 0xFF
    assert_eq!(bytes256[0], 0b0111_0100);
    assert_eq!(bytes256[1], 0xFF);
}

#[test]
fn test_pom_write_byte_encoding_short_address() {
    let packet = DccPacket::PomWriteByte {
        address: DccAddress::new_short(3).unwrap(),
        cv: pom_cv(29),
        value: 6,
    };
    let bytes = packet.to_bytes();

    assert_eq!(bytes, [0x03, 0xEC, 0x1C, 0x06, 0xF5]);
}

#[test]
fn test_pom_read_byte_encoding_long_address() {
    let packet = DccPacket::PomReadByte {
        address: DccAddress::new_long(1000).unwrap(),
        cv: pom_cv(1),
    };
    let bytes = packet.to_bytes();

    assert_eq!(bytes, [0xC3, 0xE8, 0xE4, 0x00, 0x00, 0xCF]);
}

#[test]
fn test_pom_cv_address_range() {
    let cv1 = DccPacket::PomReadByte {
        address: DccAddress::new_short(3).unwrap(),
        cv: pom_cv(1),
    };
    let cv1024 = DccPacket::PomReadByte {
        address: DccAddress::new_short(3).unwrap(),
        cv: pom_cv(1024),
    };

    let bytes1 = cv1.to_bytes();
    let bytes1024 = cv1024.to_bytes();

    assert_eq!(bytes1[1], 0b1110_0100);
    assert_eq!(bytes1[2], 0x00);
    assert_eq!(bytes1024[1], 0b1110_0111);
    assert_eq!(bytes1024[2], 0xFF);
}

#[test]
fn test_pom_cv_out_of_range_rejected() {
    assert!(PomCv::new(0).is_none());
    assert!(PomCv::new(1025).is_none());
}

#[test]
fn test_service_mode_cv_out_of_range_rejected() {
    assert!(ServiceModeCv::new(0).is_none());
    assert!(ServiceModeCv::new(257).is_none());
}

#[test]
fn test_service_mode_checksum_calculation() {
    // Verify checksum is XOR of all data bytes
    let packet = DccPacket::ServiceModeWriteByte {
        cv: service_cv(17),
        value: 192,
    };
    let bytes = packet.to_bytes();

    let expected_checksum = bytes[0] ^ bytes[1] ^ bytes[2];
    assert_eq!(bytes[3], expected_checksum);
}
