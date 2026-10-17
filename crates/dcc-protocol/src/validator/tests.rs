use super::*;
use crate::encoder::dcc_bit_to_pulse;
use crate::function_groups::GROUP_LAYOUTS;
use crate::{DccAddress, Direction, PomCv, ServiceModeCv, encode_dcc_packet};

fn service_cv(cv: u16) -> ServiceModeCv {
    ServiceModeCv::new(cv).expect("test service-mode CV must be valid")
}

fn pom_cv(cv: u16) -> PomCv {
    PomCv::new(cv).expect("test POM CV must be valid")
}

fn assert_complete(packet: DccPacket) {
    assert_eq!(validate_complete(packet), Ok(()), "{packet:?}");
}

#[test]
fn complete_validation_accepts_common_packet_kinds_at_short_and_long_addresses() {
    for packet in [
        DccPacket::Idle,
        DccPacket::Reset,
        DccPacket::BroadcastStop,
        DccPacket::ServiceModeVerifyByte {
            cv: service_cv(256),
            value: 192,
        },
        DccPacket::ServiceModeWriteByte {
            cv: service_cv(1),
            value: 3,
        },
    ] {
        assert_complete(packet);
    }

    for address in [
        DccAddress::new_short(3).unwrap(),
        DccAddress::new_long(1000).unwrap(),
    ] {
        for packet in [
            DccPacket::speed_28step(address, 15, Direction::Forward).unwrap(),
            DccPacket::speed_128step(address, 64, Direction::Reverse).unwrap(),
            DccPacket::EmergencyStop {
                address,
                direction: Direction::Forward,
            },
            DccPacket::PomReadByte {
                address,
                cv: pom_cv(1024),
            },
            DccPacket::PomWriteByte {
                address,
                cv: pom_cv(29),
                value: 6,
            },
        ] {
            assert_complete(packet);
        }
        for layout in GROUP_LAYOUTS {
            assert_complete(DccPacket::function_group(address, layout.kind, 0b1010_0101));
        }
    }
}

#[test]
fn timing_rejects_a_pulse_shorter_than_a_one_bit() {
    let result = validate_timing(&[PulseCode::new(true, 50, false, 50)]);
    assert!(matches!(
        result,
        Err(ValidationError::TimingOutOfRange { actual_us: 50, .. })
    ));
}

#[test]
fn timing_rejects_a_pulse_between_the_one_and_zero_ranges() {
    let result = validate_timing(&[PulseCode::new(true, 75, false, 75)]);
    assert!(matches!(
        result,
        Err(ValidationError::TimingOutOfRange { actual_us: 75, .. })
    ));
}

#[test]
fn structure_rejects_a_preamble_shorter_than_14_bits() {
    let one = dcc_bit_to_pulse(true);
    let zero = dcc_bit_to_pulse(false);
    let mut pulses = heapless::Vec::<PulseCode, 64>::new();
    pulses.extend_from_slice(&[one; 10]).unwrap();
    pulses.push(zero).unwrap(); // start bit
    pulses.extend_from_slice(&[one; 8]).unwrap(); // 0xFF
    pulses.push(zero).unwrap(); // separator
    pulses.extend_from_slice(&[zero; 8]).unwrap(); // 0x00
    pulses.push(zero).unwrap(); // separator
    pulses.extend_from_slice(&[one; 8]).unwrap(); // 0xFF checksum
    pulses.push(one).unwrap(); // end bit

    assert_eq!(
        validate_packet_structure(&pulses),
        Err(ValidationError::InsufficientPreamble { found: 10 })
    );
}

#[test]
fn structure_rejects_a_stream_shorter_than_a_minimal_packet() {
    let pulses = [dcc_bit_to_pulse(true); 10];
    assert_eq!(
        validate_packet_structure(&pulses),
        Err(ValidationError::PacketTooShort {
            min: 15,
            actual: 10
        })
    );
}

#[test]
fn checksum_check_fails_on_a_corrupted_pulse_stream() {
    // Idle is 0xFF 0x00 0xFF; clearing the first address bit leaves a
    // well-formed stream carrying 0x7F 0x00 0xFF.
    let mut pulses = encode_dcc_packet(&DccPacket::Idle).unwrap();
    let start_bit = pulses
        .iter()
        .position(|pulse| *pulse == dcc_bit_to_pulse(false))
        .expect("encoded packet has a start bit");
    pulses[start_bit + 1] = dcc_bit_to_pulse(false);

    assert_eq!(validate_timing(&pulses), Ok(()));
    assert_eq!(validate_packet_structure(&pulses), Ok(()));
    let corrupted = Err(ValidationError::InvalidChecksum {
        expected: 0x7F,
        actual: 0xFF,
    });
    assert_eq!(validate_checksum(&pulses), corrupted);
    assert_eq!(validate_full(&pulses), corrupted);
}

#[test]
fn full_validation_rejects_a_packet_longer_than_the_byte_limit() {
    let one = dcc_bit_to_pulse(true);
    let zero = dcc_bit_to_pulse(false);
    let mut pulses = heapless::Vec::<PulseCode, 256>::new();
    pulses.extend_from_slice(&[one; 14]).unwrap();
    for _ in 0..=DCC_MAX_PACKET_BYTES {
        pulses.push(zero).unwrap(); // start bit or separator
        pulses.extend_from_slice(&[zero; 8]).unwrap();
    }
    pulses.push(one).unwrap(); // end bit

    assert_eq!(
        validate_full(&pulses),
        Err(ValidationError::PacketTooLong {
            max_bytes: DCC_MAX_PACKET_BYTES
        })
    );
}
