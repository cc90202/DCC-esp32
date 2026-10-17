use super::wire::*;
use super::*;
use dcc_protocol::{DccAddress, Direction, FunctionState, LogicalSpeed, SpeedFormat};

fn cstate(track_on: bool, estop: bool, short_circuit: bool) -> CentralState {
    CentralState {
        track_on,
        estop,
        short_circuit,
    }
}

fn addr_short(n: u8) -> DccAddress {
    DccAddress::new_short(n).unwrap()
}

fn addr_long(n: u16) -> DccAddress {
    DccAddress::new_long(n).unwrap()
}

// Helper: build a minimal valid 4-byte frame with just header
fn frame_header_only(header: u16) -> [u8; 4] {
    let len = 4u16;
    [
        len as u8,
        (len >> 8) as u8,
        header as u8,
        (header >> 8) as u8,
    ]
}

// Helper: compute xbus checksum over a slice
fn cs(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |a, &b| a ^ b)
}

// Helper: build an XBus frame
fn xbus_frame(xheader: u8, extra: &[u8]) -> heapless::Vec<u8, 64> {
    let mut payload: heapless::Vec<u8, 64> = heapless::Vec::new();
    let _ = payload.push(xheader);
    for &b in extra {
        let _ = payload.push(b);
    }
    let checksum = cs(&payload);
    let _ = payload.push(checksum);

    let data_len = (4 + payload.len()) as u16;
    let mut frame: heapless::Vec<u8, 64> = heapless::Vec::new();
    let _ = frame.push(data_len as u8);
    let _ = frame.push((data_len >> 8) as u8);
    let _ = frame.push(HEADER_XBUS as u8);
    let _ = frame.push((HEADER_XBUS >> 8) as u8);
    for b in &payload {
        let _ = frame.push(*b);
    }
    frame
}

// --- Parse tests ---

#[test]
fn test_parse_get_serial_number() {
    let frame = frame_header_only(HEADER_GET_SERIAL_NUMBER);
    assert_eq!(parse_frame(&frame), Ok(Z21Command::GetSerialNumber));
}

#[test]
fn test_parse_get_hwinfo() {
    let frame = frame_header_only(HEADER_GET_HWINFO);
    assert_eq!(parse_frame(&frame), Ok(Z21Command::GetHwInfo));
}

#[test]
fn test_parse_systemstate_getdata() {
    // LAN_SYSTEMSTATE_GETDATA as the spec writes it: DataLen 0x0004, Header 0x0085.
    let frame = [0x04, 0x00, 0x85, 0x00];
    assert_eq!(frame, frame_header_only(HEADER_SYSTEMSTATE_GETDATA));
    assert_eq!(parse_frame(&frame), Ok(Z21Command::GetSystemState));
}

#[test]
fn test_parse_set_broadcast_flags() {
    // DataLen=8, header=0x0050, flags=0x00000001 (LE)
    let buf = [0x08, 0x00, 0x50, 0x00, 0x01, 0x00, 0x00, 0x00];
    assert_eq!(parse_frame(&buf), Ok(Z21Command::SetBroadcastFlags));
}

#[test]
fn test_parse_set_broadcast_flags_too_short() {
    // Only 4 bytes, missing the flags payload
    let buf = [0x04, 0x00, 0x50, 0x00];
    assert_eq!(parse_frame(&buf), Err(ParseError::FrameTooShort));
}

#[test]
fn test_encode_hwinfo() {
    let mut buf = [0u8; 32];
    let n = encode_hwinfo(&mut buf);
    assert_eq!(n, Some(12));
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 12);
    assert_eq!(u16::from_le_bytes([buf[2], buf[3]]), 0x001A);
    // HW_TYPE = Z21 black = 0x00000200
    assert_eq!(
        u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
        0x0000_0200
    );
    // FW_VERSION = 1.40 = 0x00000140
    assert_eq!(
        u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        0x0000_0140
    );
}

#[test]
fn test_parse_get_xbus_version() {
    let frame = xbus_frame(0x21, &[0x21]);
    assert_eq!(parse_frame(&frame), Ok(Z21Command::GetXBusVersion));
}

#[test]
fn test_parse_get_firmware_version() {
    let frame = xbus_frame(0xF1, &[0x0A]);
    assert_eq!(parse_frame(&frame), Ok(Z21Command::GetFirmwareVersion));
}

#[test]
fn test_parse_set_track_power_on() {
    // Per spec §2.6: XBus X-Header=0x21, DB0=0x81, XCS=0xA0
    let frame = xbus_frame(0x21, &[0x81]);
    assert_eq!(parse_frame(&frame), Ok(Z21Command::SetTrackPowerOn));
}

#[test]
fn test_parse_set_track_power_off() {
    // Per spec §2.5: XBus X-Header=0x21, DB0=0x80, XCS=0xA1
    let frame = xbus_frame(0x21, &[0x80]);
    assert_eq!(parse_frame(&frame), Ok(Z21Command::SetTrackPowerOff));
}

#[test]
fn test_parse_set_loco_drive_28_short_addr() {
    // SetLocoDrive Speed28: addr=3, forward, speed step 14
    // NMRA encoding: N=14 (even) → v=((14+1)>>1)+1=8, |0x10=0x18; fwd=bit7 → 0x98
    let frame = [
        0x0A, 0x00, // Length: 10 bytes
        0x40, 0x00, // Header: XBus
        0xE4, // XBus: SetLocoDrive
        0x12, // DB0: Speed28
        0x00, 0x03, // short address 3
        0x98, // forward, speed step 14
        0x6D, // XBus checksum
    ];
    let Ok(Z21Command::SetLocoDrive {
        address,
        speed,
        direction,
        format,
    }) = parse_frame(&frame)
    else {
        panic!("expected SetLocoDrive, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_short(3));
    assert_eq!(direction, Direction::Forward);
    assert_eq!(format, SpeedFormat::Speed28);
    assert_eq!(speed, 14);
}

#[test]
fn short_loco_address_ignores_both_high_flag_bits() {
    for high in [0x00, 0x40, 0x80, 0xc0] {
        let frame = xbus_frame(0xE4, &[0x13, high, 0x03, 0x82]);

        assert!(matches!(
            parse_frame(&frame),
            Ok(Z21Command::SetLocoDrive { address, .. }) if address == addr_short(3)
        ));
    }
}

#[test]
fn long_loco_address_requires_both_high_flag_bits() {
    let frame = xbus_frame(0xE4, &[0x13, 0x01, 0x03, 0x82]);

    assert_eq!(parse_frame(&frame), Err(ParseError::InvalidAddress));
}

#[test]
fn loco_address_rejects_zero_and_values_above_protocol_limit() {
    for [high, low] in [[0xc0, 0x00], [0xe8, 0x00], [0xff, 0xff]] {
        let frame = xbus_frame(0xE4, &[0x13, high, low, 0x82]);

        assert_eq!(parse_frame(&frame), Err(ParseError::InvalidAddress));
    }
}

#[test]
fn test_parse_set_loco_drive_28_long_addr() {
    // addr=1000 long: high = 0xC0 | (1000>>8) = 0xC3, low = 0xE8
    // speed=5 step28 (odd): N=5 → v=((5+1)>>1)+1=4, no mask → 0x04; reverse=bit7 clear → 0x04
    let frame = xbus_frame(0xE4, &[0x12, 0xC3, 0xE8, 0x04]);
    let Ok(Z21Command::SetLocoDrive {
        address,
        speed,
        direction,
        format,
    }) = parse_frame(&frame)
    else {
        panic!("expected SetLocoDrive, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_long(1000));
    assert_eq!(direction, Direction::Reverse);
    assert_eq!(format, SpeedFormat::Speed28);
    assert_eq!(speed, 5);
}

#[test]
fn test_parse_set_loco_drive_128() {
    // addr=5 short, Speed128, forward, wire speed=64 -> logical speed=63
    // SpeedByte = 0x80 | 64 = 0xC0
    let frame = xbus_frame(0xE4, &[0x13, 0x00, 0x05, 0xC0]);
    let Ok(Z21Command::SetLocoDrive {
        address,
        speed,
        direction,
        format,
    }) = parse_frame(&frame)
    else {
        panic!("expected SetLocoDrive, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_short(5));
    assert_eq!(direction, Direction::Forward);
    assert_eq!(format, SpeedFormat::Speed128);
    assert_eq!(speed, 63);
}

#[test]
fn test_parse_set_loco_function_on() {
    // F5 ON: func_byte = (1<<6) | 5 = 0x45
    let frame = xbus_frame(0xE4, &[0xF8, 0x00, 0x03, 0x45]);
    let Ok(Z21Command::SetLocoFunction {
        address,
        function,
        action,
    }) = parse_frame(&frame)
    else {
        panic!("expected SetLocoFunction, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_short(3));
    assert_eq!(function, 5);
    assert_eq!(action, FunctionAction::On);
}

#[test]
fn test_parse_set_loco_function_off() {
    // F0 OFF: func_byte = (0<<6) | 0 = 0x00
    let frame = xbus_frame(0xE4, &[0xF8, 0x00, 0x03, 0x00]);
    let Ok(Z21Command::SetLocoFunction {
        action, function, ..
    }) = parse_frame(&frame)
    else {
        panic!("expected SetLocoFunction, got {:?}", parse_frame(&frame));
    };
    assert_eq!(function, 0);
    assert_eq!(action, FunctionAction::Off);
}

#[test]
fn test_parse_set_loco_function_toggle() {
    // F2 Toggle: func_byte = (2<<6) | 2 = 0x82
    let frame = xbus_frame(0xE4, &[0xF8, 0x00, 0x03, 0x82]);
    let Ok(Z21Command::SetLocoFunction {
        function, action, ..
    }) = parse_frame(&frame)
    else {
        panic!("expected SetLocoFunction, got {:?}", parse_frame(&frame));
    };
    assert_eq!(function, 2);
    assert_eq!(action, FunctionAction::Toggle);
}

#[test]
fn test_parse_set_loco_function_rejects_out_of_range_index() {
    // F29 ON: low 6 bits encode 29, which is outside supported F0..F28 range.
    let frame = xbus_frame(0xE4, &[0xF8, 0x00, 0x03, 0x5D]);
    assert_eq!(parse_frame(&frame), Err(ParseError::InvalidFunction));
    // F28 ON is the last supported function.
    let frame = xbus_frame(0xE4, &[0xF8, 0x00, 0x03, 0x5C]);
    assert!(matches!(
        parse_frame(&frame),
        Ok(Z21Command::SetLocoFunction { function: 28, .. })
    ));
}

#[test]
fn test_parse_set_loco_function_rejects_reserved_action() {
    // Action bits 0b11 are reserved by the Z21 protocol and must fail closed.
    let frame = xbus_frame(0xE4, &[0xF8, 0x00, 0x03, 0xC5]);
    assert_eq!(parse_frame(&frame), Err(ParseError::InvalidFunctionAction));
}

#[test]
fn test_parse_set_stop() {
    let frame = xbus_frame(0x80, &[]);
    assert_eq!(parse_frame(&frame), Ok(Z21Command::SetStop));
}

#[test]
fn test_parse_set_loco_estop() {
    // addr=3 short
    let frame = xbus_frame(0x92, &[0x00, 0x03]);
    let Ok(Z21Command::SetLocoEstop { address }) = parse_frame(&frame) else {
        panic!("expected SetLocoEstop, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_short(3));
}

#[test]
fn test_parse_get_loco_info() {
    // addr=3 short
    let frame = xbus_frame(0xE3, &[0xF0, 0x00, 0x03]);
    let Ok(Z21Command::GetLocoInfo { address }) = parse_frame(&frame) else {
        panic!("expected GetLocoInfo, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_short(3));
}

#[test]
fn test_parse_get_loco_mode() {
    let frame = [0x06, 0x00, 0x60, 0x00, 0x00, 0x03];
    let Ok(Z21Command::GetLocoMode { address }) = parse_frame(&frame) else {
        panic!("expected GetLocoMode, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_short(3));
}

#[test]
fn test_parse_set_loco_mode() {
    let frame = [0x07, 0x00, 0x61, 0x00, 0xC3, 0xE8, 0x00];
    let Ok(Z21Command::SetLocoMode { address, mode }) = parse_frame(&frame) else {
        panic!("expected SetLocoMode, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_long(1000));
    assert_eq!(mode, 0);
}

#[test]
fn test_parse_railcom_getdata() {
    let frame = [0x07, 0x00, 0x89, 0x00, 0x01, 0x03, 0x00];
    let Ok(Z21Command::RailcomGetData {
        request_type,
        address,
    }) = parse_frame(&frame)
    else {
        panic!("expected RailcomGetData, got {:?}", parse_frame(&frame));
    };
    assert_eq!(request_type, RailcomRequestType::Locomotive);
    assert_eq!(address, Some(addr_short(3)));
}

#[test]
fn test_parse_railcom_getdata_zero_address_requests_latest() {
    let frame = [0x07, 0x00, 0x89, 0x00, 0x01, 0x00, 0x00];
    let Ok(Z21Command::RailcomGetData {
        request_type,
        address,
    }) = parse_frame(&frame)
    else {
        panic!("expected RailcomGetData, got {:?}", parse_frame(&frame));
    };
    assert_eq!(request_type, RailcomRequestType::Locomotive);
    assert_eq!(address, None);
}

#[test]
fn test_parse_loconet_detector() {
    let frame = [0x07, 0x00, 0xA4, 0x00, 0x80, 0x00, 0x00];
    assert_eq!(parse_frame(&frame), Ok(Z21Command::LoconetDetector));
}

#[test]
fn test_parse_cv_pom_write_byte() {
    let frame = xbus_frame(0xE6, &[0x30, 0x00, 0x03, 0xEC, 0x1C, 0x06]);
    let Ok(Z21Command::CvPomWriteByte { address, cv, value }) = parse_frame(&frame) else {
        panic!("expected CvPomWriteByte, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_short(3));
    assert_eq!(cv, 29);
    assert_eq!(value, 6);
}

#[test]
fn test_parse_cv_pom_read_byte() {
    let frame = xbus_frame(0xE6, &[0x30, 0xC3, 0xE8, 0xE4, 0x00, 0x00]);
    let Ok(Z21Command::CvPomReadByte { address, cv }) = parse_frame(&frame) else {
        panic!("expected CvPomReadByte, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_long(1000));
    assert_eq!(cv, 1);
}

#[test]
fn test_parse_cv_pom_reads_the_high_cv_bits_from_the_operation_byte() {
    // CV 1024 is wire CV 0x3FF: the top two bits ride in the operation byte.
    let frame = xbus_frame(0xE6, &[0x30, 0x00, 0x03, 0xE7, 0xFF, 0x00]);
    assert_eq!(
        parse_frame(&frame),
        Ok(Z21Command::CvPomReadByte {
            address: addr_short(3),
            cv: 1024,
        })
    );
}

// --- Encode tests ---

#[test]
fn test_encode_loco_info_checksum() {
    let state = LocoInfo {
        address: addr_short(3),
        speed: LogicalSpeed::new(14, SpeedFormat::Speed28).unwrap(),
        direction: Direction::Forward,
        functions: FunctionState::empty(),
    };
    let mut buf = [0u8; 32];
    let n = encode_loco_info(&state, &mut buf);
    assert_eq!(n, Some(14));
    // DataLen = 14
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 14);
    // Header = 0x0040
    assert_eq!(u16::from_le_bytes([buf[2], buf[3]]), 0x0040);
    // X-Header = 0xEF
    assert_eq!(buf[4], 0xEF);
    // Checksum: XOR of buf[4..13)
    let expected_cs = xbus_checksum(&buf[4..13]);
    assert_eq!(buf[13], expected_cs);
}

#[test]
fn test_encode_firmware_version() {
    let mut buf = [0u8; 16];
    let n = encode_firmware_version(&mut buf);
    assert_eq!(n, Some(9));
    assert_eq!(&buf[..8], &[0x09, 0x00, 0x40, 0x00, 0xF3, 0x0A, 0x01, 0x40]);
    assert_eq!(buf[8], 0xF3 ^ 0x0A ^ 0x01 ^ 0x40);
}

#[test]
fn test_encode_loco_mode() {
    let mut buf = [0u8; 16];
    let n = encode_loco_mode(addr_short(3), &mut buf);
    assert_eq!(n, Some(7));
    assert_eq!(&buf[..7], &[0x07, 0x00, 0x60, 0x00, 0x00, 0x03, 0x00]);
}

#[test]
fn test_encode_railcom_data() {
    let mut buf = [0u8; 24];
    let n = encode_railcom_data(addr_long(1000), 42, 1, &mut buf);
    assert_eq!(n, Some(17));
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 17);
    assert_eq!(u16::from_le_bytes([buf[2], buf[3]]), 0x0088);
    assert_eq!(u16::from_le_bytes([buf[4], buf[5]]), 1000);
    assert_eq!(u32::from_le_bytes([buf[6], buf[7], buf[8], buf[9]]), 42);
    assert_eq!(u16::from_le_bytes([buf[10], buf[11]]), 1);
    assert_eq!(buf[13], 0);
}

#[test]
fn test_encode_loco_info_function_layout() {
    // F0=1, F1=1, F4=1
    // functions bitmask: bit0=F0=1, bit1=F1=1, bit4=F4=1
    let functions = FunctionState::from_bits((1 << 0) | (1 << 1) | (1 << 4)).unwrap();
    let state = LocoInfo {
        address: addr_short(3),
        speed: LogicalSpeed::zero(SpeedFormat::Speed128),
        direction: Direction::Forward,
        functions,
    };
    let mut buf = [0u8; 32];
    encode_loco_info(&state, &mut buf).expect("test response buffer must fit");
    // buf[7] = DB2 = KKK=4 (128-step), B=0
    assert_eq!(buf[7], 4, "DB2 KKK should be 4 for Speed128");
    // buf[8] = DB3 = speed (0, forward) = 0x80
    assert_eq!(buf[8], 0x80, "DB3 should be forward+stop");
    // buf[9] = DB4 = F0(bit4)=1, F4(bit3)=1, F3(bit2)=0, F2(bit1)=0, F1(bit0)=1
    // = 0b00011001 = 0x19
    assert_eq!(buf[9], 0x19, "DB4 function byte mismatch");
}

#[test]
fn test_encode_cv_result() {
    let mut buf = [0u8; 16];
    let n = encode_cv_result(29, 6, &mut buf);
    assert_eq!(n, Some(10));
    assert_eq!(
        &buf[..9],
        &[0x0A, 0x00, 0x40, 0x00, 0x64, 0x14, 0x00, 0x1C, 0x06]
    );
    assert_eq!(buf[9], 0x64 ^ 0x14 ^ 0x1C ^ 0x06);

    // CV 1024 is wire CV 0x3FF, split over DB1 and DB2.
    encode_cv_result(1024, 6, &mut buf).expect("CV 1024 is in range");
    assert_eq!(&buf[6..8], &[0x03, 0xFF]);
}

#[test]
fn test_encode_cv_nack() {
    let mut buf = [0u8; 16];
    let n = encode_cv_nack(&mut buf);
    assert_eq!(n, Some(7));
    assert_eq!(&buf[..7], &[0x07, 0x00, 0x40, 0x00, 0x61, 0x13, 0x72]);
}

#[test]
fn test_encode_loco_info_speed28() {
    // NMRA intermediate-bit encoding (Z21 spec §4.2):
    // Step 14 (even): v=((14+1)>>1)+1=8, |0x10=0x18, fwd → 0x80|0x18=0x98
    let state = LocoInfo {
        address: addr_short(3),
        speed: LogicalSpeed::new(14, SpeedFormat::Speed28).unwrap(),
        direction: Direction::Forward,
        functions: FunctionState::empty(),
    };
    let mut buf = [0u8; 32];
    encode_loco_info(&state, &mut buf).expect("test response buffer must fit");
    // DB2: KKK=2 (28-step)
    assert_eq!(buf[7], 2, "DB2 KKK should be 2 for Speed28");
    // DB3: forward(0x80) | 0x18 = 0x98
    assert_eq!(buf[8], 0x98, "Speed28 step14: NMRA wire=0x18, fwd=0x98");

    // Step 0 → stop → wire=0x00, forward = 0x80
    let mut state0 = state;
    state0.speed = LogicalSpeed::zero(SpeedFormat::Speed28);
    encode_loco_info(&state0, &mut buf).expect("test response buffer must fit");
    assert_eq!(buf[8], 0x80, "Speed28 stop: DB3=0x80 (fwd, stop)");

    // Step 1 (odd): v=((1+1)>>1)+1=2, no mask → 0x02, fwd → 0x82
    let mut state1 = state;
    state1.speed = LogicalSpeed::new(1, SpeedFormat::Speed28).unwrap();
    encode_loco_info(&state1, &mut buf).expect("test response buffer must fit");
    assert_eq!(buf[8], 0x82, "Speed28 step1: wire=0x02, fwd=0x82");

    // Step 28 (even): v=((28+1)>>1)+1=15, |0x10=0x1F, fwd → 0x9F
    let mut state28 = state;
    state28.speed = LogicalSpeed::new(28, SpeedFormat::Speed28).unwrap();
    encode_loco_info(&state28, &mut buf).expect("test response buffer must fit");
    assert_eq!(buf[8], 0x9F, "Speed28 step28: NMRA wire=0x1F, fwd=0x9F");
}

#[test]
fn test_encode_bc_track_power_on_off() {
    let mut buf = [0u8; 32];

    let n = encode_bc_track_power(true, &mut buf);
    assert_eq!(n, Some(7));
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 7);
    assert_eq!(buf[4], 0x61);
    assert_eq!(buf[5], 0x01);
    let cs_on = xbus_checksum(&buf[4..6]);
    assert_eq!(buf[6], cs_on);

    let n = encode_bc_track_power(false, &mut buf);
    assert_eq!(n, Some(7));
    assert_eq!(buf[5], 0x00);
    let cs_off = xbus_checksum(&buf[4..6]);
    assert_eq!(buf[6], cs_off);
}

#[test]
fn test_encode_system_state_central_state_bits() {
    let mut buf = [0u8; 32];

    // Normal: all bits clear
    let n = encode_system_state(cstate(true, false, false), &mut buf);
    assert_eq!(n, Some(20));
    assert_eq!(buf[16], 0x00);

    // E-stop: bit0
    encode_system_state(cstate(true, true, false), &mut buf)
        .expect("test response buffer must fit");
    assert_eq!(buf[16], 0x01);

    // Track off: bit1
    encode_system_state(cstate(false, false, false), &mut buf)
        .expect("test response buffer must fit");
    assert_eq!(buf[16], 0x02);

    // Short circuit: bit1 + bit2 (track off implied by short)
    encode_system_state(cstate(false, false, true), &mut buf)
        .expect("test response buffer must fit");
    assert_eq!(buf[16], 0x06);

    // All flags: bit0 + bit1 + bit2
    encode_system_state(cstate(false, true, true), &mut buf)
        .expect("test response buffer must fit");
    assert_eq!(buf[16], 0x07);
}

#[test]
fn test_encode_bc_stopped() {
    let mut buf = [0u8; 32];
    let n = encode_bc_stopped(&mut buf);
    assert_eq!(n, Some(6));
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 6);
    assert_eq!(buf[4], 0x81);
    assert_eq!(buf[5], xbus_checksum(&[0x81]));
}

#[test]
fn test_encode_unknown_command() {
    let mut buf = [0u8; 32];
    let n = encode_unknown_command(&mut buf);
    assert_eq!(n, Some(7));
    assert_eq!(buf[4], 0x61);
    assert_eq!(buf[5], 0x82);
    let cs = xbus_checksum(&[0x61, 0x82]);
    assert_eq!(buf[6], cs);
}

#[test]
fn test_encode_serial_number() {
    let mut buf = [0u8; 32];
    let n = encode_serial_number(0xDEAD_BEEF, &mut buf);
    assert_eq!(n, Some(8));
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 8);
    // little-endian
    assert_eq!(buf[4], 0xEF);
    assert_eq!(buf[5], 0xBE);
    assert_eq!(buf[6], 0xAD);
    assert_eq!(buf[7], 0xDE);
}

#[test]
fn test_all_encoders_reject_short_output_buffers() {
    let state = LocoInfo {
        address: addr_short(3),
        speed: LogicalSpeed::zero(SpeedFormat::Speed128),
        direction: Direction::Forward,
        functions: FunctionState::empty(),
    };
    let mut empty = [];

    assert_eq!(encode_loco_info(&state, &mut empty), None);
    assert_eq!(encode_bc_track_power(true, &mut empty), None);
    assert_eq!(encode_bc_stopped(&mut empty), None);
    assert_eq!(encode_unknown_command(&mut empty), None);
    assert_eq!(encode_cv_result(1, 0, &mut empty), None);
    assert_eq!(encode_cv_nack(&mut empty), None);
    assert_eq!(encode_turnout_info(addr_short(3), &mut empty), None);
    assert_eq!(encode_serial_number(0, &mut empty), None);
    assert_eq!(encode_xbus_version(&mut empty), None);
    assert_eq!(encode_firmware_version(&mut empty), None);
    assert_eq!(encode_code(&mut empty), None);
    assert_eq!(encode_hwinfo(&mut empty), None);
    assert_eq!(encode_status(cstate(true, false, false), &mut empty), None);
    assert_eq!(
        encode_system_state(cstate(true, false, false), &mut empty),
        None
    );
    assert_eq!(encode_loco_mode(addr_short(3), &mut empty), None);
    assert_eq!(encode_railcom_data(addr_short(3), 0, 0, &mut empty), None);
}

#[test]
fn test_encode_cv_result_rejects_out_of_range_cv() {
    let mut buf = [0u8; 10];

    assert_eq!(encode_cv_result(0, 0, &mut buf), None);
    assert_eq!(encode_cv_result(1025, 0, &mut buf), None);
}

#[test]
fn test_encode_status_flags() {
    let mut buf = [0u8; 32];

    // XBus LAN_X_STATUS_CHANGED: header=0x0040, X-Header=0x62, DB0=0x22, DB1=Status, XCS
    // Status is at buf[6], X-Header at buf[4], DB0 at buf[5]

    // track on, no estop → status = 0x00
    encode_status(cstate(true, false, false), &mut buf).expect("test response buffer must fit");
    assert_eq!(buf[4], 0x62); // X-Header
    assert_eq!(buf[5], 0x22); // DB0
    assert_eq!(buf[6], 0x00); // status: no flags

    // track off, no estop → status = 0x02 (csTrackVoltageOff)
    encode_status(cstate(false, false, false), &mut buf).expect("test response buffer must fit");
    assert_eq!(buf[6], 0x02);

    // track on, estop → status = 0x01 (csEmergencyStop)
    encode_status(cstate(true, true, false), &mut buf).expect("test response buffer must fit");
    assert_eq!(buf[6], 0x01);

    // track off, estop → status = 0x03
    encode_status(cstate(false, true, false), &mut buf).expect("test response buffer must fit");
    assert_eq!(buf[6], 0x03);
}

// --- Error/fuzz tests ---

#[test]
fn test_invalid_frame_too_short() {
    assert_eq!(
        parse_frame(&[0x04, 0x00, 0x10]),
        Err(ParseError::FrameTooShort)
    );
    assert_eq!(parse_frame(&[]), Err(ParseError::FrameTooShort));
}

#[test]
fn every_command_cut_short_is_rejected_before_its_fields_are_read() {
    let raw_frames: [&[u8]; 4] = [
        &[0x05, 0x00, HEADER_GET_LOCOMODE as u8, 0x00, 0x00],
        &[0x06, 0x00, HEADER_SET_LOCOMODE as u8, 0x00, 0xC3, 0xE8],
        &[0x06, 0x00, HEADER_RAILCOM_GETDATA as u8, 0x00, 0x01, 0x03],
        &[0x06, 0x00, HEADER_LOCONET_DETECTOR as u8, 0x00, 0x80, 0x00],
    ];
    for frame in raw_frames {
        assert_eq!(
            parse_frame(frame),
            Err(ParseError::FrameTooShort),
            "{frame:02X?}"
        );
    }

    let xbus_frames = [
        xbus_frame(XHEADER_GET_VERSION, &[]),
        xbus_frame(XHEADER_SET_LOCO_ESTOP, &[0x00]),
        xbus_frame(XHEADER_GET_LOCO_INFO, &[0xF0, 0x00]),
        xbus_frame(XHEADER_SET_LOCO_DRIVE, &[0x13, 0x00, 0x03]),
        xbus_frame(XHEADER_CV_POM, &[0x30, 0x00, 0x03, 0xEC, 0x1C]),
        xbus_frame(XHEADER_GET_FIRMWARE_VERSION, &[]),
    ];
    for frame in &xbus_frames {
        assert_eq!(
            parse_frame(frame),
            Err(ParseError::FrameTooShort),
            "{frame:02X?}"
        );
    }
}

#[test]
fn test_invalid_frame_len_mismatch() {
    // Says len=8 but only 4 bytes
    let buf = [0x08, 0x00, 0x10, 0x00];
    assert_eq!(parse_frame(&buf), Err(ParseError::LenMismatch));
}

#[test]
fn test_invalid_xbus_checksum() {
    // Build a valid frame then corrupt the checksum
    let mut frame = xbus_frame(0x21, &[0x21]);
    let last = frame.len() - 1;
    let last_byte = frame[last];
    frame[last] = last_byte ^ 0xFF; // corrupt checksum
    assert_eq!(parse_frame(&frame), Err(ParseError::BadXBusChecksum));
}

#[test]
fn test_fuzz_random_bytes() {
    // Various random-ish byte sequences; must not panic, only return Err or Ok(Unknown)
    let cases: &[&[u8]] = &[
        &[0xFF, 0xFF, 0xFF, 0xFF],
        &[0x04, 0x00, 0xFF, 0xFF],
        &[0x06, 0x00, 0x40, 0x00, 0xAA, 0xBB],
        &[0x07, 0x00, 0x40, 0x00, 0x21, 0x21, 0xFF],
        &[0x04, 0x00, 0x00, 0x00],
    ];
    for &buf in cases {
        let _ = parse_frame(buf); // must not panic
    }
}

#[test]
fn test_loco_address_short_encoding() {
    // Short address round-trip in drive frame
    let frame = xbus_frame(0xE4, &[0x13, 0x00, 0x7F, 0x80]);
    let Ok(Z21Command::SetLocoDrive { address, .. }) = parse_frame(&frame) else {
        panic!("expected SetLocoDrive, got {:?}", parse_frame(&frame));
    };
    assert!(address.is_short());
    assert_eq!(address.value(), 127);
}

#[test]
fn test_loco_address_long_encoding() {
    // addr=200: high = 0xC0|(200>>8)=0xC0, low=200
    let frame = xbus_frame(0xE4, &[0x13, 0xC0, 200, 0x80]);
    let Ok(Z21Command::SetLocoDrive { address, .. }) = parse_frame(&frame) else {
        panic!("expected SetLocoDrive, got {:?}", parse_frame(&frame));
    };
    assert!(address.is_long());
    assert_eq!(address.value(), 200);
}

#[test]
fn test_function_bitmask_f0_f28() {
    // F1 and F4 stay off so no neighbouring bit leaks into their places in
    // DB4; F5/F12, F13 and F28 mark the edges of DB5, DB6 and DB7.
    let on = [0, 2, 3, 5, 12, 13, 28];
    let state = LocoInfo {
        address: addr_long(1000),
        speed: LogicalSpeed::zero(SpeedFormat::Speed128),
        direction: Direction::Forward,
        functions: FunctionState::from_bits(on.iter().fold(0, |bits, f| bits | (1 << f))).unwrap(),
    };
    let mut buf = [0u8; 32];
    encode_loco_info(&state, &mut buf).expect("test response buffer must fit");
    assert_eq!(
        &buf[5..7],
        &[0xC3, 0xE8],
        "long address 1000, flag bits set"
    );
    // DB4 = 0DSLFGHJ: L=F0, F=F4, G=F3, H=F2, J=F1.
    assert_eq!(&buf[9..13], &[0b0001_0110, 0x81, 0x01, 0x80]);
}

#[test]
fn test_encode_loco_info_speed128_avoids_wire_estop_value() {
    let state = LocoInfo {
        address: addr_short(3),
        speed: LogicalSpeed::new(1, SpeedFormat::Speed128).unwrap(),
        direction: Direction::Forward,
        functions: FunctionState::empty(),
    };
    let mut buf = [0u8; 32];
    encode_loco_info(&state, &mut buf).expect("test response buffer must fit");
    assert_eq!(buf[8], 0x82, "logical speed 1 must encode as wire speed 2");
}

// --- Turnout (0x43) tests ---

#[test]
fn test_parse_get_turnout_info_address_5() {
    // LAN_X_GET_TURNOUT_INFO: X-Header=0x43, AddrH=0x00, AddrL=0x05
    let frame = xbus_frame(0x43, &[0x00, 0x05]);
    let Ok(Z21Command::GetTurnoutInfo { address }) = parse_frame(&frame) else {
        panic!("expected GetTurnoutInfo, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_short(5));
}

#[test]
fn test_parse_get_turnout_info_too_short() {
    // xbus_frame generates a valid checksum, so the CRC check passes and we
    // reach the payload-length guard inside the XHEADER_TURNOUT_INFO arm.
    // payload = [0x43, 0x00, xcs] = 3 bytes < 4 required → FrameTooShort.
    let frame = xbus_frame(0x43, &[0x00]); // xheader + AddrH only, missing AddrL
    assert_eq!(parse_frame(&frame), Err(ParseError::FrameTooShort));
    // Raw buffer shorter than 6 bytes hits the early guard before checksum.
    let buf = [0x05, 0x00, 0x40, 0x00, 0x43];
    assert_eq!(parse_frame(&buf), Err(ParseError::FrameTooShort));
}

#[test]
fn test_parse_get_turnout_info_high_address() {
    // address = 0x01_00 = 256
    let frame = xbus_frame(0x43, &[0x01, 0x00]);
    let Ok(Z21Command::GetTurnoutInfo { address }) = parse_frame(&frame) else {
        panic!("expected GetTurnoutInfo, got {:?}", parse_frame(&frame));
    };
    assert_eq!(address, addr_long(256));
}

#[test]
fn test_encode_turnout_info_unknown_state() {
    // Response for address=5, state=0 (unknown/not-owned)
    // LEN=9, Header=0x0040, X-Header=0x43, AddrH=0x00, AddrL=0x05, DB2=0x00, XCS
    // XCS = 0x43^0x00^0x05^0x00 = 0x46
    let mut buf = [0u8; 16];
    let n = encode_turnout_info(addr_short(5), &mut buf);
    assert_eq!(n, Some(9));
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 9); // DataLen
    assert_eq!(u16::from_le_bytes([buf[2], buf[3]]), 0x0040); // Header
    assert_eq!(buf[4], 0x43); // X-Header
    assert_eq!(buf[5], 0x00); // AddrH
    assert_eq!(buf[6], 0x05); // AddrL
    assert_eq!(buf[7], 0x00); // DB2: state unknown
    assert_eq!(buf[8], 0x43 ^ 0x05); // XCS

    encode_turnout_info(addr_long(300), &mut buf).expect("test response buffer must fit");
    assert_eq!(&buf[5..7], &[0x01, 0x2C], "address 300 high byte first");
}

#[test]
fn frame_iterator_reports_one_terminal_boundary_error() {
    let first = frame_header_only(0x0010);
    let mut datagram = [0u8; 6];
    datagram[..4].copy_from_slice(&first);
    datagram[4..].copy_from_slice(&[0x06, 0x00]);

    let mut frames = iter_frames(&datagram);
    assert_eq!(frames.next(), Some(Ok(first.as_slice())));
    assert_eq!(
        frames.next(),
        Some(Err(FrameBoundaryError::TruncatedHeader { remaining: 2 }))
    );
    assert_eq!(frames.next(), None);
}

#[test]
fn frame_iterator_rejects_declared_length_past_datagram() {
    let datagram = [0x08, 0x00, 0x10, 0x00];
    assert_eq!(
        iter_frames(&datagram).next(),
        Some(Err(FrameBoundaryError::InvalidLength {
            declared: 8,
            remaining: 4,
        }))
    );
}

// --- Shared encoding helpers ---

#[test]
fn status_changed_omits_the_short_bit_while_system_state_sets_it() {
    let shorted = CentralState {
        track_on: false,
        estop: false,
        short_circuit: true,
    };
    let mut status = [0u8; 8];
    let mut system_state = [0u8; 20];

    encode_status(shorted, &mut status).expect("test response buffer must fit");
    encode_system_state(shorted, &mut system_state).expect("test response buffer must fit");

    assert_eq!(status[6], CS_TRACK_VOLTAGE_OFF);
    assert_eq!(status[7], xbus_checksum(&status[4..7]));
    assert_eq!(system_state[16], CS_TRACK_VOLTAGE_OFF | CS_SHORT_CIRCUIT);
}

#[test]
fn speed128_logical_round_trips_for_0_to_126() {
    for logical in 0..=126u8 {
        let raw = speed128_logical_to_wire(logical);
        assert!(raw <= 0x7F, "logical {logical} encoded outside 7 bits");
        assert_eq!(speed128_wire_to_logical(raw), logical);
    }
}

#[test]
fn speed128_raw_edge_cases_0_1_127() {
    // Raw 0 is stop, raw 1 is emergency stop and decodes to stop as well.
    assert_eq!(speed128_wire_to_logical(0), 0);
    assert_eq!(speed128_wire_to_logical(1), 0);
    // Raw 127 is the top step, logical 126.
    assert_eq!(speed128_wire_to_logical(127), 126);
    assert_eq!(speed128_logical_to_wire(0), 0);
    assert_eq!(speed128_logical_to_wire(1), 2);
    assert_eq!(speed128_logical_to_wire(126), 127);
}
