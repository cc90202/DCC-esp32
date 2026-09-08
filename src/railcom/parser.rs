use heapless::Vec;

use crate::dcc::DccAddress;
pub use crate::railcom_data::{RailcomDatagram, RailcomItem};

const SYMBOL_NACK: u8 = 0xfc;
const SYMBOL_ACK: u8 = 0xfd;
const SYMBOL_RESERVED: u8 = 0xfe;
const SYMBOL_INVALID: u8 = 0xff;

const SIX_TO_4_8: [u8; 64] = [
    0b1010_1100,
    0b1010_1010,
    0b1010_1001,
    0b1010_0101,
    0b1010_0011,
    0b1010_0110,
    0b1001_1100,
    0b1001_1010,
    0b1001_1001,
    0b1001_0101,
    0b1001_0011,
    0b1001_0110,
    0b1000_1110,
    0b1000_1101,
    0b1000_1011,
    0b1011_0001,
    0b1011_0010,
    0b1011_0100,
    0b1011_1000,
    0b0111_0100,
    0b0111_0010,
    0b0110_1100,
    0b0110_1010,
    0b0110_1001,
    0b0110_0101,
    0b0110_0011,
    0b0110_0110,
    0b0101_1100,
    0b0101_1010,
    0b0101_1001,
    0b0101_0101,
    0b0101_0011,
    0b0101_0110,
    0b0100_1110,
    0b0100_1101,
    0b0100_1011,
    0b0100_0111,
    0b0111_0001,
    0b1110_1000,
    0b1110_0100,
    0b1110_0010,
    0b1101_0001,
    0b1100_1001,
    0b1100_0101,
    0b1101_1000,
    0b1101_0100,
    0b1101_0010,
    0b1100_1010,
    0b1100_0110,
    0b1100_1100,
    0b0111_1000,
    0b0001_0111,
    0b0001_1011,
    0b0001_1101,
    0b0001_1110,
    0b0010_1110,
    0b0011_0110,
    0b0011_1010,
    0b0010_0111,
    0b0010_1011,
    0b0010_1101,
    0b0011_0101,
    0b0011_1001,
    0b0011_0011,
];

pub(crate) const ACK_1_CODE: u8 = 0b0000_1111;
pub(crate) const ACK_2_CODE: u8 = 0b1111_0000;
pub(crate) const NACK_CODE: u8 = 0b0011_1100;

const fn build_reverse_4_of_8() -> [u8; 256] {
    let mut table = [SYMBOL_INVALID; 256];
    let mut index = 0;
    while index < SIX_TO_4_8.len() {
        table[SIX_TO_4_8[index] as usize] = index as u8;
        index += 1;
    }

    table[ACK_1_CODE as usize] = SYMBOL_ACK;
    table[ACK_2_CODE as usize] = SYMBOL_ACK;
    table[NACK_CODE as usize] = SYMBOL_NACK;
    table[0b1110_0001usize] = SYMBOL_RESERVED;
    table[0b1100_0011usize] = SYMBOL_RESERVED;
    table[0b1000_0111usize] = SYMBOL_RESERVED;
    table
}

const REVERSE_4_OF_8: [u8; 256] = build_reverse_4_of_8();

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodedSymbol {
    Data6(u8),
    Ack,
    Nack,
    Reserved(u8),
    Invalid(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum ParseError {
    Invalid4Of8Code(u8),
    Reserved4Of8Code(u8),
    ControlSymbolNotAllowed(u8),
    DatagramAfterControl,
    TruncatedDatagram {
        id: u8,
        needed_symbols: usize,
        available_symbols: usize,
    },
    InvalidLogonCrc {
        expected: u8,
        actual: u8,
    },
    InvalidLogonAddress(u16),
    UnsupportedDatagramId(u8),
    TooManyItems,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomLogonId {
    pub manufacturer_id: u16,
    pub decoder_id: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct RailcomLogonSelect {
    pub address: DccAddress,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomLogonResponse {
    DecoderId(RailcomLogonId),
    Select(RailcomLogonSelect),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomParseStatus {
    Complete,
    PartialUnsupportedDatagram(u8),
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
/// Parsed items plus completion status for one RailCom receive window.
pub struct RailcomParseResult {
    pub items: Vec<RailcomItem, 6>,
    pub status: RailcomParseStatus,
}

#[must_use]
/// Decode one NMRA 4-of-8 symbol into data, control, or error classification.
pub fn decode_4_of_8(code: u8) -> DecodedSymbol {
    match REVERSE_4_OF_8[code as usize] {
        SYMBOL_ACK => DecodedSymbol::Ack,
        SYMBOL_NACK => DecodedSymbol::Nack,
        SYMBOL_RESERVED => DecodedSymbol::Reserved(code),
        SYMBOL_INVALID => DecodedSymbol::Invalid(code),
        value => DecodedSymbol::Data6(value),
    }
}

/// Decodes a raw 4/8-coded byte window into symbols, capped at `N` symbols.
///
/// Channel-specific packet rules are checked by the callers. RCN-217 §2.5
/// requires strict 4-of-8 validation, including for single-byte responses.
fn decode_symbols<const N: usize>(raw_bytes: &[u8]) -> Result<Vec<DecodedSymbol, N>, ParseError> {
    let mut symbols = Vec::<DecodedSymbol, N>::new();
    for &byte in raw_bytes {
        let decoded = decode_4_of_8(byte);
        match decoded {
            DecodedSymbol::Invalid(code) => return Err(ParseError::Invalid4Of8Code(code)),
            DecodedSymbol::Reserved(code) => return Err(ParseError::Reserved4Of8Code(code)),
            _ => symbols
                .push(decoded)
                .map_err(|_| ParseError::TooManyItems)?,
        }
    }
    Ok(symbols)
}

enum ParsedSymbolItem {
    Item {
        item: RailcomItem,
        consumed_symbols: usize,
    },
    UnsupportedDatagram(u8),
}

fn parse_symbol_item(
    symbols: &[DecodedSymbol],
    index: usize,
) -> Result<ParsedSymbolItem, ParseError> {
    match symbols[index] {
        DecodedSymbol::Ack => Ok(ParsedSymbolItem::Item {
            item: RailcomItem::Ack,
            consumed_symbols: 1,
        }),
        DecodedSymbol::Nack => Ok(ParsedSymbolItem::Item {
            item: RailcomItem::Nack,
            consumed_symbols: 1,
        }),
        DecodedSymbol::Data6(first) => parse_symbol_datagram(symbols, index, first),
        DecodedSymbol::Reserved(code) => Err(ParseError::Reserved4Of8Code(code)),
        DecodedSymbol::Invalid(code) => Err(ParseError::Invalid4Of8Code(code)),
    }
}

fn parse_symbol_datagram(
    symbols: &[DecodedSymbol],
    index: usize,
    first: u8,
) -> Result<ParsedSymbolItem, ParseError> {
    let id = first >> 2;
    let symbol_count = match datagram_symbol_count(id) {
        Ok(symbol_count) => symbol_count,
        Err(ParseError::UnsupportedDatagramId(id)) => {
            return Ok(ParsedSymbolItem::UnsupportedDatagram(id));
        }
        Err(error) => return Err(error),
    };
    let available_symbols = symbols.len() - index;
    if available_symbols < symbol_count {
        return Err(ParseError::TruncatedDatagram {
            id,
            needed_symbols: symbol_count,
            available_symbols,
        });
    }

    let mut useful_bits = 0u64;
    for symbol in &symbols[index..index + symbol_count] {
        let DecodedSymbol::Data6(value) = *symbol else {
            return Err(ParseError::TruncatedDatagram {
                id,
                needed_symbols: symbol_count,
                available_symbols,
            });
        };
        useful_bits = (useful_bits << 6) | u64::from(value);
    }

    let payload_bits = symbol_count * 6 - 4;
    let payload = useful_bits & ((1u64 << payload_bits) - 1);
    Ok(ParsedSymbolItem::Item {
        item: RailcomItem::Datagram(parse_datagram(id, payload, payload_bits as u8)?),
        consumed_symbols: symbol_count,
    })
}

/// Turns already-decoded symbols into RailCom items (ACK/NACK/datagrams).
fn items_from_symbols(symbols: &[DecodedSymbol]) -> Result<RailcomParseResult, ParseError> {
    let mut items = Vec::<RailcomItem, 6>::new();
    let mut index = 0usize;
    while index < symbols.len() {
        match parse_symbol_item(symbols, index)? {
            ParsedSymbolItem::Item {
                item,
                consumed_symbols,
            } => {
                items.push(item).map_err(|_| ParseError::TooManyItems)?;
                index += consumed_symbols;
            }
            ParsedSymbolItem::UnsupportedDatagram(id) => {
                return Ok(RailcomParseResult {
                    items,
                    status: RailcomParseStatus::PartialUnsupportedDatagram(id),
                });
            }
        }
    }

    Ok(RailcomParseResult {
        items,
        status: RailcomParseStatus::Complete,
    })
}

pub fn parse_channel2(raw_bytes: &[u8]) -> Result<RailcomParseResult, ParseError> {
    let symbols = decode_symbols::<6>(raw_bytes)?;
    // RCN-217 §3: a response starting with ACK/NACK may contain further
    // ACK/NACK, but no datagrams. Datagrams followed by ACK padding are valid.
    if matches!(
        symbols.first(),
        Some(DecodedSymbol::Ack | DecodedSymbol::Nack)
    ) && symbols
        .iter()
        .any(|symbol| matches!(symbol, DecodedSymbol::Data6(_)))
    {
        return Err(ParseError::DatagramAfterControl);
    }
    items_from_symbols(&symbols)
}

/// Number of bytes a channel-1 window can legitimately hold (one 12-bit
/// datagram). A third byte is only ever the UART's break-frame tail.
const CHANNEL1_MAX_BYTES: usize = 2;

/// Strips the break-frame tail the UART deposits ahead of channel 1.
///
/// Outside the cutout the detector line sits LOW while the traction current
/// crosses the sense resistor, and it stays LOW until ~49 µs after the
/// reference edge (bench, 2026-09-07). The UART keeps assembling break
/// frames on it; the one in flight when the capture opens completes just
/// before the decoder's first byte and shows up as a leading `0xff`, `0xfe`,
/// `0xfc` (or, with comparator chatter, `0xfd`, `0xec`, ...). None of those
/// is a 4-of-8 code, so dropping a non-code first byte from a three-byte
/// window can never discard decoder data: a three-byte window was invalid
/// for channel 1 anyway.
fn strip_channel1_break_tail(raw_bytes: &[u8]) -> &[u8] {
    match raw_bytes {
        [first, rest @ ..] if rest.len() == CHANNEL1_MAX_BYTES && first.count_ones() != 4 => rest,
        _ => raw_bytes,
    }
}

pub fn parse_channel1(raw_bytes: &[u8]) -> Result<RailcomParseResult, ParseError> {
    let raw_bytes = strip_channel1_break_tail(raw_bytes);
    let symbols = decode_symbols::<2>(raw_bytes)?;
    // RCN-217 §3 prohibits ACK/NACK in CH1. Preserve the raw code in errors.
    for (symbol, &raw) in symbols.iter().zip(raw_bytes) {
        if matches!(symbol, DecodedSymbol::Ack | DecodedSymbol::Nack) {
            return Err(ParseError::ControlSymbolNotAllowed(raw));
        }
    }
    items_from_symbols(&symbols)
}

pub fn parse_logon_response_48(
    channel1_raw: &[u8],
    channel2_raw: &[u8],
) -> Result<RailcomLogonResponse, ParseError> {
    if channel1_raw.len() != 2 {
        return Err(ParseError::TruncatedDatagram {
            id: 15,
            needed_symbols: 8,
            available_symbols: channel1_raw.len() + channel2_raw.len(),
        });
    }
    if channel2_raw.len() != 6 {
        return Err(ParseError::TruncatedDatagram {
            id: 15,
            needed_symbols: 8,
            available_symbols: channel1_raw.len() + channel2_raw.len(),
        });
    }

    let mut useful_bits = 0u64;
    for &byte in channel1_raw.iter().chain(channel2_raw.iter()) {
        match decode_4_of_8(byte) {
            DecodedSymbol::Data6(value) => {
                useful_bits = (useful_bits << 6) | u64::from(value);
            }
            DecodedSymbol::Invalid(code) => return Err(ParseError::Invalid4Of8Code(code)),
            DecodedSymbol::Reserved(code) => return Err(ParseError::Reserved4Of8Code(code)),
            DecodedSymbol::Ack | DecodedSymbol::Nack => {
                return Err(ParseError::ControlSymbolNotAllowed(byte));
            }
        }
    }

    let id = (useful_bits >> 44) as u8;
    if id == 15 {
        let payload = useful_bits & ((1u64 << 44) - 1);
        return Ok(RailcomLogonResponse::DecoderId(RailcomLogonId {
            manufacturer_id: ((payload >> 32) & 0x0fff) as u16,
            decoder_id: (payload & 0xffff_ffff) as u32,
        }));
    }

    let bytes = [
        ((useful_bits >> 40) & 0xff) as u8,
        ((useful_bits >> 32) & 0xff) as u8,
        ((useful_bits >> 24) & 0xff) as u8,
        ((useful_bits >> 16) & 0xff) as u8,
        ((useful_bits >> 8) & 0xff) as u8,
        (useful_bits & 0xff) as u8,
    ];
    if bytes[0] & 0x80 == 0 {
        return Err(ParseError::UnsupportedDatagramId(id));
    }

    let expected_crc = crate::logon::crc8_dallas_maxim(&bytes[..5]);
    let actual_crc = bytes[5];
    if expected_crc != actual_crc {
        return Err(ParseError::InvalidLogonCrc {
            expected: expected_crc,
            actual: actual_crc,
        });
    }

    let raw_address = ((u16::from(bytes[0] & 0x3f)) << 8) | u16::from(bytes[1]);
    let address = if raw_address <= 127 {
        DccAddress::new_short(raw_address as u8)
    } else {
        DccAddress::new_long(raw_address)
    }
    .ok_or(ParseError::InvalidLogonAddress(raw_address))?;

    Ok(RailcomLogonResponse::Select(RailcomLogonSelect { address }))
}

fn datagram_symbol_count(id: u8) -> Result<usize, ParseError> {
    match id {
        0..=2 | 14 => Ok(2),
        3 | 7 => Ok(3),
        8..=11 => Ok(6),
        _ => Err(ParseError::UnsupportedDatagramId(id)),
    }
}

fn parse_datagram(id: u8, payload: u64, payload_bits: u8) -> Result<RailcomDatagram, ParseError> {
    match (id, payload_bits) {
        (0, 8) => Ok(RailcomDatagram::CvData(payload as u8)),
        (1, 8) => Ok(RailcomDatagram::AdrHigh(payload as u8)),
        (2, 8) => Ok(RailcomDatagram::AdrLow(payload as u8)),
        (3, 14) => Ok(RailcomDatagram::Ext(payload as u16)),
        (7, 14) => Ok(RailcomDatagram::Dyn {
            value: (payload >> 6) as u8,
            sub_index: (payload & 0x3f) as u8,
        }),
        (8..=11, 32) => Ok(RailcomDatagram::Xpom {
            sequence: id - 8,
            values: [
                ((payload >> 24) & 0xff) as u8,
                ((payload >> 16) & 0xff) as u8,
                ((payload >> 8) & 0xff) as u8,
                (payload & 0xff) as u8,
            ],
        }),
        (14, 8) => Ok(RailcomDatagram::Search(payload as u8)),
        _ => Err(ParseError::UnsupportedDatagramId(id)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode_data6_for_test(value: u8) -> u8 {
        SIX_TO_4_8[value as usize]
    }

    fn encode_12_bit_for_test(id: u8, payload: u8) -> [u8; 2] {
        let first = (id << 2) | (payload >> 6);
        let second = payload & 0x3f;
        [encode_data6_for_test(first), encode_data6_for_test(second)]
    }

    fn encode_18_bit_for_test(id: u8, payload: u16) -> [u8; 3] {
        let raw = (u32::from(id) << 14) | u32::from(payload);
        [
            encode_data6_for_test(((raw >> 12) & 0x3f) as u8),
            encode_data6_for_test(((raw >> 6) & 0x3f) as u8),
            encode_data6_for_test((raw & 0x3f) as u8),
        ]
    }

    fn encode_36_bit_for_test(id: u8, payload: u32) -> [u8; 6] {
        let raw = (u64::from(id) << 32) | u64::from(payload);
        [
            encode_data6_for_test(((raw >> 30) & 0x3f) as u8),
            encode_data6_for_test(((raw >> 24) & 0x3f) as u8),
            encode_data6_for_test(((raw >> 18) & 0x3f) as u8),
            encode_data6_for_test(((raw >> 12) & 0x3f) as u8),
            encode_data6_for_test(((raw >> 6) & 0x3f) as u8),
            encode_data6_for_test((raw & 0x3f) as u8),
        ]
    }

    fn encode_48_bit_for_test(id: u8, payload: u64) -> [u8; 8] {
        let raw = (u64::from(id) << 44) | payload;
        [
            encode_data6_for_test(((raw >> 42) & 0x3f) as u8),
            encode_data6_for_test(((raw >> 36) & 0x3f) as u8),
            encode_data6_for_test(((raw >> 30) & 0x3f) as u8),
            encode_data6_for_test(((raw >> 24) & 0x3f) as u8),
            encode_data6_for_test(((raw >> 18) & 0x3f) as u8),
            encode_data6_for_test(((raw >> 12) & 0x3f) as u8),
            encode_data6_for_test(((raw >> 6) & 0x3f) as u8),
            encode_data6_for_test((raw & 0x3f) as u8),
        ]
    }

    fn encode_48_data_for_test(payload: u64) -> [u8; 8] {
        [
            encode_data6_for_test(((payload >> 42) & 0x3f) as u8),
            encode_data6_for_test(((payload >> 36) & 0x3f) as u8),
            encode_data6_for_test(((payload >> 30) & 0x3f) as u8),
            encode_data6_for_test(((payload >> 24) & 0x3f) as u8),
            encode_data6_for_test(((payload >> 18) & 0x3f) as u8),
            encode_data6_for_test(((payload >> 12) & 0x3f) as u8),
            encode_data6_for_test(((payload >> 6) & 0x3f) as u8),
            encode_data6_for_test((payload & 0x3f) as u8),
        ]
    }

    #[test]
    fn test_decode_4_of_8_data_ack_and_nack() {
        assert_eq!(decode_4_of_8(0b1010_1100), DecodedSymbol::Data6(0x00));
        assert_eq!(decode_4_of_8(0b0011_0011), DecodedSymbol::Data6(0x3f));
        assert_eq!(decode_4_of_8(ACK_1_CODE), DecodedSymbol::Ack);
        assert_eq!(decode_4_of_8(ACK_2_CODE), DecodedSymbol::Ack);
        assert_eq!(decode_4_of_8(NACK_CODE), DecodedSymbol::Nack);
    }

    #[test]
    fn test_parse_channel2_ack_only() {
        let parsed = parse_channel2(&[ACK_1_CODE, ACK_2_CODE]).expect("ack parse must succeed");
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomItem::Ack, RailcomItem::Ack]
        );
    }

    #[test]
    fn test_channels_reject_every_non_4_of_8_byte() {
        for code in 0u8..=255 {
            if code.count_ones() != 4 {
                assert_eq!(
                    parse_channel1(&[code]),
                    Err(ParseError::Invalid4Of8Code(code))
                );
                assert_eq!(
                    parse_channel2(&[code]),
                    Err(ParseError::Invalid4Of8Code(code))
                );
            }
        }
    }

    #[test]
    fn test_parse_channel2_rejects_jittered_ack_inside_longer_window() {
        let err =
            parse_channel2(&[ACK_1_CODE, 0xf8]).expect_err("invalid symbol must not become an ACK");
        assert_eq!(err, ParseError::Invalid4Of8Code(0xf8));
    }

    #[test]
    fn test_parse_channel2_cv_data_datagram() {
        let raw = encode_12_bit_for_test(0, 0xab);
        let parsed = parse_channel2(&raw).expect("id0 parse must succeed");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomItem::Datagram(RailcomDatagram::CvData(0xab))]
        );
    }

    #[test]
    fn test_parse_channel1_address_low_datagram() {
        let raw = encode_12_bit_for_test(2, 42);
        let parsed = parse_channel1(&raw).expect("channel1 adr low parse must succeed");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomItem::Datagram(RailcomDatagram::AdrLow(42))]
        );
    }

    #[test]
    fn test_parse_channel1_address_high_datagram() {
        let raw = encode_12_bit_for_test(1, 0x03);
        let parsed = parse_channel1(&raw).expect("channel1 adr high parse must succeed");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x03))]
        );
    }

    #[test]
    fn test_parse_channel1_drops_break_tail_byte_before_datagram() {
        // Bench 2026-09-07: the UART chews on the LOW detector line before the
        // cutout and the break frame in flight lands just before the decoder's
        // first byte. Every such byte has more or fewer than four ones.
        let data = encode_12_bit_for_test(1, 0x2a);
        for glitch in [0xffu8, 0xfe, 0xfc, 0xfd, 0xfb, 0xec, 0x6d] {
            let parsed = parse_channel1(&[glitch, data[0], data[1]])
                .expect("leading break-tail byte must be dropped");
            assert_eq!(parsed.status, RailcomParseStatus::Complete);
            assert_eq!(
                parsed.items.as_slice(),
                &[RailcomItem::Datagram(RailcomDatagram::AdrHigh(0x2a))]
            );
        }
    }

    #[test]
    fn test_parse_channel1_keeps_rejecting_three_valid_codes() {
        // A valid 4-of-8 code in first position is not a glitch: the window
        // is still oversized for channel 1 and must fail as before.
        let data = encode_12_bit_for_test(1, 0x2a);
        assert!(parse_channel1(&[SIX_TO_4_8[0], data[0], data[1]]).is_err());
    }

    #[test]
    fn test_parse_channel1_rejects_controls_in_either_position() {
        for code in [ACK_1_CODE, ACK_2_CODE, NACK_CODE] {
            for raw in [
                &[code][..],
                &[code, SIX_TO_4_8[0]][..],
                &[SIX_TO_4_8[0], code][..],
            ] {
                assert_eq!(
                    parse_channel1(raw),
                    Err(ParseError::ControlSymbolNotAllowed(code))
                );
            }
        }
    }

    #[test]
    fn test_channel2_preserves_ack_then_nack() {
        let parsed = parse_channel2(&[ACK_2_CODE, NACK_CODE]).unwrap();
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomItem::Ack, RailcomItem::Nack]
        );
    }

    #[test]
    fn test_parse_channel2_dyn_datagram() {
        let payload = (u16::from(0x5a_u8) << 6) | u16::from(0x15_u8);
        let raw = encode_18_bit_for_test(7, payload);
        let parsed = parse_channel2(&raw).expect("dyn parse must succeed");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomItem::Datagram(RailcomDatagram::Dyn {
                value: 0x5a,
                sub_index: 0x15,
            })]
        );
    }

    #[test]
    fn test_parse_channel2_xpom_datagram() {
        let raw = encode_36_bit_for_test(10, 0x01_02_03_04);
        let parsed = parse_channel2(&raw).expect("xpom parse must succeed");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomItem::Datagram(RailcomDatagram::Xpom {
                sequence: 2,
                values: [1, 2, 3, 4],
            })]
        );
    }

    #[test]
    fn test_parse_channel2_search_datagram() {
        let raw = encode_12_bit_for_test(14, 7);
        let parsed = parse_channel2(&raw).expect("search parse must succeed");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomItem::Datagram(RailcomDatagram::Search(7))]
        );
    }

    #[test]
    fn test_parse_logon_id15_split_across_channels() {
        let payload = (u64::from(0x0D_u16) << 32) | 0x12_34_56_78;
        let raw = encode_48_bit_for_test(15, payload);
        let parsed =
            parse_logon_response_48(&raw[..2], &raw[2..]).expect("id15 parse must succeed");

        assert_eq!(
            parsed,
            RailcomLogonResponse::DecoderId(RailcomLogonId {
                manufacturer_id: 0x0D,
                decoder_id: 0x12_34_56_78,
            })
        );
    }

    #[test]
    fn test_parse_logon_response_select_short_address() {
        let payload = (u64::from(0x80_u8) << 40)
            | (u64::from(3_u8) << 32)
            | (u64::from(0_u8) << 24)
            | (u64::from(0_u8) << 16)
            | (u64::from(0_u8) << 8)
            | u64::from(0x43_u8);
        let raw = encode_48_data_for_test(payload);
        let parsed =
            parse_logon_response_48(&raw[..2], &raw[2..]).expect("select parse must succeed");

        assert_eq!(
            parsed,
            RailcomLogonResponse::Select(RailcomLogonSelect {
                address: DccAddress::new_short(3).unwrap(),
            })
        );
    }

    #[test]
    fn test_parse_logon_response_select_long_address() {
        let payload = (u64::from(0x83_u8) << 40)
            | (u64::from(0xE9_u8) << 32)
            | (u64::from(0_u8) << 24)
            | (u64::from(0_u8) << 16)
            | (u64::from(0_u8) << 8)
            | u64::from(0x5F_u8);
        let raw = encode_48_data_for_test(payload);
        let parsed =
            parse_logon_response_48(&raw[..2], &raw[2..]).expect("select parse must succeed");

        assert_eq!(
            parsed,
            RailcomLogonResponse::Select(RailcomLogonSelect {
                address: DccAddress::new_long(1001).unwrap(),
            })
        );
    }

    #[test]
    fn test_parse_logon_response_select_rejects_bad_crc() {
        let payload = (u64::from(0x80_u8) << 40)
            | (u64::from(3_u8) << 32)
            | (u64::from(0_u8) << 24)
            | (u64::from(0_u8) << 16)
            | (u64::from(0_u8) << 8)
            | u64::from(0x44_u8);
        let raw = encode_48_data_for_test(payload);
        let err =
            parse_logon_response_48(&raw[..2], &raw[2..]).expect_err("bad crc must be rejected");

        assert_eq!(
            err,
            ParseError::InvalidLogonCrc {
                expected: 0x43,
                actual: 0x44,
            }
        );
    }

    #[test]
    fn test_parse_channel2_datagram_with_ack_padding() {
        let mut raw = Vec::<u8, 3>::new();
        for byte in encode_12_bit_for_test(0, 0x42) {
            raw.push(byte).unwrap();
        }
        raw.push(ACK_1_CODE).unwrap();

        let parsed = parse_channel2(raw.as_slice()).expect("mixed parse must succeed");
        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[
                RailcomItem::Datagram(RailcomDatagram::CvData(0x42)),
                RailcomItem::Ack,
            ]
        );
    }

    #[test]
    fn test_channel2_rejects_datagrams_after_initial_control() {
        for control in [ACK_1_CODE, ACK_2_CODE, NACK_CODE] {
            for id in [0, 4] {
                let data = encode_12_bit_for_test(id, 0x42);
                assert_eq!(
                    parse_channel2(&[control, data[0], data[1]]),
                    Err(ParseError::DatagramAfterControl)
                );
            }
        }
    }

    #[test]
    fn test_parse_channel2_rejects_invalid_4_of_8_code() {
        let err = parse_channel2(&[0xff]).expect_err("invalid code must fail");
        assert_eq!(err, ParseError::Invalid4Of8Code(0xff));
    }

    #[test]
    fn test_parse_channel2_rejects_unsupported_id() {
        let raw = encode_12_bit_for_test(4, 0x00);
        let parsed = parse_channel2(&raw).expect("unsupported id should produce partial result");
        assert_eq!(
            parsed.status,
            RailcomParseStatus::PartialUnsupportedDatagram(4)
        );
        assert!(parsed.items.is_empty());
    }

    #[test]
    fn test_parse_channel2_keeps_prefix_before_unsupported_id() {
        let mut raw = Vec::<u8, 4>::new();
        for byte in encode_12_bit_for_test(0, 0x42) {
            raw.push(byte).unwrap();
        }
        for byte in encode_12_bit_for_test(4, 0x00) {
            raw.push(byte).unwrap();
        }

        let parsed =
            parse_channel2(raw.as_slice()).expect("unsupported trailing id should keep prefix");
        assert_eq!(
            parsed.status,
            RailcomParseStatus::PartialUnsupportedDatagram(4)
        );
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomItem::Datagram(RailcomDatagram::CvData(0x42))]
        );
    }
}
