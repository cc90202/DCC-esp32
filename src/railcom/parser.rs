use heapless::Vec;

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
    TruncatedDatagram {
        id: u8,
        needed_symbols: usize,
        available_symbols: usize,
    },
    UnsupportedDatagramId(u8),
    TooManyItems,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomDatagram {
    CvData(u8),
    AdrHigh(u8),
    AdrLow(u8),
    Ext(u16),
    Dyn { value: u8, sub_index: u8 },
    Xpom { sequence: u8, values: [u8; 4] },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomChannel2Item {
    Ack,
    Nack,
    Datagram(RailcomDatagram),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum Channel2ParseStatus {
    Complete,
    PartialUnsupportedDatagram(u8),
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct Channel2ParseResult {
    pub items: Vec<RailcomChannel2Item, 6>,
    pub status: Channel2ParseStatus,
}

#[must_use]
pub fn decode_4_of_8(code: u8) -> DecodedSymbol {
    match REVERSE_4_OF_8[code as usize] {
        SYMBOL_ACK => DecodedSymbol::Ack,
        SYMBOL_NACK => DecodedSymbol::Nack,
        SYMBOL_RESERVED => DecodedSymbol::Reserved(code),
        SYMBOL_INVALID => DecodedSymbol::Invalid(code),
        value => DecodedSymbol::Data6(value),
    }
}

pub fn parse_channel2(raw_bytes: &[u8]) -> Result<Channel2ParseResult, ParseError> {
    let mut symbols = Vec::<DecodedSymbol, 6>::new();
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

    let mut items = Vec::<RailcomChannel2Item, 6>::new();
    let mut index = 0usize;
    while index < symbols.len() {
        match symbols[index] {
            DecodedSymbol::Ack => {
                items
                    .push(RailcomChannel2Item::Ack)
                    .map_err(|_| ParseError::TooManyItems)?;
                index += 1;
            }
            DecodedSymbol::Nack => {
                items
                    .push(RailcomChannel2Item::Nack)
                    .map_err(|_| ParseError::TooManyItems)?;
                index += 1;
            }
            DecodedSymbol::Data6(first) => {
                let id = first >> 2;
                let symbol_count = match datagram_symbol_count(id) {
                    Ok(symbol_count) => symbol_count,
                    Err(ParseError::UnsupportedDatagramId(id)) => {
                        return Ok(Channel2ParseResult {
                            items,
                            status: Channel2ParseStatus::PartialUnsupportedDatagram(id),
                        });
                    }
                    Err(err) => return Err(err),
                };
                let available = symbols.len() - index;
                if available < symbol_count {
                    return Err(ParseError::TruncatedDatagram {
                        id,
                        needed_symbols: symbol_count,
                        available_symbols: available,
                    });
                }

                let mut useful_bits: u64 = 0;
                for symbol in &symbols[index..index + symbol_count] {
                    let DecodedSymbol::Data6(value) = *symbol else {
                        return Err(ParseError::TruncatedDatagram {
                            id,
                            needed_symbols: symbol_count,
                            available_symbols: available,
                        });
                    };
                    useful_bits = (useful_bits << 6) | u64::from(value);
                }

                let payload_bits = symbol_count * 6 - 4;
                let payload_mask = (1u64 << payload_bits) - 1;
                let payload = useful_bits & payload_mask;
                let datagram = parse_datagram(id, payload, payload_bits as u8)?;
                items
                    .push(RailcomChannel2Item::Datagram(datagram))
                    .map_err(|_| ParseError::TooManyItems)?;
                index += symbol_count;
            }
            DecodedSymbol::Reserved(code) => return Err(ParseError::Reserved4Of8Code(code)),
            DecodedSymbol::Invalid(code) => return Err(ParseError::Invalid4Of8Code(code)),
        }
    }

    Ok(Channel2ParseResult {
        items,
        status: Channel2ParseStatus::Complete,
    })
}

fn datagram_symbol_count(id: u8) -> Result<usize, ParseError> {
    match id {
        0..=2 => Ok(2),
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
            &[RailcomChannel2Item::Ack, RailcomChannel2Item::Ack]
        );
    }

    #[test]
    fn test_parse_channel2_cv_data_datagram() {
        let raw = encode_12_bit_for_test(0, 0xab);
        let parsed = parse_channel2(&raw).expect("id0 parse must succeed");

        assert_eq!(parsed.status, Channel2ParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomChannel2Item::Datagram(RailcomDatagram::CvData(0xab))]
        );
    }

    #[test]
    fn test_parse_channel2_dyn_datagram() {
        let payload = (u16::from(0x5a_u8) << 6) | u16::from(0x15_u8);
        let raw = encode_18_bit_for_test(7, payload);
        let parsed = parse_channel2(&raw).expect("dyn parse must succeed");

        assert_eq!(parsed.status, Channel2ParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomChannel2Item::Datagram(RailcomDatagram::Dyn {
                value: 0x5a,
                sub_index: 0x15,
            })]
        );
    }

    #[test]
    fn test_parse_channel2_xpom_datagram() {
        let raw = encode_36_bit_for_test(10, 0x01_02_03_04);
        let parsed = parse_channel2(&raw).expect("xpom parse must succeed");

        assert_eq!(parsed.status, Channel2ParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[RailcomChannel2Item::Datagram(RailcomDatagram::Xpom {
                sequence: 2,
                values: [1, 2, 3, 4],
            })]
        );
    }

    #[test]
    fn test_parse_channel2_mixed_ack_and_datagram() {
        let mut raw = Vec::<u8, 3>::new();
        raw.push(ACK_1_CODE).unwrap();
        for byte in encode_12_bit_for_test(0, 0x42) {
            raw.push(byte).unwrap();
        }

        let parsed = parse_channel2(raw.as_slice()).expect("mixed parse must succeed");
        assert_eq!(parsed.status, Channel2ParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[
                RailcomChannel2Item::Ack,
                RailcomChannel2Item::Datagram(RailcomDatagram::CvData(0x42)),
            ]
        );
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
            Channel2ParseStatus::PartialUnsupportedDatagram(4)
        );
        assert!(parsed.items.is_empty());
    }

    #[test]
    fn test_parse_channel2_keeps_prefix_before_unsupported_id() {
        let mut raw = Vec::<u8, 4>::new();
        raw.push(ACK_1_CODE).unwrap();
        for byte in encode_12_bit_for_test(4, 0x00) {
            raw.push(byte).unwrap();
        }

        let parsed =
            parse_channel2(raw.as_slice()).expect("unsupported trailing id should keep prefix");
        assert_eq!(
            parsed.status,
            Channel2ParseStatus::PartialUnsupportedDatagram(4)
        );
        assert_eq!(parsed.items.as_slice(), &[RailcomChannel2Item::Ack]);
    }
}
