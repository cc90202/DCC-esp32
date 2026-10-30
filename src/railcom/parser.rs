use core::fmt;

use heapless::Vec;

use crate::cutout::RailcomChannel;
use crate::dcc::DccAddress;
use dcc_protocol::railcom_data::{ExtPayload, RailcomDatagram, RailcomItem};

// Reverse-table sentinels. Every one must stay above 0x3f, the largest value
// a six-bit data symbol can decode to, or it would be read back as data.
const SYMBOL_BUSY: u8 = 0xfb;
const SYMBOL_NACK: u8 = 0xfc;
const SYMBOL_ACK: u8 = 0xfd;
const SYMBOL_RESERVED: u8 = 0xfe;
const SYMBOL_INVALID: u8 = 0xff;
const _: () = assert!(
    SYMBOL_BUSY > 0x3f
        && SYMBOL_NACK > 0x3f
        && SYMBOL_ACK > 0x3f
        && SYMBOL_RESERVED > 0x3f
        && SYMBOL_INVALID > 0x3f
);

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

/// RCN-217 §2.5: the decoder accepted the command. Two codes carry it.
pub(crate) const ACK_1_CODE: u8 = 0b0000_1111;
pub(crate) const ACK_2_CODE: u8 = 0b1111_0000;
/// RCN-217 §2.5: the decoder refuses the command.
pub(crate) const NACK_CODE: u8 = 0b0011_1100;
/// NMRA S-9.3.2: the decoder is busy and cannot answer this request.
pub(crate) const BUSY_CODE: u8 = 0b1110_0001;
/// One of the two codes of weight four no standard has given a meaning yet.
const RESERVED_1_CODE: u8 = 0b1100_0011;
/// The other code of weight four no standard has given a meaning yet.
const RESERVED_2_CODE: u8 = 0b1000_0111;

const fn build_reverse_4_of_8() -> [u8; 256] {
    let mut table = [SYMBOL_INVALID; 256];
    let mut index = 0;
    while index < SIX_TO_4_8.len() {
        table[SIX_TO_4_8[index] as usize] = index as u8;
        index += 1;
    }

    // There are 70 codes of weight four and the data table holds 64 of them, so
    // the six special codes must be exactly the six left over. Each must
    // therefore weigh four, and must find its slot still unassigned: together
    // those two checks prove that none of the six shadows a data symbol and
    // that no two of them are the same code.
    let special = [
        (ACK_1_CODE, SYMBOL_ACK),
        (ACK_2_CODE, SYMBOL_ACK),
        (NACK_CODE, SYMBOL_NACK),
        (BUSY_CODE, SYMBOL_BUSY),
        (RESERVED_1_CODE, SYMBOL_RESERVED),
        (RESERVED_2_CODE, SYMBOL_RESERVED),
    ];
    let mut index = 0;
    while index < special.len() {
        let (code, symbol) = special[index];
        assert!(code.count_ones() == 4);
        assert!(table[code as usize] == SYMBOL_INVALID);
        table[code as usize] = symbol;
        index += 1;
    }
    table
}

const REVERSE_4_OF_8: [u8; 256] = build_reverse_4_of_8();

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodedSymbol {
    Data6(u8),
    Ack,
    Nack,
    Busy,
    Reserved(u8),
    Invalid(u8),
}

impl DecodedSymbol {
    /// Whether this is a control symbol: a reply in its own right, not payload.
    ///
    /// Written as an exhaustive match so that giving a meaning to one of the
    /// codes still reserved — which is exactly what happened to `0xE1` — makes
    /// the compiler point at this one place instead of leaving the rules that
    /// depend on it silently unchanged.
    const fn is_control(self) -> bool {
        match self {
            Self::Ack | Self::Nack | Self::Busy => true,
            Self::Data6(_) | Self::Reserved(_) | Self::Invalid(_) => false,
        }
    }
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
    /// `offset` counts symbols from the start of the datagram, not of the window.
    ControlSymbolInsideDatagram {
        id: u8,
        offset: usize,
    },
    InvalidLogonCrc {
        expected: u8,
        actual: u8,
    },
    InvalidLogonAddress(u16),
    UnsupportedDatagramId(u8),
    TooManyItems,
}

impl fmt::Display for ParseError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Invalid4Of8Code(value) => {
                write!(formatter, "invalid RailCom 4-of-8 code {value:#04x}")
            }
            Self::Reserved4Of8Code(value) => {
                write!(formatter, "reserved RailCom 4-of-8 code {value:#04x}")
            }
            Self::ControlSymbolNotAllowed(value) => write!(
                formatter,
                "RailCom control symbol {value:#04x} is not allowed here"
            ),
            Self::DatagramAfterControl => {
                formatter.write_str("RailCom datagram follows a terminal control symbol")
            }
            Self::TruncatedDatagram {
                id,
                needed_symbols,
                available_symbols,
            } => write!(
                formatter,
                "truncated RailCom datagram {id}: needs {needed_symbols} symbols, has {available_symbols}"
            ),
            Self::ControlSymbolInsideDatagram { id, offset } => write!(
                formatter,
                "RailCom control symbol at position {offset} inside datagram {id}"
            ),
            Self::InvalidLogonCrc { expected, actual } => write!(
                formatter,
                "invalid RailCom logon CRC: expected {expected:#04x}, received {actual:#04x}"
            ),
            Self::InvalidLogonAddress(address) => {
                write!(formatter, "invalid RailCom logon address {address}")
            }
            Self::UnsupportedDatagramId(id) => {
                write!(formatter, "unsupported RailCom datagram identifier {id}")
            }
            Self::TooManyItems => formatter.write_str("RailCom decoded-item capacity exceeded"),
        }
    }
}

impl core::error::Error for ParseError {}

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
        SYMBOL_BUSY => DecodedSymbol::Busy,
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
            DecodedSymbol::Data6(_)
            | DecodedSymbol::Ack
            | DecodedSymbol::Nack
            | DecodedSymbol::Busy => symbols
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
    channel: RailcomChannel,
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
        DecodedSymbol::Busy => Ok(ParsedSymbolItem::Item {
            item: RailcomItem::Busy,
            consumed_symbols: 1,
        }),
        DecodedSymbol::Data6(first) => parse_symbol_datagram(symbols, index, first, channel),
        DecodedSymbol::Reserved(code) => Err(ParseError::Reserved4Of8Code(code)),
        DecodedSymbol::Invalid(code) => Err(ParseError::Invalid4Of8Code(code)),
    }
}

fn parse_symbol_datagram(
    symbols: &[DecodedSymbol],
    index: usize,
    first: u8,
    channel: RailcomChannel,
) -> Result<ParsedSymbolItem, ParseError> {
    let id = first >> 2;
    let symbol_count = match resolve_symbol_count(symbols, index, id, channel) {
        Ok(symbol_count) => symbol_count,
        Err(ParseError::UnsupportedDatagramId(id)) => {
            return Ok(ParsedSymbolItem::UnsupportedDatagram(id));
        }
        Err(error) => return Err(error),
    };
    let needed_symbols = usize::from(symbol_count);
    let available_symbols = symbols.len() - index;
    if available_symbols < needed_symbols {
        return Err(ParseError::TruncatedDatagram {
            id,
            needed_symbols,
            available_symbols,
        });
    }

    let mut useful_bits = 0u64;
    for (offset, symbol) in symbols[index..index + needed_symbols].iter().enumerate() {
        let DecodedSymbol::Data6(value) = *symbol else {
            return Err(ParseError::ControlSymbolInsideDatagram { id, offset });
        };
        useful_bits = (useful_bits << 6) | u64::from(value);
    }

    let payload_bits = symbol_count * 6 - 4;
    let payload = useful_bits & ((1u64 << payload_bits) - 1);
    Ok(ParsedSymbolItem::Item {
        item: RailcomItem::Datagram(parse_datagram(id, payload, payload_bits, channel)?),
        consumed_symbols: needed_symbols,
    })
}

/// Turns already-decoded symbols into RailCom items: control symbols and
/// datagrams.
fn items_from_symbols(
    symbols: &[DecodedSymbol],
    channel: RailcomChannel,
) -> Result<RailcomParseResult, ParseError> {
    let mut items = Vec::<RailcomItem, 6>::new();
    let mut index = 0usize;
    while index < symbols.len() {
        match parse_symbol_item(symbols, index, channel)? {
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

/// Decodes one channel-2 receive window into RailCom items.
///
/// Rejects the window whole on the first byte that is not a 4-of-8 code, and on
/// a window that opens with a control symbol and then carries a datagram: a
/// control symbol between two datagrams is padding and is kept. Accepts the
/// short `app:ext` some decoders emit in place of the conforming three-symbol
/// one, whenever nothing that could be its third symbol follows.
pub fn parse_channel2(raw_bytes: &[u8]) -> Result<RailcomParseResult, ParseError> {
    let symbols = decode_symbols::<6>(raw_bytes)?;
    // RCN-217 §3: a response starting with a control symbol may contain
    // further control symbols, but no datagrams. Datagrams followed by ACK
    // padding are valid.
    if symbols.first().is_some_and(|symbol| symbol.is_control())
        && symbols
            .iter()
            .any(|symbol| matches!(symbol, DecodedSymbol::Data6(_)))
    {
        return Err(ParseError::DatagramAfterControl);
    }
    items_from_symbols(&symbols, RailcomChannel::Channel2)
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

/// Decodes one channel-1 receive window into RailCom items.
///
/// Strips the UART break-frame tail, rejects the window whole on the first byte
/// that is not a 4-of-8 code, and rejects the control symbols RCN-217 §3
/// prohibits here. Reads identifier 3 as the two-symbol `app:info1`, which is
/// the only form that fits a channel-1 window.
pub fn parse_channel1(raw_bytes: &[u8]) -> Result<RailcomParseResult, ParseError> {
    let raw_bytes = strip_channel1_break_tail(raw_bytes);
    let symbols = decode_symbols::<2>(raw_bytes)?;
    // RCN-217 §3 prohibits control symbols in CH1. Preserve the raw code.
    for (symbol, &raw) in symbols.iter().zip(raw_bytes) {
        if symbol.is_control() {
            return Err(ParseError::ControlSymbolNotAllowed(raw));
        }
    }
    items_from_symbols(&symbols, RailcomChannel::Channel1)
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
            DecodedSymbol::Ack | DecodedSymbol::Nack | DecodedSymbol::Busy => {
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

    let expected_crc = dcc_protocol::logon::crc8_dallas_maxim(&bytes[..5]);
    let actual_crc = bytes[5];
    if expected_crc != actual_crc {
        return Err(ParseError::InvalidLogonCrc {
            expected: expected_crc,
            actual: actual_crc,
        });
    }

    // Not the two-byte address form [`DccAddress::from_address_fragments`]
    // reads: here bit 7 is the frame's own marker — required a few lines above,
    // and so set for short and long alike — which leaves magnitude as the only
    // thing that can tell the two apart in this frame.
    let raw_address = ((u16::from(bytes[0] & 0x3f)) << 8) | u16::from(bytes[1]);
    let address = DccAddress::from_magnitude(raw_address)
        .ok_or(ParseError::InvalidLogonAddress(raw_address))?;

    Ok(RailcomLogonResponse::Select(RailcomLogonSelect { address }))
}

/// The extended datagram: the one identifier whose size is not fixed.
const ID_APP_EXT: u8 = 3;

/// Symbols one datagram occupies, by the book, in the given receive window.
///
/// [`ID_APP_EXT`] is the only identifier whose size depends on the channel, and
/// the only one whose arms name the channel rather than matching any window.
/// RCN-217 gives it three symbols in channel 2, where it is `app:ext`. In
/// channel 1 it is `app:info1`, a different two-symbol datagram that happens to
/// share the identifier: the standard reserves that channel for the address
/// fragments, but decoders do send `app:info1` there and ZIMO's dissector reads
/// it as two symbols.
///
/// What a decoder really sends can still be shorter than the book says: see
/// [`resolve_symbol_count`].
fn datagram_symbol_count(id: u8, channel: RailcomChannel) -> Result<u8, ParseError> {
    match (id, channel) {
        (0..=2 | 14, _) => Ok(2),
        (ID_APP_EXT, RailcomChannel::Channel1) => Ok(2),
        (ID_APP_EXT, RailcomChannel::Channel2) => Ok(3),
        (7, _) => Ok(3),
        (8..=11, _) => Ok(6),
        _ => Err(ParseError::UnsupportedDatagramId(id)),
    }
}

/// Symbols this datagram actually occupies, allowing for the short `app:ext`.
///
/// An ESU LokPilot 3 is documented emitting `app:ext` in two symbols where
/// RCN-217 asks for three. OpenMRN fixes its length at two for that reason,
/// which in exchange misreads every conforming decoder. Shortening it only when
/// the third symbol carries no data — the window ends there, or that position
/// holds a control symbol — costs a conforming decoder nothing, because a
/// conforming `app:ext` always carries a data symbol in third position.
///
/// Only that one position is examined. A control symbol there does not prove
/// the rest of the window is padding; it proves only that the datagram cannot
/// be the conforming three-symbol form.
///
/// Two cases stay out of reach, both deliberately. A short `app:ext` with
/// another datagram behind it is indistinguishable from a conforming one: the
/// encoding carries no length, so nothing can tell them apart. That window is
/// lost whole, exactly as before — the three symbols read as one datagram and
/// what follows lands mid-datagram — so the rule below neither helps nor hurts
/// there. And a conforming `app:ext` whose third symbol was lost now reads as a
/// short one carrying a wrong value, where it used to fail: the corruption is
/// confined to the trailing datagram, and keeping the window preserves the CV
/// answer in front of it. Which width was read stays visible in the
/// [`ExtPayload`] variant, so a reader can weigh the value accordingly.
fn resolve_symbol_count(
    symbols: &[DecodedSymbol],
    index: usize,
    id: u8,
    channel: RailcomChannel,
) -> Result<u8, ParseError> {
    // `decode_symbols` has already rejected the reserved and invalid codes, so
    // these cases are the whole of what can stand in third position without
    // carrying data.
    let third_carries_no_data = symbols
        .get(index + 2)
        .is_none_or(|symbol| symbol.is_control());
    if id == ID_APP_EXT && channel == RailcomChannel::Channel2 && third_carries_no_data {
        return Ok(2);
    }
    datagram_symbol_count(id, channel)
}

/// Turns an identifier and its payload into the datagram it stands for.
///
/// The channel separates the two datagrams that share identifier 3: `app:ext`
/// in channel 2, `app:info1` in channel 1.
fn parse_datagram(
    id: u8,
    payload: u64,
    payload_bits: u8,
    channel: RailcomChannel,
) -> Result<RailcomDatagram, ParseError> {
    match (id, payload_bits, channel) {
        (0, 8, _) => Ok(RailcomDatagram::CvData(payload as u8)),
        (1, 8, _) => Ok(RailcomDatagram::AdrHigh(payload as u8)),
        (2, 8, _) => Ok(RailcomDatagram::AdrLow(payload as u8)),
        (ID_APP_EXT, 8, RailcomChannel::Channel1) => Ok(RailcomDatagram::Info1(payload as u8)),
        (ID_APP_EXT, 8, RailcomChannel::Channel2) => {
            Ok(RailcomDatagram::Ext(ExtPayload::Short(payload as u8)))
        }
        (ID_APP_EXT, 14, RailcomChannel::Channel2) => {
            Ok(RailcomDatagram::Ext(ExtPayload::Long(payload as u16)))
        }
        (7, 14, _) => Ok(RailcomDatagram::Dyn {
            value: (payload >> 6) as u8,
            sub_index: (payload & 0x3f) as u8,
        }),
        (8..=11, 32, _) => Ok(RailcomDatagram::Xpom {
            sequence: id - 8,
            values: [
                ((payload >> 24) & 0xff) as u8,
                ((payload >> 16) & 0xff) as u8,
                ((payload >> 8) & 0xff) as u8,
                (payload & 0xff) as u8,
            ],
        }),
        (14, 8, _) => Ok(RailcomDatagram::Search(payload as u8)),
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
        assert_eq!(decode_4_of_8(BUSY_CODE), DecodedSymbol::Busy);
        // The other two codes of that family stay reserved.
        for code in [RESERVED_1_CODE, RESERVED_2_CODE] {
            assert_eq!(decode_4_of_8(code), DecodedSymbol::Reserved(code));
        }
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

    fn short_ext(value: u8) -> RailcomItem {
        RailcomItem::Datagram(RailcomDatagram::Ext(ExtPayload::Short(value)))
    }

    fn conforming_ext(value: u16) -> RailcomItem {
        RailcomItem::Datagram(RailcomDatagram::Ext(ExtPayload::Long(value)))
    }

    fn info1(value: u8) -> RailcomItem {
        RailcomItem::Datagram(RailcomDatagram::Info1(value))
    }

    #[test]
    fn test_symbol_sizes_match_the_flat_table_except_identifier_3_in_channel1() {
        for id in 0u8..16 {
            let by_the_book = match id {
                0..=2 | 14 => Ok(2),
                3 | 7 => Ok(3),
                8..=11 => Ok(6),
                _ => Err(ParseError::UnsupportedDatagramId(id)),
            };
            assert_eq!(
                datagram_symbol_count(id, RailcomChannel::Channel2),
                by_the_book,
                "identifier {id} in channel 2"
            );
            let in_channel1 = if id == 3 { Ok(2) } else { by_the_book };
            assert_eq!(
                datagram_symbol_count(id, RailcomChannel::Channel1),
                in_channel1,
                "identifier {id} in channel 1"
            );
        }
    }

    #[test]
    fn test_parse_channel2_reads_busy_as_a_reply() {
        let parsed = parse_channel2(&[BUSY_CODE]).expect("busy is a reply, not corruption");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(parsed.items.as_slice(), &[RailcomItem::Busy]);
    }

    #[test]
    fn test_parse_channel2_keeps_the_cv_answer_before_busy() {
        // Busy used to be a reserved code, so a decoder saying it was busy read
        // as corruption and took the CV answer in front of it down with it.
        let cv = encode_12_bit_for_test(0, 0xab);
        let parsed =
            parse_channel2(&[cv[0], cv[1], BUSY_CODE]).expect("cv answer must survive busy");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[
                RailcomItem::Datagram(RailcomDatagram::CvData(0xab)),
                RailcomItem::Busy,
            ]
        );
    }

    #[test]
    fn test_parse_channel2_rejects_a_datagram_after_busy() {
        // RCN-217 §3 treats busy like the other control symbols: a response
        // that opens with one carries no datagrams.
        let cv = encode_12_bit_for_test(0, 0xab);

        assert_eq!(
            parse_channel2(&[BUSY_CODE, cv[0], cv[1]]),
            Err(ParseError::DatagramAfterControl)
        );
    }

    #[test]
    fn test_parse_channel1_accepts_the_two_symbol_identifier_3() {
        // Channel 1 holds two symbols, so the three-symbol channel-2 size
        // leaves no window in which the datagram can ever be read.
        let raw = encode_12_bit_for_test(3, 0x5a);
        let parsed = parse_channel1(&raw).expect("channel1 id3 parse must succeed");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(parsed.items.as_slice(), &[info1(0x5a)]);
    }

    #[test]
    fn test_parse_channel1_drops_the_break_tail_before_an_identifier_3() {
        // The break-frame strip and the channel-1 size must compose: a glitch
        // byte ahead of the datagram still leaves a readable window.
        let data = encode_12_bit_for_test(3, 0x5a);
        let parsed = parse_channel1(&[0xff, data[0], data[1]]).expect("break tail must be dropped");

        assert_eq!(parsed.items.as_slice(), &[info1(0x5a)]);
    }

    #[test]
    fn test_parse_channel2_keeps_the_conforming_identifier_3() {
        let raw = encode_18_bit_for_test(3, 0x3fff);
        let parsed = parse_channel2(&raw).expect("channel2 id3 parse must succeed");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(parsed.items.as_slice(), &[conforming_ext(0x3fff)]);
    }

    #[test]
    fn test_parse_channel2_accepts_a_short_identifier_3_closing_the_window() {
        let raw = encode_12_bit_for_test(3, 0xff);
        let parsed = parse_channel2(&raw).expect("short id3 must parse");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(parsed.items.as_slice(), &[short_ext(0xff)]);
    }

    #[test]
    fn test_parse_channel2_keeps_the_cv_answer_before_a_short_identifier_3() {
        // The case that costs data. RCN-217 lets the rest of the window carry
        // ACK padding, and a decoder may close with NACK, so these are the forms
        // really seen: all of them used to make the trailing datagram truncated,
        // and the truncation discarded the CV answer standing in front of it.
        let cv = encode_12_bit_for_test(0, 0xab);
        let ext = encode_12_bit_for_test(3, 0x5a);
        let padded: [(&[u8], &[RailcomItem]); 5] = [
            (&[], &[]),
            (&[ACK_1_CODE], &[RailcomItem::Ack]),
            (
                &[ACK_1_CODE, ACK_2_CODE],
                &[RailcomItem::Ack, RailcomItem::Ack],
            ),
            (&[NACK_CODE], &[RailcomItem::Nack]),
            (&[BUSY_CODE], &[RailcomItem::Busy]),
        ];
        for (padding, padding_items) in padded {
            let mut raw = Vec::<u8, 8>::new();
            raw.extend_from_slice(&cv).unwrap();
            raw.extend_from_slice(&ext).unwrap();
            raw.extend_from_slice(padding).unwrap();
            let parsed =
                parse_channel2(&raw).expect("cv answer must survive the short id3 and its padding");

            let mut expected = Vec::<RailcomItem, 6>::new();
            expected
                .push(RailcomItem::Datagram(RailcomDatagram::CvData(0xab)))
                .unwrap();
            expected.push(short_ext(0x5a)).unwrap();
            expected.extend_from_slice(padding_items).unwrap();

            assert_eq!(parsed.status, RailcomParseStatus::Complete);
            assert_eq!(parsed.items, expected, "padding {padding:?}");
        }
    }

    #[test]
    fn test_parse_channel2_keeps_the_length_when_the_third_symbol_carries_data() {
        // The shortening turns on the third symbol, not on the symbol count: a
        // data symbol in that position means a conforming datagram, whatever
        // stands behind it.
        let ext = encode_18_bit_for_test(3, 0x1234);
        let cv = encode_12_bit_for_test(0, 0xab);
        let raw = [ext[0], ext[1], ext[2], cv[0], cv[1]];
        let parsed = parse_channel2(&raw).expect("conforming id3 must keep its length");

        assert_eq!(parsed.status, RailcomParseStatus::Complete);
        assert_eq!(
            parsed.items.as_slice(),
            &[
                conforming_ext(0x1234),
                RailcomItem::Datagram(RailcomDatagram::CvData(0xab)),
            ]
        );
    }

    #[test]
    fn test_parse_channel2_still_loses_a_short_identifier_3_with_data_behind_it() {
        // The case the shortening cannot reach, pinned so nobody reads the rule
        // as wider than it is: with a datagram behind it, the short form is
        // indistinguishable from the conforming one, the three symbols read as
        // one datagram, and what follows lands mid-datagram. The window is lost
        // whole — exactly as it was before the rule existed.
        let ext = encode_12_bit_for_test(3, 0x5a);
        let cv = encode_12_bit_for_test(0, 0xab);
        let raw = [ext[0], ext[1], cv[0], cv[1]];

        assert!(parse_channel2(&raw).is_err());
    }

    #[test]
    fn test_parse_channel2_reports_a_one_symbol_identifier_3_against_the_short_size() {
        // The truncation is now measured against the size the window can really
        // hold, not against the conforming three.
        let ext = encode_12_bit_for_test(3, 0x5a);
        let error = parse_channel2(&ext[..1]).expect_err("one symbol cannot hold a datagram");

        assert_eq!(
            error,
            ParseError::TruncatedDatagram {
                id: 3,
                needed_symbols: 2,
                available_symbols: 1,
            }
        );
    }

    #[test]
    fn test_parse_channel2_reads_a_truncated_conforming_identifier_3_as_short() {
        // The exchange the shortening buys, pinned down so that changing it is
        // a deliberate act: a conforming datagram that lost its third symbol
        // reads as a short one carrying a wrong value, where it used to fail.
        // Only the trailing datagram is affected, and `payload_bits` says which
        // width was read.
        let conforming = encode_18_bit_for_test(3, 0x1234);
        let parsed =
            parse_channel2(&conforming[..2]).expect("a lost third symbol reads as the short form");

        assert_eq!(parsed.items.as_slice(), &[short_ext(0x48)]);
    }

    #[test]
    fn test_parse_channel2_reports_a_control_symbol_inside_a_datagram_as_such() {
        // The shortening can put a control symbol in second position, where the
        // datagram wanted data. That is not a truncation, and reporting it as
        // one would print a window that needs two symbols and has two.
        let ext = encode_12_bit_for_test(3, 0x5a);
        for code in [ACK_1_CODE, ACK_2_CODE, NACK_CODE, BUSY_CODE] {
            let error = parse_channel2(&[ext[0], code])
                .expect_err("a control symbol inside a datagram must not parse");

            assert_eq!(
                error,
                ParseError::ControlSymbolInsideDatagram { id: 3, offset: 1 },
                "control code {code:#04x}"
            );
        }
    }

    #[test]
    fn test_every_size_the_table_gives_has_a_datagram_to_parse() {
        // The two tables must agree: a size the first hands out with no payload
        // width to match in the second turns a readable window into a hard
        // error instead of a partial result.
        for channel in [RailcomChannel::Channel1, RailcomChannel::Channel2] {
            for id in 0u8..16 {
                let Ok(symbol_count) = datagram_symbol_count(id, channel) else {
                    continue;
                };
                let payload_bits = symbol_count * 6 - 4;
                assert!(
                    parse_datagram(id, 0, payload_bits, channel).is_ok(),
                    "identifier {id} in {channel:?} has no datagram at {payload_bits} bits"
                );
            }
        }
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
        for code in [ACK_1_CODE, ACK_2_CODE, NACK_CODE, BUSY_CODE] {
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
    fn test_parse_logon_response_rejects_control_symbols_anywhere() {
        // The 48-bit logon answer is all data. Its own decode path never met a
        // test for the control symbols it rejects.
        let valid = encode_48_bit_for_test(15, 0x0abc_1234_5678);
        for code in [ACK_1_CODE, ACK_2_CODE, NACK_CODE, BUSY_CODE] {
            for position in 0..8 {
                let mut raw = valid;
                raw[position] = code;

                assert_eq!(
                    parse_logon_response_48(&raw[..2], &raw[2..]),
                    Err(ParseError::ControlSymbolNotAllowed(code)),
                    "control code {code:#04x} at position {position}"
                );
            }
        }
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
