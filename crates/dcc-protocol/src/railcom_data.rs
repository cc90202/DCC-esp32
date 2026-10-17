//! Neutral RailCom feedback data shared across parser, DCC POM logic, and
//! runtime dispatch.
//!
//! This module intentionally contains data contracts only. Keeping these types
//! outside the firmware's RailCom parser lets DCC code consume decoded feedback
//! without depending on the parser implementation.

/// Payload of an `app:ext` datagram, which comes in two widths.
///
/// RCN-217 gives the datagram three symbols and so 14 payload bits. Some
/// decoders emit it in two symbols, leaving 8. Carrying the width as the
/// variant keeps a short payload from ever holding a long value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ExtPayload {
    /// The two-symbol form, 8 payload bits.
    Short(u8),
    /// The conforming three-symbol form, 14 payload bits.
    Long(u16),
}

/// One decoded RailCom datagram (RCN-217), by identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RailcomDatagram {
    /// `app:pom` (identifier 0): the CV value answering a POM read or write.
    CvData(u8),
    /// `app:adr_high` (identifier 1): the high part of the decoder address.
    AdrHigh(u8),
    /// `app:adr_low` (identifier 2): the low part of the decoder address.
    AdrLow(u8),
    /// `app:ext`, channel 2 only.
    Ext(ExtPayload),
    /// `app:info1`: identifier 3 as it appears in channel 1. ZIMO's dissector
    /// reads it there as a two-symbol datagram of its own, not as a narrow
    /// `app:ext`, and this crate keeps the two apart rather than merging them
    /// on the strength of a shared identifier.
    Info1(u8),
    /// `app:dyn` (identifier 7): one dynamic variable.
    Dyn {
        /// The variable's value.
        value: u8,
        /// Which dynamic variable the value belongs to.
        sub_index: u8,
    },
    /// `app:xpom` (identifiers 8..=11): four CV values of an XPOM read.
    Xpom {
        /// Position of the datagram in the XPOM sequence, from 0.
        sequence: u8,
        /// The CV values, in wire order.
        values: [u8; 4],
    },
    /// `app:search` (identifier 14), the answer to a track search.
    Search(u8),
}

/// One item read from a RailCom cutout: an acknowledgement code or a
/// datagram.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RailcomItem {
    /// The decoder acknowledged the packet.
    Ack,
    /// The decoder refused the packet.
    Nack,
    /// The decoder cannot answer right now. NMRA S-9.3.2 gives this its own
    /// code; it is neither an acknowledgement nor a refusal, and it says
    /// nothing about whether the command was accepted.
    Busy,
    /// A decoded datagram.
    Datagram(RailcomDatagram),
}
