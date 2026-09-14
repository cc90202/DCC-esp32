//! Neutral RailCom feedback data shared across parser, DCC POM logic, and
//! runtime dispatch.
//!
//! This module intentionally contains data contracts only. Keeping these types
//! outside `railcom::parser` lets DCC code consume decoded feedback without
//! depending on the parser implementation module.

/// Payload of an `app:ext` datagram, which comes in two widths.
///
/// RCN-217 gives the datagram three symbols and so 14 payload bits. Some
/// decoders emit it in two symbols, leaving 8. Carrying the width as the
/// variant keeps a short payload from ever holding a long value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum ExtPayload {
    /// The two-symbol form, 8 payload bits.
    Short(u8),
    /// The conforming three-symbol form, 14 payload bits.
    Long(u16),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomDatagram {
    CvData(u8),
    AdrHigh(u8),
    AdrLow(u8),
    /// `app:ext`, channel 2 only.
    Ext(ExtPayload),
    /// `app:info1`: identifier 3 as it appears in channel 1. ZIMO's dissector
    /// reads it there as a two-symbol datagram of its own, not as a narrow
    /// `app:ext`, and this crate keeps the two apart rather than merging them
    /// on the strength of a shared identifier.
    Info1(u8),
    Dyn {
        value: u8,
        sub_index: u8,
    },
    Xpom {
        sequence: u8,
        values: [u8; 4],
    },
    Search(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomItem {
    Ack,
    Nack,
    Datagram(RailcomDatagram),
}
