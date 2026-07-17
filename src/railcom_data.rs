//! Neutral RailCom feedback data shared across parser, DCC POM logic, and
//! runtime dispatch.
//!
//! This module intentionally contains data contracts only. Keeping these types
//! outside `railcom::parser` lets DCC code consume decoded feedback without
//! depending on the parser implementation module.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomDatagram {
    CvData(u8),
    AdrHigh(u8),
    AdrLow(u8),
    Ext(u16),
    Dyn { value: u8, sub_index: u8 },
    Xpom { sequence: u8, values: [u8; 4] },
    Search(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum RailcomItem {
    Ack,
    Nack,
    Datagram(RailcomDatagram),
}
