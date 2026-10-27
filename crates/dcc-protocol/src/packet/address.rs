//! Locomotive decoder addresses and the high byte that carries them.
//!
//! [`DccAddress`] holds a validated NMRA S-9.2 short (1-127) or long
//! (128-10239) address; [`HighFragment`] reads the RailCom `ADR1` datagram
//! that precedes the low byte of an address.
//!
//! `new_short`, `new_long`, `is_short` and `value` are `#[inline(always)]`:
//! these helpers must inline into their callers, which may run from RAM and
//! cannot afford a call into a separately placed function.

use heapless::Vec;

/// Flag bits of the first byte of a long address in a DCC packet (NMRA S-9.2.1).
const LONG_ADDRESS_PREFIX: u8 = 0b1100_0000;

/// DCC decoder address (opaque type enforcing NMRA address validation)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DccAddress {
    kind: AddressKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum AddressKind {
    Short(u8),
    Long(u16),
}

/// Public view of which NMRA S-9.2 addressing mode a `DccAddress` uses.
///
/// Exposed separately from the private `AddressKind` enum so the on-wire
/// representation (`Short(u8)` / `Long(u16)`) stays encapsulated while callers
/// can still pattern-match on the addressing mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DccAddressKind {
    /// One-byte short address.
    Short,
    /// Two-byte long address.
    Long,
}

impl DccAddress {
    /// Highest short address; short addresses run from 1 to this value.
    pub const SHORT_MAX: u16 = 127;
    /// Lowest long address, one above [`Self::SHORT_MAX`].
    pub const LONG_MIN: u16 = 128;
    /// Highest long address NMRA S-9.2 assigns.
    pub const LONG_MAX: u16 = 10_239;

    /// Create a new short address (1-127)
    ///
    /// Returns `None` if address is 0 or > 127
    // Forced inline: see the module documentation.
    #[inline(always)]
    pub fn new_short(addr: u8) -> Option<Self> {
        if addr == 0 || u16::from(addr) > Self::SHORT_MAX {
            None
        } else {
            Some(Self {
                kind: AddressKind::Short(addr),
            })
        }
    }

    /// Create a new long address (128-10239)
    ///
    /// Returns `None` if address is < 128 or > 10239
    // Forced inline: see the module documentation.
    #[inline(always)]
    pub fn new_long(addr: u16) -> Option<Self> {
        if !(Self::LONG_MIN..=Self::LONG_MAX).contains(&addr) {
            None
        } else {
            Some(Self {
                kind: AddressKind::Long(addr),
            })
        }
    }

    /// Create an address from its plain numeric magnitude (1-127 short,
    /// 128-10239 long), picking the addressing mode implied by NMRA's
    /// non-overlapping short/long ranges.
    ///
    /// Returns `None` for 0 or values above 10239.
    ///
    /// This differs from the Z21 wire decoding in the `z21` crate's `parse_loco_address`,
    /// which reads a 2-byte on-wire field where the addressing mode is tagged by
    /// the top two bits of the high byte rather than inferred from magnitude:
    /// a wire long-address encoding could in principle carry a value that also
    /// fits the short range. `from_magnitude` is for callers that only have a
    /// plain numeric address (e.g. a RailCom-reported address) and want the
    /// same short/long split NMRA uses.
    #[must_use]
    pub fn from_magnitude(addr: u16) -> Option<Self> {
        match addr {
            short @ 1..=Self::SHORT_MAX => Self::new_short(short as u8),
            long @ Self::LONG_MIN..=Self::LONG_MAX => Self::new_long(long),
            _ => None,
        }
    }

    /// Create an address from the RailCom `ADR1` and `ADR2` datagrams.
    ///
    /// The high byte is read by [`HighFragment`], and its form decides: a base
    /// or consist address is short, an extended address is long, and a byte of
    /// no known form yields nothing. A consist address is the one the decoder
    /// currently answers to, so it is returned as the short address it is on
    /// the track.
    ///
    /// Returns `None` for a malformed high byte, for zero, for a value above
    /// 10239, and for an extended address below 128 — legal on the wire, but
    /// outside what this type can hold. Reading that last one as the short
    /// address of the same number would name a different decoder.
    ///
    /// This is not the Z21 wire rule in the `z21` crate's `parse_loco_address`, which
    /// reads the same two bytes for the host protocol: there any value below
    /// 128 is a short address whatever the flag bits say, because the Z21
    /// clients encode it that way.
    #[must_use]
    pub fn from_address_fragments(high: u8, low: u8) -> Option<Self> {
        let fragment = HighFragment::read(high);
        let value = fragment.address_value(low)?;
        match fragment {
            HighFragment::Long(_) => Self::new_long(value),
            HighFragment::Short | HighFragment::Consist => {
                Self::new_short(u8::try_from(value).ok()?)
            }
            HighFragment::Malformed => None,
        }
    }

    /// Returns `true` if this is a short address
    // Forced inline: see the module documentation.
    #[inline(always)]
    pub fn is_short(&self) -> bool {
        matches!(self.kind, AddressKind::Short(_))
    }

    /// Returns `true` if this is a long address
    pub fn is_long(&self) -> bool {
        matches!(self.kind, AddressKind::Long(_))
    }

    /// Returns the addressing mode (short vs long) as a typed enum.
    #[must_use]
    pub fn kind(&self) -> DccAddressKind {
        match self.kind {
            AddressKind::Short(_) => DccAddressKind::Short,
            AddressKind::Long(_) => DccAddressKind::Long,
        }
    }

    /// Returns the numeric address value
    // Forced inline: see the module documentation.
    #[inline(always)]
    pub fn value(&self) -> u16 {
        match self.kind {
            AddressKind::Short(addr) => u16::from(addr),
            AddressKind::Long(addr) => addr,
        }
    }

    /// Encode address to bytes according to NMRA S-9.2
    pub(super) fn to_bytes(self) -> Vec<u8, 2> {
        match self.kind {
            AddressKind::Short(addr) => Vec::from_array([addr]),
            AddressKind::Long(addr) => {
                // Long address format: 11AAAAAA AAAAAAAA
                // Top 6 bits in first byte (with 0xC0 prefix)
                // Bottom 8 bits in second byte
                let high = LONG_ADDRESS_PREFIX | ((addr >> 8) as u8);
                let low = (addr & 0xFF) as u8;
                Vec::from_array([high, low])
            }
        }
    }
}

/// The RailCom `ADR1` datagram (ID 1), read by its form.
///
/// RCN-217 table 11 gives it three forms: `00000000` before a base (short)
/// address, `01100000` before a consist address, and `10AAAAAA` carrying the
/// top six bits of an extended (long) address. Every other byte is malformed.
/// This is not the long-address byte of a DCC packet, which is `11AAAAAA`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HighFragment {
    /// `00000000`: `ADR2` holds a base address, 1 to 127.
    Short,
    /// `01100000`: `ADR2` holds a consist address in its low seven bits.
    Consist,
    /// `10AAAAAA`: the top six bits of an extended address.
    Long(u8),
    /// Any other byte.
    Malformed,
}

impl HighFragment {
    const SHORT: u8 = 0b0000_0000;
    const CONSIST: u8 = 0b0110_0000;
    const LONG_MARKER_MASK: u8 = 0b1100_0000;
    const LONG_MARKER: u8 = 0b1000_0000;
    const VALUE_MASK: u8 = 0b0011_1111;
    /// Bit 7 of `ADR2` after a consist marker is the direction bit `R`.
    const CONSIST_ADDRESS_MASK: u8 = 0b0111_1111;

    /// Reads an `ADR1` byte and tells which form it has.
    #[must_use]
    pub fn read(high: u8) -> Self {
        match high {
            Self::SHORT => Self::Short,
            Self::CONSIST => Self::Consist,
            long if long & Self::LONG_MARKER_MASK == Self::LONG_MARKER => {
                Self::Long(long & Self::VALUE_MASK)
            }
            _ => Self::Malformed,
        }
    }

    /// The six value bits an address puts in its high fragment.
    #[must_use]
    pub fn value_bits_of(address: DccAddress) -> u8 {
        ((address.value() >> 8) as u8) & Self::VALUE_MASK
    }

    const fn compose(value: u8, low: u8) -> u16 {
        ((value as u16) << 8) | low as u16
    }

    /// The address value this fragment carries together with `low`, or `None`
    /// when the fragment is malformed or `low` does not fit its form.
    #[must_use]
    pub fn address_value(self, low: u8) -> Option<u16> {
        match self {
            Self::Short => (low <= Self::CONSIST_ADDRESS_MASK).then_some(u16::from(low)),
            Self::Consist => Some(u16::from(low & Self::CONSIST_ADDRESS_MASK)),
            Self::Long(value) => Some(Self::compose(value, low)),
            Self::Malformed => None,
        }
    }
}
