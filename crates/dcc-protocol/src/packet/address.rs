//! Locomotive decoder addresses and the high byte that carries them.
//!
//! [`DccAddress`] holds a validated NMRA S-9.2 short (1-127) or long
//! (128-10239) address; [`HighFragment`] reads the flag bits of the first
//! byte of a two-byte address.
//!
//! `new_short`, `new_long`, `is_short` and `value` are `#[inline(always)]`:
//! the firmware's RMT and cutout interrupt handlers reach them, and those
//! handlers run from RAM, so the helpers must not land in flash as separate
//! functions. `scripts/check-isr-ram.sh` checks the linked firmware for it.

use heapless::Vec;

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
    Short,
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
    // Forced inline: reached from the firmware's RMT and cutout handlers,
    // which run from RAM, through its packed address helpers.
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
    // Forced inline: reached from the firmware's RMT and cutout handlers,
    // which run from RAM, through its packed address helpers.
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

    /// Create an address from the two bytes that carry one on the wire.
    ///
    /// The high byte is read by [`HighFragment`], and its form decides: a
    /// stated long address is taken at its word, a plain fragment leaves the
    /// mode to magnitude, and a byte of no known form yields nothing.
    ///
    /// Returns `None` for a malformed high byte, for zero, for a value above
    /// 10239, and for a stated long address below 128 — legal on the wire, but
    /// outside what this type can hold. Reading that last one as the short
    /// address of the same number would name a different decoder.
    ///
    /// This is not the Z21 wire rule in the `z21` crate's `parse_loco_address`, which
    /// reads the same two bytes for the host protocol: there any value below
    /// 128 is a short address whatever the flag bits say, because the Z21
    /// clients encode it that way.
    #[must_use]
    pub fn from_address_fragments(high: u8, low: u8) -> Option<Self> {
        match HighFragment::read(high) {
            HighFragment::Long(value) => Self::new_long(HighFragment::compose(value, low)),
            HighFragment::Plain(value) => Self::from_magnitude(HighFragment::compose(value, low)),
            HighFragment::Malformed => None,
        }
    }

    /// Returns `true` if this is a short address
    // Forced inline: reached from the firmware's RMT and cutout handlers,
    // which run from RAM, through its packed address helpers.
    #[inline(always)]
    pub fn is_short(&self) -> bool {
        matches!(self.kind, AddressKind::Short(_))
    }

    /// Returns `true` if this is a long address
    pub fn is_long(&self) -> bool {
        matches!(self.kind, AddressKind::Long(_))
    }

    /// Returns the addressing mode (short vs long) as a typed enum.
    pub fn kind(&self) -> DccAddressKind {
        match self.kind {
            AddressKind::Short(_) => DccAddressKind::Short,
            AddressKind::Long(_) => DccAddressKind::Long,
        }
    }

    /// Returns the numeric address value
    // Forced inline: reached from the firmware's RMT and cutout handlers,
    // which run from RAM, through its packed address helpers.
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
                let high = HighFragment::FLAG_MASK | ((addr >> 8) as u8);
                let low = (addr & 0xFF) as u8;
                Vec::from_array([high, low])
            }
        }
    }
}

/// The high byte of a two-byte address, read by its flag bits.
///
/// A long address travels as `0b11AAAAAA AAAAAAAA`; a short one is carried in
/// the low byte alone, behind a zero high byte. The two flag bits are read
/// together, because that is the pattern the standard defines and the only one
/// this crate emits: a byte carrying just one of them belongs to neither form.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HighFragment {
    /// Both flag bits set: the fragment states a long address.
    Long(u8),
    /// Neither flag bit set: six value bits and no statement about the mode.
    Plain(u8),
    /// One flag bit alone: a byte of no known form.
    Malformed,
}

impl HighFragment {
    const FLAG_MASK: u8 = 0b1100_0000;
    const VALUE_MASK: u8 = 0b0011_1111;

    /// Reads the flag bits of a high address byte and tells which form it has.
    #[must_use]
    pub fn read(high: u8) -> Self {
        match high & Self::FLAG_MASK {
            Self::FLAG_MASK => Self::Long(high & Self::VALUE_MASK),
            0 => Self::Plain(high & Self::VALUE_MASK),
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
    /// when the fragment has no known form.
    #[must_use]
    pub fn address_value(self, low: u8) -> Option<u16> {
        match self {
            Self::Long(value) | Self::Plain(value) => Some(Self::compose(value, low)),
            Self::Malformed => None,
        }
    }
}
