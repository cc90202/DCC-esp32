//! Compact address encodings for atomics and ISR-side metadata.

use super::DccAddress;

/// Compact crate-internal representation for atomics and ISR metadata.
///
/// Raw layout:
/// - `0` = no address
/// - `1..=127` = short address
/// - `0x8000 | value` = long address, with `value` in `128..=10239`
#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct PackedDccAddress(u16);

#[cfg(any(test, target_arch = "riscv32"))]
impl PackedDccAddress {
    const LONG_TAG: u16 = 0x8000;
    const LONG_VALUE_MASK: u16 = 0x3fff;

    pub(crate) const fn from_raw(raw: u16) -> Self {
        Self(raw)
    }

    pub(crate) const fn raw(self) -> u16 {
        self.0
    }

    pub(crate) fn from_address(address: Option<DccAddress>) -> Self {
        match address {
            Some(address) if address.is_short() => Self(address.value()),
            Some(address) => Self(Self::LONG_TAG | address.value()),
            None => Self(0),
        }
    }

    pub(crate) fn address(self) -> Option<DccAddress> {
        if self.0 == 0 {
            return None;
        }

        if (self.0 & Self::LONG_TAG) != 0 {
            DccAddress::new_long(self.0 & Self::LONG_VALUE_MASK)
        } else {
            let value = u8::try_from(self.0).ok()?;
            DccAddress::new_short(value)
        }
    }
}

/// Shared bit layout for packing a [`PackedDccAddress`] together with a
/// handful of boolean flags into a single `u32` for atomic-word metadata
/// side channels.
///
/// The address occupies bits `0..=15` (see [`PackedDccAddress`]); flag `n`
/// (0-based) occupies bit `16 + n`. This is the common backing storage for
/// `track_output::PackedRailcomPacketMetadata`, which records the POM mode and
/// whether a separately stored request identifier is present.
#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PackedAddressFlags(u32);

#[cfg(any(test, target_arch = "riscv32"))]
impl PackedAddressFlags {
    const ADDRESS_MASK: u32 = 0xffff;
    const FLAG_BASE_BIT: u32 = 16;

    /// Called from ISR-resident code (`track_output` metadata capture); the
    /// inline attributes keep these helpers out of flash on non-LTO builds.
    #[inline(always)]
    pub(crate) fn new(address: Option<DccAddress>) -> Self {
        Self(u32::from(PackedDccAddress::from_address(address).raw()))
    }

    #[inline(always)]
    pub(crate) const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }

    #[inline(always)]
    pub(crate) const fn raw(self) -> u32 {
        self.0
    }

    pub(crate) fn address(self) -> Option<DccAddress> {
        PackedDccAddress::from_raw((self.0 & Self::ADDRESS_MASK) as u16).address()
    }

    /// Sets flag `index` (0-based, added to [`Self::FLAG_BASE_BIT`]) when
    /// `value` is true; leaves the word unchanged otherwise.
    #[must_use]
    #[inline(always)]
    pub(crate) fn with_flag(self, index: u32, value: bool) -> Self {
        if value {
            Self(self.0 | (1 << (Self::FLAG_BASE_BIT + index)))
        } else {
            self
        }
    }

    #[inline(always)]
    pub(crate) const fn flag(self, index: u32) -> bool {
        (self.0 & (1 << (Self::FLAG_BASE_BIT + index))) != 0
    }

    /// Replaces a compact field starting at flag `index`.
    #[must_use]
    #[inline(always)]
    pub(crate) const fn with_field(self, index: u32, width: u32, value: u32) -> Self {
        let field_mask = (1u32 << width) - 1;
        let shifted_mask = field_mask << (Self::FLAG_BASE_BIT + index);
        Self((self.0 & !shifted_mask) | ((value & field_mask) << (Self::FLAG_BASE_BIT + index)))
    }

    #[inline(always)]
    pub(crate) const fn field(self, index: u32, width: u32) -> u32 {
        let field_mask = (1u32 << width) - 1;
        (self.0 >> (Self::FLAG_BASE_BIT + index)) & field_mask
    }
}
