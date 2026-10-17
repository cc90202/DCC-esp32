//! Mapping from locomotive functions F0..=F28 to NMRA function groups.
//!
//! A locomotive's functions are held as one `u32` with bit `n` set when `Fn`
//! is on. Each NMRA function group (S-9.2.1 Function Group One and Two, and
//! the binary state control long forms 0xDE and 0xDF) carries a contiguous
//! run of those bits in its data byte. [`GROUP_LAYOUTS`] is the one place
//! that run is written down; the packet encoder and the firmware's slot
//! scheduler both read it.

/// NMRA function group carried by one multi-function decoder packet.
///
/// The discriminant is the group's position in `GROUP_LAYOUTS`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FunctionGroupKind {
    /// Function Group One: FL (F0) and F1..=F4, instruction `100DDDDD`.
    G1 = 0,
    /// Function Group Two, F5..=F8, instruction `1011DDDD`.
    G2a = 1,
    /// Function Group Two, F9..=F12, instruction `1010DDDD`.
    G2b = 2,
    /// Binary state control F13..=F20, instruction `0xDE` + data byte.
    G3 = 3,
    /// Binary state control F21..=F28, instruction `0xDF` + data byte.
    G4 = 4,
}

impl FunctionGroupKind {
    /// Position of this group in [`GROUP_LAYOUTS`], also used as its bit in
    /// per-slot group masks.
    #[must_use]
    pub const fn index(self) -> u8 {
        self as u8
    }

    /// Data bits this group can carry; the rest of the byte is opcode.
    #[must_use]
    pub(crate) const fn data_mask(self) -> u8 {
        match self {
            Self::G1 => 0b0001_1111,
            Self::G2a | Self::G2b => 0b0000_1111,
            Self::G3 | Self::G4 => 0b1111_1111,
        }
    }
}

/// Where one function group's functions sit in the `u32` function word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GroupLayout {
    pub kind: FunctionGroupKind,
    /// Function-word bits carried by this group.
    pub mask: u32,
    /// Right shift that moves the group's lowest numbered function to data
    /// bit 0. For G1 that is F1: F0 (FL) is not part of the shifted run and
    /// is placed in data bit 4 instead.
    pub shift: u8,
}

/// Every function group in transmission order, indexed by
/// [`FunctionGroupKind::index`].
pub const GROUP_LAYOUTS: [GroupLayout; 5] = [
    GroupLayout {
        kind: FunctionGroupKind::G1,
        mask: 0x0000_001F, // F0..=F4
        shift: 1,
    },
    GroupLayout {
        kind: FunctionGroupKind::G2a,
        mask: 0x0000_01E0, // F5..=F8
        shift: 5,
    },
    GroupLayout {
        kind: FunctionGroupKind::G2b,
        mask: 0x0000_1E00, // F9..=F12
        shift: 9,
    },
    GroupLayout {
        kind: FunctionGroupKind::G3,
        mask: 0x001F_E000, // F13..=F20
        shift: 13,
    },
    GroupLayout {
        kind: FunctionGroupKind::G4,
        mask: 0x1FE0_0000, // F21..=F28
        shift: 21,
    },
];

/// Group that carries function `Fn`.
///
/// Callers pass a validated function number (0..=28); anything above F28 is
/// folded into G4, which carries no bit for it.
#[must_use]
pub const fn group_of(function: u8) -> FunctionGroupKind {
    let bit = match 1u32.checked_shl(function as u32) {
        Some(bit) => bit,
        None => 0,
    };
    let mut index = 0;
    while index < GROUP_LAYOUTS.len() {
        if GROUP_LAYOUTS[index].mask & bit != 0 {
            return GROUP_LAYOUTS[index].kind;
        }
        index += 1;
    }
    FunctionGroupKind::G4
}

/// Data bits of group `kind` for the function word `functions`.
///
/// F0 lands in bit 4 of G1 (NMRA FL position); every other function lands at
/// its offset from the group's first function.
#[must_use]
pub const fn group_bits(functions: u32, kind: FunctionGroupKind) -> u8 {
    let layout = GROUP_LAYOUTS[kind.index() as usize];
    let run = ((functions & layout.mask) >> layout.shift) as u8;
    match kind {
        FunctionGroupKind::G1 => run | (((functions & 1) as u8) << 4),
        _ => run,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layouts_are_indexed_by_kind_and_tile_f0_to_f28() {
        let mut covered = 0u32;
        for (index, layout) in GROUP_LAYOUTS.iter().enumerate() {
            assert_eq!(usize::from(layout.kind.index()), index);
            assert_eq!(covered & layout.mask, 0, "{:?} overlaps", layout.kind);
            covered |= layout.mask;
        }
        assert_eq!(covered, (1 << 29) - 1);
    }

    #[test]
    fn group_of_folds_functions_above_f28_into_g4() {
        for function in [29, 31, 32, u8::MAX] {
            assert_eq!(group_of(function), FunctionGroupKind::G4, "F{function}");
        }
    }

    #[test]
    fn group_bits_ignores_functions_of_other_groups() {
        let all = (1u32 << 29) - 1;
        for layout in GROUP_LAYOUTS {
            let others = all & !layout.mask;
            assert_eq!(group_bits(others, layout.kind), 0, "{:?}", layout.kind);
            assert_eq!(
                group_bits(all, layout.kind),
                layout.kind.data_mask(),
                "{:?}",
                layout.kind
            );
        }
    }
}
