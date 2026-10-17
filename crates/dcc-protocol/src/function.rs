//! Locomotive functions F0..=F28: a validated index and the on/off state of
//! all of them.

/// Highest function index the command station handles.
const MAX_FUNCTION_INDEX: u8 = 28;

/// Validated function index (F0..F28).
///
/// Index 0 is F0, the headlight (FL); index `n` above it is `Fn`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FunctionIndex(u8);

/// Error returned when a function index is above F28; carries the rejected
/// value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidFunctionIndex(u8);

impl InvalidFunctionIndex {
    /// The function index that was rejected.
    #[must_use]
    pub const fn value(self) -> u8 {
        self.0
    }
}

impl ::core::fmt::Display for InvalidFunctionIndex {
    fn fmt(&self, formatter: &mut ::core::fmt::Formatter<'_>) -> ::core::fmt::Result {
        write!(
            formatter,
            "function index {} is outside the F0..=F{MAX_FUNCTION_INDEX} range",
            self.0
        )
    }
}

impl ::core::error::Error for InvalidFunctionIndex {}

impl FunctionIndex {
    /// Create a validated function index; `None` above F28.
    #[must_use]
    pub fn new(value: u8) -> Option<Self> {
        (value <= MAX_FUNCTION_INDEX).then_some(Self(value))
    }

    /// Return the raw function index value.
    #[must_use]
    pub const fn get(self) -> u8 {
        self.0
    }
}

/// State of locomotive functions F0 through F28.
///
/// Bit `n` is set when function `Fn` is on; the bits above F28 are always
/// clear.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(transparent)]
pub struct FunctionState(u32);

impl FunctionState {
    /// One bit per function F0..=F28.
    const VALID_BITS: u32 = (1 << (MAX_FUNCTION_INDEX + 1)) - 1;

    /// All functions off.
    #[must_use]
    pub const fn empty() -> Self {
        Self(0)
    }

    /// Build a function state when no bits above F28 are set.
    ///
    /// Only for tests (here, and in other crates through the `test-support`
    /// feature); production code builds the state with `from_bits_truncate`.
    #[cfg(any(test, feature = "test-support"))]
    #[must_use]
    pub const fn from_bits(bits: u32) -> Option<Self> {
        if bits & !Self::VALID_BITS == 0 {
            Some(Self(bits))
        } else {
            None
        }
    }

    /// Build a function state while explicitly discarding bits above F28.
    #[must_use]
    pub const fn from_bits_truncate(bits: u32) -> Self {
        Self(bits & Self::VALID_BITS)
    }

    /// The function word: bit `n` set when `Fn` is on.
    #[must_use]
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// Whether `function` is on in this state.
    #[must_use]
    pub const fn is_enabled(self, function: FunctionIndex) -> bool {
        self.0 & (1u32 << function.get()) != 0
    }
}

impl TryFrom<u8> for FunctionIndex {
    type Error = InvalidFunctionIndex;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Self::new(value).ok_or(InvalidFunctionIndex(value))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_function_index_validation() {
        assert_eq!(FunctionIndex::new(0).map(FunctionIndex::get), Some(0));
        assert_eq!(FunctionIndex::new(28).map(FunctionIndex::get), Some(28));
        assert_eq!(FunctionIndex::new(29), None);
        assert_eq!(FunctionIndex::try_from(12).map(FunctionIndex::get), Ok(12));
        let error = FunctionIndex::try_from(40).expect_err("F40 is out of range");
        assert_eq!(error.value(), 40);
    }

    #[test]
    fn function_state_rejects_reserved_bits_unless_truncation_is_explicit() {
        const VALID_BITS: u32 = 0x1fff_ffff;

        assert_eq!(
            FunctionState::from_bits(VALID_BITS),
            Some(FunctionState::from_bits_truncate(VALID_BITS))
        );
        assert_eq!(FunctionState::from_bits(1 << 29), None);
        assert_eq!(
            FunctionState::from_bits_truncate(VALID_BITS | (1 << 31)).bits(),
            VALID_BITS
        );
    }
}
