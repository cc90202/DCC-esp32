//! Locomotive functions F0..=F28: a validated index and the on/off state of
//! all of them.

/// Function index mapping:
/// - 0 => FL
/// - 1..=28 => F1..F28
const MAX_FUNCTION_INDEX: u8 = 28;

/// Validated function index (F0..F28).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FunctionIndex(u8);

/// Error returned when function index is out of range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidFunctionIndex;

impl ::core::fmt::Display for InvalidFunctionIndex {
    fn fmt(&self, formatter: &mut ::core::fmt::Formatter<'_>) -> ::core::fmt::Result {
        formatter.write_str("function index must be in the F0..=F28 range")
    }
}

impl ::core::error::Error for InvalidFunctionIndex {}

impl FunctionIndex {
    /// Create a validated function index in range 0..=28.
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(transparent)]
pub struct FunctionState(u32);

impl FunctionState {
    const VALID_BITS: u32 = 0x1fff_ffff;

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
        Self::new(value).ok_or(InvalidFunctionIndex)
    }
}
