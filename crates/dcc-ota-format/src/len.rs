//! Image lengths, checked once where they enter.

/// Length in bytes of an application image: at least one flash sector, so
/// the first sector holds the whole image prefix, at most one application
/// slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ImageLen(u32);

/// One 4 KiB flash sector.
const MIN_LEN: u32 = 0x1000;
/// One 3 MiB application slot.
const MAX_LEN: u32 = 0x30_0000;

impl ImageLen {
    /// One 4 KiB flash sector.
    pub const MIN: Self = Self(MIN_LEN);
    /// One 3 MiB application slot.
    pub const MAX: Self = Self(MAX_LEN);

    /// `len` bytes, if between [`Self::MIN`] and [`Self::MAX`].
    #[must_use]
    pub const fn new(len: u32) -> Option<Self> {
        match len {
            MIN_LEN..=MAX_LEN => Some(Self(len)),
            _ => None,
        }
    }

    /// The length of `bytes`, if it is a possible image length.
    #[must_use]
    pub fn of(bytes: &[u8]) -> Option<Self> {
        u32::try_from(bytes.len()).ok().and_then(Self::new)
    }

    /// The length in bytes.
    #[must_use]
    pub const fn get(self) -> u32 {
        self.0
    }

    /// The length as a buffer size.
    #[must_use]
    pub const fn as_usize(self) -> usize {
        self.0 as usize
    }
}

impl core::fmt::Display for ImageLen {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}
