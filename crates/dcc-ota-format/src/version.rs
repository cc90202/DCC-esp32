//! Release versions: three numeric components, compared in order.

/// Major, minor and patch release numbers, ordered lexicographically.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Version([u16; 3]);

impl Version {
    /// Size of [`Self::to_le_bytes`].
    pub const ENCODED_LEN: usize = 6;

    /// Builds a version from its three components.
    #[must_use]
    pub const fn new(major: u16, minor: u16, patch: u16) -> Self {
        Self([major, minor, patch])
    }

    /// Parses `major.minor.patch` in canonical decimal, without leading zeroes.
    ///
    /// Usable in constants, so the running version is checked at compile time.
    ///
    /// # Errors
    /// Returns [`VersionError`] for any other spelling or a component above
    /// `u16::MAX`.
    pub const fn parse(text: &str) -> Result<Self, VersionError> {
        let bytes = text.as_bytes();
        let mut parts = [0; 3];
        let mut at = 0;
        let mut part = 0;
        while part < 3 {
            if part > 0 {
                if at >= bytes.len() || bytes[at] != b'.' {
                    return Err(VersionError);
                }
                at += 1;
            }
            match component(bytes, at) {
                Ok((value, end)) => {
                    parts[part] = value;
                    at = end;
                }
                Err(error) => return Err(error),
            }
            part += 1;
        }
        if at != bytes.len() {
            return Err(VersionError);
        }
        Ok(Self(parts))
    }

    /// Encodes the three components as little-endian `u16`s.
    #[must_use]
    pub fn to_le_bytes(self) -> [u8; Self::ENCODED_LEN] {
        let mut bytes = [0; Self::ENCODED_LEN];
        for (pair, value) in bytes.as_chunks_mut::<2>().0.iter_mut().zip(self.0) {
            *pair = value.to_le_bytes();
        }
        bytes
    }

    /// Decodes [`Self::to_le_bytes`]; every bit pattern is a valid version.
    #[must_use]
    pub fn from_le_bytes(bytes: [u8; Self::ENCODED_LEN]) -> Self {
        let pairs = bytes.as_chunks::<2>().0;
        Self(core::array::from_fn(|i| u16::from_le_bytes(pairs[i])))
    }
}

/// Reads one decimal component starting at `start`; returns it and the
/// index after its last digit. Empty components and leading zeroes are not
/// canonical.
const fn component(bytes: &[u8], start: usize) -> Result<(u16, usize), VersionError> {
    let mut at = start;
    let mut value: u32 = 0;
    while at < bytes.len() && bytes[at].is_ascii_digit() {
        value = value * 10 + (bytes[at] - b'0') as u32;
        if value > u16::MAX as u32 {
            return Err(VersionError);
        }
        at += 1;
    }
    let digits = at - start;
    if digits == 0 || (digits > 1 && bytes[start] == b'0') {
        return Err(VersionError);
    }
    Ok((value as u16, at))
}

/// A version that is not canonical `major.minor.patch`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VersionError;

impl core::fmt::Display for VersionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("version is not canonical major.minor.patch")
    }
}

impl core::error::Error for VersionError {}

impl core::fmt::Display for Version {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let [major, minor, patch] = self.0;
        write!(f, "{major}.{minor}.{patch}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_canonical_spelling_parses() {
        assert_eq!(Version::parse("65535.0.42"), Ok(Version::new(65535, 0, 42)));
        for bad in [
            "",
            "1.2",
            "1.2.3.4",
            "01.2.3",
            "1.02.3",
            "1.2.03",
            "65536.0.0",
            "1.2.3-dev",
            "+1.2.3",
            "1.2. 3",
            "1..3",
            "1.2.3.",
        ] {
            assert!(Version::parse(bad).is_err(), "{bad}");
        }
    }
}
