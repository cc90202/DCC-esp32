//! Signed OTA format and power-loss-safe persistence, independent of the HAL.
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod admission;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod boot_state;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod health;
pub mod image;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod journal;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod otadata;
pub mod package;
#[cfg(target_arch = "riscv32")]
pub(crate) mod runtime;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod session;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) mod status;

#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) const SECTOR_SIZE: usize = 4096;
/// Maximum non-merged application image length for the approved 8 MB layout.
pub const SLOT_SIZE: u32 = 0x300000;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) const SLOT_OFFSETS: [u32; 2] = [0x10000, 0x310000];
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) const JOURNAL_OFFSET: u32 = 0x613000;
#[cfg(any(test, target_arch = "riscv32"))]
pub(crate) const OTADATA_OFFSET: u32 = 0xD000;

/// Package, storage and runtime failures exposed through stable status codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Flash,
    Corrupt,
    BadPackage,
    UnknownKey,
    BadSignature,
    SameVersion,
    Downgrade,
    RejectedVersion,
    TooLarge,
    BadImage,
    DigestMismatch,
    Timeout,
    Forbidden,
    Busy,
    TrackOn,
    BootNotReady,
    PendingVerify,
    Unavailable,
}

impl Error {
    /// Returns the stable machine-readable code used by the CLI and HTTP API.
    pub const fn code(self) -> &'static str {
        match self {
            Self::Flash => "flash_error",
            Self::Corrupt | Self::Unavailable => "ota_unavailable",
            Self::BadPackage => "bad_package",
            Self::UnknownKey => "unknown_key",
            Self::BadSignature => "bad_signature",
            Self::SameVersion => "same_version",
            Self::Downgrade => "downgrade",
            Self::RejectedVersion => "rejected_version",
            Self::TooLarge => "too_large",
            Self::BadImage => "bad_image",
            Self::DigestMismatch => "digest_mismatch",
            Self::Timeout => "timeout",
            Self::Forbidden => "forbidden",
            Self::Busy => "busy",
            Self::TrackOn => "track_on",
            Self::BootNotReady => "boot_not_ready",
            Self::PendingVerify => "pending_verify",
        }
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.code())
    }
}

impl core::error::Error for Error {}

/// Numeric major/minor/patch version, ordered lexicographically.
/// Every component is valid; textual parsing separately enforces canonical spelling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Version(pub [u16; 3]);

impl core::fmt::Display for Version {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}.{}.{}", self.0[0], self.0[1], self.0[2])
    }
}

#[cfg(any(test, target_arch = "riscv32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ImageId {
    pub slot: u8,
    pub version: Version,
    pub len: u32,
    pub digest: [u8; 32],
}

#[cfg(any(test, target_arch = "riscv32"))]
impl ImageId {
    pub fn validate(self) -> Result<Self, Error> {
        if self.slot > 1 || !(4096..=SLOT_SIZE).contains(&self.len) {
            return Err(Error::Corrupt);
        }
        Ok(self)
    }
}
