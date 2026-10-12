//! Errors by area, and the one error an operation reports to the outside.
//!
//! Each area returns its own type: [`FlashError`] for raw flash access,
//! [`StateError`] for the journal and the bootloader selection,
//! [`RequestError`] for the HTTP framing, [`Refusal`] for admission,
//! [`Expired`] for the upload pacing, and
//! [`TransferError`](crate::writer::TransferError) for the writer.
//! [`Error`] gathers them where a request or the status page has to name a
//! reason; [`Error::code`] is the stable spelling shared by the HTTP API, the
//! status page and the `ota-pack` tool.

use dcc_ota_format::{
    image::ImageError,
    package::{Ineligible, PackageError},
};
use embedded_storage::nor_flash::{NorFlashError, NorFlashErrorKind};

/// A flash operation that failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    /// The driver failed; carries its own classification.
    Driver(NorFlashErrorKind),
    /// Flash accepted a write but reads back different bytes.
    ReadBack,
    /// The flash is smaller than the layout, or its read, write or erase
    /// units do not fit the word and sector access of the update.
    Unsuitable,
}

impl FlashError {
    /// Wraps a driver error, keeping its kind.
    #[must_use]
    #[expect(clippy::needless_pass_by_value, reason = "`map_err` callback")]
    pub fn driver(error: impl NorFlashError) -> Self {
        Self::Driver(error.kind())
    }

    /// Returns the stable machine-readable code.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::Driver(_) | Self::ReadBack => "flash_error",
            Self::Unsuitable => "ota_unavailable",
        }
    }
}

impl core::fmt::Display for FlashError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Driver(kind) => write!(f, "flash driver failed ({kind:?})"),
            Self::ReadBack => f.write_str("flash reads back other bytes than written"),
            Self::Unsuitable => f.write_str("flash does not fit the update layout"),
        }
    }
}

impl core::error::Error for FlashError {}

/// Why the stored update state, journal or bootloader selection, cannot
/// be read or changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateError {
    /// The flash under it failed.
    Flash(FlashError),
    /// The stored state is inconsistent and cannot be trusted.
    Corrupt,
    /// The journal does not allow this step in its current phase.
    WrongPhase,
}

impl From<FlashError> for StateError {
    fn from(error: FlashError) -> Self {
        Self::Flash(error)
    }
}

impl core::fmt::Display for StateError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Flash(error) => error.fmt(f),
            Self::Corrupt => f.write_str("stored update state is inconsistent"),
            Self::WrongPhase => f.write_str("the journal does not allow this step now"),
        }
    }
}

impl core::error::Error for StateError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Flash(error) => Some(error),
            Self::Corrupt | Self::WrongPhase => None,
        }
    }
}

/// Why a request is refused before any of its body is used.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RequestError {
    /// The framing is malformed.
    Malformed,
    /// An unknown route, or a request not addressed to this device.
    Foreign,
    /// The head or the body is larger than allowed.
    TooLarge,
    /// The upload is too short to hold a package.
    TooShort,
}

impl RequestError {
    /// Returns the stable machine-readable code.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::Malformed => "bad_request",
            Self::Foreign => "forbidden",
            Self::TooLarge => "too_large",
            Self::TooShort => "bad_package",
        }
    }
}

dcc_ota_format::coded_error!(RequestError);

/// Why an operation is not admitted now.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    /// Update state is unusable; only a USB flash repairs it.
    Recovery,
    /// A new image is on trial; it must be confirmed or rolled back first.
    PendingVerify,
    /// Boot has not finished arming the track yet.
    BootNotReady,
    /// Another operation holds the lock, or a failure asks for a pause.
    Busy,
    /// The track is powered; updates start only with the track off.
    TrackOn,
}

impl Refusal {
    /// Returns the stable machine-readable code.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::Recovery => "ota_unavailable",
            Self::PendingVerify => "pending_verify",
            Self::BootNotReady => "boot_not_ready",
            Self::Busy => "busy",
            Self::TrackOn => "track_on",
        }
    }
}

dcc_ota_format::coded_error!(Refusal);

/// The upload ran past its deadline or below the minimum rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Expired;

impl Expired {
    /// Returns the stable machine-readable code.
    #[must_use]
    pub const fn code(self) -> &'static str {
        "timeout"
    }
}

dcc_ota_format::coded_error!(Expired);

/// Why an update, a boot check or a status request failed, as reported to
/// the client, the status page and the log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// A flash operation failed.
    Flash(FlashError),
    /// Stored update state (journal, otadata, partition table or image) is
    /// inconsistent and cannot be trusted.
    Corrupt,
    /// The step is not allowed in the current state.
    Forbidden,
    /// The update service lacks a task or the flash it needs.
    Unavailable,
    /// A response did not fit its fixed-size buffer: a firmware bug,
    /// reported rather than panicking so the device keeps running.
    Internal,
    /// The network connection failed while reading or writing.
    Network,
    /// A transfer, flash step or acknowledgement missed its deadline.
    Timeout,
    /// The HTTP request was refused.
    Request(RequestError),
    /// The package header was refused.
    Package(PackageError),
    /// The application image was refused.
    Image(ImageError),
    /// The authentic release may not be installed on this device.
    Ineligible(Ineligible),
    /// The operation is not admitted now.
    Refused(Refusal),
}

impl Error {
    /// Returns the stable machine-readable code used by the CLI and HTTP API.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::Flash(error) => error.code(),
            Self::Corrupt => "ota_corrupt",
            Self::Forbidden => "forbidden",
            Self::Unavailable => "ota_unavailable",
            Self::Internal => "internal_error",
            Self::Network => "network_error",
            Self::Timeout => "timeout",
            Self::Request(error) => error.code(),
            Self::Package(error) => error.code(),
            Self::Image(error) => error.code(),
            Self::Ineligible(error) => error.code(),
            Self::Refused(error) => error.code(),
        }
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let what = match self {
            Self::Flash(error) => return write!(f, "{error} ({})", self.code()),
            Self::Corrupt => "stored update state is inconsistent",
            Self::Forbidden => "step not allowed in the current state",
            Self::Unavailable => "update service unavailable",
            Self::Internal => "response does not fit its buffer",
            Self::Network => "network connection failed",
            Self::Timeout => "deadline passed",
            Self::Request(_) => "request refused",
            Self::Package(_) => "package refused",
            Self::Image(_) => "image refused",
            Self::Ineligible(_) => "release not eligible",
            Self::Refused(_) => "operation not admitted now",
        };
        write!(f, "{what} ({})", self.code())
    }
}

impl core::error::Error for Error {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Flash(error) => Some(error),
            Self::Request(error) => Some(error),
            Self::Package(error) => Some(error),
            Self::Image(error) => Some(error),
            Self::Ineligible(error) => Some(error),
            Self::Refused(error) => Some(error),
            Self::Corrupt
            | Self::Forbidden
            | Self::Unavailable
            | Self::Internal
            | Self::Network
            | Self::Timeout => None,
        }
    }
}

/// Wraps each area error in the [`Error`] variant that carries it.
macro_rules! wrap_in_error {
    ($($area:ty => $variant:ident),+ $(,)?) => {$(
        impl From<$area> for Error {
            fn from(error: $area) -> Self {
                Self::$variant(error)
            }
        }
    )+};
}

wrap_in_error! {
    FlashError => Flash,
    RequestError => Request,
    PackageError => Package,
    ImageError => Image,
    Ineligible => Ineligible,
    Refusal => Refused,
}

impl From<StateError> for Error {
    fn from(error: StateError) -> Self {
        match error {
            StateError::Flash(error) => Self::Flash(error),
            StateError::Corrupt => Self::Corrupt,
            StateError::WrongPhase => Self::Forbidden,
        }
    }
}

impl From<Expired> for Error {
    fn from(_: Expired) -> Self {
        Self::Timeout
    }
}
