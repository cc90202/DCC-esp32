//! The upload page repeats the package limits and explains every error code
//! the device may answer with; this keeps both in step with the update
//! crates.

use dcc_ota::{Error, FlashError, Refusal, RequestError};
use dcc_ota_format::{
    ImageLen,
    image::ImageError,
    package::{HEADER_LEN, Ineligible, PackageError},
};
use embedded_storage::nor_flash::NorFlashErrorKind;

const PAGE: &str = include_str!("page.html");

/// Every error the update service can report, one per distinct code path.
fn every_error() -> impl Iterator<Item = Error> {
    let flash = [
        FlashError::Driver(NorFlashErrorKind::Other),
        FlashError::ReadBack,
        FlashError::Unsuitable,
    ]
    .map(Error::Flash);
    let request = [
        RequestError::Malformed,
        RequestError::Foreign,
        RequestError::TooLarge,
        RequestError::TooShort,
    ]
    .map(Error::Request);
    let package = [
        PackageError::Malformed,
        PackageError::TooLarge,
        PackageError::UnknownKey,
        PackageError::BadSignature,
    ]
    .map(Error::Package);
    let image = [
        ImageError::Malformed,
        ImageError::DigestMismatch,
        ImageError::BadBuild,
        ImageError::WrongChipRevision,
    ]
    .map(Error::Image);
    let ineligible = [
        Ineligible::SameVersion,
        Ineligible::Downgrade,
        Ineligible::Rejected,
    ]
    .map(Error::Ineligible);
    let refused = [
        Refusal::Recovery,
        Refusal::PendingVerify,
        Refusal::BootNotReady,
        Refusal::Busy,
        Refusal::TrackOn,
    ]
    .map(Error::Refused);
    let plain = [
        Error::Corrupt,
        Error::Forbidden,
        Error::Unavailable,
        Error::Internal,
        Error::Network,
        Error::Timeout,
    ];
    flash
        .into_iter()
        .chain(request)
        .chain(package)
        .chain(image)
        .chain(ineligible)
        .chain(refused)
        .chain(plain)
}

#[test]
fn page_checks_the_package_limits_of_the_firmware() {
    let shortest = HEADER_LEN + ImageLen::MIN.as_usize();
    let longest = format!("{HEADER_LEN} + {:#x}", ImageLen::MAX.get());
    assert!(
        PAGE.contains(&format!("selected.size < {shortest}")),
        "{shortest}"
    );
    assert!(
        PAGE.contains(&format!("selected.size > {longest}")),
        "{longest}"
    );
}

#[test]
fn page_explains_every_error_code() {
    for error in every_error() {
        let entry = format!("\n {}:'", error.code());
        assert!(
            PAGE.contains(&entry),
            "{error:?} has no message on the page"
        );
    }
}
