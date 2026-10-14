//! Allocation-free framing of the update requests; the caller streams the
//! body separately.
//!
//! Only four requests exist, and every one must name the device by its IP
//! address, so a page on another origin cannot drive an update: a foreign
//! `Origin` or `Host` is refused, and the two writing requests also need the
//! `X-DCC-OTA: 1` header, which a cross-site form cannot set.
//!
//! [`status_of`] maps each error to the status its response carries.

use core::net::Ipv4Addr;

use dcc_ota_format::{
    ImageLen,
    package::{HEADER_LEN, PackageError},
};

use crate::{Error, Refusal, RequestError};

/// Longest request head accepted, terminator included.
pub const MAX_HEAD_BYTES: usize = 1024;

/// The requests the update service answers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    /// `GET /update`: the upload page.
    Page,
    /// `GET /update/status`: the JSON status.
    Status,
    /// `POST /update/check`: authenticate a package header only.
    Check,
    /// `POST /update`: header followed by the image.
    Upload,
}

/// A parsed request head.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Head {
    /// Requested operation.
    pub route: Route,
    /// Declared body length, already bounded for the route.
    pub content_length: usize,
    /// Offset of the first body byte in the parsed buffer.
    pub body_start: usize,
}

/// Parses the head at the start of `bytes`, for a device at `ip`.
///
/// Returns `Ok(None)` until the blank line that ends the head arrives.
///
/// # Errors
/// Returns [`RequestError::TooLarge`] for an oversized head or body,
/// [`RequestError::Foreign`] for an unknown route or a request not
/// addressed to this device, [`RequestError::TooShort`] for an upload too
/// short to hold a package, and [`RequestError::Malformed`] for malformed
/// framing.
pub fn parse(bytes: &[u8], ip: Ipv4Addr) -> Result<Option<Head>, RequestError> {
    let Some(end) = bytes.windows(4).position(|w| w == b"\r\n\r\n") else {
        return if bytes.len() >= MAX_HEAD_BYTES {
            Err(RequestError::TooLarge)
        } else {
            Ok(None)
        };
    };
    let body_start = end + 4;
    if body_start > MAX_HEAD_BYTES {
        return Err(RequestError::TooLarge);
    }
    let text = core::str::from_utf8(&bytes[..end]).map_err(|_| RequestError::Malformed)?;
    let mut lines = text.split("\r\n");
    let route = parse_route(lines.next().ok_or(RequestError::Malformed)?)?;
    let headers = Headers::parse(lines)?;
    headers.authorize(route, ip)?;
    let content_length = route_length(route, headers.length)?;
    if bytes.len() - body_start > content_length {
        return Err(RequestError::Malformed);
    }
    Ok(Some(Head {
        route,
        content_length,
        body_start,
    }))
}

fn parse_route(line: &str) -> Result<Route, RequestError> {
    let mut request = line.split(' ');
    let method = request.next().ok_or(RequestError::Malformed)?;
    let path = request.next().ok_or(RequestError::Malformed)?;
    let version = request.next().ok_or(RequestError::Malformed)?;
    if request.next().is_some() || !matches!(version, "HTTP/1.0" | "HTTP/1.1") {
        return Err(RequestError::Malformed);
    }
    Ok(match (method, path) {
        ("GET", "/update") => Route::Page,
        ("GET", "/update/status") => Route::Status,
        ("POST", "/update/check") => Route::Check,
        ("POST", "/update") => Route::Upload,
        _ => return Err(RequestError::Foreign),
    })
}

#[derive(Default)]
struct Headers<'a> {
    length: Option<usize>,
    host: Option<&'a str>,
    origin: Option<&'a str>,
    token: Option<&'a str>,
}

impl<'a> Headers<'a> {
    fn parse(lines: impl Iterator<Item = &'a str>) -> Result<Self, RequestError> {
        let mut headers = Self::default();
        for line in lines {
            let (name, value) = parse_header(line)?;
            if name.eq_ignore_ascii_case("content-length") {
                if headers.length.is_some() {
                    return Err(RequestError::Malformed);
                }
                headers.length = Some(decimal(value)?);
            } else if name.eq_ignore_ascii_case("transfer-encoding") {
                return Err(RequestError::Malformed);
            } else if name.eq_ignore_ascii_case("host") {
                unique(&mut headers.host, value)?;
            } else if name.eq_ignore_ascii_case("origin") {
                unique(&mut headers.origin, value)?;
            } else if name.eq_ignore_ascii_case("x-dcc-ota") {
                unique(&mut headers.token, value)?;
            }
        }
        Ok(headers)
    }

    fn authorize(&self, route: Route, ip: Ipv4Addr) -> Result<(), RequestError> {
        if !self.host.is_some_and(|h| authority(h, ip))
            || self
                .origin
                .is_some_and(|o| !o.strip_prefix("http://").is_some_and(|h| authority(h, ip)))
            || (matches!(route, Route::Check | Route::Upload) && self.token != Some("1"))
        {
            return Err(RequestError::Foreign);
        }
        Ok(())
    }
}

fn parse_header(line: &str) -> Result<(&str, &str), RequestError> {
    let (name, value) = line.split_once(':').ok_or(RequestError::Malformed)?;
    if name.is_empty() || !name.bytes().all(is_token) {
        return Err(RequestError::Malformed);
    }
    if !value
        .bytes()
        .all(|b| b == b'\t' || b == b' ' || b.is_ascii_graphic())
    {
        return Err(RequestError::Malformed);
    }
    Ok((name, value.trim_matches([' ', '\t'])))
}

fn route_length(route: Route, length: Option<usize>) -> Result<usize, RequestError> {
    const MAX_UPLOAD: usize = HEADER_LEN + ImageLen::MAX.as_usize();
    const MIN_UPLOAD: usize = HEADER_LEN + ImageLen::MIN.as_usize();
    match (route, length) {
        (Route::Page | Route::Status, None | Some(0)) => Ok(0),
        (Route::Check, Some(HEADER_LEN)) => Ok(HEADER_LEN),
        (Route::Upload, Some(n @ MIN_UPLOAD..=MAX_UPLOAD)) => Ok(n),
        (Route::Upload, Some(..MIN_UPLOAD)) => Err(RequestError::TooShort),
        (Route::Upload, Some(_)) => Err(RequestError::TooLarge),
        _ => Err(RequestError::Malformed),
    }
}

fn is_token(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b"!#$%&'*+-.^_`|~".contains(&b)
}

fn unique<'a>(slot: &mut Option<&'a str>, value: &'a str) -> Result<(), RequestError> {
    if slot.replace(value).is_some() {
        Err(RequestError::Malformed)
    } else {
        Ok(())
    }
}

fn decimal(value: &str) -> Result<usize, RequestError> {
    // Canonical decimal only: no sign, no leading zero.
    if value.is_empty()
        || !value.bytes().all(|b| b.is_ascii_digit())
        || (value.len() > 1 && value.starts_with('0'))
    {
        return Err(RequestError::Malformed);
    }
    value.bytes().try_fold(0usize, |n, b| {
        n.checked_mul(10)
            .and_then(|n| n.checked_add(usize::from(b - b'0')))
            .ok_or(RequestError::TooLarge)
    })
}

/// The status of a response: its code and reason phrase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// `200 OK`.
    Ok,
    /// `400 Bad Request`.
    BadRequest,
    /// `403 Forbidden`.
    Forbidden,
    /// `408 Request Timeout`.
    RequestTimeout,
    /// `409 Conflict`.
    Conflict,
    /// `413 Content Too Large`.
    ContentTooLarge,
    /// `503 Service Unavailable`.
    ServiceUnavailable,
}

impl Status {
    /// The numeric status code.
    #[must_use]
    pub const fn code(self) -> u16 {
        match self {
            Self::Ok => 200,
            Self::BadRequest => 400,
            Self::Forbidden => 403,
            Self::RequestTimeout => 408,
            Self::Conflict => 409,
            Self::ContentTooLarge => 413,
            Self::ServiceUnavailable => 503,
        }
    }

    /// The reason phrase that follows the code on the status line.
    #[must_use]
    pub const fn reason(self) -> &'static str {
        match self {
            Self::Ok => "OK",
            Self::BadRequest => "Bad Request",
            Self::Forbidden => "Forbidden",
            Self::RequestTimeout => "Request Timeout",
            Self::Conflict => "Conflict",
            Self::ContentTooLarge => "Content Too Large",
            Self::ServiceUnavailable => "Service Unavailable",
        }
    }
}

/// The status of the response that reports `error`: a refused request or
/// package is the client's fault, a busy device a conflict, and a device
/// that cannot take an update at all is unavailable.
#[must_use]
pub const fn status_of(error: Error) -> Status {
    match error {
        Error::Forbidden | Error::Request(RequestError::Foreign) => Status::Forbidden,
        Error::Request(RequestError::TooLarge) | Error::Package(PackageError::TooLarge) => {
            Status::ContentTooLarge
        }
        Error::Timeout => Status::RequestTimeout,
        Error::Refused(Refusal::Recovery)
        | Error::Flash(_)
        | Error::Corrupt
        | Error::Unavailable
        | Error::Internal
        | Error::Network => Status::ServiceUnavailable,
        Error::Refused(_) => Status::Conflict,
        Error::Request(_) | Error::Package(_) | Error::Image(_) | Error::Ineligible(_) => {
            Status::BadRequest
        }
    }
}

/// Whether `value` names this device: its address, optionally with the
/// default port. The standard parser refuses leading zeroes and short forms.
fn authority(value: &str, ip: Ipv4Addr) -> bool {
    value.strip_suffix(":80").unwrap_or(value).parse() == Ok(ip)
}

#[cfg(test)]
mod tests {
    use super::*;
    const IP: Ipv4Addr = Ipv4Addr::new(192, 168, 1, 20);
    fn request(method: &str, path: &str, headers: &str) -> std::string::String {
        std::format!("{method} {path} HTTP/1.1\r\nHost: 192.168.1.20\r\n{headers}\r\n")
    }
    #[test]
    fn routes_and_partial_body() {
        for (method, path, headers, route, len) in [
            ("GET", "/update", "", Route::Page, 0),
            ("GET", "/update/status", "", Route::Status, 0),
            (
                "POST",
                "/update/check",
                "X-DCC-OTA: 1\r\nContent-Length: 256\r\n",
                Route::Check,
                256,
            ),
            (
                "POST",
                "/update",
                "X-DCC-OTA: 1\r\nContent-Length: 4352\r\n",
                Route::Upload,
                4352,
            ),
        ] {
            let r = request(method, path, headers);
            let h = parse(r.as_bytes(), IP).unwrap().unwrap();
            assert_eq!(
                (h.route, h.content_length, h.body_start),
                (route, len, r.len())
            );
        }
    }
    #[test]
    fn realistic_browser_and_default_port() {
        let r = request(
            "POST",
            "/update/check",
            concat!(
                "Origin: http://192.168.1.20:80\r\nX-DCC-OTA: 1\r\nContent-Length: 256\r\n",
                "Content-Type: application/octet-stream\r\nAccept: */*\r\n",
                "User-Agent: Mozilla/5.0 (Linux; Android 14) AppleWebKit/537.36 Chrome/126.0 Mobile Safari/537.36\r\n",
                "Accept-Language: it-IT,it;q=0.9\r\nConnection: keep-alive\r\n"
            ),
        );
        assert!(parse(r.as_bytes(), IP).unwrap().is_some());
        assert!(
            parse(
                r.replace("Host: 192.168.1.20", "Host: 192.168.1.20:80")
                    .as_bytes(),
                IP
            )
            .is_ok()
        );
    }
    #[test]
    fn malicious_headers() {
        for headers in [
            "Content-Length: 0\r\ncontent-length: 0\r\n",
            "Transfer-Encoding: chunked\r\n",
            "Host: evil.test\r\n",
            "Origin: http://192.168.1.20\r\nOrigin: http://192.168.1.20\r\n",
            "X-DCC-OTA: 1\r\nX-DCC-OTA: 1\r\n",
            " Host: evil.test\r\n",
            "Content-Length: +0\r\n",
            "Content-Length: 1\r\n",
            "Content-Length : 0\r\n",
        ] {
            assert!(parse(request("GET", "/update", headers).as_bytes(), IP).is_err());
        }
        for host in [
            "evil.test",
            "192.168.1.20.evil.test",
            "192.168.1.20:81",
            "0192.168.1.20",
            "192.168.1.20@evil.test",
        ] {
            assert_eq!(
                parse(
                    request("GET", "/update", "")
                        .replace("192.168.1.20", host)
                        .as_bytes(),
                    IP
                ),
                Err(RequestError::Foreign)
            );
        }
        for origin in [
            "null",
            "https://192.168.1.20",
            "http://192.168.1.20/",
            "http://evil.test",
            "http://192.168.1.20:080",
        ] {
            assert_eq!(
                parse(
                    request("GET", "/update", &std::format!("Origin: {origin}\r\n")).as_bytes(),
                    IP
                ),
                Err(RequestError::Foreign)
            );
        }
    }
    #[test]
    fn bounds_and_framing() {
        assert_eq!(parse(b"GET /update", IP), Ok(None));
        assert_eq!(parse(&[b' '; 1024], IP), Err(RequestError::TooLarge));
        let prefix = request("GET", "/update", "X-Padding: ");
        let mut r = prefix.trim_end_matches("\r\n").to_owned();
        r.push_str(&"a".repeat(1020 - r.len()));
        r.push_str("\r\n\r\n");
        assert!(parse(r.as_bytes(), IP).is_ok());
        r.insert(100, 'a');
        assert_eq!(parse(r.as_bytes(), IP), Err(RequestError::TooLarge));
        for (n, error) in [
            ("4351", RequestError::TooShort),
            ("3145985", RequestError::TooLarge),
            ("99999999999999999999999999999", RequestError::TooLarge),
            ("04352", RequestError::Malformed),
        ] {
            assert_eq!(
                parse(
                    request(
                        "POST",
                        "/update",
                        &std::format!("X-DCC-OTA: 1\r\nContent-Length: {n}\r\n")
                    )
                    .as_bytes(),
                    IP
                ),
                Err(error),
                "Content-Length: {n}"
            );
        }
        assert!(
            parse(
                request("GET", "/update", "Content-Length: 0\r\n").as_bytes(),
                IP
            )
            .is_ok()
        );
        assert!(parse(request("GET", "/update", "Accept: a\tb\r\n").as_bytes(), IP).is_ok());
        for r in [
            request("GET", "/update", "Accept: a\x01b\r\n"),
            request("GET", "/update", "").replace("HTTP/1.1", "HTTP/2.0"),
            request("GET", "/update", "").replace("HTTP/1.1", "HTTP/1.1 x"),
        ] {
            assert_eq!(
                parse(r.as_bytes(), IP),
                Err(RequestError::Malformed),
                "{r:?}"
            );
        }
        let mut r = request(
            "POST",
            "/update/check",
            "X-DCC-OTA: 1\r\nContent-Length: 256\r\n",
        );
        r.push_str(&"x".repeat(257));
        assert_eq!(parse(r.as_bytes(), IP), Err(RequestError::Malformed));
        for (method, path) in [
            ("OPTIONS", "/update"),
            ("PUT", "/update"),
            ("GET", "/update/check"),
            ("GET", "/unknown"),
        ] {
            assert_eq!(
                parse(request(method, path, "").as_bytes(), IP),
                Err(RequestError::Foreign)
            );
        }
    }

    #[test]
    fn status_code_table() {
        use crate::{Error, FlashError, Refusal};
        use dcc_ota_format::{
            image::ImageError,
            package::{Ineligible, PackageError},
        };

        for (error, line) in [
            (Error::Forbidden, "403 Forbidden"),
            (Error::Request(RequestError::Foreign), "403 Forbidden"),
            (
                Error::Request(RequestError::TooLarge),
                "413 Content Too Large",
            ),
            (
                Error::Package(PackageError::TooLarge),
                "413 Content Too Large",
            ),
            (Error::Timeout, "408 Request Timeout"),
            (Error::Refused(Refusal::Recovery), "503 Service Unavailable"),
            (
                Error::Flash(FlashError::ReadBack),
                "503 Service Unavailable",
            ),
            (Error::Corrupt, "503 Service Unavailable"),
            (Error::Unavailable, "503 Service Unavailable"),
            (Error::Internal, "503 Service Unavailable"),
            (Error::Network, "503 Service Unavailable"),
            (Error::Refused(Refusal::Busy), "409 Conflict"),
            (Error::Refused(Refusal::TrackOn), "409 Conflict"),
            (Error::Request(RequestError::Malformed), "400 Bad Request"),
            (Error::Request(RequestError::TooShort), "400 Bad Request"),
            (
                Error::Package(PackageError::BadSignature),
                "400 Bad Request",
            ),
            (Error::Image(ImageError::DigestMismatch), "400 Bad Request"),
            (Error::Ineligible(Ineligible::Downgrade), "400 Bad Request"),
        ] {
            let status = status_of(error);
            let sent = std::format!("{} {}", status.code(), status.reason());
            assert_eq!(sent, line, "{error:?}");
        }
        assert_eq!((Status::Ok.code(), Status::Ok.reason()), (200, "OK"));
    }
}
