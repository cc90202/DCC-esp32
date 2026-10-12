//! HTTP responses.

use core::fmt::Write as _;

use dcc_ota::{Error, Refusal, RequestError};
use dcc_ota_format::package::PackageError;
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Instant};

use super::io;

/// Longest time to write one response. The page is about 13 KiB through a
/// transmit buffer of [`super::TX_BUFFER`] bytes, about eighteen windows,
/// which a weak Wi-Fi link may need seconds for.
const WRITE_TIMEOUT: Duration = Duration::from_secs(10);

/// Response status line.
#[derive(Clone, Copy)]
pub(super) enum Status {
    Ok,
    BadRequest,
    Forbidden,
    RequestTimeout,
    Conflict,
    ContentTooLarge,
    ServiceUnavailable,
}

impl Status {
    const fn line(self) -> &'static str {
        match self {
            Self::Ok => "200 OK",
            Self::BadRequest => "400 Bad Request",
            Self::Forbidden => "403 Forbidden",
            Self::RequestTimeout => "408 Request Timeout",
            Self::Conflict => "409 Conflict",
            Self::ContentTooLarge => "413 Content Too Large",
            Self::ServiceUnavailable => "503 Service Unavailable",
        }
    }

    const fn of(error: Error) -> Self {
        match error {
            Error::Forbidden | Error::Request(RequestError::Foreign) => Self::Forbidden,
            Error::Request(RequestError::TooLarge) | Error::Package(PackageError::TooLarge) => {
                Self::ContentTooLarge
            }
            Error::Timeout => Self::RequestTimeout,
            Error::Refused(Refusal::Recovery)
            | Error::Flash(_)
            | Error::Corrupt
            | Error::Unavailable
            | Error::Internal
            | Error::Network => Self::ServiceUnavailable,
            Error::Refused(_) => Self::Conflict,
            Error::Request(_) | Error::Package(_) | Error::Image(_) | Error::Ineligible(_) => {
                Self::BadRequest
            }
        }
    }
}

/// Body media type.
#[derive(Clone, Copy)]
pub(super) enum Content {
    Html,
    Json,
}

impl Content {
    const fn mime(self) -> &'static str {
        match self {
            Self::Html => "text/html; charset=utf-8",
            Self::Json => "application/json",
        }
    }
}

/// `{"ok":true}`.
pub(super) async fn ok(socket: &mut TcpSocket<'_>) -> Result<(), Error> {
    send(socket, Status::Ok, Content::Json, "{\"ok\":true}").await
}

/// `{"ok":false,"error":CODE}` with the status that fits `error`.
pub(super) async fn error(socket: &mut TcpSocket<'_>, error: Error) -> Result<(), Error> {
    // `{"ok":false,"error":""}` plus the longest code.
    let mut json = heapless::String::<64>::new();
    write!(json, "{{\"ok\":false,\"error\":\"{}\"}}", error.code()).map_err(|_| Error::Internal)?;
    send(socket, Status::of(error), Content::Json, &json).await
}

/// Sends one complete response and flushes it.
pub(super) async fn send(
    socket: &mut TcpSocket<'_>,
    status: Status,
    content: Content,
    body: &str,
) -> Result<(), Error> {
    // Status line and five fixed headers.
    let mut head = heapless::String::<256>::new();
    write!(head, "HTTP/1.1 {}\r\n", status.line()).map_err(|_| Error::Internal)?;
    write!(head, "Content-Type: {}\r\n", content.mime()).map_err(|_| Error::Internal)?;
    write!(head, "Content-Length: {}\r\n", body.len()).map_err(|_| Error::Internal)?;
    head.push_str(
        "Connection: close\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\n\r\n",
    )
    .map_err(|_| Error::Internal)?;
    let deadline = Instant::now() + WRITE_TIMEOUT;
    for bytes in [head.as_bytes(), body.as_bytes()] {
        io(deadline, embedded_io_async::Write::write_all(socket, bytes)).await?;
    }
    io(deadline, socket.flush()).await
}
