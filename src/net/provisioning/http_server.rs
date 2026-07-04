//! Minimal provisioning HTTP server over `embassy-net` TCP sockets.

use core::fmt::Write as _;

use defmt::{info, warn};
use embassy_net::Stack;
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Timer, with_timeout};
use static_cell::StaticCell;

use crate::net::wifi_config::WifiCredentialsStore;

use super::html::{INVALID_FORM_PAGE, SAVE_ERROR_PAGE, SAVE_OK_PAGE, SETUP_PAGE};
use super::http_parser::{MAX_REQUEST_BYTES, ParseError, ParsedRequest, parse_request};
use super::net_config::SETUP_URL;

const HTTP_PORT: u16 = 80;
const SOCKET_TIMEOUT: Duration = Duration::from_secs(10);
const REBOOT_DELAY: Duration = Duration::from_millis(500);

static HTTP_RX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
static HTTP_TX_BUFFER: StaticCell<[u8; 1536]> = StaticCell::new();

pub async fn run_http_server<S>(stack: Stack<'static>, store: &mut S) -> !
where
    S: WifiCredentialsStore,
{
    let rx_buffer = HTTP_RX_BUFFER.init([0; 1024]);
    let tx_buffer = HTTP_TX_BUFFER.init([0; 1536]);
    let mut socket = TcpSocket::new(stack, rx_buffer, tx_buffer);
    socket.set_timeout(Some(SOCKET_TIMEOUT));

    loop {
        info!("Provisioning HTTP listening on {}", SETUP_URL);
        let abort = match socket.accept(HTTP_PORT).await {
            Ok(()) => handle_connection(&mut socket, store).await,
            Err(_) => {
                warn!("Provisioning HTTP accept failed");
                true
            }
        };
        if abort {
            socket.abort();
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

async fn handle_connection<S>(socket: &mut TcpSocket<'_>, store: &mut S) -> bool
where
    S: WifiCredentialsStore,
{
    let request = match read_request(socket).await {
        RequestOutcome::Request(request) => request,
        RequestOutcome::Invalid(ParseError::InvalidCredentials) => {
            let _ = send_response(
                socket,
                Status::Ok,
                "text/html; charset=utf-8",
                INVALID_FORM_PAGE,
            )
            .await;
            return false;
        }
        RequestOutcome::Invalid(error) => {
            let status = Status::from_parse_error(error);
            let _ = send_response(socket, status, "text/plain", status.reason()).await;
            return false;
        }
        RequestOutcome::Timeout => {
            let _ = send_response(socket, Status::RequestTimeout, "text/plain", "Timeout").await;
            return false;
        }
        RequestOutcome::Disconnected => return true,
    };

    match request {
        ParsedRequest::GetRoot => {
            let _ = send_response(socket, Status::Ok, "text/html; charset=utf-8", SETUP_PAGE).await;
        }
        ParsedRequest::SaveCredentials(credentials) => match store.save(&credentials) {
            Ok(()) => {
                info!("WiFi credentials saved from provisioning form");
                match send_response(socket, Status::Ok, "text/html; charset=utf-8", SAVE_OK_PAGE)
                    .await
                {
                    Ok(()) => {
                        // Reboot only after the success page went out; a save
                        // failure never reaches this arm, so a reboot with
                        // unsaved credentials is impossible by construction.
                        Timer::after(REBOOT_DELAY).await;
                        info!("WiFi credentials saved; rebooting into station mode");
                        esp_hal::system::software_reset();
                    }
                    Err(_) => {
                        warn!("WiFi credential save succeeded but success response failed");
                    }
                }
            }
            Err(_) => {
                warn!("WiFi credential save failed");
                let _ = send_response(
                    socket,
                    Status::InternalServerError,
                    "text/html; charset=utf-8",
                    SAVE_ERROR_PAGE,
                )
                .await;
            }
        },
    }
    false
}

enum RequestOutcome {
    Request(ParsedRequest),
    Invalid(ParseError),
    Timeout,
    Disconnected,
}

async fn read_request(socket: &mut TcpSocket<'_>) -> RequestOutcome {
    let mut request = [0u8; MAX_REQUEST_BYTES];
    let mut len = 0usize;

    loop {
        match parse_request(&request[..len]) {
            Ok(request) => return RequestOutcome::Request(request),
            Err(ParseError::EarlyEof) if len < request.len() => {}
            // Buffer full while the request is still incomplete: too large.
            Err(ParseError::EarlyEof) => {
                return RequestOutcome::Invalid(ParseError::PayloadTooLarge);
            }
            Err(error) => return RequestOutcome::Invalid(error),
        }

        match with_timeout(SOCKET_TIMEOUT, socket.read(&mut request[len..])).await {
            Err(_) => return RequestOutcome::Timeout,
            Ok(Err(_)) => return RequestOutcome::Disconnected,
            // Peer closed before completing the request.
            Ok(Ok(0)) => return RequestOutcome::Invalid(ParseError::EarlyEof),
            Ok(Ok(read)) => len += read,
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum Status {
    Ok,
    BadRequest,
    NotFound,
    MethodNotAllowed,
    LengthRequired,
    PayloadTooLarge,
    UriTooLong,
    UnsupportedMediaType,
    RequestTimeout,
    RequestHeaderFieldsTooLarge,
    InternalServerError,
    HttpVersionNotSupported,
}

impl Status {
    const fn code(self) -> u16 {
        match self {
            Self::Ok => 200,
            Self::BadRequest => 400,
            Self::NotFound => 404,
            Self::MethodNotAllowed => 405,
            Self::RequestTimeout => 408,
            Self::LengthRequired => 411,
            Self::PayloadTooLarge => 413,
            Self::UriTooLong => 414,
            Self::UnsupportedMediaType => 415,
            Self::RequestHeaderFieldsTooLarge => 431,
            Self::InternalServerError => 500,
            Self::HttpVersionNotSupported => 505,
        }
    }

    const fn reason(self) -> &'static str {
        match self {
            Self::Ok => "OK",
            Self::BadRequest => "Bad Request",
            Self::NotFound => "Not Found",
            Self::MethodNotAllowed => "Method Not Allowed",
            Self::RequestTimeout => "Request Timeout",
            Self::LengthRequired => "Length Required",
            Self::PayloadTooLarge => "Payload Too Large",
            Self::UriTooLong => "URI Too Long",
            Self::UnsupportedMediaType => "Unsupported Media Type",
            Self::RequestHeaderFieldsTooLarge => "Request Header Fields Too Large",
            Self::InternalServerError => "Internal Server Error",
            Self::HttpVersionNotSupported => "HTTP Version Not Supported",
        }
    }

    const fn from_parse_error(error: ParseError) -> Self {
        match error {
            ParseError::BadRequest | ParseError::EarlyEof => Self::BadRequest,
            ParseError::HeaderTooLarge => Self::RequestHeaderFieldsTooLarge,
            // Handled with the dedicated error page before reaching here.
            ParseError::InvalidCredentials => Self::Ok,
            ParseError::LengthRequired => Self::LengthRequired,
            ParseError::MethodNotAllowed => Self::MethodNotAllowed,
            ParseError::NotFound => Self::NotFound,
            ParseError::PayloadTooLarge => Self::PayloadTooLarge,
            ParseError::RequestLineTooLong => Self::UriTooLong,
            ParseError::UnsupportedContentType => Self::UnsupportedMediaType,
            ParseError::UnsupportedVersion => Self::HttpVersionNotSupported,
        }
    }
}

async fn send_response(
    socket: &mut TcpSocket<'_>,
    status: Status,
    content_type: &str,
    body: &str,
) -> Result<(), embassy_net::tcp::Error> {
    let mut header = heapless::String::<192>::new();
    let _ = core::write!(
        &mut header,
        "HTTP/1.1 {} {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        status.code(),
        status.reason(),
        content_type,
        body.len()
    );

    write_all(socket, header.as_bytes()).await?;
    write_all(socket, body.as_bytes()).await?;
    socket.close();
    socket.flush().await
}

async fn write_all(
    socket: &mut TcpSocket<'_>,
    mut bytes: &[u8],
) -> Result<(), embassy_net::tcp::Error> {
    while !bytes.is_empty() {
        let written = socket.write(bytes).await?;
        bytes = &bytes[written..];
    }
    Ok(())
}
