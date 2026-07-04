//! Minimal provisioning HTTP server over `embassy-net` TCP sockets.

use core::fmt::Write as _;

use defmt::{info, warn};
use embassy_net::Stack;
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Timer, with_timeout};
use static_cell::StaticCell;

use crate::net::provisioning_http::{MAX_REQUEST_BYTES, ParseError, ParsedRequest, parse_request};
use crate::net::provisioning_net::SETUP_URL;
use crate::net::provisioning_reboot::{PostSaveAction, post_save_action};
use crate::net::wifi_config::WifiCredentialsStore;

use super::html::{INVALID_FORM_PAGE, SAVE_ERROR_PAGE, SAVE_OK_PAGE, SETUP_PAGE};

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
    let mut request = [0u8; MAX_REQUEST_BYTES];
    let request_len = match read_request(socket, &mut request).await {
        Ok(len) => len,
        Err(ReadRequestError::Timeout) => {
            let _ = send_response(socket, Status::RequestTimeout, "text/plain", "Timeout").await;
            return false;
        }
        Err(ReadRequestError::Connection) => return true,
    };

    match parse_request(&request[..request_len]) {
        Ok(ParsedRequest::GetRoot) => {
            let _ = send_response(socket, Status::Ok, "text/html; charset=utf-8", SETUP_PAGE).await;
        }
        Ok(ParsedRequest::SaveCredentials(credentials)) => match store.save(&credentials) {
            Ok(()) => {
                info!("WiFi credentials saved from provisioning form");
                let response_sent = match send_response(
                    socket,
                    Status::Ok,
                    "text/html; charset=utf-8",
                    SAVE_OK_PAGE,
                )
                .await
                {
                    Ok(()) => true,
                    Err(_) => {
                        warn!("WiFi credential save succeeded but success response failed");
                        false
                    }
                };
                if post_save_action(true, response_sent) == PostSaveAction::Reboot {
                    Timer::after(REBOOT_DELAY).await;
                    info!("WiFi credentials saved; rebooting into station mode");
                    esp_hal::system::software_reset();
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
        Err(ParseError::InvalidCredentials) => {
            let _ = send_response(
                socket,
                Status::Ok,
                "text/html; charset=utf-8",
                INVALID_FORM_PAGE,
            )
            .await;
        }
        Err(error) => {
            let status = Status::from_parse_error(error);
            let _ = send_response(socket, status, "text/plain", status.reason()).await;
        }
    }
    false
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReadRequestError {
    Timeout,
    Connection,
}

async fn read_request(
    socket: &mut TcpSocket<'_>,
    request: &mut [u8; MAX_REQUEST_BYTES],
) -> Result<usize, ReadRequestError> {
    let mut len = 0usize;

    loop {
        match parse_request(&request[..len]) {
            Ok(_) => return Ok(len),
            Err(ParseError::EarlyEof) => {}
            Err(_) => return Ok(len),
        }

        if len == request.len() {
            return Ok(len);
        }

        let read = with_timeout(SOCKET_TIMEOUT, socket.read(&mut request[len..]))
            .await
            .map_err(|_| ReadRequestError::Timeout)?
            .map_err(|_| ReadRequestError::Connection)?;
        if read == 0 {
            return Ok(len);
        }
        len += read;
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
