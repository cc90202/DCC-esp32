//! Host-testable HTTP/form parser for WiFi provisioning.

use core::str;

#[cfg(test)]
use core::fmt::Write as _;
use heapless::String;

use crate::net::wifi_config::WifiCredentials;

// Limits sized for real phone browsers: Chrome/Safari send 15-25 headers
// (client hints included) totalling 600-900 bytes, with User-Agent lines
// well over 128 bytes. Tighter values bounce legitimate requests with 431.
pub(crate) const MAX_REQUEST_BYTES: usize = 2048;
pub(crate) const MAX_HEADER_BYTES: usize = MAX_REQUEST_BYTES - MAX_BODY_BYTES;
pub(crate) const MAX_BODY_BYTES: usize = 256;
pub(crate) const MAX_REQUEST_LINE_BYTES: usize = 256;
pub(crate) const MAX_HEADER_LINE_BYTES: usize = 512;
pub(crate) const MAX_HEADER_COUNT: usize = 32;

#[derive(Debug, PartialEq, Eq)]
pub(crate) enum ParsedRequest {
    GetRoot,
    SaveCredentials(WifiCredentials),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ParseError {
    BadRequest,
    EarlyEof,
    HeaderTooLarge,
    MethodNotAllowed,
    NotFound,
    PayloadTooLarge,
    RequestLineTooLong,
    UnsupportedContentType,
    UnsupportedVersion,
    LengthRequired,
    InvalidCredentials,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Method {
    Get,
    Post,
}

struct RequestLine<'a> {
    method: Method,
    path: &'a str,
}

#[derive(Default)]
struct HeaderState {
    content_length: Option<usize>,
    form_content_type: bool,
    count: usize,
}

struct RequestMetadata<'a> {
    request_line: RequestLine<'a>,
    header_state: HeaderState,
    body_start: usize,
}

pub(crate) fn parse_request(bytes: &[u8]) -> Result<ParsedRequest, ParseError> {
    if bytes.len() > MAX_REQUEST_BYTES {
        return Err(ParseError::PayloadTooLarge);
    }

    let metadata = parse_request_metadata(bytes)?;
    match (metadata.request_line.method, metadata.request_line.path) {
        (Method::Get, "/") => Ok(ParsedRequest::GetRoot),
        (Method::Get, _) => Err(ParseError::NotFound),
        (Method::Post, "/save") => parse_save_request(bytes, metadata),
        (Method::Post, _) => Err(ParseError::NotFound),
    }
}

fn parse_request_metadata(bytes: &[u8]) -> Result<RequestMetadata<'_>, ParseError> {
    let header_end = match find_header_end(bytes) {
        Some(header_end) => header_end,
        None if bytes.len() > MAX_HEADER_BYTES => return Err(ParseError::HeaderTooLarge),
        None => return Err(ParseError::EarlyEof),
    };
    let header_section_len = header_end + 4;
    if header_section_len > MAX_HEADER_BYTES {
        return Err(ParseError::HeaderTooLarge);
    }

    let headers = str::from_utf8(&bytes[..header_end]).map_err(|_| ParseError::BadRequest)?;
    let (request_line, header_lines) = headers.split_once("\r\n").ok_or(ParseError::BadRequest)?;
    if request_line.len() > MAX_REQUEST_LINE_BYTES {
        return Err(ParseError::RequestLineTooLong);
    }

    let request_line = parse_request_line(request_line)?;
    let header_state = parse_headers(header_lines)?;
    Ok(RequestMetadata {
        request_line,
        header_state,
        body_start: header_end + 4,
    })
}

fn parse_save_request(
    bytes: &[u8],
    metadata: RequestMetadata<'_>,
) -> Result<ParsedRequest, ParseError> {
    if !metadata.header_state.form_content_type {
        return Err(ParseError::UnsupportedContentType);
    }

    let content_length = metadata
        .header_state
        .content_length
        .ok_or(ParseError::LengthRequired)?;
    let body_end = metadata
        .body_start
        .checked_add(content_length)
        .ok_or(ParseError::PayloadTooLarge)?;
    if content_length > MAX_BODY_BYTES || body_end > MAX_REQUEST_BYTES {
        return Err(ParseError::PayloadTooLarge);
    }
    if bytes.len() < body_end {
        return Err(ParseError::EarlyEof);
    }

    parse_form(&bytes[metadata.body_start..body_end]).map(ParsedRequest::SaveCredentials)
}

fn find_header_end(bytes: &[u8]) -> Option<usize> {
    bytes.windows(4).position(|window| window == b"\r\n\r\n")
}

fn parse_request_line(line: &str) -> Result<RequestLine<'_>, ParseError> {
    let mut parts = line.split(' ');
    let method = match parts.next() {
        Some("GET") => Method::Get,
        Some("POST") => Method::Post,
        Some(_) => return Err(ParseError::MethodNotAllowed),
        None => return Err(ParseError::BadRequest),
    };
    let path = parts.next().ok_or(ParseError::BadRequest)?;
    let version = parts.next().ok_or(ParseError::BadRequest)?;
    if parts.next().is_some() {
        return Err(ParseError::BadRequest);
    }
    if !matches!(version, "HTTP/1.0" | "HTTP/1.1") {
        return Err(ParseError::UnsupportedVersion);
    }
    Ok(RequestLine { method, path })
}

fn parse_headers(lines: &str) -> Result<HeaderState, ParseError> {
    let mut state = HeaderState::default();
    if lines.is_empty() {
        return Ok(state);
    }

    for line in lines.split("\r\n") {
        if line.len() > MAX_HEADER_LINE_BYTES {
            return Err(ParseError::HeaderTooLarge);
        }
        state.count += 1;
        if state.count > MAX_HEADER_COUNT {
            return Err(ParseError::HeaderTooLarge);
        }

        let (name, value) = line.split_once(':').ok_or(ParseError::BadRequest)?;
        let value = value.trim();
        if name.eq_ignore_ascii_case("content-length") {
            if state.content_length.is_some() {
                return Err(ParseError::BadRequest);
            }
            state.content_length = Some(parse_content_length(value)?);
        } else if name.eq_ignore_ascii_case("content-type") {
            state.form_content_type = value.split(';').next().is_some_and(|content_type| {
                content_type
                    .trim()
                    .eq_ignore_ascii_case("application/x-www-form-urlencoded")
            });
        }
    }

    Ok(state)
}

fn parse_content_length(value: &str) -> Result<usize, ParseError> {
    if value.is_empty() {
        return Err(ParseError::BadRequest);
    }

    let mut length = 0usize;
    for byte in value.bytes() {
        if !byte.is_ascii_digit() {
            return Err(ParseError::BadRequest);
        }
        length = length
            .checked_mul(10)
            .and_then(|current| current.checked_add((byte - b'0') as usize))
            .ok_or(ParseError::PayloadTooLarge)?;
    }
    Ok(length)
}

fn parse_form(body: &[u8]) -> Result<WifiCredentials, ParseError> {
    let mut ssid: Option<String<32>> = None;
    let mut password: Option<String<64>> = None;

    for field in body.split(|byte| *byte == b'&') {
        if field.is_empty() {
            continue;
        }
        let (name, value) = split_once_byte(field, b'=').ok_or(ParseError::BadRequest)?;
        if name == b"ssid" {
            if ssid.is_some() {
                return Err(ParseError::BadRequest);
            }
            ssid = Some(decode_form_value(value)?);
        } else if name == b"password" {
            if password.is_some() {
                return Err(ParseError::BadRequest);
            }
            password = Some(decode_form_value(value)?);
        }
    }

    let ssid = ssid.ok_or(ParseError::BadRequest)?;
    let password = password.ok_or(ParseError::BadRequest)?;
    WifiCredentials::new(ssid.as_str(), password.as_str())
        .map_err(|_| ParseError::InvalidCredentials)
}

fn decode_form_value<const N: usize>(value: &[u8]) -> Result<String<N>, ParseError> {
    let mut decoded = heapless::Vec::<u8, N>::new();
    let mut index = 0usize;

    while index < value.len() {
        let byte = value[index];
        match byte {
            b'+' => decoded
                .push(b' ')
                .map_err(|_| ParseError::InvalidCredentials)?,
            b'%' => {
                let high = *value.get(index + 1).ok_or(ParseError::BadRequest)?;
                let low = *value.get(index + 2).ok_or(ParseError::BadRequest)?;
                decoded
                    .push((hex_value(high)? << 4) | hex_value(low)?)
                    .map_err(|_| ParseError::InvalidCredentials)?;
                index += 2;
            }
            _ => decoded
                .push(byte)
                .map_err(|_| ParseError::InvalidCredentials)?,
        }
        index += 1;
    }

    let decoded = str::from_utf8(decoded.as_slice()).map_err(|_| ParseError::BadRequest)?;
    String::try_from(decoded).map_err(|_| ParseError::InvalidCredentials)
}

fn hex_value(byte: u8) -> Result<u8, ParseError> {
    match byte {
        b'0'..=b'9' => Ok(byte - b'0'),
        b'a'..=b'f' => Ok(byte - b'a' + 10),
        b'A'..=b'F' => Ok(byte - b'A' + 10),
        _ => Err(ParseError::BadRequest),
    }
}

fn split_once_byte(bytes: &[u8], needle: u8) -> Option<(&[u8], &[u8])> {
    let index = bytes.iter().position(|byte| *byte == needle)?;
    Some((&bytes[..index], &bytes[index + 1..]))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn save_request(body: &str) -> heapless::String<512> {
        let mut request = heapless::String::new();
        core::write!(
            &mut request,
            "POST /save HTTP/1.1\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: {}\r\n\r\n{}",
            body.len(),
            body
        )
        .unwrap();
        request
    }

    #[test]
    fn get_root_is_accepted() {
        assert_eq!(
            parse_request(b"GET / HTTP/1.1\r\nHost: 192.168.4.1\r\n\r\n"),
            Ok(ParsedRequest::GetRoot)
        );
    }

    #[test]
    fn realistic_phone_browser_request_is_accepted() {
        // Regression: an Android Chrome GET / with client hints was bounced
        // with 431 by the previous 128-byte line / 16-header / 768-byte
        // section limits.
        let request = concat!(
            "GET / HTTP/1.1\r\n",
            "Host: 192.168.4.1\r\n",
            "Connection: keep-alive\r\n",
            "Cache-Control: max-age=0\r\n",
            "Upgrade-Insecure-Requests: 1\r\n",
            "User-Agent: Mozilla/5.0 (Linux; Android 14; Pixel 8 Pro) AppleWebKit/537.36 \
             (KHTML, like Gecko) Chrome/126.0.6478.122 Mobile Safari/537.36\r\n",
            "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,\
             image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.7\r\n",
            "sec-ch-ua: \"Not/A)Brand\";v=\"8\", \"Chromium\";v=\"126\", \"Google Chrome\";v=\"126\"\r\n",
            "sec-ch-ua-mobile: ?1\r\n",
            "sec-ch-ua-platform: \"Android\"\r\n",
            "Sec-Fetch-Site: none\r\n",
            "Sec-Fetch-Mode: navigate\r\n",
            "Sec-Fetch-User: ?1\r\n",
            "Sec-Fetch-Dest: document\r\n",
            "Accept-Encoding: gzip, deflate\r\n",
            "Accept-Language: it-IT,it;q=0.9,en-US;q=0.8,en;q=0.7\r\n",
            "\r\n"
        );

        assert_eq!(
            parse_request(request.as_bytes()),
            Ok(ParsedRequest::GetRoot)
        );
    }

    #[test]
    fn valid_save_form_returns_credentials() {
        let request = save_request("ssid=DCC-Lab&password=password123");
        let ParsedRequest::SaveCredentials(credentials) =
            parse_request(request.as_bytes()).unwrap()
        else {
            panic!("expected credentials")
        };

        assert_eq!(credentials.ssid(), "DCC-Lab");
        assert_eq!(credentials.password(), "password123");
    }

    #[test]
    fn form_decodes_plus_and_percent_escape() {
        let request = save_request("ssid=DCC+Lab&password=password%20123");
        let ParsedRequest::SaveCredentials(credentials) =
            parse_request(request.as_bytes()).unwrap()
        else {
            panic!("expected credentials")
        };

        assert_eq!(credentials.ssid(), "DCC Lab");
        assert_eq!(credentials.password(), "password 123");
    }

    #[test]
    fn duplicate_fields_are_rejected() {
        let request = save_request("ssid=A&ssid=B&password=password123");
        assert_eq!(
            parse_request(request.as_bytes()),
            Err(ParseError::BadRequest)
        );
    }

    #[test]
    fn duplicate_password_is_rejected() {
        let request = save_request("ssid=DCC-Lab&password=password123&password=password456");
        assert_eq!(
            parse_request(request.as_bytes()),
            Err(ParseError::BadRequest)
        );
    }

    #[test]
    fn unsupported_content_type_is_rejected() {
        let request =
            b"POST /save HTTP/1.1\r\nContent-Type: text/plain\r\nContent-Length: 3\r\n\r\nabc";
        assert_eq!(
            parse_request(request),
            Err(ParseError::UnsupportedContentType)
        );
    }

    #[test]
    fn body_larger_than_limit_is_rejected() {
        let request =
            b"POST /save HTTP/1.1\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: 257\r\n\r\n";
        assert_eq!(parse_request(request), Err(ParseError::PayloadTooLarge));
    }

    #[test]
    fn invalid_percent_escape_is_rejected() {
        let request = save_request("ssid=DCC%XZ&password=password123");
        assert_eq!(
            parse_request(request.as_bytes()),
            Err(ParseError::BadRequest)
        );
    }

    #[test]
    fn invalid_utf8_is_rejected() {
        let request = b"POST /save HTTP/1.1\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: 29\r\n\r\nssid=DCC%FF&password=password123";
        assert_eq!(parse_request(request), Err(ParseError::BadRequest));
    }

    #[test]
    fn duplicate_content_length_is_rejected() {
        let request = b"POST /save HTTP/1.1\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: 3\r\nContent-Length: 3\r\n\r\nabc";
        assert_eq!(parse_request(request), Err(ParseError::BadRequest));
    }

    #[test]
    fn early_eof_is_reported_for_partial_body() {
        let request = b"POST /save HTTP/1.1\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: 20\r\n\r\nssid=DCC";
        assert_eq!(parse_request(request), Err(ParseError::EarlyEof));
    }
}
