//! Allocation-free OTA HTTP framing. The caller streams the body separately.
use crate::ota::{Error, SLOT_SIZE};

pub(crate) const MAX_HEAD_BYTES: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Route {
    Page,
    Status,
    Check,
    Upload,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Head {
    pub(crate) route: Route,
    pub(crate) content_length: usize,
    pub(crate) body_start: usize,
}

pub(crate) fn parse(bytes: &[u8], ip: [u8; 4]) -> Result<Option<Head>, Error> {
    let Some(end) = bytes.windows(4).position(|w| w == b"\r\n\r\n") else {
        return if bytes.len() >= MAX_HEAD_BYTES {
            Err(Error::TooLarge)
        } else {
            Ok(None)
        };
    };
    let body_start = end + 4;
    if body_start > MAX_HEAD_BYTES {
        return Err(Error::TooLarge);
    }
    let text = core::str::from_utf8(&bytes[..end]).map_err(|_| Error::BadPackage)?;
    let mut lines = text.split("\r\n");
    let route = parse_route(lines.next().ok_or(Error::BadPackage)?)?;
    let headers = Headers::parse(lines)?;
    headers.authorize(route, ip)?;
    let content_length = route_length(route, headers.length)?;
    if bytes.len() - body_start > content_length {
        return Err(Error::BadPackage);
    }
    Ok(Some(Head {
        route,
        content_length,
        body_start,
    }))
}

fn parse_route(line: &str) -> Result<Route, Error> {
    let mut request = line.split(' ');
    let method = request.next().ok_or(Error::BadPackage)?;
    let path = request.next().ok_or(Error::BadPackage)?;
    let version = request.next().ok_or(Error::BadPackage)?;
    if request.next().is_some() || !matches!(version, "HTTP/1.0" | "HTTP/1.1") {
        return Err(Error::BadPackage);
    }
    Ok(match (method, path) {
        ("GET", "/update") => Route::Page,
        ("GET", "/update/status") => Route::Status,
        ("POST", "/update/check") => Route::Check,
        ("POST", "/update") => Route::Upload,
        _ => return Err(Error::Forbidden),
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
    fn parse(lines: impl Iterator<Item = &'a str>) -> Result<Self, Error> {
        let mut headers = Self::default();
        for line in lines {
            let (name, value) = parse_header(line)?;
            if name.eq_ignore_ascii_case("content-length") {
                if headers.length.is_some() {
                    return Err(Error::BadPackage);
                }
                headers.length = Some(decimal(value)?);
            } else if name.eq_ignore_ascii_case("transfer-encoding") {
                return Err(Error::BadPackage);
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

    fn authorize(&self, route: Route, ip: [u8; 4]) -> Result<(), Error> {
        if !self.host.is_some_and(|h| authority(h, ip))
            || self
                .origin
                .is_some_and(|o| !o.strip_prefix("http://").is_some_and(|h| authority(h, ip)))
            || (matches!(route, Route::Check | Route::Upload) && self.token != Some("1"))
        {
            return Err(Error::Forbidden);
        }
        Ok(())
    }
}

fn parse_header(line: &str) -> Result<(&str, &str), Error> {
    let (name, value) = line.split_once(':').ok_or(Error::BadPackage)?;
    if name.is_empty() || !name.bytes().all(is_token) {
        return Err(Error::BadPackage);
    }
    if !value.bytes().all(|b| b == b'\t' || (32..=126).contains(&b)) {
        return Err(Error::BadPackage);
    }
    Ok((name, value.trim_matches([' ', '\t'])))
}

fn route_length(route: Route, length: Option<usize>) -> Result<usize, Error> {
    Ok(match route {
        Route::Page | Route::Status => {
            if length.unwrap_or(0) != 0 {
                return Err(Error::BadPackage);
            }
            0
        }
        Route::Check => {
            if length != Some(256) {
                return Err(Error::BadPackage);
            }
            256
        }
        Route::Upload => {
            let n = length.ok_or(Error::BadPackage)?;
            if n > 256 + SLOT_SIZE as usize {
                return Err(Error::TooLarge);
            }
            if n < 256 + 4096 {
                return Err(Error::BadPackage);
            }
            n
        }
    })
}

fn is_token(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b"!#$%&'*+-.^_`|~".contains(&b)
}

fn unique<'a>(slot: &mut Option<&'a str>, value: &'a str) -> Result<(), Error> {
    if slot.replace(value).is_some() {
        Err(Error::BadPackage)
    } else {
        Ok(())
    }
}

fn decimal(value: &str) -> Result<usize, Error> {
    if value.is_empty() || !value.bytes().all(|b| b.is_ascii_digit()) {
        return Err(Error::BadPackage);
    }
    value.bytes().try_fold(0usize, |n, b| {
        n.checked_mul(10)
            .and_then(|n| n.checked_add((b - b'0') as usize))
            .ok_or(Error::TooLarge)
    })
}

fn authority(value: &str, ip: [u8; 4]) -> bool {
    let value = value.strip_suffix(":80").unwrap_or(value);
    let mut parts = value.split('.');
    for octet in ip {
        let Some(part) = parts.next() else {
            return false;
        };
        if part.len() > 1 && part.starts_with('0') {
            return false;
        }
        if decimal(part) != Ok(octet as usize) {
            return false;
        }
    }
    parts.next().is_none()
}

#[cfg(test)]
mod tests {
    use super::*;
    const IP: [u8; 4] = [192, 168, 1, 20];
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
                Err(Error::Forbidden)
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
                Err(Error::Forbidden)
            );
        }
    }
    #[test]
    fn bounds_and_framing() {
        assert_eq!(parse(b"GET /update", IP), Ok(None));
        assert_eq!(parse(&[b' '; 1024], IP), Err(Error::TooLarge));
        let prefix = request("GET", "/update", "X-Padding: ");
        let mut r = prefix.trim_end_matches("\r\n").to_owned();
        r.push_str(&"a".repeat(1020 - r.len()));
        r.push_str("\r\n\r\n");
        assert!(parse(r.as_bytes(), IP).is_ok());
        r.insert(100, 'a');
        assert_eq!(parse(r.as_bytes(), IP), Err(Error::TooLarge));
        for n in ["4351", "3145985", "99999999999999999999999999999"] {
            assert!(
                parse(
                    request(
                        "POST",
                        "/update",
                        &std::format!("X-DCC-OTA: 1\r\nContent-Length: {n}\r\n")
                    )
                    .as_bytes(),
                    IP
                )
                .is_err()
            );
        }
        let mut r = request(
            "POST",
            "/update/check",
            "X-DCC-OTA: 1\r\nContent-Length: 256\r\n",
        );
        r.push_str(&"x".repeat(257));
        assert_eq!(parse(r.as_bytes(), IP), Err(Error::BadPackage));
        for (method, path) in [
            ("OPTIONS", "/update"),
            ("PUT", "/update"),
            ("GET", "/update/check"),
            ("GET", "/unknown"),
        ] {
            assert_eq!(
                parse(request(method, path, "").as_bytes(), IP),
                Err(Error::Forbidden)
            );
        }
    }
}
