//! Bounded, single-request HTTP OTA service. No flash lock crosses network I/O.
mod head;

use core::{fmt::Write as _, future::Future};
use embassy_net::{Stack, tcp::TcpSocket};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer, with_deadline, with_timeout};

use crate::{
    ota::{
        self, Error, runtime,
        session::{Buffers, Writer},
    },
    runtime_channels::FaultEventSender,
};
use head::Route;

static BUFFERS: Mutex<CriticalSectionRawMutex, Buffers> = Mutex::new(Buffers {
    first: [255; ota::SECTOR_SIZE],
    sector: [255; ota::SECTOR_SIZE],
});
const PAGE: &str = include_str!("page.html");

fn beat(stack: Stack<'static>) {
    runtime::heartbeat(runtime::Heartbeat::Http);
    runtime::network_ready(stack.is_link_up() && stack.config_v4().is_some());
}

/// Poll without cancelling the operation every heartbeat interval.
async fn live<F: Future>(stack: Stack<'static>, future: F) -> F::Output {
    use embassy_futures::select::{Either, select};
    let mut future = core::pin::pin!(future);
    loop {
        beat(stack);
        match select(future.as_mut(), Timer::after_secs(1)).await {
            Either::First(result) => return result,
            Either::Second(_) => {}
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn server_task(stack: Stack<'static>, fault_sender: FaultEventSender) -> ! {
    let mut rx = [0; 4096];
    let mut tx = [0; 1536];
    let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);
    socket.set_timeout(Some(Duration::from_secs(10)));
    loop {
        // Keep accept alive across heartbeats: cancelling/restarting accept can
        // discard a connection that arrived between polls.
        let accepted = {
            use embassy_futures::select::{Either, select};
            let mut accept = core::pin::pin!(socket.accept(80));
            loop {
                match select(accept.as_mut(), Timer::after_secs(1)).await {
                    Either::First(result) => break result,
                    Either::Second(_) => {
                        beat(stack);
                        runtime::http_ready();
                    }
                }
            }
        };
        beat(stack);
        match accepted {
            Ok(()) => {
                runtime::http_ready();
                if live(stack, handle(stack, &mut socket, fault_sender))
                    .await
                    .is_err()
                {
                    socket.abort();
                } else {
                    socket.close();
                    let _ = live(stack, with_timeout(Duration::from_secs(2), socket.flush())).await;
                    socket.abort();
                }
            }
            Err(_) => {
                socket.abort();
                Timer::after_millis(100).await;
            }
        }
    }
}

/// Remaining declared bytes, including any body coalesced with the HTTP head.
struct Body {
    bytes: [u8; head::MAX_HEAD_BYTES],
    pos: usize,
    end: usize,
    remaining: usize,
}

impl Body {
    /// Establish framing before trusting the body length or draining a rejection.
    async fn read_head(
        &mut self,
        socket: &mut TcpSocket<'_>,
        ip: [u8; 4],
        deadline: Instant,
    ) -> Result<head::Head, Error> {
        loop {
            if let Some(head) = head::parse(&self.bytes[..self.end], ip)? {
                self.pos = head.body_start;
                self.remaining = head.content_length;
                return Ok(head);
            }
            let n = with_deadline(deadline, socket.read(&mut self.bytes[self.end..]))
                .await
                .map_err(|_| Error::Timeout)?
                .map_err(|_| Error::BadPackage)?;
            if n == 0 {
                return Err(Error::BadPackage);
            }
            self.end += n;
        }
    }

    async fn read(
        &mut self,
        socket: &mut TcpSocket<'_>,
        out: &mut [u8],
        deadline: Instant,
    ) -> Result<usize, Error> {
        if Instant::now() >= deadline {
            return Err(Error::Timeout);
        }
        let limit = out.len().min(self.remaining);
        if limit == 0 {
            return Ok(0);
        }
        let n = if self.pos < self.end {
            let n = limit.min(self.end - self.pos);
            out[..n].copy_from_slice(&self.bytes[self.pos..self.pos + n]);
            self.pos += n;
            n
        } else {
            with_deadline(deadline, socket.read(&mut out[..limit]))
                .await
                .map_err(|_| Error::Timeout)?
                .map_err(|_| Error::BadPackage)?
        };
        if n == 0 {
            return Err(Error::BadPackage);
        }
        self.remaining -= n;
        Ok(n)
    }

    async fn reject(&mut self, socket: &mut TcpSocket<'_>, error: Error) -> Result<(), Error> {
        let deadline = Instant::now() + Duration::from_secs(30);
        let mut scratch = [0; 512];
        while self.remaining != 0 {
            self.read(socket, &mut scratch, deadline).await?;
        }
        error_response(socket, error).await
    }
}

async fn handle(
    stack: Stack<'static>,
    socket: &mut TcpSocket<'_>,
    sender: FaultEventSender,
) -> Result<(), Error> {
    let start = Instant::now();
    let deadline = start + Duration::from_secs(2);
    let ip = stack
        .config_v4()
        .ok_or(Error::Unavailable)?
        .address
        .address()
        .octets();
    let mut body = Body {
        bytes: [0; head::MAX_HEAD_BYTES],
        pos: 0,
        end: 0,
        remaining: 0,
    };
    // Invalid framing aborts TCP: an untrusted length must never be drained.
    let head = body.read_head(socket, ip, deadline).await?;
    match head.route {
        Route::Page => return response(socket, "200 OK", "text/html; charset=utf-8", PAGE).await,
        Route::Status => {
            return response(
                socket,
                "200 OK",
                "application/json",
                runtime::status_json()?.as_str(),
            )
            .await;
        }
        _ => {}
    }
    let mut signed = [0; 256];
    let mut used = 0;
    while used < signed.len() {
        match body.read(socket, &mut signed[used..], deadline).await {
            Ok(n) => used += n,
            Err(e) => return body.reject(socket, e).await,
        }
    }
    let mut guard = match runtime::acquire(sender).await {
        Ok(guard) => guard,
        Err(e) => return body.reject(socket, e).await,
    };
    let manifest = match guard.verify(&signed) {
        Ok(manifest) => manifest,
        Err(e) => {
            drop(guard);
            return body.reject(socket, e).await;
        }
    };
    if head.route == Route::Check {
        drop(guard);
        if socket.recv_queue() != 0 {
            return Err(Error::BadPackage);
        }
        return response(socket, "200 OK", "application/json", "{\"ok\":true}").await;
    }
    if head.content_length != 256 + manifest.image_len as usize {
        drop(guard);
        return body.reject(socket, Error::BadPackage).await;
    }
    let result = async {
        let mut buffers = BUFFERS.lock().await;
        // OTA owns flash ordering, including early first-sector invalidation.
        let mut writer = guard.begin(manifest, &mut buffers).await?;
        upload(socket, &mut body, &mut writer, &mut guard, start).await?;
        guard
            .finish(&mut writer, start + Duration::from_secs(300))
            .await
    }
    .await;
    let image = match result {
        Ok(image) => image,
        Err(e) => {
            // Runtime owns cleanup failure -> recovery and transfer backoff.
            guard.abort(e).await;
            drop(guard);
            return body.reject(socket, e).await;
        }
    };
    // Once activation is attempted, selection may have partially succeeded.
    // Conservatively reset for BOTH results and never release the OTA guard.
    // The activated guard retains the inhibit even if READY/selection failed;
    // cleanup enters recovery if it cannot restore the confirmed selection.
    match guard.activate(image).await {
        Ok(()) => {
            let _ = response(socket, "200 OK", "application/json", "{\"ok\":true}").await;
        }
        Err(e) => {
            guard.abort(e).await;
            let _ = error_response(socket, e).await;
        }
    }
    esp_hal::system::software_reset();
}

struct Budget {
    end: Instant,
    window: Instant,
    window_bytes: u32,
}
impl Budget {
    fn check(&mut self, received: u32) -> Result<(), Error> {
        let now = Instant::now();
        if now >= self.end {
            return Err(Error::Timeout);
        }
        let elapsed = now - self.window;
        if elapsed >= Duration::from_secs(15) {
            // Include flash time, and scale the minimum if a step runs late.
            if u64::from(received - self.window_bytes) * 1000 < elapsed.as_millis() * 1024 {
                return Err(Error::Timeout);
            }
            self.window = now;
            self.window_bytes = received;
        }
        Ok(())
    }
}

async fn upload(
    socket: &mut TcpSocket<'_>,
    body: &mut Body,
    writer: &mut Writer<'_>,
    guard: &mut runtime::UpdateGuard,
    start: Instant,
) -> Result<(), Error> {
    let mut budget = Budget {
        end: start + Duration::from_secs(300),
        window: start,
        window_bytes: 0,
    };
    let mut chunk = [0; 1024];
    let mut last_data = Instant::now();
    while body.remaining != 0 {
        budget.check(writer.received())?;
        let idle = last_data + Duration::from_secs(10);
        let n = loop {
            let window_end = budget.window + Duration::from_secs(15);
            let deadline = budget.end.min(window_end).min(idle);
            match body.read(socket, &mut chunk, deadline).await {
                Err(Error::Timeout) if Instant::now() >= window_end && Instant::now() < idle => {
                    budget.check(writer.received())?;
                }
                result => break result?,
            }
        };
        last_data = Instant::now();
        guard.push(writer, &chunk[..n]).await?;
        let received = writer.received();
        budget.check(received)?;
        Timer::after_millis(1).await;
    }
    if socket.recv_queue() != 0 {
        return Err(Error::BadPackage);
    }
    Ok(())
}

async fn error_response(socket: &mut TcpSocket<'_>, error: Error) -> Result<(), Error> {
    let status = match error {
        Error::Forbidden => "403 Forbidden",
        Error::TooLarge => "413 Content Too Large",
        Error::Timeout => "408 Request Timeout",
        Error::Busy | Error::TrackOn | Error::BootNotReady | Error::PendingVerify => "409 Conflict",
        Error::Flash | Error::Corrupt | Error::Unavailable => "503 Service Unavailable",
        _ => "400 Bad Request",
    };
    let mut json = heapless::String::<96>::new();
    write!(json, "{{\"ok\":false,\"error\":\"{}\"}}", error.code()).map_err(|_| Error::Corrupt)?;
    response(socket, status, "application/json", &json).await
}

async fn response(
    socket: &mut TcpSocket<'_>,
    status: &str,
    content_type: &str,
    body: &str,
) -> Result<(), Error> {
    let mut head = heapless::String::<256>::new();
    write!(head, "HTTP/1.1 {status}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nConnection: close\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\n\r\n", body.len()).map_err(|_| Error::Corrupt)?;
    let deadline = Instant::now() + Duration::from_secs(2);
    for bytes in [head.as_bytes(), body.as_bytes()] {
        let mut offset = 0;
        while offset < bytes.len() {
            let n = with_deadline(deadline, socket.write(&bytes[offset..]))
                .await
                .map_err(|_| Error::Timeout)?
                .map_err(|_| Error::BadPackage)?;
            if n == 0 {
                return Err(Error::BadPackage);
            }
            offset += n;
        }
    }
    with_deadline(deadline, socket.flush())
        .await
        .map_err(|_| Error::Timeout)?
        .map_err(|_| Error::BadPackage)
}
