//! The four update requests.

use dcc_ota::{
    Error, RequestError,
    admission::Operation,
    http::{Head, Route, Status},
    notice::Notice,
    pacing::{self, Pacing},
    runtime::{Admitted, Restart},
    writer::{Buffers, Writer},
};
use dcc_ota_format::package::{HEADER_LEN, PackageError};
use embassy_net::{Stack, tcp::TcpSocket};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};

use super::{
    body::Body,
    reply::{self, Content},
};
use crate::{
    clock::{deadline, now, since_boot},
    ota::{self, UpdateGuard},
};

const PAGE: &str = include_str!("page.html");
/// Head and package header must arrive within this time.
const HEAD_TIMEOUT: Duration = Duration::from_secs(2);
/// Bytes moved from the socket to flash per step.
const CHUNK: usize = 1024;
/// Pause between two chunks, so the other tasks keep running.
const CHUNK_YIELD: Duration = Duration::from_millis(1);

/// The 8 KiB of sector buffers live in a static: too large for a task stack,
/// and the lock serialises the two server instances.
static BUFFERS: Mutex<CriticalSectionRawMutex, Buffers> = Mutex::new(Buffers::new());

/// Answers one request on an accepted connection.
pub(super) async fn handle(stack: Stack<'static>, socket: &mut TcpSocket<'_>) -> Result<(), Error> {
    let start = Instant::now();
    let ip = stack
        .config_v4()
        .ok_or(Error::Unavailable)?
        .address
        .address();
    let mut body = Body::new();
    // Bad framing aborts the connection: an untrusted length is never drained.
    let head = body.read_head(socket, ip, start + HEAD_TIMEOUT).await?;
    match head.route {
        Route::Page => reply::send(socket, Status::Ok, Content::Html, PAGE).await,
        Route::Status => match ota::status_json() {
            Ok(json) => reply::send(socket, Status::Ok, Content::Json, &json).await,
            Err(error) => reply::error(socket, error).await,
        },
        Route::Check | Route::Upload => {
            let mut signed = [0; HEADER_LEN];
            if let Err(error) = body
                .read_exact(socket, &mut signed, start + HEAD_TIMEOUT)
                .await
            {
                return body.reject(socket, error).await;
            }
            let (guard, admitted) = match admit(&signed).await {
                Ok(admitted) => admitted,
                Err(error) => return body.reject(socket, error).await,
            };
            if head.route == Route::Check {
                drop(guard);
                // Extra bytes after the header are a malformed request.
                if socket.recv_queue() != 0 {
                    return reply::error(socket, RequestError::Malformed.into()).await;
                }
                ota::notify(Notice::Checked);
                return reply::ok(socket).await;
            }
            upload(socket, &mut body, &head, guard, admitted, start).await
        }
    }
}

/// Releases the lock over a package refused before any flash write, and
/// tells the operator the track waits; returns `error` for the reply.
fn refuse(guard: UpdateGuard, error: Error) -> Error {
    drop(guard);
    ota::notify(Notice::Rejected);
    error
}

/// Takes the update lock and authenticates the package header.
async fn admit(signed: &[u8; HEADER_LEN]) -> Result<(UpdateGuard, Admitted), Error> {
    let guard = ota::acquire(Operation::Update).await?;
    match guard.verify(signed) {
        Ok(admitted) => Ok((guard, admitted)),
        Err(error) => Err(refuse(guard, error)),
    }
}

/// Streams the image, publishes it and restarts; returns only on a
/// refusal or failure.
async fn upload(
    socket: &mut TcpSocket<'_>,
    body: &mut Body,
    head: &Head,
    mut guard: UpdateGuard,
    admitted: Admitted,
    start: Instant,
) -> Result<(), Error> {
    if !admitted.expects_body(head.content_length) {
        let error = refuse(guard, PackageError::Malformed.into());
        return body.reject(socket, error).await;
    }
    let deadline = since_boot(start) + pacing::UPLOAD_TIMEOUT;
    let written = async {
        let mut buffers = BUFFERS.lock().await;
        let mut writer = guard.begin(admitted, &mut buffers).await?;
        receive(socket, body, &mut writer, &mut guard, start).await?;
        guard.finish(&mut writer, deadline).await
    }
    .await;
    let image = match written {
        Ok(image) => image,
        Err(error) => {
            guard.abort(error).await;
            drop(guard);
            return body.reject(socket, error).await;
        }
    };
    let activated = guard
        .activate(image, async |result| {
            let sent = match result {
                Ok(()) => reply::ok(socket).await,
                Err(error) => reply::error(socket, error).await,
            };
            if let Err(error) = sent {
                defmt::warn!("update HTTP: final reply lost: {}", error.code());
            }
        })
        .await;
    match activated {
        Ok(Restart) => crate::restart::restart(),
        // The reply already went out; the running image never stopped.
        Err(error) => defmt::warn!("update HTTP: activation failed: {}", error.code()),
    }
    Ok(())
}

/// Moves the image from the socket to the writer, chunk by chunk.
async fn receive(
    socket: &mut TcpSocket<'_>,
    body: &mut Body,
    writer: &mut Writer<'_>,
    guard: &mut UpdateGuard,
    start: Instant,
) -> Result<(), Error> {
    let mut pacing = Pacing::new(since_boot(start));
    let mut chunk = [0; CHUNK];
    let mut last_data = now();
    while body.remaining() != 0 {
        pacing.check(now(), writer.received())?;
        let n = loop {
            let wake = deadline(pacing.next_wake(last_data));
            match body.read(socket, &mut chunk, wake).await {
                // The client is not silent for too long: the upload deadline
                // or a rate window ended, and the pacing decides.
                Err(Error::Timeout) if !Pacing::idle_expired(now(), last_data) => {
                    pacing.check(now(), writer.received())?;
                }
                result => break result?,
            }
        };
        last_data = now();
        guard.push(writer, &chunk[..n]).await?;
        pacing.check(last_data, writer.received())?;
        Timer::after(CHUNK_YIELD).await;
    }
    if socket.recv_queue() != 0 {
        return Err(RequestError::Malformed.into());
    }
    Ok(())
}
