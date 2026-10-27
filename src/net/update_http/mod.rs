//! Bounded, single-request HTTP server for the update page and uploads.
//!
//! The flash is never locked across network I/O, and every read has a
//! deadline. Request framing is [`dcc_ota::http`]; this module only moves
//! bytes between the socket and the update runtime.

mod body;
mod reply;
mod requests;

use core::future::Future;
use dcc_ota::Error;
use embassy_futures::select::{Either, select};
use embassy_net::{Stack, tcp::TcpSocket};
use embassy_time::{Duration, Instant, Timer, with_deadline, with_timeout};

use crate::liveness::{self, Task};

/// Socket idle timeout.
const SOCKET_TIMEOUT: Duration = Duration::from_secs(10);
/// Heartbeat period while waiting.
const BEAT_PERIOD: Duration = Duration::from_secs(1);
/// Longest wait for the last reply bytes after closing.
const CLOSE_FLUSH_TIMEOUT: Duration = Duration::from_secs(2);
/// Pause after a failed accept before listening again.
const ACCEPT_RETRY: Duration = Duration::from_millis(100);
/// Time the stack gets to send the RST of an aborted connection.
const ABORT_SETTLE: Duration = Duration::from_millis(50);
/// Receive window: room for a full sector of image data in flight.
const RX_BUFFER: usize = 4096;
/// Transmit window: holds the status JSON in one go; the larger page
/// streams through it.
const TX_BUFFER: usize = dcc_ota::status::JSON_CAPACITY;

/// Runs one socket operation before `deadline`.
async fn io<T, E: core::fmt::Debug>(
    deadline: Instant,
    operation: impl Future<Output = Result<T, E>>,
) -> Result<T, Error> {
    with_deadline(deadline, operation)
        .await
        .map_err(|_| Error::Timeout)?
        .map_err(|error| {
            defmt::warn!("update HTTP: socket error {}", defmt::Debug2Format(&error));
            Error::Network
        })
}

fn beat(stack: Stack<'static>, task: Task) {
    liveness::beat(task);
    liveness::network_ready(stack.is_link_up() && stack.config_v4().is_some());
}

/// Polls `future` to completion, beating every second without cancelling it.
async fn live<F: Future>(stack: Stack<'static>, task: Task, future: F) -> F::Output {
    let mut future = core::pin::pin!(future);
    loop {
        beat(stack, task);
        if let Either::First(result) = select(future.as_mut(), Timer::after(BEAT_PERIOD)).await {
            return result;
        }
    }
}

/// Serves one connection at a time on port 80, beating as `task`.
#[embassy_executor::task(pool_size = 2)]
pub async fn server_task(stack: Stack<'static>, task: Task) -> ! {
    let mut rx = [0; RX_BUFFER];
    let mut tx = [0; TX_BUFFER];
    let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);
    socket.set_timeout(Some(SOCKET_TIMEOUT));
    loop {
        // Keep one accept alive across heartbeats: restarting it could drop
        // a connection that arrived between two polls.
        let accepted = {
            let mut accept = core::pin::pin!(socket.accept(80));
            loop {
                match select(accept.as_mut(), Timer::after(BEAT_PERIOD)).await {
                    Either::First(result) => break result,
                    Either::Second(()) => beat(stack, task),
                }
            }
        };
        beat(stack, task);
        if accepted.is_err() {
            socket.abort();
            Timer::after(ACCEPT_RETRY).await;
            continue;
        }
        if live(stack, task, requests::handle(stack, &mut socket))
            .await
            .is_err()
        {
            socket.abort();
            // Let the stack send the RST before `accept` reuses the socket,
            // or the client is left waiting on a connection nobody serves.
            Timer::after(ABORT_SETTLE).await;
            continue;
        }
        socket.close();
        let _ = live(
            stack,
            task,
            with_timeout(CLOSE_FLUSH_TIMEOUT, socket.flush()),
        )
        .await;
        socket.abort();
    }
}
