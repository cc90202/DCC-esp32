//! The request body: bytes already read with the head first, then the socket.

use core::net::Ipv4Addr;

use dcc_ota::{
    Error, RequestError,
    http::{self, Head, MAX_HEAD_BYTES},
};
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Instant};

use super::io;

/// Longest time spent discarding a refused body.
const DRAIN_TIMEOUT: Duration = Duration::from_secs(30);
/// Chunk used to discard a refused body.
const DRAIN_CHUNK: usize = 512;

/// Remaining declared bytes, including any body read together with the head.
pub(super) struct Body {
    bytes: [u8; MAX_HEAD_BYTES],
    pos: usize,
    end: usize,
    remaining: usize,
}

impl Body {
    pub(super) const fn new() -> Self {
        Self {
            bytes: [0; MAX_HEAD_BYTES],
            pos: 0,
            end: 0,
            remaining: 0,
        }
    }

    /// Bytes still to read.
    pub(super) const fn remaining(&self) -> usize {
        self.remaining
    }

    /// Reads and parses the head; its declared length then bounds the body.
    pub(super) async fn read_head(
        &mut self,
        socket: &mut TcpSocket<'_>,
        ip: Ipv4Addr,
        deadline: Instant,
    ) -> Result<Head, Error> {
        loop {
            if let Some(head) = http::parse(&self.bytes[..self.end], ip)? {
                self.pos = head.body_start;
                self.remaining = head.content_length;
                return Ok(head);
            }
            let n = io(deadline, socket.read(&mut self.bytes[self.end..])).await?;
            if n == 0 {
                return Err(Error::Network);
            }
            self.end += n;
        }
    }

    /// Reads up to `out.len()` body bytes before `deadline`.
    pub(super) async fn read(
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
            io(deadline, socket.read(&mut out[..limit])).await?
        };
        if n == 0 {
            return Err(Error::Network);
        }
        self.remaining -= n;
        Ok(n)
    }

    /// Fills `out` completely; a body that ends first is too short.
    pub(super) async fn read_exact(
        &mut self,
        socket: &mut TcpSocket<'_>,
        out: &mut [u8],
        deadline: Instant,
    ) -> Result<(), Error> {
        let mut used = 0;
        while used < out.len() {
            let n = self.read(socket, &mut out[used..], deadline).await?;
            if n == 0 {
                return Err(RequestError::TooShort.into());
            }
            used += n;
        }
        Ok(())
    }

    /// Discards the rest of the declared body, then answers with `error`,
    /// so the client reads the reason instead of a reset connection. The
    /// answer is tried even if discarding fails: a client that stopped
    /// sending may still read it.
    pub(super) async fn reject(
        &mut self,
        socket: &mut TcpSocket<'_>,
        error: Error,
    ) -> Result<(), Error> {
        let deadline = Instant::now() + DRAIN_TIMEOUT;
        let mut scratch = [0; DRAIN_CHUNK];
        let mut drained = Ok(());
        while self.remaining != 0 && drained.is_ok() {
            drained = self.read(socket, &mut scratch, deadline).await.map(|_| ());
        }
        let answered = super::reply::error(socket, error).await;
        drained.and(answered)
    }
}
