//! Shared send step for Z21 replies and broadcasts.
//!
//! ESP32-C6 only, gated behind `#[cfg(target_arch = "riscv32")]` at the
//! `net` module declaration.

use embassy_net::IpEndpoint;
use embassy_net::udp::UdpSocket;

/// Sends the first `len` bytes of `buf` to `endpoint`, and nothing when `len` is zero.
///
/// A zero length is the normal silent-refusal path of the encoders and is not
/// an error. `Err` means only that `send_to` failed; each caller logs and
/// counts that failure in its own words.
pub(super) async fn send_reply(
    socket: &UdpSocket<'_>,
    buf: &[u8],
    len: usize,
    endpoint: IpEndpoint,
) -> Result<(), ()> {
    if len == 0 {
        return Ok(());
    }
    socket.send_to(&buf[..len], endpoint).await.map_err(|_| ())
}
