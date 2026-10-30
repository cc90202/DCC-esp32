//! Request/response correlation for the loco client.
//!
//! The client talks to a single-flight actor over a request channel and a
//! response channel. A reply can arrive after its request already timed out,
//! so every request carries an id from a task-local counter and only the reply
//! carrying that id is returned; anything else is discarded.

use core::cell::Cell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Duration, Instant, with_deadline};

/// Sends one request and waits for the reply carrying its id.
///
/// Drains stale replies, allocates the id, computes the response deadline,
/// sends (waiting for room at most until the same deadline), then waits for the
/// reply carrying that id until the deadline. Replies carrying any other id
/// are discarded. The counter advances once per call, also when the send
/// fails, and wraps at `u32::MAX`.
///
/// `build` receives the allocated id and the instant the deadline was
/// computed from, so the caller can derive its own execution deadline.
///
/// Returns `None` when the send fails or no matching reply arrives before
/// `response_timeout` has elapsed.
pub(crate) async fn correlated_request<Req, Resp, Id, const N: usize>(
    sender: &Sender<'static, CriticalSectionRawMutex, Req, N>,
    receiver: &Receiver<'static, CriticalSectionRawMutex, Resp, N>,
    counter: &Cell<u32>,
    make_id: fn(u32) -> Id,
    build: impl FnOnce(Id, Instant) -> Req,
    id_of: fn(&Resp) -> Id,
    response_timeout: Duration,
) -> Option<Resp>
where
    Id: Copy + PartialEq,
{
    while receiver.try_receive().is_ok() {}

    let raw_id = counter.get();
    counter.set(raw_id.wrapping_add(1));
    let request_id = make_id(raw_id);

    let now = Instant::now();
    let response_deadline = now + response_timeout;
    let request = build(request_id, now);

    with_deadline(response_deadline, sender.send(request))
        .await
        .ok()?;

    loop {
        let response = with_deadline(response_deadline, receiver.receive())
            .await
            .ok()?;
        if id_of(&response) == request_id {
            return Some(response);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use std::task::{Context, Poll, Wake, Waker};
    use std::thread;
    use std::time::Instant as StdInstant;

    use embassy_sync::channel::Channel;

    use super::*;

    /// Long enough that every timing assertion below keeps a margin of
    /// `TIMEOUT_MS / 3` (100 ms) against host scheduling delays.
    const TIMEOUT_MS: u64 = 300;
    const TIMEOUT: Duration = Duration::from_millis(TIMEOUT_MS);
    const TIMEOUT_STD: std::time::Duration = std::time::Duration::from_millis(TIMEOUT_MS);

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct TestId(u32);

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct TestReply {
        id: TestId,
        tag: u8,
    }

    type Requests = Channel<CriticalSectionRawMutex, TestId, 1>;
    type Replies = Channel<CriticalSectionRawMutex, TestReply, 1>;

    struct ThreadWake(thread::Thread);

    impl Wake for ThreadWake {
        fn wake(self: Arc<Self>) {
            self.0.unpark();
        }

        fn wake_by_ref(self: &Arc<Self>) {
            self.0.unpark();
        }
    }

    fn block_on<F: Future>(future: F) -> F::Output {
        let waker = Waker::from(Arc::new(ThreadWake(thread::current())));
        let mut context = Context::from_waker(&waker);
        let mut future = Box::pin(future);
        loop {
            match future.as_mut().poll(&mut context) {
                Poll::Ready(output) => return output,
                Poll::Pending => thread::park(),
            }
        }
    }

    #[expect(
        clippy::trivially_copy_pass_by_ref,
        reason = "matches the borrowed-reply extractor the correlator takes"
    )]
    fn reply_id(reply: &TestReply) -> TestId {
        reply.id
    }

    fn request(
        requests: &'static Requests,
        replies: &'static Replies,
        counter: &Cell<u32>,
    ) -> Option<TestReply> {
        block_on(correlated_request(
            &requests.sender(),
            &replies.receiver(),
            counter,
            TestId,
            |id, _now| id,
            reply_id,
            TIMEOUT,
        ))
    }

    #[test]
    fn stale_response_with_old_id_is_skipped() {
        static REQUESTS: Requests = Channel::new();
        static REPLIES: Replies = Channel::new();
        // A leftover reply whose old id equals the id about to be allocated,
        // as after the counter wrapped. Any other id would also be discarded
        // by the id match, so only this case shows that the drain runs.
        REPLIES
            .try_send(TestReply {
                id: TestId(7),
                tag: 1,
            })
            .expect("reply channel starts empty");

        let responder = thread::spawn(|| {
            let id = block_on(REQUESTS.receive());
            block_on(REPLIES.send(TestReply { id, tag: 2 }));
        });

        let reply = request(&REQUESTS, &REPLIES, &Cell::new(7));

        assert_eq!(
            reply,
            Some(TestReply {
                id: TestId(7),
                tag: 2
            })
        );
        responder.join().expect("responder thread must finish");
    }

    #[test]
    fn late_reply_to_a_timed_out_request_is_not_matched_to_the_next_one() {
        static REQUESTS: Requests = Channel::new();
        static REPLIES: Replies = Channel::new();
        let counter = Cell::new(40);

        let first = request(&REQUESTS, &REPLIES, &counter);
        assert_eq!(first, None);

        // The late reply to the first request lands between the two calls.
        REPLIES
            .try_send(TestReply {
                id: TestId(40),
                tag: 1,
            })
            .expect("reply channel is empty after the timeout");

        let responder = thread::spawn(|| {
            let first_id = block_on(REQUESTS.receive());
            let second_id = block_on(REQUESTS.receive());
            assert_eq!((first_id, second_id), (TestId(40), TestId(41)));
            // Another late copy for the first id arrives after the second
            // request was sent, ahead of the matching reply.
            block_on(REPLIES.send(TestReply {
                id: first_id,
                tag: 2,
            }));
            block_on(REPLIES.send(TestReply {
                id: second_id,
                tag: 3,
            }));
        });

        let second = request(&REQUESTS, &REPLIES, &counter);

        assert_eq!(
            second,
            Some(TestReply {
                id: TestId(41),
                tag: 3
            })
        );
        assert_eq!(counter.get(), 42);
        responder.join().expect("responder thread must finish");
    }

    #[test]
    fn timeout_returns_none() {
        static REQUESTS: Requests = Channel::new();
        static REPLIES: Replies = Channel::new();
        let started = StdInstant::now();

        let reply = request(&REQUESTS, &REPLIES, &Cell::new(0));

        assert_eq!(reply, None);
        assert!(started.elapsed() >= TIMEOUT_STD);
        assert_eq!(REQUESTS.try_receive(), Ok(TestId(0)));
    }

    #[test]
    fn blocking_send_shares_the_response_deadline() {
        static REQUESTS: Requests = Channel::new();
        static REPLIES: Replies = Channel::new();
        REQUESTS
            .try_send(TestId(99))
            .expect("request channel starts empty");

        // The consumer frees the request slot at 2/3 of the timeout, timed
        // from the same start instant as the request, and never replies.
        // A shared deadline returns at 1x the timeout; a fresh timeout for the
        // reply wait would return at 5/3. The 4/3 bound sits 100 ms from both,
        // and the slot is freed 100 ms before the deadline.
        let started = StdInstant::now();
        let consumer = thread::spawn(move || {
            let free_at = started + TIMEOUT_STD * 2 / 3;
            thread::sleep(free_at.saturating_duration_since(StdInstant::now()));
            assert_eq!(REQUESTS.try_receive(), Ok(TestId(99)));
        });

        let reply = request(&REQUESTS, &REPLIES, &Cell::new(0));
        let elapsed = started.elapsed();

        consumer.join().expect("consumer thread must finish");
        assert_eq!(reply, None);
        assert!(elapsed >= TIMEOUT_STD, "returned early: {elapsed:?}");
        assert!(
            elapsed < TIMEOUT_STD * 4 / 3,
            "send and reply wait must share one deadline: {elapsed:?}"
        );
        // The request did leave, so the reply wait was the part under test.
        assert_eq!(REQUESTS.try_receive(), Ok(TestId(0)));
    }

    #[test]
    fn blocking_send_on_a_channel_that_stays_full_returns_none_after_the_timeout() {
        static REQUESTS: Requests = Channel::new();
        static REPLIES: Replies = Channel::new();
        REQUESTS
            .try_send(TestId(99))
            .expect("request channel starts empty");
        let started = StdInstant::now();

        let reply = request(&REQUESTS, &REPLIES, &Cell::new(0));
        let elapsed = started.elapsed();

        assert_eq!(reply, None);
        assert!(elapsed >= TIMEOUT_STD, "returned early: {elapsed:?}");
        // A second full timeout after the failed send would reach 2x; the
        // bound sits 150 ms from both outcomes.
        assert!(
            elapsed < TIMEOUT_STD * 3 / 2,
            "waited too long: {elapsed:?}"
        );
    }
}
