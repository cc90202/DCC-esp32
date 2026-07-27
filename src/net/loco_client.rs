//! Correlated locomotive-state client for the network task.

use core::cell::Cell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Duration, Instant, with_timeout};

use crate::dcc::{
    LocoRequest, LocoRequestDeadline, LocoRequestId, LocoRequestMessage, LocoResponse,
};

const LOCO_EXECUTION_TIMEOUT_MS: u64 = 200;
const LOCO_RESPONSE_TIMEOUT_MS: u64 = 250;
const _: () = assert!(LOCO_EXECUTION_TIMEOUT_MS < LOCO_RESPONSE_TIMEOUT_MS);
const LOCO_EXECUTION_TIMEOUT: Duration = Duration::from_millis(LOCO_EXECUTION_TIMEOUT_MS);
const LOCO_RESPONSE_TIMEOUT: Duration = Duration::from_millis(LOCO_RESPONSE_TIMEOUT_MS);

static LOCO_RESPONSE_TIMEOUT_COUNT: AtomicU32 = AtomicU32::new(0);

#[must_use]
pub(crate) fn loco_response_timeout_count() -> u32 {
    LOCO_RESPONSE_TIMEOUT_COUNT.load(Ordering::Relaxed)
}

fn next_loco_request_id(counter: &Cell<u32>) -> LocoRequestId {
    let value = counter.get();
    counter.set(value.wrapping_add(1));
    LocoRequestId::new(value)
}

pub(super) async fn request_loco(
    request_sender: &Sender<'static, CriticalSectionRawMutex, LocoRequestMessage, 1>,
    response_receiver: &Receiver<'static, CriticalSectionRawMutex, LocoResponse, 1>,
    next_request_id: &Cell<u32>,
    request: LocoRequest,
) -> Option<LocoResponse> {
    while response_receiver.try_receive().is_ok() {}

    let request_id = next_loco_request_id(next_request_id);
    let now = Instant::now();
    let response_deadline = now + LOCO_RESPONSE_TIMEOUT;
    let message = LocoRequestMessage {
        request_id,
        request,
        deadline: LocoRequestDeadline::from_ticks((now + LOCO_EXECUTION_TIMEOUT).as_ticks()),
    };

    if with_timeout(LOCO_RESPONSE_TIMEOUT, request_sender.send(message))
        .await
        .is_err()
    {
        LOCO_RESPONSE_TIMEOUT_COUNT.fetch_add(1, Ordering::Relaxed);
        return None;
    }

    loop {
        let remaining = response_deadline.saturating_duration_since(Instant::now());
        let Ok(response) = with_timeout(remaining, response_receiver.receive()).await else {
            LOCO_RESPONSE_TIMEOUT_COUNT.fetch_add(1, Ordering::Relaxed);
            return None;
        };
        if response.request_id == request_id {
            return Some(response);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use std::task::{Context, Poll, Wake, Waker};
    use std::thread;

    use embassy_sync::channel::Channel;

    use super::*;
    use crate::application::LocoState;
    use crate::application::loco_projection::apply_loco_result;
    use crate::dcc::{
        DccAddress, Direction, FunctionState, LocoRequestResult, LocoSnapshot, LogicalSpeed,
        SpeedFormat,
    };

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

    fn address() -> DccAddress {
        DccAddress::new_short(3).expect("valid test address")
    }

    #[test]
    fn async_client_ignores_stale_response_and_accepts_the_correlated_response() {
        static REQUESTS: Channel<CriticalSectionRawMutex, LocoRequestMessage, 1> = Channel::new();
        static RESPONSES: Channel<CriticalSectionRawMutex, LocoResponse, 1> = Channel::new();

        let responder = thread::spawn(|| {
            let request = block_on(REQUESTS.receiver().receive());
            block_on(RESPONSES.sender().send(LocoResponse {
                request_id: LocoRequestId::new(request.request_id.value().wrapping_sub(1)),
                result: LocoRequestResult::NotFound,
            }));
            block_on(RESPONSES.sender().send(LocoResponse {
                request_id: request.request_id,
                result: LocoRequestResult::NotFound,
            }));
        });

        let response = block_on(request_loco(
            &REQUESTS.sender(),
            &RESPONSES.receiver(),
            &Cell::new(7),
            LocoRequest::GetState { address: address() },
        ));

        assert!(matches!(
            response,
            Some(LocoResponse {
                request_id,
                result: LocoRequestResult::NotFound,
            }) if request_id == LocoRequestId::new(7)
        ));
        responder.join().expect("responder thread must finish");
    }

    #[test]
    fn async_client_times_out_when_request_channel_remains_full() {
        static REQUESTS: Channel<CriticalSectionRawMutex, LocoRequestMessage, 1> = Channel::new();
        static RESPONSES: Channel<CriticalSectionRawMutex, LocoResponse, 1> = Channel::new();
        REQUESTS
            .sender()
            .try_send(LocoRequestMessage {
                request_id: LocoRequestId::new(1),
                request: LocoRequest::GetState { address: address() },
                deadline: LocoRequestDeadline::from_ticks(u64::MAX),
            })
            .expect("test request channel starts empty");
        let timeout_count_before = loco_response_timeout_count();

        let response = block_on(request_loco(
            &REQUESTS.sender(),
            &RESPONSES.receiver(),
            &Cell::new(2),
            LocoRequest::GetState { address: address() },
        ));

        assert_eq!(response, None);
        assert!(
            loco_response_timeout_count().wrapping_sub(timeout_count_before) >= 1,
            "send timeout must be observable"
        );
        assert!(REQUESTS.receiver().try_receive().is_ok());
    }

    #[test]
    fn wrong_response_id_then_timeout_does_not_change_projection() {
        static REQUESTS: Channel<CriticalSectionRawMutex, LocoRequestMessage, 1> = Channel::new();
        static RESPONSES: Channel<CriticalSectionRawMutex, LocoResponse, 1> = Channel::new();

        let responder = thread::spawn(|| {
            let request = block_on(REQUESTS.receiver().receive());
            block_on(RESPONSES.sender().send(LocoResponse {
                request_id: LocoRequestId::new(request.request_id.value().wrapping_add(1)),
                result: LocoRequestResult::Updated(LocoSnapshot {
                    address: address(),
                    speed: LogicalSpeed::new(20, SpeedFormat::Speed128).unwrap(),
                    direction: Direction::Reverse,
                    functions: FunctionState::from_bits(1).unwrap(),
                }),
            }));
        });

        let mut projection = [None; 12];
        projection[0] = Some(LocoState {
            address: address(),
            speed: LogicalSpeed::new(3, SpeedFormat::Speed28).unwrap(),
            direction: Direction::Forward,
            functions: FunctionState::empty(),
        });
        let before = projection;

        let response = block_on(request_loco(
            &REQUESTS.sender(),
            &RESPONSES.receiver(),
            &Cell::new(11),
            LocoRequest::GetState { address: address() },
        ));
        if let Some(response) = response {
            apply_loco_result(&mut projection, response.result).unwrap();
        }

        assert_eq!(response, None);
        assert_eq!(projection, before);
        responder.join().expect("responder thread must finish");
    }

    #[test]
    fn async_client_keeps_two_consecutive_requests_separate_across_id_wrap() {
        static REQUESTS: Channel<CriticalSectionRawMutex, LocoRequestMessage, 1> = Channel::new();
        static RESPONSES: Channel<CriticalSectionRawMutex, LocoResponse, 1> = Channel::new();

        let responder = thread::spawn(|| {
            for _ in 0..2 {
                let request = block_on(REQUESTS.receiver().receive());
                block_on(RESPONSES.sender().send(LocoResponse {
                    request_id: request.request_id,
                    result: LocoRequestResult::NotFound,
                }));
            }
        });
        let next_request_id = Cell::new(u32::MAX);

        let first = block_on(request_loco(
            &REQUESTS.sender(),
            &RESPONSES.receiver(),
            &next_request_id,
            LocoRequest::GetState { address: address() },
        ));
        let second = block_on(request_loco(
            &REQUESTS.sender(),
            &RESPONSES.receiver(),
            &next_request_id,
            LocoRequest::GetState { address: address() },
        ));

        assert!(matches!(
            first,
            Some(LocoResponse { request_id, .. }) if request_id == LocoRequestId::new(u32::MAX)
        ));
        assert!(matches!(
            second,
            Some(LocoResponse { request_id, .. }) if request_id == LocoRequestId::new(0)
        ));
        responder.join().expect("responder thread must finish");
    }
}
