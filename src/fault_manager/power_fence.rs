//! Interruptible power-on fence exchange, shared by firmware and host tests.

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};

use crate::dcc::PowerGeneration;
use crate::system_status::FaultEvent;

/// The caller bounds the entire exchange with one timeout. Old acknowledgements
/// are drained without completing or restarting it. Events have priority even
/// when a matching ACK (or request capacity) is simultaneously available.
pub(super) async fn request_and_wait(
    generation: PowerGeneration,
    events: &Receiver<'_, CriticalSectionRawMutex, FaultEvent, 16>,
    requests: &Sender<'_, CriticalSectionRawMutex, PowerGeneration, 1>,
    acknowledgements: &Receiver<'_, CriticalSectionRawMutex, PowerGeneration, 1>,
) -> Result<(), FaultEvent> {
    if let Either::First(event) = select(events.receive(), requests.send(generation)).await {
        return Err(event);
    }
    loop {
        match select(events.receive(), acknowledgements.receive()).await {
            Either::First(event) => return Err(event),
            Either::Second(ack) if ack == generation => return Ok(()),
            Either::Second(_stale_ack) => {
                #[cfg(target_arch = "riscv32")]
                defmt::warn!(
                    "fault_manager: ignored stale power fence ack={} expected={}",
                    _stale_ack.get(),
                    generation.get()
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::system_status::FaultCause;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, Waker};
    use embassy_sync::channel::Channel;

    type Events = Channel<CriticalSectionRawMutex, FaultEvent, 16>;
    type Fences = Channel<CriticalSectionRawMutex, PowerGeneration, 1>;

    #[test]
    fn interrupted_resume_then_stale_ack_can_resume_again() {
        let events = Events::new();
        let requests = Fences::new();
        let acks = Fences::new();
        let (event_rx, request_tx, ack_rx) =
            (events.receiver(), requests.sender(), acks.receiver());
        let mut cx = Context::from_waker(Waker::noop());
        let first = PowerGeneration::new(1);
        {
            let mut wait = pin!(request_and_wait(first, &event_rx, &request_tx, &ack_rx));
            assert_eq!(wait.as_mut().poll(&mut cx), Poll::Pending);
            assert_eq!(requests.try_receive(), Ok(first));
            events.try_send(FaultEvent::StopPressed).unwrap();
            assert_eq!(
                wait.as_mut().poll(&mut cx),
                Poll::Ready(Err(FaultEvent::StopPressed))
            );
        }
        acks.try_send(first).unwrap();
        let second = PowerGeneration::new(2);
        let mut wait = pin!(request_and_wait(second, &event_rx, &request_tx, &ack_rx));
        assert_eq!(wait.as_mut().poll(&mut cx), Poll::Pending);
        assert_eq!(requests.try_receive(), Ok(second));
        acks.try_send(first).unwrap();
        assert_eq!(wait.as_mut().poll(&mut cx), Poll::Pending);
        acks.try_send(second).unwrap();
        assert_eq!(wait.as_mut().poll(&mut cx), Poll::Ready(Ok(())));
    }

    #[test]
    fn stop_and_fault_take_priority_over_matching_ack() {
        for event in [
            FaultEvent::StopPressed,
            FaultEvent::FaultLatched(FaultCause::TrackShort),
        ] {
            let events = Events::new();
            let requests = Fences::new();
            let acks = Fences::new();
            let (event_rx, request_tx, ack_rx) =
                (events.receiver(), requests.sender(), acks.receiver());
            let generation = PowerGeneration::new(4);
            let mut wait = pin!(request_and_wait(
                generation,
                &event_rx,
                &request_tx,
                &ack_rx
            ));
            let mut cx = Context::from_waker(Waker::noop());
            assert_eq!(wait.as_mut().poll(&mut cx), Poll::Pending);
            events.try_send(event).unwrap();
            acks.try_send(generation).unwrap();
            assert_eq!(wait.as_mut().poll(&mut cx), Poll::Ready(Err(event)));
        }
    }

    #[test]
    fn full_request_queue_remains_interruptible() {
        let events = Events::new();
        let requests = Fences::new();
        let acks = Fences::new();
        requests.try_send(PowerGeneration::new(1)).unwrap();
        let (event_rx, request_tx, ack_rx) =
            (events.receiver(), requests.sender(), acks.receiver());
        let mut wait = pin!(request_and_wait(
            PowerGeneration::new(2),
            &event_rx,
            &request_tx,
            &ack_rx
        ));
        let mut cx = Context::from_waker(Waker::noop());
        assert_eq!(wait.as_mut().poll(&mut cx), Poll::Pending);
        events.try_send(FaultEvent::StopPressed).unwrap();
        assert_eq!(
            wait.as_mut().poll(&mut cx),
            Poll::Ready(Err(FaultEvent::StopPressed))
        );
    }

    #[test]
    fn stale_acks_do_not_extend_outer_timeout() {
        use embassy_time::{Duration, with_timeout};
        let events = Events::new();
        let requests = Fences::new();
        let acks = Fences::new();
        let (event_rx, request_tx, ack_rx) =
            (events.receiver(), requests.sender(), acks.receiver());
        let mut wait = pin!(with_timeout(
            Duration::from_millis(5),
            request_and_wait(PowerGeneration::new(2), &event_rx, &request_tx, &ack_rx,)
        ));
        let mut cx = Context::from_waker(Waker::noop());
        assert_eq!(wait.as_mut().poll(&mut cx), Poll::Pending);
        std::thread::sleep(std::time::Duration::from_millis(10));
        acks.try_send(PowerGeneration::new(1)).unwrap();
        assert!(matches!(wait.as_mut().poll(&mut cx), Poll::Ready(Err(_))));
    }
}
