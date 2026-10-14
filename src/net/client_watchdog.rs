//! Dedicated controller-lease watchdog.
//!
//! This task owns the pure lease policy.  It is deliberately independent from
//! UDP command execution so slow POM or scheduler work cannot delay bridge-off.

use embassy_futures::select::{Either, select};
use embassy_net::IpAddress;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Instant, Timer};

use crate::application::client_safety::{
    ClientSafetyPolicy, LeaseDecision, LeaseRequest, TripReason,
};
use crate::system_status::channels::{BootReadySender, FaultEventSender};
use crate::system_status::{BootReadyEvent, FaultEvent, announce_ready};

const Z21_KEEPALIVE_TIMEOUT_MS: u64 = 30_000;

pub(crate) type LeaseRequestChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, LeaseRequest<IpAddress>, 1>;
pub(crate) type LeaseResponseChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, LeaseDecision<IpAddress>, 1>;
pub(crate) type LeaseTripChannel =
    embassy_sync::channel::Channel<CriticalSectionRawMutex, LeaseTripNotification, 1>;
pub(crate) type LeaseRequestSender =
    Sender<'static, CriticalSectionRawMutex, LeaseRequest<IpAddress>, 1>;
pub(crate) type LeaseRequestReceiver =
    Receiver<'static, CriticalSectionRawMutex, LeaseRequest<IpAddress>, 1>;
pub(crate) type LeaseResponseSender =
    Sender<'static, CriticalSectionRawMutex, LeaseDecision<IpAddress>, 1>;
pub(crate) type LeaseResponseReceiver =
    Receiver<'static, CriticalSectionRawMutex, LeaseDecision<IpAddress>, 1>;
pub(crate) type LeaseTripSender =
    Sender<'static, CriticalSectionRawMutex, LeaseTripNotification, 1>;
pub(crate) type LeaseTripReceiver =
    Receiver<'static, CriticalSectionRawMutex, LeaseTripNotification, 1>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct LeaseTripNotification {
    pub(crate) client: IpAddress,
    pub(crate) reason: TripReason,
}

pub(crate) struct ClientWatchdogContext {
    pub(crate) request_receiver: LeaseRequestReceiver,
    pub(crate) response_sender: LeaseResponseSender,
    pub(crate) trip_sender: LeaseTripSender,
    pub(crate) fault_sender: FaultEventSender,
    pub(crate) ready_sender: BootReadySender,
}

#[embassy_executor::task]
pub(crate) async fn client_watchdog_task(context: ClientWatchdogContext) -> ! {
    let ClientWatchdogContext {
        request_receiver,
        response_sender,
        trip_sender,
        fault_sender,
        ready_sender,
    } = context;
    let mut policy = ClientSafetyPolicy::new(Z21_KEEPALIVE_TIMEOUT_MS);

    crate::track_authority::invalidate_and_disable();
    announce_ready(ready_sender, BootReadyEvent::LeaseWatchdog).await;

    loop {
        let fallback_wake = crate::clock::now_ms().saturating_add(Z21_KEEPALIVE_TIMEOUT_MS);
        let wake_ms = policy.next_deadline_ms().unwrap_or(fallback_wake);

        match select(
            request_receiver.receive(),
            Timer::at(Instant::from_millis(wake_ms)),
        )
        .await
        {
            Either::First(request) => {
                let decision = policy.handle(request, crate::clock::now_ms());
                apply_decision(decision, &fault_sender, &trip_sender).await;
                response_sender.send(decision).await;
            }
            Either::Second(_) => {
                let Some(decision) = policy.on_timer(crate::clock::now_ms()) else {
                    continue;
                };
                apply_decision(decision, &fault_sender, &trip_sender).await;
            }
        }
    }
}

async fn apply_decision(
    decision: LeaseDecision<IpAddress>,
    fault_sender: &FaultEventSender,
    trip_sender: &LeaseTripSender,
) {
    match decision {
        LeaseDecision::Acquired(permit)
        | LeaseDecision::Renewed(permit)
        | LeaseDecision::Maintained(permit) => {
            crate::track_authority::publish(permit);
        }
        LeaseDecision::Trip { client, reason } => {
            // Invalidate first and cut the bridge synchronously.  StopPressed
            // is a guaranteed delivery; only the network notification is
            // best-effort because it carries no safety state.
            crate::track_authority::invalidate_and_disable();
            fault_sender.send(FaultEvent::StopPressed).await;
            if trip_sender
                .try_send(LeaseTripNotification { client, reason })
                .is_err()
            {
                defmt::warn!("lease watchdog: trip notification dropped");
            }
            defmt::warn!("lease watchdog: controller lease tripped ({:?})", reason);
        }
        LeaseDecision::Rejected(_) => {}
    }
}
