//! Readiness barrier that keeps track power disabled until critical tasks reply.

use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, with_timeout};

use crate::runtime_channels::{BootReadyReceiver, RuntimeReceiver};
use crate::system_status::BootReadyEvent;

use super::error::log_degraded_boot_error;
use super::{BootError, CriticalTaskInit};

#[repr(u8)]
enum CriticalReady {
    DccEngine = 1 << 0,
    StatusLed = 1 << 1,
    Scheduler = 1 << 2,
    FaultManager = 1 << 3,
    StopButton = 1 << 4,
    ResumeButton = 1 << 5,
    ShortDetector = 1 << 6,
}

#[derive(Default)]
struct ReadySet(u8);

impl ReadySet {
    const REQUIRED: u8 = (1 << 7) - 1;

    fn record(&mut self, task: CriticalReady) {
        self.0 |= task as u8;
    }

    fn is_complete(&self) -> bool {
        self.0 == Self::REQUIRED
    }
}

pub(super) async fn wait_for_runtime_ready(
    receiver: BootReadyReceiver,
    failure_receiver: RuntimeReceiver<CriticalTaskInit, 4>,
) -> Result<(), BootError> {
    const READINESS_TIMEOUT: Duration = Duration::from_secs(10);
    let mut ready = ReadySet::default();

    while !ready.is_complete() {
        match with_timeout(
            READINESS_TIMEOUT,
            select(receiver.receive(), failure_receiver.receive()),
        )
        .await
        {
            Ok(Either::First(event)) => {
                info!("boot: readiness received from {:?}", event);
                match event {
                    BootReadyEvent::DisplayReady => {}
                    BootReadyEvent::DisplayDegraded(reason) => {
                        log_degraded_boot_error(BootError::OptionalPeripheralInit(reason));
                    }
                    BootReadyEvent::DccEngine => ready.record(CriticalReady::DccEngine),
                    BootReadyEvent::StatusLed => ready.record(CriticalReady::StatusLed),
                    BootReadyEvent::Scheduler => ready.record(CriticalReady::Scheduler),
                    // The WiFi task reports this only after it has obtained a
                    // DHCP address. A missing home network must not reboot
                    // the firmware: the blue button remains available to
                    // switch to the setup access point in that situation.
                    BootReadyEvent::Net => {}
                    BootReadyEvent::FaultManager => ready.record(CriticalReady::FaultManager),
                    BootReadyEvent::StopButton => ready.record(CriticalReady::StopButton),
                    BootReadyEvent::ResumeButton => ready.record(CriticalReady::ResumeButton),
                    BootReadyEvent::ShortDetector => ready.record(CriticalReady::ShortDetector),
                }
            }
            Ok(Either::Second(failure)) => return Err(BootError::CriticalTaskInit(failure)),
            Err(_) => {
                return Err(BootError::CriticalTaskInit(
                    CriticalTaskInit::ReadinessTimeout,
                ));
            }
        }
    }

    Ok(())
}
