//! Readiness barrier that keeps track power disabled until critical tasks reply.

use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, with_timeout};

use crate::runtime_channels::{BootReadyReceiver, RuntimeReceiver};
use crate::system_status::BootReadyEvent;

use super::error::log_degraded_boot_error;
use super::{BootError, CriticalTaskInit};

pub(super) async fn wait_for_runtime_ready(
    receiver: BootReadyReceiver,
    failure_receiver: RuntimeReceiver<CriticalTaskInit, 4>,
) -> Result<(), BootError> {
    const READINESS_TIMEOUT: Duration = Duration::from_secs(10);
    const READY_DCC_ENGINE: u16 = 1 << 0;
    const READY_STATUS_LED: u16 = 1 << 1;
    const READY_SCHEDULER: u16 = 1 << 2;
    const READY_FAULT_MANAGER: u16 = 1 << 3;
    const READY_STOP_BUTTON: u16 = 1 << 4;
    const READY_RESUME_BUTTON: u16 = 1 << 5;
    const READY_SHORT_DETECTOR: u16 = 1 << 6;
    const REQUIRED_READY_MASK: u16 = READY_DCC_ENGINE
        | READY_STATUS_LED
        | READY_SCHEDULER
        | READY_FAULT_MANAGER
        | READY_STOP_BUTTON
        | READY_RESUME_BUTTON
        | READY_SHORT_DETECTOR;

    let mut ready_mask = 0u16;

    while ready_mask != REQUIRED_READY_MASK {
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
                    BootReadyEvent::DccEngine => ready_mask |= READY_DCC_ENGINE,
                    BootReadyEvent::StatusLed => ready_mask |= READY_STATUS_LED,
                    BootReadyEvent::Scheduler => ready_mask |= READY_SCHEDULER,
                    // The WiFi task reports this only after it has obtained a
                    // DHCP address. A missing home network must not reboot
                    // the firmware: the blue button remains available to
                    // switch to the setup access point in that situation.
                    BootReadyEvent::Net => {}
                    BootReadyEvent::FaultManager => ready_mask |= READY_FAULT_MANAGER,
                    BootReadyEvent::StopButton => ready_mask |= READY_STOP_BUTTON,
                    BootReadyEvent::ResumeButton => ready_mask |= READY_RESUME_BUTTON,
                    BootReadyEvent::ShortDetector => ready_mask |= READY_SHORT_DETECTOR,
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
