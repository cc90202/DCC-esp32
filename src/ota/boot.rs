//! Start-up of the update runtime on this device: open the flash, start the
//! monitor and run the boot check of [`dcc_ota::runtime`].

use dcc_ota::{
    Error,
    journal::ResetCause,
    layout::{FLASH_SIZE, PARTITIONS, Slot},
    runtime::Device as _,
    status::BootId,
};
use embedded_storage::nor_flash::ReadNorFlash;
use esp_bootloader_esp_idf::partitions::{PARTITION_TABLE_MAX_LEN, read_partition_table};

use super::{
    device::{Esp, enter_recovery, obey, runtime},
    interlock, monitor, notify,
};
use crate::{
    flash::Flash,
    runtime_channels::{DisplaySender, FaultEventSender},
};

/// Opens the flash, starts the watchdog and reconciles the journal before
/// anything else touches flash or the track.
pub(crate) async fn start(
    spawner: embassy_executor::Spawner,
    flash: esp_hal::peripherals::FLASH<'static>,
    lpwr: esp_hal::peripherals::LPWR<'static>,
    sender: FaultEventSender,
    display: DisplaySender,
) {
    let rng = esp_hal::rng::Rng::new();
    let boot_id = BootId::new((u64::from(rng.random()) << 32) | u64::from(rng.random()));
    notify::attach(display);
    interlock::attach(sender);
    Esp.state(|s| s.set_boot_id(boot_id));
    crate::flash::init(flash);
    if spawner.spawn(monitor::monitor_task(lpwr)).is_err() {
        enter_recovery(Error::Unavailable);
        return;
    }
    obey(runtime().start(booted_slot, reset_cause()).await);
}

/// The slot the bootloader started, after checking that the partition
/// table is exactly the required layout.
fn booted_slot(flash: &mut Flash) -> Result<Slot, Error> {
    if flash.capacity() != FLASH_SIZE {
        return Err(Error::Unavailable);
    }
    let mut bytes = [0; PARTITION_TABLE_MAX_LEN];
    let table = read_partition_table(flash, &mut bytes).map_err(|error| {
        defmt::error!("OTA: partition table unreadable: {}", error);
        Error::Corrupt
    })?;
    let matches_layout = table.len() == PARTITIONS.len()
        && PARTITIONS.iter().all(|required| {
            table.iter().any(|p| {
                p.label_as_str() == required.name
                    && p.raw_type() == required.kind
                    && p.raw_subtype() == required.subtype
                    && p.offset() == required.offset
                    && p.len() == required.size
            })
        });
    if !matches_layout {
        return Err(Error::Corrupt);
    }
    let booted = table
        .booted_partition()
        .map_err(|_| Error::Corrupt)?
        .ok_or(Error::Corrupt)?;
    Slot::at(booted.offset()).ok_or(Error::Corrupt)
}

/// Why the chip last restarted: a power-on or a brown-out is a power loss.
fn reset_cause() -> ResetCause {
    use esp_hal::rtc_cntl::SocResetReason;
    match esp_hal::system::reset_reason() {
        Some(SocResetReason::ChipPowerOn | SocResetReason::SysBrownOut) => ResetCause::PowerLoss,
        _ => ResetCause::Other,
    }
}
