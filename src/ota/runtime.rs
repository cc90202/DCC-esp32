//! ESP runtime: one flash owner, physical inhibit, durable boot and trial health.
use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_bootloader_esp_idf::partitions::{PARTITION_TABLE_MAX_LEN, read_partition_table};
use esp_hal::rtc_cntl::{Rtc, RwdtStage};
use esp_storage::FlashStorage;
use sha2::{Digest, Sha256};

use super::{
    Error, ImageId, SECTOR_SIZE, SLOT_OFFSETS, SLOT_SIZE,
    admission::{Admission, Operation},
    boot_state::{self, Action},
    health::{Decision, Window},
    image::{self, BuildKind, BuildMetadata},
    journal::{self, Outcome, Phase, Record},
    otadata::{self, State as BootState},
    package::{self, Manifest},
    session::{Buffers, Writer},
    status::{Snapshot, Stage},
};
pub(crate) use crate::track_output::inhibited;
use crate::{
    runtime_channels::{DisplaySender, FaultEventSender},
    system_status::{BootStep, DisplayEvent, FaultEvent},
    track_output::set_inhibited,
};

pub static FLASH: Mutex<CriticalSectionRawMutex, Option<FlashStorage<'static>>> = Mutex::new(None);
static BUSY: AtomicBool = AtomicBool::new(false);
static PENDING: AtomicBool = AtomicBool::new(false);
static RECOVERY: AtomicBool = AtomicBool::new(false);
static INITIALIZING: AtomicBool = AtomicBool::new(true);
static READY: AtomicBool = AtomicBool::new(false);
static ARMED: AtomicBool = AtomicBool::new(false);
static STOP_ACK: AtomicBool = AtomicBool::new(false);
static EPOCH: AtomicU32 = AtomicU32::new(0);
static WATCHDOG_ACTIVE: AtomicBool = AtomicBool::new(false);
static HEALTH_EPOCH: AtomicU32 = AtomicU32::new(0);
static NETWORK: AtomicBool = AtomicBool::new(false);
static Z21: AtomicBool = AtomicBool::new(false);
static HTTP: AtomicBool = AtomicBool::new(false);
static BEATS: [AtomicU32; 5] = [const { AtomicU32::new(0) }; 5];
#[derive(Clone, Copy)]
pub enum Heartbeat {
    Dcc,
    Scheduler,
    Fault,
    Net,
    Http,
}
pub fn heartbeat(task: Heartbeat) {
    let now = (Instant::now().as_millis() as u32).max(1);
    let old = BEATS[task as usize].swap(now, Ordering::Relaxed);
    if old != 0 && now.wrapping_sub(old) >= 2500 {
        HEALTH_EPOCH.fetch_add(1, Ordering::Relaxed);
    }
}
pub fn network_ready(ready: bool) {
    if NETWORK.swap(ready, Ordering::Relaxed) && !ready {
        HEALTH_EPOCH.fetch_add(1, Ordering::Relaxed);
    }
}
pub fn z21_ready() {
    Z21.store(true, Ordering::Relaxed);
}
pub fn http_ready() {
    HTTP.store(true, Ordering::Relaxed);
}
pub fn busy() -> bool {
    BUSY.load(Ordering::Acquire)
}
pub fn epoch() -> u32 {
    EPOCH.load(Ordering::Acquire)
}
pub fn pending() -> bool {
    PENDING.load(Ordering::Acquire)
}
pub fn recovery() -> bool {
    RECOVERY.load(Ordering::Acquire)
}
pub fn stop_ack() {
    STOP_ACK.store(true, Ordering::Release);
}
pub fn armed_ack() {
    ARMED.store(true, Ordering::Release);
}
pub fn ready() {
    READY.store(true, Ordering::Release);
}

#[cfg(feature = "ota-dev-key")]
const KEY: [u8; 32] = *include_bytes!("../../keys/ota_dev.pub");
#[cfg(not(feature = "ota-dev-key"))]
const KEY: [u8; 32] = *include_bytes!("../../keys/ota_release.pub");
#[cfg(feature = "ota-fault-inject")]
const KIND: BuildKind = BuildKind::Fault;
#[cfg(all(feature = "ota-dev-key", not(feature = "ota-fault-inject")))]
const KIND: BuildKind = BuildKind::Dev;
#[cfg(not(feature = "ota-dev-key"))]
const KIND: BuildKind = BuildKind::Release;
#[used]
#[unsafe(link_section = ".rodata.ota_key")]
static BUILD_KEY: [u8; 44] = BuildMetadata {
    kind: KIND,
    public_key: KEY,
}
.encode();
fn public_key() -> [u8; 32] {
    // Keep the complete canonical record in the loaded image, including marker.
    // SAFETY: BUILD_KEY is an aligned, immutable static; its reference remains
    // valid for the complete array read and no writer can race this access.
    let record = unsafe { core::ptr::read_volatile(&BUILD_KEY) };
    record[12..].try_into().unwrap()
}

#[derive(Clone, Copy)]
struct State {
    record: Option<Record>,
    display: Option<DisplaySender>,
    slot: u8,
    boot_id: u64,
    received: u32,
    written: u32,
    total: u32,
    phase: Stage,
    reason: Option<Error>,
    next_check: u64,
    backoff: u64,
}
static STATE: critical_section::Mutex<RefCell<State>> =
    critical_section::Mutex::new(RefCell::new(State {
        record: None,
        display: None,
        slot: 0,
        boot_id: 0,
        received: 0,
        written: 0,
        total: 0,
        phase: Stage::Boot,
        reason: None,
        next_check: 0,
        backoff: 0,
    }));
fn state() -> State {
    critical_section::with(|cs| *STATE.borrow(cs).borrow())
}
fn mutate(f: impl FnOnce(&mut State)) {
    critical_section::with(|cs| f(&mut STATE.borrow(cs).borrow_mut()));
}
pub fn booted_slot() -> u8 {
    state().slot
}
pub fn hold_track_off() -> bool {
    state().record.is_some_and(|r| r.hold_track_off)
}
pub fn status_json() -> Result<heapless::String<1536>, Error> {
    let s = state();
    Snapshot {
        record: s.record,
        boot_id: s.boot_id,
        uptime: core::time::Duration::from_secs(Instant::now().as_secs()),
        version: package::parse_version(env!("CARGO_PKG_VERSION"))?,
        slot: s.slot,
        pending: pending(),
        recovery: recovery(),
        busy: busy(),
        track_enabled: crate::track_output::TrackOutput.is_track_enabled(),
        received: s.received,
        written: s.written,
        total: s.total,
        stage: s.phase,
        reason: s.reason,
    }
    .json()
}

fn progress(received: u32, written: u32, total: u32, phase: Stage) {
    mutate(|s| {
        s.received = received;
        s.written = written;
        s.total = total;
        s.phase = phase;
    });
}
pub fn enter_recovery(error: Error) {
    critical_section::with(|_| {
        set_inhibited(true);
        RECOVERY.store(true, Ordering::Release);
    });
    crate::track_authority::invalidate_and_disable();
    mutate(|s| {
        s.reason = Some(error);
        s.phase = Stage::Recovery;
    });
    defmt::error!(
        "OTA recovery: {}; track disabled, restore via USB",
        error.code()
    );
    if let Some(display) = state().display {
        let _ = display.try_send(DisplayEvent::BootProgress(BootStep::UsbRecovery));
    }
}

async fn hash_image(slot: u8, len: u32) -> Result<[u8; 32], Error> {
    if slot > 1 || !(4096..=SLOT_SIZE).contains(&len) {
        return Err(Error::BadImage);
    }
    let mut bytes = [0; SECTOR_SIZE];
    let mut hash = Sha256::new();
    let mut offset = 0;
    while offset < len {
        let n = (len - offset).min(SECTOR_SIZE as u32) as usize;
        {
            let mut guard = FLASH.lock().await;
            guard
                .as_mut()
                .ok_or(Error::Unavailable)?
                .read(
                    SLOT_OFFSETS[slot as usize] + offset,
                    &mut bytes[..n.next_multiple_of(4)],
                )
                .map_err(|_| Error::Flash)?;
        }
        hash.update(&bytes[..n]);
        offset += n as u32;
        Timer::after_millis(1).await;
    }
    Ok(hash.finalize().into())
}
async fn matches(id: ImageId) -> Result<bool, Error> {
    Ok(hash_image(id.slot, id.len).await? == id.digest)
}
async fn save(record: Record) -> Result<Record, Error> {
    let saved = {
        let mut guard = FLASH.lock().await;
        journal::save(guard.as_mut().ok_or(Error::Unavailable)?, record)?
    };
    mutate(|s| s.record = Some(saved));
    Ok(saved)
}
async fn select(slot: u8, state: BootState) -> Result<(), Error> {
    let mut guard = FLASH.lock().await;
    let flash = guard.as_mut().ok_or(Error::Unavailable)?;
    let selected = otadata::active(otadata::read(flash)?);
    if selected.is_some_and(|(_, e)| (e.seq - 1) % 2 == u32::from(slot) && e.state == state as u32)
    {
        return Ok(());
    }
    otadata::select(flash, slot, state)
}
async fn erase_candidate(record: Record) -> Result<(), Error> {
    if let Some(c) = record
        .candidate
        .filter(|c| c.slot != booted_slot() && c != &record.current)
    {
        let mut guard = FLASH.lock().await;
        let flash = guard.as_mut().ok_or(Error::Unavailable)?;
        let base = SLOT_OFFSETS[c.slot as usize];
        flash
            .erase(base, base + SECTOR_SIZE as u32)
            .map_err(|_| Error::Flash)?;
    }
    Ok(())
}
async fn restore(mut record: Record) -> Result<(), Error> {
    if !matches(record.current).await? {
        return Err(Error::Corrupt);
    }
    record.hold_track_off = true;
    let record = save(record).await?;
    select(record.current.slot, BootState::Valid).await?;
    erase_candidate(record).await?;
    if record.current.slot != booted_slot() {
        esp_hal::system::software_reset();
    }
    Ok(())
}

pub async fn start(
    spawner: embassy_executor::Spawner,
    flash: esp_hal::peripherals::FLASH<'static>,
    lpwr: esp_hal::peripherals::LPWR<'static>,
    sender: FaultEventSender,
    display: DisplaySender,
) {
    mutate(|s| s.display = Some(display));
    *FLASH.lock().await = Some(FlashStorage::new(flash));
    let rng = esp_hal::rng::Rng::new();
    mutate(|s| s.boot_id = (u64::from(rng.random()) << 32) | u64::from(rng.random()));
    if spawner.spawn(monitor_task(lpwr, sender)).is_err() {
        enter_recovery(Error::Unavailable);
    }
    if let Err(error) = boot_check().await {
        enter_recovery(error);
    }
    INITIALIZING.store(false, Ordering::Release);
    if !pending() && !recovery() && !hold_track_off() {
        set_inhibited(false);
    }
}
/// Validate the flash layout and read the boot environment before reconciliation.
fn read_boot_environment(flash: &mut FlashStorage<'_>) -> Result<(u8, Option<Record>, u32), Error> {
    if flash.capacity() != 0x800000 {
        return Err(Error::Unavailable);
    }
    let mut bytes = [0; PARTITION_TABLE_MAX_LEN];
    let table = read_partition_table(flash, &mut bytes).map_err(|_| Error::Corrupt)?;
    let required = [
        ("nvs", 1, 2, 0x9000, 0x4000),
        ("phy_init", 1, 1, 0xF000, 0x1000),
        ("ota_0", 0, 0x10, 0x10000, 0x300000),
        ("ota_1", 0, 0x11, 0x310000, 0x300000),
        ("otadata", 1, 0, 0xD000, 0x2000),
        ("dcc_cfg", 1, 2, 0x610000, 0x3000),
        ("ota_journal", 1, 6, 0x613000, 0x2000),
    ];
    if table.len() != required.len()
        || required.iter().any(|&(name, ty, sub, offset, size)| {
            !table.iter().any(|p| {
                p.label_as_str() == name
                    && p.raw_type() == ty
                    && p.raw_subtype() == sub
                    && p.offset() == offset
                    && p.len() == size
            })
        })
    {
        return Err(Error::Corrupt);
    }
    let booted = table
        .booted_partition()
        .map_err(|_| Error::Corrupt)?
        .ok_or(Error::Corrupt)?;
    let slot = SLOT_OFFSETS
        .iter()
        .position(|&o| o == booted.offset())
        .ok_or(Error::Corrupt)? as u8;
    let record = journal::load(flash)?;
    let len = image::encoded_length(flash, SLOT_OFFSETS[slot as usize], SLOT_SIZE)?;
    Ok((slot, record, len))
}

/// Non-destructive signature and inactive-storage checks before admitting a trial.
async fn self_test(slot: u8) -> Result<(), Error> {
    if ed25519_dalek::VerifyingKey::from_bytes(&public_key())
        .map_err(|_| Error::UnknownKey)?
        .is_weak()
    {
        return Err(Error::UnknownKey);
    }
    let fixture =
        Manifest::encode_unsigned(super::Version([1, 0, 0]), 4096, [0; 32], &public_key())?;
    if Manifest::authenticate(&fixture, &public_key()) != Err(Error::BadSignature) {
        return Err(Error::Corrupt);
    }
    // Non-destructive inactive-slot and metadata path test before every trial.
    {
        let mut g = FLASH.lock().await;
        let flash = g.as_mut().ok_or(Error::Unavailable)?;
        let mut b = [0; 4];
        flash
            .read(SLOT_OFFSETS[1 - slot as usize], &mut b)
            .map_err(|_| Error::Flash)?;
        otadata::read(flash)?;
    }
    Ok(())
}

async fn boot_check() -> Result<(), Error> {
    let (slot, record, len) = {
        let mut guard = FLASH.lock().await;
        read_boot_environment(guard.as_mut().ok_or(Error::Unavailable)?)?
    };
    mutate(|s| {
        s.slot = slot;
        s.record = record;
    });
    let booted = ImageId {
        slot,
        version: package::parse_version(env!("CARGO_PKG_VERSION"))?,
        len,
        digest: hash_image(slot, len).await?,
    };
    self_test(slot).await?;
    match record {
        None => {
            save(Record {
                generation: 0,
                phase: Phase::Confirmed,
                current: booted,
                candidate: None,
                previous: None,
                rejected: None,
                outcome: Outcome::None,
                hold_track_off: false,
            })
            .await?;
            select(slot, BootState::Valid).await?;
        }
        Some(record) => {
            let current_ok = if record.current == booted {
                true
            } else {
                matches(record.current).await?
            };
            match boot_state::decide(record, booted, current_ok)? {
                Action::Run(r) => {
                    select(r.current.slot, BootState::Valid).await?;
                    erase_candidate(r).await?;
                }
                Action::Trial(r) => {
                    save(r).await?;
                    PENDING.store(true, Ordering::Release);
                }
                Action::Restore(r) => restore(r).await?,
            }
        }
    }
    if !recovery() {
        mutate(|s| {
            s.phase = if pending() {
                Stage::Health
            } else {
                Stage::Idle
            }
        });
    }
    Ok(())
}

async fn stop_and_wait(sender: FaultEventSender) -> Result<(), Error> {
    STOP_ACK.store(false, Ordering::Release);
    crate::track_authority::invalidate_and_disable();
    with_timeout(Duration::from_secs(2), async {
        sender.send(FaultEvent::StopPressed).await;
        while !STOP_ACK.load(Ordering::Acquire) {
            Timer::after_millis(10).await;
        }
    })
    .await
    .map_err(|_| Error::Timeout)
}
pub struct UpdateGuard {
    receiving: bool,
    activated: bool,
}
impl Drop for UpdateGuard {
    fn drop(&mut self) {
        if self.activated {
            return;
        }
        critical_section::with(|_| {
            if !pending() && !recovery() {
                set_inhibited(false);
            }
            BUSY.store(false, Ordering::Release);
        });
    }
}
pub async fn acquire(sender: FaultEventSender) -> Result<UpdateGuard, Error> {
    acquire_inner(sender, Operation::Update).await
}
pub async fn acquire_provisioning(sender: FaultEventSender) -> Result<UpdateGuard, Error> {
    acquire_inner(sender, Operation::Provisioning).await
}
async fn acquire_inner(
    sender: FaultEventSender,
    operation: Operation,
) -> Result<UpdateGuard, Error> {
    let now = Instant::now().as_millis();
    critical_section::with(|cs| {
        let mut s = STATE.borrow(cs).borrow_mut();
        Admission {
            available: !recovery() && s.record.is_some(),
            pending: pending(),
            boot_ready: READY.load(Ordering::Acquire),
            armed: ARMED.load(Ordering::Acquire),
            busy: busy(),
            track_on: crate::track_output::TrackOutput.is_track_enabled(),
            backoff_until: s.backoff,
            next_check: s.next_check,
        }
        .check(now, operation)?;
        BUSY.store(true, Ordering::Release);
        set_inhibited(true);
        EPOCH.fetch_add(1, Ordering::AcqRel);
        if operation == Operation::Update {
            s.next_check = now + 1000;
        }
        Ok(())
    })?;
    let guard = UpdateGuard {
        receiving: false,
        activated: false,
    };
    // No signature/flash work until the monitor has actually armed RWDT.
    with_timeout(Duration::from_secs(2), async {
        while !WATCHDOG_ACTIVE.load(Ordering::Acquire) {
            Timer::after_millis(10).await;
        }
    })
    .await
    .map_err(|_| Error::Timeout)?;
    if let Err(e) = stop_and_wait(sender).await {
        drop(guard);
        return Err(e);
    }
    Ok(guard)
}
impl UpdateGuard {
    pub fn verify(&self, header: &[u8; 256]) -> Result<Manifest, Error> {
        let r = state().record.ok_or(Error::Unavailable)?;
        let manifest = Manifest::authenticate(header, &public_key())?;
        manifest.check_eligibility(r.current.version, r.rejected_version())?;
        Ok(manifest)
    }
    pub async fn begin<'a>(
        &mut self,
        m: Manifest,
        buffers: &'a mut Buffers,
    ) -> Result<Writer<'a>, Error> {
        let r = state().record.ok_or(Error::Unavailable)?;
        let target = ImageId {
            slot: 1 - booted_slot(),
            version: m.version,
            len: m.image_len,
            digest: m.digest,
        }
        .validate()?;
        if let Err(error) = save(r.begin(target)?).await {
            enter_recovery(error);
            return Err(error);
        }
        self.receiving = true;
        let writer = {
            let mut g = FLASH.lock().await;
            let f = g.as_mut().ok_or(Error::Unavailable)?;
            otadata::remove_slot(f, target.slot)?;
            Writer::new(m, booted_slot(), buffers, f)?
        };
        mutate(|s| s.reason = None);
        progress(0, 0, target.len, Stage::Receiving);
        Ok(writer)
    }

    /// Persists one bounded chunk without holding flash across network I/O.
    pub async fn push(&mut self, writer: &mut Writer<'_>, bytes: &[u8]) -> Result<(), Error> {
        let mut flash = FLASH.lock().await;
        writer.push(flash.as_mut().ok_or(Error::Unavailable)?, bytes)?;
        let received = writer.received();
        let written = received / SECTOR_SIZE as u32 * SECTOR_SIZE as u32;
        progress(
            received,
            written.saturating_sub(SECTOR_SIZE as u32),
            writer.target().len,
            Stage::Receiving,
        );
        Ok(())
    }

    /// Owns the durable publication order; the transport only supplies a deadline.
    pub async fn finish(
        &mut self,
        writer: &mut Writer<'_>,
        deadline: Instant,
    ) -> Result<ImageId, Error> {
        {
            let mut flash = FLASH.lock().await;
            writer.seal(flash.as_mut().ok_or(Error::Unavailable)?)?;
        }
        let target = writer.target();
        progress(
            writer.received(),
            target.len - SECTOR_SIZE as u32,
            target.len,
            Stage::Verifying,
        );
        loop {
            if Instant::now() >= deadline {
                return Err(Error::Timeout);
            }
            let done = {
                let mut flash = FLASH.lock().await;
                writer.verify_step(flash.as_mut().ok_or(Error::Unavailable)?)?
            };
            Timer::after_millis(1).await;
            if done {
                break;
            }
        }
        if Instant::now() >= deadline {
            return Err(Error::Timeout);
        }
        {
            let mut flash = FLASH.lock().await;
            writer.commit_first(flash.as_mut().ok_or(Error::Unavailable)?)?;
        }
        progress(writer.received(), target.len, target.len, Stage::Ready);
        Ok(target)
    }
    pub async fn activate(&mut self, target: ImageId) -> Result<(), Error> {
        self.activated = true;
        let mut r = state().record.ok_or(Error::Unavailable)?;
        if r.phase != Phase::Receiving || r.candidate != Some(target) {
            return Err(Error::Corrupt);
        }
        r.phase = Phase::Ready;
        if let Err(e) = save(r).await {
            enter_recovery(e);
            return Err(e);
        }
        select(target.slot, BootState::New).await?;
        Ok(())
    }
    pub async fn abort(&mut self, error: Error) {
        mutate(|s| {
            s.reason = Some(error);
            s.backoff = Instant::now().as_millis() + 30000;
        });
        if !self.receiving {
            return;
        }
        let cleanup = async {
            let r = state().record.ok_or(Error::Unavailable)?;
            // Restore selection first if activation partially succeeded.
            select(r.current.slot, BootState::Valid).await?;
            erase_candidate(r).await?;
            let outcome = if matches!(error, Error::Timeout | Error::BadPackage) {
                Outcome::Aborted
            } else {
                Outcome::NotBootable
            };
            save(r.abort(outcome)?).await?;
            Ok::<(), Error>(())
        }
        .await;
        if let Err(e) = cleanup {
            enter_recovery(e);
        } else if !recovery() {
            mutate(|s| s.phase = Stage::Idle);
        }
    }
}

pub async fn rollback() -> Result<(), Error> {
    let mut r = state().record.ok_or(Error::Unavailable)?;
    r.reject_candidate();
    r.phase = Phase::RolledBack;
    r.outcome = Outcome::RolledBack;
    restore(r).await
}
async fn confirm(sender: FaultEventSender, window: &Window) -> Result<bool, Error> {
    stop_and_wait(sender).await?;
    let mut r = state().record.ok_or(Error::Unavailable)?;
    let candidate = r.candidate.ok_or(Error::Corrupt)?;
    if r.phase != Phase::Trying || candidate.slot != booted_slot() {
        return Err(Error::Corrupt);
    }
    r.previous = Some(r.current);
    r.current = candidate;
    r.phase = Phase::Confirmed;
    r.outcome = Outcome::Ok;
    {
        let mut guard = FLASH.lock().await;
        if !window.eligible(
            Instant::now().as_millis(),
            healthy(),
            HEALTH_EPOCH.load(Ordering::Relaxed),
        ) {
            return Ok(false);
        }
        let saved = journal::save(guard.as_mut().ok_or(Error::Unavailable)?, r)?;
        mutate(|s| s.record = Some(saved));
    }
    select(candidate.slot, BootState::Valid).await?;
    arm_stopped(sender).await?;
    PENDING.store(false, Ordering::Release);
    mutate(|s| s.phase = Stage::Idle);
    Ok(true)
}

async fn arm_stopped(sender: FaultEventSender) -> Result<(), Error> {
    // Keep inhibit through the FIFO arming ACK; older Resume requests are refused.
    with_timeout(Duration::from_secs(2), async {
        sender.send(FaultEvent::TrackPowerArmed).await;
        while !ARMED.load(Ordering::Acquire) {
            Timer::after_millis(10).await;
        }
    })
    .await
    .map_err(|_| Error::Timeout)?;
    set_inhibited(false);
    Ok(())
}
pub async fn complete_boot(sender: FaultEventSender) {
    ready();
    #[cfg(feature = "ota-fault-inject")]
    if pending() {
        match option_env!("OTA_FAULT").unwrap_or("none") {
            "none" | "health-timeout" => {}
            "panic" => panic!("OTA bench injected panic"),
            "hang" => loop {
                core::hint::spin_loop();
            },
            _ => panic!("Unknown OTA_FAULT selector"),
        }
    }
    if pending() || recovery() {
        return;
    }
    let result = if hold_track_off() {
        match stop_and_wait(sender).await {
            Ok(()) => arm_stopped(sender).await,
            Err(e) => Err(e),
        }
    } else {
        sender.send(FaultEvent::TrackPowerArmed).await;
        Ok(())
    };
    if let Err(e) = result {
        enter_recovery(e);
    }
}

fn healthy() -> bool {
    let now_ms = Instant::now().as_millis() as u32;
    let healthy = READY.load(Ordering::Acquire)
        && NETWORK.load(Ordering::Relaxed)
        && Z21.load(Ordering::Relaxed)
        && HTTP.load(Ordering::Relaxed)
        && BEATS.iter().all(|b| {
            let v = b.load(Ordering::Relaxed);
            v != 0 && now_ms.wrapping_sub(v) < 2500
        });
    #[cfg(feature = "ota-fault-inject")]
    let healthy = healthy && option_env!("OTA_FAULT") != Some("health-timeout");
    healthy
}

#[embassy_executor::task]
async fn monitor_task(lpwr: esp_hal::peripherals::LPWR<'static>, sender: FaultEventSender) -> ! {
    let mut rtc = Rtc::new(lpwr);
    rtc.rwdt.enable();
    rtc.rwdt
        .set_timeout(RwdtStage::Stage0, esp_hal::time::Duration::from_secs(10));
    WATCHDOG_ACTIVE.store(true, Ordering::Release);
    let mut window = Window::new(Instant::now().as_millis());
    let mut watchdog = true;
    loop {
        Timer::after_secs(1).await;
        let needs_watchdog = INITIALIZING.load(Ordering::Acquire) || pending() || busy();
        if needs_watchdog {
            if !watchdog {
                rtc.rwdt.enable();
                rtc.rwdt
                    .set_timeout(RwdtStage::Stage0, esp_hal::time::Duration::from_secs(10));
                watchdog = true;
                WATCHDOG_ACTIVE.store(true, Ordering::Release);
            }
            rtc.rwdt.feed();
        } else if watchdog {
            rtc.rwdt.disable();
            watchdog = false;
            WATCHDOG_ACTIVE.store(false, Ordering::Release);
        }
        if !pending() || INITIALIZING.load(Ordering::Acquire) || recovery() {
            continue;
        }
        let result = match window.observe(
            Instant::now().as_millis(),
            healthy(),
            HEALTH_EPOCH.load(Ordering::Relaxed),
        ) {
            Decision::Wait => continue,
            Decision::Rollback => rollback().await,
            Decision::Confirm => confirm(sender, &window).await.map(|_| ()),
        };
        if let Err(e) = result {
            enter_recovery(e);
            PENDING.store(false, Ordering::Release);
        }
    }
}
