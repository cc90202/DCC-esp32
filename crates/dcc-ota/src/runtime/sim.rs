//! A simulated device for host tests: flash, clock, track and display in
//! memory, and a bootloader that picks the slot the way the real one does.

use core::{
    cell::{Cell, RefCell},
    task::{Context, Poll, Waker},
    time::Duration,
};
use std::vec::Vec;

use dcc_ota_format::{
    Version,
    image::{BuildKind, BuildMetadata},
    target::ChipRevision,
    testing::{fixture, reseal, signing_key},
};
use ed25519_dalek::VerifyingKey;

use super::{Clock, Device, Event, Feedback, Runtime, State, Storage, Track};
use crate::{
    Error,
    journal::ResetCause,
    layout::{FLASH_SIZE, Slot},
    notice::Notice,
    otadata,
    test_flash::SimFlash,
    trial::Health,
    uptime::Uptime,
};

/// Polls a future of the simulated device once: it finishes, or it hangs
/// for good on a pause the device was told to hang in.
pub(crate) fn poll_once<F: Future>(future: F) -> Poll<F::Output> {
    core::pin::pin!(future)
        .as_mut()
        .poll(&mut Context::from_waker(Waker::noop()))
}

/// Runs a future of the simulated device, which never waits.
pub(crate) fn block_on<F: Future>(future: F) -> F::Output {
    match poll_once(future) {
        Poll::Ready(output) => output,
        Poll::Pending => panic!("the simulated device never waits"),
    }
}

/// An application image of `version`, carrying a release build record with
/// the test key.
pub(crate) fn release_image(version: &str) -> Vec<u8> {
    let mut image = fixture(version, 9120, false);
    let record = BuildMetadata {
        kind: BuildKind::Release,
        public_key: signing_key().verifying_key().to_bytes(),
    }
    .encode();
    image[300..300 + record.len()].copy_from_slice(&record);
    reseal(&mut image, false);
    image
}

pub(crate) struct SimDevice {
    pub flash: RefCell<SimFlash>,
    pub flash_open: Cell<bool>,
    pub state: RefCell<State>,
    pub clock: Cell<Uptime>,
    pub health: Cell<Health>,
    pub version: Version,
    pub key: Option<VerifyingKey>,
    pub track_on: Cell<bool>,
    pub inhibited: Cell<bool>,
    /// Every pause hangs for good, as a task stuck on an await.
    pub hang: Cell<bool>,
    /// How many of the next stops and armings fail.
    pub failing_stops: Cell<u32>,
    pub failing_arms: Cell<u32>,
    pub shown: RefCell<Vec<Notice>>,
    pub events: RefCell<Vec<Event>>,
    pub recovery: Cell<Option<Error>>,
}

impl SimDevice {
    /// A blank device whose slot A holds `version`, about to boot it.
    pub fn with_image(version: &str) -> Self {
        let mut flash = SimFlash::new(0, FLASH_SIZE);
        let image = release_image(version);
        let at = Slot::A.offset() as usize;
        flash.bytes[at..at + image.len()].copy_from_slice(&image);
        Self::booting(flash, Version::parse(version).unwrap())
    }

    fn booting(flash: SimFlash, version: Version) -> Self {
        Self {
            flash: RefCell::new(flash),
            flash_open: Cell::new(true),
            state: RefCell::new(State::new()),
            clock: Cell::new(Uptime::BOOT),
            health: Cell::new(Health {
                alive: true,
                outages: 0,
            }),
            version,
            key: Some(signing_key().verifying_key()),
            track_on: Cell::new(false),
            // The firmware holds the bridge off until the update runtime
            // lets the operator power it.
            inhibited: Cell::new(true),
            hang: Cell::new(false),
            failing_stops: Cell::new(0),
            failing_arms: Cell::new(0),
            shown: RefCell::new(Vec::new()),
            events: RefCell::new(Vec::new()),
            recovery: Cell::new(None),
        }
    }

    /// The same flash after a restart, running `version`.
    pub fn restarted(&self, version: &str) -> Self {
        Self::booting(
            self.flash.borrow().clone(),
            Version::parse(version).unwrap(),
        )
    }

    /// The slot the bootloader starts: the selected one if it holds an
    /// image, otherwise the other.
    pub fn bootloader_slot(&self) -> Slot {
        let mut flash = self.flash.borrow_mut();
        let selected = otadata::active(otadata::read(&mut *flash).unwrap()).map(|(_, e)| e.slot());
        let bootable = |slot: Slot| flash.bytes[slot.offset() as usize] == 0xE9;
        match selected {
            Some(slot) if bootable(slot) => slot,
            Some(slot) => slot.other(),
            None => Slot::A,
        }
    }

    pub fn runtime(&self) -> Runtime<'_, Self> {
        Runtime::new(self)
    }

    /// Boots the device after a restart for `cause`.
    pub fn start(&self, cause: ResetCause) -> Option<super::Restart> {
        let slot = self.bootloader_slot();
        block_on(self.runtime().start(|_| Ok(slot), cause))
    }

    pub fn advance(&self, duration: Duration) {
        self.clock.set(self.clock.get() + duration);
    }

    pub fn lifecycle(&self) -> crate::admission::Lifecycle {
        self.state.borrow().lifecycle()
    }

    pub fn record(&self) -> crate::journal::Record {
        self.state.borrow().record.unwrap()
    }

    pub fn last_shown(&self) -> Option<Notice> {
        self.shown.borrow().last().copied()
    }
}

impl Clock for SimDevice {
    fn now(&self) -> Uptime {
        self.clock.get()
    }

    async fn pause(&self, duration: Duration) {
        if self.hang.get() {
            core::future::pending::<()>().await;
        }
        self.advance(duration);
    }
}

/// Uses up one of `failures`, if any is left.
fn fails(failures: &Cell<u32>) -> bool {
    let left = failures.get();
    failures.set(left.saturating_sub(1));
    left != 0
}

impl Track for SimDevice {
    fn is_on(&self) -> bool {
        self.track_on.get()
    }

    fn inhibit(&self, inhibited: bool) {
        self.inhibited.set(inhibited);
    }

    async fn stop(&self) -> Result<(), Error> {
        if fails(&self.failing_stops) {
            return Err(Error::Timeout);
        }
        self.track_on.set(false);
        Ok(())
    }

    async fn arm(&self) -> Result<(), Error> {
        if fails(&self.failing_arms) {
            return Err(Error::Timeout);
        }
        Ok(())
    }
}

impl Storage for SimDevice {
    type Flash = SimFlash;

    async fn flash<R>(&self, step: impl FnOnce(&mut SimFlash) -> R) -> Option<R> {
        self.flash_open
            .get()
            .then(|| step(&mut self.flash.borrow_mut()))
    }
}

impl Feedback for SimDevice {
    fn show(&self, notice: Notice) {
        self.shown.borrow_mut().push(notice);
    }

    fn report(&self, event: Event) {
        self.events.borrow_mut().push(event);
    }
}

impl Device for SimDevice {
    fn version(&self) -> Version {
        self.version
    }

    fn build_kind(&self) -> BuildKind {
        BuildKind::Release
    }

    fn chip_revision(&self) -> ChipRevision {
        ChipRevision::new(0, 1)
    }

    fn state<R>(&self, change: impl FnOnce(&mut State) -> R) -> R {
        change(&mut self.state.borrow_mut())
    }

    fn health(&self) -> Health {
        self.health.get()
    }

    fn trusted_key(&self) -> Result<VerifyingKey, Error> {
        self.key.ok_or(Error::Corrupt)
    }

    async fn guarded(&self) -> Result<(), Error> {
        Ok(())
    }

    fn recovery_entered(&self, error: Error) {
        self.recovery.set(Some(error));
    }
}
