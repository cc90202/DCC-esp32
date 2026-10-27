//! The update runtime end to end on a simulated device: boot, transfer,
//! restart, trial and the ways back.

use core::time::Duration;

use dcc_ota_format::{
    Digest, ImageLen, Version, image::ImageError, package::Ineligible, testing::signed,
};

use embedded_storage::nor_flash::NorFlashErrorKind;

use super::{
    Event, Restart, Trial,
    sim::{SimDevice, block_on, poll_once, release_image},
};
use crate::{
    Error, FlashError, Refusal,
    admission::{Lifecycle, Operation},
    journal::{Ending, ResetCause, Update, store},
    layout::{SECTOR_SIZE, Slot},
    notice::Notice,
    trial::Health,
    writer::Buffers,
};

const SECOND: Duration = Duration::from_secs(1);
const DEAD: Health = Health {
    alive: false,
    outages: 0,
};

/// Signs `version`'s image and sends `sent` in its place: the whole
/// transfer, from the lock to the restart.
fn upload_as(device: &SimDevice, version: &str, sent: &[u8]) -> Result<Restart, Error> {
    let image = release_image(version);
    let header = signed(
        Version::parse(version).unwrap(),
        ImageLen::of(&image).unwrap(),
        Digest::of(&image),
    );
    block_on(async {
        let mut guard = device.runtime().acquire(Operation::Update).await?;
        let manifest = guard.verify(&header)?;
        let mut buffers = Buffers::new();
        let written = async {
            let mut writer = guard.begin(manifest, &mut buffers).await?;
            for chunk in sent.chunks(1000) {
                guard.push(&mut writer, chunk).await?;
            }
            guard
                .finish(&mut writer, device.clock.get() + 300 * SECOND)
                .await
        }
        .await;
        match written {
            Ok(image) => guard.activate(image, async |_| {}).await,
            Err(error) => {
                guard.abort(error).await;
                Err(error)
            }
        }
    })
}

fn upload(device: &SimDevice, version: &str) -> Result<Restart, Error> {
    upload_as(device, version, &release_image(version))
}

/// A device running 1.0.0, booted and armed.
fn armed() -> SimDevice {
    let device = SimDevice::with_image("1.0.0");
    assert_eq!(device.start(ResetCause::External), None);
    block_on(device.runtime().arm_after_boot());
    assert_eq!(device.lifecycle(), Lifecycle::Armed);
    device
}

/// The device after 1.1.0 was uploaded and booted into its trial.
fn on_trial() -> SimDevice {
    let device = armed();
    assert_eq!(upload(&device, "1.1.0"), Ok(Restart));
    let next = device.restarted("1.1.0");
    assert_eq!(next.start(ResetCause::Other), None);
    assert_eq!(next.lifecycle(), Lifecycle::Trial);
    next
}

/// Ticks once a second until a tick asks for a restart or `limit` passes;
/// returns the time of the restart.
fn tick_until_restart(device: &SimDevice, trial: &mut Trial, limit: Duration) -> Option<Duration> {
    while device.clock.get().since_boot() < limit {
        device.advance(SECOND);
        if block_on(device.runtime().tick(trial)).is_some() {
            return Some(device.clock.get().since_boot());
        }
    }
    None
}

fn stored(device: &SimDevice) -> crate::journal::Record {
    store::load(&mut *device.flash.borrow_mut())
        .unwrap()
        .unwrap()
}

#[test]
fn the_first_boot_installs_the_running_image_and_frees_the_track() {
    let device = SimDevice::with_image("1.0.0");
    assert_eq!(device.start(ResetCause::External), None);
    assert_eq!(device.lifecycle(), Lifecycle::Running);
    assert_eq!(stored(&device).current().version(), Version::new(1, 0, 0));
    assert_eq!(stored(&device).update(), Update::None);
    assert!(!device.inhibited.get());
    assert_eq!(device.last_shown(), None);
}

#[test]
fn an_update_is_tried_after_the_restart_and_confirmed_after_a_healthy_minute() {
    let device = on_trial();
    assert_eq!(
        device.last_shown(),
        Some(Notice::Trial {
            version: Version::new(1, 1, 0),
            left: None,
        })
    );
    assert!(device.inhibited.get(), "no track during the trial");
    let mut trial = Trial::default();
    assert_eq!(tick_until_restart(&device, &mut trial, 70 * SECOND), None);
    let confirmed = stored(&device);
    assert_eq!(confirmed.current().slot(), Slot::B);
    assert!(matches!(
        confirmed.update().ending(),
        Some(Ending::Confirmed { .. })
    ));
    assert_eq!(device.lifecycle(), Lifecycle::Armed);
    assert!(!device.inhibited.get());
    assert_eq!(
        device.last_shown(),
        Some(Notice::Confirmed(Version::new(1, 1, 0)))
    );
    // The confirmed image keeps booting. Its first boot waits for the
    // operator to power the track; the ones after it start as before.
    let next = device.restarted("1.1.0");
    assert_eq!(next.start(ResetCause::External), None);
    assert_eq!(next.lifecycle(), Lifecycle::Running);
    assert!(
        next.inhibited.get(),
        "the boot after an update holds the track"
    );
    assert_eq!(next.last_shown(), Some(Notice::TrackOff));
    assert!(!stored(&next).holds_track_off());
    let later = next.restarted("1.1.0");
    assert_eq!(later.start(ResetCause::External), None);
    assert_eq!(later.lifecycle(), Lifecycle::Running);
    assert!(!later.inhibited.get(), "later boots power the track");
    assert_eq!(later.last_shown(), None);
}

#[test]
fn a_trial_that_never_gets_healthy_rolls_back_and_rejects_the_release() {
    let device = on_trial();
    device.health.set(DEAD);
    let mut trial = Trial::default();
    let restart = tick_until_restart(&device, &mut trial, 300 * SECOND).unwrap();
    assert!(restart >= 180 * SECOND, "rolled back at {restart:?}");
    assert_eq!(device.last_shown(), Some(Notice::RestartingBack));
    // Until the restart lands the page still reports a pending trial.
    let status: serde_json::Value =
        serde_json::from_str(&device.runtime().status().unwrap()).unwrap();
    assert_eq!(status["last_update"]["result"], "rolled_back");
    assert_eq!(status["last_update"]["reason"], "pending_verify");
    let back = device.restarted("1.0.0");
    assert_eq!(back.start(ResetCause::Other), None);
    assert_eq!(back.record().current().slot(), Slot::A);
    assert_eq!(back.record().rejected(), Some(Version::new(1, 1, 0)));
    // The track waits for the operator after the failed update.
    assert!(back.inhibited.get());
    assert_eq!(back.last_shown(), Some(Notice::TrackOff));
    block_on(back.runtime().arm_after_boot());
    assert_eq!(
        upload(&back, "1.1.0"),
        Err(Error::Ineligible(Ineligible::Rejected))
    );
    // Only that one boot waits.
    let later = back.restarted("1.0.0");
    assert_eq!(later.start(ResetCause::External), None);
    assert!(!later.inhibited.get());
}

#[test]
fn the_trial_deadline_counts_from_the_trial_not_from_boot() {
    let device = armed();
    assert_eq!(upload(&device, "1.1.0"), Ok(Restart));
    let next = device.restarted("1.1.0");
    next.health.set(DEAD);
    // The monitor ticks while a slow boot check still runs.
    let mut trial = Trial::default();
    assert_eq!(tick_until_restart(&next, &mut trial, 100 * SECOND), None);
    assert_eq!(next.start(ResetCause::Other), None);
    let restart = tick_until_restart(&next, &mut trial, 400 * SECOND).unwrap();
    assert!(restart >= 280 * SECOND, "rolled back at {restart:?}");
}

#[test]
fn a_confirmation_waits_for_the_track_and_a_late_failure_finishes_by_restart() {
    let device = on_trial();
    device.failing_stops.set(1);
    device.failing_arms.set(1);
    let mut trial = Trial::default();
    assert!(tick_until_restart(&device, &mut trial, 70 * SECOND).is_some());
    let events = device.events.borrow();
    assert!(matches!(events[0], Event::ConfirmPostponed(Error::Timeout)));
    assert!(matches!(
        events[1],
        Event::ConfirmedRestarting(Error::Timeout)
    ));
    // The journal already holds the confirmation; the restart finishes it.
    assert!(matches!(
        stored(&device).update().ending(),
        Some(Ending::Confirmed { .. })
    ));
    let next = device.restarted("1.1.0");
    assert_eq!(next.start(ResetCause::Other), None);
    assert_eq!(next.lifecycle(), Lifecycle::Running);
}

#[test]
fn a_failed_transfer_cleans_up_releases_the_lock_and_backs_off() {
    let device = armed();
    let mut tampered = release_image("1.1.0");
    tampered[5000] ^= 1;
    assert_eq!(
        upload_as(&device, "1.1.0", &tampered),
        Err(Error::Image(ImageError::DigestMismatch))
    );
    assert_eq!(stored(&device).update().ending(), Some(Ending::NotBootable));
    assert_eq!(stored(&device).rejected(), None);
    assert_eq!(device.flash.borrow().bytes[Slot::B.offset() as usize], 0xFF);
    assert!(!device.state.borrow().busy());
    assert!(!device.inhibited.get());
    assert!(device.shown.borrow().contains(&Notice::Failed));
    assert_eq!(upload(&device, "1.1.0"), Err(Error::Refused(Refusal::Busy)));
    device.advance(30 * SECOND);
    assert_eq!(upload(&device, "1.1.0"), Ok(Restart));
}

#[test]
fn a_journal_that_cannot_be_written_ends_in_recovery() {
    let device = armed();
    let image = release_image("1.1.0");
    let header = signed(
        Version::new(1, 1, 0),
        ImageLen::of(&image).unwrap(),
        Digest::of(&image),
    );
    let result = block_on(async {
        let mut guard = device.runtime().acquire(Operation::Update).await?;
        let manifest = guard.verify(&header)?;
        device.flash_open.set(false);
        guard.begin(manifest, &mut Buffers::new()).await.map(|_| ())
    });
    assert_eq!(result, Err(Error::Unavailable));
    assert_eq!(device.recovery.get(), Some(Error::Unavailable));
    assert_eq!(device.lifecycle(), Lifecycle::Recovery);
    assert!(device.inhibited.get(), "recovery keeps the track off");
    assert_eq!(
        upload(&device, "1.1.0"),
        Err(Error::Refused(Refusal::Recovery))
    );
}

#[test]
fn a_candidate_failing_its_own_boot_check_is_rolled_back() {
    let device = armed();
    assert_eq!(upload(&device, "1.1.0"), Ok(Restart));
    let mut broken = device.restarted("1.1.0");
    broken.key = None;
    assert_eq!(broken.start(ResetCause::Other), Some(Restart));
    assert!(matches!(
        broken.events.borrow()[0],
        Event::CandidateFailedBootCheck(_)
    ));
    assert_eq!(stored(&broken).update().ending(), Some(Ending::RolledBack));
    assert_eq!(stored(&broken).rejected(), Some(Version::new(1, 1, 0)));
    assert_eq!(broken.recovery.get(), None);
}

#[test]
fn a_trial_ended_by_setup_or_by_power_loss_keeps_the_release_eligible() {
    let device = on_trial();
    assert_eq!(
        block_on(device.runtime().withdraw_trial()),
        Ok(Some(Restart))
    );
    assert_eq!(stored(&device).update().ending(), Some(Ending::Aborted));
    assert_eq!(stored(&device).rejected(), None);

    let device = on_trial();
    let cut = device.restarted("1.1.0");
    assert_eq!(cut.start(ResetCause::External), Some(Restart));
    assert_eq!(stored(&cut).update().ending(), Some(Ending::Aborted));
    assert_eq!(stored(&cut).rejected(), None);
    let back = cut.restarted("1.0.0");
    assert_eq!(back.start(ResetCause::External), None);
    block_on(back.runtime().arm_after_boot());
    assert_eq!(upload(&back, "1.1.0"), Ok(Restart));
}

#[test]
fn a_boot_arming_that_fails_is_retried_on_the_next_tick() {
    let device = SimDevice::with_image("1.0.0");
    assert_eq!(device.start(ResetCause::External), None);
    device.failing_arms.set(1);
    block_on(device.runtime().arm_after_boot());
    assert_eq!(device.lifecycle(), Lifecycle::Running);
    assert!(matches!(
        device.events.borrow()[0],
        Event::ArmingRetried(Error::Timeout)
    ));
    assert_eq!(
        upload(&device, "1.1.0"),
        Err(Error::Refused(Refusal::BootNotReady))
    );
    assert_eq!(block_on(device.runtime().tick(&mut Trial::default())), None);
    assert_eq!(device.lifecycle(), Lifecycle::Armed);
    assert!(!device.state.borrow().arming_pending);
}

#[test]
fn a_candidate_that_hangs_in_its_boot_check_is_rolled_back_by_the_restart() {
    let device = armed();
    assert_eq!(upload(&device, "1.1.0"), Ok(Restart));
    let hung = device.restarted("1.1.0");
    hung.hang.set(true);
    let slot = hung.bootloader_slot();
    assert!(poll_once(hung.runtime().start(|_| Ok(slot), ResetCause::Other)).is_pending());
    // The watchdog restarts the chip and the bootloader starts the
    // candidate again: this time it is a failed trial.
    let again = hung.restarted("1.1.0");
    assert_eq!(again.start(ResetCause::Other), Some(Restart));
    assert_eq!(stored(&again).update().ending(), Some(Ending::RolledBack));
    assert_eq!(stored(&again).rejected(), Some(Version::new(1, 1, 0)));
    let back = again.restarted("1.0.0");
    assert_eq!(back.start(ResetCause::Other), None);
    assert_eq!(back.record().current().version(), Version::new(1, 0, 0));
}

#[test]
fn an_abandoned_transfer_keeps_the_lock_and_restarts_to_clean_up() {
    let device = armed();
    let image = release_image("1.1.0");
    let header = signed(
        Version::new(1, 1, 0),
        ImageLen::of(&image).unwrap(),
        Digest::of(&image),
    );
    block_on(async {
        let mut guard = device.runtime().acquire(Operation::Update).await.unwrap();
        let manifest = guard.verify(&header).unwrap();
        let mut buffers = Buffers::new();
        let mut writer = guard.begin(manifest, &mut buffers).await.unwrap();
        guard.push(&mut writer, &image[..5000]).await.unwrap();
        // Dropped here, without an abort.
    });
    assert!(device.state.borrow().busy());
    assert!(device.inhibited.get());
    assert_eq!(
        device.events.borrow().last(),
        Some(&Event::TransferAbandoned)
    );
    assert_eq!(
        block_on(device.runtime().tick(&mut Trial::default())),
        Some(Restart)
    );
    let next = device.restarted("1.0.0");
    assert_eq!(next.start(ResetCause::Other), None);
    assert_eq!(stored(&next).update().ending(), Some(Ending::Aborted));
    block_on(next.runtime().arm_after_boot());
    assert_eq!(upload(&next, "1.1.0"), Ok(Restart));
}

#[test]
fn a_transfer_start_that_fails_before_touching_flash_only_fails_the_transfer() {
    let device = armed();
    let before = stored(&device);
    {
        let mut flash = device.flash.borrow_mut();
        flash.start_log();
        // Two journal reads, then the erase of the sector the save writes.
        flash.cut = Some((2, 0));
    }
    assert_eq!(
        upload(&device, "1.1.0"),
        Err(Error::Flash(FlashError::Driver(NorFlashErrorKind::Other)))
    );
    assert_eq!(
        device.flash.borrow().mutable[2],
        SECTOR_SIZE,
        "the cut hit the erase"
    );
    device.flash.borrow_mut().cut = None;
    assert_eq!(device.recovery.get(), None);
    assert_eq!(stored(&device), before);
    assert!(!device.state.borrow().busy());
    device.advance(30 * SECOND);
    assert_eq!(upload(&device, "1.1.0"), Ok(Restart));
}

#[test]
fn a_candidate_is_not_tried_when_the_image_to_fall_back_to_is_damaged() {
    let device = armed();
    assert_eq!(upload(&device, "1.1.0"), Ok(Restart));
    let next = device.restarted("1.1.0");
    next.flash.borrow_mut().bytes[Slot::A.offset() as usize + 5000] ^= 1;
    assert_eq!(next.start(ResetCause::Other), None);
    assert_eq!(next.lifecycle(), Lifecycle::Recovery);
    assert!(next.recovery.get().is_some());
}

#[test]
fn the_trial_countdown_is_shown_when_its_whole_seconds_change() {
    let device = on_trial();
    let mut trial = Trial::default();
    for _ in 0..6 {
        device.advance(SECOND / 2);
        assert_eq!(block_on(device.runtime().tick(&mut trial)), None);
    }
    let shown: std::vec::Vec<_> = device
        .shown
        .borrow()
        .iter()
        .filter_map(|notice| match notice {
            Notice::Trial {
                left: Some(left), ..
            } => Some(left.as_secs()),
            _ => None,
        })
        .collect();
    assert_eq!(shown, [60, 59, 58, 57]);
}

#[test]
fn a_flash_failure_in_the_boot_check_does_not_reject_the_candidate() {
    let device = armed();
    assert_eq!(upload(&device, "1.1.0"), Ok(Restart));
    let mut failed_checks = 0;
    // The first two calls belong to the simulated bootloader.
    for call in 2..40 {
        let next = device.restarted("1.1.0");
        next.flash.borrow_mut().start_log();
        next.flash.borrow_mut().cut = Some((call, 0));
        next.start(ResetCause::Other);
        next.flash.borrow_mut().cut = None;
        if !matches!(
            next.events.borrow().first(),
            Some(Event::CandidateFailedBootCheck(Error::Flash(_)))
        ) {
            continue;
        }
        failed_checks += 1;
        assert_eq!(
            stored(&next).update().ending(),
            Some(Ending::Aborted),
            "call {call}"
        );
        assert_eq!(stored(&next).rejected(), None, "call {call}");
        assert_eq!(
            next.flash.borrow().bytes[Slot::B.offset() as usize],
            0xE9,
            "call {call}: the running candidate must not be erased under it"
        );
    }
    assert!(failed_checks > 0);
}

#[test]
fn update_attempts_are_spaced_and_provisioning_does_not_count_as_one() {
    let device = armed();
    let acquire = |operation| block_on(device.runtime().acquire(operation)).map(drop);
    assert_eq!(acquire(Operation::Provisioning), Ok(()));
    assert_eq!(acquire(Operation::Update), Ok(()));
    assert_eq!(
        acquire(Operation::Update),
        Err(Error::Refused(Refusal::Busy))
    );
    device.advance(SECOND);
    assert_eq!(acquire(Operation::Update), Ok(()));
}

#[test]
fn a_tick_arms_the_boot_only_while_an_arming_is_pending() {
    let device = armed();
    device.failing_arms.set(1);
    assert_eq!(block_on(device.runtime().tick(&mut Trial::default())), None);
    assert!(device.events.borrow().is_empty());
    assert_eq!(device.failing_arms.get(), 1, "no arming was attempted");
}
