//! Track output ownership boundary and RailCom cutout driver.
//!
//! Sole owner of the track `SLEEP` master-enable pin (`GPIO18`) and fast
//! RailCom cutout/run pin (`GPIO4`). The RMT boundary ISR starts the cutout
//! sequence and a hardware TIMG timer opens/closes the physical cutout.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU8, AtomicU32, Ordering};

use critical_section::with as critical_section_with;
#[cfg(target_arch = "riscv32")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(target_arch = "riscv32")]
use embassy_sync::channel::Receiver;
use esp_hal::Blocking;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::handler;
use esp_hal::interrupt::{InterruptHandler, Priority};
use esp_hal::ram;
use esp_hal::timer::OneShotTimer;
use esp_hal::timer::timg;
use heapless::Vec;
use static_cell::StaticCell;

use crate::dcc::{DccAddress, PackedDccAddress};
use crate::railcom::pipeline::RailcomChannel;

/// Cutout duration in microseconds. NMRA S-9.3.2 requires 454..=488 µs; 460 µs
/// sits comfortably in the middle of that window.
const CUTOUT_DURATION_US: u32 = 460;
/// Delay from the last DCC packet edge to the physical cutout start.
const CUTOUT_START_DEADTIME_US: u32 = 29;
/// RailCom receive channel timing, relative to the physical cutout start.
const RAILCOM_CH1_START_US: u32 = 80;
const RAILCOM_CH1_END_US: u32 = 177;
const RAILCOM_CH2_START_US: u32 = 193;
const RAILCOM_CH2_END_US: u32 = 454;

const RAILCOM_CH1_OPEN_DELAY_US: u32 = RAILCOM_CH1_START_US;
const RAILCOM_CH1_DURATION_US: u32 = RAILCOM_CH1_END_US - RAILCOM_CH1_START_US;
const RAILCOM_CH1_TO_CH2_GAP_US: u32 = RAILCOM_CH2_START_US - RAILCOM_CH1_END_US;
const RAILCOM_CH2_DURATION_US: u32 = RAILCOM_CH2_END_US - RAILCOM_CH2_START_US;
const RAILCOM_POST_CH2_CLOSE_US: u32 = CUTOUT_DURATION_US - RAILCOM_CH2_END_US;

const _: () = assert!(CUTOUT_DURATION_US >= 454 && CUTOUT_DURATION_US <= 488);
const _: () = assert!(RAILCOM_CH1_START_US >= 75);
const _: () = assert!(RAILCOM_CH1_START_US < RAILCOM_CH1_END_US);
const _: () = assert!(RAILCOM_CH1_END_US < RAILCOM_CH2_START_US);
const _: () = assert!(RAILCOM_CH2_START_US < RAILCOM_CH2_END_US);
const _: () = assert!(RAILCOM_CH2_END_US <= CUTOUT_DURATION_US);

const CUTOUT_STATE_IDLE: u8 = 0;
const CUTOUT_STATE_WAITING_TO_OPEN: u8 = 1;
const CUTOUT_STATE_WAITING_CH1_OPEN: u8 = 2;
const CUTOUT_STATE_CH1_OPEN: u8 = 3;
const CUTOUT_STATE_WAITING_CH2_OPEN: u8 = 4;
const CUTOUT_STATE_CH2_OPEN: u8 = 5;
const CUTOUT_STATE_WAITING_TO_CLOSE: u8 = 6;

static CUTOUT_REQUEST_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_STARTED_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_SKIPPED_DISABLED_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_ENDED_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_SCHEDULE_FAIL_COUNT: AtomicU32 = AtomicU32::new(0);

pub(crate) const CUTOUT_EVENT_RING_CAPACITY: usize = 32;
static CUTOUT_EVENT_SEQUENCES: [AtomicU32; CUTOUT_EVENT_RING_CAPACITY] =
    [const { AtomicU32::new(0) }; CUTOUT_EVENT_RING_CAPACITY];
static CUTOUT_EVENT_METADATA: [AtomicU32; CUTOUT_EVENT_RING_CAPACITY] =
    [const { AtomicU32::new(0) }; CUTOUT_EVENT_RING_CAPACITY];
static CUTOUT_EVENT_WRITE_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_EVENT_DROPPED_COUNT: AtomicU32 = AtomicU32::new(0);
static CUTOUT_EVENT_NOTIFY_FAIL_COUNT: AtomicU32 = AtomicU32::new(0);

// Ring size for the packet_sequence -> RailCom metadata side channel.
//
// TODO(tuning): no recorded derivation for `64`. Observable constraint: a
// consumer (e.g. `pom_dispatch`) must read back a given `packet_sequence`'s
// metadata before that slot is recycled by 64 further cutouts; at typical
// scheduler cadence that is on the order of seconds of slack, which is ample
// for an Embassy task woken by a channel notification rather than polling.
const RAILCOM_PACKET_META_CAPACITY: usize = 64;
static RAILCOM_PACKET_META_SEQUENCES: [AtomicU32; RAILCOM_PACKET_META_CAPACITY] =
    [const { AtomicU32::new(0) }; RAILCOM_PACKET_META_CAPACITY];
static RAILCOM_PACKET_META_WORDS: [AtomicU32; RAILCOM_PACKET_META_CAPACITY] =
    [const { AtomicU32::new(0) }; RAILCOM_PACKET_META_CAPACITY];

#[cfg(target_arch = "riscv32")]
type CutoutEventNotifyChannel = embassy_sync::channel::Channel<CriticalSectionRawMutex, u8, 1>;
#[cfg(target_arch = "riscv32")]
static CUTOUT_EVENT_NOTIFY: CutoutEventNotifyChannel = CutoutEventNotifyChannel::new();

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CutoutRuntimeEvent {
    Opened {
        packet_sequence: u32,
        pom_requested: bool,
        pom_read_requested: bool,
        target_address: Option<DccAddress>,
    },
}

static TRACK_ENABLED: AtomicBool = AtomicBool::new(false);
static CUTOUT_STATE: AtomicU8 = AtomicU8::new(CUTOUT_STATE_IDLE);
static PENDING_CUTOUT_PACKET_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static PENDING_CUTOUT_METADATA: AtomicU32 = AtomicU32::new(0);

// `RAILCOM_WINDOW_PACKET_SEQUENCE`/`RAILCOM_WINDOW_CHANNEL` are a functional
// guard, not telemetry: `close_realtime_window_from_isr` uses them to confirm
// the window it is about to close still belongs to the window
// `open_realtime_window_from_isr` most recently opened, before forwarding the
// close to `isr_capture`'s FIFO read. A previously-existing
// `RAILCOM_WINDOW_GENERATION`/`RAILCOM_WINDOW_ACTIVE` pair (plus a
// `railcom_realtime_window_snapshot()` public accessor) tracked the same
// open/close transitions purely for external observability and had no
// callers anywhere in the crate (confirmed by grep before removal); it was
// deleted here without touching this guard.
static RAILCOM_WINDOW_PACKET_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static RAILCOM_WINDOW_CHANNEL: AtomicU8 = AtomicU8::new(0);
static TRACK_OUTPUT_HW: StaticCell<TrackOutputHwCell> = StaticCell::new();
static TRACK_OUTPUT_HW_READY: AtomicBool = AtomicBool::new(false);
static TRACK_OUTPUT_HW_PTR: AtomicPtr<TrackOutputHwCell> = AtomicPtr::new(core::ptr::null_mut());

struct TrackOutputHwCell(UnsafeCell<TrackOutputHw>);

// SAFETY: access to the contained hardware is serialized by `CUTOUT_STATE`.
// Task-side access additionally runs in a critical section. RMT and TIMG
// interrupts are registered at the same priority, so they cannot preempt each
// other on ESP32-C6 while one of them holds the singleton.
unsafe impl Sync for TrackOutputHwCell {}

struct TrackOutputHw {
    enable: Output<'static>,
    cutout: Output<'static>,
    cutout_timer: OneShotTimer<'static, Blocking>,
}

/// Snapshot of track-output cutout counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub struct TrackOutputStats {
    pub cutout_request_count: u32,
    pub cutout_started_count: u32,
    pub cutout_event_dropped_count: u32,
    pub cutout_event_notify_fail_count: u32,
    pub cutout_skipped_disabled_count: u32,
    pub cutout_ended_count: u32,
    pub cutout_schedule_fail_count: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RailcomRealtimeWindowSnapshot {
    pub generation: u32,
    pub active: bool,
    pub packet_sequence: u32,
    pub channel: Option<RailcomChannel>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RailcomPacketMetadata {
    pub target_address: Option<DccAddress>,
    pub pom_requested: bool,
    pub pom_read_requested: bool,
}

/// Bit-packed cutout metadata: `PackedAddressFlags` carries the target
/// address plus two flags (`pom_requested`, `pom_read_requested`). See
/// `PackedAddressFlags` for the shared layout also used by
/// `railcom::pom_dispatch::PendingPomRailcomMetadata`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PackedRailcomPacketMetadata(PackedAddressFlags);

impl PackedRailcomPacketMetadata {
    const POM_REQUESTED_BIT: u32 = 0;
    const POM_READ_REQUESTED_BIT: u32 = 1;

    const fn from_raw(raw: u32) -> Self {
        Self(PackedAddressFlags::from_raw(raw))
    }

    const fn raw(self) -> u32 {
        self.0.raw()
    }

    fn new(
        target_address: Option<DccAddress>,
        pom_requested: bool,
        pom_read_requested: bool,
    ) -> Self {
        Self(
            PackedAddressFlags::new(target_address)
                .with_flag(Self::POM_REQUESTED_BIT, pom_requested)
                .with_flag(Self::POM_READ_REQUESTED_BIT, pom_read_requested),
        )
    }

    fn target_address(self) -> Option<DccAddress> {
        self.0.address()
    }

    const fn pom_requested(self) -> bool {
        self.0.flag(Self::POM_REQUESTED_BIT)
    }

    const fn pom_read_requested(self) -> bool {
        self.0.flag(Self::POM_READ_REQUESTED_BIT)
    }

    fn runtime_event(self, packet_sequence: u32) -> CutoutRuntimeEvent {
        CutoutRuntimeEvent::Opened {
            packet_sequence,
            pom_requested: self.pom_requested(),
            pom_read_requested: self.pom_read_requested(),
            target_address: self.target_address(),
        }
    }

    fn public_metadata(self) -> RailcomPacketMetadata {
        RailcomPacketMetadata {
            target_address: self.target_address(),
            pom_requested: self.pom_requested(),
            pom_read_requested: self.pom_read_requested(),
        }
    }
}

/// Handle used by normal runtime code.
///
/// The actual hardware state lives in a singleton so tasks and ISRs reach the
/// same physical owner without duplicating GPIO18/SLEEP control.
#[derive(Debug)]
pub struct TrackOutput;

impl TrackOutput {
    /// Initialize the singleton track-output owner.
    #[must_use]
    pub fn new(
        gpio18: esp_hal::peripherals::GPIO18<'static>,
        gpio4: esp_hal::peripherals::GPIO4<'static>,
        timer0: timg::Timer<'static>,
    ) -> Self {
        let mut cutout_timer = OneShotTimer::new(timer0);
        cutout_timer.set_interrupt_handler(InterruptHandler::new(
            cutout_timer_interrupt.handler().aligned_ptr(),
            Priority::Priority2,
        ));
        cutout_timer.listen();

        let hw = TRACK_OUTPUT_HW.init(TrackOutputHwCell(UnsafeCell::new(TrackOutputHw {
            enable: Output::new(gpio18, Level::Low, OutputConfig::default()),
            // GPIO4 is HIGH for normal DCC output and LOW for the timed
            // RailCom cutout. Hardware may wire it directly to DRV8874 EN/IN1
            // in PH/EN mode or use it as DCC_RUN for external PWM-mode logic.
            cutout: Output::new(gpio4, Level::High, OutputConfig::default()),
            cutout_timer,
        })));
        TRACK_OUTPUT_HW_PTR.store(hw as *const _ as *mut _, Ordering::Release);
        let was_ready = TRACK_OUTPUT_HW_READY.swap(true, Ordering::AcqRel);
        assert!(!was_ready, "TrackOutput must be initialized only once");
        let _ = hw;

        TRACK_ENABLED.store(false, Ordering::Release);
        CUTOUT_STATE.store(CUTOUT_STATE_IDLE, Ordering::Release);
        CUTOUT_EVENT_WRITE_COUNT.store(0, Ordering::Release);
        CUTOUT_EVENT_DROPPED_COUNT.store(0, Ordering::Release);
        CUTOUT_EVENT_NOTIFY_FAIL_COUNT.store(0, Ordering::Release);

        Self
    }

    /// Applies the normal track power state.
    pub fn set_track_enabled(&mut self, enabled: bool) {
        TRACK_ENABLED.store(enabled, Ordering::Release);

        critical_section_with(|_| {
            let Some(hw) = hw_mut() else {
                return;
            };

            if !enabled {
                stop_cutout_timer_fast(hw);
                cutout_off_fast(hw);
                enable_low_fast(hw);
                CUTOUT_STATE.store(CUTOUT_STATE_IDLE, Ordering::Release);
            } else if CUTOUT_STATE.load(Ordering::Acquire) == CUTOUT_STATE_IDLE && enabled {
                enable_high_fast(hw);
            }
        });
    }
}

#[inline(always)]
fn hw_mut() -> Option<&'static mut TrackOutputHw> {
    if !TRACK_OUTPUT_HW_READY.load(Ordering::Acquire) {
        None
    } else {
        let ptr = TRACK_OUTPUT_HW_PTR.load(Ordering::Acquire);
        if ptr.is_null() {
            return None;
        }
        // SAFETY: `TrackOutputHwCell` documents the singleton ownership invariant.
        // `CUTOUT_STATE` remains non-idle until every ISR-side hardware access is
        // complete, preventing RMT from entering while TIMG owns the hardware.
        Some(unsafe { &mut *(*ptr).0.get() })
    }
}

// NOTE: These `*_fast()` wrappers intentionally isolate the ISR/timer critical path.
// Today they still use esp-hal internally; if future multi-loco/WiFi stress tests show
// timing margin issues, only these wrappers should be rewritten to more direct
// register-level access. Keep policy and branching out of this layer.
#[inline(always)]
fn enable_low_fast(hw: &mut TrackOutputHw) {
    hw.enable.set_low();
}

#[inline(always)]
fn enable_high_fast(hw: &mut TrackOutputHw) {
    hw.enable.set_high();
}

#[inline(always)]
fn cutout_on_fast(hw: &mut TrackOutputHw) {
    hw.cutout.set_low();
}

#[inline(always)]
fn cutout_off_fast(hw: &mut TrackOutputHw) {
    hw.cutout.set_high();
}

#[inline(always)]
fn schedule_timer_us_fast(hw: &mut TrackOutputHw, delay_us: u32) -> bool {
    hw.cutout_timer
        .schedule(esp_hal::time::Duration::from_micros(delay_us as u64))
        .is_ok()
}

#[inline(always)]
fn clear_cutout_timer_interrupt_fast(hw: &mut TrackOutputHw) {
    hw.cutout_timer.clear_interrupt();
}

#[inline(always)]
fn stop_cutout_timer_fast(hw: &mut TrackOutputHw) {
    hw.cutout_timer.stop();
}

#[inline(always)]
fn open_realtime_window_from_isr(packet_sequence: u32, channel: RailcomChannel) {
    RAILCOM_WINDOW_PACKET_SEQUENCE.store(packet_sequence, Ordering::Release);
    RAILCOM_WINDOW_CHANNEL.store(u8::from(channel), Ordering::Release);
    RAILCOM_WINDOW_ACTIVE.store(true, Ordering::Release);
    RAILCOM_WINDOW_GENERATION.fetch_add(1, Ordering::AcqRel);
    #[cfg(target_arch = "riscv32")]
    crate::railcom::isr_capture::open_window_from_isr();
}

#[inline(always)]
fn close_realtime_window_from_isr(packet_sequence: u32, channel: RailcomChannel) {
    if RAILCOM_WINDOW_PACKET_SEQUENCE.load(Ordering::Acquire) == packet_sequence
        && RAILCOM_WINDOW_CHANNEL.load(Ordering::Acquire) == u8::from(channel)
    {
        #[cfg(target_arch = "riscv32")]
        crate::railcom::isr_capture::close_window_from_isr(packet_sequence, channel);
    }
}

#[must_use]
pub fn railcom_realtime_window_snapshot() -> RailcomRealtimeWindowSnapshot {
    let generation = RAILCOM_WINDOW_GENERATION.load(Ordering::Acquire);
    let active = RAILCOM_WINDOW_ACTIVE.load(Ordering::Acquire);
    let packet_sequence = RAILCOM_WINDOW_PACKET_SEQUENCE.load(Ordering::Acquire);
    let channel = RailcomChannel::try_from(RAILCOM_WINDOW_CHANNEL.load(Ordering::Acquire)).ok();

    RailcomRealtimeWindowSnapshot {
        generation,
        active,
        packet_sequence,
        channel,
    }
}

#[inline(always)]
fn record_railcom_packet_metadata_from_isr(
    packet_sequence: u32,
    pom_requested: bool,
    pom_read_requested: bool,
    target_address: Option<DccAddress>,
) {
    let slot = packet_sequence as usize % RAILCOM_PACKET_META_CAPACITY;
    let metadata =
        PackedRailcomPacketMetadata::new(target_address, pom_requested, pom_read_requested);
    RAILCOM_PACKET_META_SEQUENCES[slot].store(0, Ordering::Release);
    RAILCOM_PACKET_META_WORDS[slot].store(metadata.raw(), Ordering::Release);
    RAILCOM_PACKET_META_SEQUENCES[slot].store(packet_sequence, Ordering::Release);
}

// Same seqlock-style publication as `record_railcom_packet_metadata_from_isr`
// above (zero -> payload -> real sequence); see that function's comment for
// why the zero pre-store must not be dropped.
#[inline(always)]
fn push_cutout_event_from_isr(event: CutoutRuntimeEvent) {
    let write_count = CUTOUT_EVENT_WRITE_COUNT.load(Ordering::Relaxed);
    let slot = write_count as usize % CUTOUT_EVENT_RING_CAPACITY;

    let CutoutRuntimeEvent::Opened {
        packet_sequence,
        pom_requested,
        pom_read_requested,
        target_address,
    } = event;
    let metadata =
        PackedRailcomPacketMetadata::new(target_address, pom_requested, pom_read_requested);
    CUTOUT_EVENT_SEQUENCES[slot].store(0, Ordering::Release);
    CUTOUT_EVENT_METADATA[slot].store(metadata.raw(), Ordering::Release);
    CUTOUT_EVENT_SEQUENCES[slot].store(packet_sequence, Ordering::Release);

    CUTOUT_EVENT_WRITE_COUNT.store(write_count.wrapping_add(1), Ordering::Release);

    #[cfg(target_arch = "riscv32")]
    {
        if CUTOUT_EVENT_NOTIFY.sender().try_send(0).is_err() {
            CUTOUT_EVENT_NOTIFY_FAIL_COUNT.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Requests a physical empty cutout from the RMT packet-boundary ISR.
///
/// This function is intentionally minimal: no policy, no mutex, only the
/// hardware transition required to start the cutout.
#[inline(always)]
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".rwtext"))]
pub fn request_cutout_from_isr(
    packet_sequence: u32,
    pom_requested: bool,
    pom_read_requested: bool,
    target_address: Option<DccAddress>,
) -> bool {
    CUTOUT_REQUEST_COUNT.fetch_add(1, Ordering::Relaxed);

    if !TRACK_ENABLED.load(Ordering::Acquire)
        || CUTOUT_STATE
            .compare_exchange(
                CUTOUT_STATE_IDLE,
                CUTOUT_STATE_WAITING_TO_OPEN,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_err()
    {
        CUTOUT_SKIPPED_DISABLED_COUNT.fetch_add(1, Ordering::Relaxed);
        return false;
    }

    let Some(hw) = hw_mut() else {
        CUTOUT_STATE.store(CUTOUT_STATE_IDLE, Ordering::Release);
        CUTOUT_SKIPPED_DISABLED_COUNT.fetch_add(1, Ordering::Relaxed);
        return false;
    };
    // Keep GPIO18/SLEEP unchanged during RailCom. The DRV8874 sleep/wake path
    // is too slow for the sub-millisecond cutout, so GPIO4 owns this sequence.
    cutout_off_fast(hw);
    PENDING_CUTOUT_PACKET_SEQUENCE.store(packet_sequence, Ordering::Release);
    PENDING_CUTOUT_METADATA.store(
        PackedRailcomPacketMetadata::new(target_address, pom_requested, pom_read_requested).raw(),
        Ordering::Release,
    );
    record_railcom_packet_metadata_from_isr(
        packet_sequence,
        pom_requested,
        pom_read_requested,
        target_address,
    );

    if schedule_timer_us_fast(hw, CUTOUT_START_DEADTIME_US) {
        true
    } else {
        cutout_off_fast(hw);
        CUTOUT_SCHEDULE_FAIL_COUNT.fetch_add(1, Ordering::Relaxed);
        CUTOUT_STATE.store(CUTOUT_STATE_IDLE, Ordering::Release);
        false
    }
}

/// Returns a copy of the current cutout statistics.
#[must_use]
pub fn stats() -> TrackOutputStats {
    TrackOutputStats {
        cutout_request_count: CUTOUT_REQUEST_COUNT.load(Ordering::Acquire),
        cutout_started_count: CUTOUT_STARTED_COUNT.load(Ordering::Acquire),
        cutout_event_dropped_count: CUTOUT_EVENT_DROPPED_COUNT.load(Ordering::Acquire),
        cutout_event_notify_fail_count: CUTOUT_EVENT_NOTIFY_FAIL_COUNT.load(Ordering::Acquire),
        cutout_skipped_disabled_count: CUTOUT_SKIPPED_DISABLED_COUNT.load(Ordering::Acquire),
        cutout_ended_count: CUTOUT_ENDED_COUNT.load(Ordering::Acquire),
        cutout_schedule_fail_count: CUTOUT_SCHEDULE_FAIL_COUNT.load(Ordering::Acquire),
    }
}

// Reader half of the seqlock-style publication documented on
// `record_railcom_packet_metadata_from_isr` above: do not drop the
// `sequence_after` re-check, it is what actually detects a torn read.
#[must_use]
pub fn railcom_packet_metadata(packet_sequence: u32) -> Option<RailcomPacketMetadata> {
    let slot = packet_sequence as usize % RAILCOM_PACKET_META_CAPACITY;
    let sequence_before = RAILCOM_PACKET_META_SEQUENCES[slot].load(Ordering::Acquire);
    if sequence_before != packet_sequence {
        return None;
    }

    let metadata = PackedRailcomPacketMetadata::from_raw(
        RAILCOM_PACKET_META_WORDS[slot].load(Ordering::Acquire),
    );
    let sequence_after = RAILCOM_PACKET_META_SEQUENCES[slot].load(Ordering::Acquire);
    (sequence_after == sequence_before).then(|| metadata.public_metadata())
}

/// Drain cutout runtime events observed since `next_read`.
///
/// If the producer outruns the consumer beyond ring capacity, the oldest
/// events are dropped and counted in diagnostics.
///
/// This ring-drain shape (write counter + modulo slot + overflow accounting)
/// is duplicated in `railcom::isr_capture::drain_captured_windows`. The two
/// are not unified into one generic helper: this ring additionally verifies
/// per-slot data with the seqlock check above (`sequence_before == 0 ||
/// sequence_before != sequence_after` => drop), uses `saturating_sub` for the
/// backlog calculation, and stores a packed `u32` word per slot, whereas the
/// `isr_capture` ring stores a small byte array per slot, uses
/// `wrapping_sub`, and relies on a release/acquire barrier on the write
/// counter instead of a per-slot sequence check (see the comment there for
/// why that is sufficient in that case). A shared generic ring would need to
/// abstract over both invariants and slot layouts without being able to
/// verify by inspection that the abstraction preserves either one, so the
/// two are kept side by side and cross-referenced instead.
pub fn drain_cutout_runtime_events(
    next_read: &mut u32,
    out: &mut Vec<CutoutRuntimeEvent, CUTOUT_EVENT_RING_CAPACITY>,
) {
    let write = CUTOUT_EVENT_WRITE_COUNT.load(Ordering::Acquire);
    let available = write.saturating_sub(*next_read);
    if available > CUTOUT_EVENT_RING_CAPACITY as u32 {
        let dropped = available - CUTOUT_EVENT_RING_CAPACITY as u32;
        *next_read = next_read.wrapping_add(dropped);
        CUTOUT_EVENT_DROPPED_COUNT.fetch_add(dropped, Ordering::Relaxed);
    }

    while *next_read < write && out.len() < CUTOUT_EVENT_RING_CAPACITY {
        let slot = *next_read as usize % CUTOUT_EVENT_RING_CAPACITY;
        let sequence_before = CUTOUT_EVENT_SEQUENCES[slot].load(Ordering::Acquire);
        let metadata = PackedRailcomPacketMetadata::from_raw(
            CUTOUT_EVENT_METADATA[slot].load(Ordering::Acquire),
        );
        let sequence_after = CUTOUT_EVENT_SEQUENCES[slot].load(Ordering::Acquire);
        if sequence_before != 0 && sequence_before == sequence_after {
            let _ = out.push(metadata.runtime_event(sequence_before));
        } else {
            CUTOUT_EVENT_DROPPED_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        *next_read = next_read.wrapping_add(1);
    }
}

#[cfg(target_arch = "riscv32")]
#[must_use]
pub fn cutout_event_notify_receiver() -> Receiver<'static, CriticalSectionRawMutex, u8, 1> {
    CUTOUT_EVENT_NOTIFY.receiver()
}

#[handler(priority = Priority::Priority2)]
#[ram]
fn cutout_timer_interrupt() {
    let Some(hw) = hw_mut() else {
        return;
    };

    clear_cutout_timer_interrupt_fast(hw);
    stop_cutout_timer_fast(hw);

    match CUTOUT_STATE.load(Ordering::Acquire) {
        CUTOUT_STATE_WAITING_TO_OPEN => {
            cutout_on_fast(hw);
            if schedule_timer_us_fast(hw, RAILCOM_CH1_OPEN_DELAY_US) {
                CUTOUT_STATE.store(CUTOUT_STATE_WAITING_CH1_OPEN, Ordering::Release);
                let pending_metadata = PackedRailcomPacketMetadata::from_raw(
                    PENDING_CUTOUT_METADATA.load(Ordering::Acquire),
                );
                push_cutout_event_from_isr(CutoutRuntimeEvent::Opened {
                    packet_sequence: PENDING_CUTOUT_PACKET_SEQUENCE.load(Ordering::Acquire),
                    pom_requested: pending_metadata.pom_requested(),
                    pom_read_requested: pending_metadata.pom_read_requested(),
                    target_address: pending_metadata.target_address(),
                });
                CUTOUT_STARTED_COUNT.fetch_add(1, Ordering::Relaxed);
            } else {
                cutout_off_fast(hw);
                CUTOUT_SCHEDULE_FAIL_COUNT.fetch_add(1, Ordering::Relaxed);
                CUTOUT_STATE.store(CUTOUT_STATE_IDLE, Ordering::Release);
            }
        }
        CUTOUT_STATE_WAITING_CH1_OPEN => {
            let packet_sequence = PENDING_CUTOUT_PACKET_SEQUENCE.load(Ordering::Acquire);
            if schedule_timer_us_fast(hw, RAILCOM_CH1_DURATION_US) {
                CUTOUT_STATE.store(CUTOUT_STATE_CH1_OPEN, Ordering::Release);
                open_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel1);
            } else {
                cutout_off_fast(hw);
                CUTOUT_SCHEDULE_FAIL_COUNT.fetch_add(1, Ordering::Relaxed);
                CUTOUT_STATE.store(CUTOUT_STATE_IDLE, Ordering::Release);
            }
        }
        CUTOUT_STATE_CH1_OPEN => {
            let packet_sequence = PENDING_CUTOUT_PACKET_SEQUENCE.load(Ordering::Acquire);
            if schedule_timer_us_fast(hw, RAILCOM_CH1_TO_CH2_GAP_US) {
                CUTOUT_STATE.store(CUTOUT_STATE_WAITING_CH2_OPEN, Ordering::Release);
                close_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel1);
            } else {
                cutout_off_fast(hw);
                close_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel1);
                CUTOUT_SCHEDULE_FAIL_COUNT.fetch_add(1, Ordering::Relaxed);
                CUTOUT_STATE.store(CUTOUT_STATE_IDLE, Ordering::Release);
            }
        }
        CUTOUT_STATE_WAITING_CH2_OPEN => {
            let packet_sequence = PENDING_CUTOUT_PACKET_SEQUENCE.load(Ordering::Acquire);
            if schedule_timer_us_fast(hw, RAILCOM_CH2_DURATION_US) {
                CUTOUT_STATE.store(CUTOUT_STATE_CH2_OPEN, Ordering::Release);
                open_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel2);
            } else {
                cutout_off_fast(hw);
                CUTOUT_SCHEDULE_FAIL_COUNT.fetch_add(1, Ordering::Relaxed);
                CUTOUT_STATE.store(CUTOUT_STATE_IDLE, Ordering::Release);
            }
        }
        CUTOUT_STATE_CH2_OPEN => {
            let packet_sequence = PENDING_CUTOUT_PACKET_SEQUENCE.load(Ordering::Acquire);
            close_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel2);

            if schedule_timer_us_fast(hw, RAILCOM_POST_CH2_CLOSE_US) {
                CUTOUT_STATE.store(CUTOUT_STATE_WAITING_TO_CLOSE, Ordering::Release);
            } else {
                cutout_off_fast(hw);
                CUTOUT_ENDED_COUNT.fetch_add(1, Ordering::Relaxed);
                CUTOUT_STATE.store(CUTOUT_STATE_IDLE, Ordering::Release);
            }
        }
        CUTOUT_STATE_WAITING_TO_CLOSE => {
            cutout_off_fast(hw);

            CUTOUT_ENDED_COUNT.fetch_add(1, Ordering::Relaxed);
            CUTOUT_STATE.store(CUTOUT_STATE_IDLE, Ordering::Release);
        }
        _ => {
            cutout_off_fast(hw);
            CUTOUT_STATE.store(CUTOUT_STATE_IDLE, Ordering::Release);
        }
    }
}
