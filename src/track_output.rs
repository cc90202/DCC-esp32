//! Track output ownership boundary and RailCom cutout driver.
//!
//! Sole owner of the track `SLEEP` master-enable pin (`GPIO18`) and the
//! RailCom cutout/run pin (`GPIO4`). The RMT packet-boundary interrupt arms a
//! TIMG timer that drives GPIO4 and opens the receiver windows.

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
use esp_hal::time::Instant;
use esp_hal::timer::OneShotTimer;
use esp_hal::timer::timg;
use heapless::Vec;
use static_cell::StaticCell;

use crate::cutout::timing::{
    CHANNEL2_END_US, CUTOUT_CONTROL_START_US, CutoutTimeline, RX_CAPTURE_START_US,
    RX_CHANNEL_SPLIT_US, RX_ISR_GUARD_US,
};
use crate::cutout::{CutoutMode, PacketSequence, RailcomChannel};
use crate::dcc::{DccAddress, PackedAddressFlags, PomRequestId};
use crate::diagnostics::diagnostic_counters;
use crate::seqlock::SeqSlot;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CutoutState {
    Idle = 0,
    WaitingToOpen = 1,
    WaitingCh1Open = 2,
    Ch1Open = 3,
    // Value 4 is intentionally unused to preserve the historical wire/storage
    // representation while making the gap explicit.
    Ch2Open = 5,
}

impl CutoutState {
    const fn as_u8(self) -> u8 {
        self as u8
    }

    fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::Idle),
            1 => Some(Self::WaitingToOpen),
            2 => Some(Self::WaitingCh1Open),
            3 => Some(Self::Ch1Open),
            5 => Some(Self::Ch2Open),
            _ => None,
        }
    }
}

#[inline(always)]
fn store_cutout_state(state: CutoutState) {
    CUTOUT_STATE.store(state.as_u8(), Ordering::Release);
}

diagnostic_counters! {
    static CUTOUT: TrackOutputCounters;

    /// Snapshot of track-output cutout counters.
    pub snapshot TrackOutputStats;

    cutout_request_count,
    cutout_started_count,
    cutout_event_dropped_count,
    cutout_event_notify_fail_count,
    cutout_skipped_disabled_count,
    cutout_ended_count,
    cutout_schedule_fail_count,
    cutout_stale_recovery_count,
    cutout_invalid_state_count,
    /// Worst observed lateness, published with `fetch_max` rather than added.
    cutout_max_deadline_lateness_us,
}

pub(crate) const CUTOUT_EVENT_RING_CAPACITY: usize = 32;
static CUTOUT_EVENT_SLOTS: [SeqSlot<3>; CUTOUT_EVENT_RING_CAPACITY] =
    [const { SeqSlot::new() }; CUTOUT_EVENT_RING_CAPACITY];
static CUTOUT_EVENT_WRITE_COUNT: AtomicU32 = AtomicU32::new(0);

// Ring size for the packet_sequence -> RailCom metadata side channel.
//
// TODO(tuning): no recorded derivation for `64`. Observable constraint: a
// consumer (e.g. `pom_dispatch`) must read back a given `packet_sequence`'s
// metadata before that slot is recycled by 64 further cutouts; at typical
// scheduler cadence that is on the order of seconds of slack, which is ample
// for an Embassy task woken by a channel notification rather than polling.
const RAILCOM_PACKET_META_CAPACITY: usize = 64;
static RAILCOM_PACKET_META_SLOTS: [SeqSlot<3>; RAILCOM_PACKET_META_CAPACITY] =
    [const { SeqSlot::new() }; RAILCOM_PACKET_META_CAPACITY];

#[cfg(target_arch = "riscv32")]
type CutoutEventNotifyChannel = embassy_sync::channel::Channel<CriticalSectionRawMutex, u8, 1>;
#[cfg(target_arch = "riscv32")]
static CUTOUT_EVENT_NOTIFY: CutoutEventNotifyChannel = CutoutEventNotifyChannel::new();

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum CutoutRuntimeEvent {
    Opened {
        packet_sequence: PacketSequence,
        cutout: CutoutMode,
        target_address: Option<DccAddress>,
        pom_request_id: Option<PomRequestId>,
    },
}

static TRACK_ENABLED: AtomicBool = AtomicBool::new(false);
static CUTOUT_STATE: AtomicU8 = AtomicU8::new(CutoutState::Idle as u8);
static PENDING_CUTOUT_PACKET_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static PENDING_CUTOUT_METADATA: AtomicU32 = AtomicU32::new(0);
static PENDING_CUTOUT_POM_REQUEST_ID: AtomicU32 = AtomicU32::new(0);
static CUTOUT_TIMER_ORIGIN_US: AtomicU32 = AtomicU32::new(0);
static PENDING_PACKET_END_OFFSET_US: AtomicU32 = AtomicU32::new(0);

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RailcomPacketMetadata {
    pub target_address: Option<DccAddress>,
    pub cutout: CutoutMode,
    pub pom_request_id: Option<PomRequestId>,
}

/// Bit-packed cutout metadata: `PackedAddressFlags` carries the target,
/// POM mode, and presence of the separately stored request identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PackedRailcomPacketMetadata(PackedAddressFlags);

impl PackedRailcomPacketMetadata {
    const CUTOUT_MODE_FIELD: u32 = 0;
    const CUTOUT_MODE_WIDTH: u32 = 2;
    const HAS_POM_REQUEST_ID_BIT: u32 = 2;

    const fn from_raw(raw: u32) -> Self {
        Self(PackedAddressFlags::from_raw(raw))
    }

    const fn raw(self) -> u32 {
        self.0.raw()
    }

    fn new(
        target_address: Option<DccAddress>,
        cutout: CutoutMode,
        pom_request_id: Option<PomRequestId>,
    ) -> Self {
        Self(
            PackedAddressFlags::new(target_address)
                .with_field(
                    Self::CUTOUT_MODE_FIELD,
                    Self::CUTOUT_MODE_WIDTH,
                    match cutout {
                        CutoutMode::None => 0,
                        CutoutMode::Telemetry => 1,
                        CutoutMode::PomWrite => 2,
                        CutoutMode::PomRead => 3,
                    },
                )
                .with_flag(Self::HAS_POM_REQUEST_ID_BIT, pom_request_id.is_some()),
        )
    }

    fn target_address(self) -> Option<DccAddress> {
        self.0.address()
    }

    const fn cutout(self) -> CutoutMode {
        match self
            .0
            .field(Self::CUTOUT_MODE_FIELD, Self::CUTOUT_MODE_WIDTH)
        {
            0 => CutoutMode::None,
            1 => CutoutMode::Telemetry,
            2 => CutoutMode::PomWrite,
            3 => CutoutMode::PomRead,
            _ => unreachable!(),
        }
    }

    fn pom_request_id(self, raw_request_id: u32) -> Option<PomRequestId> {
        self.0
            .flag(Self::HAS_POM_REQUEST_ID_BIT)
            .then(|| PomRequestId::new(raw_request_id))
    }

    fn runtime_event(
        self,
        packet_sequence: PacketSequence,
        raw_request_id: u32,
    ) -> CutoutRuntimeEvent {
        CutoutRuntimeEvent::Opened {
            packet_sequence,
            cutout: self.cutout(),
            target_address: self.target_address(),
            pom_request_id: self.pom_request_id(raw_request_id),
        }
    }

    fn public_metadata(self, raw_request_id: u32) -> RailcomPacketMetadata {
        RailcomPacketMetadata {
            target_address: self.target_address(),
            cutout: self.cutout(),
            pom_request_id: self.pom_request_id(raw_request_id),
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
            Priority::Priority3,
        ));
        cutout_timer.listen();

        let hw = TRACK_OUTPUT_HW.init(TrackOutputHwCell(UnsafeCell::new(TrackOutputHw {
            enable: Output::new(gpio18, Level::Low, OutputConfig::default()),
            cutout: Output::new(gpio4, Level::High, OutputConfig::default()),
            cutout_timer,
        })));
        TRACK_OUTPUT_HW_PTR.store(hw as *const _ as *mut _, Ordering::Release);
        let was_ready = TRACK_OUTPUT_HW_READY.swap(true, Ordering::AcqRel);
        assert!(!was_ready, "TrackOutput must be initialized only once");

        TRACK_ENABLED.store(false, Ordering::Release);
        store_cutout_state(CutoutState::Idle);
        CUTOUT_EVENT_WRITE_COUNT.store(0, Ordering::Release);
        CUTOUT
            .cutout_event_dropped_count
            .store(0, Ordering::Release);
        CUTOUT
            .cutout_event_notify_fail_count
            .store(0, Ordering::Release);

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
                store_cutout_state(CutoutState::Idle);
            } else if enabled {
                match CutoutState::from_u8(CUTOUT_STATE.load(Ordering::Acquire)) {
                    Some(CutoutState::Idle) => enable_high_fast(hw),
                    None => {
                        CUTOUT
                            .cutout_invalid_state_count
                            .fetch_add(1, Ordering::Relaxed);
                        store_cutout_state(CutoutState::Idle);
                        enable_high_fast(hw);
                    }
                    Some(_) => {}
                }
            }
        });
    }
}

/// Immediately disables the track output before asynchronous fault handling.
///
/// This is the fail-safe path for the short detector: policy and latching still
/// belong to the fault manager, but GPIO18 is not left HIGH while a message is
/// waiting in a queue.
pub fn emergency_disable() {
    TRACK_ENABLED.store(false, Ordering::Release);
    critical_section_with(|_| {
        let Some(hw) = hw_mut() else {
            return;
        };
        stop_cutout_timer_fast(hw);
        cutout_off_fast(hw);
        enable_low_fast(hw);
        store_cutout_state(CutoutState::Idle);
    });
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
fn now_us() -> u32 {
    Instant::now().duration_since_epoch().as_micros() as u32
}

#[inline(always)]
fn schedule_cutout_deadline_fast(hw: &mut TrackOutputHw, offset_us: u32) -> bool {
    let elapsed_us = now_us().wrapping_sub(CUTOUT_TIMER_ORIGIN_US.load(Ordering::Acquire));
    schedule_timer_us_fast(hw, offset_us.saturating_sub(elapsed_us).max(1))
}

#[inline(always)]
fn schedule_cutout_deadline_guarded_fast(hw: &mut TrackOutputHw, offset_us: u32) -> bool {
    schedule_cutout_deadline_fast(hw, offset_us.saturating_sub(RX_ISR_GUARD_US))
}

#[inline(always)]
fn wait_until_cutout_offset(expected_offset_us: u32) {
    while now_us().wrapping_sub(CUTOUT_TIMER_ORIGIN_US.load(Ordering::Acquire)) < expected_offset_us
    {
        core::hint::spin_loop();
    }
}

#[inline(always)]
fn record_deadline_lateness(expected_offset_us: u32) {
    let elapsed_us = now_us().wrapping_sub(CUTOUT_TIMER_ORIGIN_US.load(Ordering::Acquire));
    let lateness_us = elapsed_us.saturating_sub(expected_offset_us);
    CUTOUT
        .cutout_max_deadline_lateness_us
        .fetch_max(lateness_us, Ordering::Relaxed);
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
fn open_realtime_window_from_isr(packet_sequence: PacketSequence, channel: RailcomChannel) {
    RAILCOM_WINDOW_PACKET_SEQUENCE.store(packet_sequence.value(), Ordering::Release);
    RAILCOM_WINDOW_CHANNEL.store(u8::from(channel), Ordering::Release);
    #[cfg(target_arch = "riscv32")]
    crate::railcom_capture::open_window_from_isr();
}

#[inline(always)]
fn close_realtime_window_from_isr(packet_sequence: PacketSequence, channel: RailcomChannel) {
    // Guard: only forward the close (and its FIFO read) if this is still the
    // window that was most recently opened. See the comment on
    // `RAILCOM_WINDOW_PACKET_SEQUENCE`/`RAILCOM_WINDOW_CHANNEL` above.
    if RAILCOM_WINDOW_PACKET_SEQUENCE.load(Ordering::Acquire) == packet_sequence.value()
        && RAILCOM_WINDOW_CHANNEL.load(Ordering::Acquire) == u8::from(channel)
    {
        #[cfg(target_arch = "riscv32")]
        crate::railcom_capture::close_window_from_isr(packet_sequence, channel);
    }
}

#[inline(always)]
fn close_stale_window_from_isr(state: CutoutState) {
    let packet_sequence =
        PacketSequence::new(PENDING_CUTOUT_PACKET_SEQUENCE.load(Ordering::Acquire));
    match state {
        CutoutState::Ch1Open => {
            close_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel1);
        }
        CutoutState::Ch2Open => {
            close_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel2);
        }
        _ => {}
    }
}

// The packet sequence remains payload, not an invalidation sentinel: zero is a
// valid value after the u32 counter wraps. SeqSlot owns the odd/even generation
// protocol and all associated memory orderings.
#[inline(always)]
fn record_railcom_packet_metadata_from_isr(
    packet_sequence: PacketSequence,
    metadata_raw: u32,
    pom_request_id_raw: u32,
) {
    let packet_sequence = packet_sequence.value();
    let slot = packet_sequence as usize % RAILCOM_PACKET_META_CAPACITY;
    RAILCOM_PACKET_META_SLOTS[slot].publish([packet_sequence, metadata_raw, pom_request_id_raw]);
}

// SeqSlot keeps the packet sequence a pure payload value (including zero after
// wraparound) and rejects a slot overwritten while it is being copied.
#[inline(always)]
fn push_cutout_event_from_isr(event: CutoutRuntimeEvent) {
    let write_count = CUTOUT_EVENT_WRITE_COUNT.load(Ordering::Relaxed);
    let slot = write_count as usize % CUTOUT_EVENT_RING_CAPACITY;

    let CutoutRuntimeEvent::Opened {
        packet_sequence,
        cutout,
        target_address,
        pom_request_id,
    } = event;
    let metadata = PackedRailcomPacketMetadata::new(target_address, cutout, pom_request_id);
    CUTOUT_EVENT_SLOTS[slot].publish([
        packet_sequence.value(),
        metadata.raw(),
        pom_request_id.map_or(0, PomRequestId::value),
    ]);

    CUTOUT_EVENT_WRITE_COUNT.store(write_count.wrapping_add(1), Ordering::Release);

    #[cfg(target_arch = "riscv32")]
    {
        if CUTOUT_EVENT_NOTIFY.sender().try_send(0).is_err() {
            CUTOUT
                .cutout_event_notify_fail_count
                .fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Complete context captured at an RMT packet boundary for one cutout request.
pub struct CutoutRequest {
    pub packet_boundary_us: u32,
    pub packet_sequence: PacketSequence,
    pub dcc_packet_duration_us: u32,
    pub cutout: CutoutMode,
    pub target_address: Option<DccAddress>,
    pub pom_request_id: Option<PomRequestId>,
}

/// Requests a physical empty cutout from the RMT packet-boundary ISR.
///
/// This function is intentionally minimal: no policy, no mutex, only the
/// hardware transition required to start the cutout.
#[inline(always)]
#[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".rwtext"))]
pub fn request_cutout_from_isr(request: CutoutRequest) -> bool {
    let CutoutRequest {
        packet_boundary_us,
        packet_sequence,
        dcc_packet_duration_us,
        cutout,
        target_address,
        pom_request_id,
    } = request;
    // The RMT handler captures this timestamp at the hardware packet boundary,
    // before counters and shared-state access. Work performed below therefore
    // cannot shift the receiver windows relative to the waveform in flight.
    CUTOUT_TIMER_ORIGIN_US.store(packet_boundary_us, Ordering::Release);
    CUTOUT.cutout_request_count.fetch_add(1, Ordering::Relaxed);

    if !TRACK_ENABLED.load(Ordering::Acquire) {
        CUTOUT
            .cutout_skipped_disabled_count
            .fetch_add(1, Ordering::Relaxed);
        return false;
    }

    let previous_state = CutoutState::from_u8(
        CUTOUT_STATE.swap(CutoutState::WaitingToOpen.as_u8(), Ordering::AcqRel),
    );
    match previous_state {
        Some(CutoutState::Idle) => {}
        Some(state) => {
            CUTOUT
                .cutout_stale_recovery_count
                .fetch_add(1, Ordering::Relaxed);
            close_stale_window_from_isr(state);
        }
        None => {
            CUTOUT
                .cutout_invalid_state_count
                .fetch_add(1, Ordering::Relaxed);
            CUTOUT
                .cutout_stale_recovery_count
                .fetch_add(1, Ordering::Relaxed);
        }
    }

    let Some(hw) = hw_mut() else {
        store_cutout_state(CutoutState::Idle);
        CUTOUT
            .cutout_skipped_disabled_count
            .fetch_add(1, Ordering::Relaxed);
        return false;
    };
    stop_cutout_timer_fast(hw);
    clear_cutout_timer_interrupt_fast(hw);
    // Keep GPIO18/SLEEP unchanged during RailCom. The DRV8874 sleep/wake path
    // is too slow for the sub-millisecond cutout, so GPIO4 owns this sequence.
    cutout_off_fast(hw);
    let metadata_raw =
        PackedRailcomPacketMetadata::new(target_address, cutout, pom_request_id).raw();
    let pom_request_id_raw = pom_request_id.map_or(0, PomRequestId::value);
    PENDING_CUTOUT_PACKET_SEQUENCE.store(packet_sequence.value(), Ordering::Release);
    PENDING_CUTOUT_METADATA.store(metadata_raw, Ordering::Release);
    PENDING_CUTOUT_POM_REQUEST_ID.store(pom_request_id_raw, Ordering::Release);
    record_railcom_packet_metadata_from_isr(packet_sequence, metadata_raw, pom_request_id_raw);

    let packet_end_offset_us =
        CutoutTimeline::new(dcc_packet_duration_us).packet_end_from_packet_start_us();
    PENDING_PACKET_END_OFFSET_US.store(packet_end_offset_us, Ordering::Release);
    let cutout_start_deadline = packet_end_offset_us + CUTOUT_CONTROL_START_US;
    if schedule_cutout_deadline_fast(hw, cutout_start_deadline) {
        true
    } else {
        cutout_off_fast(hw);
        CUTOUT
            .cutout_schedule_fail_count
            .fetch_add(1, Ordering::Relaxed);
        store_cutout_state(CutoutState::Idle);
        false
    }
}

/// Returns a copy of the current cutout statistics.
#[must_use]
pub fn stats() -> TrackOutputStats {
    CUTOUT.snapshot()
}

#[must_use]
pub fn railcom_packet_metadata(packet_sequence: PacketSequence) -> Option<RailcomPacketMetadata> {
    let packet_sequence = packet_sequence.value();
    let slot = packet_sequence as usize % RAILCOM_PACKET_META_CAPACITY;
    let [recorded_sequence, metadata_raw, pom_request_id] =
        RAILCOM_PACKET_META_SLOTS[slot].snapshot()?;
    let metadata = PackedRailcomPacketMetadata::from_raw(metadata_raw);
    (recorded_sequence == packet_sequence).then(|| metadata.public_metadata(pom_request_id))
}

/// Drain cutout runtime events observed since `next_read`.
///
/// If the producer outruns the consumer beyond ring capacity, the oldest
/// events are dropped and counted in diagnostics.
///
/// The counters use wrapping arithmetic because overflowing `u32` is expected
/// during long-running operation. Per-slot generations reject torn snapshots.
pub fn drain_cutout_runtime_events(
    next_read: &mut u32,
    out: &mut Vec<CutoutRuntimeEvent, CUTOUT_EVENT_RING_CAPACITY>,
) {
    let write = CUTOUT_EVENT_WRITE_COUNT.load(Ordering::Acquire);
    let available = write.wrapping_sub(*next_read);
    if available > CUTOUT_EVENT_RING_CAPACITY as u32 {
        let dropped = available - CUTOUT_EVENT_RING_CAPACITY as u32;
        *next_read = write.wrapping_sub(CUTOUT_EVENT_RING_CAPACITY as u32);
        CUTOUT
            .cutout_event_dropped_count
            .fetch_add(dropped, Ordering::Relaxed);
    }

    let mut remaining = write
        .wrapping_sub(*next_read)
        .min(CUTOUT_EVENT_RING_CAPACITY as u32);
    while remaining > 0 && out.len() < CUTOUT_EVENT_RING_CAPACITY {
        let slot = *next_read as usize % CUTOUT_EVENT_RING_CAPACITY;
        let Some([sequence, metadata_raw, pom_request_id]) = CUTOUT_EVENT_SLOTS[slot].snapshot()
        else {
            CUTOUT
                .cutout_event_dropped_count
                .fetch_add(1, Ordering::Relaxed);
            *next_read = next_read.wrapping_add(1);
            remaining -= 1;
            continue;
        };
        let metadata = PackedRailcomPacketMetadata::from_raw(metadata_raw);
        let _ = out.push(metadata.runtime_event(PacketSequence::new(sequence), pom_request_id));
        *next_read = next_read.wrapping_add(1);
        remaining -= 1;
    }
}

#[cfg(target_arch = "riscv32")]
#[must_use]
pub fn cutout_event_notify_receiver() -> Receiver<'static, CriticalSectionRawMutex, u8, 1> {
    CUTOUT_EVENT_NOTIFY.receiver()
}

#[handler(priority = Priority::Priority3)]
#[ram]
fn cutout_timer_interrupt() {
    let Some(hw) = hw_mut() else {
        return;
    };

    clear_cutout_timer_interrupt_fast(hw);
    stop_cutout_timer_fast(hw);

    let raw_state = CUTOUT_STATE.load(Ordering::Acquire);
    match CutoutState::from_u8(raw_state) {
        Some(CutoutState::WaitingToOpen) => {
            let cutout_start_deadline =
                PENDING_PACKET_END_OFFSET_US.load(Ordering::Acquire) + CUTOUT_CONTROL_START_US;
            record_deadline_lateness(cutout_start_deadline);
            cutout_on_fast(hw);
            let capture_start_deadline =
                PENDING_PACKET_END_OFFSET_US.load(Ordering::Acquire) + RX_CAPTURE_START_US;
            if schedule_cutout_deadline_fast(hw, capture_start_deadline) {
                store_cutout_state(CutoutState::WaitingCh1Open);
            } else {
                cutout_off_fast(hw);
                CUTOUT
                    .cutout_schedule_fail_count
                    .fetch_add(1, Ordering::Relaxed);
                store_cutout_state(CutoutState::Idle);
            }
        }
        Some(CutoutState::WaitingCh1Open) => {
            let packet_sequence =
                PacketSequence::new(PENDING_CUTOUT_PACKET_SEQUENCE.load(Ordering::Acquire));
            let capture_start_deadline =
                PENDING_PACKET_END_OFFSET_US.load(Ordering::Acquire) + RX_CAPTURE_START_US;
            record_deadline_lateness(capture_start_deadline);
            let split_deadline =
                PENDING_PACKET_END_OFFSET_US.load(Ordering::Acquire) + RX_CHANNEL_SPLIT_US;
            if schedule_cutout_deadline_guarded_fast(hw, split_deadline) {
                store_cutout_state(CutoutState::Ch1Open);
                open_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel1);
                let pending_metadata = PackedRailcomPacketMetadata::from_raw(
                    PENDING_CUTOUT_METADATA.load(Ordering::Acquire),
                );
                push_cutout_event_from_isr(CutoutRuntimeEvent::Opened {
                    packet_sequence,
                    cutout: pending_metadata.cutout(),
                    target_address: pending_metadata.target_address(),
                    pom_request_id: pending_metadata
                        .pom_request_id(PENDING_CUTOUT_POM_REQUEST_ID.load(Ordering::Acquire)),
                });
                CUTOUT.cutout_started_count.fetch_add(1, Ordering::Relaxed);
            } else {
                cutout_off_fast(hw);
                CUTOUT
                    .cutout_schedule_fail_count
                    .fetch_add(1, Ordering::Relaxed);
                store_cutout_state(CutoutState::Idle);
            }
        }
        Some(CutoutState::Ch1Open) => {
            let packet_sequence =
                PacketSequence::new(PENDING_CUTOUT_PACKET_SEQUENCE.load(Ordering::Acquire));
            let split_deadline =
                PENDING_PACKET_END_OFFSET_US.load(Ordering::Acquire) + RX_CHANNEL_SPLIT_US;
            wait_until_cutout_offset(split_deadline);
            record_deadline_lateness(split_deadline);
            let channel2_end_deadline =
                PENDING_PACKET_END_OFFSET_US.load(Ordering::Acquire) + CHANNEL2_END_US;
            if schedule_cutout_deadline_guarded_fast(hw, channel2_end_deadline) {
                store_cutout_state(CutoutState::Ch2Open);
                close_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel1);
                open_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel2);
            } else {
                cutout_off_fast(hw);
                close_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel1);
                CUTOUT
                    .cutout_schedule_fail_count
                    .fetch_add(1, Ordering::Relaxed);
                store_cutout_state(CutoutState::Idle);
            }
        }
        Some(CutoutState::Ch2Open) => {
            let packet_sequence =
                PacketSequence::new(PENDING_CUTOUT_PACKET_SEQUENCE.load(Ordering::Acquire));
            let channel2_end_deadline =
                PENDING_PACKET_END_OFFSET_US.load(Ordering::Acquire) + CHANNEL2_END_US;
            wait_until_cutout_offset(channel2_end_deadline);
            record_deadline_lateness(channel2_end_deadline);
            close_realtime_window_from_isr(packet_sequence, RailcomChannel::Channel2);
            cutout_off_fast(hw);
            CUTOUT.cutout_ended_count.fetch_add(1, Ordering::Relaxed);
            store_cutout_state(CutoutState::Idle);
        }
        Some(CutoutState::Idle) | None => {
            if CutoutState::from_u8(raw_state).is_none() {
                CUTOUT
                    .cutout_invalid_state_count
                    .fetch_add(1, Ordering::Relaxed);
            }
            cutout_off_fast(hw);
            store_cutout_state(CutoutState::Idle);
        }
    }
}
