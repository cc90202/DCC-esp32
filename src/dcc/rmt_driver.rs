//! Low-level ISR-driven RMT backend for gap-free DCC transmission.
//!
//! This module exists because the current `esp-hal` API does not provide a way
//! to update a continuously looping TX buffer without blocking the executor or
//! burning CPU in a software feed loop. To keep the DCC waveform continuous
//! while leaving Embassy free to run networking and safety tasks, this backend:
//!
//! - starts RMT channel 0 in continuous loop mode with a prebuilt idle packet,
//! - publishes task-built packet snapshots through two alternating static slots,
//! - patches only the variable packet tail from the ISR on each loop boundary,
//! - keeps the fixed preamble in place inside RMT RAM.
//!
//! The cost of this approach is that it depends on ESP32-C6 RMT RAM layout and
//! register semantics from the TRM. Treat this module as hardware-coupled code:
//! design changes elsewhere should not bypass its invariants.

use core::cell::UnsafeCell;
use core::mem::{ManuallyDrop, size_of};
use core::sync::atomic::{AtomicPtr, AtomicU8, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use esp_hal::Blocking;
use esp_hal::gpio::Level;
use esp_hal::handler;
use esp_hal::interrupt::{self, Priority};
use esp_hal::peripherals::{Interrupt, RMT};
use esp_hal::ram;
use esp_hal::rmt::{Channel as RmtChannel, ContinuousTxTransaction, LoopMode, PulseCode, Tx};
use esp_hal::time::Instant;
use static_cell::StaticCell;

use crate::cutout::timing::CUTOUT_CONTROL_END_US;
use crate::cutout::{CutoutMode, PacketSequence};
use crate::dcc::timing::{MAX_DATA_PULSES, PREAMBLE_DURATION_US, PREAMBLE_RMT_OFFSET};
use crate::dcc::{DccAddress, PomRequestId};

// ESP32-C6 RMT peripheral RAM base address (from TRM §29.5, APB address space).
// This constant is only valid for ESP32-C6; the crate enables this backend only
// on the riscv32 ESP target and selects the `esp32c6` HAL feature in Cargo.toml,
// so there is no supported runtime where this address should point elsewhere.
//
// The backend programs channel 0 and assumes the continuous TX instance owns
// exactly that channel for its full lifetime. The raw RAM writes below therefore
// target the first channel window starting at this peripheral base.
//
// Each channel memory block holds 48 PulseCode entries (48 × 4 bytes = 192 bytes).
// Channel 0 uses three contiguous blocks for DCC.
const RMT_RAM_START: usize = 0x6000_6400;
const RMT_CHANNEL_RAM_SIZE: usize = 48;
const RMT_CHANNEL_INDEX: usize = 0;
const RMT_CHANNEL_MEM_BLOCKS: usize = 3;
const RMT_CHANNEL_TOTAL_RAM_SIZE: usize = RMT_CHANNEL_RAM_SIZE * RMT_CHANNEL_MEM_BLOCKS;
const CUTOUT_PADDING_US: u32 = CUTOUT_CONTROL_END_US;
const PACKET_SLOT_COUNT: usize = 2;
const PACKET_SLOT_A: u8 = 0;
const PACKET_SLOT_B: u8 = 1;
const NO_PACKET_SLOT: u8 = PACKET_SLOT_COUNT as u8;
const _: () = assert!(
    PREAMBLE_RMT_OFFSET < RMT_CHANNEL_TOTAL_RAM_SIZE,
    "preamble offset must fit inside channel RAM"
);
const _: () = assert!(
    PREAMBLE_RMT_OFFSET + MAX_DATA_PULSES < RMT_CHANNEL_TOTAL_RAM_SIZE,
    "packet tail plus end marker must fit inside the configured RMT RAM window"
);

#[derive(Clone, Copy)]
struct PacketMeta {
    cutout: CutoutMode,
    target_address: Option<DccAddress>,
    pom_request_id: u32,
}

impl PacketMeta {
    const fn idle() -> Self {
        Self {
            cutout: CutoutMode::None,
            target_address: None,
            pom_request_id: 0,
        }
    }
}

struct SharedPacket {
    data: [PulseCode; MAX_DATA_PULSES],
    len: u8,
    meta: PacketMeta,
    dcc_duration_us: u32,
}

struct PacketSlot(UnsafeCell<SharedPacket>);

impl PacketSlot {
    const fn new() -> Self {
        Self(UnsafeCell::new(SharedPacket::empty()))
    }
}

// SAFETY: PacketSlot is accessed only by the single-core ESP32-C6 task/ISR
// publication protocol below. The task mutates only the unpublished slot; a
// Release store publishes it, and the ISR reads that slot after an Acquire load.
unsafe impl Sync for PacketSlot {}

impl SharedPacket {
    const fn empty() -> Self {
        Self {
            data: [PulseCode::end_marker(); MAX_DATA_PULSES],
            len: 0,
            meta: PacketMeta::idle(),
            dcc_duration_us: PREAMBLE_DURATION_US,
        }
    }
}

const _: () = assert!(
    size_of::<SharedPacket>() <= 404,
    "shared packet slots must remain within their static memory budget"
);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum InitError {
    InvalidIdleWaveform,
    StartContinuousTx,
    InterruptEnable,
}

/// Timing observed inside the RMT packet-boundary interrupt.
///
/// These are diagnostic maxima in microseconds. They describe software
/// service time, not the hardware-generated DCC or GPIO4 waveform.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub(crate) struct RmtTimingStats {
    pub max_isr_duration_us: u32,
    pub max_cutout_request_latency_us: u32,
}

static PACKET_SLOTS: [PacketSlot; PACKET_SLOT_COUNT] = [PacketSlot::new(), PacketSlot::new()];
static PUBLISHED_PACKET_SLOT: AtomicU8 = AtomicU8::new(NO_PACKET_SLOT);
static PACKET_CONSUMED_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static ISR_HEARTBEAT: AtomicU32 = AtomicU32::new(0);
static RMT_ISR_MAX_DURATION_US: AtomicU32 = AtomicU32::new(0);
static RMT_CUTOUT_REQUEST_MAX_LATENCY_US: AtomicU32 = AtomicU32::new(0);
// Read-only after init, so no Mutex is needed, ISR reads directly via raw pointer.
static IDLE_DATA_CELL: StaticCell<SharedPacket> = StaticCell::new();
static IDLE_DATA_PTR: AtomicPtr<SharedPacket> = AtomicPtr::new(core::ptr::null_mut());
/// Keeps the ContinuousTxTransaction alive, preventing esp-hal's Drop from stopping TX.
static RMT_TX_KEEPALIVE: StaticCell<ManuallyDrop<ContinuousTxTransaction<'static>>> =
    StaticCell::new();

#[must_use]
pub(crate) fn timing_stats() -> RmtTimingStats {
    RmtTimingStats {
        max_isr_duration_us: RMT_ISR_MAX_DURATION_US.load(Ordering::Acquire),
        max_cutout_request_latency_us: RMT_CUTOUT_REQUEST_MAX_LATENCY_US.load(Ordering::Acquire),
    }
}

/// Start continuous DCC transmission via one RMT hardware loop.
///
/// Writes the fixed preamble and initial idle waveform to RMT RAM, starts the
/// continuous TX channel, and registers the loop-boundary ISR at Priority3.
///
/// # Errors
///
/// Returns [`InitError`] if the idle waveform is malformed, RMT TX fails
/// to start, or interrupt registration fails.
pub fn init(
    channel: RmtChannel<'static, Blocking, Tx>,
    idle_rmt: &[PulseCode],
) -> Result<(), InitError> {
    let idle_data = build_idle_data(idle_rmt)?;
    let idle_ref = IDLE_DATA_CELL.init(idle_data);
    IDLE_DATA_PTR.store(idle_ref as *const _ as *mut _, Ordering::Release);

    // SAFETY: the RMT interrupt is not bound or enabled yet, so no reader can
    // access either packet slot while initialization resets them.
    for slot in &PACKET_SLOTS {
        unsafe { slot.0.get().write(SharedPacket::empty()) };
    }
    PUBLISHED_PACKET_SLOT.store(NO_PACKET_SLOT, Ordering::Release);
    PACKET_CONSUMED_SIGNAL.reset();
    ISR_HEARTBEAT.store(0, Ordering::Relaxed);
    RMT_ISR_MAX_DURATION_US.store(0, Ordering::Relaxed);
    RMT_CUTOUT_REQUEST_MAX_LATENCY_US.store(0, Ordering::Relaxed);

    let tx = channel
        .transmit_continuously(idle_rmt, LoopMode::InfiniteWithInterrupt(1))
        .map_err(|error| {
            defmt::error!("RMT continuous transmission start failed: {:?}", error);
            InitError::StartContinuousTx
        })?;
    let _ = RMT_TX_KEEPALIVE.init(ManuallyDrop::new(tx));

    clear_tx_loop_interrupt();
    enable_tx_loop_interrupt();

    // SAFETY: We own the RMT peripheral (channel consumed above) and no other
    // handler is registered for Interrupt::RMT in Blocking mode.
    unsafe { interrupt::bind_interrupt(Interrupt::RMT, rmt_interrupt.handler()) };
    interrupt::enable(Interrupt::RMT, Priority::Priority3).map_err(|error| {
        defmt::error!("RMT interrupt enable failed: {:?}", error);
        InitError::InterruptEnable
    })?;

    Ok(())
}

/// Queue a pre-encoded data portion for the ISR to write on the next loop boundary.
///
/// Non-blocking. The ISR picks up the data within one packet cycle (~6ms).
/// If called again before the ISR consumes the previous submission, the new
/// data overwrites the old; use [`is_consumed`] to enforce ACK-based pacing.
/// This internal entry point is owned by the normal-context DCC feeder task; it
/// must not be called from an interrupt handler.
pub fn submit_packet(
    data: &[PulseCode],
    dcc_duration_us: u32,
    cutout: CutoutMode,
    target_address: Option<DccAddress>,
    pom_request_id: Option<PomRequestId>,
) {
    debug_assert!(
        data.len() <= MAX_DATA_PULSES,
        "packet tail exceeds MAX_DATA_PULSES"
    );
    debug_assert!(
        dcc_duration_us >= PREAMBLE_DURATION_US,
        "packet duration must include the fixed preamble"
    );
    let len = data.len().min(MAX_DATA_PULSES);
    let data = &data[..len];

    let published_slot = PUBLISHED_PACKET_SLOT.load(Ordering::Acquire);
    let write_slot = if published_slot == PACKET_SLOT_A {
        PACKET_SLOT_B
    } else {
        PACKET_SLOT_A
    };

    // SAFETY: submit_packet runs in normal task context on the single-core
    // ESP32-C6. It writes only the slot that is not currently published. The
    // ISR can preempt this code, but it will either read the previously
    // published slot or idle data; it cannot observe this slot until the
    // Release store below. Normal tasks cannot run while the ISR is reading.
    unsafe {
        let packet = &mut *PACKET_SLOTS[write_slot as usize].0.get();
        packet.len = len as u8;
        packet.data[..len].copy_from_slice(data);
        packet.meta = PacketMeta {
            cutout,
            target_address,
            pom_request_id: pom_request_id.map_or(0, PomRequestId::value),
        };
        packet.dcc_duration_us = dcc_duration_us;
    }

    PUBLISHED_PACKET_SLOT.store(write_slot, Ordering::Release);
}

/// Returns `true` when the ISR has consumed the last submitted packet.
#[inline(always)]
pub fn is_consumed() -> bool {
    PUBLISHED_PACKET_SLOT.load(Ordering::Acquire) == NO_PACKET_SLOT
}

/// Wait until the ISR reports that a published packet was consumed.
///
/// The atomic slot state remains authoritative; callers must re-check
/// [`is_consumed`] after waking because signals are intentionally coalesced.
pub async fn wait_for_packet_consumed() {
    PACKET_CONSUMED_SIGNAL.wait().await;
}

/// Monotonic counter incremented by the ISR on each loop boundary.
///
/// Used by the feeder task as a watchdog: if this value stops advancing,
/// the ISR is no longer running and a fault should be latched.
#[inline(always)]
pub fn isr_heartbeat() -> u32 {
    ISR_HEARTBEAT.load(Ordering::Relaxed)
}

fn build_idle_data(idle_rmt: &[PulseCode]) -> Result<SharedPacket, InitError> {
    if idle_rmt.len() <= PREAMBLE_RMT_OFFSET || idle_rmt.last() != Some(&PulseCode::end_marker()) {
        return Err(InitError::InvalidIdleWaveform);
    }
    // Strip preamble (offset 0..19) and the trailing end_marker (last entry).
    // The ISR writes its own end_marker at the correct position for each packet.
    let tail = &idle_rmt[PREAMBLE_RMT_OFFSET..idle_rmt.len() - 1];
    if tail.len() > MAX_DATA_PULSES {
        return Err(InitError::InvalidIdleWaveform);
    }

    let mut packet = SharedPacket::empty();
    packet.len = tail.len() as u8;
    packet.data[..tail.len()].copy_from_slice(tail);
    packet.dcc_duration_us = packet_duration_us(tail);
    Ok(packet)
}

#[handler(priority = Priority::Priority3)]
#[ram]
fn rmt_interrupt() {
    let regs = RMT::regs();
    let st = regs.int_st().read();

    if !st.ch_tx_loop(RMT_CHANNEL_INDEX as u8).bit() {
        return;
    }

    // Capture the hardware packet boundary before counters, shared-state access
    // or RMT RAM writes can add path-dependent latency to the RailCom timeline.
    let packet_boundary_us = now_us();
    clear_tx_loop_interrupt();
    reset_tx_loop_counter();
    let packet_sequence = PacketSequence::new(
        ISR_HEARTBEAT
            .fetch_add(1, Ordering::Relaxed)
            .wrapping_add(1),
    );
    // `on_dcc_packet_boundary()` is observation-only telemetry. It intentionally
    // runs before the cutout authorization check so packet-boundary health can be
    // counted on every loop. Do not add hardware side effects here: real cutout
    // execution must remain gated by the current packet metadata below.
    crate::railcom::on_dcc_packet_boundary();
    let published_slot = PUBLISHED_PACKET_SLOT.load(Ordering::Acquire);
    if published_slot < NO_PACKET_SLOT {
        // SAFETY: the Acquire load above observes a fully initialized slot. On
        // the single-core ESP32-C6 the publishing task cannot run while this ISR
        // holds the reference, and future submissions write the other slot.
        let pkt = unsafe { &*PACKET_SLOTS[published_slot as usize].0.get() };
        debug_assert!(pkt.len as usize <= MAX_DATA_PULSES);
        let data = &pkt.data[..pkt.len as usize];
        let cutout_requested = !matches!(pkt.meta.cutout, CutoutMode::None);
        let cutout_allowed = if cutout_requested {
            RMT_CUTOUT_REQUEST_MAX_LATENCY_US
                .fetch_max(now_us().wrapping_sub(packet_boundary_us), Ordering::Relaxed);
            crate::track_output::request_cutout_from_isr(
                packet_boundary_us,
                packet_sequence,
                pkt.dcc_duration_us,
                pkt.meta.cutout,
                pkt.meta.target_address,
                matches!(pkt.meta.cutout, CutoutMode::PomWrite | CutoutMode::PomRead)
                    .then(|| PomRequestId::new(pkt.meta.pom_request_id)),
            )
        } else {
            false
        };

        write_data_to_rmt_ram(data, cutout_allowed);
    } else {
        // IDLE_DATA is read-only after init, so no critical section is needed.
        let idle_ptr = IDLE_DATA_PTR.load(Ordering::Relaxed);
        if !idle_ptr.is_null() {
            // SAFETY: IDLE_DATA_PTR is set once in init() from a StaticCell reference
            // and never modified again. The pointee is read-only after init,
            // has static lifetime, and remains valid for the whole program.
            let idle = unsafe { &*idle_ptr };
            write_data_to_rmt_ram(&idle.data[..idle.len as usize], false);
        }
    }

    if published_slot < NO_PACKET_SLOT {
        PUBLISHED_PACKET_SLOT.store(NO_PACKET_SLOT, Ordering::Release);
        PACKET_CONSUMED_SIGNAL.signal(());
    }

    RMT_ISR_MAX_DURATION_US.fetch_max(now_us().wrapping_sub(packet_boundary_us), Ordering::Relaxed);
}

#[inline(always)]
fn now_us() -> u32 {
    Instant::now().duration_since_epoch().as_micros() as u32
}

#[inline(always)]
fn clear_tx_loop_interrupt() {
    let regs = RMT::regs();
    regs.int_clr()
        .write(|w| w.ch_tx_loop(RMT_CHANNEL_INDEX as u8).set_bit());
}

#[inline(always)]
fn enable_tx_loop_interrupt() {
    let regs = RMT::regs();
    regs.int_ena()
        .modify(|_, w| w.ch_tx_loop(RMT_CHANNEL_INDEX as u8).set_bit());
}

#[inline(always)]
fn reset_tx_loop_counter() {
    let regs = RMT::regs();
    regs.ch_tx_lim(RMT_CHANNEL_INDEX)
        .modify(|_, w| w.loop_count_reset().set_bit());
}

/// Write only the data entries + end_marker to RMT RAM at the data offset.
#[ram]
fn write_data_to_rmt_ram(data: &[PulseCode], cutout_allowed: bool) {
    debug_assert!(
        data.len() <= MAX_DATA_PULSES,
        "RMT tail write must fit inside reserved data area"
    );

    // The first PREAMBLE_RMT_OFFSET entries in channel RAM are left untouched:
    // they contain the fixed "1" preamble that was loaded when continuous TX
    // started from the idle waveform. We patch only the variable tail after that
    // fixed preamble so every loop keeps a gap-free packet prefix in hardware.
    //
    // With memsize = 3, channel 0 owns 144 entries total. Starting at
    // PREAMBLE_RMT_OFFSET reserves 20 entries for the preamble, leaving 124.
    // The compile-time assertions above guarantee that MAX_DATA_PULSES plus the
    // trailing end_marker still fit inside this remaining window.
    let base = (RMT_RAM_START as *mut PulseCode)
        .wrapping_add(RMT_CHANNEL_INDEX * RMT_CHANNEL_RAM_SIZE + PREAMBLE_RMT_OFFSET);

    for (idx, &pulse) in data.iter().enumerate() {
        // SAFETY: base points to RMT channel 0 RAM at the data offset.
        // Compile-time assertions guarantee idx < MAX_DATA_PULSES < channel RAM size.
        // The hardware is reading the preamble region (~2.3ms) so no read/write race.
        unsafe {
            base.add(idx).write_volatile(pulse);
        }
    }

    let end_offset = data.len() + usize::from(cutout_allowed);
    if cutout_allowed {
        // Keep the DCC RMT output low until the GPIO4 control pulse rises at
        // the RCN-217 channel-2 end. The next loop then starts a fresh
        // preamble. The external RC logic keeps the DRV8874 disabled for a few
        // more microseconds, so only the first preamble pulse can be partial on
        // the physical track; the remaining 19 preamble bits stay intact.
        unsafe {
            base.add(data.len()).write_volatile(PulseCode::new(
                Level::Low,
                (CUTOUT_PADDING_US - 1) as u16,
                Level::Low,
                1,
            ));
        }
    }

    // SAFETY: same as above; optional padding still fits in the channel RAM.
    unsafe {
        base.add(end_offset).write_volatile(PulseCode::end_marker());
    }
}

#[inline(always)]
fn packet_duration_us(data: &[PulseCode]) -> u32 {
    PREAMBLE_DURATION_US
        + data
            .iter()
            .map(|pulse| u32::from(pulse.length1()) + u32::from(pulse.length2()))
            .sum::<u32>()
}
