//! Low-level ISR-driven RMT backend for gap-free DCC transmission.
//!
//! This module exists because the current `esp-hal` API does not provide a way
//! to update a continuously looping TX buffer without blocking the executor or
//! burning CPU in a software feed loop. To keep the DCC waveform continuous
//! while leaving Embassy free to run networking and safety tasks, this backend:
//!
//! - starts RMT in continuous loop mode with a prebuilt idle packet,
//! - patches only the variable packet tail from the ISR on each loop boundary,
//! - keeps the fixed preamble in place inside RMT RAM.
//!
//! The cost of this approach is that it depends on ESP32-C6 RMT RAM layout and
//! register semantics from the TRM. Treat this module as hardware-coupled code:
//! design changes elsewhere should not bypass its invariants.

use core::cell::RefCell;
use core::mem::ManuallyDrop;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering};

use critical_section::Mutex;
use esp_hal::Blocking;
use esp_hal::handler;
use esp_hal::interrupt::{self, Priority};
use esp_hal::peripherals::{Interrupt, RMT};
use esp_hal::ram;
use esp_hal::rmt::{Channel as RmtChannel, ContinuousTxTransaction, LoopMode, PulseCode, Tx};
use static_cell::StaticCell;

use crate::dcc::timing::{MAX_DATA_PULSES, PREAMBLE_RMT_OFFSET};

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
// The channel is configured with memsize = 2 blocks by the RMT setup used for
// continuous transmission, so channel 0 has 96 writable entries available.
const RMT_RAM_START: usize = 0x6000_6400;
const RMT_CHANNEL_RAM_SIZE: usize = 48;
const RMT_CHANNEL_INDEX: usize = 0;
const RMT_CHANNEL_MEM_BLOCKS: usize = 2;
const RMT_CHANNEL_TOTAL_RAM_SIZE: usize = RMT_CHANNEL_RAM_SIZE * RMT_CHANNEL_MEM_BLOCKS;

const _: () = assert!(
    PREAMBLE_RMT_OFFSET < RMT_CHANNEL_TOTAL_RAM_SIZE,
    "preamble offset must fit inside channel RAM"
);
const _: () = assert!(
    PREAMBLE_RMT_OFFSET + MAX_DATA_PULSES < RMT_CHANNEL_TOTAL_RAM_SIZE,
    "packet tail plus end marker must fit inside the configured RMT RAM window"
);

#[derive(Clone, Copy)]
struct SharedPacket {
    data: [PulseCode; MAX_DATA_PULSES],
    len: u8,
}

impl SharedPacket {
    const fn empty() -> Self {
        Self {
            data: [PulseCode::end_marker(); MAX_DATA_PULSES],
            len: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(target_arch = "riscv32", derive(defmt::Format))]
pub enum InitError {
    InvalidIdleWaveform,
    StartContinuousTx,
    InterruptEnable,
}

static DATA_READY: AtomicBool = AtomicBool::new(false);
static ISR_HEARTBEAT: AtomicU32 = AtomicU32::new(0);
static NEXT_PACKET: Mutex<RefCell<SharedPacket>> = Mutex::new(RefCell::new(SharedPacket::empty()));
// Read-only after init — no Mutex needed, ISR reads directly via raw pointer.
static IDLE_DATA_CELL: StaticCell<SharedPacket> = StaticCell::new();
static IDLE_DATA_PTR: AtomicPtr<SharedPacket> = AtomicPtr::new(core::ptr::null_mut());
/// Keeps the ContinuousTxTransaction alive, preventing esp-hal's Drop from stopping TX.
static RMT_TX_KEEPALIVE: StaticCell<ManuallyDrop<ContinuousTxTransaction<'static>>> =
    StaticCell::new();

/// Start continuous DCC transmission via RMT hardware loop and ISR.
///
/// Writes the fixed preamble and initial idle waveform to RMT RAM,
/// starts continuous TX, and registers the loop-boundary ISR at Priority2.
/// The channel is consumed and kept alive for the lifetime of the program.
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

    DATA_READY.store(false, Ordering::Release);
    ISR_HEARTBEAT.store(0, Ordering::Relaxed);

    let tx = channel
        .transmit_continuously(idle_rmt, LoopMode::InfiniteWithInterrupt(1))
        .map_err(|_| InitError::StartContinuousTx)?;
    let _ = RMT_TX_KEEPALIVE.init(ManuallyDrop::new(tx));

    clear_tx_loop_interrupt();
    enable_tx_loop_interrupt();

    // SAFETY: We own the RMT peripheral (channel consumed above) and no other
    // handler is registered for Interrupt::RMT in Blocking mode.
    unsafe { interrupt::bind_interrupt(Interrupt::RMT, rmt_interrupt.handler()) };
    interrupt::enable(Interrupt::RMT, Priority::Priority2)
        .map_err(|_| InitError::InterruptEnable)?;

    Ok(())
}

/// Queue a pre-encoded data portion for the ISR to write on the next loop boundary.
///
/// Non-blocking. The ISR picks up the data within one packet cycle (~6ms).
/// If called again before the ISR consumes the previous submission, the new
/// data overwrites the old — use [`is_consumed`] to enforce ACK-based pacing.
pub fn submit_packet(data: &[PulseCode]) {
    debug_assert!(
        data.len() <= MAX_DATA_PULSES,
        "packet tail exceeds MAX_DATA_PULSES"
    );
    critical_section::with(|cs| {
        let mut packet = NEXT_PACKET.borrow_ref_mut(cs);
        let len = data.len().min(MAX_DATA_PULSES);
        packet.len = len as u8;
        packet.data[..len].copy_from_slice(&data[..len]);
    });

    DATA_READY.store(true, Ordering::Release);
}

/// Returns `true` when the ISR has consumed the last submitted packet.
#[inline(always)]
pub fn is_consumed() -> bool {
    !DATA_READY.load(Ordering::Acquire)
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
    Ok(packet)
}

#[handler(priority = Priority::Priority2)]
#[ram]
fn rmt_interrupt() {
    let regs = RMT::regs();
    let st = regs.int_st().read();

    if !st.ch_tx_loop(RMT_CHANNEL_INDEX as u8).bit() {
        return;
    }

    clear_tx_loop_interrupt();
    reset_tx_loop_counter();
    ISR_HEARTBEAT.fetch_add(1, Ordering::Relaxed);

    if DATA_READY.load(Ordering::Acquire) {
        critical_section::with(|cs| {
            let pkt = NEXT_PACKET.borrow_ref(cs);
            debug_assert!(pkt.len as usize <= MAX_DATA_PULSES);
            write_data_to_rmt_ram(&pkt.data[..pkt.len as usize]);
        });
        DATA_READY.store(false, Ordering::Release);
    } else {
        // IDLE_DATA is read-only after init — no critical section needed.
        let idle_ptr = IDLE_DATA_PTR.load(Ordering::Relaxed);
        if !idle_ptr.is_null() {
            // SAFETY: IDLE_DATA_PTR is set once in init() from a StaticCell reference
            // and never modified again. The pointee is read-only after init,
            // has static lifetime, and remains valid for the whole program.
            let idle = unsafe { &*idle_ptr };
            write_data_to_rmt_ram(&idle.data[..idle.len as usize]);
        }
    }
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
#[inline(always)]
fn write_data_to_rmt_ram(data: &[PulseCode]) {
    debug_assert!(
        data.len() <= MAX_DATA_PULSES,
        "RMT tail write must fit inside reserved data area"
    );

    // The first PREAMBLE_RMT_OFFSET entries in channel RAM are left untouched:
    // they contain the fixed "1" preamble that was loaded when continuous TX
    // started from the idle waveform. We patch only the variable tail after that
    // fixed preamble so every loop keeps a gap-free packet prefix in hardware.
    //
    // With memsize = 2, channel 0 owns 96 entries total. Starting at
    // PREAMBLE_RMT_OFFSET reserves 20 entries for the preamble, leaving 76.
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

    // SAFETY: same as above; data.len() <= MAX_DATA_PULSES, so offset is in bounds.
    unsafe {
        base.add(data.len()).write_volatile(PulseCode::end_marker());
    }
}
