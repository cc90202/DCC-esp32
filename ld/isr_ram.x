/*
 * Library code and data that the DCC interrupt handlers use on every pass,
 * moved from the cached flash mapping into internal RAM.
 *
 * The handlers themselves are #[ram], but the functions below come from
 * esp-hal, esp-sync, esp-rtos, embassy-sync and embassy-executor, which offer
 * no option to place them in RAM. Left in flash, each call would stall on a
 * cache miss, and the handler latency would change from build to build.
 *
 * This file must come before linkall.x on the linker command line (see
 * .cargo/config.toml): an input section belongs to the first pattern that
 * names it, and esp-hal's .text and .rodata rules match everything.
 *
 * The patterns match Rust v0-mangled section names (".text.<symbol>"). They
 * use the length-prefixed identifiers and skip crate hashes, so they survive
 * rebuilds and toolchain updates. A dependency update that renames one of
 * these functions leaves it in flash; scripts/check-isr-ram.sh then fails and
 * names it, and the pattern here must follow the new name.
 */

SECTIONS {
  .rwtext.isr : ALIGN(4)
  {
    /* esp-hal's interrupt dispatcher (handle_interrupts, already in RAM)
       calls these before and between the handlers on every interrupt. */
    *(.text.*9interrupt5riscv4plic16current_runlevel)
    *(.text.*23InterruptStatusIterator*8Iterator4next)
    /* Channel<_, u8, 1>::try_send: the cutout event notification. */
    *(.text.*23CriticalSectionRawMutex*8RawMutex4lock*12TrySendErrorhE*8try_send0*)
    /* Signal<_, ()>::signal: the packet-consumed notification. */
    *(.text.*23CriticalSectionRawMutex*8RawMutex4lock*6signal5StateuE*6signal0*)
    /* Cutout timer rescheduling. */
    *(.text.*15GenericRawMutex*23SingleCoreInterruptLock*4lock*5Alarm10set_enable0*)
    *(.text.*12OneShotTimer*8Blocking*8schedule*)
    *(.text.*7esp_hal5clock*6Clocks3get)
    *(.text._critical_section_1_0_acquire)
    /* Task wake-up behind both notifications: the embassy waker and the
       esp-rtos scheduler functions __pender reaches. */
    *(.text.*16embassy_executor3raw5waker4wake)
    *(.text.*8esp_sync*9LockGuard*23SingleCoreInterruptLock*17new_non_reentrant*)
    *(.text.*8esp_rtos5timer*10TimerQueue6remove)
    *(.text.*8esp_rtos10wait_queue*9WaitQueue6remove)
    . = ALIGN(4);
  } > RWTEXT
}
INSERT AFTER .rwtext;

SECTIONS {
  .data.isr : ALIGN(4)
  {
    /* Jump tables of the two DCC handlers. esp-hal already keeps the
       timer code's switch tables in RAM (place-switch-tables-in-ram). */
    *(.rodata.*7rmt_dcc32___esp_hal_internal_rmt_interrupt)
    *(.rodata.*12track_output41___esp_hal_internal_cutout_timer_interrupt)
    /* Read through the waker on every wake-up. */
    *(.rodata.*16embassy_executor3raw5waker6VTABLE)
    . = ALIGN(4);
  } > RWDATA
}
INSERT AFTER .rwtext.wifi;
