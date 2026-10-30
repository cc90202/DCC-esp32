//! Chip restart that resets the Wi-Fi modem first, as ESP-IDF does.
//!
//! `esp_hal::system::software_reset` jumps straight to the ROM reset and
//! leaves the modem subsystem as it was. ESP-IDF's `esp_restart_noos` for the
//! ESP32-C6 pulses the modem resets before resetting the CPU
//! (`esp_system_reset_modules_on_exit` in
//! `components/esp_system/port/soc/esp32c6/system_internal.c`, v5.5.1).
//! On 2026-10-10 an OTA reset taken while the station was transmitting left
//! the board associated, with a DHCP address, yet unreachable through every
//! later reset until a power cycle.

use esp_hal::peripherals::{MODEM_LPCON, MODEM_SYSCON, PCR};

/// Every bit of `MODEM_SYSCON.MODEM_RST_CONF`, as `modem_syscon_ll_reset_all`.
const MODEM_SYSCON_RESET_ALL: u32 = u32::MAX;
/// The four reset bits of `MODEM_LPCON.RST_CONF`, as `modem_lpcon_ll_reset_all`.
const MODEM_LPCON_RESET_ALL: u32 = 0xf;

/// Resets the modem subsystem, then the chip; never returns.
///
/// Use it for every intentional restart: the Wi-Fi radio may be mid-frame
/// when it is called.
pub fn restart() -> ! {
    // As in esp_restart_noos, no interrupt may reach the radio driver while
    // its blocks are held in reset. The section is never released.
    // SAFETY: the matching release is never called; the chip resets below.
    let _ = unsafe { critical_section::acquire() };

    // SAFETY: raw values copied from the ESP-IDF LL functions named above;
    // every bit of these registers is a reset request for a modem block.
    MODEM_SYSCON::regs()
        .modem_rst_conf()
        .write(|w| unsafe { w.bits(MODEM_SYSCON_RESET_ALL) });
    // SAFETY: 0 releases every modem block reset requested above.
    MODEM_SYSCON::regs()
        .modem_rst_conf()
        .write(|w| unsafe { w.bits(0) });
    // SAFETY: as for MODEM_SYSCON, every bit is a modem block reset request.
    MODEM_LPCON::regs()
        .rst_conf()
        .write(|w| unsafe { w.bits(MODEM_LPCON_RESET_ALL) });
    // SAFETY: 0 releases every modem block reset requested above.
    MODEM_LPCON::regs()
        .rst_conf()
        .write(|w| unsafe { w.bits(0) });

    PCR::regs()
        .modem_apb_conf()
        .modify(|_, w| w.modem_rst_en().set_bit());
    PCR::regs()
        .modem_apb_conf()
        .modify(|_, w| w.modem_rst_en().clear_bit());

    esp_hal::system::software_reset()
}
