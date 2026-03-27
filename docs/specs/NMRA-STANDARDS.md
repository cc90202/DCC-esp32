# NMRA DCC Standards Reference

Reference documentation for DCC implementation compliance.

## Core Standards (Required for MVP)

### S-9.1: DCC Electrical Standard
**Status:** ✅ Implemented in `src/dcc/timing.rs` and verified on hardware
**URL:** https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.1_electrical_standards_for_digital_command_control.pdf
**Updated:** April 30, 2025

**Key Requirements:**
- Bit timing: "1" = 58μs ±3μs, "0" = ≥100μs nominal
- Voltage levels: Track voltage 12-22V (we output 3.3V logic, needs H-bridge)
- Preamble: 14+ "1" bits minimum

**Implementation:**
```rust
// src/dcc/timing.rs
pub const DCC_ONE_HIGH_US: u16 = 58;   // 55-61μs acceptable
pub const DCC_ONE_LOW_US: u16 = 58;
pub const DCC_ZERO_HIGH_US: u16 = 100; // 95-9900μs acceptable
pub const DCC_ZERO_LOW_US: u16 = 100;
pub const PREAMBLE_BITS: usize = 20;   // measured capture showed stable 22x "1" preamble
```

**Validator:** `DccValidator::validate_timing()` checks pulse durations

**Hardware evidence:**
- Oscilloscope/logic-analyzer capture confirms valid DCC waveform on real output
- Measured packet decode showed stable preamble (`22x "1"`), valid start/end bits, address `0x03`, command `0x90`, checksum `0x93`
- This closes the previous timing-only gap between software claims and physical signal generation

---

### S-9.2: DCC Communications Standard
**Status:** ✅ Implemented in `src/dcc/packet.rs`
**URL:** https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9_2-2004-07.pdf
**Updated:** July 2004

**Key Requirements:**
- Packet format: [Preamble] 0 [Address] 0 [Instruction] 0 [Error Detection] 1
- Address formats: Short (1-127), Long (128-10239)
- Speed steps: 14, 28, 128
- Direction control in instruction byte
- Checksum: XOR of all data bytes

**Implementation:**
```rust
// src/dcc/packet.rs
pub enum DccPacket {
    Idle,                    // 0xFF 0x00 0xFF
    Reset,                   // 0x00 0x00 0x00
    Speed28 { .. },          // 28-step interleaved encoding (01DCSSSS)
    Speed128 { .. },         // 128-step (0x3F + DSSSSSSS)
    FunctionGroup1 { .. },   // FL, F1-F4 (100DDDDD)
    FunctionGroup2A { .. },  // F5-F8 (1011DDDD)
    FunctionGroup2B { .. },  // F9-F12 (1010DDDD)
    EmergencyStop { .. },    // Per-decoder e-stop (01DC0001)
    BroadcastStop,           // All-decoder e-stop (addr 0x00)
}
```

**Speed28 Interleaved Encoding (Section 2.3.2.3):**
```rust
// CRITICAL: Must use interleaved format per S-9.2
let speed_bits = match speed {
    0 => 0b0000,      // Stop
    1 => 0b0001,      // Emergency stop
    2..=29 => {
        let s = speed - 1;
        ((s >> 1) & 0x0F) | ((s & 1) << 4)  // Interleaved
    }
    _ => 0b0000,
};
```

**Validator:** `DccValidator::validate_nmra_compliance()` checks packet structure

---

## Extended Standards (Phase 2+)

### S-9.2.1: DCC Extended Packet Formats
**Status:** ✅ Partially implemented (128 speed steps)
**URL:** https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.2.1_dcc_extended_packet_formats.pdf
**Updated:** January 24, 2025

**Features:**
- ✅ 128 speed steps (implemented in `DccPacket::Speed128`)
- Advanced consisting (not yet implemented)
- Decoder lock (not yet implemented)
- Firmware upload (not yet implemented)

---

### S-9.2.2: DCC Configuration Variables (CVs)
**Status:** 🟡 Packet encoding implemented, full programming flow still pending
**URL:** https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.2.2_decoder_cvs_2012.07.pdf
**Updated:** July 2012

**Features:**
- ✅ Service Mode packet encoding for Verify Byte / Write Byte
- CV programming execution flow
- CV programming on main
- Decoder configuration
- Address assignment

---

### S-9.2.4: DCC Fail-Safe
**Status:** ✅ Implemented (idle packet fallback)
**URL:** https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9_2_4_fail-safe.pdf
**Updated:** July 3, 2025

**Requirements:**
- Continuous signal (no gaps >30ms)
- Idle packets when no commands queued
- Emergency stop capability

**Implementation:**
```rust
// src/dcc/engine.rs — idle packet reuse (no clone), error handling on RMT transmit
loop {
    let rmt_pulses = match receiver.try_receive() {
        Ok(packet) => { /* encode + convert */ &packet_rmt_pulses },
        Err(_) => &idle_rmt_pulses,  // Pre-encoded, transmitted by reference
    };
    match tx_channel.transmit(rmt_pulses) { /* error handling */ }
    yield_now().await;  // Gap verified on hardware capture; keep checking after timing changes
}
```

---

## Decoder Interface Standards (Future - H-Bridge Design)

### S-9.1.2: DCC Power Station Interface
**Status:** 📋 Reference for Phase 4 (H-bridge design)
**URL:** https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.1.2_power_station_interface.pdf
**Updated:** June 9, 2021

**Relevance:** Current limiting, short circuit protection, booster interface

---

## Recommended Practices (RPs)

### RP-9.1.1: Electrical Standards for Single-Cab Boosters
**URL:** Check NMRA website for availability
**Relevance:** When designing H-bridge booster (Phase 4)

### RP-9.2.1: Decoder and Stationary Decoder CV Definitions
**URL:** Check NMRA website for availability
**Relevance:** CV programming implementation (Phase 3)

---

## Implementation Checklist

### ✅ Completed
- [x] S-9.1 timing compliance (58μs/"1", 100μs/"0") with hardware waveform verification
- [x] S-9.2 packet structure (preamble, start/end bits)
- [x] S-9.2 address encoding (short 1-127, long 128-10239, opaque validated type)
- [x] S-9.2 Speed28 interleaved encoding (NMRA table-verified)
- [x] S-9.2 Speed128 extended format (0x3F prefix)
- [x] S-9.2 Function Group 1 (FL, F1-F4)
- [x] S-9.2 Function Group 2A/2B (F5-F12)
- [x] S-9.2 Extended function groups 3/4 (F13-F28)
- [x] S-9.2 checksum (XOR)
- [x] S-9.2.4 idle packet fallback (pre-encoded, no-clone)
- [x] S-9.2.4 emergency stop (per-decoder + broadcast)
- [x] S-9.2.4 bounded software function refresh policy (`<=400ms` target under 12-slot load)
- [x] Validator: timing, structure, address, checksum, negative tests
- [x] Hardware waveform validation (oscilloscope / logic-analyzer capture)

### 🔜 Planned (Future Phases)
- [ ] S-9.2.1 advanced consisting
- [ ] S-9.2.2 CV programming execution flow (ACK handling / programming track orchestration)
- [ ] S-9.2.2 CV programming on main (POM)
- [ ] S-9.1.2 current limiting & short protection
- [ ] Decoder verification matrix across speed / direction / stop / e-stop cases

---

## Validation Strategy

### Software Validation (Current)
```rust
// Validates packet structure and encoding
DccValidator::validate_full(&packet, &pulses)
```

Recommended command sequence:

```bash
# Host-side logic validation (fast)
cargo test-host

# Embedded target compile checks
cargo check-esp

# Firmware artifact build
cargo build-esp-release
```

Notes:
- `cargo test-host` validates packet/encoder/validator logic on host.
- Timing waveform generation has been verified on real hardware with oscilloscope / logic-analyzer capture.
- Full end-to-end compliance is strongest when combined with decoder behavior verification on real rolling stock / decoders.

### Hardware Validation
1. **Oscilloscope / logic-analyzer verification** ✅
   - GPIO output captured and decoded as valid DCC waveform
   - Preamble observed above NMRA minimum (`22x "1"` in captured packet)
   - Packet structure validated on real signal: start bit, separators, end bit, checksum

2. **Decoder verification** 🔜
   - Expand evidence matrix for speed, direction, stop, and e-stop
   - Retain screenshots / logs for regression tracking after timing or power-path changes

---

## Quick Reference Links

- **Official NMRA Standards Index:** https://www.nmra.org/index-nmra-standards-and-recommended-practices
- **DCC Wiki:** http://www.dccwiki.com/
- **OpenDCC Project:** https://www.opendcc.de/ (German, good technical details)

---

## Notes for Development

**Before implementing new features:**
1. Read relevant S-9.x standard section
2. Check if validator needs updating
3. Add unit tests for compliance
4. Update this document with implementation notes

**When standards are updated:**
- Check NMRA website quarterly for revisions
- S-9.1 was updated April 2025 (we're compliant)
- S-9.2.1 was updated January 2025 (not implemented yet)
