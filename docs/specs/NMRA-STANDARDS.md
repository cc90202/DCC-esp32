# NMRA DCC standards

Notes on which parts of the NMRA Digital Command Control standards this firmware
implements, where the implementation lives, and what has been verified on real
hardware. The standards index is at
<https://www.nmra.org/index-nmra-standards-and-recommended-practices>.

## S-9.1, electrical standard

Revision of 30 April 2025,
[PDF](https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.1_electrical_standards_for_digital_command_control.pdf).

The standard fixes a "1" bit at 58μs per half-period with a 3μs tolerance, a "0"
bit at 100μs or longer, and a preamble of at least 14 "1" bits. Track voltage is
specified between 12V and 22V, which the firmware does not produce directly: it
emits 3.3V logic and relies on the external H-bridge for the power stage.

The timings live in `src/dcc/timing.rs`:

```rust
pub const DCC_ONE_HIGH_US: u16 = 58;   // 55-61μs acceptable
pub const DCC_ONE_LOW_US: u16 = 58;
pub const DCC_ZERO_HIGH_US: u16 = 100; // 95-9900μs acceptable
pub const DCC_ZERO_LOW_US: u16 = 100;
pub const PREAMBLE_BITS: usize = 20;   // measured capture showed stable 22x "1" preamble
```

`validator::validate_timing()` checks generated pulse durations against those
ranges.

The waveform has been captured on the real output with an oscilloscope and a
logic analyzer, and decoded as valid DCC. The captured packet showed a stable
22-bit preamble, valid start and end bits, address `0x03`, command `0x90` and
checksum `0x93`. Before that capture the compliance claim rested on the software
timing constants alone.

## S-9.2, communications standard

Revision of July 2004,
[PDF](https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9_2-2004-07.pdf).

A packet is a preamble, then a zero bit before each of the address byte, the
instruction byte and the error-detection byte, and a final one bit. Addresses are
short (1 to 127) or long (128 to 10239), speed comes in 14, 28 or 128 steps,
direction travels in the instruction byte, and the checksum is the exclusive-or
of the data bytes.

The packet types are in `src/dcc/packet.rs`:

```rust
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

Section 2.3.2.3 requires the interleaved form for 28-step speed, where the least
significant bit of the speed value moves to bit 4. Getting this wrong produces
decoders that run but stutter between steps:

```rust
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

`validator::validate_nmra_compliance()` checks address and speed ranges.

## S-9.2.1, extended packet formats

Revision of 24 January 2025,
[PDF](https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.2.1_dcc_extended_packet_formats.pdf).

Only the 128 speed step format is implemented, as `DccPacket::Speed128`.
Advanced consisting, decoder lock and firmware upload are not.

## S-9.2.2, configuration variables

Revision of July 2012,
[PDF](https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.2.2_decoder_cvs_2012.07.pdf).

Service mode packet encoding for verify byte and write byte is implemented. The
surrounding programming flow is not: there is no acknowledgement detection, no
programming track orchestration, and no decoder configuration or address
assignment logic yet. Programming on the main is covered separately by the
RailCom work.

## S-9.2.4, fail-safe

Revision of 3 July 2025,
[PDF](https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9_2_4_fail-safe.pdf).

The signal must be continuous, with no gap longer than 30ms, so idle packets fill
any moment where no command is queued, and an emergency stop path must exist.

In `src/dcc/engine.rs` the idle packet is pre-encoded once and transmitted by
reference, so the fallback path allocates and clones nothing:

```rust
loop {
    let rmt_pulses = match receiver.try_receive() {
        Ok(packet) => { /* encode + convert */ &packet_rmt_pulses },
        Err(_) => &idle_rmt_pulses,  // Pre-encoded, transmitted by reference
    };
    match tx_channel.transmit(rmt_pulses) { /* error handling */ }
    yield_now().await;  // Gap verified on hardware capture; keep checking after timing changes
}
```

## S-9.1.2, power station interface

Revision of 9 June 2021,
[PDF](https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.1.2_power_station_interface.pdf).

Not implemented. Relevant to current limiting, short circuit protection and the
booster interface when the H-bridge design is revisited.

Two recommended practices matter for the same future work but were not available
on the NMRA site at the time of writing: RP-9.1.1 on electrical standards for
single-cab boosters, and RP-9.2.1 on decoder and stationary decoder CV
definitions.

## What is implemented

Timing compliance under S-9.1, with hardware waveform verification. Under S-9.2:
packet structure with preamble and start and end bits, short and long address
encoding behind a validated address type, the interleaved 28-step speed encoding
checked against the NMRA table, the 128-step format with its `0x3F` prefix,
function groups 1, 2A and 2B covering FL through F12, extended groups 3 and 4
covering F13 through F28, and the exclusive-or checksum. Under S-9.2.4: the idle
packet fallback, per-decoder and broadcast emergency stop, and a bounded software
refresh policy for function state that targets 400ms or better with all 12 slots
occupied. The validator covers timing, structure, address, checksum and the
matching negative cases.

Still open: advanced consisting, the CV programming execution flow with
acknowledgement handling and programming track orchestration, programming on the
main, current limiting and short protection at the power stage, and a decoder
verification matrix across speed, direction, stop and emergency stop.

## Validation

`validator::validate_full(&packet, &pulses)` checks an encoded packet and its
pulse train together. The usual sequence before flashing is `cargo test-host`
for the host-side packet, encoder and validator logic, then `cargo check-esp` for
the embedded compile check, then `cargo build-esp-release` for the artifact.

Waveform generation has been verified on real hardware: the GPIO output was
captured and decoded as a valid DCC waveform, with a preamble above the NMRA
minimum (22 "1" bits in the captured packet) and a packet structure that held up
on the real signal, start bit through checksum.

What is still missing on the hardware side is a broader decoder verification
matrix across speed, direction, stop and emergency stop, with captures kept for
regression tracking after timing or power-path changes. Compliance is only really
established when the encoder is confirmed against decoder behaviour on real
rolling stock.

## Other references

The DCC Wiki at <http://www.dccwiki.com/> is useful for background. The OpenDCC
project at <https://www.opendcc.de/> has good technical detail, in German.

When picking up a new standard section, read it before touching the encoder,
check whether the validator needs a matching rule, add the unit tests, and record
the outcome here. S-9.1 was last revised in April 2025 and the firmware follows
that revision. S-9.2.1 was revised in January 2025 and is only partly
implemented.
