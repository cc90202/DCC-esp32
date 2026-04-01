# RailCom Implementation Design — DCC Command Station

Date: 2026-03-31
Status: Revised after hardware/firmware review

## Overview

This document replaces the previous "full RailCom on current breadboard" proposal with a staged plan aligned to:

- the current command station hardware in [breadboard-wiring-guide.md](/home/crist/DCC-esp32/docs/hardware/breadboard-wiring-guide.md),
- the currently available loose parts in [components-inventory.md](/home/crist/DCC-esp32/docs/hardware/components-inventory.md),
- the actual firmware ownership of `GPIO18` and the H-bridge enable path,
- the physical constraints of RailCom per NMRA S-9.3.2.

The key conclusion is:

- a useful RailCom proof-of-concept is plausible on the current hardware,
- a robust "commercial-grade" RailCom implementation is not plausible with the current breadboard detector topology,
- the first implementation target should be **global RailCom reception on a single main output, CH2-first**, not "full CH1 + CH2 support".

## Review Outcome

### What was wrong in the previous draft

The previous draft assumed that:

- disabling the BTS7960 through `R_EN/L_EN` and adding a diode bridge plus `4.7Ω` burden resistor created a valid RailCom cutout path,
- the same path could also serve as the detector front-end,
- an LM393 with the proposed threshold and polarity was sufficient for direct 250 kbps UART reception.

Those assumptions are not safe enough for implementation.

### Why

Per NMRA S-9.3.2:

- during cutout the rails must be disconnected from the booster and shorted together,
- the cutout device should add at most `10 mV` at `34 mA`,
- the detector should add at most `200 mV` at `34 mA`,
- detector interpretation is based on current threshold, not on "30 mA nominal only",
- bit time is `4 μs` at `250 kbps`, with rise/fall times expected within `0.5 μs`.

The earlier bridge + `4.7Ω` + MOSFET topology is therefore not acceptable as the primary architecture. It may still be useful as an experiment fixture, but not as the design baseline.

## Project Goal

### Phase Goal

Implement **RailCom proof-of-concept reception** on the current single-output DCC station with these constraints:

- single global detector only,
- no rail section localization,
- prioritize CH2 reception,
- preserve safety ownership of `GPIO18`,
- require oscilloscope validation before parser integration.
- preserve command responsiveness even with many active locomotives,
- keep CPU load well below saturation during normal operation.

### Non-Goals for this phase

- full CH1 + CH2 feature parity,
- RailCom block detection / occupancy sectioning,
- RailComPlus-like commercial UX,
- production-ready detector hardware,
- claiming conformance based only on breadboard success.

### Operational constraints discovered during bring-up

The implementation must satisfy these runtime constraints:

- up to `12` locomotives active at the same time without visible command lag,
- RailCom must not noticeably delay speed/function updates from the app,
- CPU usage must remain bounded; RailCom must not push the ESP32-C6 "to the limit",
- DCC command throughput has priority over RailCom feedback throughput.

### Core performance principle

Keep the RMT path and all ISR paths as short as possible, and rely on hardware
wherever possible so RailCom does not burden the CPU or degrade DCC quality.

Practical meaning:

- RMT remains a DCC timing engine first,
- ISR code does only minimal fixed-time work,
- timers should be hardware timers,
- UART reception should use UART hardware,
- `Open/Close` window control should be derived from cutout lifecycle events with minimal latency; do not assume that a coarse polling loop is timing-correct enough for final RailCom reception,
- detector thresholding should be handled in hardware, not reconstructed in software,
- any policy, parsing, correlation, or non-trivial state handling must stay outside the timing-critical path.

## Current Hardware Baseline

Relevant existing wiring from [breadboard-wiring-guide.md](/home/crist/DCC-esp32/docs/hardware/breadboard-wiring-guide.md):

- `GPIO2` drives DCC through the `74HC14`
- `GPIO18` drives BTS7960 `R_EN + L_EN`
- `GPIO3` is already used by short detection from `74HC14 pin 6`
- `GPIO19/GPIO20` are occupied by I2C OLED
- one `74HC14` package is already present and has spare gates
- available loose parts include `LM393N`, `IRLZ44N`, `1N5819`, resistor kit

Implication:

- `GPIO18` is not "free"; it is part of the safety-critical track enable path,
- `GPIO3` is not available for RailCom,
- `GPIO5` is available on paper but must be validated against ESP32-C6 boot/runtime constraints before wiring,
- current firmware assumption for the UART bring-up path: `GPIO5` = RailCom RX, `GPIO4` kept free as debug/marker reserve,
- the current inventory lacks a purpose-built fast RailCom detector front-end.

## Reference Architecture

Commercial systems separate these concerns:

- **booster / cutout generation**
- **global detector**
- **local / sectional detectors**
- **higher-level protocol integration**

For this project the closest practical target is:

1. create a reliable cutout,
2. sample one global return channel,
3. decode CH2-first,
4. integrate events into Z21 transport.

## Architecture Decision

### Decision 1: Scope

Use **global RailCom reception only** in v1.

Rationale:

- the station currently has one main output only,
- without block sectioning, CH1 has limited operational value,
- CH2 is the higher-value global feedback path for early integration.

### Decision 1b: RailCom must be budgeted, not attached blindly to every packet

Do **not** execute a cutout at every packet boundary in the final design.

Rationale:

- testing showed that "cutout on every packet" introduces visible command delay,
- with many active locomotives, command bandwidth is more important than maximum possible RailCom window frequency,
- a command station must remain responsive under load; RailCom is secondary to motion/control traffic.

Decision:

- final RailCom cutout scheduling must be rate-limited,
- cutout policy must live above the RMT timing layer,
- DCC packet priority and refresh guarantees must remain intact with up to `12` active locomotives.

### Decision 2: Firmware ownership of `GPIO18`

Do **not** let both `fault_manager` and the RMT ISR directly own `GPIO18`.

Rationale:

- `GPIO18` is currently owned by the fault path in [boot.rs](/home/crist/DCC-esp32/src/boot.rs#L489) and [fault_manager.rs](/home/crist/DCC-esp32/src/fault_manager.rs#L300),
- safety and cutout cannot race on the same pin without a single arbiter.

Decision:

- introduce a dedicated track-output control layer that owns `GPIO18`,
- `fault_manager` publishes "track inhibit / track armed" state,
- cutout logic requests a temporary cutout only when track state is normal and armed.

### Decision 2b: Keep logic out of ISR and out of the RMT core

Do **not** place RailCom policy or protocol logic inside the RMT ISR or inside the
timing-critical core of the RMT driver.

Rationale:

- the current ISR-driven RMT backend is the foundation of stable DCC output,
- RailCom must not degrade packet continuity, ISR latency, or RMT RAM update timing,
- embedded failures here are often intermittent and hard to diagnose,
- "it still works once" is not enough evidence for safety-critical timing code.

Rule:

- ISR and RMT-core code may do only constant-time, timing-critical work,
- acceptable work in ISR is limited to very small atomic/counter updates, fixed register writes, and minimal event signaling,
- the current `*_fast()` cutout wrappers are an intermediate hardening step; if future stress tests show any DCC degradation, the next change is to replace their internal HAL calls with more direct low-level register access rather than adding more logic around them,
- no protocol parsing, no queue walking, no dynamic branching on complex state, no UART handling, no logging, no allocations,
- any RailCom behavior beyond a minimal timing hook must live in a separate module/task outside the RMT critical path.

Consequence:

- `rmt_driver.rs` should remain a DCC timing engine first,
- RailCom should attach through a minimal boundary hook,
- every future change that adds work to the ISR must be reviewed again against DCC continuity.

### Decision 3: Cutout vs detector

Treat **cutout generation** and **signal detection** as separate subsystems.

Rationale:

- RailCom cutout is not just "track disabled",
- the detector burden and the cutout path have different electrical constraints,
- mixing them in one breadboard path creates unclear failure modes.

Decision:

- use the BTS7960 enable line only as a candidate way to stop driving the rails,
- separately validate whether the rails are actually shorted together during cutout,
- if the BTS7960 cannot produce a valid cutout behavior, stop the project at the hardware gate and do not continue to parser work.

### Decision 4: Detector target

The breadboard detector is a **proof-of-concept receiver**, not the final detector design.

Rationale:

- LM393 is borderline for 250 kbps direct slicing,
- breadboard parasitics will materially affect edge quality,
- current inventory does not include a detector known to match commercial RailCom front-ends.

Decision:

- implement the detector only behind an explicit "experimental" gate,
- require waveform captures before enabling protocol parsing by default.

## Revised Hardware Plan

### Hardware Gate 0: Verify actual cutout behavior before building the detector

Before adding RailCom receive hardware, verify with oscilloscope:

1. BTS7960 outputs stop driving within the required timing after `GPIO18` goes LOW.
2. The two rails become effectively shorted together during the cutout window.
3. There is no damaging transient on the rails when normal DCC resumes.

Acceptance criteria:

- cutout starts within the RailCom timing budget,
- differential rail voltage during cutout is close to zero,
- no destructive overshoot is seen at resume,
- DCC timing still meets current project tolerances outside the cutout window.

If this fails, do not continue with the current hardware architecture.

### Hardware Gate 1: Experimental global detector only

If Gate 0 passes, build a detector breadboard as an experiment fixture only.

Constraints:

- detector burden must stay within RailCom limits,
- comparator threshold must target the actual NMRA detection region, not `~20 mA`,
- output polarity must match UART idle/start-bit polarity,
- analog front-end must not inject current into the 3.3 V rail during normal DCC.

### Detector notes

The previous `4.7Ω / 97mV` proposal is rejected as baseline because:

- `97 mV / 4.7Ω ≈ 20.6 mA`, too high for a valid `0` threshold,
- polarity was mapped incorrectly for UART use,
- the burden path was entangled with the cutout path,
- the clamp arrangement risked back-powering the logic rail.

For the experimental detector:

- choose a burden resistor and threshold so that:
  - current clearly above `10 mA` is seen as `0`,
  - current clearly below `6 mA` is seen as `1`,
- add only minimal filtering,
- keep wiring short,
- document the resulting burden voltage at `10 mA`, `30 mA`, and `34 mA`.

### Wiring impact

No assumptions should be made that the existing wiring remains "unchanged" after adding the detector.

Required checks after each added stage:

- DCC waveform on rails
- rail differential voltage during cutout
- `GPIO18` edge timing
- detector output edge quality at the GPIO input
- short detector on `GPIO3` still behaves correctly

## Revised GPIO Plan

### Allocated today

- `GPIO2`: DCC generation
- `GPIO3`: short-circuit detector input
- `GPIO18`: H-bridge enable / future cutout arbitration
- `GPIO19/GPIO20`: OLED I2C
- `GPIO21/GPIO22`: buttons

### RailCom candidate pins

- preferred RX candidate: `GPIO5`
- fallback candidate: `GPIO4`

Final pin choice must be validated against:

- ESP32-C6 boot constraints,
- UART peripheral routing in `esp-hal`,
- physical wiring convenience on the current breadboard.

## Revised Software Architecture

### 1. `track_output.rs` or equivalent

New owner for track output state.

Responsibilities:

- own `GPIO18`,
- expose `set_enabled(bool)` for safety state,
- expose `request_cutout()` only when allowed,
- serialize safety-off vs cutout requests,
- provide current state to diagnostics.

This prevents `fault_manager` and interrupt code from racing on the same pin.

### 2. `rmt_driver.rs`

Extend current ISR-driven RMT backend only after Hardware Gate 0 passes.

Responsibilities:

- detect packet boundary,
- request cutout from the track output arbiter,
- arm a hardware timer for cutout end,
- skip cutout if track output is inhibited,
- expose instrumentation counters for:
  - cutout requested
  - cutout executed
  - cutout skipped
  - cutout overrun / timing fault

Constraint:

- keep the ISR path strictly minimal,
- if a behavior cannot be expressed as a few fixed-time operations, it does not belong in the ISR,
- the RMT RAM refill path must not gain RailCom-specific complexity beyond a minimal hook.

Additional constraint:

- the RMT ISR must not decide RailCom policy based on "every boundary",
- it may only consume a minimal precomputed decision such as "cutout allowed now".

### 2b. RailCom cutout budget / scheduler policy

Add a small policy layer that decides when RailCom is allowed to consume DCC bandwidth.

Responsibilities:

- rate-limit cutouts,
- preserve command responsiveness,
- protect the scheduler refresh budget for up to `12` active locomotives,
- ensure command/safety packets keep priority over feedback windows,
- expose counters for:
  - cutout requested
  - cutout granted
  - cutout skipped by budget
  - cutout skipped by packet priority

Rule:

- RailCom must use only a bounded share of packet opportunities,
- the exact policy may be packet-count based or time-budget based, but it must be deterministic and measurable.

### 3. `railcom_rx.rs`

Experimental UART receive task.

Responsibilities:

- configure UART RX on selected GPIO at `250000 8N1`,
- clear stale bytes before each expected cutout,
- read bytes only inside the expected receive window,
- record framing and timeout errors,
- export raw capture statistics before protocol events are trusted.

This module must start as diagnostic-first, not parser-first.

### 4. `railcom_decode.rs`

Pure host-testable logic.

Responsibilities:

- 4/8 decoding table,
- invalid-code rejection,
- channel split handling,
- datagram extraction.

Priority:

- CH2 first
- CH1 later

### 5. `railcom_service.rs`

Bridges raw RX data to higher layers.

Responsibilities:

- aggregate decoded events,
- de-duplicate noisy repeats,
- publish Z21-visible RailCom data,
- provide counters for observability.

## Protocol Priorities

### Phase 1

- cutout timing only
- raw UART capture
- frame/error counters
- CH2 decode for simple global feedback

### Phase 2

- CH1 support
- address correlation improvements
- CV readback experiments

### Phase 3

- integration with section detectors or future PCB front-end
- feature set closer to commercial command stations

## Validation Plan

### Stage A: Electrical validation

Must be completed on bench with oscilloscope before parser work is considered valid.

Measure:

1. rail voltage during normal DCC
2. rail differential voltage during cutout
3. cutout start delay from last packet end edge
4. cutout total duration
5. detector analog burden voltage
6. detector digital output rise/fall time

Required result:

- cutout timing within target window,
- detector burden within allowed range,
- digital edges clean enough for 250 kbps reception,
- no regression in short-circuit detection path.

### Stage B: Raw receive validation

Use a known RailCom-capable decoder.

Check:

- UART sees stable bytes repeatedly,
- framing errors remain bounded,
- bytes correlate with cutout timing,
- no spurious traffic appears outside cutout windows.

### Stage B2: Throughput and responsiveness validation

This stage is mandatory before claiming the software architecture is acceptable.

Test with up to `12` active locomotives and app-driven control traffic.

Check:

- speed/function commands remain responsive,
- no visible lag appears when RailCom is active,
- scheduler refresh remains acceptable under load,
- CPU does not approach sustained saturation,
- no RMT heartbeat stalls or internal faults appear during prolonged operation.

### Stage C: Decode validation

Host-test:

- 4/8 lookup table correctness
- invalid code rejection
- channel slicing
- datagram extraction

Bench-test:

- same locomotive repeatedly reports stable data,
- repeated runs give the same decoded result,
- disabling cutout removes all valid received data.

## Risks

### High risk

- BTS7960 may not create a RailCom-valid cutout even if it "stops driving"
- breadboard parasitics may make the detector unreliable at 250 kbps
- LM393 may be too slow or too sloppy for direct UART slicing
- RailCom changes may accidentally destabilize the ISR-driven RMT path if logic leaks into the timing-critical code
- RailCom may steal too much packet bandwidth and degrade control responsiveness under multi-loco load

### Medium risk

- `GPIO18` arbitration may complicate current safety flow
- DCC resume after cutout may introduce glitches
- a global detector on a single output may have limited operational value

### Low risk

- 4/8 decode logic itself
- Z21 transport plumbing once raw events are trustworthy

## Implementation Sequence

1. Introduce track-output arbitration around `GPIO18`.
2. Add cutout timing instrumentation only, with RailCom RX disabled.
3. Validate cutout electrically on the real hardware.
4. Build experimental detector front-end.
5. Validate detector analog and digital waveforms.
6. Add raw UART capture with diagnostics.
7. Add CH2 decode.
8. Add Z21 integration for experimental RailCom events.
9. Reassess whether CH1 is worth implementing on this hardware.

## Exit Criteria

The phase is successful if all of the following are true:

- cutout timing is repeatable and electrically validated,
- the firmware preserves safety ownership semantics around track enable,
- raw RailCom bytes are captured repeatably from a known decoder,
- CH2 decoding produces stable events,
- no regression is introduced in normal DCC output or short-circuit protection,
- with up to `12` active locomotives, command responsiveness remains acceptable,
- CPU headroom remains sufficient during combined DCC + RailCom operation.

The phase is not successful if:

- cutout cannot be shown to meet the physical requirements,
- detector burden exceeds limits,
- reliable receive depends on fragile breadboard tuning,
- safety behavior around `GPIO18` becomes ambiguous.

## Practical Conclusion

For the current hardware, the correct next step is not "implement full RailCom".

The correct next step is:

1. prove that the station can generate a physically valid cutout,
2. prove that a single experimental detector can receive global RailCom data,
3. decode CH2 first,
4. postpone anything resembling commercial-grade RailCom until the hardware front-end is improved.
