# RailCom Channel 1 Implementation Plan

## Goal

Add RailCom Channel 1 reception so the command station can detect locomotive
address broadcast datagrams, while preserving the currently working Channel 2
ACK/POM path.

Current state:

- UART RX on GPIO5 reads RailCom Channel 2 ACK reliably.
- `railcom diag` shows `ack` increasing and low parser error rate.
- `loco_id_windows` remains `0` because only ACK items are currently received.
- Runtime forces all RailCom UART windows to `RailcomChannel::Channel2`.

Primary outcome:

- `loco_id_ok > 0` when a decoder emits address broadcast datagrams.
- Channel 2 ACK/POM behavior remains stable.

Reference timing:

- RailCommunity RCN-217, RailCom DCC feedback protocol:
  <https://normen.railcommunity.de/RCN-217.pdf>
- Relevant timing from RCN-217:
  - cutout end: 454..488 us
  - Channel 1 starts around 80 us after cutout start; detector should be ready by 75 us
  - Channel 1 ends around 177 us
  - Channel 2 starts around 193 us
  - Channel 2 ends by 454 us

## Addendum 2026-05-18: ZIMO/RCN-218 Discovery and Logon

Decision:

- The original Channel 1 plan remains valid for window timing, parser split and
  diagnostics.
- The discovery part must be changed. Do not continue with ad-hoc probes,
  hardcoded locomotive address checks, or continuous aggressive broadcast.
- Implement the ZIMO/RCN-218-style flow as a small state machine outside ISR
  paths.

Reference implementation findings from ZIMO DCC:

- CH1 is used for decoder address and the first part of RCN-218 logon
  datagrams.
- CH2 is used for POM/dynamic data and the remaining part of RCN-218 logon
  datagrams.
- Decoder track-search is triggered by DCC binary state address `0`, function
  `2`, state `false`.
- RCN-218 logon enable/select packets use DCC address `254`.
- ZIMO sends search/logon traffic as a bounded discovery activity, not as an
  unbounded high-rate background task.

Revised outcome:

- First prove that CH1 ACK/address/logon symbols are decoded without parser
  hacks.
- Then run a bounded discovery/logon controller that can produce a real
  locomotive address from RailCom, not just ACK counts.
- Keep DCC packet scheduling predictable under many trains: discovery traffic
  is budgeted and must never preempt safety-critical or user command traffic.

Non-goals:

- No decoder-specific implementation for ESU, Rivarossi or ZIMO.
- No hardcoded address `3` probe.
- No parser rule that treats invalid bytes as valid RailCom.
- No policy/state machine inside ISR, timer, or UART capture paths.
- No continuous full-rate discovery while normal train traffic is active.

## Revised Implementation Tasks

### Task Z0: Reconcile Exploratory Changes

Target files:

- `src/boot.rs`
- `src/dcc/scheduler.rs`
- `src/dcc/packet.rs`
- `src/railcom/parser.rs`

Work:

- Audit current uncommitted RailCom changes.
- Remove exploratory behavior that is not in the ZIMO/RCN-218 flow.
- Keep only bounded diagnostics that explain what was received.
- Keep CH1/CH2 timing and parser work if it matches this plan.

Acceptance criteria:

- No hardcoded locomotive address probe remains.
- No aggressive always-on search loop remains.
- Logs still show enough to distinguish CH1 ACK, CH1 address/logon and CH2 data.

### Task Z1: Packet Encoding for Discovery and Logon

Target files:

- `src/dcc/packet.rs`
- `src/dcc/validator.rs`

Work:

- Add or verify exact DCC encoders for:
  - binary state short packet: address `0`, binary state `2`, state `false`
  - RCN-218 logon enable packet at address `254`
  - RCN-218 logon select packet at address `254`
- Add golden tests using byte sequences derived from the ZIMO reference.
- Support the 10-byte `LOGON_SELECT` packet without truncation.

Acceptance criteria:

- Host tests prove exact byte output for search, logon enable and logon select.
- `LOGON_SELECT` includes its CRC byte and final DCC XOR byte.

### Task Z2: Transport Capacity for Long DCC Packets

Target files:

- `src/dcc/timing.rs`
- `src/dcc/rmt_driver.rs`
- `src/boot.rs`

Work:

- Verify that the RMT encoder and channel memory can transmit the longest
  required RailCom/logon packet.
- Keep the change minimal and document why the memory budget is sufficient.
- Avoid increasing timing work in interrupt paths.

Acceptance criteria:

- `cargo test-host` passes.
- ESP build/check passes.
- Long packet tests prove the encoder does not silently truncate.

### Task Z3: RCN-218 ID15 Parser

Target files:

- `src/railcom/parser.rs`
- `src/railcom/pipeline.rs`

Work:

- Parse ID15 split across CH1 and CH2:
  - CH1 carries the first two symbols.
  - CH2 carries the remaining six symbols.
- Distinguish logon-enable response from logon-select response.
- Validate CRC where the RCN-218 payload requires it.
- Do not reinterpret invalid 4-of-8 bytes as useful data.

Acceptance criteria:

- Tests cover valid split ID15 datagrams.
- Tests cover invalid CRC and invalid 4-of-8 symbols.
- Diagnostics can say whether a non-empty window is ACK, address, logon or
  invalid.

### Task Z4: RailCom Discovery State Machine

Target files:

- `src/dcc/scheduler.rs`
- `src/boot.rs`

Work:

- Implement a small controller with explicit states:
  - idle
  - startup/rerail discovery window
  - logon enable sent
  - decoder ID received
  - logon select sent
  - address result received
  - backoff/done
- Use bounded retries and a low duty cycle.
- Send `LOGON_SELECT` only after a valid logon ID response.
- Keep controller decisions outside ISR and UART capture paths.

Acceptance criteria:

- `logon_sent` and `search_sent` counters are bounded and explainable.
- Discovery stops or backs off after the configured window.
- Normal packet scheduling continues under load.

### Task Z5: Scheduler Priority and CPU Budget

Target files:

- `src/dcc/scheduler.rs`
- `src/dcc/engine.rs`

Work:

- Treat RailCom discovery as low-priority telemetry.
- Never let discovery preempt emergency stop, safety refresh, or user command
  traffic.
- Add explicit counters for granted/skipped discovery packets.
- Keep allocation-free no_std behavior in the hot path.

Acceptance criteria:

- Scheduler tests cover command traffic plus discovery traffic.
- Discovery skips are visible in diagnostics.
- No extra heap allocation or unbounded queue is introduced.

### Task Z6: Hardware Verification Gates

Expected hardware sequence:

1. CH1 ACK/address/logon bytes decode without floods of parse errors.
2. `logon_sent` increments during the bounded discovery window.
3. A valid ID15 logon response is logged once.
4. `LOGON_SELECT` is sent for that decoder ID.
5. A valid address/logon-select response is decoded.
6. `loco_id_ok` increments and logs the locomotive address.
7. Z21-facing state can expose the discovered address.

Acceptance criteria:

- If CH1 only shows ACK but no ID15, the problem is discovery/logon scheduling.
- If ID15 appears but address does not, the problem is logon-select/parser.
- If address appears but Z21 does not show it, the problem is application
  integration, not RailCom capture.

### Task Z7: Cleanup After Proof

Target files:

- `src/boot.rs`
- `src/railcom/pipeline.rs`
- `docs/plans/2026-05-17-railcom-channel1-implementation-plan.md`

Work:

- Demote per-window logs to debug.
- Keep only useful summary counters at info level.
- Record final hardware evidence in this plan.

Acceptance criteria:

- Stable run produces low-volume logs.
- The final implementation path is documented without exploratory detours.

## Constraints

- Target: ESP32-C6, `esp-hal`, `esp-rtos`, Embassy, no_std firmware.
- Do not move parser work into ISR/timer paths.
- Keep ISR work bounded: GPIO/timer state changes and event-ring writes only.
- Preserve current Channel 2 ACK reliability before expanding behavior.
- Treat initial Embassy task timing as diagnostic only if used; final CH1 timing
  should be driven from the same bounded timer/event path as physical cutout.

## Phase 1: Model RailCom Sub-Windows

Target files:

- `src/track_output.rs`
- `src/railcom/uart_reader.rs`
- `src/boot.rs`

Add explicit logical events for RailCom receive sub-windows:

- `Channel1Open { packet_sequence }`
- `Channel1Close`
- `Channel2Open { packet_sequence }`
- `Channel2Close`

Implementation direction:

- Keep physical cutout lifecycle in `track_output`.
- After the physical cutout opens, schedule logical receive edges:
  - CH1 open at about 75..80 us after cutout open
  - CH1 close at about 177 us
  - CH2 open at about 193 us
  - CH2 close before physical cutout close
- Emit these events through the existing cutout runtime event mechanism or a new
  bounded event ring.
- Do not emit a duplicate generic `Close` at physical cutout close once CH2 has
  already been closed.

Acceptance criteria:

- Runtime can produce one CH1 window and one CH2 window per physical cutout.
- Existing CH2 ACK counters still increase.
- No `NestedOpen` warnings during normal operation.

## Phase 2: Extend Window Control Without Breaking UART Reader

Target files:

- `src/railcom/uart_reader.rs`
- `src/boot.rs`

Reuse `RailcomWindowControl::Open { packet_sequence, channel }` and `Close`.

Producer behavior:

- Send `Open { channel: Channel1 }`
- Send `Close`
- Send `Open { channel: Channel2 }`
- Send `Close`

Reader behavior:

- Keep the reader single-window-at-a-time.
- Continue draining idle-time UART noise only while no window is open.
- Keep overflow recovery bounded.
- Preserve current glitch policy:
  - `Glitch` is counted but does not corrupt a valid byte.
  - `Framing` after payload corrupts the window.

Acceptance criteria:

- Host tests cover sequential CH1 then CH2 windows.
- ESP check passes.
- No increase in `rx_overflow`.

## Phase 3: Implement Channel 1 Parser

Target files:

- `src/railcom/parser.rs`
- `src/railcom/pipeline.rs`

Current issue:

- `RailcomChannel::Channel1` is currently classified as
  `RailcomRxOutcome::UnsupportedChannel`.

Add:

- `parse_channel1(raw_bytes)`
- a Channel 1 item/result type, or a unified item enum if cleaner.
- support for address datagrams needed by loco ID:
  - `AdrLow`
  - `AdrHigh`

Keep Channel 1 and Channel 2 parsing separate:

- CH2 supports ACK/NACK/POM/CV data.
- CH1 should not accept ACK/NACK as a normal success path.
- Invalid CH1 data should fail closed and increment parse error counters.

Implementation detail to decide during coding:

- If CH1 datagram coding is identical to the existing 4-of-8 datagram decoder,
  reuse the low-level `decode_4_of_8` and `parse_datagram` logic.
- If CH1 has distinct framing or length semantics, create a dedicated parser
  wrapper with strict lengths.

Acceptance criteria:

- `Channel1` no longer maps to `UnsupportedChannel` when valid address bytes are
  present.
- Parser tests cover:
  - CH1 `AdrLow`
  - CH1 `AdrHigh`
  - invalid CH1 code
  - too-long CH1 window

## Phase 4: Make Loco ID Tracking Stateful

Target files:

- `src/railcom/loco_tracker.rs`
- `src/boot.rs`

Current behavior:

- Short address is identified from `AdrLow`.
- Long address requires `AdrHigh` and `AdrLow` in the same item slice.

Required change:

- Track recent `AdrHigh` by packet sequence/time.
- Combine `AdrHigh` with a following `AdrLow` if it arrives within a short,
  bounded window.
- Keep short-address identification from `AdrLow` alone.

Suggested state:

- `pending_adr_high: Option<{ value: u8, packet_sequence: u32 }>`
- maximum sequence gap for combining high+low, initially small and documented.

Acceptance criteria:

- `loco_id_windows` increments when CH1 address datagrams are seen.
- `loco_id_ok` increments for short and long addresses.
- `loco_id_invalid` increments for address fragments that cannot be completed.

## Phase 5: Keep POM Correlation CH2-Only

Target files:

- `src/boot.rs`
- `src/dcc/cv.rs`

Current POM protection:

- POM result forwarding is gated by `PENDING_POM_RAILCOM_PACKET_SEQUENCE`.

Required rule:

- Only Channel 2 parsed items may feed POM completion.
- Channel 1 address datagrams must never complete a POM request.

Acceptance criteria:

- Existing POM tests still pass.
- New test verifies CH1 address datagrams do not produce `PomRailcomResult`.

## Phase 6: Diagnostics

Target files:

- `src/railcom/pipeline.rs`
- `src/railcom.rs`
- `src/boot.rs`

Add counters split by channel:

- `ch1_windows`
- `ch1_empty`
- `ch1_ok`
- `ch1_err`
- `ch1_adr_low`
- `ch1_adr_high`
- `ch2_windows`
- `ch2_ack`

Keep normal logs low-volume:

- keep per-byte/per-window logs at `debug`
- keep `railcom diag` at `info`
- keep first loco identification at `info`

Acceptance criteria:

- `railcom diag` can show whether CH1 is empty, invalid, or decoded.
- Normal monitor is not flooded during stable ACK reception.

## Phase 7: Tests and Verification

Host tests:

- `cargo test-host`
- parser CH1 valid/invalid cases
- pipeline CH1 valid address cases
- loco tracker stateful high/low combination
- POM ignores CH1

Firmware checks:

- `cargo check-esp`
- `cargo build-esp-release`
- optionally `cargo clippy-host`
- optionally `cargo clippy-esp`

Bench validation:

1. Flash firmware.
2. Confirm CH2 baseline remains good:
   - `ack` increases
   - `rx_err` remains low
   - `rx_corrupted` remains 0
3. Confirm CH1 is active:
   - `ch1_windows > 0`
   - `ch1_empty` not equal to `ch1_windows`, if decoder transmits broadcast
4. Confirm loco ID:
   - `loco_id_windows > 0`
   - `loco_id_ok > 0`
   - log shows `railcom loco identified: addr=...`

If CH1 windows are consistently empty while CH2 ACK is stable:

- decoder may not have RailCom address broadcast enabled,
- decoder CV28/CV29 settings need review,
- detector polarity/single-ended behavior may be missing the CH1 direction,
- CH1 timing offsets need scope validation.

## Risks

- Embassy task jitter may be too high for precise CH1/CH2 slicing if the first
  implementation uses async delays. Prefer hardware timer/event sequencing for
  the final version.
- The existing single UART peripheral cannot truly receive two independent
  simultaneous streams; correctness depends on cleanly closing CH1 before CH2.
- Single-ended detector hardware may miss responses depending on current
  direction, which can appear as CH1 empties even with correct firmware.
- Some decoders only emit address broadcast when configured via CVs.

## Rollback Plan

- Keep CH1 implementation behind a small config constant or feature flag during
  bring-up.
- If CH1 destabilizes CH2, disable CH1 sub-window emission and restore the
  current single CH2 window behavior.
- Parser and tracker additions can remain dormant if no CH1 windows are emitted.
