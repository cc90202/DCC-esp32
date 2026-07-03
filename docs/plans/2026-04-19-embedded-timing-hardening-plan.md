# Embedded Timing Hardening Plan

## Goal

Increase confidence that the ESP32-C6 hardware/software stack can sustain up to 12 active decoders without stalls, hidden drops, timing regressions, or recovery paths that steal too much budget from critical runtime paths.

This plan prioritizes:

- bounded work in ISR and cutout-adjacent paths,
- explicit backpressure semantics,
- runtime timing observability,
- stress validation under realistic multi-train load.

## Roadmap

### 1. Add minimal timing and backpressure instrumentation

Before changing channel capacities or moving logic, add lightweight metrics to the critical runtime path.

Target files:

- `src/dcc/rmt_driver.rs`
- `src/track_output.rs`
- `src/railcom/uart_reader.rs`
- `src/boot.rs`

Recommended metrics:

- `rmt_heartbeat_gap_max`
- `cutout_request_to_open_notify_max`
- `cutout_open_to_uart_window_open_max`
- `uart_drain_iterations_max`
- `uart_reader_error_count` per error kind
- `channel_full_count` for every `try_send()`
- `railcom_processing_window_max`

Expected outcome:

- timing-sensitive behavior is measurable from diagnostics,
- channel sizing discussions can be based on data rather than assumptions,
- regressions become visible during stress runs.

### 2. Classify channels by delivery semantics

Review every channel and explicitly classify it as one of:

- `control-critical`
- `best-effort telemetry`
- `latest-state`

Implications:

- `control-critical`: no silent drop behavior
- `best-effort telemetry`: drops allowed, but counted
- `latest-state`: consider `watch` if queued history is not useful

Channels that should currently remain in the `control-critical` bucket:

- `SchedulerCommandChannel`
- `DccPacketChannel`
- `FaultEventChannel`
- `RailcomWindowControlChannel`

Expected outcome:

- future changes to queue depth or send semantics are constrained by documented intent,
- non-critical paths cannot accidentally starve critical paths.

### 3. Fully bound recovery paths in RailCom UART reader

The RailCom UART reader remains one of the most timing-sensitive integration points. Every error branch must have a clearly bounded maximum cost.

Target file:

- `src/railcom/uart_reader.rs`

Focus areas:

- FIFO drain policy
- number of drain iterations
- adapter reset behavior after UART errors
- explicit "abandon current window, recover on next window" behavior where appropriate

Expected outcome:

- recovery cannot monopolize cutout-adjacent time budget,
- worst-case behavior is easier to reason about,
- noise bursts degrade data quality rather than destabilize runtime behavior.

### 4. Keep logging off the hot path

Per-packet and per-window paths should avoid `info!` unless they represent rare state transitions.

Target files:

- `src/boot.rs`
- `src/railcom/uart_reader.rs`
- `src/railcom/uart_adapter.rs`

Rules:

- `debug!` or less for frequent runtime observations
- `info!` only for rare, operator-relevant transitions
- repetitive warnings should be aggregated or rate-limited if needed

Expected outcome:

- less timing variance under WiFi and runtime load,
- better signal-to-noise ratio in field logs.

### 5. Validate channel sizing with measurements, not intuition

Do not globally reduce channel capacities based on average producer frequency. Size queues against burst behavior, producer fan-in, path criticality, and consequences of backpressure.

Likely keep as-is unless measurements justify change:

- `SchedulerCommandChannel`
- `DccPacketChannel`
- `FaultEventChannel`
- `RailcomWindowControlChannel`
- `RailcomUartRuntimeResultChannel`

Possible future reduction candidates after measurement:

- `DisplayChannel`
- `NetStatusChannel`
- possibly `SystemStatusChannel`

Expected outcome:

- channel depth changes become low-risk tuning rather than speculative refactors.

### 6. Build a repeatable "12 decoder" stress scenario

Create a validation scenario that approximates the actual deployment target.

The scenario should combine:

- 12 active locomotives or equivalent scheduler load
- frequent speed and direction updates
- function toggles
- continuous Z21/network traffic
- RailCom enabled
- occasional POM activity
- injected UART noise or recovery cases if possible

Observe during the run:

- watchdog stability
- critical-path queue saturation
- bounded timing metrics
- absence of growing backlog
- absence of dropped control-critical events

Expected outcome:

- the target operating envelope is tested explicitly rather than inferred.

### 7. Document timing invariants and ownership boundaries

Document the rules that future changes must not violate.

Target files:

- `src/dcc/rmt_driver.rs`
- `src/track_output.rs`
- `src/railcom/uart_reader.rs`
- `src/dcc/scheduler.rs`

Invariants to document:

- what is allowed in ISR context
- which recovery paths must remain bounded
- which queues must never silently drop
- which module owns policy vs. pure mechanism
- which metrics define a healthy runtime state

Expected outcome:

- reduced regression risk when evolving RailCom, scheduling, or timing logic.

## Execution Plan

### P0. Timing and backpressure instrumentation

Files:

- `src/dcc/rmt_driver.rs`
- `src/track_output.rs`
- `src/railcom/uart_reader.rs`
- `src/boot.rs`

Done when:

- the runtime exposes max/aggregate counters for heartbeat gaps, cutout-to-window delays, UART drain work, UART error classes, and `try_send()` saturation.

### P0. Bounded RailCom UART recovery

Files:

- `src/railcom/uart_reader.rs`

Done when:

- every UART error branch has an explicit bounded maximum cost,
- recovery behavior is documented inline,
- the implementation favors abandoning a bad window over overworking inside it.

### P1. Explicit channel policy

Files:

- `src/dcc/engine.rs`
- `src/dcc/scheduler.rs`
- `src/fault_manager.rs`
- `src/system_status.rs`
- `src/display.rs`
- `src/railcom/uart_reader.rs`

Done when:

- each channel has a clear intended delivery contract,
- every non-critical `try_send()` path records drops.

### P1. Hot-path logging cleanup

Files:

- `src/boot.rs`
- `src/railcom/uart_reader.rs`
- `src/railcom/uart_adapter.rs`

Done when:

- logs in per-packet/per-window paths are limited to debug-level or rarer,
- warnings cannot become sustained log storms under noise.

### P1. Repeatable 12-decoder stress validation

Files:

- `docs/plans/`
- any supporting test or harness code needed later

Done when:

- a documented stress procedure exists,
- pass/fail criteria are defined,
- diagnostics are sufficient to explain failures.

### P2. Selective non-critical channel tuning

Files:

- `src/display.rs`
- `src/system_status.rs`

Done when:

- reductions are applied only after repeated stress runs show zero harmful pressure,
- changes are made one step at a time.

### P2. Timing invariant documentation

Files:

- `src/dcc/rmt_driver.rs`
- `src/track_output.rs`
- `src/railcom/uart_reader.rs`
- `src/dcc/scheduler.rs`

Done when:

- critical timing assumptions and ownership boundaries are written directly where future maintainers will touch the code.

## Recommendation on Channel Sizing

Do not treat queue downsizing as a quick win.

For the target of up to 12 decoders, prefer elasticity on critical control paths over small memory savings. A slightly larger queue is acceptable; hidden backpressure or dropped control events are not.

Current recommendation:

- keep critical control-plane and RailCom runtime queues as they are unless instrumentation proves they are oversized,
- only tune non-critical queues after stress validation,
- never apply a blanket depth reduction rule from average event frequency alone.
