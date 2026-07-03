# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DCC-esp32 is a `no_std` Rust project (edition 2024) for ESP32-C6 implementing a Digital Command Control (DCC) command station for model railroading. Uses Embassy async runtime on RISC-V (`riscv32imac-unknown-none-elf`).

**Current status:** Core DCC engine complete and hardware-verified (oscilloscope). Z21 UDP network layer complete and tested with real decoder + Z21 app (2026-03-17). Next phase: CV programming hardware integration (prog track relay, ACK circuit), short detector production enablement, multi-client support.

## Build and Run Commands

Custom cargo aliases are defined in `.cargo/config.toml`:

```bash
cargo test-host              # Run host-side unit tests (protocol logic, fast feedback)
cargo check-esp              # Type-check for ESP32-C6 target (no flash)
cargo build-esp              # Build firmware for ESP32-C6
cargo build-esp-release      # Release build (LTO enabled)
cargo run                    # Flash to device via espflash and monitor
cargo run --features rmt-stats  # Flash with RMT throughput telemetry (diagnostics only)
cargo fmt                    # Format code
cargo clippy                 # Lint
```

**Testing:** `cargo test-host` runs tests on the host (`x86_64-unknown-linux-gnu`). The crate uses `#![cfg_attr(not(test), no_std)]` so tests compile with `std`. Only `heapless` is available in test builds — ESP-specific deps are gated behind `cfg(target_arch = "riscv32")`. Tests are in-module (`#[cfg(test)] mod tests`) across the host-testable modules: the `dcc/` tree (`packet.rs`, `encoder.rs`, `validator.rs`, `scheduler.rs` + submodules, `speed28.rs`, `cv/`), the `railcom/` tree (`parser.rs`, `pipeline.rs`, `loco_tracker.rs`), `net/z21_proto` (+ `wire.rs`), `fault_manager.rs`, `system_status.rs`, `display.rs`, `control_buttons.rs`, and `control_logic.rs`.

**Running a single test:**
```bash
cargo test-host -- test_name
```

**Hardware:** GPIO2 outputs 3.3V DCC logic signal. GPIO3 is the digital input for track short-circuit detection (via 74HC14 Schmitt trigger). GPIO18 controls H-bridge enable. Monitor serial output with `espflash monitor`.

## Architecture

### Conditional Compilation Pattern

The crate compiles for both host (tests) and RISC-V (firmware). Modules that depend on ESP hardware (`engine.rs`, embedded parts of `scheduler.rs` and `cv.rs`) are gated with `#[cfg(target_arch = "riscv32")]`. The `dcc/mod.rs` re-exports follow the same pattern. Mock types (`MockTrackSwitch`, `MockAckDetector`) are available under `#[cfg(any(test, not(target_arch = "riscv32")))]`.

### Signal Generation Pipeline

```
DccPacket → encode_dcc_packet() → [PulseCode] → RMT peripheral → GPIO2
  (packet.rs)    (encoder.rs)      (timing.rs)    (engine.rs)
```

1. **`packet.rs`** — `DccPacket` enum (Idle, Reset, Speed28, Speed128, FunctionGroup1-4, ServiceModeVerifyByte, ServiceModeWriteByte) with `to_bytes()` encoding per NMRA S-9.2. Uses `heapless::Vec<u8, 6>` for no-alloc byte output. Speed28 uses NMRA interleaved bit encoding.

2. **`encoder.rs`** — Converts packet bytes to `PulseCode` sequences (high/low duration pairs). Output is `heapless::Vec<PulseCode, 64>`. Adds 14-bit preamble, start/end bits.

3. **`engine.rs`** — Embassy async task (`dcc_engine_task`) that runs an infinite loop: dequeue packets from `embassy_sync::Channel<..., 16>`, encode to RMT pulses, transmit via `esp_hal::rmt`. Falls back to pre-encoded idle packets when queue is empty. Uses `yield_now()` (not Timer delay) to maintain continuous DCC signal.

4. **`scheduler.rs`** — `SlotManager` handles cyclic packet refresh for active locomotives (max 12 slots) with priority queuing (e-stop > dirty > refresh). Supports Pause/Resume for coordination with CV programming. `packet_scheduler_task` is the async actor that processes commands via `SchedulerCommandChannel`.

5. **`cv.rs`** — CV Programming Service Mode implementation with trait-based hardware abstraction (`TrackSwitch`, `AckDetector`). Automatically coordinates main track pause/resume during CV operations. Supports Verify Byte and Write Byte operations per NMRA S-9.2.2. RMT transmission is currently stubbed pending hardware integration.

6. **`timing.rs`** — NMRA S-9.1 constants. RMT clock is 1MHz so microsecond values equal tick counts directly.

7. **`validator.rs`** — module-level validation functions check timing ranges, packet structure, checksum, and speed ranges against NMRA specs.

### Main binary (`src/bin/main.rs`)

Initializes ESP32-C6 peripherals, configures RMT on GPIO2 at 1MHz, creates static channels via `StaticCell`, spawns `dcc_engine_task` and `packet_scheduler_task` as Embassy tasks, then loops sending speed-ramping commands every 2 seconds.

Key patterns:
- `StaticCell` for static channel allocation (required by Embassy task spawning)
- Wrapper `#[embassy_executor::task]` functions needed because library async fns can't be directly spawned
- `#[deny(clippy::mem_forget)]` — `mem::forget` is unsafe with esp_hal types holding DMA buffers
- `#[deny(clippy::large_stack_frames)]` with `#[expect]` on specific functions

### System Modules (outside `dcc/`)

- **`fault_manager.rs`** — `FaultManagerState` state machine for E-stop and fault handling. Coordinates with scheduler to halt/resume track output. Testable on host.
- **`system_status.rs`** — `StatusModel` tracks runtime state (boot, running, paused, e-stop, fault) as an event-driven model consumed by LED and other subsystems. Testable on host.
- **`status_led.rs`** — Embassy task mapping `StatusModel` states to GPIO14 (green) and GPIO15 (red) LED blink patterns. ESP-only (`cfg(target_arch = "riscv32")`).
- **`control_buttons.rs`** — GPIO22 (stop) and GPIO21 (resume) with debounce and long-press detection. ESP-only.
- **`short_detector.rs`** — Digital short-circuit detection on GPIO3 via 74HC14 Schmitt trigger (active-low, falling-edge interrupt). BTS7960 R_IS/L_IS current sense through 10KΩ trimmer sets trip threshold. Boot blanking (5s) and debounce (50ms) filter transients. Emits `FaultEvent::FaultLatched(TrackShort)` to the fault manager. ESP-only (no host-testable state machine).

### Memory

- 65KB heap via `esp_alloc::heap_allocator!`
- Hot path (DCC engine loop) uses only stack-allocated `heapless` containers
- All `heapless::Vec` capacities are 64 or less to avoid stack overflow

## NMRA Standards Compliance

**CRITICAL:** All DCC features must comply with NMRA standards. Consult `docs/specs/NMRA-STANDARDS.md` before implementing.

- **S-9.1**: Bit timing — "1" = 58μs ±3μs, "0" = ≥100μs
- **S-9.2**: Packet format, addressing (short 1-127, long 128-10239), speed encoding, functions (F0-F28)
- **S-9.2.2**: Service Mode CV programming (Verify Byte, Write Byte), Direct Mode addressing (CV 1-256)
- **S-9.2.4**: Fail-safe — continuous signal, idle packets when queue empty

Use `validator::validate_full()` to verify packet + pulse compliance in tests.

## Key Dependencies

- `esp-hal` (with `unstable` feature) — Hardware abstraction, RMT peripheral
- `esp-rtos` — Embassy integration, alloc, WiFi radio support
- `embassy-sync` — Channel for inter-task communication
- `heapless` — Fixed-capacity collections for no-alloc paths (only dep available in host tests)
- `defmt` + `esp-println` — Efficient embedded logging (level via `DEFMT_LOG` env var in `.cargo/config.toml`)

## Conventions

- **Commits:** Conventional Commit style — `feat:`, `fix:`, `docs:`, `refactor:`, `perf:`, `style:`
- **Lints:** `#![deny(clippy::mem_forget)]` (unsafe with esp_hal DMA buffers), `#![deny(clippy::large_stack_frames)]` in firmware binary
