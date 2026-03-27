## Summary
- Behavioral impact:
- Why this change:
- Risk level (low/medium/high):

## Verification
- [ ] `cargo fmt --all -- --check`
- [ ] `cargo test-host`
- [ ] `cargo check-esp`
- [ ] `cargo clippy --lib --target x86_64-unknown-linux-gnu -- -D warnings`
- [ ] `cargo clippy --lib --target riscv32imac-unknown-none-elf -- -D warnings`

## Safety / Runtime Notes
- [ ] No regressions in fault handling (`Stop`, `Resume`, short detector)
- [ ] No regressions in keepalive/deceleration and emergency-stop behavior
- [ ] No new magic numbers in protocol/timing/threshold logic

## Hardware Validation (required when touching DCC timing/power path)
- [ ] Flash tested on ESP32-C6
- [ ] Scope/logic-analyzer evidence attached
- [ ] Decoder behavior verified (idle, speed, direction, stop/e-stop)

## Test Evidence
- Key command outputs (paste concise logs):
  - `cargo test-host`:
  - `cargo check-esp`:
  - clippy host:
  - clippy riscv32:
