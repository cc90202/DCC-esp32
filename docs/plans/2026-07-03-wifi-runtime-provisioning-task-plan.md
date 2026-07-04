# WiFi Runtime Provisioning Task Plan

## Scope

Plan the implementation of runtime WiFi provisioning for the ESP32-C6 command
station. This is a task plan, not implementation.

Reference design:

- `docs/specs/wifi-runtime-provisioning.md`

Target stack:

- ESP32-C6
- `esp-hal`
- `esp-rtos`
- Embassy async tasks
- no ESP-IDF dependency unless a later storage spike proves it necessary

## Architecture Rules

Use Clean Architecture as a practical boundary guide:

- Pure policy and validation stay host-testable.
- `esp-radio`, HTTP, flash/NVS, GPIO, reboot, and display are details.
- `boot.rs` remains the composition root.
- Avoid abstract layers that do not buy testability or isolation.

Rust/ESP32 constraints:

- Invalid credential states should be hard to construct.
- Critical failures must return typed errors or emit explicit warnings.
- No silent drops for safety-relevant actions.
- Track output must stay disabled in provisioning mode.
- Task spawn failures remain explicit boot errors.
- Host tests validate policy and parsing.
- `cargo check-esp` validates embedded target integration.

## Task 0: Mainline Hygiene

Goal: start implementation work from the public main state.

Steps:

1. Create an implementation branch from `public/main`.
2. Confirm working tree is clean.
3. Run baseline checks:
   - `cargo test-host`
   - `cargo check-esp`
   - `cargo build-esp-release`

Acceptance:

- Branch is based on current `public/main`.
- Baseline checks pass before code changes.

## Task 1: Pure Credential Types And Validation

Goal: introduce host-testable WiFi credential policy without touching hardware.

Files likely involved:

- `src/net/wifi_config/mod.rs`
- `src/net/wifi_config/credentials.rs`
- `src/net/mod.rs`

Design:

```rust
pub struct WifiCredentials {
    ssid: heapless::String<32>,
    password: heapless::String<64>,
}
```

Rules:

- Constructor validates SSID and password.
- SSID is non-empty and <= 32 bytes.
- WPA/WPA2 password is 8..=63 bytes.
- Open networks are rejected in first version.
- Password is not exposed through `Debug` or logging helpers.

Tests:

- valid SSID/password accepted;
- empty SSID rejected;
- SSID > 32 bytes rejected;
- password < 8 bytes rejected;
- password > 63 bytes rejected;
- accepted password length 63;
- `Debug` output must not include the password.

Acceptance:

- `cargo test-host` passes.
- `cargo check-esp` passes.
- No dependency on `esp-radio`, flash, HTTP, or GPIO from this module.

## Task 2: Provisioning Decision Policy

Goal: decide station mode vs provisioning mode using pure inputs.

Files likely involved:

- `src/net/wifi_config/provisioning_policy.rs`

Design:

```rust
pub enum ProvisioningDecision {
    StationMode,
    ProvisioningMode(ProvisioningReason),
}

pub enum ProvisioningReason {
    MissingCredentials,
    ButtonOverride,
    InvalidStoredCredentials,
}
```

Inputs:

- stored credentials state;
- GPIO21 10 second override flag;
- stored config validity/corruption state.

Rules:

- button override wins;
- missing credentials enter provisioning;
- corrupt/invalid stored config enters provisioning;
- valid credentials enter station mode;
- WiFi connection failures after valid credentials are loaded do not erase
  credentials and do not automatically enter provisioning.

Tests:

- missing credentials -> provisioning;
- button override + valid credentials -> provisioning;
- invalid stored credentials -> provisioning;
- valid credentials -> station;
- connection failure is not represented as a provisioning decision.

Acceptance:

- Policy is host-testable and does not know about WiFi APIs.

## Task 3: GPIO21 Press Classification

Goal: separate Resume short/long actions from provisioning request.

Files likely involved:

- `src/control_logic.rs`
- `src/control_buttons.rs`

Design:

```rust
pub enum ResumePress {
    Short,
    Long,
    Provisioning,
}
```

Mapping:

```text
press < 2s       -> Short
press 2s..10s    -> Long
press >= 10s     -> Provisioning
```

Implementation constraints:

- Do not emit `ResumeLongPressed` at the 2 second mark.
- Classify on release, or when the 10 second provisioning threshold is reached.
- Red Stop/GPIO22 behavior must remain unchanged.
- GPIO21 remains active-low with pull-up.

Delivery design:

- Add a separate provisioning request channel.
- Do not put provisioning into `FaultEvent`.
- If runtime provisioning is requested, a coordinator must first put track output
  in a safe disabled state, then enter setup or reboot into setup.

Tests:

- duration < 2s maps to `Short`;
- duration exactly 2s maps to `Long`;
- duration 2s..10s maps to `Long`;
- duration exactly 10s maps to `Provisioning`;
- duration > 10s maps to `Provisioning`;
- red Stop behavior unchanged at unit-test boundary where possible.

Acceptance:

- Existing button tests still pass.
- New duration policy tests pass.

## Task 4: Storage And Reset Spike

Goal: choose the persistent credential backend and reset strategy before
production integration code.

Candidates:

1. NVS/key-value storage using the current ESP32-C6 stack.
2. Dedicated versioned blob in flash.

Decision criteria:

- works with `esp-hal`/`esp-rtos` stack without ESP-IDF coupling;
- supports overwrite;
- distinguishes missing vs corrupt config;
- can avoid logging secrets;
- can be tested at least at encode/decode level on host;
- failure behavior is explicit.

Spike output:

- chosen backend;
- final concrete store API shape;
- ownership model for flash/NVS resources;
- required partition/config changes, if any;
- erase/write constraints;
- typed error surface;
- reboot/power-loss corruption behavior;
- exact platform reset API after successful save;
- whether HIL validation is required before merge.

Acceptance:

- Decision recorded in `docs/specs/wifi-runtime-provisioning.md` or a short
  implementation note.
- No production provisioning code depends on an undecided storage backend.
- Store implementation and boot integration do not start until this task is
  complete.

## Task 5: Credential Store Implementation

Goal: implement persistent credential load/save/clear and the provisioning
reboot flag using the chosen backend.

Files likely involved:

- `src/net/wifi_config/store.rs`
- `src/net/wifi_config/flash_store.rs` or equivalent

Interface:

```rust
pub trait WifiCredentialsStore {
    fn load(&self) -> Result<Option<WifiCredentials>, StoreError>;
    fn save(&self, credentials: &WifiCredentials) -> Result<(), StoreError>;
    fn clear(&self) -> Result<(), StoreError>;
}
```

Runtime provisioning needs a persistent boot flag in the first implementation:

```rust
pub trait ProvisioningFlagStore {
    fn force_on_next_boot(&self) -> Result<bool, StoreError>;
    fn set_force_on_next_boot(&self) -> Result<(), StoreError>;
    fn clear_force_on_next_boot(&self) -> Result<(), StoreError>;
}
```

The flag may be implemented by the same concrete store as credentials. Avoid an
ad-hoc second flash path.

Embedded compromise:

- If trait lifetimes or static ownership become awkward, use a concrete store
  with equivalent methods.
- Keep the policy module independent from the concrete flash API.

Tests:

- host tests for blob encode/decode if blob backend is selected;
- corrupt magic rejected;
- corrupt checksum rejected;
- missing config returns `Ok(None)`;
- valid record round-trips without password logging.
- force-provisioning flag round-trips;
- flag clear is idempotent.
- interrupted save during inactive-sector erase/write leaves the old valid
  record selected;
- completed save selects the newer generation;
- both slots corrupt maps to `InvalidStoredCredentials`.

Acceptance:

- Store errors are typed.
- Corrupt config becomes `InvalidStoredCredentials`.
- `force_provisioning_on_next_boot` is loaded before station mode starts and
  cleared after provisioning mode is selected.
- `cargo test-host` and `cargo check-esp` pass.

## Task 6: Runtime WiFi Credentials Input

Goal: remove normal WiFi path dependency on `.env`.

Files likely involved:

- `src/net/wifi.rs`
- `src/net/udp_control.rs`
- `src/boot.rs`
- `build.rs` only if fallback policy changes

Change:

```rust
pub(super) fn client_mode_config(credentials: &WifiCredentials) -> ModeConfig
```

Rules:

- `wifi.rs` must not use `env!("WIFI_SSID")` or `env!("WIFI_PASS")` in the
  normal product path.
- `net_task` receives credentials from boot wiring.
- `.env` can remain only as a temporary developer fallback if explicitly wired
  outside the production decision path behind an explicit feature flag or
  debug-only path.

Tests:

- host tests for any conversion from `WifiCredentials` to config-safe strings;
- embedded check for station path.

Acceptance:

- `src/net/wifi.rs` station config has no direct `env!("WIFI_*")` usage.
- Any `.env` fallback is isolated behind a feature flag or debug-only adapter.
- Existing Z21 tests pass.
- `cargo check-esp` passes.

## Task 7: Boot Decision Integration

Goal: wire store + GPIO21 override + provisioning policy into boot.

Files likely involved:

- `src/boot.rs`
- `src/control_buttons.rs`
- `src/net/wifi_config/*`

Rules:

- Boot must not wait 10 seconds unless GPIO21 is already pressed.
- If GPIO21 is pressed at decision time, wait up to 10 seconds for override.
- If released before 10 seconds, continue normal boot.
- Missing/corrupt credentials enter provisioning.
- Valid credentials start station mode unless `force_provisioning_on_next_boot`
  is set.
- Valid credentials plus unreachable WiFi retry station mode; no automatic erase.
- If `force_provisioning_on_next_boot` is set, clear the flag and enter
  provisioning.

Safety:

- In provisioning path, keep track output disabled.
- Do not spawn normal Z21 networking in provisioning path.
- Do not emit DCC packets in provisioning path.

Acceptance:

- Boot task spawn failure behavior remains explicit.
- Existing runtime readiness logic remains understandable.
- `cargo check-esp` passes.

## Task 8: AP Mode Spike

Goal: prove the radio/network shape for setup AP before implementing the web
flow.

Questions to answer:

- How does `esp-radio` configure AP mode on the current stack?
- Can the AP use static IP `192.168.4.1`?
- Is a DHCP server available/needed for phone clients?
- Does AP mode require different `embassy-net` resources than station mode?
- Can AP mode and HTTP sockets run without normal Z21 networking?
- What ownership/lifetime changes are required in `boot.rs` and `net` modules?

Spike output:

- AP configuration API and minimal code shape;
- IP/DHCP decision;
- required feature flags or dependencies;
- memory/static buffer requirements;
- risks requiring HIL validation.

Acceptance:

- Hardware can see and join a test AP from phone, or the spike records the
  precise blocker.
- Decision is recorded before production AP task starts.

## Task 9: Provisioning AP Mode

Goal: start ESP32 AP mode for setup.

Files likely involved:

- `src/net/provisioning/mod.rs`
- `src/net/provisioning/ap.rs`
- `src/net/wifi.rs` or shared radio setup helpers

AP behavior:

```text
SSID: DCC-Setup-ABCD
PASS: dcc-setup
URL:  http://192.168.4.1
```

Rules:

- WPA2 AP, not open.
- `ABCD` derived from stable MAC/chip suffix when practical.
- Serial log may print AP SSID and setup URL.
- Serial log must not print any password, including the fixed AP password and
  submitted station password.

Acceptance:

- Hardware can see and join the AP from phone.
- AP mode does not start station/Z21 runtime.
- Track output remains disabled.
- AP provides or documents the IP assignment path needed to reach
  `http://192.168.4.1`.

## Task 10: HTTP Implementation Spike

Goal: choose the smallest robust HTTP implementation for the setup form.

Options:

1. Minimal hand-written HTTP/1.1 handler over embedded sockets.
2. A no_std-compatible HTTP crate if it is demonstrably small and compatible.

Preferred first version:

- minimal hand-written handler;
- support only `GET /` and `POST /save`;
- no chunked encoding;
- no keep-alive requirement;
- host-tested request/form parser.

Spike output:

- chosen HTTP approach;
- dependency impact, if any;
- parser boundaries;
- max request/header/body sizes;
- behavior for malformed requests.

Acceptance:

- Decision recorded before production HTTP task starts.

## Task 11: Embedded HTTP Setup Server

Goal: serve the setup page and accept submitted credentials.

Files likely involved:

- `src/net/provisioning/http.rs`
- `src/net/provisioning/html.rs`

Routes:

```text
GET  /      -> setup page
POST /save  -> validate and save credentials
```

Request constraints:

- `POST /save` uses `application/x-www-form-urlencoded`;
- max body length 256 bytes;
- accepted fields: `ssid`, `password`;
- duplicate `ssid` or `password` rejected;
- unknown fields ignored;
- unsupported content type rejected;
- invalid credentials show an error page and do not save.

UI constraints:

- no CDN;
- no frontend framework;
- mobile-first;
- title: `Configura WiFi DCC`;
- show/hide password toggle may use small inline JS;
- success page clearly says credentials were saved and device is rebooting.

Tests:

- host-test form parser if parser can be isolated;
- invalid form cases;
- duplicate fields rejected;
- oversized body rejected.
- URL form decoding handles `+` as space;
- URL form decoding handles `%20`;
- invalid percent escapes are rejected;
- invalid UTF-8 or unsupported bytes are rejected.

Acceptance:

- Phone can load `http://192.168.4.1`;
- valid submit saves credentials;
- invalid submit does not save;
- password is never logged.

## Task 11.5: Provisioning DHCP Server

Goal: assign an IP address automatically to a phone/laptop connected to the
setup AP.

Files likely involved:

- `src/net/provisioning/dhcp.rs`
- `src/net/provisioning_dhcp.rs`
- `src/net/provisioning/ap.rs`

Behavior:

- listen on UDP port 67 in provisioning mode only;
- answer DHCP Discover with Offer;
- answer DHCP Request for this server/address with Ack;
- lease only `192.168.4.2/24`;
- advertise router/server `192.168.4.1`;
- ignore unsupported DHCP messages instead of panicking.

Implementation constraints:

- use existing `smoltcp` DHCP wire packet/repr primitives;
- keep parser/encoder host-testable;
- do not start DHCP in station mode;
- no captive portal/DNS in this task.

Acceptance:

- host tests cover Discover -> Offer and Request -> Ack;
- phone can get an address without manual static IP setup;
- provisioning still never starts DCC, RailCom, Z21, or track output.

## Task 12: Save, Reboot, And Recovery

Goal: complete provisioning transaction safely.

Behavior:

1. Validate form.
2. Save credentials.
3. Return confirmation.
4. Reboot through the reset API selected in Task 4.

Failure behavior:

- Save failure returns an error page.
- Save failure does not claim success.
- Corrupt next boot enters provisioning as `InvalidStoredCredentials`.

Acceptance:

- Manual hardware test proves credentials survive reboot.
- Manual hardware test proves bad/corrupt config does not panic boot.
- Runtime GPIO21 provisioning request sets the boot flag, disables track output,
  and reboots rather than trying to live-transition the active runtime.

## Task 13: Operator Feedback

Goal: make setup discoverable without serial logs only.

Files likely involved:

- `src/display.rs`
- `src/system_status.rs`
- `src/status_led.rs`
- `src/boot.rs`

First version:

- OLED shows setup mode, AP SSID, `http://192.168.4.1`, and track-disabled
  state when display is available.
- Status LEDs may reuse WiFi connecting pattern.
- Serial logs show AP SSID and setup URL.

Non-goal:

- dedicated LED pattern unless it is low-cost and does not disrupt existing
  fault/estop semantics.

Acceptance:

- User can discover AP SSID, URL, and track-disabled state from OLED or serial
  log.
- Fault/estop LED semantics remain unchanged outside provisioning.

## Task 14: Documentation And UAT

Goal: document setup and validate on hardware.

Files likely involved:

- `README.md`
- `docs/hardware/main-dcc-checklist.md`
- `docs/specs/wifi-runtime-provisioning.md`

Docs:

- remove `.env` as primary user setup path;
- document AP SSID format;
- document AP password `dcc-setup`;
- document `http://192.168.4.1`;
- document GPIO21 10 second setup entry;
- document GPIO22 unchanged as Stop/E-stop.

UAT:

- flash firmware with no stored credentials;
- verify AP appears;
- connect phone to AP;
- open setup page;
- submit invalid SSID/password and see error;
- submit valid credentials;
- reboot and connect to configured WiFi;
- power cycle and verify credentials persist;
- make configured WiFi unavailable and verify retry/no erase;
- hold GPIO21 for >= 10 seconds and verify setup mode;
- verify GPIO21 short and long resume below 10 seconds;
- verify GPIO22 stop unchanged.

Final checks:

- `cargo test-host`
- `cargo check-esp`
- `cargo build-esp-release`

## Suggested Implementation Order

1. Task 0: Mainline hygiene.
2. Task 1: Credential types and validation.
3. Task 2: Provisioning decision policy.
4. Task 3: GPIO21 classification.
5. Task 4: Storage and reset spike.
6. Task 5: Credential store.
7. Task 6: Runtime WiFi credential input.
8. Task 7: Boot decision integration.
9. Task 8: AP mode spike.
10. Task 9: Provisioning AP.
11. Task 10: HTTP implementation spike.
12. Task 11: HTTP setup server.
13. Task 11.5: Provisioning DHCP server.
14. Task 12: Save/reboot/recovery.
15. Task 13: Operator feedback.
16. Task 14: Docs and UAT.

## Commit Strategy

Keep commits reviewable:

1. `feat(wifi): add credential validation policy`
2. `feat(input): classify resume provisioning press`
3. `feat(wifi): add persistent credential store`
4. `refactor(wifi): pass runtime credentials to station mode`
5. `feat(wifi): add provisioning boot decision`
6. `feat(wifi): add provisioning AP`
7. `feat(wifi): add provisioning web form`
8. `docs(wifi): document runtime provisioning`

Do not mix hardware AP work with pure policy tests in the same commit.

## Open Decisions Before Coding

- Persistent backend: dedicated versioned blob in a `dcc_cfg` flash partition.
  Do not use ESP-IDF/NVS for the first release.
- Exact platform reset API after successful save:
  `esp_hal::system::software_reset()`.
- AP IP/DHCP shape for phone clients.
- HTTP implementation approach and parser limits.
- Whether `.env` remains as a dev-only fallback after first release.
