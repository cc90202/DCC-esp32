# WiFi Runtime Provisioning Design

## Goal

Replace build-time WiFi credentials from `.env` with runtime provisioning on
the ESP32-C6.

The command station must:

- use WiFi credentials stored in persistent flash;
- enter setup automatically when no credentials are stored;
- enter setup manually when the blue Resume button on GPIO21 is held for at
  least 10 seconds;
- expose a simple mobile-friendly web page from the ESP32 during setup;
- save the submitted credentials and use them on later boots.

The first implementation should favor predictable embedded behavior over a
fully featured captive portal.

## Current State

WiFi credentials are compiled into the firmware:

- `build.rs` reads `.env`;
- `src/net/wifi.rs` uses `env!("WIFI_SSID")` and `env!("WIFI_PASS")`;
- WiFi client mode is configured before the network task starts.

The physical control buttons are already wired and used:

- red Stop button: GPIO22, active-low;
- blue Resume button: GPIO21, active-low;
- both use internal pull-ups and debounce logic in `control_buttons`.

GPIO0/BOOT is not part of this design.

## User Flow

### Normal Boot

1. Firmware reads stored WiFi credentials from flash.
2. If credentials exist and the blue button override is not active, it starts
   station mode using those credentials.
3. Existing Z21 UDP and runtime services start after WiFi is configured.

If credentials exist but the configured WiFi network is unavailable, the
firmware must keep retrying station mode and must not erase the stored
credentials automatically. The operator can enter setup manually with the
GPIO21 10 second press if the credentials need to be replaced.

### First Boot Or Missing Credentials

1. Firmware finds no valid stored credentials.
2. It enters WiFi setup mode.
3. Track power remains disabled.
4. ESP32 starts an access point, for example `DCC-Setup-ABCD`.
5. User connects from phone or laptop.
6. User opens `http://192.168.4.1`.
7. The page asks for SSID and password.
8. Firmware validates and saves credentials.
9. Firmware reboots.
10. Next boot starts station mode with the saved credentials.

### Manual Setup

The blue Resume button keeps its runtime behavior, with a longer threshold for
provisioning:

```text
GPIO21 blue / Resume
  press < 2s       -> ResumeShortPressed
  press 2s..10s    -> ResumeLongPressed
  press >= 10s     -> WiFi provisioning mode

GPIO22 red / Stop
  press            -> StopPressed
```

The Resume task must not emit `ResumeLongPressed` as soon as 2 seconds elapse.
It should classify the press on release, or on the 10 second timeout, so that a
10 second setup request does not also trigger the 2 second resume action.

Runtime provisioning requests are allowed. When GPIO21 reaches the 10 second
threshold, the firmware should transition to provisioning mode only after
putting the track in a safe disabled state. If this transition is too invasive
for the first implementation, the fallback is to set a persistent
`force_provisioning_on_next_boot` flag and reboot.

## Architecture

Use Clean Architecture principles pragmatically. This is bare-metal firmware,
so the goal is not many abstract layers. The useful boundary is:

- pure policy and validation should not depend on `esp-radio`, HTTP, HTML, or
  flash APIs;
- hardware and protocol details should stay in outer modules;
- the boot composition root wires the concrete pieces together.

## Core Types

Introduce a small pure module for WiFi configuration policy, for example:

```text
src/net/wifi_config/
  mod.rs
  credentials.rs
  provisioning_policy.rs
```

Core data:

```rust
pub struct WifiCredentials {
    ssid: heapless::String<32>,
    password: heapless::String<64>,
}

impl WifiCredentials {
    pub fn new(ssid: &str, password: &str) -> Result<Self, CredentialError>;
    pub fn ssid(&self) -> &str;
    pub fn password(&self) -> &str;
}
```

Validation rules:

- SSID must not be empty;
- SSID must be at most 32 bytes;
- password must be empty only if open networks are explicitly supported;
- WPA/WPA2 password must be 8 to 63 bytes;
- password must never be logged.

Initial version should reject open networks unless there is a clear hardware
test need for them.

`password` uses capacity 64 so the type has headroom, but accepted WPA/WPA2
password length is still 8..=63 bytes.

## Provisioning Decision

The boot decision should be testable on the host:

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

- result of loading stored credentials;
- whether GPIO21 was held for at least 10 seconds;
- whether the stored blob validates.

Rules:

- button override wins;
- missing credentials enter provisioning;
- invalid stored credentials enter provisioning;
- otherwise start station mode.

Connection failures after valid credentials are loaded are not a provisioning
decision. They remain a station-mode reconnect problem until the operator
explicitly requests setup or the stored config is found corrupt.

## Storage Port

Define the storage boundary near the policy:

```rust
pub trait WifiCredentialsStore {
    fn load(&self) -> Result<Option<WifiCredentials>, StoreError>;
    fn save(&self, credentials: &WifiCredentials) -> Result<(), StoreError>;
    fn clear(&self) -> Result<(), StoreError>;
}
```

On embedded Rust this can be implemented as a concrete type with equivalent
methods if trait ergonomics or lifetime constraints become awkward. The
important part is the direction of dependency: WiFi policy should not know the
flash driver.

## Persistent Format

Use a small versioned record in flash or NVS.

Preferred if practical: NVS/key-value storage.

Fallback: a dedicated blob with:

```text
magic: "DCCWIFI1"
version: 1
ssid_len
password_len
ssid bytes
password bytes
checksum
```

Requirements:

- distinguish missing config from corrupt config;
- reject partial/corrupt writes;
- allow overwriting credentials;
- allow clearing credentials later;
- avoid logging secret contents.

Before implementation, run a short storage spike and pick exactly one storage
backend for the first release. If NVS is straightforward on the current
ESP32-C6 stack, use NVS. If it is not, use the versioned blob format above and
keep the encode/decode logic host-testable.

## WiFi Module Changes

Change `src/net/wifi.rs` from compile-time credentials:

```rust
env!("WIFI_SSID")
env!("WIFI_PASS")
```

to runtime input:

```rust
pub(super) fn client_mode_config(credentials: &WifiCredentials) -> ModeConfig
```

The network task should receive credentials from boot wiring rather than read
them from environment variables.

`.env` can remain temporarily as a developer fallback, but it should not be the
normal product path once provisioning storage works.

## Provisioning Mode

Provisioning mode should be exclusive in the first implementation.

When active:

- do not start normal Z21 networking;
- keep track power disabled;
- start WiFi AP mode;
- start HTTP server;
- save credentials;
- reboot after successful save.

This is safer and simpler than trying to run setup mode while the DCC runtime
is active.

Operator feedback:

- OLED, when available, should show setup mode, AP SSID, and
  `http://192.168.4.1`;
- serial logs may show AP SSID and AP password;
- serial logs must never show the submitted WiFi password;
- status LEDs may keep the existing WiFi-connecting indication unless a
  dedicated setup pattern is added later.

## Access Point

Initial AP:

```text
SSID: DCC-Setup-ABCD
URL:  http://192.168.4.1
```

`ABCD` should be derived from a stable chip identifier or MAC suffix when easy.

AP password:

- first version uses WPA2 with fixed password `dcc-setup`;
- do not use an open AP;
- document the password in README and, if useful, show it on the serial log.

## HTTP Interface

Initial routes:

```text
GET  /      -> setup page
POST /save  -> validate and save credentials
```

Optional later route:

```text
GET /status -> JSON status for a richer UI
```

The HTML should be embedded in firmware. It must not depend on CDN assets or
Internet access because the phone is connected to the ESP32 setup AP.

`POST /save` uses `application/x-www-form-urlencoded`.

Request limits:

- reject unsupported content types;
- reject oversized request bodies before parsing;
- first implementation max body length: 256 bytes;
- accepted fields: `ssid`, `password`;
- ignore unknown fields;
- reject duplicate `ssid` or `password` fields.

UI guidelines:

- mobile-first layout;
- clear title: `Configura WiFi DCC`;
- SSID input;
- password input with show/hide toggle;
- primary action: `Salva e riavvia`;
- visible error messages for invalid SSID/password;
- confirmation page after successful save.

Keep it visually polished but simple. No frontend framework.

## Captive Portal

Do not include captive portal in the first implementation.

First release:

- user connects to `DCC-Setup-ABCD`;
- user opens `http://192.168.4.1`.

Future release:

- DNS server replies with `192.168.4.1` for all names;
- common Android/iOS captive portal probes redirect to setup page.

Captive portal is useful but adds DNS behavior and mobile OS edge cases. It is
not required to validate the main provisioning path.

## Boot Integration

High-level boot flow:

```text
boot
  init minimal runtime
  if GPIO21 is already pressed, wait up to 10s for provisioning override
  load WiFi credentials
  decide StationMode vs ProvisioningMode

  if ProvisioningMode:
      keep track output disabled
      start AP + HTTP setup
      save credentials
      reboot

  if StationMode:
      start normal WiFi station mode
      start Z21 and runtime tasks
```

The exact placement in `boot.rs` should minimize churn. The boot module can
remain the composition root.

Boot must not add a 10 second delay when GPIO21 is not pressed. It should only
wait for the 10 second override window if GPIO21 is already active-low at the
time the provisioning decision is made.

After successful credential save, use the platform reset API to reboot. If no
safe reset API is available during the first implementation, show a saved
confirmation page and ask the operator to power-cycle manually.

## Button Integration

`control_buttons` should classify GPIO21 press duration into a richer enum:

```rust
pub enum ResumePress {
    Short,
    Long,
    Provisioning,
}
```

Mapping:

```text
Short        < 2s
Long         2s..10s
Provisioning >= 10s
```

Runtime delivery options:

1. Add a provisioning request sender to `resume_button_task`.
2. Emit a generic control event and let a coordinator dispatch to fault manager
   or provisioning.

Recommended first implementation: add a separate provisioning channel. Do not
put provisioning into `FaultEvent`; it is not a fault-manager responsibility.

If provisioning is requested while normal runtime is active, the receiver of
that channel must first disable track output and pause DCC scheduling before
starting setup mode or rebooting into setup mode.

## Tests

Host tests:

- credential validation accepts valid WPA2 credentials;
- credential validation rejects empty SSID;
- credential validation rejects too long SSID;
- credential validation rejects invalid password lengths;
- provisioning decision chooses setup for missing credentials;
- provisioning decision chooses setup for button override;
- provisioning decision chooses station mode for valid credentials;
- button duration maps `<2s`, `2s..10s`, `>=10s` correctly.

Embedded/manual tests:

- flash without credentials enters AP setup;
- phone connects to `DCC-Setup-ABCD`;
- `http://192.168.4.1` serves the setup page;
- invalid form shows an error and does not save;
- valid form saves and reboots;
- next boot connects to configured WiFi;
- valid stored credentials with WiFi unavailable retry station mode and do not
  erase credentials;
- holding GPIO21 for at least 10 seconds re-enters setup;
- boot without GPIO21 pressed does not wait 10 seconds;
- red Stop/GPIO22 behavior is unchanged;
- blue short and long Resume behavior is unchanged below 10 seconds.

## Security And Safety

Safety:

- provisioning mode must keep track power disabled;
- setup should not emit DCC packets;
- failed WiFi connection should not energize the track by itself.

Security:

- never log WiFi password;
- do not expose provisioning page in normal station mode;
- do not run AP indefinitely when station mode is working;
- use physical button or missing credentials as the setup gate.

First implementation does not need HTTPS. The setup AP is local, temporary,
and physically gated. This is acceptable for the project scope.

## Implementation Phases

### Phase 1: Policy And Button Classification

- Add `WifiCredentials`.
- Add validation.
- Add provisioning decision logic.
- Add GPIO21 duration classification tests.

### Phase 2: Runtime Credentials Input

- Change `wifi::client_mode_config` to accept `WifiCredentials`.
- Remove direct `env!` usage from normal WiFi path.
- Keep `.env` only as temporary dev fallback if needed.

### Phase 3: Persistent Store

- Implement flash/NVS storage.
- Add encode/decode tests for stored record format if using blob storage.
- Treat corrupt config as `InvalidStoredCredentials`.

### Phase 4: Provisioning AP And HTTP

- Start AP mode.
- Serve embedded HTML.
- Validate form submission.
- Save credentials.
- Reboot.

### Phase 5: Documentation And UAT

- Update README setup instructions.
- Update hardware docs for GPIO21 10 second setup action.
- Run host tests and `cargo check-esp`.
- Perform hardware provisioning test from phone.

## Explicit Non-Goals For First Version

- captive portal DNS;
- WiFi network scanning;
- admin password management;
- OTA firmware upload;
- editing credentials while normal Z21 runtime is active;
- HTTPS.

These can be added after the basic setup path is stable on hardware.
