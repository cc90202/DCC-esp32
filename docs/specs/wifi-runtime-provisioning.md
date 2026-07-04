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

WiFi credentials are now loaded from the dedicated `dcc_cfg` flash partition
before station mode starts. The old `.env`/`env!("WIFI_*")` path has been
removed from the runtime and build script.

Provisioning AP and HTTP setup are still pending:

- missing credentials select provisioning mode and keep the normal Z21 runtime
  stopped;
- valid stored credentials start station mode;
- runtime GPIO21 provisioning requests first request track stop, then set a
  next-boot flag and reboot;
- the setup AP and HTML form are implemented in later tasks.

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
    fn load(&mut self) -> Result<Option<WifiCredentials>, StoreError>;
    fn save(&mut self, credentials: &WifiCredentials) -> Result<(), StoreError>;
    fn clear(&mut self) -> Result<(), StoreError>;
}
```

On embedded Rust this can be implemented as a concrete type with equivalent
methods if trait ergonomics or lifetime constraints become awkward. The
important part is the direction of dependency: WiFi policy should not know the
flash driver.

## Persistent Format

Use a small versioned record in a dedicated application-owned flash partition.

Selected format:

```text
magic: "DCCWIFI"
version: 1
generation
flags
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

### Storage Spike Decision

Decision for the first runtime provisioning release: use a dedicated versioned
blob in a small application-owned flash partition, not NVS.

Rationale:

- the current stack is `esp-hal`/`esp-rtos`/`esp-radio`, not ESP-IDF;
- `esp-radio` on ESP32-C6 currently needs local NVS symbol stubs in
  `src/net/mod.rs`, so treating NVS as an application persistence dependency
  would couple this feature to a storage path that is not actually initialized;
- the project already has `partitions.csv`, so a dedicated data partition is
  explicit and reviewable;
- blob encode/decode, checksum validation, corruption handling, and flag
  semantics can be host-tested without flash hardware;
- the concrete flash driver remains an outer detail and can be replaced later
  without changing WiFi provisioning policy.

Required partition change for the implementation task:

```text
# Name,     Type, SubType, Offset,   Size,     Flags
dcc_cfg,    data, 0x40,    0x1E0000, 0x3000,
```

To keep a 2 MiB flash layout valid, shrink the current factory app partition
from `0x1F0000` to `0x1D0000` and place `dcc_cfg` at `0x1E0000`. Do not store
application credentials in the existing `nvs` partition; leave that partition
available for platform/radio use.

Flash layout inside `dcc_cfg`:

- three 4 KiB erase sectors, with the first two used by the store;
- one fixed-size credential record per sector: slot A and slot B;
- each record contains a monotonically increasing generation counter;
- load chooses the newest valid record using wrapping generation comparison;
- save erases and writes only the inactive sector, then subsequent loads prefer
  the newer valid slot after checksum validation;
- clear erases both slots;
- missing means both slots are erased (`0xff`);
- corrupt means at least one slot contains non-empty bytes but no slot validates.

The third sector is reserved to keep the implementation on
`esp_bootloader_esp_idf::partitions::FlashRegion`: version `0.4.0` rejects an
erase whose `to` bound is exactly the partition end, while slot B erase ends at
`0x2000`.

Record shape:

```text
magic: "DCCWIFI"
version: 1
generation: u32
flags: u8
ssid_len: u8
password_len: u8
ssid bytes [32]
password bytes [64]
padding to 4-byte boundary
checksum: u32
```

Flags:

- bit 0: `force_provisioning_on_next_boot`;
- all other bits must be zero in version 1.

Checksum:

- use a small deterministic checksum implemented in the host-testable codec;
- checksum covers every record byte except the checksum field itself;
- checksum mismatch maps to corrupt config, never to missing config.

Reset strategy:

- after credentials are saved successfully from the setup flow, call
  `esp_hal::system::software_reset()`;
- the same API is already used by panic/fail-fast paths in `src/bin/main.rs`,
  so no new reset dependency is introduced;
- do not reset until the store reports a successful save.

Power-loss behavior:

- power loss during inactive-sector erase/write leaves the previous valid
  sector active;
- power loss after the inactive slot validates makes the newer generation
  active;
- if both slots are corrupt, boot enters provisioning due to
  `InvalidStoredCredentials`;
- this behavior needs HIL validation before relying on field updates.

Typed error surface:

```rust
pub enum StoreError {
    FlashRead,
    FlashErase,
    FlashWrite,
    Corrupt,
    MissingCredentials,
    BufferTooSmall,
}
```

The codec should distinguish `Missing` from `Corrupt`; the flash adapter should
map driver errors to read/erase/write variants without logging credentials.

## WiFi Module Changes

`src/net/wifi.rs` no longer uses compile-time credentials:

```rust
env!("WIFI_SSID")
env!("WIFI_PASS")
```

to runtime input:

```rust
pub(super) fn client_mode_config(credentials: &WifiCredentials) -> ModeConfig
```

The network task receives credentials from boot wiring rather than reading them
from environment variables.

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

### AP Mode Spike Decision

Use the SoftAP support already exposed by `esp-radio`; do not introduce a
custom WiFi/AP abstraction or raw radio setup.

Relevant upstream APIs in `esp-radio 0.17.0`:

```rust
esp_radio::wifi::ModeConfig::AccessPoint(AccessPointConfig)
```

`WifiController::set_config` maps `ModeConfig::AccessPoint` to the native
`WIFI_MODE_AP` mode and applies the AP configuration through `apply_ap_config`.
`esp_radio::wifi::Interfaces` already exposes both station and AP network
devices:

```rust
pub struct Interfaces<'d> {
    pub sta: WifiDevice<'d>,
    pub ap: WifiDevice<'d>,
}
```

Production provisioning mode should therefore use `interfaces.ap` with a
separate `embassy-net` stack runner. The station path should keep using
`interfaces.sta`.

Minimal configuration shape:

```rust
let ap_config = ModeConfig::AccessPoint(
    AccessPointConfig::default()
        .with_ssid("DCC-Setup-ABCD".into())
        .with_password("dcc-setup".into())
        .with_auth_method(AuthMethod::Wpa2Personal)
        .with_max_connections(1),
);
wifi_controller.set_config(&ap_config)?;
wifi_controller.start_async().await?;
```

Keep `max_connections` low for setup mode so the AP admits only the expected
single-phone provisioning flow.

AP IP address:

- bind the ESP32 AP interface to `192.168.4.1/24`;
- use `embassy_net::Config::ipv4_static`;
- no gateway is needed for the ESP32 setup page;
- DNS servers can remain empty for the first release.

Expected stack configuration:

```rust
let config = embassy_net::Config::ipv4_static(StaticConfigV4 {
    address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 4, 1), 24),
    gateway: None,
    dns_servers: heapless::Vec::new(),
});
```

DHCP decision:

- phone clients need automatic IP assignment for acceptable UX;
- `embassy-net`'s current `dhcpv4` feature is a DHCP client socket, not a DHCP
  server;
- `esp-radio` AP mode does not provide a DHCP server through `embassy-net`;
- the production AP task must either add a small no-std DHCP server or select a
  maintained crate compatible with the current stack;
- static phone IP is acceptable only as a temporary HIL blocker workaround, not
  as the product behavior.

HTTP dependency decision:

- the current `embassy-net` feature set is `dhcpv4`, `medium-ethernet`, `udp`;
- serving the setup page requires TCP sockets;
- the HTTP task must add the `tcp` feature before implementing the server;
- AP-only provisioning does not need Z21 UDP sockets, station reconnect logic,
  or normal network status events.

Composition and lifetime decision:

- `boot.rs` remains the composition root;
- station mode and provisioning mode are mutually exclusive boot branches;
- the provisioning branch owns `peripherals.WIFI`, starts AP + HTTP, and never
  starts Z21 or DCC runtime tasks;
- radio initialization should be factored so station and provisioning paths do
  not duplicate `esp_radio::init` statics;
- track output must stay disabled before and during AP/HTTP provisioning.

HIL checks required before production HTTP work:

- ESP32-C6 advertises `DCC-Setup-ABCD` with WPA2;
- a phone can associate with the AP;
- ESP32 binds `192.168.4.1/24` on `interfaces.ap`;
- DHCP strategy assigns a phone address without manual static configuration;
- added TCP/HTTP buffers fit RAM with display and control tasks disabled in
  provisioning mode.

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

After successful credential save, call `esp_hal::system::software_reset()` to
reboot. Do not show success or reset until the store reports that credentials
were written successfully.

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

- Implement `dcc_cfg` flash blob storage.
- Add encode/decode tests for stored record format.
- Treat corrupt config as `InvalidStoredCredentials`.
- Keep the previously valid slot active if save is interrupted during inactive
  sector erase/write.
- Prefer the newer generation only after the inactive slot validates.

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
