```
                               .=########%%####%@@@*:
                               +############%%@@@@@@%
              %@%               %@@%%@%@%=-*@@+@@@@@:                  .
           :#%@@@%##%%%%%%%##%%%@@@@@#%+*-:+@@@@@@@@%%%%@@@@@@@%%%%@@@@@@########%%#=.:.
          .@@@@@@@@@@@@@@@@@@@@@%@@@@%%#%%%%@@@@@@@@@@@@@@@@@@@@@@@@%@@@@%%%%%%%%%%%-=+*+
          .@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@-=@@%.
       ---:@@@@@@@@@@@@@@@@@@@@@@@@@@@@@**@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@--@@@.  -
       :  .%@@@@@@@@@@@@@@@@@@@@@@@@@@@%%#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@--@@@:  :
      ::  :%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@-::=@-  :-
      -==*:%@@@@@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@=::-@+  -=
      -#+*#-:..........-:.........................................:-:::::---====*%%***:-@@#*#=
    @+-@@@@#@@@@@@@@@%@@@@@@@@@@@@@%%@@@@%%###%@@@@@@@@@@@@@@@*#@@@@@@@@@@@@@@@*@%@@@%:=@@@@@@@@.
    + -#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@-=@@@@@#=.
     -*%@@@#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#*@@@@@@@@*.
   .=#@@@@@:.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.            @@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@      .*+
              ...%%@@@@@@-   ..*%#@@@@@%...              ..*##@@@@@*.:.. .##*@@@@@=....
#%#%##%%##%%#####%#%##%%%#%%###%#%#%##%%##%%#####%#%##%%##%%%####%#%##%%##%%#####%#%##%%##%%###%#%#%
###%#%%#%%%%%%#####%#%##%%%%%%#####%#%%#%%%%%#%####%#%##%%%%%%#####%#%##%%%%%%#####%#%%#%%%%%%#####%
+++++*++++++*++++++++*++++++*++++++++*++++++*++++++++*=+++++*++++++++*++++++*++++++++*++++++*+++++=+
```

# DCC-esp32

`no_std` Rust firmware for a DCC command station built on the ESP32-C6, using
`esp-hal`, Embassy async tasks and `esp-wifi`. Together with its companion
hardware it drives up to 12 locomotives at once, controlled from the Roco Z21
app over a Z21-compatible UDP layer.

## What this software does

The track waveform comes out of the RMT peripheral, with packet swaps handled
inside the interrupt while the preamble is still playing. That keeps the signal
free of inter-packet gaps and costs under 0.4% of the CPU, which is what makes
12 simultaneous decoders possible without the signal degrading.

Above that sit the NMRA packet encoder for both locomotive control and service
mode, a scheduler that keeps every active decoder refreshed with its speed,
direction and function state, emergency stop and fault handling, and the
Z21-compatible network layer the app talks to.

The station also listens to what the decoders answer. It opens the RailCom
cutout in the waveform, captures the reply with a comparator front end and a
UART, and uses it to recognise which locomotives are on the track and to read
and write their CVs while they run.

WiFi credentials are configured at runtime from a setup page the board serves
itself, so nothing is baked in at build time. Protocol, encoder and scheduler
logic is pure and covered by host-side tests; the rest runs on the target.

## Hardware documentation

Read this before wiring or powering anything:

- [Breadboard wiring](docs/hardware/wiring.md) — what is connected to what, and why
- [Components inventory](docs/hardware/components-inventory.md) — mounted, in the drawer, left the design, to buy

The bench circuit is built around a Pololu DRV8874 carrier (#4035) driving the
two rails, with the RailCom cutout obtained by braking the bridge: two AND
gates and two inverters combine the waveform with a run signal so that both
bridge inputs rise together during the cutout. The decoder's answer is picked
up as a few tens of millivolts across a sense resistor in series with each
rail, squared up by an LM339 comparator and read by a serial port on the
board.

The firmware drives the DCC signal path and the control logic, but safe operation depends on the
external hardware around it: the power stage, the protection circuitry, and how the signals are
routed. Those documents describe one specific breadboard, not a general recipe, so do not wire a
track from this README alone.

## Getting started

1. Verify the board has **8 MB physical flash** with `espflash board-info`
   (espflash **4.3.0**), then build and flash the firmware:

   ```bash
   cargo run --release
   ```

   **USB migration/recovery is destructive:** the runner writes the pinned
   bootloader/table and `ota_0`, erasing `otadata`, `ota_journal` and stale
   `ota_1`. After layout migration, reconfigure WiFi. For subsequent updates
   use the signed `.dccfw` workflow, not this runner; see [OTA guide](docs/ota.md).

2. Configure WiFi from the ESP32 setup page.

   On first boot, or whenever no valid WiFi credentials are stored, the ESP32
   starts safe WiFi setup mode instead of the normal command-station runtime.
   Track output, DCC, RailCom, and Z21 services remain disabled during setup.

   Connect a phone or computer to:

   - AP SSID: `DCC-Setup-XXXX`, where `XXXX` is derived from the ESP32 MAC
     suffix
   - AP password: `dcc-setup`
   - Setup URL: `http://192.168.4.1`

   The setup page asks for the station WiFi SSID and password. After a valid
   save, the ESP32 sends the success page, reboots, and starts station mode
   using the stored credentials.

3. To re-enter setup mode later, hold the blue Resume button on GPIO21 for at
   least 10 seconds, then release it. The board reboots and starts the setup
   access point. This also works when the saved home WiFi network is absent.

   Short GPIO21 presses still perform Resume. A press below the 10 second setup
   threshold does not enter WiFi setup. The red Stop button on GPIO22 remains
   Stop/E-stop and is unchanged by WiFi provisioning.

4. Once station mode is connected, the OLED display shows the station's IP
   address.

5. In the Roco Z21 app, go to settings and enter that IP address as the command station.

6. Select the locomotive address and drive.

Programming on the main track works: the station reads and writes CVs over
RailCom while the locomotive is running, verified on the bench with ESU and
ZIMO decoders. Service-mode programming on a separate track is not available,
because the hardware it needs (track relay and acknowledgement detection) has
not been built yet.

## Cargo aliases

Custom aliases are defined in `.cargo/config.toml` for common workflows:

| Alias | Description |
|-------|-------------|
| `cargo test-host` | Run host-side unit tests (protocol logic, fast feedback) |
| `cargo check-esp` | Type-check for ESP32-C6 target (no flash) |
| `cargo build-esp` | Build firmware for ESP32-C6 |
| `cargo build-esp-release` | Release build (LTO enabled) |
| `bash scripts/check-isr-ram.sh` | Verify RMT/cutout ISR symbols are linked in internal RAM |
| `cargo clippy-host` | Lint for host target |
| `cargo clippy-esp` | Lint for ESP32-C6 target |
| `cargo ota-pack` | Host-target OTA package tool: keygen/sign/inspect/verify/extract |
| `cargo run` | Destructive USB migration/recovery via espflash 4.3.0 and monitor |

## Cargo features

| Feature | Default | Description |
|---------|---------|-------------|
| `bench-diag` | off | Bench diagnostics: per-window and per-command `defmt` logging, plus the periodic RailCom counter dump task. Off by default because every `defmt` line is written inside a critical section, which delays the DCC waveform and RailCom cutout interrupts. The counters are always maintained; only their output is gated. |
| `ota-dev-key` | off | Bench only: embeds the development public key and dev build kind. |
| `ota-fault-inject` | off | Bench only: includes `ota-dev-key`, embeds fault build kind; compile-time `OTA_FAULT=none\|panic\|hang\|health-timeout`. |

Enable it for a bench session, then go back to the default build for normal use:

```bash
cargo run --release --features bench-diag   # verbose, bench only
cargo run --release                         # normal: boot, heartbeat, warnings and errors
```

## Build

```bash
cargo build-esp            # debug build
cargo build-esp-release    # release build (LTO, size-optimized)
```

## Flash

USB only, **destructive migration/recovery**, with verified 8 MB hardware:

```bash
cargo run                  # resets OTA history and clears inactive image
cargo run --release        # same destructive runner, release build
```

## Firmware updates over WiFi

From a trusted LAN, stop track power and open `http://<station-IP>/update`.
Upload a signed `.dccfw` with a strictly newer version (initial baseline:
0.1.0). Upload 100% is not flash verification or confirmation: wait for the
post-reboot result. Track power stays off until a fresh Resume/Z21 action.
Follow the [Italian OTA user/release guide and hardware checklist](docs/ota.md).
Host tests and CI are not hardware validation; OTA hardware tests remain required.

The signature authenticates the file, not the LAN operator. There is no TLS,
Secure Boot or universal startup rollback guarantee; some failures require USB.
Private signing seeds stay outside the repository and CI; see
[key custody and rotation](keys/README.md) and [pinned bootloader](bootloader/README.md).

## Test and validation

```bash
cargo test-host            # host-side unit tests
cargo check-esp            # fast embedded compile check
cargo clippy-host          # lint (host)
cargo clippy-esp           # lint (ESP32-C6, all features)
```

CI runs the same commands with `-D warnings`, and lints the ESP32-C6 target
both in the default configuration and with all features (bench/dev/fault,
**not a signing release**). It also tests/lints/formats the host OTA tool,
pins public-key fingerprints and bootloader hash, and validates a real
non-merged release image through a disposable-key package roundtrip.

## Commit conventions

Commit messages follow Conventional Commits and are validated with `cocogitto`.

- Format: `type(scope)!: short imperative summary`
- Reference: [CONTRIBUTING.md](CONTRIBUTING.md)
- Local check: `cog check origin/main..HEAD`
- Local hook install: `cog install-hook`

## Project layout

- `src/bin/main.rs`: firmware entrypoint
- `src/boot.rs` and `src/boot/`: composition root, hardware setup, readiness, and task spawning
- `src/application/`: framework-independent use cases, safety policies, and read projections
- `src/dcc/`: DCC domain types, packet encoding, timing, scheduling, and CV/POM actors
- `src/net/`: WiFi, provisioning, UDP transport, and Z21 interface adapters
- `src/railcom.rs` and `src/railcom/`: RailCom capture, parsing, attribution, and runtime dispatch
- `src/z21/`: pure Z21 wire protocol, independent of transport
- `src/fault_manager.rs` and `src/fault_manager/`: track-safety policy, power fence, and effect relay
- `src/track_*.rs`: physical output, authorization, and safety boundaries
- `docs/specs/`: protocol and standards references
- `docs/hardware/`: the breadboard as it is wired today, inventory and assembly notes

## Safety notes

- Current output/control behavior must match the external amplifier and protection hardware.
- For protocol, power-stage, or GPIO wiring changes, re-check the hardware documentation before flashing.
- DCC track power can damage decoders or hardware if the power stage is wired incorrectly.

## TODO

- [ ] Service-mode programming on a separate track. The packet encoder already
      builds the verify and write packets; what is missing is hardware.
- [ ] Z21 multi-client support (multiple apps controlling the same station)
- [ ] PCB design for a standalone command station board

## License

MIT
