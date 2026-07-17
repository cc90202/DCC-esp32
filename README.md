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

WiFi credentials are configured at runtime from a setup page the board serves
itself, so nothing is baked in at build time. Protocol, encoder and scheduler
logic is pure and covered by host-side tests; the rest runs on the target.

## Hardware documentation

Read this before wiring or powering anything:

- [Components inventory](docs/hardware/components-inventory.md)

Two track drivers have been used on the bench. The BTS7960 is the
higher-current H-bridge, and the older breadboard and RailCom notes still refer
to it. The Pololu DRV8874 carrier (#4035) is the compact option used for the
current bring-up and short-recovery testing.

The firmware drives the DCC signal path and the control logic, but safe operation depends on the
external hardware around it: the power stage, the protection circuitry, and how the signals are
routed. Detailed wiring notes for the two drivers are kept outside this repository for now, so do
not wire a track from this README alone.

## Getting started

1. Build and flash the firmware:

   ```bash
   cargo run --release
   ```

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

Build-time `.env` credentials have been removed entirely: WiFi credentials are
configured from the setup page and stored in flash.

CV programming is still being worked on. The RailCom and programming-on-main
read and write paths are being integrated, but the programming-track hardware
support is not finished.

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
| `cargo run` | Flash to device via espflash and monitor |

## Build

```bash
cargo build-esp            # debug build
cargo build-esp-release    # release build (LTO, size-optimized)
```

## Flash

```bash
cargo run                  # flash and monitor via espflash
cargo run --release        # flash release build
```

## Test and validation

```bash
cargo test-host            # host-side unit tests
cargo check-esp            # fast embedded compile check
cargo clippy-host          # lint (host)
cargo clippy-esp           # lint (ESP32-C6)
```

## Commit conventions

Commit messages follow Conventional Commits and are validated with `cocogitto`.

- Format: `type(scope)!: short imperative summary`
- Reference: [CONTRIBUTING.md](CONTRIBUTING.md)
- Local check: `cog check origin/main..HEAD`
- Local hook install: `cog install-hook`

## Project layout

- `src/bin/main.rs`: firmware entrypoint
- `src/dcc/`: DCC packet, encoder, timing, scheduler, validator, CV logic, ISR-driven RMT backend
- `src/net/`: Z21-compatible network protocol, UDP control, WiFi, and provisioning
- `src/railcom/`: RailCom capture, parsing, runtime dispatch, and POM integration
- `docs/specs/`: protocol and standards references
- `docs/hardware/`: component inventory and hardware notes

## Safety notes

- Current output/control behavior must match the external amplifier and protection hardware.
- For protocol, power-stage, or GPIO wiring changes, re-check the hardware documentation before flashing.
- DCC track power can damage decoders or hardware if the power stage is wired incorrectly.

## TODO

- [ ] CV programming hardware integration (prog track relay, ACK detection circuit)
- [ ] Z21 multi-client support (multiple apps controlling the same station)
- [ ] RailCom bi-directional communication (hardware + firmware)
- [ ] PCB design for a standalone command station board

## License

MIT
