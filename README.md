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

# DCC-ESP32

Rust firmware for an **ESP32-C6 based DCC command station** for model railroading.

Together, this software and its companion hardware implement a complete DCC command station
that can control **up to 12 active DCC locomotives/decoders at the same time**.
The system is operated through the **Roco Z21 app**, using a Z21-compatible UDP control layer.

The firmware generates NMRA-compliant DCC packets, schedules continuous track commands,
handles emergency-stop and fault logic, and exposes network control through a Z21-compatible
interface. The project is designed as command-station firmware, not just a waveform demo:
it includes packet encoding, scheduler logic, service-mode CV support, safety state handling,
and runtime status/reporting.

## What This Software Does

- Generates gap-free DCC waveforms via ISR-driven RMT (interrupt swaps packets during preamble playback, CPU overhead <0.4%)
- Encodes NMRA DCC packets for locomotive control and service mode
- Continuously refreshes active locomotive state on the track
- Controls up to 12 active DCC locomotives/decoders simultaneously without signal degradation
- Supports speed, direction, functions, emergency stop, and fault handling
- Exposes control through the Roco Z21 app over a Z21-compatible UDP protocol layer
- Targets real hardware with host-side tests for pure logic

## Hardware Documentation

Read these first before wiring or powering anything:

- [Breadboard Wiring Guide](docs/hardware/breadboard-wiring-guide.md)

Important:

- The firmware drives the DCC signal path and control logic, but safe operation depends on the
  external hardware around it.
- Wiring, power stage, protection, and signal routing are documented in the hardware files above.
- Do not wire directly from the README alone. Use the hardware docs as the source of truth.

## Getting Started

1. Copy `.env.example` to `.env` and fill in your WiFi credentials:

   ```bash
   cp .env.example .env
   ```

   Edit `.env` with your network name and password.

2. Build and flash the firmware:

   ```bash
   cargo run --release
   ```

3. Once booted, the OLED display shows the station's IP address.

4. In the **Roco Z21 app**, go to settings and enter that IP address as the command station.

5. Select the locomotive address and drive.

> **Note:** CV programming is not yet available.
> Locomotives can only be controlled at their factory default address until
> the programming track hardware is integrated.

## Cargo Aliases

Custom aliases are defined in `.cargo/config.toml` for common workflows:

| Alias | Description |
|-------|-------------|
| `cargo test-host` | Run host-side unit tests (protocol logic, fast feedback) |
| `cargo check-esp` | Type-check for ESP32-C6 target (no flash) |
| `cargo build-esp` | Build firmware for ESP32-C6 |
| `cargo build-esp-release` | Release build (LTO enabled) |
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

## Test and Validation

```bash
cargo test-host            # host-side unit tests
cargo check-esp            # fast embedded compile check
cargo clippy-host          # lint (host)
cargo clippy-esp           # lint (ESP32-C6)
```

## Project Layout

- `src/bin/main.rs`: firmware entrypoint
- `src/dcc/`: DCC packet, encoder, timing, scheduler, validator, CV logic, ISR-driven RMT backend
- `src/net/`: Z21-compatible network protocol and UDP control
- `docs/specs/`: protocol and standards references
- `docs/hardware/`: wiring and hardware usage notes

## Safety Notes

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
