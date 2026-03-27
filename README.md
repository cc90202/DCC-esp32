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

- Generates DCC waveforms with the ESP32-C6 RMT peripheral
- Encodes NMRA DCC packets for locomotive control and service mode
- Continuously refreshes active locomotive state on the track
- Controls up to 12 active DCC locomotives/decoders
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
- `src/dcc/`: DCC packet, encoder, timing, scheduler, validator, CV logic
- `src/net/`: Z21-compatible network protocol and UDP control
- `docs/specs/`: protocol and standards references
- `docs/hardware/`: wiring and hardware usage notes

## Safety Notes

- Current output/control behavior must match the external amplifier and protection hardware.
- For protocol, power-stage, or GPIO wiring changes, re-check the hardware documentation before flashing.
- DCC track power can damage decoders or hardware if the power stage is wired incorrectly.

## License

MIT
