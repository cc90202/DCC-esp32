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

A complete DCC command station with RailCom, written in Rust for the ESP32, together with the design files of its printed board ([`hardware/pcb/`](hardware/pcb/)).

`no_std` Rust firmware for a DCC command station built on the ESP32-C6, using
`esp-hal`, Embassy async tasks and `esp-radio`. Together with its companion
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

The firmware is updated over WiFi by uploading a signed file from the browser.
If the new version does not work, the station goes back to the previous one
on its own.

## Hardware documentation

Read this before wiring or powering anything:

- [Command-station board](hardware/pcb/) — KiCad project and fabrication files of the printed board (rev A)
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

1. Check with `espflash board-info` (espflash **4.3.0**) that the board has
   8 MB of flash. Then build the firmware and write it over USB:

   ```bash
   cargo run --release
   ```

   The first installation is done over USB. Later updates go over WiFi, as
   described in [Firmware updates over WiFi](#firmware-updates-over-wifi).

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

## Build and flash

| Command | What it does |
|---------|--------------|
| `cargo build-esp` | Debug build for the ESP32-C6 |
| `cargo build-esp-release` | Release build (LTO, size-optimized) |
| `cargo check-esp` | Type-check for the ESP32-C6, without building an image |
| `cargo test-host` | Run the unit tests on the PC |
| `cargo run --release` | Write the firmware over USB and open the serial monitor |

The aliases are defined in `.cargo/config.toml`. Writing over USB erases the
update history and the previous version kept in memory.

## Firmware updates over WiFi

The firmware is updated over WiFi, without a USB cable. The new firmware
comes in a signed file with the `.dccfw` extension.

Download the `.dccfw` file to a folder on the PC. Turn the track off from
Z21. Open `http://<station-address>/update` in the browser; the station
address is shown on the display. Upload the file from the web page.

Before writing the file to memory, the station checks its signature, hash,
chip, flash layout and project name. It also checks that the file's version
is newer than the installed one and has not been rejected before. If a check
fails, the station refuses the file and the page explains why. If every check
passes, the station writes the new firmware to a partition of the ESP32.

The station then runs the new firmware on trial for one minute. During the
trial the two LEDs alternate, the display counts down the seconds left, the
track stays off and Z21 cannot turn it on. If the new firmware was installed
correctly, the station confirms the new version and the display asks to turn
the track on from Z21. The update is then complete.

If during the trial the new firmware hangs, restarts on its own or fails a
check, the station goes back to the previous version. The new version is
marked as rejected and the station will not accept it again. If instead the
trial is cut short by a power loss, the new version is not rejected and can
be uploaded again.

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
- `src/dcc/`: scheduling, the waveform driver and the CV/POM actors; re-exports the protocol types from `dcc-protocol`
- `src/net/`: WiFi, provisioning, UDP transport, and Z21 interface adapters
- `src/railcom.rs` and `src/railcom/`: RailCom capture, parsing, attribution, and runtime dispatch
- `src/fault_manager.rs` and `src/fault_manager/`: track-safety policy, power fence, and effect relay
- `src/track_*.rs`: physical output, authorization, and safety boundaries
- `src/ota/`: firmware side of the WiFi update: flash, watchdog, track interlock and display
- `crates/dcc-protocol/`: DCC packets, their pulse encoding and timing, and the RailCom and logon data, with no hardware access
- `crates/z21/`: pure Z21 wire protocol, independent of transport
- `crates/dcc-provisioning/`: WiFi provisioning with no hardware access: setup page parser, DHCP replies, stored credentials and the boot decision
- `crates/dcc-ota/`: update logic with no hardware access: journal, trial, rollback and status page data
- `crates/dcc-ota-format/`: signed package and firmware image formats
- `docs/specs/`: protocol and standards references
- `docs/hardware/`: the breadboard as it is wired today, inventory and assembly notes
- `hardware/pcb/`: the printed board, KiCad project and fabrication files

## Safety notes

- The firmware is written for the power stage and the protection circuits described in the hardware documentation. With different hardware its behaviour is not guaranteed.
- For protocol, power-stage, or GPIO wiring changes, re-check the hardware documentation before flashing.
- DCC track power can damage decoders or hardware if the power stage is wired incorrectly.

## TODO

- [ ] Service-mode programming on a separate track. The packet encoder already
      builds the verify and write packets; what is missing is hardware.
- [ ] Z21 multi-client support (multiple apps controlling the same station)
- [ ] 3D-printed box for the board
- [ ] LocoNet integration

## License

MIT
