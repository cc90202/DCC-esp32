# Components Inventory

Hardware components available for the DCC Command Station project.

## Core Modules (in use on breadboard / planned for PCB)

| Component | Qty | Notes |
|---|---|---|
| Waveshare ESP32-C6 Mini | 1 | Main MCU, RISC-V, WiFi 6 |
| 74HC14 Schmitt Trigger (DIP-14) | 1 | DCC signal inversion for H-bridge |
| BTS7960 43A H-Bridge (Binghe) | 1 | Track power driver, R_IS/L_IS current sense |
| OLED SSD1306 1.3" I2C (128x64) | 1 | Main display, GPIO19=SDA, GPIO20=SCL |

## Additional Components (available, not yet allocated)

### Active Components

| Component | Qty | Package | Key Specs | Use |
|---|---|---|---|---|
| TLV3501AIDR comparator | 2 | SOIC-8 on ProtoAdvantage PA0002 (DIP-8 breakout) | Push-pull output, 4.5ns rise/fall, active-HIGH SHDN | RailCom bidirectional detection (one per rail, diode-OR combined) |
| LM393N dual comparator | 2 | DIP-8 + socket | Open-collector output, ~1.3µs rise/fall | Interim RailCom comparator (to be replaced by TLV3501) |
| IRLZ44N N-channel MOSFET | 10 | TO-220AB | 55V, 47A, logic-level gate (Vgs_th ~1-2V) | RailCom v2 cutout switches (Qc1, Qc2) |
| 1N5819 Schottky diode | 100 | DO-41 | 1A, 40V, Vf ~0.6V | General-purpose protection/clamping, flyback |

### Needed For RailCom Option A (not yet in inventory)

| Component | Qty | Package | Key Specs | Use |
|---|---|---|---|---|
| 74HC08 / 74AHC08 / 74LVC08 quad AND gate | 1 | DIP-14 preferred for breadboard | 3.3 V logic, fast enough for 1 MHz-class DCC edges | Derive DRV8874 PWM-mode `IN1 = DCC_RUN & DCC`, `IN2 = DCC_RUN & !DCC` |
| BAT54 / BAT54S small-signal Schottky diode | 4+ | through-hole adapter or SMD breakout | low capacitance preferred | Comparator input clamps for the cutout-only RailCom detector |
| 2.7 Ω resistor, >= 0.5 W | 2 | through-hole | 1 W preferred for bench margin | RailCom cutout-only burden resistors |

### Passive Components

| Component | Qty | Notes |
|---|---|---|
| Resistor kit 1Ω-1MΩ (1/2W, 1%) | 1000 pcs (25 values) | Metal film, through-hole |

### Displays

| Component | Qty | Notes |
|---|---|---|
| OLED 0.96" I2C (128x64, blue/yellow) | 3 | SSD1306, spare/secondary displays |

### Indicators

| Component | Qty | Notes |
|---|---|---|
| Green status LED | 1 | Reused from old main DCC breadboard, GPIO14 |
| Red status LED | 1 | Reused from old main DCC breadboard, GPIO15 |

### Connectors & Mechanical

| Component | Qty | Notes |
|---|---|---|
| WAGO 221-2411 lever connectors | 10 | 2-conductor, 4mm², quick-connect |

## PCB Components (JLCPCB PCBA, not yet ordered)

See [PCB design spec](../superpowers/specs/2026-03-22-pcb-command-station-design.md) for full BOM.

Key additions on PCB: MP1584EN buck converter (15V→5V), fuse holder, SMD passives, LED indicators, tactile buttons, screw terminals, test points.
