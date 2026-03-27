# Breadboard Wiring Guide — DCC Command Station

Last updated: 2026-03-26

## Components

| # | Component | Notes |
|---|-----------|-------|
| 1 | **Waveshare ESP32-C6 Mini** | 160MHz, WiFi 6, BT5, USB-powered from PC |
| 2 | **BTS7960 43A H-Bridge** | Binghe, drives DCC track signal |
| 3 | **74HC14 Schmitt Trigger Inverter** | Powered at 3.3V from ESP32 |
| 4 | **Trimmer 10K** | BOJACK P103 blue, 3-pin, IS threshold adjustment |
| 5 | **Pull-down resistor** | On GPIO2 |
| 6 | **4x Capacitors 100nF** | 74HC14 bypass + IS filter on pin 5 + stop button debounce + resume button debounce |
| 7 | **Resistor 1K** | Series protection between 74HC14 pin 6 and GPIO3 |
| 8 | **Blue pushbutton** | GPIO21 — Resume |
| 9 | **Red pushbutton** | GPIO22 — Stop / E-Stop |
| 10 | **OLED Display 0.96"** | SSD1306/SSD1315 I2C module (128x64, blue+yellow) |
| 11 | **Bench power supply** | 15V |
| 12 | **Fuse holder + fuse** | On BTS7960 B+ line |
| 13 | **Breadboard 30 rows** | |

## Connections

### ESP32-C6 Mini (straddling both banks, row 30)

- **5V** (row 30 left) -> left + rail
- **GND** -> left - rail
- **3.3V** -> 74HC14 pin 14 (VCC)
- **GPIO2** -> pull-down resistor to GND + jumper to **74HC14 pin 1** (DCC signal)
- **GPIO3** -> 1K resistor -> **74HC14 pin 6** (short-circuit detection, digital)
- **GPIO18** -> R_EN + L_EN on BTS7960 (H-bridge enable)
- **GPIO21** -> blue pushbutton (Resume) with 100nF cap, row 23
- **GPIO22** -> red pushbutton (Stop) with 100nF cap, row 22
- **GPIO19** -> OLED SDA (I2C data)
- **GPIO20** -> OLED SCL (I2C clock)

### 74HC14 Schmitt Trigger Inverter

GND (pin 7) on right bank row 16, jumpered to left - rail.

| Pin | Function | Connection |
|-----|----------|------------|
| 1 | IN | <- GPIO2 (DCC signal) |
| 2 | OUT | -> LPWM on BTS7960 + jumper to pin 3 |
| 3 | IN | <- pin 2 (cascade for inverted signal) |
| 4 | OUT | -> RPWM on BTS7960 |
| 5 | IN | <- trimmer + BTS7960 R_IS/L_IS + 100nF filter cap to GND |
| 6 | OUT | -> 1K resistor -> GPIO3 (short detection) |
| 7 | GND | -> left - rail |
| 14 | VCC | <- ESP32 3.3V output |

100nF bypass capacitor between pin 14 (row 10) and GND (row 11 -> left - rail).

### Short-Circuit Detection Circuit (IS)

- BTS7960 **R_IS** + **L_IS** -> row 2 (right bank)
- **Trimmer 10K**: center pin to GND (left - rail), one outer pin to row 2
- Row 2 (right) -> jumper to row 2 (left) -> **74HC14 pin 5**
- **100nF capacitor** between pin 5 and GND (filters DCC commutation spikes)
- **74HC14 pin 6** -> **1K resistor** -> **GPIO3** (protects ESP32 GPIO)
- Normal: GPIO3 = HIGH (3.3V); Short detected: GPIO3 = LOW (falling edge triggers fault)

### BTS7960 H-Bridge

- **B+** -> fuse holder -> red terminal of bench PSU (15V)
- **B-** -> black terminal of bench PSU (common GND with ESP32)
- **LPWM** <- 74HC14 pin 2
- **RPWM** <- 74HC14 pin 4
- **R_EN + L_EN** <- GPIO18
- **R_IS + L_IS** -> trimmer circuit (see above)

### OLED Display 0.96" (SSD1306/SSD1315)

I2C interface for system status visualization (WiFi, boot progress, faults).

**Module pinout:**
- **VCC** -> ESP32 3.3V
- **GND** -> Common ground (left - rail)
- **SDA** -> **GPIO19**
- **SCL** -> **GPIO20**

I2C address: **0x3C** (default). Bus speed: 400 kHz.

**Compatible Modules:**
- SSD1306 (original) or SSD1315 (updated, fully compatible)
- 0.96" display, 128×64 resolution
- I2C interface (4 pins total)
- Blue + Yellow (pixels 0-15 yellow, 16-63 blue)

**Display layout:**
- Yellow zone (top 16px): "DCC Command Station" title
- Blue zone (bottom 48px): State, IP address, active loco count, faults

**Software:**
Display is integrated in `develop`. Currently shows:
- Boot progress indicator
- WiFi connection status + IP address
- System running state + active locomotive count
- Fault events (track short, e-stop)

### Power

- **ESP32**: USB from PC
- **BTS7960**: 15V bench power supply via fuse holder
- **74HC14**: 3.3V from ESP32 3.3V output
- **Common GND**: ESP32 GND <-> left - rail <-> BTS7960 B- <-> bench PSU black terminal
