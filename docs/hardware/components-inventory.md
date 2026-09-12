# Components inventory

Updated on 8 September 2026. It separates what is mounted on the breadboard
today, what sits in the drawer, what has left the design and what still has to
be bought. The mounted circuit is described in `wiring.md`.

## Mounted on the breadboard

| Component | Qty | Role |
|---|---|---|
| Waveshare ESP32-C6 Mini | 1 | Microcontroller, RISC-V, WiFi 6 |
| Pololu DRV8874 carrier (#4035) | 1 | Track H-bridge, PWM mode |
| 74HC14 (six Schmitt inverters, DIP-14) | 1 | Inverts the DCC waveform and the bridge commands |
| 74HC08 (four AND gates, DIP-14) | 1 | Combines the DCC waveform with the run signal for the bridge |
| LM339 (four comparators, DIP-14) with socket | 1 | RailCom read front end |
| 4.7 Ω resistors, 1/2 W | 4 | RailCom sense resistors, one parallel pair per rail (2.35 Ω) |
| 1N5819 diodes | 4 | Clamps across each sense resistor |
| 1 kΩ resistors | 3 | Series resistors on the comparator inputs and pull-up of its output to 3.3 V |
| 22 kΩ, 4.7 kΩ, 100 Ω resistors | 1 each | Comparator threshold divider (about 18.5 mV) |
| 100 kΩ resistor | 1 | Pull-down on the run signal between GPIO4 and the 74HC08 |
| 100 nF capacitors | 4 | The only capacitors in the circuit: one on the supply of the 74HC08, one on the 74HC14, one between pins 3 and 12 of the LM339, one on the ESP32-C6 board supply |
| OLED SSD1306 1.3" I2C, 128x64 | 1 | Main display |
| Green LED and red LED | 1 + 1 | Status indicators |
| Push buttons | 2 | Stop and resume |
| WAGO 221-2411 | a few | Quick connections to the track and the supply |

## In the drawer, available

| Component | Qty | Notes |
|---|---|---|
| LM393N (dual comparator, DIP-8) with socket | 2 | Alternative to the LM339, not used |
| IRLZ44N N-channel MOSFET, TO-220 | 10 | Used in the external terminator trial, then removed |
| 1N5819 Schottky | about 96 | Spare |
| Resistor kit 1 Ω to 1 MΩ, 1/2 W, 1 % | 25 values | Metal film |
| OLED 0.96" I2C, 128x64 | 3 | Spare displays |
| WAGO 221-2411 | rest of the pack | |

## Left the design

| Component | Why |
|---|---|
| BTS7960 43 A H-bridge | Replaced by the DRV8874: the BTS7960 cannot short the rails together as the RailCom cutout requires |
| TLV3501 comparator (2 pieces on adapters) | Input limited to 3.3 V, it saturated on the full DCC swing; replaced by the LM339 in series with the rail |
| External terminator with IRLZ44N and 2.7 Ω resistors | The cutout is done with the DRV8874 brake, which shorts the rails at very low impedance; the terminator was out of specification |
| BAT54 Schottky | No longer needed: the clamps are 1N5819 and the circuit works |

## To buy

Nothing for the current circuit. The printed board will be assessed separately.
