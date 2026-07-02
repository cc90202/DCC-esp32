# RailCom Option A - external cutout terminator

Status: selected direction for the next RailCom rebuild.

Last updated: 2026-05-28.

## Decision

Use an external RailCom terminator/detector that is physically active only
during the RailCom cutout. Do not connect TLV3501 inputs directly to rail-side
nodes during normal DCC.

This is inspired by the Atanisoft ESP32 Command Station approach: the RailCom
receive circuit has its own gated nodes and comparators instead of leaving the
comparator front-end exposed to the full DCC waveform.

The selected implementation changes the DRV8874 control mode:

- `PMODE = 3.3 V`, so the DRV8874 runs in PWM mode.
- `GPIO18` remains `nSLEEP`, the master track enable.
- `GPIO2` remains the RMT DCC waveform.
- `GPIO4` becomes `DCC_RUN`: HIGH during normal DCC, LOW during RailCom cutout.
- External logic derives `IN1`, `IN2`, and `CUTOUT_EN` from `GPIO2` and `GPIO4`.

No firmware timing change is required for the first build. The existing cutout
state machine already drives `GPIO4` HIGH outside cutout and LOW during cutout.

## Why this replaces the direct-sense DRV8874 build

The previous DRV8874/TLV3501 detector put the sense nodes in series with the
rails and fed them directly into 3.3 V comparators. During normal DCC those
nodes can see the rail waveform, so the comparator input protection can be
overdriven before every cutout. That makes the detector unreliable even when
the firmware cutout timing is correct.

Option A fixes that at the topology level:

- during normal DCC, the detector MOSFETs are off and the comparator sense nodes
  sit near ground;
- during cutout, the DRV8874 outputs are put in high impedance;
- only then does the external terminator connect both rails to a local
  RailCom return path through small burden resistors;
- the TLV3501 inputs only see tens of millivolts around ground.

## DRV8874 PWM-mode drive

Use one DCC waveform and one run/blanking signal:

```text
DCC       = GPIO2 RMT waveform
DCC_N     = 74HC14(DCC)
DCC_RUN   = GPIO4, HIGH normal, LOW cutout
CUTOUT_EN = 74HC14(DCC_RUN), LOW normal, HIGH cutout

DRV8874 IN1 = DCC_RUN AND DCC
DRV8874 IN2 = DCC_RUN AND DCC_N
DRV8874 PMODE = 3.3 V
DRV8874 nSLEEP = GPIO18
```

Use a 74HC08/74AHC08/74LVC08 for the two AND gates. The existing 74HC14 can
provide the DCC inversion, `CUTOUT_EN`, and the RailCom RX buffer if enough
gates are still free.

The local DCC-EX DRV8874 reference schematic documents the relevant PWM-mode
truth table: with `nSLEEP=1`, `IN1=0`, and `IN2=0`, the DRV8874 outputs are
`ZZ`/Hi-Z. That is the hardware state Option A relies on during the cutout.

Expected states:

| State | GPIO18/nSLEEP | GPIO4/DCC_RUN | IN1 | IN2 | DRV8874 outputs |
| --- | --- | --- | --- | --- | --- |
| fault/off | LOW | don't care | don't care | don't care | Hi-Z after sleep |
| DCC bit 1 | HIGH | HIGH | DCC | !DCC | differential DCC |
| RailCom cutout | HIGH | LOW | LOW | LOW | Hi-Z in PWM mode |

Keep a 100 kOhm pulldown on `DCC_RUN` or the AND-gate enable input so a reset
or floating MCU pin does not accidentally drive the track. Keep the existing
hardware habit that `nSLEEP` is low until the firmware arms track power.

## External terminator and burden

Remove the old rail-series sense resistors from the normal DCC path. In normal
DCC, the DRV8874 outputs should drive the rails directly:

```text
DRV8874 OUT1 ------------------------------ TRACK_A
DRV8874 OUT2 ------------------------------ TRACK_B
```

Add a cutout-only terminator with two N-channel MOSFETs and two burden
resistors:

```text
                         CUTOUT_EN
                            |
                +-----------+-----------+
                |                       |
              220R                    220R
                |                       |
TRACK_A ---- drain Q_A          TRACK_B ---- drain Q_B
              source                  source
                |                       |
             SENSE_A                 SENSE_B
                |                       |
            R_A 2.7R                R_B 2.7R
                |                       |
              STAR_GND ------------- STAR_GND

Q_A gate -> 100 kOhm -> GND
Q_B gate -> 100 kOhm -> GND
```

Use the available IRLZ44N devices for the breadboard. For a PCB, replace them
with lower-capacitance logic-level MOSFETs or a dual MOSFET package sized for
the expected cutout current.

The MOSFET channels are bidirectional when on, so one burden resistor goes
positive for one RailCom current direction and the other goes positive for the
opposite direction. Two comparators are therefore still required.

Burden math with 2.7 ohm on each side:

```text
R_total ~= 2.7R + 2.7R + MOSFET Rds_on
drop at 34 mA ~= 5.4R * 0.034A = 184 mV
```

That stays under the 200 mV RailCom detector burden target used in the project
notes. The MOSFETs must be fully off during DCC normal operation; otherwise the
terminator is a low-value load across the track.

## Comparator front-end

Use the two TLV3501 comparators as active-low threshold detectors:

```text
SENSE_A -> 1 kOhm -> TLV3501_A IN-
SENSE_B -> 1 kOhm -> TLV3501_B IN-

VREF_RAILCOM -> TLV3501_A IN+
VREF_RAILCOM -> TLV3501_B IN+

TLV3501_A OUT -> diode OR -> RAILCOM_RAW
TLV3501_B OUT -> diode OR -> RAILCOM_RAW
RAILCOM_RAW -> one 74HC14 gate -> GPIO5
```

VREF:

```text
3.3 V -> 150 kOhm -> VREF_RAILCOM -> 1 kOhm -> STAR_GND
VREF_RAILCOM -> 100 nF -> STAR_GND
```

Expected value:

```text
VREF_RAILCOM ~= 21.9 mV
I_trip ~= 21.9 mV / 2.7R = 8.1 mA
```

Add input clamps on each comparator input after the 1 kOhm series resistor:

```text
GND ----|<|---- TLV_IN-      negative clamp, cathode at TLV_IN-
TLV_IN- ----|<|---- 3.3 V   positive clamp, cathode at 3.3 V
```

Small-signal Schottky diodes such as BAT54 are preferred. The available 1N5819
diodes are acceptable for a first bench test if the waveform at `TLV_IN-` is
confirmed clean on the scope.

Start without hysteresis. If `RAILCOM_RAW` chatters, add per-comparator
hysteresis by splitting VREF first:

```text
VREF_RAILCOM -> 10 kOhm -> VREF_A -> TLV3501_A IN+
VREF_RAILCOM -> 10 kOhm -> VREF_B -> TLV3501_B IN+
OUT_A -> 680 kOhm .. 1 MOhm -> VREF_A
OUT_B -> 680 kOhm .. 1 MOhm -> VREF_B
```

Do not put one feedback resistor onto the shared VREF node; one comparator
would move the threshold of the other.

## RailCom RX polarity

The firmware currently inverts the UART RX input on `GPIO5`.

Use one physical 74HC14 gate between `RAILCOM_RAW` and `GPIO5`:

```text
RAILCOM_RAW idle HIGH -> 74HC14 output LOW -> GPIO5 physical LOW
firmware input inverter -> UART idle HIGH

RAILCOM_RAW pulse LOW -> 74HC14 output HIGH -> GPIO5 physical HIGH
firmware input inverter -> UART bit LOW
```

Do not use the old double-inverter RX path unless the firmware input inversion
is removed.

## Parts needed

Already in inventory:

- 2x TLV3501AIDR on PA0002 breakout
- 2x IRLZ44N for the cutout-only terminator
- 1x 74HC14, if enough gates are free
- 2x 1N5819 for diode-OR or temporary input clamps
- resistor kit values: 220R, 1k, 2.7R if present, 4.7k, 10k, 100k, 150k,
  680k/1M
- 100 nF capacitors

Add or order:

- 1x 74HC08, 74AHC08, or 74LVC08 quad AND gate
- BAT54/BAT54S small-signal Schottky diodes for comparator input clamps
- 2x 2.7 ohm resistors rated at least 0.5 W, preferably 1 W for bench margin

## Bring-up checklist

With all power off:

- [ ] `PMODE` is moved from GND to 3.3 V.
- [ ] old rail-series sense resistors are removed from the normal DCC path.
- [ ] `TRACK_A` and `TRACK_B` are not shorted through the terminator when
      `CUTOUT_EN` is LOW.
- [ ] `Q_A` and `Q_B` gates have 100 kOhm pulldowns.
- [ ] `DCC_RUN` has a defined default state.
- [ ] `OUT_A` and `OUT_B` from the TLV3501 are not tied together directly.
- [ ] `RAILCOM_RAW` has a 4.7 kOhm pull-up to 3.3 V.

With USB only, no 15 V:

- [ ] `GPIO4/DCC_RUN` idles HIGH after boot.
- [ ] during a cutout, `GPIO4/DCC_RUN` goes LOW for about 460 us.
- [ ] `CUTOUT_EN` is LOW outside cutout and HIGH during cutout.
- [ ] outside cutout, `IN1 = DCC` and `IN2 = !DCC`.
- [ ] during cutout, `IN1 = LOW` and `IN2 = LOW`.
- [ ] `VREF_RAILCOM` is about 22 mV.
- [ ] `GPIO5` physical idle is LOW after the single 74HC14 RX gate.

With 15 V current-limited and no locomotive:

- [ ] normal DCC appears between the rails.
- [ ] during cutout, the DRV8874 outputs go Hi-Z and the external terminator
      clamps both rails near `STAR_GND`.
- [ ] current limit does not trip when `CUTOUT_EN` turns on.
- [ ] `SENSE_A` and `SENSE_B` stay near 0 V without RailCom current.
- [ ] `RAILCOM_RAW` stays stable HIGH without RailCom current.

With a RailCom-capable locomotive:

- [ ] `SENSE_A` shows positive pulses in some windows.
- [ ] `SENSE_B` shows positive pulses in complementary windows.
- [ ] each TLV input remains inside the clamp range.
- [ ] `RAILCOM_RAW` shows active-low UART pulses in the cutout.
- [ ] `GPIO5` shows active-high pulses after the 74HC14.
- [ ] firmware logs show `rx_windows` with non-empty bytes, then valid ACK or
      address/CV data.

## Failure interpretation

If `IN1/IN2` are not both LOW during cutout, debug the AND-gate logic before
connecting the rails.

If the 15 V current limit trips at cutout start, the terminator is turning on
while the DRV8874 is still actively driving. Verify `PMODE`, AND-gate wiring,
and `GPIO4` polarity before retrying.

If `SENSE_A/B` show clean analog pulses but `RAILCOM_RAW` is noisy, debug the
TLV3501 threshold, clamps, bypass capacitors, and optional hysteresis.

If `RAILCOM_RAW` is clean but the firmware reports empty/corrupt windows, the
problem has moved back to UART polarity or cutout window timing.
