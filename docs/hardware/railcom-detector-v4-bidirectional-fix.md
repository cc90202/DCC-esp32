# RailCom detector v4 - bidirectional hardware fix

Status: proposed hardware fix after CV8 captures showed ACK/address data but no
reliable CH2 CV data.

## Why v3 is not enough

The v3 detector is still single-ended:

- Q1 side has `4.7 ohm` sense and is measured.
- Q2 side has `1 ohm` current limit and is not measured.
- The comparator only detects one RailCom current direction.

That is good enough to prove that RailCom exists: captures show `F0` ACK and
`99 A5` = `AdrLow(3)`. It is not good enough as a production detector. Long CH2
datagrams can be missed or distorted depending on current direction, packet
polarity, ground bounce, and threshold margin.

The fix is not more firmware filtering. The detector must sense both current
directions.

## Quick confirmation test before rebuilding

Before changing components, swap `TRACK_A` and `TRACK_B` at the RailCom detector
input only, then repeat a single CV8 read.

Expected interpretation:

- If CV data appears after the swap, the current detector was seeing the wrong
  polarity for the app:pom response.
- If ACK/address changes but CV data is still absent, the detector is still too
  marginal for long CH2 bursts.

This is only a diagnostic test. Do not keep swapping wires as the permanent fix.

## Permanent v4 topology

Use symmetric burden resistors and two fast comparators. Either comparator can
detect current; their active-low outputs are diode-combined into the existing
74HC14 buffer chain.

```text
                  cutout ON

TRACK_A -- Q1 -- SENSE_A -- 2.7R -- STAR_GND
                         |
                         +--> comparator A IN-

TRACK_B -- Q2 -- SENSE_B -- 2.7R -- STAR_GND
                         |
                         +--> comparator B IN-

VREF_22mV ------------------> comparator A IN+
VREF_22mV ------------------> comparator B IN+

comparator A OUT --|<|--+
                   D1   |
comparator B OUT --|<|--+-- RAILCOM_RAW -- 4.7K -- 3V3
                   D2

RAILCOM_RAW -> 74HC14 gate 5 -> 74HC14 gate 6 -> GPIO5
```

Diode orientation:

- diode anodes tied together at `RAILCOM_RAW`
- diode cathode of D1 to comparator A OUT
- diode cathode of D2 to comparator B OUT

Reason: each TLV3501 output is HIGH when inactive and LOW when its sensed
current exceeds threshold. The diode-combine node stays HIGH only when both
comparators are inactive. If either comparator goes LOW, `RAILCOM_RAW` is pulled
LOW.

## Component changes

Remove from v3:

- `R_sense 4.7 ohm`
- `R_limit 1 ohm`
- `R_hyst 47K` if installed

Install:

| Ref | Value | Purpose |
| --- | --- | --- |
| R_A | 2.7 ohm, 1%, >= 0.5 W | Track A burden and sense |
| R_B | 2.7 ohm, 1%, >= 0.5 W | Track B burden and sense |
| U_A | TLV3501 or one half of TLV3502 | Comparator for SENSE_A |
| U_B | TLV3501 or second half of TLV3502 | Comparator for SENSE_B |
| R_ref_high | 150K | VREF divider high side |
| R_ref_low | 1K | VREF divider low side |
| C_vref | 100 nF | VREF filter to STAR_GND |
| R_hyst_A | 680K to 1M | Light hysteresis for comparator A |
| R_hyst_B | 680K to 1M | Light hysteresis for comparator B |
| D1, D2 | BAT54 / 1N5819 small Schottky | Active-low diode combine |
| R_raw_pullup | 4.7K | Pull-up for RAILCOM_RAW |

Do not use `47K` hysteresis with the new `2.7 ohm` sense resistors. It pushes
the effective switching threshold too high. Use `680K` first.

## Electrical budget

With two symmetric burden resistors:

```text
R_total ~= 2.7R + 2.7R + MOSFET Rds_on
        ~= 5.4R + ~0.05R

burden at 34 mA ~= 5.45R * 0.034A = 185 mV
```

This stays below the 200 mV RailCom burden budget.

Threshold with `VREF ~= 22 mV`:

```text
I_threshold ~= 22 mV / 2.7R = 8.1 mA
```

That sits between the RailCom low-current and high-current decision region used
by the existing v2/v3 notes.

## Wiring changes from v3

Keep:

- Q1/Q2 gate drive from `CUTOUT_INV`
- Q1/Q2 MOSFET placement
- star ground concept
- 74HC14 gate 5 + gate 6 buffer into GPIO5
- twisted pairs for long wires
- TLV3501 bypass capacitors close to the package

Change:

1. Q1 source becomes `SENSE_A`.
2. Q2 source becomes `SENSE_B`.
3. `SENSE_A -> R_A 2.7R -> STAR_GND`.
4. `SENSE_B -> R_B 2.7R -> STAR_GND`.
5. Comparator A:
   - `IN- = SENSE_A`
   - `IN+ = VREF_22mV`
   - `OUT = COMP_A_OUT`
6. Comparator B:
   - `IN- = SENSE_B`
   - `IN+ = VREF_22mV`
   - `OUT = COMP_B_OUT`
7. `COMP_A_OUT` and `COMP_B_OUT` go through D1/D2 to `RAILCOM_RAW`.
8. `RAILCOM_RAW` goes to the same 74HC14 double-inverter buffer previously fed
   by the single TLV3501 output.

## Expected scope result

Probe points:

- `SENSE_A`
- `SENSE_B`
- `RAILCOM_RAW`
- GPIO5 if using the 74HC14 buffered output

With one channel, probe in this order:

1. `RAILCOM_RAW`: should be idle HIGH and show clean active-low UART pulses.
2. `SENSE_A`: should show positive pulses for roughly half of useful windows.
3. `SENSE_B`: should show positive pulses for the complementary windows.

Expected firmware/log result after v4:

- `ACK` still appears.
- `AdrLow(3)` still appears.
- CH2 multi-byte windows should stop collapsing into invalid/truncated bursts.
- For CV8, a real `CvData(...)` should appear if the decoder emits app:pom.

If `SENSE_A` and `SENSE_B` show valid analog pulses but `RAILCOM_RAW` is bad,
the problem is comparator/diode-combine/buffer. If `RAILCOM_RAW` is clean but
firmware still misses bytes, the problem moves back to UART window timing.
