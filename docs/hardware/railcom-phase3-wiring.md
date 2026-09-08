# RailCom phase 3 — wiring the read front end

> Done. The build described here was completed and verified on
> 7 September 2026: the board identifies the locomotive and reads and
> writes CVs over RailCom. The current state of the circuit is in
> `current-circuit.md`; this document remains as the assembly procedure.

Bench document, written on 2 September 2026. It describes the circuit that
turns the decoder's answer, small current pulses inside the RailCom cutout,
into a digital signal the board's serial port can read.

The circuit comes from comparing the RCN-217 standard (edition of
24 November 2025) with three working designs: the example circuit in the
standard itself, the ESP32 detector by Christophe Bobille published by
Locoduino, and the MTB-RC detector by kmzbrnoI. All three use the same chip
we use.

## What is needed

One LM339 with its fourteen-pin socket. Two 4.7 ohm resistors, to be put in
parallel, identical to the pair already mounted on the other rail. Four
1N5819 diodes. Three 1 kiloohm resistors. One 22 kiloohm, one 4.7 kiloohm,
one 100 ohm. One or two 100 nanofarad capacitors, if available. Short wires.

All the assembly is done with the circuit off, supply disconnected and USB
cable unplugged.

## Step 0 — check before powering

With the multimeter, circuit off, check that there is no continuity between
the 5 volt line and ground, and likewise between 3.3 volts and ground. Repeat
the check at the end, before applying power.

## The chip and its supply

1. Plug the socket across the centre channel of the breadboard, in a free
   area, with the reference notch facing up. Insert the LM339 in the socket
   following that notch.

2. A wire from pin 3 to the 5 volt line. A wire from pin 12 to the star
   ground, the same used for everything else.

3. If available, a 100 nanofarad capacitor directly between pin 3 and
   pin 12, as short as possible. It keeps the chip's supply clean; not
   essential for the first test.

## The threshold voltage

It is the yardstick the comparator measures against: below it, "no current";
above it, "the decoder is talking". It is built once and serves both
comparators.

4. Pick a free row of the breadboard: it will be the threshold node. From
   the 5 volt line, put a 22 kiloohm and a 4.7 kiloohm resistor in series,
   with the second one landing on that row.

5. From the same row, a 100 ohm resistor to ground. About eighteen and a
   half millivolts form on that row, which on the sense resistors correspond
   to eight milliamperes of decoder current.

6. From the threshold row, one wire to pin 5 and another to pin 7.

## The first rail — the one already equipped

On the rail where the pair of 4.7 ohm resistors is already in series between
the bridge output and the track.

7. Across that pair, two 1N5819 diodes facing opposite ways: the first with
   its band towards the track side, the second with its band towards the
   bridge side. They pass the strong train current by themselves, which
   would otherwise burn the resistors, and leave the small decoder current
   untouched.

8. From the track-side node of that pair, not the bridge side, a 1 kiloohm
   resistor to pin 4. It protects the chip when, outside the cutout, that
   point swings by tens of volts.

## The second rail — to equip now

9. Find the direct link between the second bridge output and its rail and
   break it: remove the wire or jumper joining them.

10. In its place, two 4.7 ohm resistors side by side in parallel, like the
    other pair, so that the current from the bridge output to the rail has
    to pass through them.

11. Across this new pair, the other two 1N5819 diodes, again facing opposite
    ways as in step 7.

12. From the track-side node of this pair, a 1 kiloohm resistor to pin 6.

## The output to the board

13. Join pin 1 and pin 2, which are adjacent, with a short wire. The two
    outputs work together: the one that sees current pulls the common wire
    down, the other watches.

14. From that common node, a 1 kiloohm resistor to the **3.3 volt** line,
    3.3 and not five. This resistor sets the high level, and it is the reason
    the board pin will never see more than 3.3 volts.

15. From the same common node, the wire to GPIO5 on the board.

## Before powering

16. Repeat the step 0 check and add two more: that pin 3 is not in contact
    with ground, and that the common output node is not either.

## Two things not to get wrong

The **signal inputs** are pins **4 and 6**, the **threshold** goes to pins
**5 and 7**. Swapped, the circuit works backwards and the board's serial port
understands nothing.

The resistor of step 14 goes to **3.3 volts**. Taking it to five by mistake
sends five volts into a pin rated for 3.3.

## Why these values

**Threshold at eight milliamperes.** The standard requires the detector to
read a current above ten milliamperes as a zero and below six as a one: the
threshold must fall in that band, and eight is the value recommended by
OpenDCC. On our 2.35 ohm resistors eight milliamperes make eighteen and a
half millivolts, which the divider of 22 kiloohm plus 4.7 kiloohm above and
100 ohm below reproduces almost exactly from five volts.

**One sense resistor per rail.** The decoder current enters through one rail
and leaves through the other, and the direction depends on how the
locomotive sits. The standard and the other designs use a single resistor
and build a negative supply to see the opposite direction too: having only
positive voltages, we put one resistor per rail. During braking one node
goes above zero and the other below, and watching the one above is enough.
The two resistors in series drop one hundred and sixty millivolts at
thirty-four milliamperes, under the two hundred set by the standard.

**The diodes across the resistors.** They are in all three reference
designs. At eighty millivolts, that is with only the decoder signal present,
they conduct a few microamperes and steal nothing; above half a volt, that
is when traction current flows, they divert it through themselves.

**Five volt supply, 3.3 volt output.** The manufacturer characterises the
LM339 at five volts, so that is the operating point with guaranteed numbers.
Its outputs can only pull down, never push: with the pull-up resistor to
3.3 volts the board pin cannot see more than 3.3 volts under any condition,
not even a fault.

**No hysteresis, for now.** OpenDCC recommends a dead band of two
milliamperes around the threshold. We start without: the measured signal is
four times the threshold and the floor during braking is flat. If the
comparator chatters, it will be added.

## What has been verified, and what not

The model of this comparator was run on the samples recorded with the
oscilloscope on 2 September, and the output decoded as the serial port
would: all eight decoder bytes came out, with correct start and stop bits and
a valid check code on all. Raising the threshold, decoding holds up to forty
millivolts and breaks beyond: the chosen threshold sits in the middle of the
good zone.

Four things will only be known after powering up. The noise below forty
millivolts, where the oscilloscope used for that capture is blind because of
its resolution, and where our threshold sits. The actual speed of the
comparator. The offset of this particular chip, up to five millivolts,
measured on the bench. And which of the two comparators will trip, since the
second sense resistor has never been mounted.
