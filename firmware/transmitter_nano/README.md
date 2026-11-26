# Arduino Nano handheld transmitter

This PlatformIO project implements the handheld transmitter using an Arduino
Nano. Its pin assignments and 12-byte RC packet are documented in the
repository's [hardware guide](../../docs/hardware-wiring.md) and
[system design](../../docs/system-design.md).

The A4 potentiometer sets throttle boost. A5 is an optional auxiliary
potentiometer; its reading is disabled by default. To sample A5, add
`-D ENABLE_AUX_POT1=1` to this project's `build_flags` in `platformio.ini`.
The upper nibble of its packet field carries the four trim-button states.

The A7 switch selects the Cruise/Reflex throttle response. A6 controls the
binary arm state.

Build with:

```sh
pio run -d firmware/transmitter_nano
```

For a short-range bench link near the aircraft, use the low-power RF build:

```sh
pio run -d firmware/transmitter_nano -e nanoatmega328_bench
```

Check stick direction and arm/disarm behavior with the propeller removed during
initial setup.
