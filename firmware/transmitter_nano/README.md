# Arduino Nano handheld transmitter

This is a separate PlatformIO project for the existing DIY handheld remote.
It keeps the original Nano pin assignments, RF address/channel/data-rate and
12-byte packet contract used by `aircraft_f401`.

The spare potentiometer on A5 is intentionally not sampled: it is known to
produce erratic readings. Its packet field is sent as zero and is ignored by
the aircraft firmware. A4 remains the throttle-boost control.

The A7 three-position switch still selects the transmitter's existing
Cruise/Reflex throttle behavior. The A6 arm input is currently a boolean. A
future DISARM / MANUAL / STABILIZED three-position mapping needs a packet and
transmitter update together; it is not active in this firmware.

Build with:

```sh
pio run -d firmware/transmitter_nano
```

Check stick direction and arm/disarm behavior with the aircraft propeller
removed before connecting the motor for any powered test.
