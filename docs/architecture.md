# NanoLink architecture and flight-mode proposal

## Radio topology

The aircraft F401 has one nRF24 transceiver. It listens to the existing Nano
transmitter on address `NANO1`, channel 90, 250 kbps. After a valid RC packet,
it may switch briefly to transmit a fixed-size telemetry packet to the ground
station address `PC01A`, then immediately return to the RC listening pipe.
Telemetry is rate-limited to 10 packets/second and is lower priority than RC.
The ground station listens continuously and does not transmit in this stage.

The nRF24 hardware ACK is returned to the handheld transmitter before the
aircraft application sends its separate telemetry packet. Telemetry send uses
no long retry loop so a missing computer/station cannot stall RC servicing.
The existing 500 ms RC timeout remains a fail-safe boundary. A future
computer-command link must use an explicit slot and arbitration policy; it must
never be allowed to delay or override a valid manual RC command.

## Packet compatibility

The aircraft accepts the existing packed 12-byte Nano packet:

```text
uint16 throttle_us
uint16 aileron_us
uint16 elevator_us
uint8  armed
uint16 pot1
uint16 pot2
uint8  switch2_state
```

The receiver ignores `pot1` and `pot2` for now. In particular, A5 / `pot1` is
known to be erratic and must not be used for a flight-critical setting. Keep
the layout packed and guarded by a compile-time size assertion.

Telemetry is versioned, fixed-size (at most the nRF24 32-byte payload limit),
and includes a sequence counter, sensor-valid flags, accelerometer in mg, gyro
rate in centidegrees/second, raw magnetometer counts, pressure in Pa and
temperature in centidegrees C. The ground station prints CSV for easy capture.

## Elevon mixing

For manual mode, center each incoming stick channel around 1500 us. The left
and right elevons are formed from pitch plus/minus roll, with normalization to
keep both outputs within the transmitter's existing 1150–1850 us low-rate
range. Sign constants are deliberately visible in the aircraft source; they
must be confirmed from the actual servo/linkage direction on the bench.

The ESC output is gated by the transmitter's arm bit. Failsafe outputs are
neutral elevons and the configured ESC idle pulse. Confirm the particular ESC's
idle pulse with the propeller removed; the legacy project recorded 750 us for
one setup, but that value is not assumed to be universal.

## Stabilization switch proposal

Do not overload the existing throttle-mode switch: it currently chooses the
transmitter's Cruise/Reflex throttle curve. The existing arm switch is the
better candidate if it is physically three-position. Proposed interpretation:

| Arm switch position | Meaning |
|---|---|
| 1 | DISARM: motor gated off |
| 2 | ARMED + MANUAL: raw pilot control |
| 3 | ARMED + STABILIZED: pilot commands attitude/rate, controller adds correction |

The transmitter currently sends a boolean arm value, so the switch ADC ranges
and packet field must be changed together before enabling position 3. Until
that change is implemented and tested, the receiver remains manual-only. A
future mode must fall back to manual/defined failsafe if IMU data is invalid;
it must never auto-arm on boot.

## Sensor wiring

On the aircraft Black Pill F401, retain the existing RF pins and PWM pins. Use
I2C2 on PB3 (SDA) and PB10 (SCL) so the sensor bus does not collide with the
existing PB6/PB7/PB8 PWM outputs. PB3 is the optional SWO pin; it is available
when trace output is not in use. Start with 3.3 V sensor power, common ground,
and short I2C leads. Confirm the exact breakout board has pull-ups (or add
4.7 kOhm pull-ups to 3.3 V if absent).

## Safety staging

1. Sensors and radio telemetry only; no propeller.
2. RC receiver, arming, failsafe and elevon direction checks; no propeller.
3. Stabilizer mode checks by moving/tilting the aircraft by hand; no propeller.
4. Only after repeated bench checks, brief restrained power tests and a
   conservative manual maiden in a suitable open area.
5. GPS initially reports/logs position only. Do not add return-to-home or
   waypoint control in the first GPS phase.
