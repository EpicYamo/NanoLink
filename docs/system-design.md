# System design

NanoLink consists of three independently built firmware units. The transmitter
and aircraft controller exchange RC data over nRF24. The aircraft controller
also sends sensor telemetry to a ground-station nRF24 receiver; the ground
station forwards received packets to a computer over USB serial.

## Transmitter to aircraft

The handheld Nano sends a 12-byte RC packet containing throttle, aileron,
elevator, arm state, two auxiliary values, and the throttle-mode switch state.
The RF link uses address `NANO1`, channel 90, and 250 kbps.

## Aircraft to ground station

The aircraft controller uses its nRF24 radio for both RC reception and
telemetry transmission. After receiving a valid RC packet, it may briefly
transmit one 32-byte telemetry packet to address `PC01A`, then returns to
listening for RC. Telemetry is sent at up to 10 packets per second. The ground
station listens on the same RF channel and outputs CSV records over USB serial
at 115200 baud.

Telemetry fields include a sequence number, sensor-valid flags, acceleration,
angular rate, magnetometer readings, barometric pressure, temperature, and RC
packet age. Acceleration is reported in mg, gyro rate in centidegrees per
second, pressure in Pa, and temperature in centidegrees Celsius. Magnetometer
values are uncalibrated sensor counts.

## Control surfaces and failsafe

The aircraft controller mixes the transmitter's roll and pitch channels into
left and right elevon commands. Servo direction and travel depend on the
airframe's linkage and must be verified on the bench. The transmitter arm bit
gates the ESC output. If valid RC packets stop arriving, the controller sets
the ESC to its configured idle pulse and centers both elevons.

## Firmware projects

- `firmware/transmitter_nano/` — handheld control input and RC packet sender.
- `firmware/aircraft_f401/` — RC receiver, elevon/ESC outputs, sensor reader,
  and telemetry sender.
- `firmware/ground_station_f401/` — telemetry receiver and USB serial output.
