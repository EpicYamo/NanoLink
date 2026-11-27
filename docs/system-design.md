# System design

NanoLink consists of three independently built firmware units and a desktop
monitor. The transmitter and aircraft controller exchange RC data over nRF24.
The aircraft sends telemetry to the ground station, which forwards it to the
desktop application over USB serial. A separate USB diagnostic build is used
to inspect local aircraft sensors without the radio or PWM outputs.

## Transmitter to aircraft

The handheld Nano sends a 12-byte RC packet containing throttle, aileron,
elevator, arm state, two auxiliary values, and the throttle-mode switch state.
The optional A5 auxiliary analog reading occupies the low 10 bits of `pot1`; the
upper nibble reports the four momentary trim-button states. Bits 10 and 11 of
`pot1` carry the two throttle-stick trigger buttons (telemetry-dump request and
a draft return-to-home request). A4 is the control-authority knob (10%..100%).
The RF link uses address `NANO1`, channel 90, and 250 kbps, and is
fire-and-forget (no auto-ACK).

## Aircraft to ground station

The aircraft controller uses its nRF24 radio for both RC reception and
telemetry transmission. It stays in receive mode almost continuously and, every
3 seconds (or when the transmitter dump trigger is pressed), briefly transmits a
full snapshot — sensor, control/link, and GPS frames back-to-back — to address
`PC01A`, then returns to listening. All frames are fire-and-forget; the ground
station is a pure listener and never acknowledges, so there is no RX/TX
switching contention. Sensor, control/link, and GPS frames are versioned and
each fits the nRF24 32-byte payload limit.

Telemetry fields include a sequence number, sensor-valid flags, acceleration,
angular rate, magnetometer readings, barometric pressure, temperature, and RC
packet age/counts, transmitter channel values, trim/switch state, and GPS
position/fix data. Acceleration is reported in mg, gyro rate in
centidegrees/second, pressure in Pa, and temperature in centidegrees Celsius.
Magnetometer values are uncalibrated sensor counts.

## Desktop monitoring modes

`ground_app/` accepts newline-delimited JSON from either:

- `bench_usb`: local sensor/GPS diagnostics over the aircraft F401's native USB;
  RF reception and actuator PWM are disabled in this build.
- Ground station: RF telemetry received by the ground F401 and forwarded over
  USB. This mode includes aircraft-reported RC input and link counters.

The monitor is read-only; it does not issue aircraft commands.

The desktop app also includes a map and mission planner: the first GPS fix sets
home, a map click places a waypoint with a target altitude, and the planned
sequence (waypoint, return home, loiter circle) is displayed with distances and
bearings. This is a planning/visualization feature only — executing a mission
requires a navigation controller, which is a future firmware stage.

## Stabilization (stage B)

The aircraft implements a self-level mode using a complementary filter (gyro
integration blended with the accelerometer level reference) and a P(D) angle
controller on roll and pitch. The transmitter's 3-position switch selects the
mode: position 0 is manual passthrough, positions 1 and 2 engage self-level.
The A4 knob on the transmitter is a single control-authority limit (10%..100%):
it scales the elevon throw in manual mode and the correction authority in
stabilized mode the same way. The level reference and gyro bias are captured by
the ground-station `CMD_CALIBRATE_LEVEL` command (the app's "Calibrate
level/gyro" button); the gyro bias is averaged over ~1 s. Direction and axis
mapping are configured by `ROLL_SIGN`, `PITCH_SIGN`, `SWAP_ROLL_PITCH`, and
`SWAP_ELEVONS` in the aircraft firmware and must be verified on the bench before
first flight.

## Ground-to-aircraft command link

The ground station also forwards mission commands to the aircraft. The computer
sends plain-text lines over USB serial (`CMD,set_home,lat_e7,lon_e7`,
`CMD,set_waypoint,lat_e7,lon_e7,alt_mm`, `CMD,set_loiter,radius_m`,
`CMD,start_mission`, `CMD,abort_mission`, `CMD,calibrate_level`, `CMD,rth`);
the ground station packs each into a 32-byte `CommandFrame` and, briefly
switching from listen to transmit, sends it fire-and-forget on a second nRF24
pipe (`CMND1`) that the aircraft opens alongside the RC pipe. The aircraft
validates each command, stores the mission, and immediately transmits a
`CommandAckFrame` back, which the ground station forwards to the computer as a
`command_ack` JSON line.

Because the ground station is otherwise a pure listener, the only RX/TX
switching it ever performs is this short command transmit. In the current stage
the aircraft only **stores and acknowledges** missions — `start_mission` and
`rth` record a request but do not fly it, because the navigation controller is
not implemented yet.

## Transmitter RC sniffer (bench diagnostic)

The ground station also opens a receive pipe on the RC address (`NANO1`), so the
handheld transmitter can be verified against the ground station **without the
aircraft**. Received RC packets are reported over USB as `rc_sniff` JSON lines
(~5 Hz) and shown on the ground-station OLED's RF Diagnostics page as
`TX SNIFF: LIVE` with a packet counter. The desktop app displays sniffed
channels in the Transmitter tab and marks the RC tile as `RC SNIFF LIVE` while
aircraft telemetry is absent. This isolates the two RF ends: sniff live but no
air link points to the aircraft side; no sniff points to the transmitter.

## Control surfaces and failsafe

The aircraft controller mixes the transmitter's roll and pitch channels into
left and right elevon commands. Elevon throw tops out at `MIX_LIMIT_US` (300 µs
by default) and the transmitter's A4 knob scales it from 10%..100% (dual
rates). The throttle stick is self-centering, so its rest position is idle and
pushing up adds throttle. Servo direction and travel depend on the airframe's
linkage and must be verified on the bench; the `ROLL_SIGN`, `PITCH_SIGN`,
`SWAP_ROLL_PITCH`, and `SWAP_ELEVONS` constants correct any reversal. The
transmitter arm bit gates the ESC output. If valid RC packets stop arriving,
the controller sets the ESC to its configured idle pulse and centers both
elevons.

## Firmware projects

- `firmware/transmitter_nano/` — handheld control input and RC packet sender.
- `firmware/aircraft_f401/` — RC receiver, elevon/ESC outputs, sensor reader,
  and telemetry sender.
- `firmware/ground_station_f401/` — telemetry receiver and USB serial output.
- `ground_app/` — two-mode, read-only desktop debug monitor.
