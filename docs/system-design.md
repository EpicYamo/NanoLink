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
upper nibble reports the four momentary trim-button states. A4 is the throttle
boost value. The RF link uses address `NANO1`, channel 90, and 250 kbps.

## Aircraft to ground station

The aircraft controller uses its nRF24 radio for both RC reception and
telemetry transmission. After receiving a valid RC packet, it briefly
transmits one telemetry frame to address `PC01A`, then returns to listening for
RC. Sensor, control/link, and GPS frames are versioned and each fits the nRF24
32-byte payload limit. The ground station listens on the same RF channel and
forwards each validated frame as a JSON line at 115200 baud.

Telemetry fields include a sequence number, sensor-valid flags, acceleration,
angular rate, magnetometer readings, barometric pressure, temperature, and RC
packet age/counts, transmitter channel values, trim/switch state, and GPS
position/fix data. Acceleration is reported in mg, gyro rate in
centidegrees/second, pressure in Pa, and temperature in centidegrees Celsius.
Magnetometer values are uncalibrated sensor counts. The ground station's nRF24
hardware automatically acknowledges received telemetry frames; it does not yet
send application-level control commands.

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
requires aircraft stabilization and a command uplink, which are future firmware
stages.

## Ground-to-aircraft command link

The ground station also forwards mission commands to the aircraft. The computer
sends plain-text lines over USB serial (`CMD,set_home,lat_e7,lon_e7`,
`CMD,set_waypoint,lat_e7,lon_e7,alt_mm`, `CMD,set_loiter,radius_m`,
`CMD,start_mission`, `CMD,abort_mission`); the ground station packs each into a
32-byte `CommandFrame` and transmits it on a second nRF24 pipe (`CMND1`) that
the aircraft opens alongside the RC pipe. The aircraft validates each command,
stores the mission, and immediately transmits a `CommandAckFrame` back, which
the ground station forwards to the computer as a `command_ack` JSON line.

Commands are rate-limited by design: the aircraft's single radio alternates
briefly between RC listening, telemetry transmission, and command acknowledgment.
RC always keeps priority; a lost command simply reports `failed` on the ground
station. In the current stage the aircraft only **stores and acknowledges**
missions — `start_mission` records a request but does not fly it, because
stabilization and navigation controllers are not implemented yet.

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
left and right elevon commands. Servo direction and travel depend on the
airframe's linkage and must be verified on the bench. The transmitter arm bit
gates the ESC output. If valid RC packets stop arriving, the controller sets
the ESC to its configured idle pulse and centers both elevons.

## Firmware projects

- `firmware/transmitter_nano/` — handheld control input and RC packet sender.
- `firmware/aircraft_f401/` — RC receiver, elevon/ESC outputs, sensor reader,
  and telemetry sender.
- `firmware/ground_station_f401/` — telemetry receiver and USB serial output.
- `ground_app/` — two-mode, read-only desktop debug monitor.
