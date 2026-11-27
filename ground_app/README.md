# NanoLink Debug Monitor

A read-only desktop monitor for the two USB paths used during NanoLink testing.
It shows sensor health, approximate attitude, GPS status, aircraft-reported RC
inputs, and packet/link counters.

Direct USB diagnostics also report the I2C address scan and the MPU9250
`WHO_AM_I` response, which helps distinguish a wiring/address issue from a
sensor initialization error.

## Connection modes

### Direct USB diagnostics

Connect the aircraft F401 directly to the computer and build the diagnostic
firmware:

```sh
pio run -d firmware/aircraft_f401 -e bench_usb_dfu -t upload
```

Enter the STM32 system bootloader first: hold BOOT0, tap NRST, then release
BOOT0. The USB-C cable is used for both DFU upload and the later CDC serial
diagnostic connection.

This USB-only image reads the IMU, barometer, and GPS and emits JSON lines over
USB serial. It does not initialize the nRF24 receiver or drive the elevon/ESC
PWM outputs. RC data is therefore shown as unavailable in this mode.

### Ground-station telemetry

Flash `firmware/ground_station_f401` onto the ground F401. Keep the aircraft
controller on its normal firmware and power it from the aircraft BEC. Connect
the ground-station F401 to the computer over USB; it receives nRF24 telemetry
and forwards JSON lines to this application. This mode includes RC channel
values reported by the aircraft, IMU/barometer readings, GPS status and radio
packet counters.

The application is monitor-only. It does not send commands to the aircraft.

The **Overview** tab shows subsystem status, an approximate roll/pitch horizon,
sensor plots, and GPS fix/position details. **Transmitter** visualizes the
aircraft-received stick channels, switches, auxiliary controls, and trim-button
states. **Packets** shows decoded JSON lines and raw nRF24 payload bytes when
the ground station is the selected source.

The ground-station SSD1306 OLED rotates through radio/packet status, aircraft RC
inputs, sensor health, and GPS fix/position pages. It distinguishes a detected
radio module from aircraft telemetry actually arriving.

To isolate a blank display, upload the OLED-only test image:

```sh
pio run -d firmware/ground_station_f401 -e oled_test -t upload
```

It disables RF, scans I2C addresses `0x3C` and `0x3D`, displays a test pattern
when an OLED acknowledges, and reports detection over USB serial. Select Ground
Station mode in the app to see the detected address. If serial reports an OLED
address but the panel stays blank, check the panel resolution/controller variant.

The attitude display begins with a short stationary gyro/level calibration.
Keep the aircraft level and still while the counter runs; use **Calibrate level /
gyro** to repeat the process after moving or remounting the sensor. The same
command also captures the current barometer pressure as the zero-altitude
baseline; relative barometric altitude is shown next to GPS altitude in the
Overview panel and in the BMP280 status tile. The first valid pressure sample of
a session captures the baseline automatically.

## Map and mission planning

The **Map / Mission** tab shows the aircraft's live GPS position on an
OpenStreetMap tile map, its track history, home position, and the planned
mission:

- The first GPS fix of a session sets **home** automatically; you can also set
  it manually from the aircraft fix.
- **Click the map** to place a waypoint at the selected target altitude.
- The loiter circle around home shows where the aircraft would circle after
  returning.

The mission model (fly to the waypoint at altitude, return home, loiter)
is planned and visualized only. **The aircraft cannot execute it yet**: the
flight controller has no stabilization mode and no computer-to-aircraft
command uplink. Flying the mission requires those firmware stages first.

### Sending the mission to the aircraft

With the ground station connected, **Send mission to aircraft** uploads
home, the waypoint, and the loiter radius to the aircraft over the nRF24
command link; **Request START** records a start request and **Request ABORT**
clears it. The aircraft stores the mission and answers each command with an
acknowledgment frame, shown under the mission summary. This verifies the
ground-to-aircraft command path on the bench. START is only recorded —
the aircraft will not fly autonomously until stabilization and navigation
stages exist.

Map tiles are downloaded from OpenStreetMap and cached on disk, so the map needs
internet access on first view; overlays still work offline.

## Install and run

Python 3.10 or later is recommended.

```sh
cd ground_app
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py
```

Select the matching connection mode and serial port, then press **Connect**.
Both firmware paths use 115200 baud and newline-delimited JSON.

On Linux, if opening `/dev/ttyACM*` reports permission denied, ensure the user
belongs to the `dialout` group and start a fresh login session. Do not run the
application with `sudo`.

Run the data-model tests with:

```sh
python -m unittest discover -s tests -v
```

For a close-range RC bench test, build the transmitter's
`nanoatmega328_bench` environment to reduce RF output power. The aircraft must
use its normal BEC-powered firmware in ground-station mode. Do not connect
aircraft USB and BEC power at the same time.

## Attitude and units

The horizon view uses an approximate complementary filter from accelerometer
and gyro data. Magnetometer heading is not yet calibrated; treat displayed
orientation as a diagnostic visualization, not as flight-control output.
GPS coordinates, fix, satellite count, HDOP, altitude, speed, and course are
shown when provided by the receiver.
