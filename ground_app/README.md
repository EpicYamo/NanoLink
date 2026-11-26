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
