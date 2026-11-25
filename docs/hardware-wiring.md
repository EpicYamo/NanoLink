# Hardware wiring

This guide describes the reference NanoLink connections for the transmitter,
aircraft controller, sensors, and computer ground station.

## Arduino Nano transmitter

| Nano pin | Connection |
|---|---|
| D9 / D10 | nRF24 CE / CSN |
| D11 / D12 / D13 | nRF24 MOSI / MISO / SCK |
| A0 / A1 | Elevator / aileron stick |
| A2 | Throttle stick |
| A3 | Auxiliary stick axis (unused by the reference firmware) |
| A4 / A5 | Throttle-boost / optional auxiliary potentiometer |
| A6 / A7 | Arm switch / throttle-mode switch |
| D3 / D4 | Elevator trim up / down |
| D5 / D6 | Aileron trim right / left |
| D7 / D8 | Reserved switch or trim inputs |

The reference transmitter firmware sends the A5 auxiliary field as zero unless
`ENABLE_AUX_POT1` is enabled at build time. The aircraft firmware does not
currently act on that field.

## STM32F401 aircraft controller

| F401 pin | Connection |
|---|---|
| PB0 / PA4 | nRF24 CE / CSN |
| PA5 / PA6 / PA7 | nRF24 SPI1 SCK / MISO / MOSI |
| PB6 / PB7 | Left / right elevon PWM |
| PB8 | ESC PWM |
| PB3 / PB10 | Sensor I2C2 SDA / SCL |
| PC13 | Onboard link LED (active-low) |

The three PWM signal lines use open-drain outputs with external 2.2 kΩ
pull-up resistors to the ESC BEC's 5 V rail:

```text
BEC +5 V -- 2.2 kΩ -- PB6 / left elevon signal
BEC +5 V -- 2.2 kΩ -- PB7 / right elevon signal
BEC +5 V -- 2.2 kΩ -- PB8 / ESC signal
```

These are signal pull-ups, not servo power connections. The STM32 releases the
line for a high pulse and pulls it low for the low level; its GPIO never drives
5 V push-pull. Firmware enables this mode by default. Connect BEC ground and
STM32 ground together.

### MPU9250 + BMP280 module

| Sensor module | STM32F401 |
|---|---|
| VIN or VCC input | 3.3 V |
| GND | GND |
| SDA | PB3 |
| SCL | PB10 |

On common GY-91-style modules, `3V3` is the onboard regulator output; use the
module's `VIN`/`VCC` input. Check the markings on the specific breakout board.
The I2C bus uses 3.3 V logic. MPU9250 addresses are commonly `0x68` or `0x69`;
BMP280 addresses are commonly `0x76` or `0x77`. The firmware probes both common
BMP280 addresses.

## Optional GYGPSV1 NEO-8M GPS module

The GPS uses UART and does not share the sensor's I2C pins:

| GPS module | STM32F401 |
|---|---|
| VCC | 3.3 V (module input also supports 3–5 V) |
| GND | GND |
| TX | PA10 / USART1 RX |
| RX | PA9 / USART1 TX |

The module's default baud rate is 9600. For receiving position data only, GPS
TX, VCC, and GND are sufficient; GPS RX can remain disconnected. The current
aircraft firmware does not yet parse GPS data. Place the patch antenna facing
up with a clear view of the sky.

## nRF24 radio power and logic

The EBYTE E01-2G4M27D VCC input supports 2.5–5.5 V and may be powered from a
stable 5 V BEC. Its CE, CSN, and SPI signals use 3.3 V logic. The module can
draw substantial transmit-current bursts, so the supply must be stable and
adequately rated. Keep a common ground and place local decoupling close to the
radio.

## STM32F401 computer ground station

| F401 pin | Connection |
|---|---|
| PB0 / PA4 | nRF24 CE / CSN |
| PA5 / PA6 / PA7 | nRF24 SPI1 SCK / MISO / MOSI |
| USB-C | Computer USB serial connection |

The ground station can be USB-powered. Do not connect aircraft USB while the
aircraft board is powered from the ESC BEC; some Black Pill variants connect
USB VBUS directly to their 5 V rail.

## Build commands

Run these commands from the repository root:

```sh
pio run -d firmware/transmitter_nano
pio run -d firmware/aircraft_f401
pio run -d firmware/ground_station_f401
```
