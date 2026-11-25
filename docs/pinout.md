# Pinout reference

This file records the existing control-radio wiring separately from the new
sensor and telemetry wiring. The transmitter and RF pin assignments are not
being changed in the first firmware stage.

## Arduino Nano transmitter

| Nano pin | Signal |
|---|---|
| D9 | nRF24 CE |
| D10 | nRF24 CSN |
| D11 | nRF24 MOSI |
| D12 | nRF24 MISO |
| D13 | nRF24 SCK |
| A0 | Elevator stick |
| A1 | Aileron stick |
| A2 | Throttle stick |
| A3 | Unused stick axis |
| A4 | Throttle-boost potentiometer |
| A5 | Spare potentiometer; known faulty/erratic, do not trust |
| A6 | Arm switch input |
| A7 | Throttle-mode switch input |
| D3 / D4 | Elevator trim up/down |
| D5 / D6 | Aileron trim right/left |
| D7 / D8 | Reserved trim inputs |

## STM32F401 aircraft receiver / flight controller

| F401 pin | Function | Notes |
|---|---|---|
| PB0 | nRF24 CE | Existing radio pin |
| PA4 | nRF24 CSN | Existing radio pin |
| PA5 | nRF24 SCK | SPI1 |
| PA6 | nRF24 MISO | SPI1 |
| PA7 | nRF24 MOSI | SPI1 |
| PB6 | Left elevon PWM | TIM4 channel 1 |
| PB7 | Right elevon PWM | TIM4 channel 2 |
| PB8 | ESC PWM | TIM4 channel 3 |
| PB3 | Sensor SDA | I2C2; optional SWO pin |
| PB10 | Sensor SCL | I2C2 |
| PC13 | Link LED | Active-low onboard LED |
| PA9 / PA10 | USART1 TX / RX | Not used for in-flight telemetry |

## STM32F401 ground station

| F401 pin | Function |
|---|---|
| PB0 | nRF24 CE |
| PA4 | nRF24 CSN |
| PA5 / PA6 / PA7 | nRF24 SPI1 SCK / MISO / MOSI |
| USB-C | USB serial connection to computer |

Both nRF24 modules use 3.3 V logic. The EBYTE E01-2G4M27D radio's VCC may be
powered from a stable 5 V BEC (2.5–5.5 V rated), but its CE/CSN/SPI pins are
3.3 V logic only. Keep the aircraft USB disconnected when BEC is powering the
F401 board.

For the common GY-91-style sensor breakout, connect its VIN/VCC input to the
F401 3V3 rail, GND to GND, SDA to PB3, and SCL to PB10. If the breakout exposes
a separate `3V3` pin, that is commonly the onboard regulator output, not the
power input. Check the markings on the actual board before soldering.
