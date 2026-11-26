#pragma once

#include <stdint.h>

namespace NanoLinkProtocol {

constexpr uint16_t MAGIC = 0x4E4C; // "NL"
constexpr uint8_t VERSION = 2;
constexpr uint8_t TYPE_SENSORS = 1;
constexpr uint8_t TYPE_CONTROL = 2;
constexpr uint8_t TYPE_GPS = 3;

constexpr uint8_t SENSOR_IMU_OK = 1u << 0;
constexpr uint8_t SENSOR_MAG_OK = 1u << 1;
constexpr uint8_t SENSOR_BARO_OK = 1u << 2;

constexpr uint8_t CONTROL_RC_VALID = 1u << 0;
constexpr uint8_t CONTROL_ARMED = 1u << 1;
constexpr uint8_t CONTROL_GPS_STREAM = 1u << 2;
constexpr uint8_t CONTROL_GPS_FIX = 1u << 3;

constexpr uint8_t GPS_STREAM_OK = 1u << 0;
constexpr uint8_t GPS_FIX_OK = 1u << 1;

constexpr uint16_t POT1_AUX_MASK = 0x03FF;
constexpr uint8_t POT1_TRIM_SHIFT = 12;
constexpr uint8_t TRIM_ELEVATOR_UP = 1u << 0;
constexpr uint8_t TRIM_ELEVATOR_DOWN = 1u << 1;
constexpr uint8_t TRIM_AILERON_RIGHT = 1u << 2;
constexpr uint8_t TRIM_AILERON_LEFT = 1u << 3;

struct __attribute__((packed)) Header {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t sequence;
};

// 31 bytes. Acceleration is mg, gyro is centidegrees/second, pressure is Pa,
// temperature is centidegrees C, and magnetometer values are uncalibrated counts.
struct __attribute__((packed)) SensorFrame {
  Header header;
  uint8_t flags;
  int16_t accelMg[3];
  int16_t gyroCentiDps[3];
  int16_t magCounts[3];
  uint32_t pressurePa;
  int16_t temperatureCentiC;
};

// RC values let the ground station display the inputs seen by the aircraft.
struct __attribute__((packed)) ControlFrame {
  Header header;
  uint8_t flags;
  uint16_t throttleUs;
  uint16_t aileronUs;
  uint16_t elevatorUs;
  uint16_t pot1;
  uint16_t pot2;
  uint8_t throttleMode;
  uint16_t rcAgeMs;
  uint16_t rcPackets;
  uint16_t telemetryTxOk;
  uint16_t telemetryTxFailed;
};

// Latitude/longitude are degrees * 1e7, altitude is millimeters,
// speed is centimeters/second, course and HDOP are scaled by 100.
struct __attribute__((packed)) GpsFrame {
  Header header;
  uint8_t flags;
  uint8_t fixType;
  uint8_t satellites;
  uint16_t hdopCenti;
  int32_t latitudeE7;
  int32_t longitudeE7;
  int32_t altitudeMm;
  uint16_t speedCms;
  uint16_t courseCentiDeg;
};

static_assert(sizeof(Header) == 6, "Telemetry header layout changed");
static_assert(sizeof(SensorFrame) <= 32, "Sensor frame exceeds nRF24 payload");
static_assert(sizeof(ControlFrame) <= 32, "Control frame exceeds nRF24 payload");
static_assert(sizeof(GpsFrame) <= 32, "GPS frame exceeds nRF24 payload");

} // namespace NanoLinkProtocol
