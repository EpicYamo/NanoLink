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

// Ground-station-to-aircraft commands (stage A: stored and acknowledged;
// mission execution requires future stabilization and navigation stages).
constexpr uint8_t TYPE_COMMAND = 4;
constexpr uint8_t TYPE_COMMAND_ACK = 5;

constexpr uint8_t CMD_SET_HOME = 1;
constexpr uint8_t CMD_SET_WAYPOINT = 2;
constexpr uint8_t CMD_SET_LOITER = 3;
constexpr uint8_t CMD_START_MISSION = 4;
constexpr uint8_t CMD_ABORT_MISSION = 5;
constexpr uint8_t CMD_CALIBRATE_LEVEL = 6;
constexpr uint8_t CMD_RTH = 7;  // draft: return-to-home request

constexpr uint8_t MISSION_NONE = 0;
constexpr uint8_t MISSION_LOADED = 1;
constexpr uint8_t MISSION_START_REQUESTED = 2;
constexpr uint8_t MISSION_RTH_REQUESTED = 3;  // draft: recorded only

constexpr uint8_t CMD_RESULT_OK = 0;
constexpr uint8_t CMD_RESULT_NOT_READY = 1;
constexpr uint8_t CMD_RESULT_UNKNOWN_OP = 2;

constexpr uint16_t POT1_AUX_MASK = 0x03FF;
constexpr uint8_t POT1_TRIM_SHIFT = 12;
// Bits 10 and 11 of pot1 carry the two throttle-stick trigger buttons.
constexpr uint8_t POT1_DUMP_TRIGGER_SHIFT = 10;
constexpr uint8_t POT1_RTH_TRIGGER_SHIFT = 11;
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

// 29 bytes. Acceleration is mg, gyro is centidegrees/second, pressure is Pa,
// temperature is centidegrees C. attitudeRoll/Pitch are the aircraft's own
// self-level estimate in centidegrees (0 = level), sent for bench diagnosis.
struct __attribute__((packed)) SensorFrame {
  Header header;
  uint8_t flags;
  int16_t accelMg[3];
  int16_t gyroCentiDps[3];
  int16_t attitudeRollCentiDeg;
  int16_t attitudePitchCentiDeg;
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

// Ground station -> aircraft. Payload layout depends on the opcode:
// CMD_SET_HOME:    int32 lat_e7, int32 lon_e7
// CMD_SET_WAYPOINT: int32 lat_e7, int32 lon_e7, int32 alt_mm
// CMD_SET_LOITER:   uint16 radius_m
struct __attribute__((packed)) CommandFrame {
  Header header;
  uint8_t opcode;
  uint8_t payload[24];
};

// Aircraft -> ground station, sent immediately after a command is accepted.
struct __attribute__((packed)) CommandAckFrame {
  Header header;
  uint8_t opcode;
  uint8_t result;
  uint8_t missionState;
  uint16_t commandSequence;
};

static_assert(sizeof(CommandFrame) <= 32, "Command frame exceeds nRF24 payload");
static_assert(sizeof(CommandAckFrame) <= 32, "Command ACK exceeds nRF24 payload");

} // namespace NanoLinkProtocol
