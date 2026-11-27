#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <U8g2lib.h>
#include <string.h>

#include "../../common/telemetry_protocol.h"
#include "../../common/radio_init.h"

#ifndef OLED_DIAGNOSTIC
#define OLED_DIAGNOSTIC 0
#endif
#ifndef RADIO_DIAGNOSTIC
#define RADIO_DIAGNOSTIC 0
#endif

namespace {

constexpr uint8_t PIN_NRF_CE = PB0;
constexpr uint8_t PIN_NRF_CSN = PA4;
constexpr uint8_t RF_CHANNEL = 90;
constexpr uint32_t RF_SPI_HZ = 4000000UL;
constexpr uint32_t STATUS_PERIOD_MS = 1000;

const uint8_t GCS_ADDRESS[6] = "PC01A";
const uint8_t CMD_ADDRESS[6] = "CMND1";
const uint8_t RC_ADDRESS[6] = "NANO1";
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN, RF_SPI_HZ);

// Defined later in this file; declared here for the sniffer/command helpers.
void printBool(bool value);
void radioStartListening();
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

bool radioReady = false;
bool radioChipOk = false;
bool oledReady = false;
bool haveSequence = false;
bool haveSensorFrame = false;
bool haveControlFrame = false;
bool haveGpsFrame = false;
uint16_t lastSequence = 0;
uint32_t receivedPackets = 0;
uint32_t lostPackets = 0;
uint32_t autoAckCount = 0;
uint32_t lastPacketMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastOledMs = 0;
NanoLinkProtocol::SensorFrame latestSensor{};
NanoLinkProtocol::ControlFrame latestControl{};
NanoLinkProtocol::GpsFrame latestGps{};
uint8_t oledAddress = 0;
bool wireStarted = false;
uint32_t lastRadioAttemptMs = 0;
uint32_t lastOledAttemptMs = 0;

// --- computer-to-aircraft command bridge -------------------------------------
char inputLine[96]{};
uint8_t inputLength = 0;
uint16_t commandSequence = 0;

const char *opcodeName(uint8_t opcode) {
  switch (opcode) {
    case NanoLinkProtocol::CMD_SET_HOME: return "set_home";
    case NanoLinkProtocol::CMD_SET_WAYPOINT: return "set_waypoint";
    case NanoLinkProtocol::CMD_SET_LOITER: return "set_loiter";
    case NanoLinkProtocol::CMD_START_MISSION: return "start_mission";
    case NanoLinkProtocol::CMD_ABORT_MISSION: return "abort_mission";
    case NanoLinkProtocol::CMD_CALIBRATE_LEVEL: return "calibrate_level";
    default: return "unknown";
  }
}

bool parseInt32(const char *text, int32_t *out) {
  if (!text || !*text) return false;
  bool negative = false;
  if (*text == '-') { negative = true; ++text; }
  else if (*text == '+') { ++text; }
  if (!*text) return false;
  int32_t value = 0;
  for (; *text; ++text) {
    if (*text < '0' || *text > '9') return false;
    value = value * 10 + (*text - '0');
  }
  *out = negative ? -value : value;
  return true;
}

void printCommandStatus(const char *op, uint16_t seq, bool sent) {
  Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"command\",\"op\":\""));
  Serial.print(op);
  Serial.print(F("\",\"cmd_seq\":"));
  Serial.print(seq);
  Serial.print(F(",\"status\":\""));
  Serial.print(sent ? F("sent") : F("failed"));
  Serial.println(F("\"}"));
}

void sendCommand(uint8_t opcode, const uint8_t *payload) {
  const uint16_t seq = commandSequence++;
  if (!radioReady) {
    printCommandStatus(opcodeName(opcode), seq, false);
    return;
  }

  NanoLinkProtocol::CommandFrame frame{};
  frame.header.magic = NanoLinkProtocol::MAGIC;
  frame.header.version = NanoLinkProtocol::VERSION;
  frame.header.type = NanoLinkProtocol::TYPE_COMMAND;
  frame.header.sequence = seq;
  frame.opcode = opcode;
  memcpy(frame.payload, payload, sizeof(frame.payload));

  radio.stopListening();
  radio.openWritingPipe(CMD_ADDRESS);
  radio.setRetries(2, 5);
  const bool sent = radio.write(&frame, sizeof(frame));
  radio.setRetries(0, 0);
  radioStartListening();

  printCommandStatus(opcodeName(opcode), seq, sent);
}

void handleCommandLine(char *line) {
  // Expected: CMD,set_home,lat_e7,lon_e7 | CMD,set_waypoint,lat,lon,alt_mm |
  //           CMD,set_loiter,radius_m | CMD,start_mission | CMD,abort_mission
  char *fields[6]{};
  uint8_t count = 0;
  fields[count++] = line;
  for (char *p = line; *p && count < 6; ++p) {
    if (*p == ',') {
      *p = '\0';
      fields[count++] = p + 1;
    }
  }
  if (count < 2 || strcmp(fields[0], "CMD") != 0) return;

  uint8_t payload[24]{};
  const char *op = fields[1];
  int32_t a = 0, b = 0, c = 0;

  if (strcmp(op, "set_home") == 0 && count >= 4 &&
      parseInt32(fields[2], &a) && parseInt32(fields[3], &b)) {
    memcpy(payload + 0, &a, 4);
    memcpy(payload + 4, &b, 4);
    sendCommand(NanoLinkProtocol::CMD_SET_HOME, payload);
  } else if (strcmp(op, "set_waypoint") == 0 && count >= 5 &&
             parseInt32(fields[2], &a) && parseInt32(fields[3], &b) &&
             parseInt32(fields[4], &c)) {
    memcpy(payload + 0, &a, 4);
    memcpy(payload + 4, &b, 4);
    memcpy(payload + 8, &c, 4);
    sendCommand(NanoLinkProtocol::CMD_SET_WAYPOINT, payload);
  } else if (strcmp(op, "set_loiter") == 0 && count >= 3 &&
             parseInt32(fields[2], &a)) {
    memcpy(payload + 0, &a, 2);
    sendCommand(NanoLinkProtocol::CMD_SET_LOITER, payload);
  } else if (strcmp(op, "start_mission") == 0) {
    sendCommand(NanoLinkProtocol::CMD_START_MISSION, payload);
  } else if (strcmp(op, "abort_mission") == 0) {
    sendCommand(NanoLinkProtocol::CMD_ABORT_MISSION, payload);
  } else if (strcmp(op, "calibrate_level") == 0) {
    sendCommand(NanoLinkProtocol::CMD_CALIBRATE_LEVEL, payload);
  }
}

void pollSerialInput() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      inputLine[inputLength] = '\0';
      inputLength = 0;
      handleCommandLine(inputLine);
    } else if (c != '\r' && inputLength < sizeof(inputLine) - 1) {
      inputLine[inputLength++] = c;
    } else if (c != '\r') {
      inputLength = 0; // line too long: reset and wait for the next one
    }
  }
}
// -----------------------------------------------------------------------------

// --- transmitter RC sniffer (bench diagnostic) -------------------------------
// Pipe 0 also listens on the RC address so the handheld transmitter can be
// verified against the ground station directly, without the aircraft.
struct __attribute__((packed)) RcPacket {
  uint16_t throttle;
  uint16_t aileron;
  uint16_t elevator;
  uint8_t armed;
  uint16_t pot1;
  uint16_t pot2;
  uint8_t switch2State;
};
static_assert(sizeof(RcPacket) == 12, "RC packet must match the Nano transmitter");

RcPacket latestSniff{};
uint32_t sniffPackets = 0;
uint32_t lastSniffMs = 0;
uint32_t lastSniffEmitMs = 0;

void radioStartListening() {
  radio.openReadingPipe(0, RC_ADDRESS);   // bench RC sniffer pipe
  radio.openReadingPipe(1, GCS_ADDRESS);  // aircraft telemetry pipe
  radio.startListening();
}

void handleRcSniff() {
  RcPacket sniff{};
  radio.read(&sniff, sizeof(sniff));
  if (sniff.throttle < 900 || sniff.throttle > 2100 ||
      sniff.aileron < 900 || sniff.aileron > 2100 ||
      sniff.elevator < 900 || sniff.elevator > 2100 ||
      sniff.armed > 1 || sniff.switch2State > 2) {
    return; // not a plausible Nano packet; ignore
  }
  ++sniffPackets;
  latestSniff = sniff;
  lastSniffMs = millis();
  if (millis() - lastSniffEmitMs < 200) return; // ~5 Hz reporting over USB

  lastSniffEmitMs = millis();
  Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"rc_sniff\",\"seq\":"));
  Serial.print(sniffPackets);
  Serial.print(F(",\"throttle_us\":")); Serial.print(sniff.throttle);
  Serial.print(F(",\"aileron_us\":")); Serial.print(sniff.aileron);
  Serial.print(F(",\"elevator_us\":")); Serial.print(sniff.elevator);
  Serial.print(F(",\"armed\":")); printBool(sniff.armed);
  Serial.print(F(",\"switch2\":")); Serial.print(sniff.switch2State);
  Serial.print(F(",\"pot1\":")); Serial.print(sniff.pot1);
  Serial.print(F(",\"pot2\":")); Serial.print(sniff.pot2);
  Serial.println('}');
}
// -----------------------------------------------------------------------------

void beginOled() {
  if (!wireStarted) {
    Wire.begin(); // F401 I2C1 default: SDA PB7, SCL PB6.
    Wire.setClock(100000);
    wireStarted = true;
  }

  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() == 0) oledAddress = 0x3C;
  else {
    Wire.beginTransmission(0x3D);
    if (Wire.endTransmission() == 0) oledAddress = 0x3D;
  }
  if (!oledAddress) return;

  oled.setI2CAddress(static_cast<uint8_t>(oledAddress << 1));
  oled.begin();
  oled.setFont(u8g2_font_6x10_tf);
  oledReady = true;
}

// Flush the display. (U8g2's sendBuffer() returns void, so health is handled
// by the periodic beginOled() re-probe when the device never came up.)
void oledFlush() {
  oled.sendBuffer();
}

void renderBootScreen() {
  if (!oledReady) return;
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(22, 30, "NanoLink GCS");
  oled.drawStr(10, 52, "Waiting for aircraft");
  oledFlush();
}

void renderOled() {
  if (!oledReady || millis() - lastOledMs < 400) return;
  lastOledMs = millis();

#if OLED_DIAGNOSTIC
  oled.clearBuffer();
  oled.drawFrame(0, 0, 128, 64);
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(8, 14, "OLED TEST: FOUND");
  oled.setCursor(8, 29); oled.print(F("SSD1306 128x64"));
  oled.setCursor(8, 43); oled.print(F("I2C address 0x")); oled.print(oledAddress, HEX);
  oled.drawBox((millis() / 20) % 112 + 8, 52, 8, 6);
  oledFlush();
  return;
#endif

  const uint8_t page = (millis() / 2500) % 5;

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  if (page == 0) {
    oled.drawStr(0, 10, "NanoLink Ground");
    const bool airLink = receivedPackets && (millis() - lastPacketMs < 1500);
    oled.setCursor(0, 28);
    oled.print(F("AIRCRAFT: "));
    oled.print(airLink ? F("CONNECTED") : F("WAITING"));
    oled.setCursor(0, 46);
    oled.print(F("RADIO: "));
    oled.print(radioReady && radioChipOk ? F("READY") : F("ERROR"));
    oled.setCursor(0, 63);
    oled.print(F("RX ")); oled.print(receivedPackets);
    oled.print(F(" LOST ")); oled.print(lostPackets);
    oled.print(F(" ACK ")); oled.print(autoAckCount);
  } else if (page == 1) {
    oled.drawStr(0, 10, "Aircraft RC Inputs");
    if (!haveControlFrame) {
      oled.drawStr(0, 30, "No aircraft packets");
      oled.drawStr(0, 44, "RC data unavailable");
    } else {
      const bool rcOk = latestControl.flags & NanoLinkProtocol::CONTROL_RC_VALID;
      oled.setCursor(0, 24); oled.print(F("RC: ")); oled.print(rcOk ? F("OK") : F("LOST"));
      oled.print(F(" AGE ")); oled.print(latestControl.rcAgeMs); oled.print(F("ms"));
      oled.setCursor(0, 38); oled.print(F("T ")); oled.print(latestControl.throttleUs);
      oled.print(F(" ARM ")); oled.print((latestControl.flags & NanoLinkProtocol::CONTROL_ARMED) ? F("ON") : F("OFF"));
      oled.setCursor(0, 52); oled.print(F("A ")); oled.print(latestControl.aileronUs);
      oled.print(F(" E ")); oled.print(latestControl.elevatorUs);
      const uint8_t mode = latestControl.throttleMode;
      oled.setCursor(0, 63); oled.print(F("MODE "));
      oled.print(mode == 0 ? F("MANUAL") : F("STABIL"));
      oled.print(F(" RATE ")); oled.print(latestControl.pot2 / 10); oled.print(F("%"));
    }
  } else if (page == 2) {
    oled.drawStr(0, 10, "Aircraft Sensors");
    oled.setCursor(0, 24); oled.print(F("IMU: "));
    oled.print(haveSensorFrame && (latestSensor.flags & NanoLinkProtocol::SENSOR_IMU_OK) ? F("OK") : F("NO DATA"));
    oled.print(F(" MAG: "));
    oled.print(haveSensorFrame && (latestSensor.flags & NanoLinkProtocol::SENSOR_MAG_OK) ? F("OK") : F("N/A"));
    oled.setCursor(0, 38); oled.print(F("BARO: "));
    oled.print(haveSensorFrame && (latestSensor.flags & NanoLinkProtocol::SENSOR_BARO_OK) ? F("OK") : F("NO DATA"));
    if (haveSensorFrame) {
      oled.print(F(" ")); oled.print(latestSensor.temperatureCentiC / 100.0f, 1); oled.print(F("C"));
    }
    const bool gpsStream = haveGpsFrame && (latestGps.flags & NanoLinkProtocol::GPS_STREAM_OK);
    const bool gpsFix = haveGpsFrame && (latestGps.flags & NanoLinkProtocol::GPS_FIX_OK);
    oled.setCursor(0, 52); oled.print(F("GPS: "));
    oled.print(gpsFix ? F("FIX") : (gpsStream ? F("SEARCH") : (haveGpsFrame ? F("NO NMEA") : F("UNKNOWN"))));
    oled.print(F(" SAT ")); oled.print(haveGpsFrame ? latestGps.satellites : 0);
    oled.setCursor(0, 63); oled.print(F("HDOP "));
    if (haveGpsFrame) oled.print(latestGps.hdopCenti / 100.0f, 2);
    else oled.print(F("--"));
  } else if (page == 3) {
    oled.drawStr(0, 10, "GPS Position");
    const bool gpsFix = haveGpsFrame && (latestGps.flags & NanoLinkProtocol::GPS_FIX_OK);
    if (!gpsFix) {
      oled.drawStr(0, 30, haveGpsFrame ? "Waiting for GPS fix" : "No aircraft GPS data");
      oled.setCursor(0, 48); oled.print(F("Satellites "));
      oled.print(haveGpsFrame ? latestGps.satellites : 0);
    } else {
      oled.setCursor(0, 27); oled.print(F("Lat ")); oled.print(latestGps.latitudeE7 / 10000000.0f, 5);
      oled.setCursor(0, 41); oled.print(F("Lon ")); oled.print(latestGps.longitudeE7 / 10000000.0f, 5);
      oled.setCursor(0, 55); oled.print(F("Alt ")); oled.print(latestGps.altitudeMm / 1000.0f, 1);
      oled.print(F("m Sat ")); oled.print(latestGps.satellites);
    }
  } else {
    oled.drawStr(0, 10, "RF Diagnostics");
    oled.setCursor(0, 24);
    oled.print(F("RX ")); oled.print(receivedPackets);
    oled.print(F(" LOST ")); oled.print(lostPackets);
    oled.setCursor(0, 40);
    oled.print(F("ACK ")); oled.print(autoAckCount);
    const bool sniffLive = sniffPackets && (millis() - lastSniffMs < 1000);
    oled.setCursor(0, 55);
    oled.print(F("TX SNIF: "));
    oled.print(sniffLive ? F("LIVE") : F("NONE"));
    oled.print(F(" "));
    oled.print(sniffPackets);
  }
  oledFlush();
}

void printBool(bool value) { Serial.print(value ? F("true") : F("false")); }

void printCommon(uint8_t kind, uint16_t sequence) {
  Serial.print(F("{\"source\":\"ground_station\",\"kind\":"));
  if (kind == NanoLinkProtocol::TYPE_SENSORS) Serial.print(F("\"sensor\""));
  else if (kind == NanoLinkProtocol::TYPE_CONTROL) Serial.print(F("\"control\""));
  else if (kind == NanoLinkProtocol::TYPE_COMMAND_ACK) Serial.print(F("\"command_ack\""));
  else Serial.print(F("\"gps\""));
  Serial.print(F(",\"seq\":")); Serial.print(sequence);
  Serial.print(F(",\"rx_packets\":")); Serial.print(receivedPackets);
  Serial.print(F(",\"lost_packets\":")); Serial.print(lostPackets);
  Serial.print(F(",\"auto_ack_sent\":true,\"auto_ack_count\":")); Serial.print(autoAckCount);
  Serial.print(F(",\"raw_hex\":\""));
}

void printHex(const uint8_t *data, uint8_t length) {
  const char hex[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < length; ++i) {
    Serial.write(hex[data[i] >> 4]);
    Serial.write(hex[data[i] & 0x0F]);
  }
  Serial.print('"');
}

void printSensorFrame(const NanoLinkProtocol::SensorFrame &frame, const uint8_t *raw) {
  latestSensor = frame;
  haveSensorFrame = true;
  printCommon(NanoLinkProtocol::TYPE_SENSORS, frame.header.sequence);
  printHex(raw, 32);
  Serial.print(F(",\"flags\":")); Serial.print(frame.flags);
  Serial.print(F(",\"imu_ok\":")); printBool(frame.flags & NanoLinkProtocol::SENSOR_IMU_OK);
  Serial.print(F(",\"mag_ok\":")); printBool(frame.flags & NanoLinkProtocol::SENSOR_MAG_OK);
  Serial.print(F(",\"baro_ok\":")); printBool(frame.flags & NanoLinkProtocol::SENSOR_BARO_OK);
  Serial.print(F(",\"accel_mg\":["));
  Serial.print(frame.accelMg[0]); Serial.print(','); Serial.print(frame.accelMg[1]); Serial.print(','); Serial.print(frame.accelMg[2]);
  Serial.print(F("],\"gyro_cdeg_s\":["));
  Serial.print(frame.gyroCentiDps[0]); Serial.print(','); Serial.print(frame.gyroCentiDps[1]); Serial.print(','); Serial.print(frame.gyroCentiDps[2]);
  Serial.print(F("],\"mag_counts\":["));
  Serial.print(frame.magCounts[0]); Serial.print(','); Serial.print(frame.magCounts[1]); Serial.print(','); Serial.print(frame.magCounts[2]);
  Serial.print(F("],\"pressure_pa\":")); Serial.print(frame.pressurePa);
  Serial.print(F(",\"temperature_centi_c\":")); Serial.print(frame.temperatureCentiC);
  Serial.println('}');
}

void printControlFrame(const NanoLinkProtocol::ControlFrame &frame, const uint8_t *raw) {
  latestControl = frame;
  haveControlFrame = true;
  printCommon(NanoLinkProtocol::TYPE_CONTROL, frame.header.sequence);
  printHex(raw, 32);
  Serial.print(F(",\"flags\":")); Serial.print(frame.flags);
  Serial.print(F(",\"rc_valid\":")); printBool(frame.flags & NanoLinkProtocol::CONTROL_RC_VALID);
  Serial.print(F(",\"armed\":")); printBool(frame.flags & NanoLinkProtocol::CONTROL_ARMED);
  Serial.print(F(",\"gps_stream\":")); printBool(frame.flags & NanoLinkProtocol::CONTROL_GPS_STREAM);
  Serial.print(F(",\"gps_fix\":")); printBool(frame.flags & NanoLinkProtocol::CONTROL_GPS_FIX);
  Serial.print(F(",\"throttle_us\":")); Serial.print(frame.throttleUs);
  Serial.print(F(",\"aileron_us\":")); Serial.print(frame.aileronUs);
  Serial.print(F(",\"elevator_us\":")); Serial.print(frame.elevatorUs);
  Serial.print(F(",\"aux1\":")); Serial.print(frame.pot1 & NanoLinkProtocol::POT1_AUX_MASK);
  Serial.print(F(",\"trim_flags\":")); Serial.print((frame.pot1 >> NanoLinkProtocol::POT1_TRIM_SHIFT) & 0x0F);
  Serial.print(F(",\"throttle_boost\":")); Serial.print(frame.pot2);
  Serial.print(F(",\"throttle_mode\":")); Serial.print(frame.throttleMode);
  Serial.print(F(",\"rc_age_ms\":")); Serial.print(frame.rcAgeMs);
  Serial.print(F(",\"rc_packets\":")); Serial.print(frame.rcPackets);
  Serial.print(F(",\"air_tx_ok\":")); Serial.print(frame.telemetryTxOk);
  Serial.print(F(",\"air_tx_failed\":")); Serial.print(frame.telemetryTxFailed);
  Serial.println('}');
}

void printGpsFrame(const NanoLinkProtocol::GpsFrame &frame, const uint8_t *raw) {
  latestGps = frame;
  haveGpsFrame = true;
  printCommon(NanoLinkProtocol::TYPE_GPS, frame.header.sequence);
  printHex(raw, 32);
  Serial.print(F(",\"flags\":")); Serial.print(frame.flags);
  Serial.print(F(",\"stream_ok\":")); printBool(frame.flags & NanoLinkProtocol::GPS_STREAM_OK);
  Serial.print(F(",\"fix_ok\":")); printBool(frame.flags & NanoLinkProtocol::GPS_FIX_OK);
  Serial.print(F(",\"fix_type\":")); Serial.print(frame.fixType);
  Serial.print(F(",\"satellites\":")); Serial.print(frame.satellites);
  Serial.print(F(",\"hdop_centi\":")); Serial.print(frame.hdopCenti);
  Serial.print(F(",\"lat_e7\":")); Serial.print(frame.latitudeE7);
  Serial.print(F(",\"lon_e7\":")); Serial.print(frame.longitudeE7);
  Serial.print(F(",\"alt_mm\":")); Serial.print(frame.altitudeMm);
  Serial.print(F(",\"speed_cms\":")); Serial.print(frame.speedCms);
  Serial.print(F(",\"course_centi_deg\":")); Serial.print(frame.courseCentiDeg);
  Serial.println('}');
}

void printCommandAckFrame(const NanoLinkProtocol::CommandAckFrame &frame, const uint8_t *raw) {
  printCommon(NanoLinkProtocol::TYPE_COMMAND_ACK, frame.header.sequence);
  printHex(raw, 32);
  Serial.print(F(",\"op\":\""));
  Serial.print(opcodeName(frame.opcode));
  Serial.print(F("\",\"cmd_seq\":"));
  Serial.print(frame.commandSequence);
  Serial.print(F(",\"result\":"));
  Serial.print(frame.result);
  Serial.print(F(",\"mission_state\":"));
  Serial.print(frame.missionState);
  Serial.println('}');
}

void noteSequence(uint16_t sequence) {
  if (haveSequence) {
    const uint16_t delta = static_cast<uint16_t>(sequence - lastSequence);
    if (delta > 1 && delta < 0x8000) lostPackets += delta - 1;
  }
  lastSequence = sequence;
  haveSequence = true;
  ++receivedPackets;
  ++autoAckCount; // nRF24 hardware auto-ACK is enabled for this receive pipe.
  lastPacketMs = millis();
}

void emitStatus() {
  Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"link\",\"radio_ready\":"));
#if OLED_DIAGNOSTIC
  printBool(false);
  Serial.print(F(",\"radio_chip_ok\":false,\"oled_test_mode\":true"));
#else
  printBool(radioReady);
  Serial.print(F(",\"radio_chip_ok\":")); printBool(radioChipOk);
#endif
  Serial.print(F(",\"oled_ready\":")); printBool(oledReady);
  Serial.print(F(",\"oled_addr\":")); Serial.print(oledAddress);
  Serial.print(F(",\"rx_packets\":")); Serial.print(receivedPackets);
  Serial.print(F(",\"lost_packets\":")); Serial.print(lostPackets);
  Serial.print(F(",\"auto_ack_count\":")); Serial.print(autoAckCount);
  Serial.print(F(",\"rc_sniff_packets\":")); Serial.print(sniffPackets);
  Serial.print(F(",\"rc_sniff_age_ms\":"));
  Serial.print(sniffPackets ? (millis() - lastSniffMs) : 65535UL);
  Serial.print(F(",\"telemetry_age_ms\":"));
  Serial.print(receivedPackets == 0 ? 65535UL : millis() - lastPacketMs);
  Serial.println('}');
}

void handleRadioPacket() {
  uint8_t raw[32]{};
  radio.read(raw, sizeof(raw));

  NanoLinkProtocol::Header header{};
  memcpy(&header, raw, sizeof(header));
  if (header.magic != NanoLinkProtocol::MAGIC || header.version != NanoLinkProtocol::VERSION) return;
  noteSequence(header.sequence);

  switch (header.type) {
    case NanoLinkProtocol::TYPE_SENSORS: {
      NanoLinkProtocol::SensorFrame frame{};
      memcpy(&frame, raw, sizeof(frame));
      printSensorFrame(frame, raw);
      break;
    }
    case NanoLinkProtocol::TYPE_CONTROL: {
      NanoLinkProtocol::ControlFrame frame{};
      memcpy(&frame, raw, sizeof(frame));
      printControlFrame(frame, raw);
      break;
    }
    case NanoLinkProtocol::TYPE_GPS: {
      NanoLinkProtocol::GpsFrame frame{};
      memcpy(&frame, raw, sizeof(frame));
      printGpsFrame(frame, raw);
      break;
    }
    case NanoLinkProtocol::TYPE_COMMAND_ACK: {
      NanoLinkProtocol::CommandAckFrame frame{};
      memcpy(&frame, raw, sizeof(frame));
      printCommandAckFrame(frame, raw);
      break;
    }
    default:
      break;
  }
}

} // namespace

// --- radio diagnostic build --------------------------------------------------
// Stage-by-stage OLED/serial markers that pinpoint exactly where the normal
// build freezes on a board. Waits for the USB host first so the application
// records every stage; the raw SPI probe reports the radio's status byte, and
// the receive loop is enabled only after 10 seconds so a frozen counter
// cleanly separates "init hang" from "receive-loop spin".
#if RADIO_DIAGNOSTIC
uint32_t diagCounter = 0;
uint32_t diagLastDrawMs = 0;
uint32_t diagLastStatusMs = 0;
bool diagBeginOk = false;
bool diagChipOk = false;
bool diagRxOn = false;
bool diagHalted = false;

void diagSerial(const char *stage) {
  Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\""));
  Serial.print(stage);
  Serial.println(F("\"}"));
}

void diagDraw(const char *line, bool showCounter) {
  if (!oledReady) return;
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 14, "RADIO DIAG");
  oled.drawStr(0, 36, line);
  if (showCounter) {
    oled.setCursor(0, 58);
    oled.print(F("LOOP "));
    oled.print(diagCounter);
    if (diagRxOn) oled.print(F(" RX ON"));
    if (sniffPackets) {
      oled.print(F(" S"));
      oled.print(sniffPackets);
    }
  }
  oledFlush();
}

// Bit-banged SPI probe: bypasses the SPI peripheral entirely. Pure GPIO
// (already proven healthy by the GPIO check), so it cannot hang. If the
// module answers here but hardware SPI doesn't, the SPI peripheral or its
// driver is the fault; if bit-bang also reads silence, the module is dead.
uint8_t bitbangSpiProbe() {
  pinMode(PA5, OUTPUT);          // SCK
  pinMode(PA6, INPUT);           // MISO
  pinMode(PA7, OUTPUT);          // MOSI
  pinMode(PIN_NRF_CSN, OUTPUT);
  digitalWrite(PA5, LOW);
  digitalWrite(PIN_NRF_CSN, LOW);
  delayMicroseconds(5);
  uint8_t status = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    digitalWrite(PA7, HIGH);     // NOP = 0xFF, all bits high
    delayMicroseconds(5);
    digitalWrite(PA5, HIGH);
    delayMicroseconds(5);
    status = (status << 1) | (digitalRead(PA6) ? 1 : 0);
    digitalWrite(PA5, LOW);
    delayMicroseconds(5);
  }
  digitalWrite(PIN_NRF_CSN, HIGH);
  return status;
}

void radioDiagSetup() {
  // Wait for the USB host so the app can record every stage line.
  diagDraw("0 WAIT USB", false);
  const uint32_t usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 20000) delay(10);
  diagSerial("usb_ready");

  // GPIO self-test first — before SPI touches any pin mode.
  diagDraw("1 GPIO CHECK", false);
  diagSerial("gpio_check");
  {
    struct DiagPinDef { uint32_t arduinoPin; GPIO_TypeDef *port; uint32_t mask; const char *name; };
    const DiagPinDef pins[] = {
      { PIN_NRF_CE,  GPIOB, 1u << 0, "CE" },
      { PIN_NRF_CSN, GPIOA, 1u << 4, "CSN" },
      { PA5, GPIOA, 1u << 5, "SCK" },
      { PA6, GPIOA, 1u << 6, "MISO" },
      { PA7, GPIOA, 1u << 7, "MOSI" },
    };
    uint8_t gpioFaults = 0;
    for (uint8_t i = 0; i < 5; ++i) {
      const DiagPinDef &p = pins[i];
      pinMode(p.arduinoPin, OUTPUT);
      digitalWrite(p.arduinoPin, LOW);
      delay(2);
      const bool readLow = (p.port->IDR & p.mask) != 0;
      digitalWrite(p.arduinoPin, HIGH);
      delay(2);
      const bool readHigh = (p.port->IDR & p.mask) != 0;
      const bool ok = !readLow && readHigh;
      if (!ok) ++gpioFaults;
      Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\"gpio_pin\",\"pin\":\""));
      Serial.print(p.name);
      Serial.print(F("\",\"low_read\":")); printBool(readLow);
      Serial.print(F(",\"high_read\":")); printBool(readHigh);
      Serial.print(F(",\"ok\":")); printBool(ok);
      Serial.println('}');
    }
    if (gpioFaults) {
      diagSerial("gpio_fault");
      diagDraw("1 PIN FAULT", false);
      diagHalted = true;
      return;
    }
    diagSerial("gpio_ok");
  }

  // Bit-bang probe — no SPI peripheral involvement, cannot hang.
  diagDraw("2 BITBANG", false);
  diagSerial("bitbang_probe");
  const uint8_t bitbangStatus = bitbangSpiProbe();
  Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\"bitbang_status\",\"status\":"));
  Serial.print(bitbangStatus);
  Serial.println('}');
  delay(500);

  // Raw SPI1 register probe — completely bypasses HAL and Arduino SPI class.
  // This tells us exactly which layer is broken: if the raw registers work,
  // the bug is in the HAL/Arduino layer. If they don't, the peripheral itself
  // is dead.
  diagDraw("3 RAW SPI1", false);
  diagSerial("raw_spi1");
  {
    // Enable SPI1 peripheral clock (bit 12 of RCC_APB2ENR)
    RCC->APB2ENR |= (1u << 12);
    delay(1);

    const uint32_t cr1Before = SPI1->CR1;  // before configuring

    // Configure SPI1: Master, Mode 0, 8-bit, prescaler /32 (~2.6MHz)
    SPI1->CR1 = 0;  // Reset
    SPI1->CR1 = SPI_CR1_MSTR   // Master mode
              | SPI_CR1_SSM    // Software slave management
              | SPI_CR1_SSI    // Internal slave select (drive NSS high)
              | (5u << 3)      // Baud rate prescaler: 101 = /32
              | SPI_CR1_SPE;   // SPI enable
    delay(1);

    const uint32_t cr1After = SPI1->CR1;  // verify write stuck
    const uint32_t srAfter = SPI1->SR;    // TXE should be set now

    Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\"raw_spi1_regs\",\"cr1_before\":"));
    Serial.print(cr1Before);
    Serial.print(F(",\"cr1_after\":"));
    Serial.print(cr1After);
    Serial.print(F(",\"sr_after\":"));
    Serial.print(srAfter);
    Serial.print(F(",\"spe_stuck\":"));
    printBool((cr1After & SPI_CR1_SPE) != 0);
    Serial.print(F(",\"txe_set\":"));
    printBool((srAfter & SPI_SR_TXE) != 0);
    Serial.print('}');
    Serial.println();

    if (cr1After & SPI_CR1_SPE) {
      // SPE stuck → peripheral is alive at the register level.
      // Try a transfer: CSN low, write NOP, read status.
      digitalWrite(PIN_NRF_CSN, LOW);
      delayMicroseconds(5);
      int timeout = 10000;
      while (!(SPI1->SR & SPI_SR_TXE) && --timeout);
      if (timeout > 0) {
        SPI1->DR = 0xFF;  // NOP command
        timeout = 10000;
        while (!(SPI1->SR & SPI_SR_RXNE) && --timeout);
        const uint8_t rawStatus = (timeout > 0) ? (uint8_t)(SPI1->DR) : 0;
        digitalWrite(PIN_NRF_CSN, HIGH);
        Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\"raw_spi1_status\",\"status\":"));
        Serial.print(rawStatus);
        Serial.println('}');
      } else {
        digitalWrite(PIN_NRF_CSN, HIGH);
        diagSerial("raw_spi1_txe_timeout");
      }
      // Disable SPI1 to leave clean state for the Arduino SPI test
      SPI1->CR1 &= ~SPI_CR1_SPE;
    } else {
      diagSerial("raw_spi1_spe_failed");
    }
  }
  delay(500);

  // Hardware SPI from here on.
  diagDraw("4 SPI BEGIN", false);
  diagSerial("spi_begin");
  SPI.begin();
  diagSerial("spi_begin_done");

  // Check what the Arduino SPI actually configured, and fix it if broken.
  {
    const uint32_t cr1AfterBegin = SPI1->CR1;
    const uint32_t srAfterBegin = SPI1->SR;
    Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\"after_begin\",\"cr1\":"));
    Serial.print(cr1AfterBegin);
    Serial.print(F(",\"sr\":"));
    Serial.print(srAfterBegin);
    Serial.print(F(",\"txe\":"));
    printBool((srAfterBegin & SPI_SR_TXE) != 0);
    Serial.println('}');

    if (!(srAfterBegin & SPI_SR_TXE)) {
      // Arduino SPI left SPI1 in a broken state. Force-fix via raw registers
      // (proven to work by the raw_spi1 stage above).
      diagSerial("spi_fixup");
      RCC->APB2ENR |= (1u << 12);
      // PA5=SCK, PA6=MISO, PA7=MOSI → AF5
      GPIOA->MODER &= ~((3u << 10) | (3u << 12) | (3u << 14));
      GPIOA->MODER |=  ((2u << 10) | (2u << 12) | (2u << 14));
      GPIOA->AFR[0] &= ~((0xFu << 20) | (0xFu << 24) | (0xFu << 28));
      GPIOA->AFR[0] |=  ((5u << 20) | (5u << 24) | (5u << 28));
      SPI1->CR1 = 0;
      SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (5u << 3) | SPI_CR1_SPE;
      const uint32_t srFixed = SPI1->SR;
      Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\"after_fixup\",\"sr\":"));
      Serial.print(srFixed);
      Serial.print(F(",\"txe\":"));
      printBool((srFixed & SPI_SR_TXE) != 0);
      Serial.println('}');
    }
  }

  // Test 1: Raw register transfer (proven to work earlier)
  diagSerial("raw_after_begin");
  {
    digitalWrite(PIN_NRF_CSN, LOW);
    delayMicroseconds(5);
    Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\"pre_raw\",\"cr1\":"));
    Serial.print(SPI1->CR1);
    Serial.print(F(",\"sr\":"));
    Serial.print(SPI1->SR);
    Serial.println('}');
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = 0xFF;
    uint32_t timeout = 10000;
    while (!(SPI1->SR & SPI_SR_RXNE) && --timeout);
    const uint8_t rawAfterBegin = (timeout > 0) ? (uint8_t)(SPI1->DR) : 0;
    digitalWrite(PIN_NRF_CSN, HIGH);
    Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\"raw_after_begin\",\"status\":"));
    Serial.print(rawAfterBegin);
    Serial.println('}');
  }
  delay(500);

  // Test 2: Arduino SPI.transfer() (the one that hangs)
  diagDraw("5A NO XACT", false);
  diagSerial("probe_no_xact");
  {
    digitalWrite(PIN_NRF_CSN, LOW);
    delayMicroseconds(5);
    const uint8_t statusNoXact = SPI.transfer(0xFF);
    digitalWrite(PIN_NRF_CSN, HIGH);
    Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\"no_xact_status\",\"status\":"));
    Serial.print(statusNoXact);
    Serial.println('}');
  }
  delay(500);

  // Now try WITH beginTransaction to confirm it's the breaking point.
  diagDraw("5B WITH XACT", false);
  diagSerial("probe_with_xact");
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  pinMode(PIN_NRF_CSN, OUTPUT);
  digitalWrite(PIN_NRF_CSN, LOW);
  const uint8_t probeStatus = SPI.transfer(0xFF);
  digitalWrite(PIN_NRF_CSN, HIGH);
  SPI.endTransaction();
  Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\"spi_probe_status\",\"status\":"));
  Serial.print(probeStatus);
  Serial.println('}');
  delay(500);

  diagDraw("6 RF24 BEGIN", false);
  diagSerial("rf24_begin");
  diagBeginOk = radio.begin();
  diagSerial(diagBeginOk ? "rf24_begin_ok" : "rf24_begin_fail");
  diagDraw(diagBeginOk ? "7 BEGIN OK" : "7 BEGIN FAIL", false);
  if (!diagBeginOk) return;
  delay(600);
  diagChipOk = radio.isChipConnected();
  diagSerial(diagChipOk ? "chip_ok" : "chip_fail");
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setAutoAck(true);
  radio.setRetries(0, 0);
  radioStartListening();  // opens both pipes exactly like the normal build
  diagSerial("listening");
  diagDraw("8 LISTENING", false);
}

void radioDiagLoop() {
  if (diagHalted) return;
  ++diagCounter;
  if (!diagRxOn && millis() >= 10000) {
    diagRxOn = true;
    diagSerial("rx_on");
  }
  if (millis() - diagLastDrawMs >= 100) {
    diagLastDrawMs = millis();
    diagDraw(diagRxOn ? "10 RX LOOP" : "9 WAIT 10s", true);
  }
  if (millis() - diagLastStatusMs >= 1000) {
    diagLastStatusMs = millis();
    Serial.print(F("{\"source\":\"ground_station\",\"kind\":\"radio_test\",\"stage\":\"running\",\"counter\":"));
    Serial.print(diagCounter);
    Serial.print(F(",\"rx_on\":")); printBool(diagRxOn);
    Serial.print(F(",\"sniff\":")); Serial.print(sniffPackets);
    Serial.println('}');
  }
  if (diagRxOn && diagBeginOk) {
    uint8_t pipe = 0;
    while (radio.available(&pipe)) {
      if (pipe == 0) handleRcSniff();
      else handleRadioPacket();
    }
  }
}
#endif
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 1500) delay(10);

  beginOled();
#if RADIO_DIAGNOSTIC
  radioDiagSetup();
#else
  renderBootScreen();
  emitStatus(); // Publish USB/OLED health even if radio initialization fails.
#if !OLED_DIAGNOSTIC
  radioReady = nl_radio::begin(radio, RF_CHANNEL, RF24_PA_MIN);
  radioChipOk = radioReady; // nl_radio::begin() already verified a register round-trip.
  if (radioReady) radioStartListening();
  lastRadioAttemptMs = millis();
  lastOledAttemptMs = millis();
#endif
  emitStatus();
#endif
}

void loop() {
  pollSerialInput();
#if RADIO_DIAGNOSTIC
  radioDiagLoop();
  return;
#endif
#if !OLED_DIAGNOSTIC
  // Self-healing: bring the radio and OLED up if they failed at boot.
  if (!radioReady && millis() - lastRadioAttemptMs >= 2000) {
    lastRadioAttemptMs = millis();
    radioReady = nl_radio::begin(radio, RF_CHANNEL, RF24_PA_MIN);
    radioChipOk = radioReady;
    if (radioReady) radioStartListening();
  }
  if (!oledReady && millis() - lastOledAttemptMs >= 3000) {
    lastOledAttemptMs = millis();
    beginOled();
  }
  if (radioReady) {
    uint8_t pipe = 0;
    while (radio.available(&pipe)) {
      if (pipe == 0) handleRcSniff();
      else handleRadioPacket();
    }
  }
#endif
  renderOled();
  if (millis() - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = millis();
    emitStatus();
  }
}
