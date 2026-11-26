#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <U8g2lib.h>
#include <string.h>

#include "../../common/telemetry_protocol.h"

#ifndef OLED_DIAGNOSTIC
#define OLED_DIAGNOSTIC 0
#endif

namespace {

constexpr uint8_t PIN_NRF_CE = PB0;
constexpr uint8_t PIN_NRF_CSN = PA4;
constexpr uint8_t RF_CHANNEL = 90;
constexpr uint32_t RF_SPI_HZ = 4000000UL;
constexpr uint32_t STATUS_PERIOD_MS = 1000;

const uint8_t GCS_ADDRESS[6] = "PC01A";
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN, RF_SPI_HZ);
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

void beginOled() {
  Wire.begin(); // F401 I2C1 default: SDA PB7, SCL PB6.
  Wire.setClock(100000);

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

void renderBootScreen() {
  if (!oledReady) return;
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(22, 30, "NanoLink GCS");
  oled.drawStr(10, 52, "Waiting for aircraft");
  oled.sendBuffer();
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
  oled.sendBuffer();
  return;
#endif

  const uint8_t page = (millis() / 2500) % 4;

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
      const uint8_t trims = static_cast<uint8_t>((latestControl.pot1 >> NanoLinkProtocol::POT1_TRIM_SHIFT) & 0x0F);
      oled.setCursor(0, 63); oled.print(F("MODE "));
      oled.print(mode == 2 ? F("REFLEX") : F("CRUISE"));
      oled.print(F(" TRIM ")); oled.print(trims, HEX);
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
  } else {
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
  }
  oled.sendBuffer();
}

void printBool(bool value) { Serial.print(value ? F("true") : F("false")); }

void printCommon(uint8_t kind, uint16_t sequence) {
  Serial.print(F("{\"source\":\"ground_station\",\"kind\":"));
  if (kind == NanoLinkProtocol::TYPE_SENSORS) Serial.print(F("\"sensor\""));
  else if (kind == NanoLinkProtocol::TYPE_CONTROL) Serial.print(F("\"control\""));
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
    default:
      break;
  }
}

} // namespace

void setup() {
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 1500) delay(10);

  beginOled();
  renderBootScreen();
  emitStatus(); // Publish USB/OLED health even if radio initialization fails.
#if !OLED_DIAGNOSTIC
  radioReady = radio.begin();
  if (radioReady) {
    radioChipOk = radio.isChipConnected();
    radio.openReadingPipe(1, GCS_ADDRESS);
    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MIN);
    radio.setAutoAck(true);
    radio.setRetries(0, 0);
    radio.startListening();
  }
#endif
  emitStatus();
}

void loop() {
#if !OLED_DIAGNOSTIC
  if (radioReady) {
    while (radio.available()) handleRadioPacket();
  }
#endif
  renderOled();
  if (millis() - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = millis();
    emitStatus();
  }
}
