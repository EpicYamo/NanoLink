#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <string.h>

#include "../../common/telemetry_protocol.h"

namespace {

constexpr uint8_t PIN_NRF_CE = PB0;
constexpr uint8_t PIN_NRF_CSN = PA4;
constexpr uint8_t RF_CHANNEL = 90;
constexpr uint32_t RF_SPI_HZ = 4000000UL;
constexpr uint32_t STATUS_PERIOD_MS = 1000;

const uint8_t GCS_ADDRESS[6] = "PC01A";
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN, RF_SPI_HZ);

bool radioReady = false;
bool radioChipOk = false;
bool haveSequence = false;
uint16_t lastSequence = 0;
uint32_t receivedPackets = 0;
uint32_t lostPackets = 0;
uint32_t autoAckCount = 0;
uint32_t lastPacketMs = 0;
uint32_t lastStatusMs = 0;

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
  printBool(radioReady);
  Serial.print(F(",\"radio_chip_ok\":")); printBool(radioChipOk);
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
  emitStatus();
}

void loop() {
  if (radioReady) {
    while (radio.available()) handleRadioPacket();
  }
  if (millis() - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = millis();
    emitStatus();
  }
}
