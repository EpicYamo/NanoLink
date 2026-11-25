#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

namespace {

constexpr uint8_t PIN_NRF_CE = PB0;
constexpr uint8_t PIN_NRF_CSN = PA4;
constexpr uint8_t RF_CHANNEL = 90;
constexpr uint32_t RF_SPI_HZ = 4000000UL;

const uint8_t GCS_ADDRESS[6] = "PC01A";
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN, RF_SPI_HZ);

struct __attribute__((packed)) TelemetryPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t flags;
  uint16_t sequence;
  int16_t accelMg[3];
  int16_t gyroCentiDps[3];
  int16_t magCounts[3];
  uint32_t pressurePa;
  int16_t temperatureCentiC;
  uint16_t rcAgeMs;
};
static_assert(sizeof(TelemetryPacket) == 32, "Telemetry packet must match aircraft firmware");

constexpr uint16_t TELEMETRY_MAGIC = 0x4E4C;

void printHeader() {
  Serial.println(F("seq,flags,accel_x_mg,accel_y_mg,accel_z_mg,gyro_x_cdeg_s,gyro_y_cdeg_s,gyro_z_cdeg_s,mag_x_counts,mag_y_counts,mag_z_counts,pressure_pa,temp_centi_c,rc_age_ms"));
}

void printPacket(const TelemetryPacket &packet) {
  Serial.print(packet.sequence);
  Serial.print(','); Serial.print(packet.flags);
  for (uint8_t i = 0; i < 3; ++i) {
    Serial.print(','); Serial.print(packet.accelMg[i]);
  }
  for (uint8_t i = 0; i < 3; ++i) {
    Serial.print(','); Serial.print(packet.gyroCentiDps[i]);
  }
  for (uint8_t i = 0; i < 3; ++i) {
    Serial.print(','); Serial.print(packet.magCounts[i]);
  }
  Serial.print(','); Serial.print(packet.pressurePa);
  Serial.print(','); Serial.print(packet.temperatureCentiC);
  Serial.print(','); Serial.println(packet.rcAgeMs);
}

} // namespace

void setup() {
  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println(F("ERROR,NRF24_INIT"));
    return;
  }

  radio.openReadingPipe(1, GCS_ADDRESS);
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setAutoAck(true);
  radio.setRetries(0, 0);
  radio.startListening();

  Serial.println(F("#NanoLink ground station; CSV follows"));
  printHeader();
}

void loop() {
  if (!radio.isChipConnected()) {
    delay(100);
    return;
  }

  while (radio.available()) {
    TelemetryPacket packet{};
    radio.read(&packet, sizeof(packet));
    if (packet.magic == TELEMETRY_MAGIC && packet.version == 1) {
      printPacket(packet);
    }
  }
}
