#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <IWatchdog.h>
#include <math.h>
#include <string.h>

#include "../../common/telemetry_protocol.h"

#ifndef ESC_MIN_US
#define ESC_MIN_US 1000
#endif
#ifndef ESC_MAX_US
#define ESC_MAX_US 2000
#endif
#ifndef OPEN_DRAIN_OUTPUTS
#define OPEN_DRAIN_OUTPUTS 1
#endif
#ifndef USB_DIAGNOSTIC
#define USB_DIAGNOSTIC 0
#endif
#ifndef ENABLE_GPS
#define ENABLE_GPS 1
#endif

namespace {

constexpr uint8_t PIN_NRF_CE = PB0;
constexpr uint8_t PIN_NRF_CSN = PA4;
constexpr uint8_t PIN_LEFT_ELEVON = PB6;
constexpr uint8_t PIN_RIGHT_ELEVON = PB7;
constexpr uint8_t PIN_ESC = PB8;
constexpr uint8_t PIN_LED = PC13;

constexpr uint8_t RF_CHANNEL = 90;
constexpr uint32_t RF_SPI_HZ = 4000000UL;
constexpr uint32_t RC_TIMEOUT_MS = 500;
constexpr uint32_t SENSOR_PERIOD_MS = 20;
constexpr uint32_t RADIO_TELEMETRY_PERIOD_MS = 30;
constexpr uint32_t USB_DIAGNOSTIC_PERIOD_MS = 100;
constexpr uint16_t SERVO_CENTER_US = 1500;
constexpr uint16_t MIX_LIMIT_US = 350;
constexpr uint32_t GPS_BAUD = 9600;

const uint8_t RC_ADDRESS[6] = "NANO1";
const uint8_t GCS_ADDRESS[6] = "PC01A";

// I2C2: SDA PB3, SCL PB10. GPS USART1: RX PA10, TX PA9.
TwoWire SensorWire(PB3, PB10);
HardwareSerial &gpsSerial = Serial1; // USART1: RX PA10, TX PA9
MPU9250_asukiaaa imuAt68(0x68);
MPU9250_asukiaaa imuAt69(0x69);
MPU9250_asukiaaa *imu = nullptr;
Adafruit_BMP280 barometer(&SensorWire);
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN, RF_SPI_HZ);
HardwareTimer *pwm = nullptr;

#if OPEN_DRAIN_OUTPUTS
// Hold the 5 V pull-up PWM lines low during MCU startup.
struct EarlyOutputsLow {
  EarlyOutputsLow() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->OTYPER |= (1u << 6) | (1u << 7) | (1u << 8);
    GPIOB->BSRR = (1u << (6 + 16)) | (1u << (7 + 16)) | (1u << (8 + 16));
    GPIOB->MODER &= ~((3u << 12) | (3u << 14) | (3u << 16));
    GPIOB->MODER |=  ((1u << 12) | (1u << 14) | (1u << 16));
  }
};
__attribute__((used)) EarlyOutputsLow earlyOutputsLow;
#endif

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

struct GpsState {
  bool streamSeen = false;
  bool fix = false;
  uint8_t fixType = 0;
  uint8_t satellites = 0;
  uint16_t hdopCenti = 0;
  int32_t latitudeE7 = 0;
  int32_t longitudeE7 = 0;
  int32_t altitudeMm = 0;
  uint16_t speedCms = 0;
  uint16_t courseCentiDeg = 0;
  uint32_t lastSentenceMs = 0;
};

RcPacket rc{};
GpsState gps{};
NanoLinkProtocol::SensorFrame sensorData{};
bool radioReady = false;
bool imuReady = false;
bool barometerReady = false;
uint8_t imuAddress = 0;
uint8_t imuWhoAmI = 0;
uint8_t imuIdReadResult = 255;
uint8_t barometerAddress = 0;
uint8_t i2cAddresses[16]{};
uint8_t i2cAddressCount = 0;
bool haveRcPacket = false;
bool haveImuSample = false;
bool haveMagSample = false;
bool haveBaroSample = false;
uint16_t telemetrySequence = 0;
uint16_t telemetryTxOk = 0;
uint16_t telemetryTxFailed = 0;
uint16_t rcPacketCount = 0;
uint32_t lastRcPacketMs = 0;
uint32_t lastSensorSampleMs = 0;
uint32_t lastRadioTelemetryMs = 0;
uint32_t lastUsbDiagnosticMs = 0;
uint8_t telemetryTypeIndex = 0;
char nmeaLine[120]{};
uint8_t nmeaLength = 0;

int16_t clampToInt16(float value) {
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return static_cast<int16_t>(value);
}

uint16_t clampAge(uint32_t ageMs) {
  return ageMs > 65535UL ? 65535U : static_cast<uint16_t>(ageMs);
}

uint16_t clampUnsigned(float value) {
  if (value <= 0.0f) return 0;
  if (value >= 65535.0f) return 65535;
  return static_cast<uint16_t>(value);
}

void applyOutputDriveMode() {
#if OPEN_DRAIN_OUTPUTS
  GPIOB->OTYPER |= (1u << 6) | (1u << 7) | (1u << 8);
#endif
}

void outputsBegin() {
  pwm = new HardwareTimer(TIM4);
  pwm->setOverflow(20000, MICROSEC_FORMAT);
  pwm->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PIN_LEFT_ELEVON);
  pwm->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PIN_RIGHT_ELEVON);
  pwm->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PIN_ESC);
  applyOutputDriveMode();
  pwm->setCaptureCompare(1, SERVO_CENTER_US, MICROSEC_COMPARE_FORMAT);
  pwm->setCaptureCompare(2, SERVO_CENTER_US, MICROSEC_COMPARE_FORMAT);
  pwm->setCaptureCompare(3, ESC_MIN_US, MICROSEC_COMPARE_FORMAT);
  pwm->resume();
}

void outputsWrite(uint16_t left, uint16_t right, uint16_t throttle) {
  pwm->setCaptureCompare(1, left, MICROSEC_COMPARE_FORMAT);
  pwm->setCaptureCompare(2, right, MICROSEC_COMPARE_FORMAT);
  pwm->setCaptureCompare(3, throttle, MICROSEC_COMPARE_FORMAT);
}

void applyManualControl(bool linkOk) {
  if (!linkOk) {
    outputsWrite(SERVO_CENTER_US, SERVO_CENTER_US, ESC_MIN_US);
    return;
  }

  const int32_t roll = static_cast<int32_t>(constrain(rc.aileron, 1000, 2000)) - SERVO_CENTER_US;
  const int32_t pitch = static_cast<int32_t>(constrain(rc.elevator, 1000, 2000)) - SERVO_CENTER_US;
  int32_t leftDelta = pitch + roll;
  int32_t rightDelta = pitch - roll;
  const int32_t largest = max(abs(leftDelta), abs(rightDelta));
  if (largest > MIX_LIMIT_US) {
    leftDelta = leftDelta * MIX_LIMIT_US / largest;
    rightDelta = rightDelta * MIX_LIMIT_US / largest;
  }

  const uint16_t left = static_cast<uint16_t>(SERVO_CENTER_US + leftDelta);
  const uint16_t right = static_cast<uint16_t>(SERVO_CENTER_US + rightDelta);
  uint16_t throttle = ESC_MIN_US;
  if (rc.armed) {
    const uint16_t input = constrain(rc.throttle, 1000, 2000);
    throttle = map(input, 1000, 2000, ESC_MIN_US, ESC_MAX_US);
  }
  outputsWrite(left, right, throttle);
}

bool i2cDevicePresent(uint8_t address) {
  SensorWire.beginTransmission(address);
  return SensorWire.endTransmission() == 0;
}

bool sensorsBegin() {
  SensorWire.begin();
  SensorWire.setClock(100000);

  i2cAddressCount = 0;
  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    if (i2cDevicePresent(address) && i2cAddressCount < sizeof(i2cAddresses)) {
      i2cAddresses[i2cAddressCount++] = address;
    }
  }

  if (i2cDevicePresent(0x68)) {
    imu = &imuAt68;
    imuAddress = 0x68;
  } else if (i2cDevicePresent(0x69)) {
    imu = &imuAt69;
    imuAddress = 0x69;
  }
  if (imu) {
    imu->setWire(&SensorWire);
    imuIdReadResult = imu->readId(&imuWhoAmI);
    if (imuIdReadResult == 0 && imuWhoAmI == 0x71) {
      imu->beginAccel();
      imu->beginGyro();
      imu->beginMag();
      imuReady = true;
    }
  }

  if (barometer.begin(0x76)) {
    barometerAddress = 0x76;
    barometerReady = true;
  } else if (barometer.begin(0x77)) {
    barometerAddress = 0x77;
    barometerReady = true;
  }
  if (barometerReady) {
    barometer.setSampling(Adafruit_BMP280::MODE_NORMAL,
                          Adafruit_BMP280::SAMPLING_X2,
                          Adafruit_BMP280::SAMPLING_X16,
                          Adafruit_BMP280::FILTER_X16,
                          Adafruit_BMP280::STANDBY_MS_125);
  }
  return imuReady || barometerReady;
}

void initializeGps() {
#if ENABLE_GPS
  gpsSerial.begin(GPS_BAUD);
#endif
}

void sampleSensors() {
  if (millis() - lastSensorSampleMs < SENSOR_PERIOD_MS) return;
  lastSensorSampleMs = millis();

  haveImuSample = false;
  haveMagSample = false;
  if (imuReady && imu->accelUpdate() == 0 && imu->gyroUpdate() == 0) {
    sensorData.accelMg[0] = clampToInt16(imu->accelX() * 1000.0f);
    sensorData.accelMg[1] = clampToInt16(imu->accelY() * 1000.0f);
    sensorData.accelMg[2] = clampToInt16(imu->accelZ() * 1000.0f);
    sensorData.gyroCentiDps[0] = clampToInt16(imu->gyroX() * 100.0f);
    sensorData.gyroCentiDps[1] = clampToInt16(imu->gyroY() * 100.0f);
    sensorData.gyroCentiDps[2] = clampToInt16(imu->gyroZ() * 100.0f);
    haveImuSample = true;
    if (imu->magUpdate() == 0) {
      sensorData.magCounts[0] = imu->magX();
      sensorData.magCounts[1] = imu->magY();
      sensorData.magCounts[2] = imu->magZ();
      haveMagSample = true;
    }
  }

  haveBaroSample = false;
  if (barometerReady) {
    const float temperature = barometer.readTemperature();
    const float pressure = barometer.readPressure();
    if (!isnan(temperature) && !isnan(pressure) && pressure > 0.0f) {
      sensorData.temperatureCentiC = clampToInt16(temperature * 100.0f);
      sensorData.pressurePa = static_cast<uint32_t>(pressure);
      haveBaroSample = true;
    }
  }
}

float parseNmeaFloat(const char *value) {
  if (!value) return 0.0f;
  bool negative = false;
  if (*value == '-') {
    negative = true;
    ++value;
  } else if (*value == '+') {
    ++value;
  }
  float result = 0.0f;
  float place = 0.1f;
  bool fractional = false;
  for (; *value; ++value) {
    if (*value == '.') {
      fractional = true;
      continue;
    }
    if (*value < '0' || *value > '9') break;
    const uint8_t digit = static_cast<uint8_t>(*value - '0');
    if (fractional) {
      result += digit * place;
      place *= 0.1f;
    } else {
      result = result * 10.0f + digit;
    }
  }
  return negative ? -result : result;
}

int parseNmeaInt(const char *value) {
  if (!value) return 0;
  bool negative = false;
  if (*value == '-') {
    negative = true;
    ++value;
  }
  int result = 0;
  while (*value >= '0' && *value <= '9') {
    result = result * 10 + (*value++ - '0');
  }
  return negative ? -result : result;
}

uint8_t hexNibble(char c) {
  if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
  if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(c - 'a' + 10);
  return 0xFF;
}

float nmeaCoordinate(const char *value, const char *hemisphere) {
  if (!value || !value[0]) return 0.0f;
  const float raw = parseNmeaFloat(value);
  const int degrees = static_cast<int>(raw / 100.0f);
  float decimal = degrees + (raw - degrees * 100.0f) / 60.0f;
  if (hemisphere && (hemisphere[0] == 'S' || hemisphere[0] == 'W')) decimal = -decimal;
  return decimal;
}

void parseNmeaLine(char *line) {
  if (!line || line[0] != '$') return;
  char *star = strchr(line, '*');
  if (!star || !star[1] || !star[2]) return;

  uint8_t checksum = 0;
  for (char *p = line + 1; p < star; ++p) checksum ^= static_cast<uint8_t>(*p);
  char expectedText[3] = {star[1], star[2], '\0'};
  const uint8_t highNibble = hexNibble(expectedText[0]);
  const uint8_t lowNibble = hexNibble(expectedText[1]);
  if (highNibble > 0x0F || lowNibble > 0x0F ||
      checksum != static_cast<uint8_t>((highNibble << 4) | lowNibble)) return;
  *star = '\0';

  char *fields[20]{};
  uint8_t count = 0;
  fields[count++] = line;
  for (char *p = line; *p && count < 20; ++p) {
    if (*p == ',') {
      *p = '\0';
      fields[count++] = p + 1;
    }
  }

  if (strstr(fields[0], "GGA")) {
    if (count < 10) return;
    gps.streamSeen = true;
    gps.lastSentenceMs = millis();
    gps.fixType = static_cast<uint8_t>(constrain(parseNmeaInt(fields[6]), 0, 9));
    gps.fix = gps.fixType > 0;
    gps.satellites = static_cast<uint8_t>(constrain(parseNmeaInt(fields[7]), 0, 99));
    gps.hdopCenti = clampUnsigned(parseNmeaFloat(fields[8]) * 100.0f);
    gps.altitudeMm = static_cast<int32_t>(parseNmeaFloat(fields[9]) * 1000.0f);
    if (fields[2][0] && fields[4][0]) {
      gps.latitudeE7 = static_cast<int32_t>(nmeaCoordinate(fields[2], fields[3]) * 10000000.0f);
      gps.longitudeE7 = static_cast<int32_t>(nmeaCoordinate(fields[4], fields[5]) * 10000000.0f);
    }
  } else if (strstr(fields[0], "RMC")) {
    if (count < 9) return;
    gps.streamSeen = true;
    gps.lastSentenceMs = millis();
    gps.fix = fields[2][0] == 'A';
    if (gps.fix && fields[3][0] && fields[5][0]) {
      gps.latitudeE7 = static_cast<int32_t>(nmeaCoordinate(fields[3], fields[4]) * 10000000.0f);
      gps.longitudeE7 = static_cast<int32_t>(nmeaCoordinate(fields[5], fields[6]) * 10000000.0f);
    }
    gps.speedCms = clampUnsigned(parseNmeaFloat(fields[7]) * 51.4444f);
    gps.courseCentiDeg = clampUnsigned(parseNmeaFloat(fields[8]) * 100.0f);
  }
}

void updateGps() {
#if ENABLE_GPS
  while (gpsSerial.available()) {
    const char c = static_cast<char>(gpsSerial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      nmeaLine[nmeaLength] = '\0';
      parseNmeaLine(nmeaLine);
      nmeaLength = 0;
    } else if (nmeaLength < sizeof(nmeaLine) - 1) {
      nmeaLine[nmeaLength++] = c;
    } else {
      nmeaLength = 0;
    }
  }
  if (millis() - gps.lastSentenceMs > 3000) {
    gps.streamSeen = false;
    gps.fix = false;
  }
#endif
}

void radioConfigureReceiver() {
  radio.openReadingPipe(0, RC_ADDRESS);
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.setRetries(0, 0);
  radio.startListening();
}

template <typename Frame>
void fillHeader(Frame &frame, uint8_t type) {
  frame.header.magic = NanoLinkProtocol::MAGIC;
  frame.header.version = NanoLinkProtocol::VERSION;
  frame.header.type = type;
  frame.header.sequence = telemetrySequence++;
}

template <typename Frame>
void transmitFrame(Frame &frame, uint8_t type) {
  fillHeader(frame, type);
  radio.stopListening();
  radio.openWritingPipe(GCS_ADDRESS);
  radio.setRetries(0, 0);
  if (radio.write(&frame, sizeof(frame))) ++telemetryTxOk;
  else ++telemetryTxFailed;
  radioConfigureReceiver();
}

void sendRadioTelemetry(bool linkOk) {
  const uint8_t type = telemetryTypeIndex++ % 3;
  if (type == 0) {
    NanoLinkProtocol::SensorFrame frame{};
    frame.flags = (haveImuSample ? NanoLinkProtocol::SENSOR_IMU_OK : 0) |
                  (haveMagSample ? NanoLinkProtocol::SENSOR_MAG_OK : 0) |
                  (haveBaroSample ? NanoLinkProtocol::SENSOR_BARO_OK : 0);
    memcpy(frame.accelMg, sensorData.accelMg, sizeof(frame.accelMg));
    memcpy(frame.gyroCentiDps, sensorData.gyroCentiDps, sizeof(frame.gyroCentiDps));
    memcpy(frame.magCounts, sensorData.magCounts, sizeof(frame.magCounts));
    frame.pressurePa = sensorData.pressurePa;
    frame.temperatureCentiC = sensorData.temperatureCentiC;
    transmitFrame(frame, NanoLinkProtocol::TYPE_SENSORS);
  } else if (type == 1) {
    NanoLinkProtocol::ControlFrame frame{};
    frame.flags = (linkOk ? NanoLinkProtocol::CONTROL_RC_VALID : 0) |
                  (rc.armed ? NanoLinkProtocol::CONTROL_ARMED : 0) |
                  (gps.streamSeen ? NanoLinkProtocol::CONTROL_GPS_STREAM : 0) |
                  (gps.fix ? NanoLinkProtocol::CONTROL_GPS_FIX : 0);
    frame.throttleUs = rc.throttle;
    frame.aileronUs = rc.aileron;
    frame.elevatorUs = rc.elevator;
    frame.pot1 = rc.pot1;
    frame.pot2 = rc.pot2;
    frame.throttleMode = rc.switch2State;
    frame.rcAgeMs = haveRcPacket ? clampAge(millis() - lastRcPacketMs) : 65535;
    frame.rcPackets = rcPacketCount;
    frame.telemetryTxOk = telemetryTxOk;
    frame.telemetryTxFailed = telemetryTxFailed;
    transmitFrame(frame, NanoLinkProtocol::TYPE_CONTROL);
  } else {
    NanoLinkProtocol::GpsFrame frame{};
    frame.flags = (gps.streamSeen ? NanoLinkProtocol::GPS_STREAM_OK : 0) |
                  (gps.fix ? NanoLinkProtocol::GPS_FIX_OK : 0);
    frame.fixType = gps.fixType;
    frame.satellites = gps.satellites;
    frame.hdopCenti = gps.hdopCenti;
    frame.latitudeE7 = gps.latitudeE7;
    frame.longitudeE7 = gps.longitudeE7;
    frame.altitudeMm = gps.altitudeMm;
    frame.speedCms = gps.speedCms;
    frame.courseCentiDeg = gps.courseCentiDeg;
    transmitFrame(frame, NanoLinkProtocol::TYPE_GPS);
  }
}

#if USB_DIAGNOSTIC
void printJsonBool(bool value) { Serial.print(value ? F("true") : F("false")); }

void printUsbDiagnostic(bool nrfChipOk) {
  Serial.print(F("{\"source\":\"aircraft_usb\",\"kind\":\"sample\",\"uptime_ms\":"));
  Serial.print(millis());
  Serial.print(F(",\"nrf\":{\"enabled\":false,\"chip_ok\":"));
  printJsonBool(nrfChipOk);
  Serial.print(F("},\"rc\":{\"valid\":false,\"age_ms\":65535,\"throttle_us\":0,\"aileron_us\":0,\"elevator_us\":0,\"armed\":false,\"throttle_mode\":0},\"i2c_addresses\":["));
  for (uint8_t i = 0; i < i2cAddressCount; ++i) {
    if (i) Serial.print(',');
    Serial.print(i2cAddresses[i]);
  }
  Serial.print(F("],\"imu\":{\"i2c_addr\":")); Serial.print(imuAddress);
  Serial.print(F(",\"who_am_i\":")); Serial.print(imuWhoAmI);
  Serial.print(F(",\"id_read_rc\":")); Serial.print(imuIdReadResult);
  Serial.print(F(",\"ok\":"));
  printJsonBool(haveImuSample);
  Serial.print(F(",\"mag_ok\":"));
  printJsonBool(haveMagSample);
  Serial.print(F(",\"accel_mg\":["));
  Serial.print(sensorData.accelMg[0]); Serial.print(',');
  Serial.print(sensorData.accelMg[1]); Serial.print(',');
  Serial.print(sensorData.accelMg[2]);
  Serial.print(F("],\"gyro_cdeg_s\":["));
  Serial.print(sensorData.gyroCentiDps[0]); Serial.print(',');
  Serial.print(sensorData.gyroCentiDps[1]); Serial.print(',');
  Serial.print(sensorData.gyroCentiDps[2]);
  Serial.print(F("],\"mag_counts\":["));
  Serial.print(sensorData.magCounts[0]); Serial.print(',');
  Serial.print(sensorData.magCounts[1]); Serial.print(',');
  Serial.print(sensorData.magCounts[2]);
  Serial.print(F("]},\"baro\":{\"i2c_addr\":")); Serial.print(barometerAddress);
  Serial.print(F(",\"ok\":"));
  printJsonBool(haveBaroSample);
  Serial.print(F(",\"pressure_pa\":")); Serial.print(sensorData.pressurePa);
  Serial.print(F(",\"temperature_centi_c\":")); Serial.print(sensorData.temperatureCentiC);
  Serial.print(F("},\"gps\":{\"uart_enabled\":"));
  printJsonBool(ENABLE_GPS);
  Serial.print(F(",\"stream\":")); printJsonBool(gps.streamSeen);
  Serial.print(F(",\"fix\":")); printJsonBool(gps.fix);
  Serial.print(F(",\"fix_type\":")); Serial.print(gps.fixType);
  Serial.print(F(",\"satellites\":")); Serial.print(gps.satellites);
  Serial.print(F(",\"hdop_centi\":")); Serial.print(gps.hdopCenti);
  Serial.print(F(",\"lat_e7\":")); Serial.print(gps.latitudeE7);
  Serial.print(F(",\"lon_e7\":")); Serial.print(gps.longitudeE7);
  Serial.print(F(",\"alt_mm\":")); Serial.print(gps.altitudeMm);
  Serial.print(F(",\"speed_cms\":")); Serial.print(gps.speedCms);
  Serial.print(F(",\"course_centi_deg\":")); Serial.print(gps.courseCentiDeg);
  Serial.println(F("}}"));
}
#endif

void updateLed(bool linkOk) {
  digitalWrite(PIN_LED, linkOk ? LOW : ((millis() / 100) & 1) ? HIGH : LOW);
}

} // namespace

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

#if USB_DIAGNOSTIC
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 1500) delay(10);
#else
  outputsBegin();
  applyManualControl(false);
#endif

  initializeGps();
  sensorsBegin();

#if !USB_DIAGNOSTIC
  radioReady = radio.begin();
  if (radioReady) radioConfigureReceiver();
#else
  radioReady = false;
#endif

  lastRcPacketMs = millis();
  IWatchdog.begin(1000000);
}

void loop() {
  IWatchdog.reload();
  sampleSensors();
  updateGps();

  bool receivedPacket = false;
  if (radioReady) {
    while (radio.available()) {
      RcPacket incoming{};
      radio.read(&incoming, sizeof(incoming));
      if (incoming.throttle >= 900 && incoming.throttle <= 2100 &&
          incoming.aileron >= 900 && incoming.aileron <= 2100 &&
          incoming.elevator >= 900 && incoming.elevator <= 2100 &&
          incoming.armed <= 1 && incoming.switch2State <= 2) {
        rc = incoming;
        lastRcPacketMs = millis();
        haveRcPacket = true;
        ++rcPacketCount;
        receivedPacket = true;
      }
    }
  }

  const bool linkOk = haveRcPacket && (millis() - lastRcPacketMs <= RC_TIMEOUT_MS);

#if USB_DIAGNOSTIC
  if (millis() - lastUsbDiagnosticMs >= USB_DIAGNOSTIC_PERIOD_MS) {
    lastUsbDiagnosticMs = millis();
    printUsbDiagnostic(false);
  }
#else
  applyManualControl(linkOk);
  updateLed(linkOk);
  if (radioReady && receivedPacket && millis() - lastRadioTelemetryMs >= RADIO_TELEMETRY_PERIOD_MS) {
    sendRadioTelemetry(linkOk);
    lastRadioTelemetryMs = millis();
  }
#endif

  delay(1);
}
