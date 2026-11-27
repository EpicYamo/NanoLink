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
#include "../../common/radio_init.h"

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
#ifndef RADIO_DIAGNOSTIC
#define RADIO_DIAGNOSTIC 0
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
constexpr uint32_t RADIO_TELEMETRY_PERIOD_MS = 3000;  // dump every 3 s
constexpr uint32_t USB_DIAGNOSTIC_PERIOD_MS = 100;
constexpr uint16_t SERVO_CENTER_US = 1500;
constexpr uint16_t MIX_LIMIT_US = 300;   // max elevon throw (us), adjustable
constexpr uint32_t GPS_BAUD = 9600;

// --- elevon direction (flip after the bench check) ------------------------
// ROLL_SIGN/PITCH_SIGN invert each axis independently; SWAP_ROLL_PITCH swaps
// the two control axes (common <-> differential); SWAP_ELEVONS crosses the two
// servo outputs. If the surfaces move the wrong way on the bench, change these
// rather than rewiring.
constexpr int16_t ROLL_SIGN = 1;
constexpr int16_t PITCH_SIGN = 1;
constexpr bool SWAP_ROLL_PITCH = true;   // transmitter gimbal axes are crossed
constexpr bool SWAP_ELEVONS = false;

// --- stage B stabilization -------------------------------------------------
constexpr float MAX_BANK_DEG = 45.0f;   // max commanded roll angle
constexpr float MAX_PITCH_DEG = 30.0f;  // max commanded pitch angle
constexpr float ANGLE_KP = 6.0f;        // us of servo per degree of angle error
constexpr float RATE_KD = 0.25f;        // us of servo per deg/s of rotation
// Flip these if the aircraft oscillates in self-level mode: a wrong rate sign
// turns the gyro damping term into positive feedback.
constexpr float ROLL_RATE_SIGN = 1.0f;
constexpr float PITCH_RATE_SIGN = 1.0f;

const uint8_t RC_ADDRESS[6] = "NANO1";
const uint8_t GCS_ADDRESS[6] = "PC01A";
const uint8_t CMD_ADDRESS[6] = "CMND1";

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
bool imuHasMagnetometer = false;
bool barometerReady = false;
uint8_t imuAddress = 0;
uint8_t imuWhoAmI = 0;
uint8_t imuIdReadResult = 255;
uint8_t barometerAddress = 0;
uint8_t i2cAddresses[16]{};
uint8_t i2cAddressCount = 0;
bool haveRcPacket = false;
bool haveImuSample = false;
bool imuSampleFresh = false;
bool haveMagSample = false;
bool haveBaroSample = false;
float attitudeRollDeg = 0.0f;
float attitudePitchDeg = 0.0f;
bool attitudeReady = false;
uint32_t lastAttitudeMs = 0;
// Level/gyro calibration is set via the ground-station CMD_CALIBRATE_LEVEL
// command (the app's "Calibrate level/gyro" button), not automatically.
// The level reference is captured immediately; the gyro bias is averaged over
// the following ~1 s while the aircraft should be held still.
constexpr uint8_t GYRO_CALIB_SAMPLES = 50;
bool gyroCalibrating = false;
uint8_t gyroCalibCount = 0;
float gyroCalibSumRoll = 0.0f;
float gyroCalibSumPitch = 0.0f;
float gyroBiasRoll = 0.0f;
float gyroBiasPitch = 0.0f;
float rollZero = 0.0f;
float pitchZero = 0.0f;
uint16_t telemetrySequence = 0;
uint16_t telemetryTxOk = 0;
uint16_t telemetryTxFailed = 0;
uint16_t rcPacketCount = 0;
uint32_t lastRcPacketMs = 0;
uint32_t lastSensorSampleMs = 0;
uint32_t lastRadioTelemetryMs = 0;
uint32_t lastUsbDiagnosticMs = 0;
uint32_t lastRadioAttemptMs = 0;
uint32_t lastRadioHealthMs = 0;
uint32_t lastSensorRetryMs = 0;
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

void applyControl(bool linkOk) {
  if (!linkOk) {
    outputsWrite(SERVO_CENTER_US, SERVO_CENTER_US, ESC_MIN_US);
    return;
  }

  uint16_t throttle = ESC_MIN_US;
  if (rc.armed) {
    const uint16_t input = constrain(rc.throttle, 1000, 2000);
    throttle = map(input, 1000, 2000, ESC_MIN_US, ESC_MAX_US);
  }

  // The A4 knob on the transmitter sets control authority (100..1000 per-mille).
  const float rate = constrain(static_cast<int>(rc.pot2), 100, 1000) / 1000.0f;

  // Read the stick channels and apply the roll/pitch axis swap if the
  // transmitter gimbal axes are crossed.
  int32_t rollIn = constrain(rc.aileron, 1000, 2000);
  int32_t pitchIn = constrain(rc.elevator, 1000, 2000);
  if (SWAP_ROLL_PITCH) {
    const int32_t tmp = rollIn;
    rollIn = pitchIn;
    pitchIn = tmp;
  }

  int32_t rollCmd = 0;
  int32_t pitchCmd = 0;

  const bool stabilize = rc.switch2State != 0 && attitudeReady;
  if (!stabilize) {
    // Manual passthrough (dual-rate throw).
    rollCmd = static_cast<int32_t>(ROLL_SIGN * (rollIn - SERVO_CENTER_US) * rate);
    pitchCmd = static_cast<int32_t>(PITCH_SIGN * (pitchIn - SERVO_CENTER_US) * rate);
  } else {
    // Self-level: the stick commands a target angle; a P(D) controller drives
    // the elevons to hold it. The knob scales the correction authority exactly
    // like manual mode, so the pot is a single "throw/angle" limit in both.
    const float rollNorm = (rollIn - SERVO_CENTER_US) / 350.0f;
    const float pitchNorm = (pitchIn - SERVO_CENTER_US) / 350.0f;
    const float targetRoll = ROLL_SIGN * rollNorm * MAX_BANK_DEG;
    const float targetPitch = PITCH_SIGN * pitchNorm * MAX_PITCH_DEG;
    const float rollErr = targetRoll - attitudeRollDeg;
    const float pitchErr = targetPitch - attitudePitchDeg;
    const float gyroRoll = sensorData.gyroCentiDps[0] / 100.0f;   // deg/s
    const float gyroPitch = sensorData.gyroCentiDps[1] / 100.0f;  // deg/s
    const float rawRoll = ANGLE_KP * rollErr - RATE_KD * ROLL_RATE_SIGN * gyroRoll;
    const float rawPitch = ANGLE_KP * pitchErr - RATE_KD * PITCH_RATE_SIGN * gyroPitch;
    rollCmd = static_cast<int32_t>(rawRoll * rate);
    pitchCmd = static_cast<int32_t>(rawPitch * rate);
  }

  // Standard elevon mix: pitch is common mode, roll is differential.
  int32_t leftDelta = pitchCmd + rollCmd;
  int32_t rightDelta = pitchCmd - rollCmd;
  leftDelta = constrain(leftDelta, -MIX_LIMIT_US, MIX_LIMIT_US);
  rightDelta = constrain(rightDelta, -MIX_LIMIT_US, MIX_LIMIT_US);

  uint16_t left = SERVO_CENTER_US + leftDelta;
  uint16_t right = SERVO_CENTER_US + rightDelta;
  if (SWAP_ELEVONS) {
    const uint16_t tmp = left;
    left = right;
    right = tmp;
  }
  outputsWrite(left, right, throttle);
}

bool i2cDevicePresent(uint8_t address) {
  SensorWire.beginTransmission(address);
  return SensorWire.endTransmission() == 0;
}

void scanI2cBus() {
  i2cAddressCount = 0;
  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    if (i2cDevicePresent(address) && i2cAddressCount < sizeof(i2cAddresses)) {
      i2cAddresses[i2cAddressCount++] = address;
    }
  }
}

bool initImu() {
  if (imuReady) return true;

  imu = nullptr;
  if (i2cDevicePresent(0x68)) {
    imu = &imuAt68;
    imuAddress = 0x68;
  } else if (i2cDevicePresent(0x69)) {
    imu = &imuAt69;
    imuAddress = 0x69;
  }
  if (!imu) return false;

  imu->setWire(&SensorWire);
  imuIdReadResult = imu->readId(&imuWhoAmI);
  const bool mpu6500Compatible = imuWhoAmI == 0x70;
  const bool mpu9250Compatible = imuWhoAmI == 0x71 || imuWhoAmI == 0x73;
  if (imuIdReadResult != 0 || !(mpu6500Compatible || mpu9250Compatible)) {
    return false;
  }

  imu->beginAccel();
  imu->beginGyro();
  imuHasMagnetometer = mpu9250Compatible;
  if (imuHasMagnetometer) imu->beginMag();
  imuReady = true;
  return true;
}

bool initBarometer() {
  if (barometerReady) return true;

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
  return barometerReady;
}

void sensorsBegin() {
  SensorWire.begin();
  SensorWire.setClock(100000);
  scanI2cBus();
  initImu();
  initBarometer();
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
    imuSampleFresh = true;
    if (imuHasMagnetometer && imu->magUpdate() == 0) {
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

// Complementary-filter attitude estimate (roll/pitch only; yaw comes from GPS
// course later). The accelerometer is the long-term level reference and the
// gyro provides the short-term rate.
//
// NOTE: this runs once per fresh IMU sample (~20 ms), not every loop pass.
// Integrating the same stale gyro reading at the 1 kHz loop rate would corrupt
// the filter's time constant and make the attitude estimate (and therefore the
// self-level servos) jitter.
void updateAttitude() {
  if (!imuSampleFresh) return;
  imuSampleFresh = false;

  const float ax = sensorData.accelMg[0] / 1000.0f;
  const float ay = sensorData.accelMg[1] / 1000.0f;
  const float az = sensorData.accelMg[2] / 1000.0f;
  const float gyroRoll = sensorData.gyroCentiDps[0] / 100.0f;   // deg/s
  const float gyroPitch = sensorData.gyroCentiDps[1] / 100.0f;  // deg/s
  const float rollAcc = atan2f(ay, az) * 57.2957795f;
  const float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;

  // Average the gyro while the aircraft is held still during calibration.
  if (gyroCalibrating) {
    gyroCalibSumRoll += gyroRoll;
    gyroCalibSumPitch += gyroPitch;
    ++gyroCalibCount;
    if (gyroCalibCount >= GYRO_CALIB_SAMPLES) {
      gyroBiasRoll = gyroCalibSumRoll / gyroCalibCount;
      gyroBiasPitch = gyroCalibSumPitch / gyroCalibCount;
      gyroCalibrating = false;
    }
  }

  const uint32_t now = millis();

  if (!attitudeReady) {
    // First sample: seed the estimate from the accelerometer (relative to the
    // level reference, which is zero until CMD_CALIBRATE_LEVEL is received).
    attitudeRollDeg = rollAcc - rollZero;
    attitudePitchDeg = pitchAcc - pitchZero;
    attitudeReady = true;
    lastAttitudeMs = now;
    return;
  }

  float dt = (now - lastAttitudeMs) / 1000.0f;
  lastAttitudeMs = now;
  if (dt <= 0.0f || dt > 0.25f) dt = 0.02f;

  const float gr = gyroRoll - gyroBiasRoll;
  const float gp = gyroPitch - gyroBiasPitch;
  const float rollRef = rollAcc - rollZero;
  const float pitchRef = pitchAcc - pitchZero;

  attitudeRollDeg = 0.96f * (attitudeRollDeg + gr * dt) + 0.04f * rollRef;
  attitudePitchDeg = 0.96f * (attitudePitchDeg + gp * dt) + 0.04f * pitchRef;
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
  // Pipe 1 receives mission commands from the ground station.
  radio.openReadingPipe(1, CMD_ADDRESS);
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);  // ACK the RC link; telemetry itself is fire-and-forget
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
  radio.write(&frame, sizeof(frame), true);  // fire-and-forget (no auto-ACK)
  ++telemetryTxOk;
  radioConfigureReceiver();
}

// ---- Ground-station mission commands (stage A: store and acknowledge) -------
struct AircraftMission {
  int32_t homeLatE7 = 0;
  int32_t homeLonE7 = 0;
  int32_t waypointLatE7 = 0;
  int32_t waypointLonE7 = 0;
  int32_t waypointAltMm = 0;
  uint16_t loiterRadiusM = 30;
  uint8_t state = NanoLinkProtocol::MISSION_NONE;
  uint8_t lastOpcode = 0;
  uint16_t lastSequence = 0;
  bool haveHome = false;
  bool haveWaypoint = false;
};

AircraftMission mission{};

int32_t payloadInt32(const uint8_t *payload, uint8_t offset) {
  int32_t value = 0;
  memcpy(&value, payload + offset, sizeof(value));
  return value;
}

uint16_t payloadUint16(const uint8_t *payload, uint8_t offset) {
  uint16_t value = 0;
  memcpy(&value, payload + offset, sizeof(value));
  return value;
}

void sendCommandAck(uint8_t opcode, uint8_t result, uint16_t commandSequence) {
  NanoLinkProtocol::CommandAckFrame frame{};
  frame.opcode = opcode;
  frame.result = result;
  frame.missionState = mission.state;
  frame.commandSequence = commandSequence;
  transmitFrame(frame, NanoLinkProtocol::TYPE_COMMAND_ACK);
}

void handleCommandFrame() {
  NanoLinkProtocol::CommandFrame command{};
  radio.read(&command, sizeof(command));
  if (command.header.magic != NanoLinkProtocol::MAGIC ||
      command.header.version != NanoLinkProtocol::VERSION ||
      command.header.type != NanoLinkProtocol::TYPE_COMMAND) {
    return;
  }

  uint8_t result = NanoLinkProtocol::CMD_RESULT_OK;
  switch (command.opcode) {
    case NanoLinkProtocol::CMD_SET_HOME:
      mission.homeLatE7 = payloadInt32(command.payload, 0);
      mission.homeLonE7 = payloadInt32(command.payload, 4);
      mission.haveHome = true;
      if (mission.haveHome && mission.haveWaypoint) {
        mission.state = NanoLinkProtocol::MISSION_LOADED;
      }
      break;
    case NanoLinkProtocol::CMD_SET_WAYPOINT:
      mission.waypointLatE7 = payloadInt32(command.payload, 0);
      mission.waypointLonE7 = payloadInt32(command.payload, 4);
      mission.waypointAltMm = payloadInt32(command.payload, 8);
      mission.haveWaypoint = true;
      if (mission.haveHome && mission.haveWaypoint) {
        mission.state = NanoLinkProtocol::MISSION_LOADED;
      }
      break;
    case NanoLinkProtocol::CMD_SET_LOITER:
      mission.loiterRadiusM = payloadUint16(command.payload, 0);
      break;
    case NanoLinkProtocol::CMD_START_MISSION:
      if (mission.haveHome && mission.haveWaypoint) {
        // Recorded only: stabilization and navigation execution come later.
        mission.state = NanoLinkProtocol::MISSION_START_REQUESTED;
      } else {
        result = NanoLinkProtocol::CMD_RESULT_NOT_READY;
      }
      break;
    case NanoLinkProtocol::CMD_ABORT_MISSION:
      mission.state = NanoLinkProtocol::MISSION_NONE;
      break;
    case NanoLinkProtocol::CMD_CALIBRATE_LEVEL:
      // Capture the current accelerometer attitude as the level reference, then
      // average the gyro over the next ~1 s as the bias. Hold the aircraft in
      // the desired level attitude and keep it still for a second.
      {
        const float ax = sensorData.accelMg[0] / 1000.0f;
        const float ay = sensorData.accelMg[1] / 1000.0f;
        const float az = sensorData.accelMg[2] / 1000.0f;
        rollZero = atan2f(ay, az) * 57.2957795f;
        pitchZero = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
        attitudeRollDeg = 0.0f;
        attitudePitchDeg = 0.0f;
        attitudeReady = true;
        gyroCalibrating = true;
        gyroCalibCount = 0;
        gyroCalibSumRoll = 0.0f;
        gyroCalibSumPitch = 0.0f;
      }
      break;
    case NanoLinkProtocol::CMD_RTH:
      // Draft: record the request only. Navigation execution comes later.
      mission.state = NanoLinkProtocol::MISSION_RTH_REQUESTED;
      break;
    default:
      result = NanoLinkProtocol::CMD_RESULT_UNKNOWN_OP;
      break;
  }

  mission.lastOpcode = command.opcode;
  mission.lastSequence = command.header.sequence;
  sendCommandAck(command.opcode, result, command.header.sequence);
}
// -----------------------------------------------------------------------------

// Send the full telemetry snapshot (sensor + control + gps) as one burst. This
// runs every 3 s and on a transmitter trigger. Fire-and-forget: no auto-ACK, so
// the aircraft stays in receive mode ~99.9% of the time.
void sendTelemetryDump(bool linkOk) {
  {
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
  }

  {
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
  }

  {
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

// --- aircraft radio diagnostic build -----------------------------------------
// Mirrors the ground-station radio_test: staged serial markers that pinpoint
// exactly where radio init dies, reported over USB (no OLED on this board).
// Run on a known-good radio board (this aircraft flew with its module) to
// separate a firmware/stack problem from a board/wiring problem.
#if RADIO_DIAGNOSTIC
uint32_t diagCounter = 0;
uint32_t diagLastStatusMs = 0;
uint32_t diagRcCount = 0;
bool diagBeginOk = false;
bool diagRxOn = false;
bool diagHalted = false;

void diagSerial(const char *stage) {
  Serial.print(F("{\"source\":\"aircraft_usb\",\"kind\":\"radio_test\",\"stage\":\""));
  Serial.print(stage);
  Serial.println(F("\"}"));
}

void aircraftRadioDiagSetup() {
  const uint32_t usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 20000) delay(10);
  diagSerial("usb_ready");

  // GPIO self-test first — before SPI touches any pin mode.
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
      Serial.print(F("{\"source\":\"aircraft_usb\",\"kind\":\"radio_test\",\"stage\":\"gpio_pin\",\"pin\":\""));
      Serial.print(p.name);
      Serial.print(F("\",\"low_read\":")); Serial.print(readLow ? F("true") : F("false"));
      Serial.print(F(",\"high_read\":")); Serial.print(readHigh ? F("true") : F("false"));
      Serial.print(F(",\"ok\":")); Serial.print(ok ? F("true") : F("false"));
      Serial.println('}');
    }
    if (gpioFaults) {
      diagSerial("gpio_fault");
      diagHalted = true;
      return;
    }
    diagSerial("gpio_ok");
  }

  // Bit-bang probe — bypasses the SPI peripheral, cannot hang.
  diagSerial("bitbang_probe");
  {
    pinMode(PA5, OUTPUT);
    pinMode(PA6, INPUT);
    pinMode(PA7, OUTPUT);
    pinMode(PIN_NRF_CSN, OUTPUT);
    digitalWrite(PA5, LOW);
    digitalWrite(PIN_NRF_CSN, LOW);
    delayMicroseconds(5);
    uint8_t status = 0;
    for (uint8_t i = 0; i < 8; ++i) {
      digitalWrite(PA7, HIGH);
      delayMicroseconds(5);
      digitalWrite(PA5, HIGH);
      delayMicroseconds(5);
      status = (status << 1) | (digitalRead(PA6) ? 1 : 0);
      digitalWrite(PA5, LOW);
      delayMicroseconds(5);
    }
    digitalWrite(PIN_NRF_CSN, HIGH);
    Serial.print(F("{\"source\":\"aircraft_usb\",\"kind\":\"radio_test\",\"stage\":\"bitbang_status\",\"status\":"));
    Serial.print(status);
    Serial.println('}');
  }
  delay(500);

  diagSerial("spi_begin");
  SPI.begin();
  diagSerial("spi_begin_done");

  diagSerial("spi_probe");
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  pinMode(PIN_NRF_CSN, OUTPUT);
  digitalWrite(PIN_NRF_CSN, LOW);
  const uint8_t probeStatus = SPI.transfer(0xFF);
  digitalWrite(PIN_NRF_CSN, HIGH);
  SPI.endTransaction();
  Serial.print(F("{\"source\":\"aircraft_usb\",\"kind\":\"radio_test\",\"stage\":\"spi_probe_status\",\"status\":"));
  Serial.print(probeStatus);
  Serial.println('}');
  delay(500);

  diagSerial("rf24_begin");
  diagBeginOk = radio.begin();
  diagSerial(diagBeginOk ? "rf24_begin_ok" : "rf24_begin_fail");
  if (diagBeginOk) {
    radioConfigureReceiver();  // opens the RC and command pipes
    diagSerial("listening");
  }
}

void aircraftRadioDiagLoop() {
  if (diagHalted) return;
  ++diagCounter;
  if (!diagRxOn && millis() >= 10000) {
    diagRxOn = true;
    diagSerial("rx_on");
  }
  if (millis() - diagLastStatusMs >= 1000) {
    diagLastStatusMs = millis();
    Serial.print(F("{\"source\":\"aircraft_usb\",\"kind\":\"radio_test\",\"stage\":\"running\",\"counter\":"));
    Serial.print(diagCounter);
    Serial.print(F(",\"rx_on\":")); Serial.print(diagRxOn ? F("true") : F("false"));
    Serial.print(F(",\"rc\":")); Serial.print(diagRcCount);
    Serial.println('}');
  }
  if (diagRxOn && diagBeginOk) {
    uint8_t pipe = 0;
    while (radio.available(&pipe)) {
      if (pipe == 1) {
        NanoLinkProtocol::CommandFrame command{};
        radio.read(&command, sizeof(command));
      } else {
        RcPacket incoming{};
        radio.read(&incoming, sizeof(incoming));
        ++diagRcCount;
      }
    }
  }
}
#endif
// -----------------------------------------------------------------------------

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

#if RADIO_DIAGNOSTIC
  Serial.begin(115200);
  aircraftRadioDiagSetup();
#else
#if USB_DIAGNOSTIC
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 1500) delay(10);
#else
  outputsBegin();
  applyControl(false);
#endif

  // Arm the watchdog before peripheral init. If a driver ever wedges again the
  // board reboots and retries instead of sitting silently on the bench.
  IWatchdog.begin(2000000);
  IWatchdog.reload();

  initializeGps();
  IWatchdog.reload();
  sensorsBegin();
  IWatchdog.reload();

#if !USB_DIAGNOSTIC
  radioReady = nl_radio::begin(radio, RF_CHANNEL, RF24_PA_LOW);
  if (radioReady) radioConfigureReceiver();
#else
  radioReady = false;
#endif

  lastRcPacketMs = millis();
  lastRadioAttemptMs = millis();
  lastRadioHealthMs = millis();
  lastSensorRetryMs = millis();
#endif
}

void loop() {
#if RADIO_DIAGNOSTIC
  aircraftRadioDiagLoop();
  return;
#endif
  IWatchdog.reload();
  sampleSensors();
  updateAttitude();
  updateGps();

  // Self-healing peripherals: retry a failed radio, re-arm a wedged radio, and
  // re-probe sensors that were missing at boot.
#if !USB_DIAGNOSTIC
  if (!radioReady && millis() - lastRadioAttemptMs >= 2000) {
    lastRadioAttemptMs = millis();
    radioReady = nl_radio::begin(radio, RF_CHANNEL, RF24_PA_LOW);
    if (radioReady) radioConfigureReceiver();
  }
  if (radioReady && millis() - lastRadioHealthMs >= 10000) {
    lastRadioHealthMs = millis();
    if (!nl_radio::responds(radio, RF_CHANNEL)) {
      radioReady = false;
      lastRadioAttemptMs = millis();
    }
  }
#endif
  if ((!imuReady || !barometerReady) && millis() - lastSensorRetryMs >= 5000) {
    lastSensorRetryMs = millis();
    if (!imuReady) initImu();
    if (!barometerReady) initBarometer();
  }

  if (radioReady) {
    uint8_t pipe = 0;
    while (radio.available(&pipe)) {
      if (pipe == 1) {
        // Mission commands arrive on the ground-station pipe.
        handleCommandFrame();
        continue;
      }
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
  applyControl(linkOk);
  updateLed(linkOk);

  // Transmitter trigger buttons (throttle-stick trims, pot1 bits 10/11).
  // Edge-detect the dump trigger and record the RTH request (draft).
  static bool dumpTriggerPrev = false;
  const bool dumpTrigger = (rc.pot1 >> NanoLinkProtocol::POT1_DUMP_TRIGGER_SHIFT) & 1u;
  const bool rthTrigger = (rc.pot1 >> NanoLinkProtocol::POT1_RTH_TRIGGER_SHIFT) & 1u;
  if (dumpTrigger && !dumpTriggerPrev) {
    lastRadioTelemetryMs = 0;  // force an immediate dump below
  }
  dumpTriggerPrev = dumpTrigger;
  if (rthTrigger && mission.state != NanoLinkProtocol::MISSION_RTH_REQUESTED) {
    mission.state = NanoLinkProtocol::MISSION_RTH_REQUESTED;  // draft: recorded only
  }

  if (radioReady && millis() - lastRadioTelemetryMs >= RADIO_TELEMETRY_PERIOD_MS) {
    sendTelemetryDump(linkOk);
    lastRadioTelemetryMs = millis();
  }
#endif

  delay(1);
}
