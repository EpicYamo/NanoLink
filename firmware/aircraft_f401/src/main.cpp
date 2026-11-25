#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <IWatchdog.h>

#ifndef ESC_MIN_US
#define ESC_MIN_US 1000
#endif
#ifndef ESC_MAX_US
#define ESC_MAX_US 2000
#endif
#ifndef OPEN_DRAIN_OUTPUTS
#define OPEN_DRAIN_OUTPUTS 1
#endif

namespace {

// Preserve the existing F401 receiver radio and PWM pin assignments.
constexpr uint8_t PIN_NRF_CE = PB0;
constexpr uint8_t PIN_NRF_CSN = PA4;
constexpr uint8_t PIN_LEFT_ELEVON = PB6;  // TIM4_CH1
constexpr uint8_t PIN_RIGHT_ELEVON = PB7; // TIM4_CH2
constexpr uint8_t PIN_ESC = PB8;          // TIM4_CH3
constexpr uint8_t PIN_LED = PC13;

constexpr uint8_t RF_CHANNEL = 90;
constexpr uint32_t RF_SPI_HZ = 4000000UL;
constexpr uint32_t RC_TIMEOUT_MS = 500;
constexpr uint32_t SENSOR_PERIOD_MS = 20;    // 50 Hz sensor sampling
constexpr uint32_t TELEMETRY_PERIOD_MS = 100; // 10 Hz radio telemetry
constexpr uint16_t SERVO_CENTER_US = 1500;
constexpr uint16_t MIX_LIMIT_US = 350; // matches the transmitter's 1150-1850 low rate

const uint8_t RC_ADDRESS[6] = "NANO1";
const uint8_t GCS_ADDRESS[6] = "PC01A";

// I2C2 avoids the existing PWM pins PB6/PB7/PB8.
// STM32duino TwoWire constructor order is SDA, SCL.
TwoWire SensorWire(PB3, PB10);
MPU9250_asukiaaa imu;
Adafruit_BMP280 barometer(&SensorWire);
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN, RF_SPI_HZ);
HardwareTimer *pwm = nullptr;

#if OPEN_DRAIN_OUTPUTS
// The servo/ESC signal lines are pulled up to the BEC's 5 V rail through
// external resistors. Hold them low before normal startup so the pull-ups do
// not present a steady high level while the ESC is booting.
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
static_assert(sizeof(RcPacket) == 12, "RC packet must match the existing Nano transmitter");

enum TelemetryFlags : uint8_t {
  FLAG_IMU_OK = 1u << 0,
  FLAG_BARO_OK = 1u << 1,
  FLAG_RC_OK = 1u << 2,
  FLAG_ARMED = 1u << 3,
};

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
static_assert(sizeof(TelemetryPacket) == 32, "Telemetry must fit one nRF24 payload");

constexpr uint16_t TELEMETRY_MAGIC = 0x4E4C; // 'NL'
constexpr uint8_t TELEMETRY_VERSION = 1;

RcPacket rc{};
TelemetryPacket telemetry{};
bool radioReady = false;
bool imuReady = false;
bool barometerReady = false;
bool haveRcPacket = false;
bool haveImuSample = false;
bool haveBaroSample = false;
uint16_t telemetrySequence = 0;
uint32_t lastRcPacketMs = 0;
uint32_t lastSensorSampleMs = 0;
uint32_t lastTelemetryMs = 0;

int16_t clampToInt16(float value) {
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return static_cast<int16_t>(value);
}

uint16_t clampPulse(int32_t value, uint16_t low, uint16_t high) {
  if (value < low) return low;
  if (value > high) return high;
  return static_cast<uint16_t>(value);
}

uint16_t clampAge(uint32_t ageMs) {
  return ageMs > 65535UL ? 65535U : static_cast<uint16_t>(ageMs);
}

void applyOutputDriveMode();

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

void applyOutputDriveMode() {
#if OPEN_DRAIN_OUTPUTS
  // TIM4 controls pulse timing; the external pull-up creates the 5 V high
  // level while the STM32 only pulls the line low or releases it.
  GPIOB->OTYPER |= (1u << 6) | (1u << 7) | (1u << 8);
#endif
}

void applyManualControl(bool linkOk) {
  if (!linkOk) {
    outputsWrite(SERVO_CENTER_US, SERVO_CENTER_US, ESC_MIN_US);
    return;
  }

  // The transmitter sends channels centered at 1500 us. The signs must be
  // verified against the actual servo/linkage directions before flight.
  const int32_t roll = static_cast<int32_t>(constrain(rc.aileron, 1000, 2000)) - SERVO_CENTER_US;
  const int32_t pitch = static_cast<int32_t>(constrain(rc.elevator, 1000, 2000)) - SERVO_CENTER_US;
  int32_t leftDelta = pitch + roll;
  int32_t rightDelta = pitch - roll;

  // Preserve the roll/pitch ratio while keeping both mixed outputs in low-rate travel.
  const int32_t largest = max(abs(leftDelta), abs(rightDelta));
  if (largest > MIX_LIMIT_US) {
    leftDelta = leftDelta * MIX_LIMIT_US / largest;
    rightDelta = rightDelta * MIX_LIMIT_US / largest;
  }

  const uint16_t left = clampPulse(SERVO_CENTER_US + leftDelta,
                                   SERVO_CENTER_US - MIX_LIMIT_US,
                                   SERVO_CENTER_US + MIX_LIMIT_US);
  const uint16_t right = clampPulse(SERVO_CENTER_US + rightDelta,
                                    SERVO_CENTER_US - MIX_LIMIT_US,
                                    SERVO_CENTER_US + MIX_LIMIT_US);

  uint16_t throttle = ESC_MIN_US;
  if (rc.armed) {
    const uint16_t input = constrain(rc.throttle, 1000, 2000);
    throttle = map(input, 1000, 2000, ESC_MIN_US, ESC_MAX_US);
  }

  outputsWrite(left, right, throttle);
}

bool sensorsBegin() {
  SensorWire.begin();
  SensorWire.setClock(100000);

  imu.setWire(&SensorWire);
  uint8_t imuId = 0;
  const bool imuIdOk = (imu.readId(&imuId) == 0) && (imuId == 0x71);
  if (imuIdOk) {
    imu.beginAccel();
    imu.beginGyro();
    imu.beginMag();
    imuReady = true;
  }

  barometerReady = barometer.begin(0x76) || barometer.begin(0x77);
  if (barometerReady) {
    barometer.setSampling(Adafruit_BMP280::MODE_NORMAL,
                          Adafruit_BMP280::SAMPLING_X2,
                          Adafruit_BMP280::SAMPLING_X16,
                          Adafruit_BMP280::FILTER_X16,
                          Adafruit_BMP280::STANDBY_MS_125);
  }

  return imuReady || barometerReady;
}

void sampleSensors() {
  const uint32_t now = millis();
  if (now - lastSensorSampleMs < SENSOR_PERIOD_MS) return;
  lastSensorSampleMs = now;

  haveImuSample = false;
  if (imuReady) {
    const bool accelOk = imu.accelUpdate() == 0;
    const bool gyroOk = imu.gyroUpdate() == 0;
    const bool magOk = imu.magUpdate() == 0;
    if (accelOk && gyroOk) {
      telemetry.accelMg[0] = clampToInt16(imu.accelX() * 1000.0f);
      telemetry.accelMg[1] = clampToInt16(imu.accelY() * 1000.0f);
      telemetry.accelMg[2] = clampToInt16(imu.accelZ() * 1000.0f);
      telemetry.gyroCentiDps[0] = clampToInt16(imu.gyroX() * 100.0f);
      telemetry.gyroCentiDps[1] = clampToInt16(imu.gyroY() * 100.0f);
      telemetry.gyroCentiDps[2] = clampToInt16(imu.gyroZ() * 100.0f);
      if (magOk) {
        telemetry.magCounts[0] = imu.magX();
        telemetry.magCounts[1] = imu.magY();
        telemetry.magCounts[2] = imu.magZ();
      }
      haveImuSample = true;
    }
  }

  haveBaroSample = false;
  if (barometerReady) {
    const float temperature = barometer.readTemperature();
    const float pressure = barometer.readPressure();
    if (!isnan(temperature) && !isnan(pressure) && pressure > 0.0f) {
      telemetry.temperatureCentiC = clampToInt16(temperature * 100.0f);
      telemetry.pressurePa = static_cast<uint32_t>(pressure);
      haveBaroSample = true;
    }
  }
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

void sendTelemetry() {
  telemetry.magic = TELEMETRY_MAGIC;
  telemetry.version = TELEMETRY_VERSION;
  telemetry.flags = 0;
  if (haveImuSample) telemetry.flags |= FLAG_IMU_OK;
  if (haveBaroSample) telemetry.flags |= FLAG_BARO_OK;
  if (millis() - lastRcPacketMs <= RC_TIMEOUT_MS) telemetry.flags |= FLAG_RC_OK;
  if (rc.armed) telemetry.flags |= FLAG_ARMED;
  telemetry.sequence = telemetrySequence++;
  telemetry.rcAgeMs = clampAge(millis() - lastRcPacketMs);

  // The ground station is a receive-only peer in this phase. Keep retries off
  // so a missing station cannot delay servicing the RC receiver.
  radio.stopListening();
  radio.openWritingPipe(GCS_ADDRESS);
  radio.setRetries(0, 0);
  radio.write(&telemetry, sizeof(telemetry));
  radioConfigureReceiver();
  lastTelemetryMs = millis();
}

void updateLed(bool linkOk) {
  digitalWrite(PIN_LED, linkOk ? LOW : ((millis() / 100) & 1) ? HIGH : LOW);
}

} // namespace

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  outputsBegin();
  applyManualControl(false);
  sensorsBegin();

  radioReady = radio.begin();
  if (radioReady) {
    radioConfigureReceiver();
  }
  lastRcPacketMs = millis();
  IWatchdog.begin(1000000);
}

void loop() {
  IWatchdog.reload();
  sampleSensors();

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
        receivedPacket = true;
      }
    }
  }

  const bool linkOk = haveRcPacket && (millis() - lastRcPacketMs <= RC_TIMEOUT_MS);
  applyManualControl(linkOk);
  updateLed(linkOk);

  // Sending immediately after an RC frame creates a predictable gap before
  // the handheld's next ~50 Hz frame. The single aircraft radio then returns
  // to the unchanged RC address.
  if (receivedPacket && (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS)) {
    sendTelemetry();
  }

  delay(1);
}
