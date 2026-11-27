#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <RF24.h>

#include "../../common/telemetry_protocol.h"

#ifndef ENABLE_AUX_POT1
#define ENABLE_AUX_POT1 0
#endif
#ifndef BENCH_RF
#define BENCH_RF 0
#endif

namespace {

constexpr uint8_t PIN_RADIO_CE = 9;
constexpr uint8_t PIN_RADIO_CSN = 10;
constexpr uint8_t PIN_THROTTLE = A2;
constexpr uint8_t PIN_AILERON = A1;
constexpr uint8_t PIN_ELEVATOR = A0;
constexpr uint8_t PIN_RATE_KNOB = A4;   // control-throw / bank-angle limit knob
constexpr uint8_t PIN_ARM_SWITCH = A6;
constexpr uint8_t PIN_THROTTLE_MODE = A7;
constexpr uint8_t PIN_ELEVATOR_TRIM_UP = 3;
constexpr uint8_t PIN_ELEVATOR_TRIM_DOWN = 4;
constexpr uint8_t PIN_AILERON_TRIM_RIGHT = 5;
constexpr uint8_t PIN_AILERON_TRIM_LEFT = 6;

constexpr uint8_t RADIO_CHANNEL = 90;
const uint8_t RADIO_ADDRESS[6] = "NANO1";

constexpr int16_t AILERON_MIN_US = 1150;
constexpr int16_t AILERON_MAX_US = 1850;
constexpr int16_t ELEVATOR_MIN_US = 1150;
constexpr int16_t ELEVATOR_MAX_US = 1850;
constexpr int16_t THROTTLE_FULL_RAW = 950;
constexpr int16_t RATE_MIN_PM = 250;   // 25% control authority
constexpr int16_t RATE_MAX_PM = 1000;  // 100% control authority
constexpr int16_t TRIM_STEP_US = 6;
constexpr int16_t TRIM_MAX_US = 150;
constexpr uint32_t TRIM_REPEAT_MS = 60;
constexpr uint32_t EEPROM_SAVE_DELAY_MS = 250;

constexpr int EEPROM_ELEVATOR_TRIM = 0;
constexpr int EEPROM_AILERON_TRIM = 2;
constexpr int EEPROM_MAGIC_ADDRESS = 4;
constexpr uint8_t EEPROM_MAGIC = 0xA5;

constexpr bool REVERSE_AILERON = true;
constexpr bool REVERSE_ELEVATOR = true;
constexpr float FILTER_ALPHA = 0.35f;

struct __attribute__((packed)) RcPacket {
  uint16_t throttle;
  uint16_t aileron;
  uint16_t elevator;
  uint8_t armed;
  uint16_t pot1;
  uint16_t pot2;
  uint8_t switch2State;
};
static_assert(sizeof(RcPacket) == 12, "Packet must remain compatible with the existing receiver");

RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
RcPacket packet{};
int16_t elevatorTrimUs = 0;
int16_t aileronTrimUs = 0;
float filteredElevatorUs = 1500.0f;
float filteredAileronUs = 1500.0f;
uint32_t lastTrimUpdateMs = 0;
uint32_t lastTrimChangeMs = 0;
bool trimDirty = false;
bool radioReady = false;
uint8_t trimButtonFlags = 0;

int16_t stickPulse(uint16_t raw, int16_t low, int16_t high, bool reverse) {
  return reverse ? map(raw, 0, 1023, high, low)
                 : map(raw, 0, 1023, low, high);
}

void loadTrims() {
  if (EEPROM.read(EEPROM_MAGIC_ADDRESS) == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_ELEVATOR_TRIM, elevatorTrimUs);
    EEPROM.get(EEPROM_AILERON_TRIM, aileronTrimUs);
    elevatorTrimUs = constrain(elevatorTrimUs, -TRIM_MAX_US, TRIM_MAX_US);
    aileronTrimUs = constrain(aileronTrimUs, -TRIM_MAX_US, TRIM_MAX_US);
    return;
  }

  elevatorTrimUs = 0;
  aileronTrimUs = 0;
  EEPROM.put(EEPROM_ELEVATOR_TRIM, elevatorTrimUs);
  EEPROM.put(EEPROM_AILERON_TRIM, aileronTrimUs);
  EEPROM.update(EEPROM_MAGIC_ADDRESS, EEPROM_MAGIC);
}

void readSwitches() {
  packet.armed = analogRead(PIN_ARM_SWITCH) > 800 ? 1 : 0;

  const int modeRaw = analogRead(PIN_THROTTLE_MODE);
  if (modeRaw < 300) packet.switch2State = 0;
  else if (modeRaw < 700) packet.switch2State = 1;
  else packet.switch2State = 2;

#if ENABLE_AUX_POT1
  packet.pot1 = analogRead(A5);
#else
  packet.pot1 = 0;
#endif
  // The A4 knob sets the control-throw / bank-angle limit (dual rates),
  // 25%..100%. The aircraft scales its elevon throw and stabilization
  // bank angle by this value.
  packet.pot2 = map(constrain(analogRead(PIN_RATE_KNOB), 0, 1023), 0, 1023,
                    RATE_MIN_PM, RATE_MAX_PM);
}

void updateThrottle() {
  // Simple linear throttle; the reflex/cruise throttle modes are gone.
  const int distance = constrain(analogRead(PIN_THROTTLE), 0, THROTTLE_FULL_RAW);
  packet.throttle = map(distance, 0, THROTTLE_FULL_RAW, 1000, 2000);
}

void updateTrims() {
  const bool elevatorUp = digitalRead(PIN_ELEVATOR_TRIM_UP) == LOW;
  const bool elevatorDown = digitalRead(PIN_ELEVATOR_TRIM_DOWN) == LOW;
  const bool aileronRight = digitalRead(PIN_AILERON_TRIM_RIGHT) == LOW;
  const bool aileronLeft = digitalRead(PIN_AILERON_TRIM_LEFT) == LOW;
  const uint32_t now = millis();

  trimButtonFlags = (elevatorUp ? NanoLinkProtocol::TRIM_ELEVATOR_UP : 0) |
                    (elevatorDown ? NanoLinkProtocol::TRIM_ELEVATOR_DOWN : 0) |
                    (aileronRight ? NanoLinkProtocol::TRIM_AILERON_RIGHT : 0) |
                    (aileronLeft ? NanoLinkProtocol::TRIM_AILERON_LEFT : 0);

  if (now - lastTrimUpdateMs >= TRIM_REPEAT_MS) {
    lastTrimUpdateMs = now;
    if (elevatorUp && !elevatorDown) {
      elevatorTrimUs = constrain(elevatorTrimUs + TRIM_STEP_US, -TRIM_MAX_US, TRIM_MAX_US);
      trimDirty = true;
      lastTrimChangeMs = now;
    } else if (elevatorDown && !elevatorUp) {
      elevatorTrimUs = constrain(elevatorTrimUs - TRIM_STEP_US, -TRIM_MAX_US, TRIM_MAX_US);
      trimDirty = true;
      lastTrimChangeMs = now;
    }

    if (aileronRight && !aileronLeft) {
      aileronTrimUs = constrain(aileronTrimUs + TRIM_STEP_US, -TRIM_MAX_US, TRIM_MAX_US);
      trimDirty = true;
      lastTrimChangeMs = now;
    } else if (aileronLeft && !aileronRight) {
      aileronTrimUs = constrain(aileronTrimUs - TRIM_STEP_US, -TRIM_MAX_US, TRIM_MAX_US);
      trimDirty = true;
      lastTrimChangeMs = now;
    }
  }

  if (trimDirty && now - lastTrimChangeMs >= EEPROM_SAVE_DELAY_MS) {
    EEPROM.put(EEPROM_ELEVATOR_TRIM, elevatorTrimUs);
    EEPROM.put(EEPROM_AILERON_TRIM, aileronTrimUs);
    trimDirty = false;
  }
}

void readAndFilterSticks() {
  const int16_t aileron = stickPulse(analogRead(PIN_AILERON),
                                     AILERON_MIN_US, AILERON_MAX_US,
                                     REVERSE_AILERON);
  const int16_t elevator = stickPulse(analogRead(PIN_ELEVATOR),
                                      ELEVATOR_MIN_US, ELEVATOR_MAX_US,
                                      REVERSE_ELEVATOR);

  filteredAileronUs += FILTER_ALPHA * (aileron - filteredAileronUs);
  filteredElevatorUs += FILTER_ALPHA * (elevator - filteredElevatorUs);

  const int16_t aileronTrimOut = REVERSE_AILERON ? -aileronTrimUs : aileronTrimUs;
  const int16_t elevatorTrimOut = REVERSE_ELEVATOR ? -elevatorTrimUs : elevatorTrimUs;
  packet.aileron = constrain(static_cast<int16_t>(filteredAileronUs) + aileronTrimOut,
                             AILERON_MIN_US, AILERON_MAX_US);
  packet.elevator = constrain(static_cast<int16_t>(filteredElevatorUs) + elevatorTrimOut,
                              ELEVATOR_MIN_US, ELEVATOR_MAX_US);
}

} // namespace

void setup() {
  pinMode(PIN_ELEVATOR_TRIM_UP, INPUT_PULLUP);
  pinMode(PIN_ELEVATOR_TRIM_DOWN, INPUT_PULLUP);
  pinMode(PIN_AILERON_TRIM_RIGHT, INPUT_PULLUP);
  pinMode(PIN_AILERON_TRIM_LEFT, INPUT_PULLUP);
  loadTrims();

  packet.throttle = 1000;
  packet.aileron = 1500;
  packet.elevator = 1500;
  packet.armed = 0;

  radioReady = radio.begin();
  if (radioReady) {
    radio.openWritingPipe(RADIO_ADDRESS);
    radio.setChannel(RADIO_CHANNEL);
    radio.setDataRate(RF24_250KBPS);
#if BENCH_RF
    radio.setPALevel(RF24_PA_LOW);
    radio.setRetries(2, 5);
#else
    radio.setPALevel(RF24_PA_MAX);
    radio.setRetries(5, 15);
#endif
    radio.stopListening();
  }
}

void loop() {
  readSwitches();
  readAndFilterSticks();
  updateThrottle();
  updateTrims();
  packet.pot1 = (packet.pot1 & NanoLinkProtocol::POT1_AUX_MASK) |
                (static_cast<uint16_t>(trimButtonFlags) << NanoLinkProtocol::POT1_TRIM_SHIFT);

  if (radioReady) radio.write(&packet, sizeof(packet));
  delay(20);
}
