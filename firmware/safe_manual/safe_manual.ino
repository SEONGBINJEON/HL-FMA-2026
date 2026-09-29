// HL FMA manual protocol v1: BENCH ONLY. No motor enable path is compiled.
// Original firmware and original_compile_check remain separate and unchanged.
#include <Arduino.h>
#include <EEPROM.h>
#include <util/atomic.h>
#include "control_core.h"

using hl_manual::Core;

constexpr bool ACTUATION_ENABLED = false;
static_assert(!ACTUATION_ENABLED,
              "This BENCH sketch must not actuate; a reviewed hardware implementation is required.");
constexpr int INTERLOCK_PIN = -1;  // No verified E-stop/power-feedback input exists.
constexpr uint8_t ENC_A = 2;
constexpr uint8_t ENC_B = 3;
constexpr uint8_t STEER_PWM = 11;
constexpr uint8_t STEER_DIR = 6;
constexpr uint8_t STEER_BRK = 7;
constexpr uint8_t FRONT_PWM = 12;
constexpr uint8_t FRONT_DIR = 8;
constexpr uint8_t FRONT_BRK = 22;
constexpr uint8_t REAR_PWM = 44;
constexpr uint8_t REAR_DIR = 23;
constexpr uint8_t REAR_BRK = 24;
constexpr uint8_t STEER_ADC = A15;
// Historical upper-bound candidates only, never applied by this sketch.
constexpr uint8_t DRIVE_PWM_CANDIDATE = 45;
constexpr uint8_t STEER_PWM_CANDIDATE = 60;
constexpr uint8_t RX_BUDGET = 32;

// Unsigned modular ISR arithmetic avoids signed-overflow undefined behavior.
volatile uint32_t encoderBits = 0;
Core controller(0, false);

void encoderISR() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) ++encoderBits;
  else --encoderBits;
}

void stopOutputs() {
  analogWrite(FRONT_PWM, 0);
  analogWrite(REAR_PWM, 0);
  analogWrite(STEER_PWM, 0);
  digitalWrite(FRONT_BRK, HIGH);
  digitalWrite(REAR_BRK, HIGH);
  digitalWrite(STEER_BRK, HIGH);
}

bool initializeBootIdentity(uint32_t &bootId) {
  // Reserved EEPROM bytes 0..11. Never touch other bytes.
  uint8_t prior[12], next[12];
  for (uint8_t i = 0; i < 12; ++i) prior[i] = EEPROM.read(i);
  if (!hl_manual::nextBootRecord(prior, next, bootId)) return false;
  for (uint8_t i = 0; i < 12; ++i) EEPROM.update(i, next[i]);
  for (uint8_t i = 0; i < 12; ++i) {
    if (EEPROM.read(i) != next[i]) { bootId = 0; return false; }
  }
  return true;
}

void setup() {
  // Latch brake HIGH before enabling each output driver. External power remains OFF.
  const uint8_t brakes[] = {FRONT_BRK, REAR_BRK, STEER_BRK};
  const uint8_t pwms[] = {FRONT_PWM, REAR_PWM, STEER_PWM};
  const uint8_t dirs[] = {FRONT_DIR, REAR_DIR, STEER_DIR};
  for (uint8_t i = 0; i < 3; ++i) digitalWrite(brakes[i], HIGH);
  for (uint8_t i = 0; i < 3; ++i) digitalWrite(pwms[i], LOW);
  for (uint8_t i = 0; i < 3; ++i) {
    pinMode(brakes[i], OUTPUT);
    pinMode(pwms[i], OUTPUT);
    digitalWrite(dirs[i], LOW);
    pinMode(dirs[i], OUTPUT);
  }
  stopOutputs();
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(STEER_ADC, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  uint32_t bootId = 0;
  const bool identityValid = initializeBootIdentity(bootId);
  controller = Core(bootId, identityValid);
  Serial.begin(115200);
}

void loop() {
  // The stop deadline is checked before RX and before each further bounded step.
  controller.tick(millis());
  stopOutputs();
  controller.setInputs(analogRead(STEER_ADC), false, millis());
  if (Serial.available() > RX_BUDGET) {
    controller.inputOverload(millis());
    for (uint8_t i = 0; i < RX_BUDGET && Serial.available(); ++i) {
      controller.tick(millis());
      Serial.read();
    }
  } else {
    for (uint8_t i = 0; i < RX_BUDGET && Serial.available(); ++i) {
      controller.tick(millis());
      const int value = Serial.read();
      if (value >= 0) controller.feed(static_cast<uint8_t>(value), millis());
    }
  }
  controller.tick(millis());
  stopOutputs();
  const uint32_t now = millis();
  if (controller.statusDue(now) && Serial.availableForWrite() >= hl_manual::STATUS_SIZE) {
    uint32_t countBits;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { countBits = encoderBits; }
    // Explicit modular conversion, including INT32_MIN, without signed overflow.
    const int32_t count = countBits <= INT32_MAX ? static_cast<int32_t>(countBits)
        : -1 - static_cast<int32_t>(UINT32_MAX - countBits);
    uint8_t frame[hl_manual::STATUS_SIZE];
    if (controller.makeStatus(frame, now, count)) {
      // Only this loop writes serial; the entire frame fits in the TX ring now.
      if (Serial.write(frame, sizeof(frame)) != sizeof(frame))
        controller.inputOverload(millis());
    }
  }
}
