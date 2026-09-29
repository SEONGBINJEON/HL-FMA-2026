// REAL OUTPUT: one front-wheel pulse, PWM30/255, at most100ms by loop timer.
// Supported vehicle with wheels clear only. Not a driving firmware.
// No physical power/E-stop feedback, no watchdog, and no rear/steer actuation.
#include <Arduino.h>
#include <EEPROM.h>
#include <util/atomic.h>
#include "control_core.h"
#include "pulse_gate.h"

constexpr uint8_t ENC_A = 2;
constexpr uint8_t ENC_B = 3;
constexpr uint8_t FRONT_PWM = 12;
constexpr uint8_t FRONT_DIR = 8;
constexpr uint8_t FRONT_BRK = 22;
constexpr uint8_t REAR_PWM = 44;
constexpr uint8_t REAR_DIR = 23;
constexpr uint8_t REAR_BRK = 24;
constexpr uint8_t STEER_PWM = 11;
constexpr uint8_t STEER_DIR = 6;
constexpr uint8_t STEER_BRK = 7;
constexpr uint8_t STEER_ADC = A15;
constexpr uint8_t RX_BUDGET = 32;

volatile uint32_t encoderBits = 0;
hl_manual::Core controller(0, false);  // Exact copied core remains logically BENCH.
hl_front_pulse::PulseGate pulse;

void encoderISR() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) ++encoderBits;
  else --encoderBits;
}

void applyOutputs() {
  // These two motors never have a nonzero PWM or released brake path.
  analogWrite(REAR_PWM, 0);
  digitalWrite(REAR_BRK, HIGH);
  analogWrite(STEER_PWM, 0);
  digitalWrite(STEER_BRK, HIGH);
  if (pulse.active()) {
    digitalWrite(FRONT_DIR, LOW);  // Original forward setting; physical direction unmeasured.
    digitalWrite(FRONT_BRK, LOW);
    analogWrite(FRONT_PWM, hl_front_pulse::FRONT_PWM_VALUE);
  } else {
    analogWrite(FRONT_PWM, 0);
    digitalWrite(FRONT_BRK, HIGH);
  }
}

void serviceSafety() {
  const uint32_t now = millis();
  controller.tick(now);
  pulse.update(controller.snapshot(), now);
  applyOutputs();
}

bool initializeBootIdentity(uint32_t &bootId) {
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
  applyOutputs();
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(STEER_ADC, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  uint32_t bootId = 0;
  const bool valid = initializeBootIdentity(bootId);
  controller = hl_manual::Core(bootId, valid);
  Serial.begin(115200);
}

void loop() {
  serviceSafety();
  controller.setInputs(analogRead(STEER_ADC), false, millis());
  serviceSafety();
  if (Serial.available() > RX_BUDGET) {
    controller.inputOverload(millis());
    serviceSafety();
    for (uint8_t i = 0; i < RX_BUDGET && Serial.available(); ++i) {
      serviceSafety();
      Serial.read();
    }
  } else {
    for (uint8_t i = 0; i < RX_BUDGET && Serial.available(); ++i) {
      serviceSafety();
      const int value = Serial.read();
      if (value >= 0) controller.feed(static_cast<uint8_t>(value), millis());
      serviceSafety();  // Apply stop/error/accepted control immediately after this byte.
    }
  }
  serviceSafety();
  const uint32_t now = millis();
  if (controller.statusDue(now) && Serial.availableForWrite() >= hl_manual::STATUS_SIZE) {
    uint32_t bits;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { bits = encoderBits; }
    const int32_t count = bits <= INT32_MAX ? static_cast<int32_t>(bits)
        : -1 - static_cast<int32_t>(UINT32_MAX - bits);
    uint8_t frame[hl_manual::STATUS_SIZE];
    if (controller.makeStatus(frame, now, count)) {
      serviceSafety();
      frame[2] = hl_front_pulse::STATUS_TYPE;
      frame[26] = pulse.active() ? 1 : 0;
      frame[27] = 0;
      const hl_manual::Snapshot logical = controller.snapshot();
      frame[22] = logical.state;
      frame[23] = logical.reason;
      frame[24] = static_cast<uint8_t>(logical.requested_drive);
      frame[25] = static_cast<uint8_t>(logical.requested_steer);
      uint16_t flags = logical.flags & hl_manual::BOOT_ID_VALID;
      if (flags) flags |= hl_manual::DRIVE_ENABLED;
      if (pulse.used()) flags |= hl_front_pulse::PULSE_USED;
      frame[28] = static_cast<uint8_t>(flags);
      frame[29] = static_cast<uint8_t>(flags >> 8);
      const uint16_t crc = hl_manual::crc16(frame + 2, 28);
      frame[30] = static_cast<uint8_t>(crc);
      frame[31] = static_cast<uint8_t>(crc >> 8);
      if (Serial.write(frame, sizeof(frame)) != sizeof(frame)) {
        controller.inputOverload(millis());
        serviceSafety();
      }
    }
  }
}
