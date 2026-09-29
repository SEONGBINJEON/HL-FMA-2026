// REAL OUTPUT: two-stick drive PWM0..255, steer PWM60; no movement timer.
// One explicit prepare stays armed through neutral key gaps; operator and power-cutting spotter required.
// Key release commands zero output. No physical power/E-stop feedback or hardware watchdog.
#include <Arduino.h>
#include <EEPROM.h>
#include <util/atomic.h>
#include "control_core.h"
#include "step_gate.h"
const uint8_t TRIGS[3] = {53,41,37};
const uint8_t ECHOS[3] = {49,39,35};
uint16_t distances[3] = {999,999,999};
constexpr uint8_t JOY_STATUS_SIZE = 47;

constexpr uint8_t ENC_A = 2;
constexpr uint8_t ENC_B = 3;
constexpr uint8_t FRONT_PWM = 12;
constexpr uint8_t FRONT_DIR = 8;
constexpr uint8_t FRONT_BRK = 22;
constexpr uint8_t REAR_PWM = 44;
constexpr uint8_t REAR_DIR = 23;
constexpr uint8_t REAR_BRK = 24;
constexpr uint8_t STEER_PWM = 11;
// 2026-09-18: steering DIR moved D6 -> D50 (PB3, SPI MISO; SPI unused here) after the D6 line fault.
// D6 is left unconfigured (input). Diagnostics below read PORTB/PINB bit3 so they follow this pin.
constexpr uint8_t STEER_DIR = 50;
constexpr uint8_t STEER_BRK = 7;
constexpr uint8_t STEER_ADC = A15;
constexpr uint8_t RX_BUDGET = 32;
// Bytes 0..11 were found with a failed complement byte after a transport power loss.
// Preserve them as evidence and use a fresh, separately reserved record.
constexpr int EEPROM_BOOT_BASE = 16;

volatile uint32_t encoderBits = 0;

// 2026-09-19 HILL HOLD (ported from the electrical team's F8 PID HILL HOLD sketch, same gains/limits):
// while the board is ARMED and drive output is 0, the front/rear motors hold the encoder position with a
// PID capped at HOLD_MAX_PWM. Target = position captured when drive went to 0. Brake pins are engaged while
// the PID output is ~0. DISARMED -> brake only (as before). No change to steering, protocol, status, step gate.
constexpr float HOLD_KP = 0.17f, HOLD_KI = 0.23f, HOLD_KD = 0.27f;
constexpr int HOLD_MAX_PWM = 40, HOLD_MIN_PWM = 10;
constexpr long HOLD_DEADBAND = 6;
constexpr float HOLD_INTEGRAL_LIMIT = 30.0f;
constexpr uint32_t HOLD_PERIOD_MS = 20;
constexpr int HOLD_DIRECTION_SIGN = 1;   // encoder ++ on forward (A==B), forward = DIR LOW: same as the team sketch
bool holdActive = false;
long holdPosition = 0;
float holdIntegral = 0.0f, holdPrevError = 0.0f;
uint32_t holdLastTime = 0;

long encoderCountSigned() {
  uint32_t bits;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { bits = encoderBits; }
  return bits <= INT32_MAX ? static_cast<long>(bits) : -1L - static_cast<long>(UINT32_MAX - bits);
}

void driveBrakeOutputs() {
  analogWrite(FRONT_PWM, 0);
  digitalWrite(FRONT_BRK, HIGH);
  analogWrite(REAR_PWM, 0);
  digitalWrite(REAR_BRK, HIGH);
}

void driveRawOutputs(bool forward, int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(FRONT_DIR, forward ? LOW : HIGH);
  digitalWrite(REAR_DIR, forward ? LOW : HIGH);
  digitalWrite(FRONT_BRK, LOW);
  digitalWrite(REAR_BRK, LOW);
  analogWrite(FRONT_PWM, pwm);
  analogWrite(REAR_PWM, pwm);
}

void enterHold() {
  if (holdActive) return;
  driveBrakeOutputs();
  holdPosition = encoderCountSigned();
  holdIntegral = 0.0f; holdPrevError = 0.0f;
  holdLastTime = millis();
  holdActive = true;
}

void updateHillHold() {
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - holdLastTime) < HOLD_PERIOD_MS) return;
  const float dt = static_cast<uint32_t>(now - holdLastTime) / 1000.0f;
  holdLastTime = now;
  const long error = holdPosition - encoderCountSigned();
  const long controlError = (labs(error) <= HOLD_DEADBAND) ? 0 : error;
  holdIntegral = constrain(holdIntegral + controlError * dt, -HOLD_INTEGRAL_LIMIT, HOLD_INTEGRAL_LIMIT);
  const float derivative = dt > 0 ? (controlError - holdPrevError) / dt : 0.0f;
  holdPrevError = controlError;
  float output = (HOLD_KP * controlError + HOLD_KI * holdIntegral + HOLD_KD * derivative) * HOLD_DIRECTION_SIGN;
  output = constrain(output, -HOLD_MAX_PWM, HOLD_MAX_PWM);
  if (fabs(output) < 1.0f) { driveBrakeOutputs(); return; }
  int pwm = abs(static_cast<int>(output));
  if (pwm < HOLD_MIN_PWM) pwm = HOLD_MIN_PWM;
  driveRawOutputs(output > 0, pwm);   // pushed back -> forward force, pushed forward -> backward force
}
hl_manual::Core controller(0, false);  // Logical core uses BENCH; StepGate owns actual output.
hl_step_manual::StepGate step;
uint16_t steeringAdc = 0;

void encoderISR() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) ++encoderBits;
  else --encoderBits;
}

void applyOutputs() {
  const int8_t drive = step.actualDrive();
  const int8_t steer = step.actualSteer();
  if (drive != 0 && controller.drivePwm() != 0) {
    holdActive = false;   // driving: hold state is re-captured at the next stop
    driveRawOutputs(drive > 0, controller.drivePwm());
  } else if (controller.snapshot().state == hl_manual::ARMED) {
    enterHold();          // planned stop while the host session is ARMED: hold the encoder position
    updateHillHold();
  } else {
    holdActive = false;   // DISARMED (arrival/stop/E-stop/hand moving): brake only, as before
    driveBrakeOutputs();
  }
  if (steer != 0) {
    digitalWrite(STEER_DIR, steer < 0 ? LOW : HIGH);
    digitalWrite(STEER_BRK, LOW);
    analogWrite(STEER_PWM, hl_step_manual::STEER_PWM_VALUE);
  } else {
    analogWrite(STEER_PWM, 0);
    digitalWrite(STEER_BRK, HIGH);
  }
}

void serviceSafety() {
  const uint32_t now = millis();
  controller.tick(now);
  step.update(controller.snapshot(), steeringAdc, now);
  applyOutputs();
}

bool initializeBootIdentity(uint32_t &bootId) {
  uint8_t prior[12], next[12];
  for (uint8_t i = 0; i < 12; ++i) prior[i] = EEPROM.read(EEPROM_BOOT_BASE + i);
  if (!hl_manual::nextBootRecord(prior, next, bootId)) return false;
  for (uint8_t i = 0; i < 12; ++i) EEPROM.update(EEPROM_BOOT_BASE + i, next[i]);
  for (uint8_t i = 0; i < 12; ++i) {
    if (EEPROM.read(EEPROM_BOOT_BASE + i) != next[i]) { bootId = 0; return false; }
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
  for(uint8_t i=0;i<3;++i){digitalWrite(TRIGS[i],LOW);pinMode(TRIGS[i],OUTPUT);pinMode(ECHOS[i],INPUT);}
  Serial.begin(115200);
}

void loop() {
  serviceSafety();
  // One channel per pass: bounded 7ms echo wait, safety checked on both sides.
  static uint32_t lastSonar = 0;
  static uint8_t channel = 0;
  if(static_cast<uint32_t>(millis()-lastSonar)>=12){
    lastSonar=millis();
    digitalWrite(TRIGS[channel],LOW);delayMicroseconds(2);
    digitalWrite(TRIGS[channel],HIGH);delayMicroseconds(10);digitalWrite(TRIGS[channel],LOW);
    unsigned long pulse=pulseIn(ECHOS[channel],HIGH,7000);
    distances[channel]=pulse ? static_cast<uint16_t>(pulse*0.0343f/2.0f) : 999;
    channel=(channel+1)%3;
    serviceSafety();
  }
  steeringAdc = analogRead(STEER_ADC);
  controller.setInputs(steeringAdc, false, millis());
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
  if (controller.statusDue(now) && Serial.availableForWrite() >= JOY_STATUS_SIZE) {
    uint32_t bits;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { bits = encoderBits; }
    const int32_t count = bits <= INT32_MAX ? static_cast<int32_t>(bits)
        : -1 - static_cast<int32_t>(UINT32_MAX - bits);
    uint8_t frame[JOY_STATUS_SIZE] = {};
    if (controller.makeStatus(frame, now, count)) {
      serviceSafety();
      frame[2] = hl_step_manual::STATUS_TYPE;
      frame[26] = static_cast<uint8_t>(controller.drivePwm() ? step.actualDrive() : 0);
      frame[27] = static_cast<uint8_t>(step.actualSteer());
      const hl_manual::Snapshot logical = controller.snapshot();
      frame[22] = logical.state;
      frame[23] = logical.reason;
      frame[24] = static_cast<uint8_t>(logical.requested_drive);
      frame[25] = static_cast<uint8_t>(logical.requested_steer);
      uint16_t flags = logical.flags & hl_manual::BOOT_ID_VALID;
      if (flags) flags |= hl_manual::DRIVE_ENABLED | hl_manual::STEER_ENABLED;
      if (step.used()) flags |= hl_step_manual::STEP_USED;
      frame[28] = static_cast<uint8_t>(flags);
      frame[29] = static_cast<uint8_t>(flags >> 8);
      frame[3] = JOY_STATUS_SIZE-6;
      for(uint8_t i=0;i<3;++i){frame[30+i*2]=distances[i]&255;frame[31+i*2]=distances[i]>>8;}
      frame[36] = 0; // Distance-only telemetry; no ultrasonic actuation gate.
      frame[37] = controller.drivePwm();
      // Read registers directly: digitalRead(D11) would disable its PWM.
      uint16_t steerCompare;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { steerCompare = OCR1A; }
      frame[38] = static_cast<uint8_t>(steerCompare);
      frame[39] = static_cast<uint8_t>(steerCompare >> 8);
      frame[40] = TCCR1A;
      frame[41] = TCCR1B;
      // bit0 = STEER_DIR (D50 = PB3), bit1 = STEER_BRK (D7 = PH4); frame[42] bit0 = STEER_PWM (D11 = PB5) mode.
      frame[42] = ((DDRB & _BV(5)) ? 1 : 0) |
                  ((DDRB & _BV(3)) ? 2 : 0) | ((DDRH & _BV(4)) ? 4 : 0);
      frame[43] = ((PORTB & _BV(3)) ? 1 : 0) | ((PORTH & _BV(4)) ? 2 : 0);
      frame[44] = ((PINB & _BV(3)) ? 1 : 0) | ((PINH & _BV(4)) ? 2 : 0);
      const uint16_t crc = hl_manual::crc16(frame + 2, JOY_STATUS_SIZE-4);
      frame[45] = static_cast<uint8_t>(crc);
      frame[46] = static_cast<uint8_t>(crc >> 8);
      if (Serial.write(frame, sizeof(frame)) != sizeof(frame)) {
        controller.inputOverload(millis());
        serviceSafety();
      }
    }
  }
}
