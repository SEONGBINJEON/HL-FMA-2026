#ifndef HL_FMA_STEP_MANUAL_GATE_H
#define HL_FMA_STEP_MANUAL_GATE_H
#include "control_core.h"

namespace hl_step_manual {
static const uint32_t STEER_MS = 1000;
static const uint8_t DRIVE_PWM_VALUE = 110;
static const uint8_t STEER_PWM_VALUE = 60;
static const uint16_t MAX_ADC_CHANGE = 100;  // Relative trial bound, not calibration.
static const uint8_t STATUS_TYPE = 144;
static const uint16_t STEP_USED = 64;

class StepGate {
 public:
  StepGate() : used_(false), drive_(0), steer_(0), steer_input_(0),
               steering_blocked_(false), steering_started_at_(0), steering_start_adc_(0) {}

  void update(const hl_manual::Snapshot &control, uint16_t adc, uint32_t now) {
    const bool identity_ok =
        (control.flags & (hl_manual::BOOT_ID_VALID | hl_manual::BENCH)) ==
        (hl_manual::BOOT_ID_VALID | hl_manual::BENCH);
    if (identity_ok && control.state == hl_manual::DISARMED &&
        control.reason == hl_manual::NEUTRAL &&
        control.requested_drive == 0 && control.requested_steer == 0) {
      used_ = false;
      zeroOutputs();
      return;
    }

    const bool candidate = identity_ok && control.state == hl_manual::ARMED &&
        control.reason == hl_manual::ACTIVE;
    if (!candidate) {
      zeroOutputs();
      return;
    }
    const int8_t requested_drive = control.requested_drive;
    const int8_t requested_steer = control.requested_steer;
    const bool axes_ok = requested_drive >= -1 && requested_drive <= 1 &&
        requested_steer >= -1 && requested_steer <= 1 &&
        (requested_drive != 0 || requested_steer != 0);
    if (!axes_ok) {
      zeroOutputs();
      return;
    }

    used_ = true;
    drive_ = requested_drive;  // Drive remains live while its key is held.

    if (requested_steer == 0) {
      steer_ = steer_input_ = 0;
      steering_blocked_ = false;
      return;
    }
    // The observed sensor rails are directional limits. They must never stop
    // drive, and they must still allow steering back toward the usable range.
    const bool outward_at_rail = (requested_steer < 0 && adc >= 1022) ||
                                 (requested_steer > 0 && adc <= 1);
    if (outward_at_rail) {
      steer_input_ = requested_steer;
      steer_ = 0;
      steering_blocked_ = true;
      return;
    }
    if (requested_steer != steer_input_) {
      // Core requires an observed zero for this axis before sign reversal.
      steer_input_ = requested_steer;
      steer_ = requested_steer;
      steering_blocked_ = false;
      steering_started_at_ = now;
      steering_start_adc_ = adc;
    }
    if (steering_blocked_) {
      steer_ = 0;
      return;
    }
    const uint16_t change = adc >= steering_start_adc_ ? adc - steering_start_adc_
                                                        : steering_start_adc_ - adc;
    if (static_cast<uint32_t>(now - steering_started_at_) >= STEER_MS ||
        change >= MAX_ADC_CHANGE) {
      steer_ = 0;
      steering_blocked_ = true;
    }
  }

  bool used() const { return used_; }
  bool active() const { return drive_ != 0 || steer_ != 0; }
  int8_t actualDrive() const { return drive_; }
  int8_t actualSteer() const { return steer_; }

 private:
  bool used_;
  int8_t drive_;
  int8_t steer_;
  int8_t steer_input_;
  bool steering_blocked_;
  uint32_t steering_started_at_;
  uint16_t steering_start_adc_;

  void zeroOutputs() {
    drive_ = steer_ = steer_input_ = 0;
    steering_blocked_ = false;
  }
};
}  // namespace hl_step_manual
#endif
