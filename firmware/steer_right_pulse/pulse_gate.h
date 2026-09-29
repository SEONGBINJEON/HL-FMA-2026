#ifndef HL_FMA_STEER_RIGHT_PULSE_GATE_H
#define HL_FMA_STEER_RIGHT_PULSE_GATE_H
#include "control_core.h"

namespace hl_steer_pulse {
static const uint32_t PULSE_MS = 200;
static const uint8_t STEER_PWM_VALUE = 60;
static const uint16_t MAX_ADC_CHANGE = 12;  // Relative trial bound, not calibrated travel.
static const uint8_t STATUS_TYPE = 134;
static const uint16_t PULSE_USED = 64;

class PulseGate {
 public:
  PulseGate() : used_(false), active_(false), started_at_(0), start_adc_(0) {}
  void update(const hl_manual::Snapshot &control, uint16_t adc, uint32_t now) {
    const bool eligible = control.state == hl_manual::ARMED &&
        control.reason == hl_manual::ACTIVE &&
        (control.flags & (hl_manual::BOOT_ID_VALID | hl_manual::BENCH)) ==
            (hl_manual::BOOT_ID_VALID | hl_manual::BENCH) &&
        control.requested_drive == 0 && control.requested_steer == 1;
    const bool adc_valid = adc > 1 && adc < 1022;
    if (active_) {
      const uint16_t change = adc >= start_adc_ ? adc - start_adc_ : start_adc_ - adc;
      if (!eligible || !adc_valid || change >= MAX_ADC_CHANGE ||
          static_cast<uint32_t>(now - started_at_) >= PULSE_MS)
        active_ = false;
    } else if (!used_ && eligible) {
      used_ = true;  // Consume this attempt even when the initial ADC rejects it.
      if (!adc_valid) return;
      start_adc_ = adc;
      started_at_ = now;
      active_ = true;
    }
  }
  bool active() const { return active_; }
  bool used() const { return used_; }
 private:
  bool used_;
  bool active_;
  uint32_t started_at_;
  uint16_t start_adc_;
};
}  // namespace hl_steer_pulse
#endif
