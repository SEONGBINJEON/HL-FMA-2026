#ifndef HL_FMA_FRONT_PULSE_GATE_H
#define HL_FMA_FRONT_PULSE_GATE_H
#include "control_core.h"

namespace hl_front_pulse {
static const uint32_t PULSE_MS = 100;
static const uint8_t FRONT_PWM_VALUE = 30;
static const uint8_t STATUS_TYPE = 131;
static const uint16_t PULSE_USED = 64;

// Exactly one front pulse per construction/MCU boot. No reset/rearm method.
// The caller supplies snapshots only from the unchanged, BENCH-configured Core.
class PulseGate {
 public:
  PulseGate() : used_(false), active_(false), started_at_(0) {}
  void update(const hl_manual::Snapshot &control, uint32_t now) {
    const bool eligible = control.state == hl_manual::ARMED &&
        control.reason == hl_manual::ACTIVE &&
        (control.flags & (hl_manual::BOOT_ID_VALID | hl_manual::BENCH)) ==
            (hl_manual::BOOT_ID_VALID | hl_manual::BENCH) &&
        control.requested_drive == 1 && control.requested_steer == 0;
    if (active_) {
      if (!eligible || static_cast<uint32_t>(now - started_at_) >= PULSE_MS)
        active_ = false;
    } else if (!used_ && eligible) {
      used_ = true;  // Consume before any physical output is enabled.
      active_ = true;
      started_at_ = now;
    }
  }
  bool active() const { return active_; }
  bool used() const { return used_; }
 private:
  bool used_;
  bool active_;
  uint32_t started_at_;
};
}  // namespace hl_front_pulse
#endif
