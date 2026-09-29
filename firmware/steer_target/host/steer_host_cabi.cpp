// C ABI for host-side reuse of the C1.1 target-steering core AND the verified
// manual type150 core (as an offline board model). Built into one shared
// library for ctypes; see build.sh. No Arduino headers, no I/O.
//
// The board model reproduces joystick_manual.ino's 47-byte type150 status
// framing (register diagnostics are zero) and its StepGate output path so the
// host adapter is exercised with real protocol bytes. It is a model, not the
// uploaded firmware.
#include <stdint.h>
#include <string.h>
#include "../steer_target_core.h"
#include "../../joystick_manual/control_core.h"
#include "../../joystick_manual/step_gate.h"

extern "C" {

struct StcConfig {
  uint8_t calibrated;
  uint8_t point_count;
  int16_t angle_cdeg[8];
  uint16_t adc[8];
  uint16_t usable_adc_min, usable_adc_max;
  int8_t steer_sign_for_adc_increase;
  uint16_t stop_band_adc, restart_band_adc, sensor_margin_adc;
  uint32_t command_stale_ms, sensor_stale_ms, reversal_neutral_ms;
  uint8_t limit_policy;  // 1 reject, 2 clamp
};

struct StcOutput {
  int8_t steer;
  uint8_t reason;
  uint8_t fault_latched;
  uint8_t target_clamped;
  uint16_t target_adc;
};

static hl_steer_target::Config toConfig(const StcConfig *c) {
  hl_steer_target::Config k;
  if (!c) return k;
  k.calibrated = c->calibrated != 0;
  k.point_count = c->point_count > 8 ? 8 : c->point_count;
  for (uint8_t i = 0; i < k.point_count; ++i) { k.points[i].angle_cdeg = c->angle_cdeg[i]; k.points[i].adc = c->adc[i]; }
  k.usable_adc_min = c->usable_adc_min; k.usable_adc_max = c->usable_adc_max;
  k.steer_sign_for_adc_increase = c->steer_sign_for_adc_increase;
  k.stop_band_adc = c->stop_band_adc; k.restart_band_adc = c->restart_band_adc;
  k.sensor_margin_adc = c->sensor_margin_adc;
  k.command_stale_ms = c->command_stale_ms; k.sensor_stale_ms = c->sensor_stale_ms;
  k.reversal_neutral_ms = c->reversal_neutral_ms;
  k.limit_policy = static_cast<hl_steer_target::LimitPolicy>(c->limit_policy);
  return k;
}

int stc_config_valid(const StcConfig *c) { return hl_steer_target::configValid(toConfig(c)) ? 1 : 0; }
int stc_angle_to_adc(const StcConfig *c, int16_t angle_cdeg, uint16_t *adc) {
  uint16_t v = 0;
  const bool ok = hl_steer_target::angleToAdc(toConfig(c), angle_cdeg, v);
  if (adc) *adc = v;
  return ok ? 1 : 0;
}
int stc_adc_to_angle(const StcConfig *c, uint16_t adc, int16_t *angle_cdeg) {
  int16_t v = 0;
  const bool ok = hl_steer_target::adcToAngle(toConfig(c), adc, v);
  if (angle_cdeg) *angle_cdeg = v;
  return ok ? 1 : 0;
}
void *stc_create(const StcConfig *c) { return new hl_steer_target::Core(toConfig(c)); }
void stc_destroy(void *h) { delete static_cast<hl_steer_target::Core *>(h); }
void stc_set_ready(void *h, int ready, uint32_t now) { static_cast<hl_steer_target::Core *>(h)->setReady(ready != 0, now); }
void stc_stop(void *h, uint32_t now) { static_cast<hl_steer_target::Core *>(h)->stop(now); }
int stc_set_target(void *h, uint32_t sequence, int16_t angle_cdeg, uint32_t host_time_ms, uint32_t now) {
  hl_steer_target::TargetCommand t; t.sequence = sequence; t.angle_cdeg = angle_cdeg; t.host_time_ms = host_time_ms;
  return static_cast<hl_steer_target::Core *>(h)->setTarget(t, now) ? 1 : 0;
}
void stc_set_measurement(void *h, uint16_t adc, uint32_t now) { static_cast<hl_steer_target::Core *>(h)->setMeasurement(adc, now); }
void stc_update(void *h, uint32_t now, StcOutput *out) {
  const hl_steer_target::Output o = static_cast<hl_steer_target::Core *>(h)->update(now);
  if (!out) return;
  out->steer = o.steer; out->reason = o.reason; out->fault_latched = o.fault_latched ? 1 : 0;
  out->target_clamped = o.target_clamped ? 1 : 0; out->target_adc = o.target_adc;
}
uint32_t stc_last_sequence(void *h) { return static_cast<hl_steer_target::Core *>(h)->lastSequence(); }

// ---------------------------------------------------------------- board model
struct Board {
  hl_manual::Core core;
  hl_step_manual::StepGate step;
  uint16_t adc;
  explicit Board(uint32_t boot) : core(boot, boot != 0), adc(0) {}
  void service(uint32_t now) { core.tick(now); step.update(core.snapshot(), adc, now); }
};

void *brd_create(uint32_t boot_id) { return new Board(boot_id); }
void brd_destroy(void *h) { delete static_cast<Board *>(h); }
void brd_set_inputs(void *h, uint16_t adc, uint32_t now) { Board *b = static_cast<Board *>(h); b->adc = adc; b->core.setInputs(adc, false, now); b->service(now); }
void brd_tick(void *h, uint32_t now) { static_cast<Board *>(h)->service(now); }
void brd_feed(void *h, uint8_t byte, uint32_t now) { Board *b = static_cast<Board *>(h); b->service(now); b->core.feed(byte, now); b->service(now); }
int brd_status_due(void *h, uint32_t now) { return static_cast<Board *>(h)->core.statusDue(now) ? 1 : 0; }
// Returns 1 and fills 47 bytes when a status frame is produced (mirrors joystick_manual.ino).
int brd_make_status(void *h, uint8_t *out47, uint32_t now, int32_t encoder) {
  Board *b = static_cast<Board *>(h);
  b->service(now);
  uint8_t frame[47]; memset(frame, 0, sizeof(frame));
  if (!b->core.makeStatus(frame, now, encoder)) return 0;
  b->service(now);
  frame[2] = hl_step_manual::STATUS_TYPE;
  frame[3] = 47 - 6;
  frame[26] = static_cast<uint8_t>(b->core.drivePwm() ? b->step.actualDrive() : 0);
  frame[27] = static_cast<uint8_t>(b->step.actualSteer());
  const hl_manual::Snapshot logical = b->core.snapshot();
  frame[22] = logical.state; frame[23] = logical.reason;
  frame[24] = static_cast<uint8_t>(logical.requested_drive); frame[25] = static_cast<uint8_t>(logical.requested_steer);
  uint16_t flags = logical.flags & hl_manual::BOOT_ID_VALID;
  if (flags) flags |= hl_manual::DRIVE_ENABLED | hl_manual::STEER_ENABLED;
  if (b->step.used()) flags |= hl_step_manual::STEP_USED;
  frame[28] = static_cast<uint8_t>(flags); frame[29] = static_cast<uint8_t>(flags >> 8);
  for (uint8_t i = 0; i < 3; ++i) { frame[30 + i * 2] = 999 & 255; frame[31 + i * 2] = 999 >> 8; }
  frame[36] = 0; frame[37] = b->core.drivePwm();
  // frame[38..44]: register diagnostics unavailable in the model; zero.
  const uint16_t crc = hl_manual::crc16(frame + 2, 47 - 4);
  frame[45] = static_cast<uint8_t>(crc); frame[46] = static_cast<uint8_t>(crc >> 8);
  memcpy(out47, frame, 47);
  return 1;
}
void brd_snapshot(void *h, int8_t *actual_drive, int8_t *actual_steer, uint8_t *state, uint8_t *reason, uint8_t *drive_pwm) {
  Board *b = static_cast<Board *>(h);
  const hl_manual::Snapshot s = b->core.snapshot();
  if (actual_drive) *actual_drive = b->core.drivePwm() ? b->step.actualDrive() : 0;
  if (actual_steer) *actual_steer = b->step.actualSteer();
  if (state) *state = s.state;
  if (reason) *reason = s.reason;
  if (drive_pwm) *drive_pwm = b->core.drivePwm();
}

}  // extern "C"
