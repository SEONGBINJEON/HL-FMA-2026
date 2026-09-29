// CANDIDATE, NOT UPLOADED. Hardware-free target steering logic for the HL FMA
// vehicle. No Arduino headers, no serial, no pins. Shares conventions with
// firmware/joystick_manual/control_core.h (steer -1/0/+1, uint32 millis).
//
// Policy summary (tested in tests/steer_target_test.cpp):
//  * Output is always 0 unless: calibration valid AND supervisor ready AND a
//    target was accepted while ready AND target/sensor are fresh AND no fault.
//  * Boot has no target; the core never moves toward an assumed centre.
//  * Every fault (stale command, stale/invalid sensor, rejected target, stop,
//    ready dropping) latches. Resuming needs ready==true AND a new target
//    sequence. Retransmitting an old sequence never renews or resumes.
//  * Target reached -> 0 (AT_TARGET) with hysteresis (stop_band/restart_band).
//  * Direction reversal always passes through at least one 0 output for at
//    least reversal_neutral_ms, matching the manual core's neutral-before-
//    reversal rule.
//  * Targets outside the measured usable range are REJECTED or CLAMPED per
//    Config::limit_policy; the policy must be set explicitly.
//  * Host time inside a command is diagnostic only. Freshness is judged by the
//    local clock at the moment a *new* sequence was accepted.
#ifndef HL_FMA_STEER_TARGET_CORE_H
#define HL_FMA_STEER_TARGET_CORE_H

#include <stddef.h>
#include <stdint.h>

namespace hl_steer_target {

static const uint8_t MAX_POINTS = 8;
static const uint16_t ADC_MAX = 1023;
// Electrical rails, identical to step_gate.h (left>=1022 / right<=1 guards).
static const uint16_t ADC_RAIL_LOW = 1;
static const uint16_t ADC_RAIL_HIGH = 1022;

enum LimitPolicy : uint8_t { LIMIT_UNSET = 0, LIMIT_REJECT = 1, LIMIT_CLAMP = 2 };

// angle_cdeg: wheel angle in 0.01 degree, positive = left (FLU yaw sign).
struct CalPoint {
  int16_t angle_cdeg;
  uint16_t adc;
};

struct Config {
  bool calibrated;                 // false => configValid() is false
  CalPoint points[MAX_POINTS];     // strictly increasing adc, strictly monotonic angle,
  uint8_t point_count;             // must contain angle 0 (straight). >= 3 points.
  uint16_t usable_adc_min;         // measured usable limits, inside the table range
  uint16_t usable_adc_max;         // and on opposite sides of the straight point
  int8_t steer_sign_for_adc_increase;  // manual-core steer value (-1/+1) that raises ADC
  uint16_t stop_band_adc;          // |target-adc| <= stop_band  -> hold (0)
  uint16_t restart_band_adc;       // |target-adc| >= restart_band -> move; >= stop_band, > 0
  uint16_t sensor_margin_adc;      // adc outside usable +/- margin => SENSOR_INVALID
  uint32_t command_stale_ms;       // > 0
  uint32_t sensor_stale_ms;        // > 0
  uint32_t reversal_neutral_ms;    // minimum zero-output time before a sign flip
  LimitPolicy limit_policy;        // must not be LIMIT_UNSET
  Config();
};

bool configValid(const Config &c);
// Piecewise-linear table lookups. False when outside the table.
bool angleToAdc(const Config &c, int16_t angle_cdeg, uint16_t &adc);
bool adcToAngle(const Config &c, uint16_t adc, int16_t &angle_cdeg);
// True when adc increases with angle (left).
bool adcRisesWithLeft(const Config &c);

enum Reason : uint8_t {
  NOT_READY = 0, NO_CALIBRATION = 1, NO_TARGET = 2, STOPPED = 3,
  COMMAND_STALE = 4, SENSOR_STALE = 5, SENSOR_INVALID = 6,
  TARGET_REJECTED = 7, AT_TARGET = 8, REVERSAL_NEUTRAL = 9,
  AT_LIMIT = 10, MOVING = 11
};

struct Output {
  int8_t steer;         // -1/0/+1 in manual-core convention; nonzero only for MOVING
  Reason reason;
  bool fault_latched;   // true => needs ready && new sequence to resume
  bool target_clamped;  // LIMIT_CLAMP applied to the active target
  uint16_t target_adc;  // 0 when no active target
};

struct TargetCommand {
  uint32_t sequence;      // host counter; must exceed the previous accepted value
  int16_t angle_cdeg;
  uint32_t host_time_ms;  // stored for diagnostics only, never compared to local clock
};

class Core {
 public:
  explicit Core(const Config &config = Config());
  // Supervisor readiness (e.g. manual core ARMED with a valid lease).
  // A falling edge latches STOPPED and drops the target.
  void setReady(bool ready, uint32_t now);
  // Explicit stop: latch, clear target. Only a new sequence while ready resumes.
  void stop(uint32_t now);
  // Returns true when accepted as a NEW target. False: no calibration, not
  // ready, sequence not increasing, or angle outside the table under REJECT.
  // Rejection under REJECT policy latches TARGET_REJECTED.
  bool setTarget(const TargetCommand &cmd, uint32_t now);
  void setMeasurement(uint16_t adc, uint32_t now);
  // Call every loop. The returned steer must be applied for this loop only.
  Output update(uint32_t now);
  Output last() const { return last_; }
  bool hasTarget() const { return has_target_; }
  uint32_t lastSequence() const { return last_sequence_; }
  uint32_t lastHostTime() const { return last_host_time_; }

 private:
  Config config_;
  bool config_valid_;
  bool ready_;
  bool latched_;
  Reason latch_reason_;
  bool has_target_;
  bool has_sequence_;
  uint32_t last_sequence_;
  uint32_t last_host_time_;
  uint16_t target_adc_;
  bool target_clamped_;
  uint32_t target_at_;
  bool has_measurement_;
  uint16_t adc_;
  uint32_t measured_at_;
  int8_t last_steer_;
  int8_t last_dir_;        // last nonzero steer
  bool neutral_seen_;      // a 0 output happened after last_dir_
  uint32_t neutral_since_;
  Output last_;

  void latch(Reason reason);
  Output emit(int8_t steer, Reason reason, uint32_t now);
};

}  // namespace hl_steer_target
#endif
