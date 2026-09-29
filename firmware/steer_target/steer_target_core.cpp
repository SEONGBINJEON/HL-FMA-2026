#include "steer_target_core.h"

namespace hl_steer_target {

Config::Config()
    : calibrated(false), point_count(0), usable_adc_min(0), usable_adc_max(0),
      steer_sign_for_adc_increase(0), stop_band_adc(0), restart_band_adc(0),
      sensor_margin_adc(0), command_stale_ms(0), sensor_stale_ms(0),
      reversal_neutral_ms(0), limit_policy(LIMIT_UNSET) {
  for (uint8_t i = 0; i < MAX_POINTS; ++i) { points[i].angle_cdeg = 0; points[i].adc = 0; }
}

bool configValid(const Config &c) {
  if (!c.calibrated) return false;
  if (c.point_count < 3 || c.point_count > MAX_POINTS) return false;
  bool has_zero = false;
  int8_t angle_dir = 0;
  for (uint8_t i = 0; i < c.point_count; ++i) {
    const CalPoint &p = c.points[i];
    if (p.adc <= ADC_RAIL_LOW || p.adc >= ADC_RAIL_HIGH) return false;
    if (p.angle_cdeg == 0) has_zero = true;
    if (i == 0) continue;
    const CalPoint &q = c.points[i - 1];
    if (p.adc <= q.adc) return false;                   // strictly increasing adc
    const int8_t d = p.angle_cdeg > q.angle_cdeg ? 1 : (p.angle_cdeg < q.angle_cdeg ? -1 : 0);
    if (d == 0) return false;                           // strictly monotonic angle
    if (angle_dir == 0) angle_dir = d; else if (d != angle_dir) return false;
  }
  if (!has_zero) return false;
  // Usable limits inside the table and on opposite sides of straight.
  uint16_t straight = 0;
  if (!angleToAdc(c, 0, straight)) return false;
  if (c.usable_adc_min >= c.usable_adc_max) return false;
  if (c.usable_adc_min < c.points[0].adc || c.usable_adc_max > c.points[c.point_count - 1].adc)
    return false;
  if (!(c.usable_adc_min < straight && straight < c.usable_adc_max)) return false;
  if (c.steer_sign_for_adc_increase != 1 && c.steer_sign_for_adc_increase != -1) return false;
  if (c.stop_band_adc == 0 || c.restart_band_adc < c.stop_band_adc) return false;
  // Semantic ADC bounds shared with tools/steering_calibration.py: bands and
  // margin are ADC counts and can never exceed the 10-bit range.
  if (c.stop_band_adc > ADC_MAX || c.restart_band_adc > ADC_MAX || c.sensor_margin_adc > ADC_MAX)
    return false;
  if (c.command_stale_ms == 0 || c.sensor_stale_ms == 0) return false;
  if (c.limit_policy != LIMIT_REJECT && c.limit_policy != LIMIT_CLAMP) return false;
  return true;
}

bool adcRisesWithLeft(const Config &c) {
  return c.point_count >= 2 && c.points[1].angle_cdeg > c.points[0].angle_cdeg;
}

static int32_t interpolate(int32_t x, int32_t x0, int32_t x1, int32_t y0, int32_t y1) {
  // Integer linear interpolation, round half away from zero; x1 != x0.
  // Works on magnitudes so a negative denominator (angle decreasing with ADC)
  // rounds the same way as a positive one. C1.1 fix: the previous form
  // rounded every non-zero remainder up when den < 0.
  const int32_t num = (x - x0) * (y1 - y0);
  const int32_t den = x1 - x0;
  const bool negative = (num < 0) != (den < 0);
  const int32_t anum = num < 0 ? -num : num;
  const int32_t aden = den < 0 ? -den : den;
  int32_t q = anum / aden;
  const int32_t rem = anum - q * aden;
  if (rem * 2 >= aden) ++q;
  return y0 + (negative ? -q : q);
}

bool angleToAdc(const Config &c, int16_t angle_cdeg, uint16_t &adc) {
  if (c.point_count < 2) return false;
  // Angle may be increasing or decreasing along the table; find the bracket.
  for (uint8_t i = 1; i < c.point_count; ++i) {
    const int32_t a0 = c.points[i - 1].angle_cdeg, a1 = c.points[i].angle_cdeg;
    const int32_t lo = a0 < a1 ? a0 : a1, hi = a0 < a1 ? a1 : a0;
    if (angle_cdeg < lo || angle_cdeg > hi) continue;
    const int32_t v = interpolate(angle_cdeg, a0, a1, c.points[i - 1].adc, c.points[i].adc);
    if (v < 0 || v > ADC_MAX) return false;
    adc = static_cast<uint16_t>(v);
    return true;
  }
  return false;
}

bool adcToAngle(const Config &c, uint16_t adc, int16_t &angle_cdeg) {
  if (c.point_count < 2) return false;
  for (uint8_t i = 1; i < c.point_count; ++i) {
    if (adc < c.points[i - 1].adc || adc > c.points[i].adc) continue;
    const int32_t v = interpolate(adc, c.points[i - 1].adc, c.points[i].adc,
                                  c.points[i - 1].angle_cdeg, c.points[i].angle_cdeg);
    if (v < -32768 || v > 32767) return false;
    angle_cdeg = static_cast<int16_t>(v);
    return true;
  }
  return false;
}

Core::Core(const Config &config)
    : config_(config), config_valid_(configValid(config)), ready_(false),
      latched_(false), latch_reason_(NOT_READY), has_target_(false),
      has_sequence_(false), last_sequence_(0), last_host_time_(0), target_adc_(0),
      target_clamped_(false), target_at_(0), has_measurement_(false), adc_(0),
      measured_at_(0), last_steer_(0), last_dir_(0), neutral_seen_(true),
      neutral_since_(0) {
  last_.steer = 0;
  last_.reason = config_valid_ ? NOT_READY : NO_CALIBRATION;
  last_.fault_latched = false;
  last_.target_clamped = false;
  last_.target_adc = 0;
}

void Core::latch(Reason reason) {
  latched_ = true;
  latch_reason_ = reason;
  has_target_ = false;
  target_adc_ = 0;
  target_clamped_ = false;
}

void Core::setReady(bool ready, uint32_t now) {
  (void)now;
  if (ready_ && !ready) latch(NOT_READY);
  ready_ = ready;
}

void Core::stop(uint32_t now) {
  (void)now;
  latch(STOPPED);
}

bool Core::setTarget(const TargetCommand &cmd, uint32_t now) {
  if (!config_valid_ || !ready_) return false;
  if (has_sequence_ && cmd.sequence <= last_sequence_) return false;  // duplicate/old: no renewal
  has_sequence_ = true;
  last_sequence_ = cmd.sequence;
  last_host_time_ = cmd.host_time_ms;
  uint16_t adc = 0;
  bool clamped = false;
  if (!angleToAdc(config_, cmd.angle_cdeg, adc)) {
    if (config_.limit_policy == LIMIT_REJECT) { latch(TARGET_REJECTED); return false; }
    // Outside the table: clamp to the usable limit on the requested side.
    const bool wants_left = cmd.angle_cdeg > 0;
    const bool left_is_high = adcRisesWithLeft(config_);
    adc = (wants_left == left_is_high) ? config_.usable_adc_max : config_.usable_adc_min;
    clamped = true;
  }
  if (adc < config_.usable_adc_min || adc > config_.usable_adc_max) {
    if (config_.limit_policy == LIMIT_REJECT) { latch(TARGET_REJECTED); return false; }
    adc = adc < config_.usable_adc_min ? config_.usable_adc_min : config_.usable_adc_max;
    clamped = true;
  }
  // A new sequence accepted while ready clears any latched fault.
  latched_ = false;
  has_target_ = true;
  target_adc_ = adc;
  target_clamped_ = clamped;
  target_at_ = now;
  return true;
}

void Core::setMeasurement(uint16_t adc, uint32_t now) {
  has_measurement_ = true;
  adc_ = adc;
  measured_at_ = now;
}

Output Core::emit(int8_t steer, Reason reason, uint32_t now) {
  if (steer == 0) {
    if (!neutral_seen_) { neutral_seen_ = true; neutral_since_ = now; }
  } else {
    if (steer != last_dir_) { last_dir_ = steer; }
    neutral_seen_ = false;
  }
  last_steer_ = steer;
  last_.steer = steer;
  last_.reason = reason;
  last_.fault_latched = latched_;
  last_.target_clamped = has_target_ && target_clamped_;
  last_.target_adc = has_target_ ? target_adc_ : 0;
  return last_;
}

Output Core::update(uint32_t now) {
  if (!config_valid_) return emit(0, NO_CALIBRATION, now);
  if (!ready_) return emit(0, NOT_READY, now);
  if (latched_) return emit(0, latch_reason_, now);
  if (!has_target_) return emit(0, NO_TARGET, now);
  if (static_cast<uint32_t>(now - target_at_) >= config_.command_stale_ms) {
    latch(COMMAND_STALE);
    return emit(0, COMMAND_STALE, now);
  }
  if (!has_measurement_ || static_cast<uint32_t>(now - measured_at_) >= config_.sensor_stale_ms) {
    latch(SENSOR_STALE);
    return emit(0, SENSOR_STALE, now);
  }
  const int32_t lo = static_cast<int32_t>(config_.usable_adc_min) - config_.sensor_margin_adc;
  const int32_t hi = static_cast<int32_t>(config_.usable_adc_max) + config_.sensor_margin_adc;
  if (adc_ > ADC_MAX || adc_ <= ADC_RAIL_LOW || adc_ >= ADC_RAIL_HIGH || adc_ < lo || adc_ > hi) {
    latch(SENSOR_INVALID);
    return emit(0, SENSOR_INVALID, now);
  }
  const int32_t error = static_cast<int32_t>(target_adc_) - adc_;
  const int32_t magnitude = error < 0 ? -error : error;
  const bool moving = last_steer_ != 0;
  if (moving ? magnitude <= config_.stop_band_adc : magnitude < config_.restart_band_adc)
    return emit(0, AT_TARGET, now);
  const int8_t adc_dir = error > 0 ? 1 : -1;
  // Never push outward past the measured usable range.
  if ((adc_dir > 0 && adc_ >= config_.usable_adc_max) || (adc_dir < 0 && adc_ <= config_.usable_adc_min))
    return emit(0, AT_LIMIT, now);
  const int8_t steer = static_cast<int8_t>(adc_dir * config_.steer_sign_for_adc_increase);
  if (last_dir_ != 0 && steer != last_dir_) {
    // Sign reversal: at least one zero output and reversal_neutral_ms of neutral.
    if (!neutral_seen_ || static_cast<uint32_t>(now - neutral_since_) < config_.reversal_neutral_ms)
      return emit(0, REVERSAL_NEUTRAL, now);
  }
  return emit(steer, MOVING, now);
}

}  // namespace hl_steer_target
