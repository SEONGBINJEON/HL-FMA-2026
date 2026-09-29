// Scenario tests for the CANDIDATE target steering core. All numbers here are
// SYNTHETIC test values (see tests/fixtures/steering_calibration_synthetic.json)
// and are not vehicle calibration.
#include "../steer_target_core.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
using namespace hl_steer_target;

static Config synthetic(LimitPolicy policy = LIMIT_REJECT) {
  Config c;
  c.calibrated = true;
  const CalPoint pts[5] = {{-1800, 300}, {-900, 470}, {0, 600}, {800, 720}, {1600, 830}};
  for (uint8_t i = 0; i < 5; ++i) c.points[i] = pts[i];
  c.point_count = 5;
  c.usable_adc_min = 320;
  c.usable_adc_max = 820;
  c.steer_sign_for_adc_increase = -1;  // synthetic: steer -1 raises ADC (same as rail guard assumption)
  c.stop_band_adc = 4;
  c.restart_band_adc = 8;
  c.sensor_margin_adc = 12;
  c.command_stale_ms = 300;
  c.sensor_stale_ms = 200;
  c.reversal_neutral_ms = 60;
  c.limit_policy = policy;
  return c;
}

// Same synthetic geometry with the sensor wired the other way: angle DECREASES as
// ADC increases (left = low ADC). Exercises the negative-denominator branch.
static Config syntheticDecreasing(LimitPolicy policy = LIMIT_REJECT) {
  Config c = synthetic(policy);
  for (uint8_t i = 0; i < c.point_count; ++i) c.points[i].angle_cdeg = static_cast<int16_t>(-c.points[i].angle_cdeg);
  c.steer_sign_for_adc_increase = 1;
  return c;
}

// Plant + trace. Each tick: (host refresh), measurement, update, then ADC moves if steer != 0.
// A real host must keep sending fresh sequences within command_stale_ms; the
// harness does that every 200ms unless a scenario turns auto_refresh off.
struct Harness {
  Core core; uint16_t adc; uint32_t now; uint32_t seq; int rate; int8_t sign_up;
  int8_t trace[4096]; size_t trace_len; bool sensor_on;
  bool auto_refresh; bool have_angle; int16_t angle; uint32_t last_send;
  explicit Harness(const Config &c, uint32_t start = 0, uint16_t adc0 = 600)
      : core(c), adc(adc0), now(start), seq(0), rate(3), sign_up(c.steer_sign_for_adc_increase),
        trace_len(0), sensor_on(true), auto_refresh(true), have_angle(false), angle(0), last_send(start) {}
  Output tick(uint32_t dt = 10) {
    now += dt;
    if (auto_refresh && have_angle && static_cast<uint32_t>(now - last_send) >= 200) target(angle);
    if (sensor_on) core.setMeasurement(adc, now);
    const Output o = core.update(now);
    if (trace_len < sizeof(trace)) trace[trace_len++] = o.steer;
    if (o.steer != 0) {
      // Plant follows the config's measured sign (C1.1: was hard-coded to the increasing table).
      const int delta = (o.steer == sign_up ? 1 : -1) * rate;
      const int next = static_cast<int>(adc) + delta;
      adc = static_cast<uint16_t>(next < 0 ? 0 : (next > 1023 ? 1023 : next));
    }
    return o;
  }
  bool target(int16_t angle_cdeg, uint32_t host_time = 0) {
    TargetCommand t; t.sequence = ++seq; t.angle_cdeg = angle_cdeg; t.host_time_ms = host_time;
    const bool ok = core.setTarget(t, now);
    if (ok) { have_angle = true; angle = angle_cdeg; last_send = now; }
    return ok;
  }
  bool resend(int16_t angle) {  // same sequence again
    TargetCommand t; t.sequence = seq; t.angle_cdeg = angle; t.host_time_ms = 0;
    return core.setTarget(t, now);
  }
  Output runUntilReason(Reason r, size_t max_ticks) {
    Output o = tick();
    for (size_t i = 0; i < max_ticks && o.reason != r; ++i) o = tick();
    assert(o.reason == r);
    return o;
  }
  void assertNoDirectReversal() const {
    int8_t prev = 0;
    for (size_t i = 0; i < trace_len; ++i) {
      if (trace[i] != 0 && prev != 0) assert(trace[i] == prev);
      prev = trace[i];
    }
  }
  void assertAllZero() const { for (size_t i = 0; i < trace_len; ++i) assert(trace[i] == 0); }
};

static void default_config_never_outputs() {
  Config none;  // boot-time default: nothing measured
  assert(!configValid(none));
  Harness h(none);
  h.core.setReady(true, 0);
  assert(!h.target(0));
  for (int i = 0; i < 50; ++i) { const Output o = h.tick(); assert(o.steer == 0 && o.reason == NO_CALIBRATION); }
  h.assertAllZero();
}

static void not_ready_and_no_target_are_zero() {
  Harness h(synthetic(), 0, 450);          // far from straight on purpose
  assert(!h.target(0));                    // target before ready is ignored
  for (int i = 0; i < 20; ++i) { const Output o = h.tick(); assert(o.steer == 0 && o.reason == NOT_READY); }
  h.core.setReady(true, h.now);
  for (int i = 0; i < 20; ++i) { const Output o = h.tick(); assert(o.steer == 0 && o.reason == NO_TARGET); }
  assert(h.adc == 450);                    // no drift toward an assumed centre
  h.assertAllZero();
}

static void broken_calibrations_are_rejected() {
  Config c = synthetic(); assert(configValid(c));
  { Config b = c; b.calibrated = false; assert(!configValid(b)); }
  { Config b = c; b.point_count = 2; assert(!configValid(b)); }
  { Config b = c; b.points[2].angle_cdeg = 10; assert(!configValid(b)); }        // no straight point
  { Config b = c; b.points[3].adc = 460; assert(!configValid(b)); }               // adc not increasing
  { Config b = c; b.points[3].angle_cdeg = -100; assert(!configValid(b)); }       // angle not monotonic
  { Config b = c; b.points[0].adc = 1; assert(!configValid(b)); }                 // at electrical rail
  { Config b = c; b.points[4].adc = 1022; assert(!configValid(b)); }
  { Config b = c; b.usable_adc_max = 900; assert(!configValid(b)); }              // outside table
  { Config b = c; b.usable_adc_min = 650; assert(!configValid(b)); }              // same side as straight
  { Config b = c; b.usable_adc_min = 820; b.usable_adc_max = 320; assert(!configValid(b)); }
  { Config b = c; b.steer_sign_for_adc_increase = 0; assert(!configValid(b)); }
  { Config b = c; b.steer_sign_for_adc_increase = 2; assert(!configValid(b)); }
  { Config b = c; b.stop_band_adc = 0; assert(!configValid(b)); }
  { Config b = c; b.restart_band_adc = 3; assert(!configValid(b)); }              // restart < stop
  { Config b = c; b.command_stale_ms = 0; assert(!configValid(b)); }
  { Config b = c; b.sensor_stale_ms = 0; assert(!configValid(b)); }
  { Config b = c; b.limit_policy = LIMIT_UNSET; assert(!configValid(b)); }
  // C1.1: ADC-count fields are bounded by the 10-bit range (shared with the Python validator).
  { Config b = c; b.stop_band_adc = 1024; b.restart_band_adc = 1024; assert(!configValid(b)); }
  { Config b = c; b.restart_band_adc = 1024; assert(!configValid(b)); }
  { Config b = c; b.sensor_margin_adc = 1024; assert(!configValid(b)); }
  { Config b = c; b.stop_band_adc = 1023; b.restart_band_adc = 1023; b.sensor_margin_adc = 1023; assert(configValid(b)); }
  assert(configValid(syntheticDecreasing()));
  // A broken config inside a core: ready+target+sensor still yields 0.
  Config b = c; b.limit_policy = LIMIT_UNSET;
  Harness h(b); h.core.setReady(true, 0); assert(!h.target(400));
  for (int i = 0; i < 10; ++i) assert(h.tick().reason == NO_CALIBRATION);
  h.assertAllZero();
}

static void table_lookup_vectors() {
  // Shared with tests/test_steering_calibration.py; keep both in sync.
  const Config c = synthetic();
  uint16_t adc = 0; int16_t ang = 0;
  assert(angleToAdc(c, 0, adc) && adc == 600);
  assert(angleToAdc(c, 400, adc) && adc == 660);
  assert(angleToAdc(c, -450, adc) && adc == 535);
  assert(angleToAdc(c, 1500, adc) && adc == 816);
  assert(angleToAdc(c, -1750, adc) && adc == 309);
  assert(angleToAdc(c, 1600, adc) && adc == 830);
  assert(!angleToAdc(c, 1601, adc));
  assert(!angleToAdc(c, -1801, adc));
  assert(adcToAngle(c, 660, ang) && ang == 400);
  assert(adcToAngle(c, 535, ang) && ang == -450);
  assert(adcToAngle(c, 700, ang) && ang == 667);
  assert(!adcToAngle(c, 299, ang));
  assert(adcRisesWithLeft(c));
  // Half-way cases (round half away from zero on the bracket quotient). Shared with Python.
  assert(angleToAdc(c, -45, adc) && adc == 594);
  assert(angleToAdc(c, -10, adc) && adc == 599);
  assert(angleToAdc(c, 10, adc) && adc == 602);
  assert(angleToAdc(c, 45, adc) && adc == 607);
  assert(adcToAngle(c, 601, ang) && ang == 7);
}

static void table_lookup_vectors_decreasing() {
  // C1.1 regression: negative denominator (angle decreasing with ADC). Codex probe:
  // angle 899 must give 470 (470.144...), previously 471. Shared with Python test.
  const Config c = syntheticDecreasing();
  assert(!adcRisesWithLeft(c));
  uint16_t adc = 0; int16_t ang = 0;
  assert(angleToAdc(c, 899, adc) && adc == 470);
  assert(angleToAdc(c, 450, adc) && adc == 535);
  assert(angleToAdc(c, -400, adc) && adc == 660);
  assert(angleToAdc(c, 10, adc) && adc == 599);
  assert(angleToAdc(c, -10, adc) && adc == 602);
  assert(angleToAdc(c, 45, adc) && adc == 594);
  assert(angleToAdc(c, 900, adc) && adc == 470);     // exact measured points
  assert(angleToAdc(c, -1600, adc) && adc == 830);
  assert(angleToAdc(c, 1800, adc) && adc == 300);
  assert(angleToAdc(c, 0, adc) && adc == 600);
  assert(!angleToAdc(c, 1801, adc) && !angleToAdc(c, -1601, adc));
  assert(adcToAngle(c, 470, ang) && ang == 900);
  assert(adcToAngle(c, 535, ang) && ang == 450);
  assert(adcToAngle(c, 601, ang) && ang == -7);
  assert(adcToAngle(c, 300, ang) && ang == 1800);
  assert(adcToAngle(c, 830, ang) && ang == -1600);
  // Control still works with the reversed sensor: left target (+) => lower ADC => steer sign.
  Harness h(c); h.core.setReady(true, 0); assert(h.target(450));   // adc 535 from 600
  Output o = h.tick(); assert(o.reason == MOVING && o.target_adc == 535 && o.steer == -1);
  h.runUntilReason(AT_TARGET, 100); assert(h.adc >= 531 && h.adc <= 539);
  // CLAMP picks the correct side: far-left request clamps to usable_adc_min (left = low ADC).
  Harness k(syntheticDecreasing(LIMIT_CLAMP)); k.core.setReady(true, 0); assert(k.target(2500));
  o = k.tick(); assert(o.reason == MOVING && o.target_adc == 320 && o.target_clamped && o.steer == -1);
  assert(k.target(-2500)); o = k.tick(); assert(o.reason == REVERSAL_NEUTRAL);
  o = k.runUntilReason(MOVING, 20); assert(o.target_adc == 820 && o.target_clamped && o.steer == 1);
  k.assertNoDirectReversal();
}

static void reaches_target_and_holds_with_hysteresis() {
  Harness h(synthetic()); h.core.setReady(true, 0);
  assert(h.target(400));                       // adc 660 from 600
  Output o = h.tick();
  assert(o.reason == MOVING && o.steer == -1 && o.target_adc == 660 && !o.target_clamped);
  o = h.runUntilReason(AT_TARGET, 100);
  assert(o.steer == 0);
  assert(h.adc >= 656 && h.adc <= 664);
  const uint16_t settled = h.adc;
  for (int i = 0; i < 10; ++i) { o = h.tick(); assert(o.steer == 0 && o.reason == AT_TARGET); }
  assert(h.adc == settled);                    // motor really stopped
  // Small disturbance inside restart band: still hold.
  h.adc = 660 + 7; o = h.tick(); assert(o.steer == 0 && o.reason == AT_TARGET);
  // Disturbance beyond restart band: correct back (adc too high => steer +1 lowers it).
  h.adc = 660 + 9; o = h.tick(); assert(o.reason == MOVING && o.steer == 1);
  h.runUntilReason(AT_TARGET, 50);
  h.assertNoDirectReversal();
}

static void left_right_limits_reject_and_clamp() {
  // Right target (negative angle) => adc 535 below 600 => steer +1 with synthetic sign.
  { Harness h(synthetic()); h.core.setReady(true, 0); assert(h.target(-450));
    Output o = h.tick(); assert(o.reason == MOVING && o.steer == 1 && o.target_adc == 535);
    o = h.runUntilReason(AT_TARGET, 100); assert(h.adc >= 531 && h.adc <= 539); }
  // REJECT: inside table but beyond usable max (1600 -> 830 > 820) latches TARGET_REJECTED.
  { Harness h(synthetic(LIMIT_REJECT)); h.core.setReady(true, 0); assert(!h.target(1600));
    for (int i = 0; i < 5; ++i) { Output o = h.tick(); assert(o.steer == 0 && o.reason == TARGET_REJECTED && o.fault_latched); }
    assert(!h.target(-2500));                  // outside table: also rejected
    assert(h.tick().reason == TARGET_REJECTED);
    assert(h.target(400));                     // a new valid sequence resumes
    assert(h.tick().reason == MOVING);
    h.assertNoDirectReversal(); }
  // CLAMP: same requests move to the usable limit and report clamped.
  { Harness h(synthetic(LIMIT_CLAMP)); h.core.setReady(true, 0); assert(h.target(1600));
    Output o = h.tick(); assert(o.reason == MOVING && o.steer == -1 && o.target_adc == 820 && o.target_clamped);
    o = h.runUntilReason(AT_TARGET, 200); assert(h.adc >= 816 && h.adc <= 824);
    assert(h.target(-2500));                   // outside table on the right => clamp to 320
    o = h.tick(); assert(o.steer == 0 && o.reason == REVERSAL_NEUTRAL);
    o = h.runUntilReason(MOVING, 20); assert(o.steer == 1 && o.target_adc == 320 && o.target_clamped);
    h.runUntilReason(AT_TARGET, 400); assert(h.adc >= 316 && h.adc <= 324);
    h.assertNoDirectReversal(); }
  // Never push outward when already at/over the usable limit (defensive guard).
  { Config c = synthetic(LIMIT_CLAMP); Harness h(c, 0, 825); h.core.setReady(true, 0);
    assert(h.target(1600));                    // target_adc 820; adc 825 inside margin
    Output o = h.tick(); assert(o.steer == 0 && o.reason == AT_TARGET);  // |err|=5 < restart 8
    h.adc = 830; o = h.tick(); assert(o.reason == MOVING && o.steer == 1);  // inward only
  }
}

static void reversal_requires_neutral_first() {
  Harness h(synthetic()); h.core.setReady(true, 0);
  assert(h.target(800));                       // adc 720, moving -1
  for (int i = 0; i < 5; ++i) assert(h.tick().steer == -1);
  assert(h.target(-450));                      // reverse to adc 535
  Output o = h.tick(); assert(o.steer == 0 && o.reason == REVERSAL_NEUTRAL);
  // Neutral must persist >= 60ms with 10ms ticks: ticks at +10..+60 => zeros through 6 ticks.
  for (int i = 0; i < 5; ++i) { o = h.tick(); assert(o.steer == 0 && o.reason == REVERSAL_NEUTRAL); }
  o = h.tick(); assert(o.steer == 1 && o.reason == MOVING);
  h.runUntilReason(AT_TARGET, 200);
  h.assertNoDirectReversal();
  // Overshoot correction also passes through neutral.
  Harness fast(synthetic()); fast.core.setReady(true, 0); fast.rate = 7; assert(fast.target(400));
  fast.runUntilReason(AT_TARGET, 300); fast.assertNoDirectReversal();
}

static void stale_command_latches_and_resend_does_not_renew() {
  Harness h(synthetic()); h.auto_refresh = false; h.core.setReady(true, 0);
  assert(h.target(400));                       // accepted at now=0
  for (int i = 0; i < 20; ++i) h.tick();       // now=200, still fresh
  assert(!h.resend(400));                      // duplicate sequence ignored
  for (int i = 0; i < 9; ++i) assert(h.tick().reason != COMMAND_STALE);  // now=290
  Output o = h.tick();                         // now=300 >= 300 stale
  assert(o.steer == 0 && o.reason == COMMAND_STALE && o.fault_latched);
  assert(!h.resend(400));                      // stale retransmission cannot resume
  for (int i = 0; i < 10; ++i) { o = h.tick(); assert(o.steer == 0 && o.reason == COMMAND_STALE); }
  assert(h.target(400));                       // new sequence resumes
  o = h.tick(); assert(o.reason == MOVING || o.reason == AT_TARGET);
  // A host clock is never compared: absurd host_time still accepted with a new sequence.
  assert(h.target(400, 0xFFFFFFFFu));
  assert(h.core.lastHostTime() == 0xFFFFFFFFu);
}

static void stale_or_invalid_sensor_latches() {
  { Harness h(synthetic()); h.auto_refresh = false; h.core.setReady(true, 0); assert(h.target(400));
    assert(h.tick().reason == MOVING);
    h.sensor_on = false;                       // measurements stop
    Output o = h.tick(100); assert(o.reason == MOVING);  // 100ms old < 200
    o = h.tick(100);                           // 200ms old => stale
    assert(o.steer == 0 && o.reason == SENSOR_STALE && o.fault_latched);
    h.sensor_on = true;
    for (int i = 0; i < 5; ++i) { o = h.tick(); assert(o.steer == 0 && o.reason == SENSOR_STALE); }
    assert(h.target(400)); assert(h.tick().reason != SENSOR_STALE); }
  // No measurement at all yet: stale, never move.
  { Harness h(synthetic()); h.core.setReady(true, 0); assert(h.target(400)); h.sensor_on = false;
    Output o = h.tick(); assert(o.steer == 0 && o.reason == SENSOR_STALE); }
  // Electrical rails and out-of-margin readings.
  const uint16_t bad[] = {0, 1, 1022, 1023, 307, 833};
  for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
    Harness h(synthetic()); h.core.setReady(true, 0); assert(h.target(400));
    h.adc = bad[i]; Output o = h.tick();
    assert(o.steer == 0 && o.reason == SENSOR_INVALID && o.fault_latched);
    h.adc = 600; o = h.tick(); assert(o.steer == 0 && o.reason == SENSOR_INVALID);  // stays latched
  }
  // Just inside the margin is accepted.
  { Harness h(synthetic(), 0, 308); h.core.setReady(true, 0); assert(h.target(-450));
    Output o = h.tick(); assert(o.reason == MOVING && o.steer == -1); }
}

static void stop_and_ready_drop_never_resume_on_their_own() {
  { Harness h(synthetic()); h.auto_refresh = false; h.core.setReady(true, 0); assert(h.target(800));
    for (int i = 0; i < 3; ++i) assert(h.tick().steer == -1);
    h.core.stop(h.now);
    Output o = h.tick(); assert(o.steer == 0 && o.reason == STOPPED && o.fault_latched && o.target_adc == 0);
    assert(!h.resend(800));
    for (int i = 0; i < 30; ++i) { o = h.tick(); assert(o.steer == 0 && o.reason == STOPPED); }
    h.core.setReady(false, h.now); h.core.setReady(true, h.now);
    for (int i = 0; i < 5; ++i) { o = h.tick(); assert(o.steer == 0); }
    const uint16_t held = h.adc;
    assert(h.target(800)); o = h.tick(); assert(o.reason == MOVING && o.steer == -1);
    assert(h.adc != held); }
  { Harness h(synthetic()); h.auto_refresh = false; h.core.setReady(true, 0); assert(h.target(800));
    for (int i = 0; i < 3; ++i) assert(h.tick().steer == -1);
    h.core.setReady(false, h.now);
    Output o = h.tick(); assert(o.steer == 0 && o.reason == NOT_READY && o.fault_latched);  // latched on the falling edge
    h.core.setReady(true, h.now);
    for (int i = 0; i < 10; ++i) { o = h.tick(); assert(o.steer == 0 && o.reason == NOT_READY && o.fault_latched); }
    assert(!h.resend(800));
    assert(h.tick().steer == 0);
    assert(h.target(800)); assert(h.tick().reason == MOVING); }
}

static void millis_wrap_is_handled() {
  const uint32_t base = UINT32_MAX - 150;
  Harness h(synthetic(), base); h.auto_refresh = false; h.core.setReady(true, base);
  assert(h.target(400));
  Output o = h.tick(); assert(o.reason == MOVING);
  for (int i = 0; i < 16; ++i) { o = h.tick(); assert(o.reason == MOVING || o.reason == AT_TARGET); }
  assert(h.now < base);                        // wrapped
  for (int i = 0; i < 12; ++i) o = h.tick();   // now-target_at = 290
  assert(o.reason != COMMAND_STALE);
  o = h.tick(); assert(o.reason == COMMAND_STALE);
}

int main() {
  default_config_never_outputs();
  not_ready_and_no_target_are_zero();
  broken_calibrations_are_rejected();
  table_lookup_vectors();
  table_lookup_vectors_decreasing();
  reaches_target_and_holds_with_hysteresis();
  left_right_limits_reject_and_clamp();
  reversal_requires_neutral_first();
  stale_command_latches_and_resend_does_not_renew();
  stale_or_invalid_sensor_latches();
  stop_and_ready_drop_never_resume_on_their_own();
  millis_wrap_is_handled();
  puts("PASS: 12 groups; synthetic table, no hardware. boot/no-calibration/not-ready zero, broken "
       "calibration rejected (incl. ADC bounds), table vectors incl. decreasing table/half cases, reach+hysteresis, L/R reject+clamp+limit guard, "
       "neutral-before-reversal, stale command+no renewal, stale/invalid sensor, stop/ready-drop "
       "no self-resume, millis wrap.");
  return 0;
}
