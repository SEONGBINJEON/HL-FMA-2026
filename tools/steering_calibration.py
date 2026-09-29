#!/usr/bin/env python3
"""Steering calibration record: load, validate, convert. No hardware access.

The record format is config/steering_calibration.json (null = unmeasured).
validate() returns (config, reasons). config is None unless EVERY rule holds;
the rules mirror hl_steer_target::configValid() in
firmware/steer_target/steer_target_core.cpp (including C++ storage types:
ADC fields uint16 and <= 1023, millisecond fields uint32, angles int16) and add
record-level evidence rules, so READY always yields a Config the core accepts.

Synthetic records (``"synthetic": true``) are refused for operation unless
allow_synthetic=True is passed explicitly (tests only).

CLI:
  python3 tools/steering_calibration.py check [PATH] [--allow-synthetic]
  python3 tools/steering_calibration.py emit-cpp PATH [--allow-synthetic]
Exit code 0 only when the record is ready for use.
"""
import argparse, json, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PATH = ROOT / "config" / "steering_calibration.json"
ADC_MAX = 1023
ADC_RAIL_LOW, ADC_RAIL_HIGH = 1, 1022
MAX_POINTS = 8
POLICIES = {"reject": "LIMIT_REJECT", "clamp": "LIMIT_CLAMP"}
CONTROL_KEYS = ("stop_band_adc", "restart_band_adc", "sensor_margin_adc",
                "command_stale_ms", "sensor_stale_ms", "reversal_neutral_ms")
# Storage type of each hl_steer_target::Config field plus semantic ADC bound.
# ADC-count fields: uint16 in C++, semantically <= ADC_MAX (mirrors configValid).
# Millisecond fields: uint32 in C++.
CONTROL_RANGES = {
    "stop_band_adc": (0, ADC_MAX), "restart_band_adc": (0, ADC_MAX), "sensor_margin_adc": (0, ADC_MAX),
    "command_stale_ms": (0, 0xFFFFFFFF), "sensor_stale_ms": (0, 0xFFFFFFFF),
    "reversal_neutral_ms": (0, 0xFFFFFFFF),
}


def load(path=DEFAULT_PATH):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _int(value):
    return type(value) is int


def _interpolate(x, x0, x1, y0, y1):
    """Integer interpolation identical to the C++ core (round half away from zero)."""
    num = (x - x0) * (y1 - y0)
    den = x1 - x0
    q = abs(num) // abs(den) * (1 if (num < 0) == (den < 0) else -1)
    rem = num - q * den
    if rem != 0 and abs(rem * 2) >= abs(den):
        q += 1 if (num < 0) == (den < 0) else -1
    return y0 + q


def angle_to_adc(points, angle_cdeg):
    """points: list of (angle_cdeg, adc) sorted by adc. None when outside the table."""
    for (a0, v0), (a1, v1) in zip(points, points[1:]):
        lo, hi = min(a0, a1), max(a0, a1)
        if lo <= angle_cdeg <= hi:
            v = _interpolate(angle_cdeg, a0, a1, v0, v1)
            return v if 0 <= v <= ADC_MAX else None
    return None


def adc_to_angle(points, adc):
    for (a0, v0), (a1, v1) in zip(points, points[1:]):
        if v0 <= adc <= v1:
            return _interpolate(adc, v0, v1, a0, a1)
    return None


def validate(record, allow_synthetic=False):
    """Return (config dict or None, list of reasons). Empty reasons <=> config present."""
    reasons = []
    if not isinstance(record, dict):
        return None, ["record is not an object"]
    if record.get("schema_version") != 1:
        reasons.append("schema_version must be 1")
    if record.get("status") != "measured":
        reasons.append(f"status is {record.get('status')!r}, not 'measured'")
    if record.get("synthetic") is True and not allow_synthetic:
        reasons.append("synthetic record refused for operation")
    if record.get("synthetic") not in (True, False):
        reasons.append("synthetic flag must be true/false")
    meas = record.get("measurement") or {}
    for key in ("measured_kst", "measured_by", "method"):
        if not isinstance(meas.get(key), str) or not meas.get(key).strip():
            reasons.append(f"measurement.{key} missing")
    if not isinstance(meas.get("evidence_paths"), list) or not meas.get("evidence_paths"):
        reasons.append("measurement.evidence_paths empty")
    sensor = record.get("sensor") or {}
    sign = sensor.get("steer_command_sign_for_adc_increase")
    if sign not in (-1, 1) or not _int(sign):
        reasons.append("sensor.steer_command_sign_for_adc_increase must be -1 or +1")
    if sensor.get("saturation_observed") is not False:
        reasons.append("sensor.saturation_observed must be explicitly false (true = sensor hits rail inside usable range)")
    raw_points = record.get("points")
    points = []
    if not isinstance(raw_points, list):
        reasons.append("points missing")
    else:
        for i, p in enumerate(raw_points):
            a, v = (p or {}).get("angle_cdeg"), (p or {}).get("adc")
            if not (_int(a) and _int(v)):
                reasons.append(f"points[{i}] ({(p or {}).get('label')}) unmeasured")
                continue
            if not (ADC_RAIL_LOW < v < ADC_RAIL_HIGH):
                reasons.append(f"points[{i}] adc {v} at/beyond electrical rail")
            if not (-32768 <= a <= 32767):
                reasons.append(f"points[{i}] angle out of int16")
            points.append((a, v))
    if len(points) < 3:
        reasons.append("fewer than 3 measured points (need straight + at least one per side)")
    if len(points) > MAX_POINTS:
        reasons.append(f"more than {MAX_POINTS} points")
    points.sort(key=lambda t: t[1])
    if len(points) >= 2:
        adcs = [v for _, v in points]
        angs = [a for a, _ in points]
        if any(b <= a for a, b in zip(adcs, adcs[1:])):
            reasons.append("adc values not strictly increasing (duplicate adc for two angles)")
        diffs = [b - a for a, b in zip(angs, angs[1:])]
        if any(d == 0 for d in diffs) or not (all(d > 0 for d in diffs) or all(d < 0 for d in diffs)):
            reasons.append("angle not strictly monotonic along adc")
    if points and all(a != 0 for a, _ in points):
        reasons.append("no straight (angle 0) point")
    limits = record.get("usable_limits") or {}
    lo, hi = limits.get("adc_min"), limits.get("adc_max")
    if not (_int(lo) and _int(hi)):
        reasons.append("usable_limits.adc_min/adc_max unmeasured")
    elif points and not reasons:
        straight = angle_to_adc(points, 0)
        if lo >= hi:
            reasons.append("usable_limits.adc_min must be < adc_max")
        if lo < points[0][1] or hi > points[-1][1]:
            reasons.append("usable limits outside measured table")
        if straight is None or not (lo < straight < hi):
            reasons.append("usable limits must lie on both sides of the straight adc")
    if not isinstance(limits.get("basis"), str) or not limits.get("basis", "").strip():
        reasons.append("usable_limits.basis missing")
    control = record.get("control") or {}
    for key in CONTROL_KEYS:
        value = control.get(key)
        lo_r, hi_r = CONTROL_RANGES[key]
        if not _int(value):
            reasons.append(f"control.{key} unset")
        elif not (lo_r <= value <= hi_r):
            reasons.append(f"control.{key}={value} outside storage/semantic range {lo_r}..{hi_r}")
    if all(_int(control.get(k)) and CONTROL_RANGES[k][0] <= control[k] <= CONTROL_RANGES[k][1]
           for k in CONTROL_KEYS):
        if control["stop_band_adc"] == 0:
            reasons.append("control.stop_band_adc must be > 0")
        if control["restart_band_adc"] < control["stop_band_adc"]:
            reasons.append("control.restart_band_adc must be >= stop_band_adc")
        if control["command_stale_ms"] == 0 or control["sensor_stale_ms"] == 0:
            reasons.append("control stale timeouts must be > 0")
    if control.get("limit_policy") not in POLICIES:
        reasons.append("control.limit_policy must be 'reject' or 'clamp'")
    if control.get("confirmed") is not True:
        reasons.append("control.confirmed is not true (parameters unconfirmed)")
    if not isinstance(control.get("basis"), str) or not control.get("basis", "").strip():
        reasons.append("control.basis missing")
    if reasons:
        return None, reasons
    config = {
        "calibrated": True,
        "points": points,
        "usable_adc_min": lo, "usable_adc_max": hi,
        "steer_sign_for_adc_increase": sign,
        **{k: control[k] for k in CONTROL_KEYS},
        "limit_policy": POLICIES[control["limit_policy"]],
        "synthetic": record.get("synthetic") is True,
        "measured_kst": meas["measured_kst"],
    }
    return config, []


def emit_cpp(config):
    """C++ snippet building an hl_steer_target::Config from a validated record."""
    lines = ["// Generated by tools/steering_calibration.py from a validated record",
             f"// measured_kst: {config['measured_kst']}  synthetic: {str(config['synthetic']).lower()}",
             "hl_steer_target::Config cfg;", "cfg.calibrated = true;"]
    for i, (a, v) in enumerate(config["points"]):
        lines.append(f"cfg.points[{i}].angle_cdeg = {a}; cfg.points[{i}].adc = {v};")
    lines.append(f"cfg.point_count = {len(config['points'])};")
    for key in ("usable_adc_min", "usable_adc_max", "steer_sign_for_adc_increase", *CONTROL_KEYS):
        lines.append(f"cfg.{key} = {config[key]};")
    lines.append(f"cfg.limit_policy = hl_steer_target::{config['limit_policy']};")
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["check", "emit-cpp"])
    ap.add_argument("path", nargs="?", default=str(DEFAULT_PATH))
    ap.add_argument("--allow-synthetic", action="store_true")
    args = ap.parse_args(argv)
    try:
        record = load(args.path)
    except (OSError, ValueError) as exc:
        print(f"NOT READY: cannot load {args.path}: {exc}")
        return 2
    config, reasons = validate(record, allow_synthetic=args.allow_synthetic)
    if config is None:
        print(f"NOT READY: {args.path}")
        for r in reasons:
            print(f"  - {r}")
        return 1
    if args.command == "emit-cpp":
        sys.stdout.write(emit_cpp(config))
    else:
        tag = " (SYNTHETIC, tests only)" if config["synthetic"] else ""
        print(f"READY{tag}: {args.path}; {len(config['points'])} points, usable adc "
              f"{config['usable_adc_min']}..{config['usable_adc_max']}, policy {config['limit_policy']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
