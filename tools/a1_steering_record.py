#!/usr/bin/env python3
"""A1 steering measurement recorder: links live type150 ADC logs to human wheel angles.

Works on a running or finished joystick_desktop/joystick_control session folder
(events.jsonl with "status" events). It never opens a serial port or moves the
vehicle. Angles are typed by the operator from the protractor (left +, right -).

  mark    --session DIR --label {straight,left_mid,left_limit,right_mid,right_limit,other}
          --wheel {left,right,both} --angle-deg A [--angle-deg-right B]
          [--window-s 3] [--mechanical-end] [--note TEXT]
          -> appends one record to DIR/a1_marks.jsonl using the board statuses of
             the last window seconds. Rejected when: no fresh status, steering output
             or request non-zero inside the window (wheel still moving), too few
             samples. Saturation (ADC<=1 or >=1022) is flagged automatically.
  sign    --session DIR   -> derives which steer command sign raised the ADC
                             (evidence counts; candidate only).
  summary --session DIR   -> table of marks + DIR/a1_calibration_draft.json.
                             The draft is NOT config/steering_calibration.json.
"""
import argparse, json, statistics, sys
from datetime import datetime, timezone, timedelta
from pathlib import Path

KST = timezone(timedelta(hours=9))
LABELS = ("straight", "left_mid", "left_limit", "right_mid", "right_limit", "other")
MIN_SAMPLES = 10
FRESH_S = 0.5
UNSTABLE_RANGE = 6


def parse_utc(s):
    return datetime.fromisoformat(s.replace("Z", "+00:00"))


def read_status_events(session: Path):
    """Yield (host_utc datetime, status dict, byte_offset) for each status event."""
    path = session / "events.jsonl"
    if not path.is_file():
        raise SystemExit(f"no events.jsonl in {session}")
    out = []
    with open(path, "rb") as f:
        offset = 0
        for raw in f:
            line = raw.decode("utf-8", "replace")
            if '"event": "status"' in line or '"event":"status"' in line:
                try:
                    ev = json.loads(line)
                    if ev.get("event") == "status" and isinstance(ev.get("status"), dict):
                        out.append((parse_utc(ev["host_utc"]), ev["status"], offset))
                except (ValueError, KeyError):
                    pass
            offset += len(raw)
    return out


def take_window(events, t_mark, window_s):
    lo = t_mark - timedelta(seconds=window_s)
    return [e for e in events if lo <= e[0] <= t_mark]


def evaluate_window(events, t_mark, window_s):
    """Return (record_fragment or None, rejection reason)."""
    if not events:
        return None, "no status events in session"
    latest = events[-1][0]
    age = (t_mark - latest).total_seconds()
    if age > FRESH_S:
        return None, f"stale: latest board status is {age:.1f}s old (> {FRESH_S}s); board not live"
    win = take_window(events, t_mark, window_s)
    if len(win) < MIN_SAMPLES:
        return None, f"only {len(win)} status samples in {window_s}s window (< {MIN_SAMPLES})"
    moving = [s for _, s, _ in win if s.get("actual_steer", 0) != 0 or s.get("requested_steer", 0) != 0]
    if moving:
        return None, f"steering output/request non-zero in {len(moving)} of {len(win)} samples: release the stick and wait"
    adcs = [int(s["steering_adc"]) for _, s, _ in win]
    boots = sorted({s.get("boot_id") for _, s, _ in win})
    frag = {
        "adc": {"n": len(adcs), "min": min(adcs), "max": max(adcs), "median": int(statistics.median(adcs)),
                "range": max(adcs) - min(adcs)},
        "adc_samples": adcs,
        "board_boot_ids": boots,
        "window_utc": [win[0][0].isoformat(), win[-1][0].isoformat()],
        "events_byte_range": [win[0][2], win[-1][2]],
        "saturation_electrical": min(adcs) <= 1 or max(adcs) >= 1022,
        "unstable": (max(adcs) - min(adcs)) > UNSTABLE_RANGE,
        "drive_output_seen": any(s.get("actual_drive", 0) != 0 for _, s, _ in win),
    }
    return frag, ""


def cmd_mark(a):
    session = Path(a.session)
    t_mark = parse_utc(a.now_utc) if a.now_utc else datetime.now(timezone.utc)
    events = read_status_events(session)
    frag, why = evaluate_window(events, t_mark, a.window_s)
    if frag is None:
        print(f"REJECTED: {why}")
        return 1
    if a.wheel == "both" and a.angle_deg_right is None:
        print("REJECTED: --wheel both needs --angle-deg (left wheel) and --angle-deg-right")
        return 1
    angles = {"left": a.angle_deg if a.wheel in ("left", "both") else None,
              "right": (a.angle_deg_right if a.wheel == "both" else a.angle_deg) if a.wheel in ("right", "both") else None}
    warnings = []
    if a.label == "straight" and any(v is not None and abs(v) > 0.5 for v in angles.values()):
        warnings.append("label straight but |angle| > 0.5deg: record as measured, do not force 0")
    if a.label.startswith("left") and any(v is not None and v < 0 for v in angles.values()):
        warnings.append("label left but angle negative: sign convention is left +")
    if a.label.startswith("right") and any(v is not None and v > 0 for v in angles.values()):
        warnings.append("label right but angle positive: sign convention is right -")
    if frag["unstable"]:
        warnings.append(f"ADC range {frag['adc']['range']} > {UNSTABLE_RANGE}: wheel or sensor not settled")
    if frag["saturation_electrical"]:
        warnings.append("electrical saturation seen (ADC<=1 or >=1022): not a calibration point")
    if frag["drive_output_seen"]:
        warnings.append("drive output non-zero in window: this is a steering-only measurement")
    rec = {
        "mark_utc": t_mark.isoformat(), "mark_kst": t_mark.astimezone(KST).isoformat(timespec="seconds"),
        "label": a.label, "wheel": a.wheel, "angle_deg": angles,
        "angle_cdeg": {k: (None if v is None else round(v * 100)) for k, v in angles.items()},
        "mechanical_end": bool(a.mechanical_end), "note": a.note, "warnings": warnings,
        "source": {"session": str(session), "events": "events.jsonl", "window_s": a.window_s},
        **frag,
    }
    with open(session / "a1_marks.jsonl", "a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    print(f"MARKED {a.label} wheel={a.wheel} angle={angles} ADC median {frag['adc']['median']} "
          f"(min {frag['adc']['min']} max {frag['adc']['max']} n {frag['adc']['n']}) boot {frag['board_boot_ids']}")
    for w in warnings:
        print(f"  WARNING: {w}")
    return 0


def derive_sign(events, min_run=5, min_delta=5):
    """Runs of constant non-zero actual_steer within one boot: does ADC rise or fall?"""
    runs = []
    cur = None
    for t, s, _ in events:
        st = s.get("actual_steer", 0)
        key = (s.get("boot_id"), st)
        if st != 0 and cur is not None and cur["key"] == key:
            cur["adcs"].append(int(s["steering_adc"]))
        else:
            if cur is not None and len(cur["adcs"]) >= min_run:
                runs.append(cur)
            cur = {"key": key, "adcs": [int(s["steering_adc"])]} if st != 0 else None
    if cur is not None and len(cur["adcs"]) >= min_run:
        runs.append(cur)
    tally = {}
    for r in runs:
        delta = r["adcs"][-1] - r["adcs"][0]
        if abs(delta) < min_delta:
            continue
        steer = r["key"][1]
        tally.setdefault(steer, {"adc_up": 0, "adc_down": 0})
        tally[steer]["adc_up" if delta > 0 else "adc_down"] += 1
    candidate = None
    votes = {}
    for steer, c in tally.items():
        total = c["adc_up"] + c["adc_down"]
        if total >= 3 and c["adc_up"] / total >= 0.9:
            votes[steer] = "raises"
        elif total >= 3 and c["adc_down"] / total >= 0.9:
            votes[steer] = "lowers"
    raisers = [s for s, v in votes.items() if v == "raises"]
    lowerers = [s for s, v in votes.items() if v == "lowers"]
    if len(raisers) == 1 and (not lowerers or lowerers == [-raisers[0]]):
        candidate = raisers[0]
    elif not raisers and len(lowerers) == 1:
        candidate = -lowerers[0]
    return {"runs_used": sum(sum(c.values()) for c in tally.values()), "tally_by_actual_steer": tally,
            "steer_command_sign_for_adc_increase_candidate": candidate,
            "basis": "derived from logged actual_steer runs and ADC deltas; confirm against wheel motion, not a measurement of angle"}


def cmd_sign(a):
    events = read_status_events(Path(a.session))
    print(json.dumps(derive_sign(events), ensure_ascii=False, indent=1))
    return 0


def cmd_summary(a):
    session = Path(a.session)
    marks_path = session / "a1_marks.jsonl"
    marks = [json.loads(l) for l in open(marks_path, encoding="utf-8")] if marks_path.is_file() else []
    if not marks:
        print(f"no marks in {marks_path}")
        return 1
    print(f"{'kst':20} {'label':12} {'wheel':6} {'angL':>7} {'angR':>7} {'adc_med':>7} {'min':>5} {'max':>5} {'n':>3} sat mech warn")
    for m in marks:
        print(f"{m['mark_kst'][11:19]:20} {m['label']:12} {m['wheel']:6} "
              f"{str(m['angle_deg']['left']):>7} {str(m['angle_deg']['right']):>7} {m['adc']['median']:>7} "
              f"{m['adc']['min']:>5} {m['adc']['max']:>5} {m['adc']['n']:>3} {int(m['saturation_electrical'])}   "
              f"{int(m['mechanical_end'])}    {len(m['warnings'])}")
    events = read_status_events(session)
    sign = derive_sign(events)
    by_wheel = {"left": [], "right": []}
    for m in marks:
        if m["saturation_electrical"]:
            continue  # never a calibration point
        for w in ("left", "right"):
            cd = m["angle_cdeg"][w]
            if cd is not None:
                by_wheel[w].append({"label": m["label"], "angle_cdeg": cd, "adc": m["adc"]["median"],
                                    "adc_range": m["adc"]["range"], "mark_kst": m["mark_kst"],
                                    "mechanical_end": m["mechanical_end"]})
    for w in by_wheel:
        by_wheel[w].sort(key=lambda p: p["adc"])
    draft = {
        "schema_version": 1, "status": "draft_from_marks", "synthetic": False, "applied_to_runtime": False,
        "purpose": "A1 draft assembled from a1_marks.jsonl. Review, choose the wheel/points, then copy into config/steering_calibration.json by hand. Not validated, not a runtime file.",
        "session": str(session), "marks": len(marks),
        "points_by_wheel": by_wheel,
        "sensor_sign_candidate": sign,
        "excluded_saturated_marks": [m["mark_kst"] for m in marks if m["saturation_electrical"]],
        "open_items": ["choose measured wheel for the table (no averaging of left/right)",
                       "usable_limits with basis; exclude saturation and forced mechanical ends",
                       "control.* values with basis and confirmed=true",
                       "measurement.* evidence paths: this session folder and a1_marks.jsonl"],
    }
    out = session / "a1_calibration_draft.json"
    out.write_text(json.dumps(draft, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"sign candidate: {sign['steer_command_sign_for_adc_increase_candidate']} (runs {sign['runs_used']})")
    print(f"draft written: {out} (not config/steering_calibration.json)")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("mark"); m.add_argument("--session", required=True); m.add_argument("--label", required=True, choices=LABELS)
    m.add_argument("--wheel", required=True, choices=("left", "right", "both"))
    m.add_argument("--angle-deg", type=float, required=True, help="measured angle of the named wheel (left wheel when --wheel both); left +, right -")
    m.add_argument("--angle-deg-right", type=float, default=None, help="right wheel angle when --wheel both")
    m.add_argument("--window-s", type=float, default=3.0); m.add_argument("--mechanical-end", action="store_true")
    m.add_argument("--note", default=""); m.add_argument("--now-utc", default=None, help="tests/replay only")
    m.set_defaults(fn=cmd_mark)
    s = sub.add_parser("sign"); s.add_argument("--session", required=True); s.set_defaults(fn=cmd_sign)
    u = sub.add_parser("summary"); u.add_argument("--session", required=True); u.set_defaults(fn=cmd_summary)
    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
