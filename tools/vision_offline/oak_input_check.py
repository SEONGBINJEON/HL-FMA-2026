#!/usr/bin/env python3
"""Claude 2 · read-only INPUT/OUTPUT compatibility check for stored OAK recordings.

WHAT THIS IS: proof that the existing offline prototype can open the OAK session
files, restore original-pixel coordinates, emit JSON/overlay, and that a frames.jsonl
record can be tied to a stored-video frame index.

WHAT THIS IS NOT: a boundary-detection performance test. The stored OAK material is an
indoor session (corridor/stairs). Lines found in a room are not road boundaries, and
nothing here says anything about curbs, the course, OAK outdoor behaviour or driving.

Frozen: boundary_candidates.DEFAULTS are used unchanged (no --config, no overrides).
No device access, no writes to the source folders, no re-encoding.

usage: oak_input_check.py [--root DIR] [--out DIR]
"""
import argparse, hashlib, json, sys
from datetime import datetime, timezone, timedelta
from pathlib import Path

import cv2, numpy as np

cv2.setNumThreads(1)
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import boundary_candidates as bc  # noqa: E402

KST = timezone(timedelta(hours=9))
DEFAULT_ROOT = ROOT / "data/sensor_check_20260916_010124_09c035ea"
RUNS = ("oak", "oak_preview2")
SMALL = ("report.json", "frames.jsonl", "first_frame.jpg", "latest.jpg", "executed_oak_rgb_monitor.py")


def sha256(p):
    h = hashlib.sha256()
    with Path(p).open("rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def ncc(a, b):
    a = a.astype(np.float32).ravel(); b = b.astype(np.float32).ravel()
    a -= a.mean(); b -= b.mean()
    d = np.linalg.norm(a) * np.linalg.norm(b)
    return float(a @ b / d) if d else 0.0


def gray(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def read_index(cap, idx):
    cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
    ok, fr = cap.read()
    return fr if ok else None


def scan_journal(path):
    """Full scan of frames.jsonl (small file): count, contiguity, first/last records."""
    rows = 0; contiguous = True; first = last = None; seq_ok = True
    with Path(path).open(encoding="utf-8") as f:
        for i, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            if first is None:
                first = r
            last = r
            if r.get("recorded_frame_index") != i:
                contiguous = False
            if r.get("sequence") != i:
                seq_ok = False
            rows += 1
    return {"rows": rows, "recorded_frame_index_is_0_based_contiguous": contiguous,
            "sequence_equals_row_number": seq_ok, "first_record": first, "last_record": last}


def check_run(run_dir, out_dir, results):
    name = run_dir.name
    r = {"run": name, "dir": str(run_dir.relative_to(ROOT)), "checks": {}, "detector_samples": [],
         "provenance_decodes": []}
    report = json.loads((run_dir / "report.json").read_text())
    r["report_json"] = {k: report.get(k) for k in ("started_utc", "ended_utc", "status", "device_name", "device_id",
                                                   "usb_speed", "sdk_version", "opencv_version", "requested_fps",
                                                   "requested_size", "observed_size", "frames", "sequence_gaps",
                                                   "received_fps", "elapsed_s", "source_sha256", "limitations")}
    r["files"] = {}
    for fn in SMALL:
        p = run_dir / fn
        r["files"][fn] = {"size": p.stat().st_size, "sha256": sha256(p)} if p.is_file() else None
    avi = run_dir / "rgb.avi"
    r["files"]["rgb.avi"] = {"size": avi.stat().st_size, "sha256": sha256(avi)}

    jr = scan_journal(run_dir / "frames.jsonl")
    r["frames_jsonl"] = jr

    cap = cv2.VideoCapture(str(avi))
    if not cap.isOpened():
        r["checks"]["avi_opens"] = False
        results.append(r); return False
    r["checks"]["avi_opens"] = True
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    r["video"] = {"reader": "OpenCV " + cv2.__version__, "frame_count_reported": n,
                  "container_fps_nominal": fps, "fourcc": "".join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4)),
                  "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}
    r["checks"]["counts_agree_report_jsonl_video"] = (report.get("frames") == jr["rows"] == n)

    # --- nominal container timeline vs real per-frame timestamps ---
    f0r, flr = jr["first_record"], jr["last_record"]
    nominal_fps = fps or 15.0
    dev_elapsed = flr["device_timestamp_s"] - f0r["device_timestamp_s"]
    nom_elapsed = (flr["recorded_frame_index"] - f0r["recorded_frame_index"]) / nominal_fps
    r["timeline"] = {
        "container_fps_nominal": nominal_fps,
        "first_index": f0r["recorded_frame_index"], "last_index": flr["recorded_frame_index"],
        "first_host_utc": f0r["host_utc"], "last_host_utc": flr["host_utc"],
        "device_elapsed_s_first_to_last": round(dev_elapsed, 6),
        "nominal_elapsed_s_first_to_last": round(nom_elapsed, 6),
        "drift_s_nominal_minus_device": round(nom_elapsed - dev_elapsed, 3),
        "actual_mean_fps_from_device_timestamps": round((flr["recorded_frame_index"] - f0r["recorded_frame_index"]) / dev_elapsed, 6) if dev_elapsed else None,
        "conclusion": ("the AVI's 15fps timeline is NOMINAL; real per-frame time must be taken from frames.jsonl "
                       "(host_utc / device_timestamp_s) at the same recorded_frame_index"),
        "gps_or_wall_clock_sync": ("NOT established here. host_utc exists per frame, but no verification of host-clock "
                                   "accuracy, no GPS log correlation and no trigger evidence was done in this bundle"),
    }

    # --- detector samples: first frame and middle frame only (2 per run, 4 total) ---
    mid = n // 2
    with (run_dir / "frames.jsonl").open(encoding="utf-8") as f:
        wanted = {0: None, mid: None}
        for i, line in enumerate(f):
            if i in wanted:
                wanted[i] = json.loads(line)
            if i > mid:
                break
    for idx in (0, mid):
        rec = wanted[idx]
        tag = f"{name}_f{idx:05d}"
        res = bc.run(avi, out_dir, frame=idx, tag=tag)  # frozen DEFAULTS
        j = out_dir / f"{tag}.json"
        sample = {"video_frame_index": idx, "detector_json": str(j.relative_to(ROOT)),
                  "overlay": str(Path(res["overlay"]).relative_to(ROOT)),
                  "status": res["status"], "reason": res["reason"],
                  "n_candidates": len(res["candidates"]),
                  "labels": [c["image_position_label"] for c in res["candidates"]],
                  "transform": res.get("transform"),
                  "journal_record_at_same_index": rec,
                  "index_link": ("frames.jsonl recorded_frame_index == stored-video frame index (by construction in "
                                 "oak_rgb_monitor.py: writer.write(frame) then row with recorded_frame_index=frames)"),
                  "time_basis": ("host_utc/device_timestamp_s come from frames.jsonl for this exact index; the AVI's "
                                 "own 15fps timeline is nominal and must not be used as real time"),
                  "note": "indoor session; candidates are room edges, NOT road boundaries"}
        t = res.get("transform") or {}
        mg = t.get("black_margins") or {}
        if any(mg.get(k, 0) for k in ("top", "bottom", "left", "right")):
            sample["margin_autodetect_note"] = {
                "detected": mg,
                "issue": ("source is 640x480 with no letterbox, so these are DARK IMAGE EDGES mistaken for margins "
                          "by the mean<16 rule; the content rect and work scale then vary frame to frame"),
                "impact": "restored coordinates stay valid (offset is applied), but the analysed area is trimmed",
            }
        if rec is not None and r.get("timeline"):
            nom_t = (idx - r["timeline"]["first_index"]) / r["timeline"]["container_fps_nominal"]
            dev_t = rec["device_timestamp_s"] - r["timeline"]["first_index"] and (rec["device_timestamp_s"] - jr["first_record"]["device_timestamp_s"])
            sample["time_at_this_index"] = {
                "nominal_from_container_s": round(nom_t, 3),
                "device_elapsed_since_first_frame_s": round(dev_t, 3),
                "difference_s": round(nom_t - dev_t, 3),
                "host_utc": rec["host_utc"],
            }
        if res["candidates"]:
            sample["candidate_polylines_original_px"] = [c["polyline_px_original"] for c in res["candidates"]]
        r["detector_samples"].append(sample)

    # --- provenance decodes (NOT detector samples, not performance evidence) ---
    f0 = read_index(cap, 0)
    ff = cv2.imread(str(run_dir / "first_frame.jpg"))
    if f0 is not None and ff is not None:
        r["provenance_decodes"].append({"what": "first_frame.jpg vs stored-video frame 0", "video_frame_index": 0,
                                        "ncc": round(ncc(gray(ff), gray(f0)), 4),
                                        "same_size": bool(ff.shape == f0.shape),
                                        "meaning": "both are lossy encodes of the same captured frame; high NCC confirms index 0 alignment"})
    fl = read_index(cap, n - 1)
    lj = cv2.imread(str(run_dir / "latest.jpg"))
    if fl is not None and lj is not None:
        r["provenance_decodes"].append({"what": "latest.jpg vs stored-video last frame", "video_frame_index": n - 1,
                                        "ncc": round(ncc(gray(lj), gray(fl)), 4),
                                        "meaning": "latest.jpg is a ~10Hz replaceable preview; a lower NCC only means it is an earlier frame, not an error"})
    cap.release()
    r["checks"]["first_frame_jpg_matches_index0"] = bool(r["provenance_decodes"] and r["provenance_decodes"][0]["ncc"] > 0.98)
    r["checks"]["detector_ran_on_all_samples"] = all(s["status"] in ("candidates", "no_candidate", "unreadable") for s in r["detector_samples"])
    r["checks"]["coords_within_original_frame"] = all(
        all(0 <= x < r["video"]["width"] and 0 <= y < r["video"]["height"] for poly in s.get("candidate_polylines_original_px", []) for x, y in poly)
        for s in r["detector_samples"])
    results.append(r)
    return all(v for k, v in r["checks"].items())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=str(DEFAULT_ROOT))
    ap.add_argument("--out", default=str(ROOT / "docs/analysis/claude2_video_2026-09-18/oak_input_check"))
    a = ap.parse_args(argv)
    root = Path(a.root); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    results = []; ok = True
    for name in RUNS:
        d = root / name
        if not d.is_dir():
            results.append({"run": name, "error": "missing folder"}); ok = False; continue
        ok = check_run(d, out, results) and ok
    doc = {"checked_kst": datetime.now(KST).isoformat(timespec="seconds"), "operator": "Claude 2",
           "hardware_access": False, "device_connected": False,
           "purpose": "input/output compatibility of the frozen offline prototype with stored OAK files",
           "not_a_performance_test": ("stored OAK material is an INDOOR session; detected lines are room edges. "
                                      "No road-boundary, curb, course, outdoor-OAK or drivability conclusion may be drawn."),
           "detector_source": "tools/vision_offline/boundary_candidates.py",
           "detector_source_sha256": sha256(ROOT / "tools/vision_offline/boundary_candidates.py"),
           "detector_config": "frozen DEFAULTS, no overrides", "config_used": bc.DEFAULTS,
           "detector_sample_count": sum(len(r.get("detector_samples", [])) for r in results),
           "runs": results}
    (out / "oak_input_check.json").write_text(json.dumps(doc, ensure_ascii=False, indent=1) + "\n")
    print(json.dumps({k: v for k, v in doc.items() if k not in ("runs", "config_used")}, ensure_ascii=False, indent=1))
    for r in results:
        print("--", r["run"], r.get("error", ""), r.get("checks"))
        v = r.get("video", {})
        print("   video:", v, "| jsonl rows:", r.get("frames_jsonl", {}).get("rows"),
              "contiguous:", r.get("frames_jsonl", {}).get("recorded_frame_index_is_0_based_contiguous"))
        tl = r.get("timeline")
        if tl:
            print(f"   timeline: nominal {tl['nominal_elapsed_s_first_to_last']}s vs device {tl['device_elapsed_s_first_to_last']}s "
                  f"-> drift {tl['drift_s_nominal_minus_device']}s, actual mean fps {tl['actual_mean_fps_from_device_timestamps']}")
        for s in r.get("detector_samples", []):
            print(f"   sample f{s['video_frame_index']}: {s['status']} {s['reason'] or ''} n={s['n_candidates']} {s['labels']}")
            print(f"      host_utc={s['journal_record_at_same_index']['host_utc']} device_ts_s={s['journal_record_at_same_index']['device_timestamp_s']}")
            if "time_at_this_index" in s:
                print(f"      time: {s['time_at_this_index']}")
            if "margin_autodetect_note" in s:
                print(f"      margin autodetect: {s['margin_autodetect_note']['detected']}")
        for p in r.get("provenance_decodes", []):
            print(f"   provenance: {p['what']} ncc={p['ncc']}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
