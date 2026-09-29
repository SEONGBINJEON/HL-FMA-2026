#!/usr/bin/env python3
"""Claude 2 · verify the V03 v2 clips (2) and new PNG samples (8) on this laptop.

What it CAN check here (no original 15-min file on this PC):
  - our own SHA256 of each file vs the hash the team wrote in manifest.md (claimed vs computed)
  - clip open / dims / fps / frame count / first-middle-last decode (OpenCV, 1 thread)
  - each PNG (extracted by the team from the ORIGINAL) vs the clip frame at the
    manifest time  (clip_idx = original_frame - clip_start_frame), best offset in ±3 frames,
    plus the same comparison against the horizontally mirrored PNG (mirror check)
  - black top/bottom margin rows in PNGs and in the clip frames
What it CANNOT check: clip <-> original full timeline, speed drift vs original, colour edits.
Outputs docs/analysis/claude2_video_2026-09-18/v03_v2_verification.json
"""
import hashlib, json, re, sys
from datetime import datetime, timezone, timedelta
from pathlib import Path
import cv2, numpy as np

cv2.setNumThreads(1)
ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "영상팀_답변 파일2/차에 붙인 영상/주행 영상 1/추가작업_20260918"
OUT = ROOT / "docs/analysis/claude2_video_2026-09-18/v03_v2_verification.json"
KST = timezone(timedelta(hours=9))
CLIPS = {"BEND": ("clips/V03_BEND_ENTRY_EXIT_v2.mp4", 2610, 87.0),
         "SCURVE": ("clips/V03_SCURVE_ENTRY_EXIT_v2.mp4", 5970, 199.0)}
FPS_CLAIMED = 30


def sha256(p):
    h = hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def claimed_hashes(md):
    out = {}
    for line in md.read_text(encoding="utf-8").splitlines():
        names = re.findall(r"`([^`]+\.(?:png|mp4))`", line)
        hashes = re.findall(r"`([0-9a-f]{64})`", line)
        if names and hashes:
            if len(names) == 1:
                out[names[0]] = hashes[0]
            else:
                for n, h in zip(names, hashes):
                    out[n] = h
    # clip table: names in header row, hashes in a later row
    lines = md.read_text(encoding="utf-8").splitlines()
    for i, line in enumerate(lines):
        if line.startswith("| SHA256 |") and "png" not in line:
            hashes = re.findall(r"`([0-9a-f]{64})`", line)
            for j in range(i, -1, -1):
                if lines[j].startswith("| 항목 |"):
                    names = re.findall(r"(V03_\w+\.mp4)", lines[j]); break
            for n, h in zip(names, hashes):
                out[n] = h
    return out


def margins(img, thr=16):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    rowmean = g.mean(axis=1)
    top = int(np.argmax(rowmean > thr)) if (rowmean > thr).any() else g.shape[0]
    bot = int(np.argmax(rowmean[::-1] > thr)) if (rowmean > thr).any() else g.shape[0]
    colmean = g.mean(axis=0)
    left = int(np.argmax(colmean > thr)) if (colmean > thr).any() else g.shape[1]
    right = int(np.argmax(colmean[::-1] > thr)) if (colmean > thr).any() else g.shape[1]
    return {"top": top, "bottom": bot, "left": left, "right": right}


def ncc(a, b):
    a = a.astype(np.float32).ravel(); b = b.astype(np.float32).ravel()
    a -= a.mean(); b -= b.mean()
    d = np.linalg.norm(a) * np.linalg.norm(b)
    return float(a @ b / d) if d else 0.0


def small_gray(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return cv2.resize(g, (360, 270), interpolation=cv2.INTER_AREA)


def read_frame(cap, idx):
    cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
    ok, fr = cap.read()
    return fr if ok else None


def main():
    md = BASE / "manifest.md"
    claimed = claimed_hashes(md)
    report = {"verified_kst": datetime.now(KST).isoformat(timespec="seconds"), "operator": "Claude 2",
              "hardware_access": False, "base": str(BASE.relative_to(ROOT)), "manifest_hashes_parsed": len(claimed),
              "original_available_on_this_pc": False, "clips": {}, "frames": []}
    caps = {}
    for key, (rel, start_frame, start_s) in CLIPS.items():
        p = BASE / rel
        c = cv2.VideoCapture(str(p)); assert c.isOpened(), p
        n = int(c.get(cv2.CAP_PROP_FRAME_COUNT))
        info = {"file": rel, "size": p.stat().st_size, "sha256_computed": sha256(p), "sha256_claimed": claimed.get(p.name),
                "width": int(c.get(cv2.CAP_PROP_FRAME_WIDTH)), "height": int(c.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                "fps": c.get(cv2.CAP_PROP_FPS), "frames_reported": n, "clip_start_original_frame_claimed": start_frame,
                "clip_start_original_s_claimed": start_s, "decoded": {}, "margins_first_mid_last": {}}
        info["sha256_match"] = info["sha256_computed"] == info["sha256_claimed"]
        for name, idx in (("first", 0), ("mid", n // 2), ("last", n - 1)):
            fr = read_frame(c, idx)
            info["decoded"][name] = {"index": idx, "ok": fr is not None}
            if fr is not None:
                info["margins_first_mid_last"][name] = margins(fr)
        caps[key] = c
        report["clips"][key] = info
    pat = re.compile(r"V03_(BEND|SCURVE)_(\w+)_t(\d\d)-(\d\d)-(\d\d)\.(\d{3})\.png")
    for p in sorted((BASE / "frames").glob("*.png")):
        m = pat.match(p.name); assert m, p.name
        key = m.group(1)
        t = int(m.group(3)) * 3600 + int(m.group(4)) * 60 + int(m.group(5)) + int(m.group(6)) / 1000
        orig_frame = int(round(t * FPS_CLAIMED))
        start_frame = CLIPS[key][1]
        clip_idx = orig_frame - start_frame
        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        row = {"file": "frames/" + p.name, "size": p.stat().st_size, "sha256_computed": sha256(p), "sha256_claimed": claimed.get(p.name),
               "original_time_s_from_filename": t, "original_frame_from_time_x30": orig_frame,
               "clip": key, "expected_clip_frame_index": clip_idx, "png_readable": img is not None}
        row["sha256_match"] = row["sha256_computed"] == row["sha256_claimed"]
        if img is None:
            report["frames"].append(row); continue
        row["width"], row["height"] = img.shape[1], img.shape[0]
        row["margins"] = margins(img)
        sg = small_gray(img); sgm = cv2.flip(sg, 1)
        cap = caps[key]; n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        best = None; comps = []
        for off in range(-3, 4):
            idx = clip_idx + off
            if idx < 0 or idx >= n:
                continue
            fr = read_frame(cap, idx)
            if fr is None:
                continue
            fg = small_gray(fr)
            r = {"offset": off, "clip_index": idx, "ncc": round(ncc(sg, fg), 4), "mad": round(float(np.abs(sg.astype(np.int16) - fg).mean()), 2),
                 "ncc_mirrored_png": round(ncc(sgm, fg), 4)}
            comps.append(r)
            if best is None or r["ncc"] > best["ncc"]:
                best = r
        row["clip_match"] = {"best": best, "all_offsets": comps,
                             "judgement": ("MATCH_OFFSET_0" if best and best["offset"] == 0 and best["ncc"] > 0.98 else
                                           "MATCH_OTHER_OFFSET" if best and best["ncc"] > 0.98 else "NO_MATCH"),
                             "mirror_lower_than_direct": bool(best and best["ncc_mirrored_png"] < best["ncc"] - 0.1)}
        report["frames"].append(row)
    for c in caps.values():
        c.release()
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k not in ("frames", "clips")}, ensure_ascii=False))
    for k, v in report["clips"].items():
        print(k, v["width"], v["height"], v["fps"], v["frames_reported"], "sha_match", v["sha256_match"], "decoded", {a: b["ok"] for a, b in v["decoded"].items()}, "margins", v["margins_first_mid_last"])
    bad = 0
    for f in report["frames"]:
        cm = f.get("clip_match", {})
        print(f"{Path(f['file']).name:42s} sha_match={f['sha256_match']} {f.get('width')}x{f.get('height')} margins={f.get('margins')} clip_idx={f['expected_clip_frame_index']} -> {cm.get('judgement')} best={cm.get('best')}")
        if not f["sha256_match"] or cm.get("judgement") != "MATCH_OFFSET_0" or not cm.get("mirror_lower_than_direct"):
            bad += 1
    print("frames_with_issue:", bad)
    return 1 if bad or not all(v["sha256_match"] for v in report["clips"].values()) else 0


if __name__ == "__main__":
    sys.exit(main())
