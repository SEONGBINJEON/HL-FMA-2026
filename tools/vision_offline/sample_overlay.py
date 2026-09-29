#!/usr/bin/env python3
"""Draw the AI provisional review polylines (sample_review.json) on the original frames.

For every sample also cross-check the PNG against the V03 v2 clip frame at
(frame_claimed - clip_start_frame) with ±3 offsets (NCC on 360x270 gray), so
each overlay is traceable to a clip time. Writes <out>/<id>_<name>_overlay.png and
<out>/overlay_index.json. Sources are never modified.
usage: sample_overlay.py [--review JSON] [--out DIR]
"""
import argparse, hashlib, json, sys
from pathlib import Path
import cv2, numpy as np
cv2.setNumThreads(1)
ROOT = Path(__file__).resolve().parents[2]
CLIPS = {"BEND": (ROOT / "영상팀_답변 파일2/차에 붙인 영상/주행 영상 1/추가작업_20260918/clips/V03_BEND_ENTRY_EXIT_v2.mp4", 2610),
         "SCURVE": (ROOT / "영상팀_답변 파일2/차에 붙인 영상/주행 영상 1/추가작업_20260918/clips/V03_SCURVE_ENTRY_EXIT_v2.mp4", 5970)}
COLORS = {"image_left": (0, 200, 255), "image_right": (255, 120, 0), "transverse": (255, 0, 255), "transverse_far": (255, 0, 255),
          "curve_left_to_bottom": (255, 0, 255), "center": (0, 255, 0)}


def sha256(p):
    h = hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def ncc(a, b):
    a = a.astype(np.float32).ravel(); b = b.astype(np.float32).ravel(); a -= a.mean(); b -= b.mean()
    d = np.linalg.norm(a) * np.linalg.norm(b); return float(a @ b / d) if d else 0.0


def sg(img):
    return cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (360, 270), interpolation=cv2.INTER_AREA)


def clip_check(cap, start, frame_claimed, img):
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); base = frame_claimed - start; best = None
    for off in range(-3, 4):
        i = base + off
        if 0 <= i < n:
            cap.set(cv2.CAP_PROP_POS_FRAMES, i); ok, fr = cap.read()
            if ok:
                v = ncc(sg(img), sg(fr))
                if best is None or v > best["ncc"]:
                    best = {"clip_index": i, "offset_vs_claimed": off, "ncc": round(v, 4), "clip_time_s": round(i / 30.0, 3)}
    return best


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--review", default=str(ROOT / "docs/analysis/claude2_video_2026-09-18/sample_review.json"))
    ap.add_argument("--out", default=str(ROOT / "docs/analysis/claude2_video_2026-09-18/sample_overlays"))
    a = ap.parse_args(argv)
    rv = json.loads(Path(a.review).read_text()); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    caps = {k: (cv2.VideoCapture(str(p)), s) for k, (p, s) in CLIPS.items()}
    index = []; fail = 0
    for s in rv["samples"]:
        src = ROOT / s["file"]; img = cv2.imread(str(src))
        if img is None:
            print("UNREADABLE", src); fail += 1; continue
        cap, start = caps[s["course"]]
        chk = clip_check(cap, start, s["frame_claimed"], img)
        ov = img.copy()
        for b in s["boundaries"]:
            pts = b["polyline_px_original"]
            if not b.get("visible", True) or len(pts) < 2:
                continue
            col = COLORS.get(b["image_side_label"], (200, 200, 200))
            cv2.polylines(ov, [np.array(pts, np.int32)], False, col, 3)
            for p in pts:
                cv2.circle(ov, tuple(p), 6, col, 2)
            cv2.putText(ov, f"{b['image_side_label']}:{b['boundary_type']}", (max(pts[0][0], 5), max(pts[0][1] - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        hdr = f"{s['id']} {src.name} t={s['t']}s f{s['frame_claimed']} | AI provisional, NOT verified label | clip_idx={chk['clip_index'] if chk else 'NA'} ncc={chk['ncc'] if chk else 'NA'}"
        cv2.putText(ov, hdr, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        # Hershey fonts cannot draw Korean; details stay in sample_review.json
        cv2.putText(ov, f"exposure/occlusion/course-LR notes: sample_review.json#{s['id']} (Korean text not rendered here)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(ov, "coords ~+-20px read from 0.5x grid preview; hidden/overexposed boundaries NOT drawn; transverse curb NOT split into L/R", (10, 1060), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        name = f"{s['id']}_{src.stem}_overlay.png"; cv2.imwrite(str(out / name), ov)
        index.append({"id": s["id"], "source": s["file"], "source_sha256": sha256(src), "overlay": str((out / name).relative_to(ROOT)),
                      "clip_check": chk, "clip_match_ok": bool(chk and chk["ncc"] > 0.98)})
        print(s["id"], src.name, "clip_check", chk)
        if not (chk and chk["ncc"] > 0.98):
            fail += 1
    (out / "overlay_index.json").write_text(json.dumps({"review": str(Path(a.review).relative_to(ROOT)), "items": index}, ensure_ascii=False, indent=1) + "\n")
    print("written", len(index), "overlays; issues:", fail)
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
