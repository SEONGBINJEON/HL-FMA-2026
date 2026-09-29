#!/usr/bin/env python3
"""Offline, FILE-INPUT-ONLY boundary-candidate prototype (Claude 2, 2026-09-18).

One simple classical method, no learning, no camera/device access:
  1. read image (PNG/JPG) or one frame of a stored clip (--frame N | --time S)
  2. detect black letterbox margins -> content rect; downscale to --work-width
  3. reject unreadable frames: over-/under-exposed ROI
  4. Canny edges (gray) OR yellow-paint mask edges (HSV) inside the lower ROI
  5. HoughLinesP segments -> greedy chaining into polylines
  6. keep the longest chains; map back to ORIGINAL pixel coordinates
Output: <out>/<stem>.json + <out>/<stem>_overlay.png.

The JSON labels a candidate only by its IMAGE position (image_left/right/center or
transverse). course_left_right is always "undetermined": screen side is not course side,
image centre is not the vehicle path, and score = normalised chain length, NOT a
verified probability or an operating threshold. Phone-video results say nothing
about the real OAK camera on the vehicle.
"""
import argparse, hashlib, json, math, sys
from datetime import datetime, timezone, timedelta
from pathlib import Path
import cv2, numpy as np

cv2.setNumThreads(1)
KST = timezone(timedelta(hours=9))
DEFAULTS = dict(work_width=720, roi_top_frac=0.45, margin_thr=16, over_frac=0.45, over_val=235, under_frac=0.6, under_val=40,
                canny_lo=80, canny_hi=200, blur=7, hough_thr=40, min_seg=40, max_gap=6, link_dist=20, link_angle=20,
                min_chain=120, min_straightness=0.85, max_candidates=4, transverse_deg=8, yellow=(15, 40, 70, 110))


def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def detect_margins(img, thr):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    rows = np.where(g.mean(axis=1) > thr)[0]; cols = np.where(g.mean(axis=0) > thr)[0]
    if len(rows) == 0 or len(cols) == 0:
        return None
    return int(cols[0]), int(rows[0]), int(cols[-1] + 1), int(rows[-1] + 1)  # x0,y0,x1,y1


def load_input(path, frame, time_s):
    p = Path(path)
    if not p.is_file() or p.stat().st_size == 0:
        raise ValueError(f"missing or empty input: {path}")
    src = {"path": str(path), "sha256": sha256(p), "kind": None}
    if p.suffix.lower() in (".mp4", ".mov", ".avi", ".mkv"):
        cap = cv2.VideoCapture(str(p))
        if not cap.isOpened():
            raise ValueError(f"clip not readable: {path}")
        fps = cap.get(cv2.CAP_PROP_FPS) or 0.0; n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if frame is None:
            if time_s is None:
                raise ValueError("clip input needs --frame or --time")
            if not fps:
                raise ValueError("clip fps unknown; give --frame")
            frame = int(round(time_s * fps))
        if frame < 0 or frame >= n:
            raise ValueError(f"frame {frame} outside clip range 0..{n-1}")
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame); ok, img = cap.read(); cap.release()
        if not ok or img is None:
            raise ValueError(f"frame {frame} could not be decoded")
        src.update(kind="clip_frame", clip_frame_index=frame, clip_fps_reported=fps, clip_time_s=(frame / fps if fps else None),
                   original_video_time="unknown unless the caller supplies the clip's documented start offset")
    else:
        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError(f"image not decodable: {path}")
        src["kind"] = "image"
    return img, src


def chain_segments(segs, link_dist, link_angle):
    """Greedy directional chaining. Start from the longest unused segment and extend each
    end with the nearest unused segment whose near endpoint is within link_dist, whose
    angle differs <= link_angle from the seed AND whose far endpoint continues in the
    chain's travel direction (no back-tracking over road texture)."""
    def ang(s):
        return math.degrees(math.atan2(s[3] - s[1], s[2] - s[0]))
    def adiff(a, b):
        d = abs(a - b) % 180
        return min(d, 180 - d)
    used = [False] * len(segs)
    order = sorted(range(len(segs)), key=lambda i: -math.hypot(segs[i][2] - segs[i][0], segs[i][3] - segs[i][1]))
    chains = []
    for i in order:
        if used[i]:
            continue
        used[i] = True
        pts = [(segs[i][0], segs[i][1]), (segs[i][2], segs[i][3])]
        a0 = ang(segs[i])
        grew = True
        while grew:
            grew = False
            for end in (0, -1):
                tip = pts[end]; prev = pts[1] if end == 0 else pts[-2]
                dx, dy = tip[0] - prev[0], tip[1] - prev[1]
                best = None
                for j in range(len(segs)):
                    if used[j] or adiff(a0, ang(segs[j])) > link_angle:
                        continue
                    for (px, py, qx, qy) in (segs[j], (segs[j][2], segs[j][3], segs[j][0], segs[j][1])):
                        d = math.hypot(px - tip[0], py - tip[1])
                        forward = (qx - tip[0]) * dx + (qy - tip[1]) * dy
                        if d <= link_dist and forward > 0 and (best is None or d < best[0]):
                            best = (d, j, (qx, qy))
                if best:
                    used[best[1]] = True
                    if end == 0:
                        pts.insert(0, best[2])
                    else:
                        pts.append(best[2])
                    grew = True
        length = sum(math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1]) for k in range(len(pts) - 1))
        span = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
        chains.append({"pts": pts, "length": length, "span": span, "straightness": (span / length if length else 0.0),
                       "angle_deg": math.degrees(math.atan2(pts[-1][1] - pts[0][1], pts[-1][0] - pts[0][0]))})
    return chains


def analyse(img, cfg):
    H, W = img.shape[:2]
    res = {"image_size": [W, H], "status": None, "reason": None, "candidates": [], "course_left_right": "undetermined",
           "config": cfg}
    m = detect_margins(img, cfg["margin_thr"])
    if m is None:
        res.update(status="unreadable", reason="entire frame dark (no content region)"); return res, None
    x0, y0, x1, y1 = m
    content = img[y0:y1, x0:x1]
    scale = cfg["work_width"] / content.shape[1]
    work = cv2.resize(content, (cfg["work_width"], int(round(content.shape[0] * scale))), interpolation=cv2.INTER_AREA)
    res["transform"] = {"content_rect_original_xyxy": [x0, y0, x1, y1], "black_margins": {"top": y0, "bottom": H - y1, "left": x0, "right": W - x1},
                        "work_scale": scale, "work_size": [work.shape[1], work.shape[0]],
                        "to_original": "x_orig = x_work/work_scale + content_x0 ; y_orig = y_work/work_scale + content_y0"}
    wh, ww = work.shape[:2]
    roi_top = int(wh * cfg["roi_top_frac"])
    roi = work[roi_top:, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    over = float((gray > cfg["over_val"]).mean()); under = float((gray < cfg["under_val"]).mean())
    res["exposure"] = {"roi_top_work_y": roi_top, "roi_top_original_y": int(round(roi_top / scale + y0)), "frac_over": round(over, 3), "frac_under": round(under, 3)}
    if over > cfg["over_frac"]:
        res.update(status="unreadable", reason=f"overexposed: {over:.0%} of ROI pixels > {cfg['over_val']}"); return res, work
    if under > cfg["under_frac"]:
        res.update(status="unreadable", reason=f"underexposed: {under:.0%} of ROI pixels < {cfg['under_val']}"); return res, work
    b = cfg["blur"]
    edges = cv2.Canny(cv2.GaussianBlur(gray, (b, b), 0), cfg["canny_lo"], cfg["canny_hi"])
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hlo, hhi, smin, vmin = cfg["yellow"]
    ymask = cv2.inRange(hsv, (hlo, smin, vmin), (hhi, 255, 255))
    ymask = cv2.morphologyEx(ymask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    yedges = cv2.Canny(ymask, 50, 150)
    comb = cv2.bitwise_or(edges, yedges)
    res["edge_stats"] = {"edge_pixels": int((edges > 0).sum()), "yellow_mask_pixels": int((ymask > 0).sum())}
    lines = cv2.HoughLinesP(comb, 1, np.pi / 180, cfg["hough_thr"], minLineLength=cfg["min_seg"], maxLineGap=cfg["max_gap"])
    if lines is None or len(lines) == 0:
        res.update(status="no_candidate", reason="no line segments above threshold"); return res, work
    segs = [tuple(int(v) for v in l[0]) for l in lines]
    res["edge_stats"]["hough_segments"] = len(segs)
    chains = [c for c in chain_segments(segs, cfg["link_dist"], cfg["link_angle"]) if c["span"] >= cfg["min_chain"] and c["straightness"] >= cfg["min_straightness"]]
    if not chains:
        res.update(status="no_candidate", reason=f"no straight chain with span >= {cfg['min_chain']} work-px (segments={len(segs)})"); return res, work
    chains.sort(key=lambda c: -c["span"])
    longest = chains[0]["span"]
    for c in chains[:cfg["max_candidates"]]:
        pts_o = [[int(round(x / scale + x0)), int(round((y + roi_top) / scale + y0))] for x, y in c["pts"]]
        near = max(pts_o, key=lambda q: q[1])  # lowest point in the image = nearest to camera
        a = abs(c["angle_deg"]) % 180; a = min(a, 180 - a)
        if a <= cfg["transverse_deg"]:
            side = "transverse_candidate"
        else:
            side = "image_left" if near[0] < W * 0.4 else "image_right" if near[0] > W * 0.6 else "image_center"
        yl = ymask[[min(max(int(y), 0), ymask.shape[0] - 1) for x, y in c["pts"]], [min(max(int(x), 0), ymask.shape[1] - 1) for x, y in c["pts"]]]
        res["candidates"].append({"image_position_label": side, "polyline_px_original": pts_o, "length_work_px": round(c["length"], 1), "span_work_px": round(c["span"], 1), "straightness": round(c["straightness"], 3),
                                  "score_relative_span": round(c["span"] / longest, 3), "angle_deg_work": round(c["angle_deg"], 1),
                                  "touches_yellow_mask": bool((yl > 0).any()),
                                  "nearest_point_px_original": near, "meaning": "edge chain only; may be curb, paint, crack, shadow or object edge; label = image side of the nearest point"})
    res.update(status="candidates", reason=None)
    return res, work


def draw(img, res, header):
    ov = img.copy()
    cols = {"image_left": (0, 200, 255), "image_right": (255, 120, 0), "image_center": (0, 255, 0), "transverse_candidate": (255, 0, 255)}
    t = res.get("transform")
    if t:
        x0, y0, x1, y1 = t["content_rect_original_xyxy"]; cv2.rectangle(ov, (x0, y0), (x1 - 1, y1 - 1), (80, 80, 80), 1)
        ry = res["exposure"]["roi_top_original_y"]; cv2.line(ov, (x0, ry), (x1, ry), (80, 80, 80), 1)
    for c in res["candidates"]:
        col = cols[c["image_position_label"]]
        cv2.polylines(ov, [np.array(c["polyline_px_original"], np.int32)], False, col, 3)
        p = c["polyline_px_original"][0]
        cv2.putText(ov, f"{c['image_position_label']} L={c['length_work_px']:.0f}", (max(p[0], 5), max(p[1] - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    cv2.putText(ov, header, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(ov, f"status={res['status']} reason={res['reason']} course L/R=undetermined (image position only; not OAK, not drivability)",
                (10, ov.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    return ov


def run(path, out_dir, frame=None, time_s=None, cfg=None, tag=None, clip_start_original_s=None):
    cfg = {**DEFAULTS, **(cfg or {})}
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    img, src = load_input(path, frame, time_s)
    if clip_start_original_s is not None and src.get("clip_time_s") is not None:
        src["original_video_time_s_if_clip_start_documented"] = round(clip_start_original_s + src["clip_time_s"], 3)
        src["clip_start_original_s_supplied_by_caller"] = clip_start_original_s
    res, _ = analyse(img, cfg)
    res["source"] = src; res["run_kst"] = datetime.now(KST).isoformat(timespec="seconds")
    res["disclaimer"] = ("Offline file-input prototype on stored phone video. Candidates are image-coordinate edge chains, "
                         "not verified boundaries, not course left/right, not distances/GPS, not OAK or drivability evidence.")
    stem = tag or (Path(path).stem + (f"_f{src['clip_frame_index']}" if src["kind"] == "clip_frame" else ""))
    ov = draw(img, res, f"{stem} | {src['kind']} | {Path(path).name}" + (f" clip_idx={src['clip_frame_index']}" if src["kind"] == "clip_frame" else ""))
    cv2.imwrite(str(out_dir / f"{stem}_overlay.png"), ov)
    res["overlay"] = str(out_dir / f"{stem}_overlay.png")
    (out_dir / f"{stem}.json").write_text(json.dumps(res, ensure_ascii=False, indent=1) + "\n")
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input"); ap.add_argument("--out", required=True)
    ap.add_argument("--frame", type=int); ap.add_argument("--time", type=float)
    ap.add_argument("--clip-start-original-s", type=float, help="documented original-video time of clip frame 0 (only if known)")
    ap.add_argument("--tag"); ap.add_argument("--config", help="JSON file overriding DEFAULTS")
    a = ap.parse_args(argv)
    cfg = json.loads(Path(a.config).read_text()) if a.config else None
    try:
        res = run(a.input, a.out, a.frame, a.time, cfg, a.tag, a.clip_start_original_s)
    except ValueError as e:
        print(json.dumps({"status": "rejected", "reason": str(e), "input": a.input}, ensure_ascii=False)); return 2
    print(json.dumps({k: res[k] for k in ("status", "reason", "overlay")} | {"n_candidates": len(res["candidates"]),
                     "labels": [c["image_position_label"] for c in res["candidates"]]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
