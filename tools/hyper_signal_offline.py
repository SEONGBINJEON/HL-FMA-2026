#!/usr/bin/env python3
"""Offline replay of HYPER hyper_object_detection's traffic-light decision on saved JPEGs.
Same rules as object_detection_node.py (2026-09-14 f853741): YOLO conf>=0.5, boxes mapped by SIGNAL_MAP,
only boxes whose centre lies in the middle 50% of the image, nearest-to-centre wins, blank->none.
Outputs per-folder tallies + annotated sample images. No ROS, no vehicle access.
usage: hyper_signal_offline.py --model best_track.pt --out DIR FOLDER [FOLDER ...]"""
import argparse, json, glob, os, sys
from collections import Counter
from pathlib import Path
import cv2
from ultralytics import YOLO

SIGNAL_MAP = {'Stop': 'red', 'Go': 'green', 'LeftTurn': 'left_arrow', 'Warn': 'none', 'Yellow': 'none',
              'Blank': 'blank', 'Ban': 'ban', 'Allow': 'allow'}

def decide(result, width):
    boxes = result.boxes
    if boxes is None or len(boxes) == 0: return 'none', None, []
    lo, hi, mid = width * .25, width * .75, width / 2
    best = None; seen = []
    for b in boxes:
        raw = str(result.names[int(b.cls[0])]); conf = float(b.conf[0]); x1, y1, x2, y2 = b.xyxy[0].tolist()
        seen.append((raw, round(conf, 2), int(x1), int(y1), int(x2), int(y2)))
        sign = SIGNAL_MAP.get(raw)
        if sign is None: continue
        cx = (x1 + x2) / 2
        if not lo <= cx <= hi: continue
        key = (abs(cx - mid), -conf)
        if best is None or key < best[0]: best = (key, sign, raw, conf, (int(x1), int(y1), int(x2), int(y2)))
    if best is None: return 'none', None, seen
    sign = 'none' if best[1] == 'blank' else best[1]
    return sign, {'raw': best[2], 'conf': round(best[3], 3), 'box': best[4]}, seen

def main():
    p = argparse.ArgumentParser(); p.add_argument('--model', required=True); p.add_argument('--out', required=True)
    p.add_argument('--conf', type=float, default=0.5); p.add_argument('--every', type=int, default=1)
    p.add_argument('--crop', type=float, default=1.0, help='centre crop fraction before inference (0.7 emulates HYPER C920 70deg FOV); resized to 1280x720')
    p.add_argument('--imgsz', type=int, default=800, help="YOLO input size; 800 is best_track.pt's training size")
    p.add_argument('folders', nargs='+')
    a = p.parse_args(); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    model = YOLO(a.model); print('classes', model.names, flush=True)
    report = {'model': a.model, 'conf': a.conf, 'crop': a.crop, 'imgsz': a.imgsz, 'classes': model.names, 'folders': {}}
    for folder in a.folders:
        files = sorted(glob.glob(os.path.join(folder, '*.jpg')))[::a.every]
        tally = Counter(); raw_tally = Counter(); rows = []; saved = 0
        for f in files:
            img = cv2.imread(f)
            if img is None: continue
            if a.crop < 1.0:
                h, w = img.shape[:2]; cw, ch = int(w * a.crop), int(h * a.crop)
                img = cv2.resize(img[(h - ch) // 2:(h - ch) // 2 + ch, (w - cw) // 2:(w - cw) // 2 + cw], (1280, 720))
            res = model(img, conf=a.conf, imgsz=a.imgsz, verbose=False)[0]
            sign, best, seen = decide(res, img.shape[1])
            tally[sign] += 1
            for s in seen: raw_tally[s[0]] += 1
            rows.append({'file': os.path.basename(f), 'sign': sign, 'best': best, 'all_boxes': seen})
            if saved < 2 and best is not None:
                ann = res.plot(); cv2.putText(ann, f'{Path(folder).name}: {sign} ({best["raw"]} {best["conf"]})', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.imwrite(str(out / f'{Path(folder).name}_{os.path.basename(f)}'), ann); saved += 1
        n = len(rows)
        report['folders'][folder] = {'images': n, 'decisions': dict(tally), 'raw_classes_seen': dict(raw_tally),
                                     'decision_rate': {k: round(v / n, 3) for k, v in tally.items()} if n else {}, 'rows': rows}
        print(f'{folder}: {n} images -> {dict(tally)} | raw {dict(raw_tally)}', flush=True)
    (out / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1))
    print('report', out / 'report.json')

if __name__ == '__main__': main()
