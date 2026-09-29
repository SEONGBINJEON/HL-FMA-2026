#!/usr/bin/env python3
"""OAK RGB, camera kept open: type a label (red/green/...) + Enter -> N frames saved IMMEDIATELY (no start delay).
q + Enter quits. Each burst goes to <base>/<label>_<HHMMSS>/ with metadata.json (host UTC per frame)."""
import argparse, json, sys, select
from datetime import datetime, timezone
from pathlib import Path
import cv2
import depthai as dai

p = argparse.ArgumentParser(); p.add_argument('--base', required=True); p.add_argument('--frames', type=int, default=100)
p.add_argument('--ae-comp', type=int, default=-6); p.add_argument('--width', type=int, default=1920); p.add_argument('--height', type=int, default=1080)
a = p.parse_args(); base = Path(a.base); base.mkdir(parents=True, exist_ok=True)
with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    try: cam.initialControl.setAutoExposureCompensation(a.ae_comp)
    except Exception as e: print('ae-comp not applied:', e)
    q = cam.requestOutput((a.width, a.height), dai.ImgFrame.Type.BGR888i, fps=30).createOutputQueue(maxSize=1, blocking=False)
    pipeline.start()
    print('camera ready. label + Enter = burst now (e.g. red / green). q = quit', flush=True)
    while True:
        f = q.tryGet()           # keep draining so the next burst starts from a fresh frame
        r, _, _ = select.select([sys.stdin], [], [], 0.01)
        if not r: continue
        label = sys.stdin.readline().strip() or 'frame'
        if label == 'q': break
        out = base / f"{label}_{datetime.now().strftime('%H%M%S')}"; out.mkdir()
        meta = {'label': label, 'started_utc': datetime.now(timezone.utc).isoformat(), 'frames': []}; n = 0
        while n < a.frames:
            f = q.tryGet()
            if f is None: continue
            ts = datetime.now(timezone.utc); name = f'{label}_{n:03d}.jpg'
            cv2.imwrite(str(out / name), f.getCvFrame(), [cv2.IMWRITE_JPEG_QUALITY, 92])
            meta['frames'].append({'file': name, 'host_utc': ts.isoformat(), 'seq': f.getSequenceNum()}); n += 1
        meta['ended_utc'] = datetime.now(timezone.utc).isoformat()
        (out / 'metadata.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1))
        print(f'saved {n} frames -> {out}  ({meta["started_utc"][11:19]}Z .. {meta["ended_utc"][11:19]}Z)', flush=True)
