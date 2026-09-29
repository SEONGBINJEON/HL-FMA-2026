#!/usr/bin/env python3
"""OAK RGB burst: save N JPEG frames (with host UTC timestamps) to a new folder. depthai v3. No vehicle access.
usage: oak_burst_capture.py --output-dir DIR [--frames 100] [--label red]"""
import argparse, json, time
from datetime import datetime, timezone
from pathlib import Path
import cv2
import depthai as dai

p = argparse.ArgumentParser(); p.add_argument('--output-dir', required=True); p.add_argument('--frames', type=int, default=100)
p.add_argument('--label', default=''); p.add_argument('--ae-comp', type=int, default=-6, help='auto-exposure compensation (-9..9); negative = darker'); p.add_argument('--every', type=int, default=1, help='save every Nth frame (30fps source)'); p.add_argument('--width', type=int, default=1920); p.add_argument('--height', type=int, default=1080)
a = p.parse_args(); out = Path(a.output_dir); out.mkdir(parents=True, exist_ok=False)
meta = {'label': a.label, 'started_utc': datetime.now(timezone.utc).isoformat(), 'frames': [], 'size': [a.width, a.height], 'ae_comp': a.ae_comp}
with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    try: cam.initialControl.setAutoExposureCompensation(a.ae_comp)
    except Exception as e: print('ae-comp not applied:', e)
    q = cam.requestOutput((a.width, a.height), dai.ImgFrame.Type.BGR888i, fps=30).createOutputQueue(maxSize=4, blocking=True)
    pipeline.start()
    for _ in range(5): q.get()   # let exposure settle
    n = 0
    k = 0
    while n < a.frames:
        f = q.get(); ts = datetime.now(timezone.utc); k += 1
        if k % a.every: continue
        name = f'{a.label}_{n:03d}.jpg' if a.label else f'{n:03d}.jpg'
        cv2.imwrite(str(out / name), f.getCvFrame(), [cv2.IMWRITE_JPEG_QUALITY, 92])
        meta['frames'].append({'file': name, 'host_utc': ts.isoformat(), 'seq': f.getSequenceNum()}); n += 1
meta['ended_utc'] = datetime.now(timezone.utc).isoformat()
(out / 'metadata.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1))
print(json.dumps({'saved': n, 'dir': str(out), 'first': meta['frames'][0]['host_utc'], 'last': meta['frames'][-1]['host_utc']}))
