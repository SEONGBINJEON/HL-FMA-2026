"""Latest-frame HYPER adapter; existing 70% crop/conf .5/imgsz800 are reused.

Stored images measure inference/contract only; never impersonate live captures.
OAK capture runs in .venv-oak; this model worker runs in .venv-yolo.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT/'reference/external/HYPER/src/perception/hyper_object_detection/models/best_track.pt'
FORK = {('allow','ban'): 'allow_left', ('ban','allow'): 'allow_right',
        ('allow','blank'): 'allow_left', ('blank','allow'): 'allow_right',
        ('ban','blank'): 'allow_right', ('blank','ban'): 'allow_left'}
MAP = {'Allow':'allow', 'Ban':'ban', 'Blank':'blank'}


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False)+'\n')
    tmp.replace(path)


def fork_decision(result):
    """HYPER main's same-row pair test and overlapping-box merge, no timeout default."""
    candidates = []
    if result.boxes is None:
        return None
    for box in result.boxes:
        sign = MAP.get(str(result.names[int(box.cls[0])]))
        if sign is None: continue
        x1,y1,x2,y2 = box.xyxy[0].tolist()
        candidates.append({'sign': sign, 'x': (x1+x2)/2, 'y': (y1+y2)/2,
                           'w': abs(x2-x1), 'h': abs(y2-y1), 'conf': float(box.conf[0])})
    merged = []
    for c in sorted(candidates, key=lambda c: c['x']):
        if merged and abs(c['x']-merged[-1]['x']) <= .5*min(c['w'], merged[-1]['w']):
            if c['conf'] > merged[-1]['conf']: merged[-1] = c
        else: merged.append(c)
    # Two visible panels can actually be panels2+3 with panel1 off-screen.
    # Preserve the HYPER pair mapping but require all three same-row slots.
    if len(merged)<3:return None
    for left,right in zip(merged, merged[1:]):
        mh, mw = (left['h']+right['h'])/2, (left['w']+right['w'])/2
        if (mh > 0 and mw > 0 and right['h'] > 0 and
                abs(left['y']-right['y']) <= .8*mh and
                .5 <= left['h']/right['h'] <= 2 and right['x']-left['x'] <= 6*mw):
            sign = FORK.get((left['sign'],right['sign']))
            third=merged[merged.index(right)+1] if merged.index(right)+1<len(merged) else None
            if third is None or third['h']<=0:return None
            if (abs(right['y']-third['y'])>.8*(right['h']+third['h'])/2 or
                    not .5<=right['h']/third['h']<=2 or
                    third['x']-right['x']>6*(right['w']+third['w'])/2):return None
            if sign: return {'sign': sign, 'confidence': min(left['conf'],right['conf']), 'panels': [left,right], 'three_panels_seen':True}
            return None
    return None


class Adapter:
    def __init__(self, model_path=MODEL):
        from ultralytics import YOLO
        self.model = YOLO(str(model_path))
        self.hash = hashlib.sha256(Path(model_path).read_bytes()).hexdigest()
        self.source = uuid.uuid4().hex

    def infer(self, image, *, frame_id, capture_at, target, epoch, input_kind, source=None):
        import cv2
        from hyper_signal_offline import decide
        begin = time.monotonic()
        h,w = image.shape[:2]
        fraction=1. if target=='finish_fork' else .7
        cw,ch = int(w*fraction),int(h*fraction)
        ox,oy = (w-cw)//2,(h-ch)//2
        crop = cv2.resize(image[oy:oy+ch,ox:ox+cw], (1280,720))
        result = self.model(crop, conf=.5, imgsz=800, verbose=False)[0]
        sign,best,boxes = decide(result,1280)
        fork = fork_decision(result) if target == 'finish_fork' else None
        if fork: sign,best = fork['sign'], {'conf':fork['confidence']}
        end = time.monotonic()
        original_boxes = [[b[0], b[1], ox+b[2]*cw/1280, oy+b[3]*ch/720,
                           ox+b[4]*cw/1280, oy+b[5]*ch/720] for b in boxes]
        fresh = capture_at is not None and 0 <= end-capture_at <= .6
        return {'source': source or self.source, 'frame_id': frame_id, 'capture_at': capture_at,
                'decision_at': end, 'target': target, 'epoch': epoch, 'sign': sign,
                'confidence': 0. if best is None else best['conf'], 'valid': fresh and input_kind == 'live',
                'input_kind': input_kind, 'inference_s': end-begin,
                'frame_age_s': None if capture_at is None else end-capture_at,
                'model_sha256': self.hash, 'crop_fraction': fraction, 'imgsz': 800,
                'centre_selection_frame': 'cropped_resized_1280x720',
                'boxes_crop': boxes, 'boxes_original': original_boxes, 'fork': fork}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    modes = p.add_mutually_exclusive_group(required=True)
    modes.add_argument('--images', nargs='+', type=Path)
    modes.add_argument('--frames', type=Path, help='OAK latest frame directory')
    p.add_argument('--request', type=Path, help='mission target/epoch request JSON')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--model', type=Path, default=MODEL)
    a = p.parse_args()
    if a.frames and not a.request: p.error('--frames needs --request')
    import cv2
    adapter = Adapter(a.model)
    if a.images:
        with a.output.open('x') as out:
            for i,path in enumerate(a.images):
                img = cv2.imread(str(path))
                if img is None: raise ValueError('unreadable image')
                event = adapter.infer(img, frame_id=i, capture_at=None, target='light_1', epoch=0, input_kind='recorded')
                event['file'] = str(path)
                out.write(json.dumps(event, allow_nan=False)+'\n'); out.flush()
                print(json.dumps({k:event[k] for k in ('file','sign','inference_s','frame_age_s','valid')}), flush=True)
        return 0
    seen = None
    print('MODEL_READY '+adapter.hash, flush=True)
    while True:
        try:
            req = json.loads(a.request.read_text())
            frame = json.loads((a.frames/'latest.json').read_text())
            identity = (frame['source'],frame['frame_id'])
            if identity == seen or not 0 <= time.monotonic()-frame['capture_at'] <= .6:
                time.sleep(.01); continue
            seen = identity
            if frame['capture_at'] < req['requested_at']:
                continue
            img = cv2.imread(str(a.frames/frame['file']))
            if img is None: continue
            result = adapter.infer(img, frame_id=frame['frame_id'], capture_at=frame['capture_at'],
                                   target=req['target'], epoch=req['epoch'], input_kind='live', source=frame['source'])
            # A target/epoch change during CPU inference invalidates the whole result.
            if json.loads(a.request.read_text()) != req: continue
            atomic_json(a.output, result)
        except FileNotFoundError:
            time.sleep(.02)


if __name__ == '__main__':
    try: raise SystemExit(main())
    except KeyboardInterrupt: raise SystemExit(130)
