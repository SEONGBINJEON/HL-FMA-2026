"""Latest OAK frame publisher; retain raw JPEGs/index. No vehicle access."""
import argparse
import json
from pathlib import Path
import time
import uuid
from main_event_signal import atomic_json


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--duration',type=float,default=0,help='finite diagnostic capture; 0 until interrupted')
    a = p.parse_args()
    import cv2
    import depthai as dai
    a.output.mkdir(parents=True, exist_ok=False)
    source = uuid.uuid4().hex
    with (a.output/'frames.jsonl').open('x') as journal, dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        q = cam.requestOutput((1280,720), dai.ImgFrame.Type.BGR888i, fps=10).createOutputQueue(maxSize=1, blocking=False)
        pipeline.start()
        started=time.monotonic();last_report=started
        stats={'received':0,'published':0,'age_rejected':0,'empty_polls':0,'requested_size':[1280,720],'requested_fps':10}
        def status():
            atomic_json(a.output/'capture_status.json',{**stats,'updated_at':time.monotonic(),'elapsed_s':time.monotonic()-started})
        status()
        while not a.duration or time.monotonic()-started<a.duration:
            if time.monotonic()-last_report>=1.:
                status();last_report=time.monotonic()
            frame = q.tryGet()
            if frame is None:
                stats['empty_polls']+=1
                time.sleep(.005); continue
            # DepthAI host-synced timestamp age; never label pipe receipt as acquisition.
            age = (dai.Clock.now()-frame.getTimestamp()).total_seconds()
            stats.update(received=stats['received']+1,last_received_at=time.monotonic(),last_sequence=frame.getSequenceNum(),last_age_s=age)
            if not 0 <= age <= .6:
                stats['age_rejected']+=1;continue
            capture_at = time.monotonic()-age
            seq = frame.getSequenceNum(); name = f'{source}_{seq}.jpg'
            write_started=time.monotonic()
            if not cv2.imwrite(str(a.output/name), frame.getCvFrame(), [cv2.IMWRITE_JPEG_QUALITY,92]):
                raise OSError('JPEG write failed')
            record={'source':source,'frame_id':seq,'file':name,
                         'capture_at':capture_at,'published_at':time.monotonic(),
                         'time_basis':'depthai_host_synced_timestamp', 'input_kind':'live','receipt_age_s':age,'jpeg_s':time.monotonic()-write_started}
            journal.write(json.dumps(record)+'\n');journal.flush()
            atomic_json(a.output/'latest.json',record)
            stats.update(published=stats['published']+1,last_publish_at=record['published_at'],last_jpeg_s=record['jpeg_s'])
        status()


if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: raise SystemExit(130)
