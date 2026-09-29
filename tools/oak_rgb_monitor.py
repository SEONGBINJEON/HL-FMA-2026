#!/usr/bin/env python3
"""Finite OAK RGB capture for the local sensor monitor; no Arduino access."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time


def now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-id', required=True)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=900)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 3600:
        parser.error('duration must be 1..3600 seconds')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    import cv2
    import depthai as dai
    stop = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    output = args.output_dir
    source = Path(__file__).read_bytes()
    (output/'executed_oak_rgb_monitor.py').write_bytes(source)
    report = {'started_utc': now(), 'status': 'connecting', 'device_id': args.device_id,
              'sdk_version': dai.__version__, 'opencv_version': cv2.__version__,
              'source_sha256': hashlib.sha256(source).hexdigest(), 'argv': sys.argv,
              'requested_fps': 15, 'requested_size': [640, 480], 'frames': 0,
              'limitations': ['RGB only; no depth/calibration/vehicle control.',
                  'MJPEG video has nominal 15 fps; frames.jsonl retains actual host/device timestamps.',
                  'latest.jpg is a replaceable live preview; rgb.avi is the session recording.']}
    writer = None
    started = time.monotonic()
    atomic_json(output/'report.json', report)
    try:
        matches = [d for d in dai.Device.getAllAvailableDevices() if d.deviceId == args.device_id]
        if len(matches) != 1:
            raise RuntimeError('Expected exactly one accessible OAK with specified device ID')
        with dai.Device(matches[0]) as device:
            report.update(device_name=device.getDeviceName(), usb_speed=str(device.getUsbSpeed()),
                          camera_features=[str(x) for x in device.getConnectedCameraFeatures()])
            with dai.Pipeline(device) as pipeline:
                camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
                queue = camera.requestOutput((640, 480), type=dai.ImgFrame.Type.BGR888p,
                                             fps=15).createOutputQueue(maxSize=2, blocking=False)
                pipeline.start()
                last = time.monotonic()
                last_publish = 0.
                first_received = None
                previous_sequence = None
                report.update(status='waiting_for_frames', sequence_gaps=0)
                atomic_json(output/'report.json', report)
                with (output/'frames.jsonl').open('x', buffering=1) as journal:
                    while pipeline.isRunning() and not stop[0] and time.monotonic()-started < args.duration:
                        msg = queue.tryGet()
                        current = time.monotonic()
                        if msg is None:
                            if current-last > 5:
                                raise RuntimeError('No new camera frame for more than 5 seconds')
                            time.sleep(.005)
                            continue
                        last = current
                        first_received = current if first_received is None else first_received
                        frame = msg.getCvFrame()
                        if writer is None:
                            h, w = frame.shape[:2]
                            writer = cv2.VideoWriter(str(output/'rgb.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 15, (w,h))
                            if not writer.isOpened():
                                raise RuntimeError('MJPEG recording could not be opened')
                            report['observed_size'] = [w,h]
                            if not cv2.imwrite(str(output/'first_frame.jpg'), frame):
                                raise RuntimeError('Could not save first frame')
                        writer.write(frame)
                        seq = msg.getSequenceNum()
                        if previous_sequence is not None and seq > previous_sequence+1:
                            report['sequence_gaps'] += seq-previous_sequence-1
                        previous_sequence = seq
                        row = {'host_utc': now(), 'host_monotonic_s': current,
                               'device_timestamp_s': msg.getTimestampDevice().total_seconds(),
                               'sequence': seq, 'recorded_frame_index': report['frames']}
                        journal.write(json.dumps(row)+'\n')
                        report['frames'] += 1
                        report.update(status='streaming', last_frame_utc=row['host_utc'],
                                      received_fps=(report['frames']-1)/(current-first_received) if current>first_received else None)
                        if current-last_publish >= .1:
                            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                            if not ok:
                                raise RuntimeError('JPEG encode failed')
                            temp = output/'latest.tmp'
                            temp.write_bytes(encoded.tobytes())
                            temp.replace(output/'latest.jpg')
                            atomic_json(output/'report.json', report)
                            last_publish = current
                report['status'] = 'stopped' if stop[0] else 'completed' if time.monotonic()-started >= args.duration else 'disconnected'
                pipeline.stop()
        code = 0 if report['frames'] and report['status'] in ('stopped','completed') else 1
    except Exception as exc:
        report.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        code = 1
    finally:
        if writer is not None:
            writer.release()
        report.update(ended_utc=now(), elapsed_s=time.monotonic()-started)
        atomic_json(output/'report.json', report)
        print(json.dumps(report, ensure_ascii=False), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
