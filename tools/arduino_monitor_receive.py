#!/usr/bin/env python3
"""Receive the reviewed ASCII MONITOR protocol. No serial write/heartbeat path.

Opening a USB serial device may reset it. Use only after reviewing the uploaded
firmware and current conditions. This tool never arms, drives, or uploads.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import select
import signal
import time
from gnss_record import open_serial
from ultrasonic_telemetry import parse_test_line

PORT_NAME = 'usb-Arduino__www.arduino.cc__0042_95431353338351A01190-if00'
PATTERN = re.compile(r'^\[MONITOR\] US=([0-9.]+)/([0-9.]+)/([0-9.]+) cm \| ENC=(-?\d+) \| POT=(\d+) \| BLOCK=([01]) \| ESTOP=([01])$')


def parse_monitor(line):
    match = PATTERN.fullmatch(line.strip())
    if not match:
        return None
    try:
        distances = [float(match[i]) for i in (1,2,3)]
        adc = int(match[5])
        count = int(match[4])
    except ValueError:
        return None
    if not all(math.isfinite(v) and 0 <= v <= 999 for v in distances) or not 0 <= adc <= 1023 or not -2**31 <= count < 2**31:
        return None
    return {'ultrasonic_cm_raw':distances,
            'ultrasonic_cm':[None if v == 999 else v for v in distances],
            'no_echo':[v == 999 for v in distances],
            'encoder_count':count,'steering_adc':adc,
            'front_blocked':match[6]=='1','manual_estop':match[7]=='1'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--reviewed-source',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=600)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 900:
        parser.error('duration must be 1..900 seconds')
    source = args.reviewed_source.read_bytes()
    port = Path('/dev/serial/by-id')/PORT_NAME
    if not port.is_char_device():
        parser.error('Expected Arduino by-id character device is missing')
    args.output_dir.mkdir(parents=True,exist_ok=False)
    out = args.output_dir
    (out/'provided_source.ino').write_bytes(source)
    (out/'executed_receiver.py').write_bytes(Path(__file__).read_bytes())
    (out/'ultrasonic_telemetry.py').write_bytes(Path(__file__).with_name('ultrasonic_telemetry.py').read_bytes())
    stop = [False]
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_:stop.__setitem__(0,True))
    report = {'status':'starting','started_utc':datetime.now(timezone.utc).isoformat(),
              'port':str(port),'baud':115200,'bytes_received':0,'monitor_samples':0,
              'bytes_transmitted_by_program':0,'provided_source_sha256':hashlib.sha256(source).hexdigest(),
              'source_readback_verified':False,
              'conditions':'User reports motor power ON for hub supply. Provided source reviewed; actual board firmware is not verified. No commands are sent.',
              'limitations':['999 means no echo in provided firmware, not clear space.',
                  'Host tty is left raw/echo-disabled/HUPCL-disabled; restoring echo could feed telemetry back as commands.',
                  'No explicit DTR/RTS, upload, heartbeat or motor command; opening USB serial may reset the board.']}
    def save():
        tmp=out/'report.tmp';tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');tmp.replace(out/'report.json')
    fd = None
    start=time.monotonic()
    save()
    try:
        fd,_=open_serial(str(port),115200)
        report['status']='receiving';save()
        pending=b''
        with (out/'raw_rx.bin').open('xb') as raw, (out/'samples.jsonl').open('x',buffering=1) as rows:
            while not stop[0] and time.monotonic()-start < args.duration:
                if not select.select([fd],[],[],.1)[0]:continue
                chunk=os.read(fd,8192)
                if not chunk:raise OSError('Arduino disconnected')
                raw.write(chunk);raw.flush();report['bytes_received']+=len(chunk)
                report['last_receive_utc']=datetime.now(timezone.utc).isoformat()
                save()
                pending+=chunk
                while b'\n' in pending:
                    line,pending=pending.split(b'\n',1)
                    decoded=line.decode('ascii','replace')
                    value=parse_monitor(decoded) or parse_test_line(decoded)
                    if value is not None:
                        row={'host_utc':datetime.now(timezone.utc).isoformat(),'elapsed_s':time.monotonic()-start,**value}
                        rows.write(json.dumps(row)+'\n')
                        report['monitor_samples']+=1
                        report.update(last_sample_utc=row['host_utc'],latest=value)
                        save()
                if len(pending)>8192:raise ValueError('Unterminated serial line exceeds 8192 bytes')
        report['status']='stopped' if stop[0] else 'completed' if report['monitor_samples'] else 'no_monitor_output'
    except Exception as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
    finally:
        if fd is not None:os.close(fd)
        report.update(ended_utc=datetime.now(timezone.utc).isoformat(),elapsed_s=time.monotonic()-start)
        save();print(json.dumps(report,ensure_ascii=False),flush=True)
    return 0 if report['status'] in ('stopped','completed') else 1


if __name__=='__main__':
    raise SystemExit(main())
