#!/usr/bin/env python3
"""Local GET-only viewer of recorded sensor streams. Never opens a device port."""
import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import time
from urllib.parse import urlparse
from ultrasonic_telemetry import parse_test_line

NODE = re.compile(rb'^(?:S)?\s+theta:\s*([0-9]+\.[0-9]+) Dist:\s*([0-9]+\.[0-9]+) Q:\s*([0-9]+)\s*$')


def tail(path, size=262144):
    try:
        with path.open('rb') as f:
            start = max(0, path.stat().st_size-size)
            f.seek(start)
            rows = f.read(size).split(b'\n')
        # Drop partial first/last line. Only newline-terminated rows are shown.
        return rows[1:-1] if start else rows[:-1]
    except OSError:
        return []


def last_json(path):
    for line in reversed(tail(path, 32768)):
        try:
            return json.loads(line)
        except ValueError:
            continue
    return {}


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def age_utc(value):
    try:
        return max(0, (datetime.now(timezone.utc)-datetime.fromisoformat(value)).total_seconds())
    except (TypeError, ValueError):
        return None


def state(folder):
    config = read_json(folder/'session.json')
    lidar = Path(config['lidar_folder']) if config.get('lidar_folder') else None
    points = []
    age = None
    if lidar:
        for line in tail(lidar/'stdout.bin'):
            m = NODE.fullmatch(line)
            if m:
                angle, distance, quality = float(m[1]), float(m[2]), int(m[3])
                if 0 <= angle <= 360 and distance > 0 and quality > 0:
                    points.append([angle, distance/1000])
        # Timestamp the received stream, not the file modification or request time.
        last = last_json(lidar/'host_chunks.jsonl')
        age = age_utc(last.get('host_utc'))
    imu_dir = Path(config.get('imu_folder', str(folder/'imu')))
    imu = last_json(imu_dir/'samples.jsonl')
    camera_dir = Path(config.get('camera_folder', str(folder/'oak')))
    camera = read_json(camera_dir/'report.json')
    arduino_dir = Path(config.get('arduino_folder', str(folder/'arduino')))
    raw = arduino_dir/'raw_rx.bin'
    lines = tail(raw, 4096)
    ultrasonic = parse_test_line(lines[-1].decode('ascii', 'replace')) if lines else None
    try:
        raw_age = max(0, time.time()-raw.stat().st_mtime)
    except OSError:
        raw_age = None
    sample = last_json(arduino_dir/'samples.jsonl')
    if sample:
        ultrasonic = sample
        raw_age = age_utc(sample.get('host_utc'))
    gps_dir = Path(config.get('gps_folder', str(folder/'gps_probe')))
    gps = read_json(gps_dir/'metadata.json')
    gps_sample = last_json(gps_dir/'gga.jsonl')
    gps_note = 'GPS 수신 대기'
    if gps.get('status') == 'no_data':
        gps_note = 'USB 인식 · 60초 수신 0바이트 · GPS 출력 미확인'
    elif gps_sample:
        gps_age = age_utc(gps_sample.get('host_utc'))
        gps_note = ('u-blox GGA 수신 중' if gps_age is not None and gps_age < 2 else 'u-blox 마지막 GGA (수신 종료/대기)')+' · '+('위치 미확정' if gps_sample.get('quality_code') == 0 else str(gps_sample.get('fix_status')))+' · 사용 위성 '+str(gps_sample.get('satellites'))+' · RTK 보정 미사용'
    arduino_present = any(Path('/dev/serial/by-id').glob('*Arduino*95431353338351A01190*'))
    try:
        controller_present = 'Microsoft X-Box 360 pad' in Path('/proc/bus/input/devices').read_text()
    except OSError:
        controller_present = False
    return {'server_utc':datetime.now(timezone.utc).isoformat(), 'session':folder.name,
            'lidar':{'points':points if age is not None and age<2 else [], 'age_s':age,
                     'plotted_points':len(points), 'report':read_json(lidar/'metadata.json') if lidar else {}},
            'imu':{'sample':imu, 'age_s':age_utc(imu.get('host_utc')), 'report':read_json(imu_dir/'report.json')},
            'camera':{**camera, 'age_s':age_utc(camera.get('last_frame_utc'))},
            'arduino':('USB 연결 · ' if arduino_present else 'USB 미검출 · ') + config.get('arduino_note', '현재 펌웨어 미확인 · 직렬 포트 미개방'),
            'controller':('Linux 게임패드 인식 · ' if controller_present else '게임패드 미검출 · ') + '주행 연결 보류',
            'ultrasonic':{'sample':ultrasonic, 'age_s':raw_age,
                          'report':read_json(arduino_dir/'report.json'),
                          'time_basis':'host receipt' if sample else 'raw receive file modification; approximate'},
            'gps':{'probe':gps, 'sample':gps_sample, 'note':gps_note}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--port', type=int, default=8766)
    args = parser.parse_args()
    folder = args.session.resolve(strict=True)
    html = Path(__file__).with_suffix('.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            path = urlparse(self.path).path
            try:
                if path == '/':
                    data, mime = html, 'text/html; charset=utf-8'
                elif path == '/ultrasonic':
                    data, mime = Path(__file__).with_name('ultrasonic_monitor.html').read_bytes(), 'text/html; charset=utf-8'
                elif path == '/state':
                    data, mime = json.dumps(state(folder), ensure_ascii=False).encode(), 'application/json'
                elif path == '/frame.jpg':
                    config = read_json(folder/'session.json')
                    camera_dir = Path(config.get('camera_folder', str(folder/'oak')))
                    data, mime = (camera_dir/'latest.jpg').read_bytes(), 'image/jpeg'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control','no-store')
                self.send_header('X-Content-Type-Options','nosniff')
                self.end_headers()
                self.wfile.write(data)
            except (OSError, ValueError):
                self.send_error(503)

    server = ThreadingHTTPServer(('127.0.0.1',args.port), Handler)
    print(f'http://127.0.0.1:{args.port}',flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
