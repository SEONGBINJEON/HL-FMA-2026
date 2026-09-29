"""Read-only post-competition audit. No device, network or credential access."""
from pathlib import Path
from collections import Counter
from datetime import datetime
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
SESSION = ROOT / 'data/me_20260920_151234_8ee5d5'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    with path.open() as stream:
        for line in stream:
            yield json.loads(line)


def main():
    manifest_path = ROOT / 'docs/handoff/main_event_release_20260920.json'
    manifest = json.loads(manifest_path.read_text())
    hashes, mismatches, missing, excluded = {}, [], [], []
    for name, expected in manifest['sha256'].items():
        if 'ntrip.local' in name or '.env' in Path(name).name:
            excluded.append(name)
            continue
        path = ROOT / name
        if not path.is_file():
            missing.append(name)
            continue
        actual = sha(path)
        hashes[name] = actual
        if actual != expected:
            mismatches.append(name)

    timeline, count, status_by_sec = [], Counter(), {}
    relevant = {'explicit_start', 'bootstrap', 'mission_decision',
                'mission_window_advanced', 'gps_continuity_reacquired',
                'nonprogress_watch', 'runtime_exception'}
    for item in rows(SESSION / 'vehicle/events.jsonl'):
        event = item['event']
        count[event] += 1
        if event in relevant:
            timeline.append({k: v for k, v in item.items()
                             if k not in ('heading_trail', 'fix')})
        if event == 'status' and '2026-09-20T15:18:00' <= item['host_kst'] < '2026-09-20T15:18:13':
            values = status_by_sec.setdefault(item['host_kst'][:19], [])
            s = item['status']
            values.append([s['encoder_count'], s['requested_drive'],
                           s['actual_drive'], s['drive_pwm']])
    board_summary = {t: {'encoder_min': min(x[0] for x in a),
                         'encoder_max': max(x[0] for x in a),
                         'commands': sorted(set(tuple(x[1:]) for x in a))}
                     for t, a in status_by_sec.items()}
    gga = [r for r in rows(SESSION / 'rtk/gga.jsonl')
           if '2026-09-20T06:18:00' <= r['host_utc'] < '2026-09-20T06:18:09']
    times = [datetime.fromisoformat(r['host_utc']).timestamp() for r in gga]
    frames = list(rows(SESSION / 'frames/frames.jsonl'))
    selected_frames = []
    for mono, note in [(218911.902679673, 'initial_signal_wait_red_visually_observed'),
                       (218919.010971082, 'recovery_latch_red_visually_observed'),
                       (218941., 'about_30s_after_stop_red_visually_observed'),
                       (218972., 'about_60s_after_stop_green_visually_observed')]:
        row = min(frames, key=lambda r: abs(r['capture_at'] - mono))
        path = SESSION / 'frames' / row['file']
        selected_frames.append({**row, 'path': str(path.relative_to(ROOT)),
                                'sha256': sha(path), 'review_note': note})
    state = json.loads((SESSION / 'state.json').read_text())
    inventory = {}
    for name in ('tools', 'tests', 'scripts', 'firmware', 'docs', 'data',
                 'logs', 'backups', 'reference', 'EX_Data'):
        files = [p for p in (ROOT / name).rglob('*') if p.is_file() and not p.is_symlink()]
        inventory[name] = {'file_count': len(files), 'bytes': sum(p.stat().st_size for p in files)}
    report = {
        'recorded_kst': datetime.now().astimezone().isoformat(),
        'hardware_access': False, 'credentials_read': False,
        'session': str(SESSION.relative_to(ROOT)),
        'release': {'id': manifest['release_id'], 'manifest_sha256': sha(manifest_path),
                    'declared_files': len(manifest['sha256']), 'checked_files': len(hashes),
                    'mismatches': mismatches, 'missing': missing, 'excluded': excluded,
                    'race_ready': manifest['race_ready']},
        'event_counts': dict(count), 'timeline': timeline,
        'board_near_lock': board_summary,
        'gga_near_lock': {'count': len(gga), 'status_counts': dict(Counter(r['fix_status'] for r in gga)),
                          'max_interval_s': max(b-a for a, b in zip(times, times[1:]))},
        'selected_frames': selected_frames,
        'final_state': {k: state.get(k) for k in ('phase', 'reason', 'fault', 'completed',
                                                'index', 'progress', 'exit_code', 'ended_utc',
                                                'stop_ack', 'stationary_observed', 'process_released')},
        'folder_inventory': inventory,
        'limitations': ['No historical per-frame inference ledger was found; signal.json is the final overwritten result.',
                       'The exact runtime held anchor and recovery_cause were not logged at the transition.',
                       'GNSS log receipt continuity does not itself prove each runtime poll had fresh data.',
                       'Visual green in a saved frame does not prove three accepted green votes.'],
    }
    (OUT / 'evidence.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    (OUT / 'release_hashes.json').write_text(json.dumps(hashes, indent=2)+'\n')
    print(json.dumps({'release': report['release'], 'session': report['session'],
                      'mission_transitions': [x for x in timeline if x['event']=='mission_decision'],
                      'gga_near_lock': report['gga_near_lock']}, ensure_ascii=False, indent=2))
    if missing or mismatches or excluded:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
