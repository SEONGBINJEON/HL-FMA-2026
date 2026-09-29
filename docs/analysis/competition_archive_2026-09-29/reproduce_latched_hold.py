"""Synthetic isolation of the observed latch; not a full race-log replay."""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))
from main_event_core import Gate, Mission


def scenario(latch):
    mission = Mission([Gate('light_1', 'signal', 10., 10.275, allowed=('green',))], 'offline', 0)
    result = []
    anchor_encoder = None
    held_changed = False
    for i in range(15):
        now = 100. + i * .1
        encoder = 1000 if i < 9 else 1001
        board = {'boot': 70, 'request': i, 'received_at': now, 'encoder': encoder,
                 'requested_drive': 0, 'fault': False, 'state': 'ARMED'}
        if mission.holding and mission.stationary.stopped_since is not None and anchor_encoder is None:
            anchor_encoder = encoder
        # Same encoder-change latch as runtime's held-anchor path.
        if latch and anchor_encoder is not None and encoder != anchor_encoder:
            held_changed = True
        if held_changed:
            mission.recovery_required = True
            mission.recovery_cause = 'held_vehicle_moved'
        signal = {'target': 'light_1', 'epoch': 0, 'capture_at': now-.02,
                  'decision_at': now-.01, 'valid': True, 'confidence': .9,
                  'source': 'synthetic', 'frame_id': i, 'sign': 'red' if i < 9 else 'green'}
        decision = mission.step(s=9.9, now=now, board=board, quality_ok=True,
                                heading_ok=not held_changed, aligned=True, signal=signal,
                                sensor_required=False, position_ok=True, resume_ready=not held_changed)
        result.append({'sample': i, 'encoder': encoder, 'sign': signal['sign'],
                       'reason': decision['reason'], 'drive': decision['drive']})
    return result


control, latched = scenario(False), scenario(True)
assert any(r['drive'] for r in control[9:]), control
assert all(not r['drive'] and r['reason']=='pose_recovery_required' for r in latched[9:]), latched
report = {'input_kind': 'synthetic', 'hardware_access': False,
          'claim': 'A latched held_vehicle_moved recovery condition blocks subsequent valid green votes.',
          'not_claimed': 'Exact replay of the historical anchor, or proof of physical vehicle movement.',
          'control': control, 'latched': latched}
(Path(__file__).parent / 'latch_reproduction.json').write_text(json.dumps(report, indent=2)+'\n')
print('PASS: ordinary red->green resumes; held encoder-change latch prevents resume despite fresh green.')
