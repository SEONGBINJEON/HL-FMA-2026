import io, json, sys, tempfile, unittest
from contextlib import redirect_stdout
from datetime import datetime, timezone, timedelta
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
import a1_steering_record as a1

T0 = datetime(2026, 9, 17, 5, 0, 0, tzinfo=timezone.utc)


def status(adc, steer=0, drive=0, boot=40, req=None):
    return {"boot_id": boot, "request_id": 1, "millis": 0, "encoder_count": 0, "steering_adc": adc, "state": 1,
            "reason": 9, "requested_drive": drive, "requested_steer": steer if req is None else req,
            "actual_drive": drive, "actual_steer": steer, "flags": 102}


def write_session(tmp, rows):
    """rows: list of (seconds_offset, status dict). 20Hz-like synthetic events."""
    d = Path(tmp); (d / 'events.jsonl').write_text(''.join(
        json.dumps({"host_utc": (T0 + timedelta(seconds=t)).isoformat(), "event": "status", "status": s}) + '\n'
        for t, s in rows) + json.dumps({"host_utc": T0.isoformat(), "event": "input_state", "phase": "ready"}) + '\n')
    return d


def run(argv):
    out = io.StringIO()
    with redirect_stdout(out):
        rc = a1.main(argv)
    return rc, out.getvalue()


class Tests(unittest.TestCase):
    def test_mark_accepts_settled_live_adc_and_records_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = [(i * 0.05, status(600 + (i % 3) - 1)) for i in range(100)]   # 5s, adc 599..601
            d = write_session(tmp, rows)
            now = (T0 + timedelta(seconds=5.0)).isoformat()
            rc, out = run(['mark', '--session', tmp, '--label', 'straight', '--wheel', 'left', '--angle-deg', '0.0',
                           '--now-utc', now, '--note', 'synthetic'])
            self.assertEqual(rc, 0, out); self.assertIn('MARKED straight', out)
            rec = json.loads((d / 'a1_marks.jsonl').read_text().splitlines()[-1])
            self.assertEqual(rec['adc']['median'], 600); self.assertEqual((rec['adc']['min'], rec['adc']['max']), (599, 601))
            self.assertGreaterEqual(rec['adc']['n'], 55); self.assertEqual(rec['angle_cdeg'], {'left': 0, 'right': None})
            self.assertFalse(rec['saturation_electrical']); self.assertFalse(rec['unstable']); self.assertEqual(rec['warnings'], [])
            self.assertEqual(rec['board_boot_ids'], [40]); self.assertIn('events_byte_range', rec)

    def test_mark_rejects_moving_stale_and_too_few(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = [(i * 0.05, status(600 + i, steer=-1 if 60 <= i < 70 else 0)) for i in range(100)]
            write_session(tmp, rows)
            now = (T0 + timedelta(seconds=5.0)).isoformat()
            rc, out = run(['mark', '--session', tmp, '--label', 'left_mid', '--wheel', 'left', '--angle-deg', '8', '--now-utc', now])
            self.assertEqual(rc, 1); self.assertIn('non-zero', out)                       # stick still active in window
            rc, out = run(['mark', '--session', tmp, '--label', 'left_mid', '--wheel', 'left', '--angle-deg', '8',
                           '--now-utc', (T0 + timedelta(seconds=9.0)).isoformat()])
            self.assertEqual(rc, 1); self.assertIn('stale', out)                          # board not live
            rc, out = run(['mark', '--session', tmp, '--label', 'left_mid', '--wheel', 'left', '--angle-deg', '8',
                           '--now-utc', now, '--window-s', '0.2'])
            self.assertEqual(rc, 1); self.assertIn('samples', out)                        # too few
            self.assertFalse((Path(tmp) / 'a1_marks.jsonl').exists())

    def test_saturation_mechanical_end_and_sign_warnings(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = [(i * 0.05, status(1023)) for i in range(80)]
            d = write_session(tmp, rows)
            now = (T0 + timedelta(seconds=4.0)).isoformat()
            rc, out = run(['mark', '--session', tmp, '--label', 'left_limit', '--wheel', 'both', '--angle-deg', '-16',
                           '--angle-deg-right', '15', '--mechanical-end', '--now-utc', now])
            self.assertEqual(rc, 0)
            rec = json.loads((d / 'a1_marks.jsonl').read_text())
            self.assertTrue(rec['saturation_electrical']); self.assertTrue(rec['mechanical_end'])
            self.assertTrue(any('saturation' in w for w in rec['warnings']))
            self.assertTrue(any('left +' in w for w in rec['warnings']))
            rc, out = run(['mark', '--session', tmp, '--label', 'left_limit', '--wheel', 'both', '--angle-deg', '16', '--now-utc', now])
            self.assertEqual(rc, 1); self.assertIn('--angle-deg-right', out)

    def test_sign_derivation_and_summary_draft_excludes_saturation(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = []
            t = 0.0
            for rep in range(4):                       # steer -1 raises ADC, +1 lowers it
                for i in range(12): rows.append((t, status(600 + i * 3, steer=-1))); t += 0.05
                for i in range(12): rows.append((t, status(636 - i * 3, steer=1))); t += 0.05
            for i in range(60): rows.append((t, status(700))); t += 0.05          # settled left_mid
            d = write_session(tmp, rows)
            sign = a1.derive_sign(a1.read_status_events(d))
            self.assertEqual(sign['steer_command_sign_for_adc_increase_candidate'], -1)
            self.assertEqual(sign['tally_by_actual_steer'][-1]['adc_up'], 4)
            now = (T0 + timedelta(seconds=t - 0.05)).isoformat()
            # default 3s window still touches the last steer=+1 sample => correctly rejected; use 2.5s
            self.assertEqual(run(['mark', '--session', tmp, '--label', 'left_mid', '--wheel', 'left', '--angle-deg', '8.5', '--now-utc', now])[0], 1)
            self.assertEqual(run(['mark', '--session', tmp, '--label', 'left_mid', '--wheel', 'left', '--angle-deg', '8.5', '--now-utc', now, '--window-s', '2.5'])[0], 0)
            # add a saturated mark by appending statuses
            with open(d / 'events.jsonl', 'a') as f:
                for i in range(60):
                    f.write(json.dumps({"host_utc": (T0 + timedelta(seconds=t + i * 0.05)).isoformat(), "event": "status", "status": status(1022)}) + '\n')
            now2 = (T0 + timedelta(seconds=t + 59 * 0.05)).isoformat()
            self.assertEqual(run(['mark', '--session', tmp, '--label', 'left_limit', '--wheel', 'left', '--angle-deg', '16', '--now-utc', now2, '--window-s', '2.5'])[0], 0)
            rc, out = run(['summary', '--session', tmp])
            self.assertEqual(rc, 0); self.assertIn('sign candidate: -1', out)
            draft = json.loads((d / 'a1_calibration_draft.json').read_text())
            self.assertEqual([p['adc'] for p in draft['points_by_wheel']['left']], [700])   # saturated 1022 excluded
            self.assertEqual(draft['points_by_wheel']['right'], [])
            self.assertEqual(len(draft['excluded_saturated_marks']), 1)
            self.assertFalse(draft['applied_to_runtime']); self.assertEqual(draft['status'], 'draft_from_marks')

    def test_ambiguous_sign_gives_none(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = []; t = 0.0
            for rep in range(4):
                up = rep % 2 == 0
                for i in range(12): rows.append((t, status(600 + (i if up else -i) * 3, steer=-1))); t += 0.05
                rows.append((t, status(600))); t += 0.05
            d = write_session(tmp, rows)
            self.assertIsNone(a1.derive_sign(a1.read_status_events(d))['steer_command_sign_for_adc_increase_candidate'])

    def test_real_session_sign_runs_without_error(self):
        real = Path(__file__).parents[1] / 'data' / 'joystick_continuous_20260916_034201'
        if not (real / 'events.jsonl').is_file():
            self.skipTest('real session not present')
        sign = a1.derive_sign(a1.read_status_events(real))
        self.assertIn('steer_command_sign_for_adc_increase_candidate', sign)
        self.assertGreater(sign['runs_used'], 0)


if __name__ == '__main__':
    unittest.main()
