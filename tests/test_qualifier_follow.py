"""Device-free bundle for tools/qualifier_follow.py (2026-09-19 Claude5): course selection/slicing, entry
range rejoin, heading bootstrap math, adaptive lookahead, FLOAT default-off, encoder-freeze stop, RTK
freshness refusal, check-only CLI for both courses and one dry-run (manual heading) end-to-end on a
synthetic live GGA stream. Nothing here validates the vehicle."""
import json, math, sys, threading, time, unittest, tempfile, io, contextlib
from datetime import datetime, timezone
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT / 'tools'))
import qualifier_follow as qf, qualifier_route as qr
from path_follow_inputs import rear_axle_points

ROUTE = ROOT / qf.DEFAULT_ROUTE


def load_points():
    t = qr.load_track_csv('c', ROUTE); meta = json.loads(ROUTE.with_suffix('.meta.json').read_text())
    return rear_axle_points(t.points, meta['reference'], .315), meta


class Pose:
    def __init__(self, x, y, yaw): self.x, self.y, self.yaw = x, y, yaw


class CourseSlicing(unittest.TestCase):
    def setUp(self): self.full, self.meta = load_points()
    def spec(self, course, **kw):
        a = qf.build_parser().parse_args(['--course', course, '--rtk-session', 'x', '--output-dir', 'y'] + sum(([k, str(v)] for k, v in kw.items()), []))
        return qf.course_spec(a)
    def test_s_course_slice_and_entry(self):
        spec = self.spec('s'); pts, fidx = qf.slice_course(self.full, spec)
        self.assertEqual((fidx[0], fidx[-1], len(pts)), (105, 182, 78)); self.assertEqual(spec['lead_in_m'], 0.)
        allowed = qf.allowed_entry_indices(fidx, spec)
        self.assertEqual([fidx[i] for i in allowed], list(range(105, 141)))
        self.assertAlmostEqual(qr.cumulative_s(pts)[-1], 137.0 - 77.9, delta=.3)
    def test_bend_course_lead_in_option(self):
        spec = self.spec('bend', **{'--lead-in-m': 8, '--lane': 'common'}); pts, fidx = qf.slice_course(self.full, spec)
        lead = [p for p in pts if p.source == 'lead_in']; self.assertEqual(len(lead), 16); self.assertEqual(fidx[:2], ['lead:-16', 'lead:-15'])
        a, b = self.full[0], self.full[1]; hd = math.atan2(b.y - a.y, b.x - a.x)
        self.assertAlmostEqual(math.hypot(lead[0].x - a.x, lead[0].y - a.y), 8., places=6)
        self.assertAlmostEqual(qr.wrap(math.atan2(a.y - lead[0].y, a.x - lead[0].x) - hd), 0., places=6)
        allowed = qf.allowed_entry_indices(fidx, spec); self.assertEqual(fidx[allowed[0]], 'lead:-16'); self.assertEqual(fidx[allowed[-1]], 12)
        self.assertEqual(fidx[-1], 50); self.assertEqual(self.spec('bend')['lead_in_m'], 0.)
    def test_bend_lanes_routes_and_seams(self):
        spec = self.spec('bend'); lanes = qf.load_lane_tracks(spec, .315); self.assertEqual(set(lanes), {'left', 'right'})
        routes = qf.build_routes(self.full, spec, lanes); self.assertEqual(set(routes), {'left', 'right', 'common'})
        self.assertEqual(routes['common']['lane_points'], 0); self.assertEqual(len(routes['common']['points']), 51)
        for name in ('left', 'right'):
            r = routes[name]; self.assertEqual(r['lane_points'], len(lanes[name]['points']))
            self.assertLess(r['seam']['gap_m'], 2.); self.assertLess(max(abs(r['seam']['turn_lane_to_seam_deg']), abs(r['seam']['turn_seam_to_common_deg'])), 10.)  # recorded connection, no sharp kink
            self.assertEqual(r['labels'][r['lane_points']], 0); self.assertEqual(r['labels'][-1], 50)
            self.assertTrue(all(i in r['allowed'] for i in range(r['lane_points'])))  # whole lane is an entry candidate
        self.assertEqual(set(qf.build_routes(self.full, spec, lanes, 20)), {'common'})  # resume -> common only
    def test_lane_choice(self):
        spec = self.spec('bend'); lanes = qf.load_lane_tracks(spec, .315); routes = qf.build_routes(self.full, spec, lanes)
        L, R = lanes['left']['points'], lanes['right']['points']
        def pose_on(pts, i): return Pose(pts[i].x, pts[i].y, qr.path_heading_at(pts, i))
        self.assertEqual(qf.choose_route(routes, pose_on(L, 40), 5., 90.)[0], 'left')
        self.assertEqual(qf.choose_route(routes, pose_on(R, 40), 5., 90.)[0], 'right')
        name, i, d, err = qf.choose_route(routes, pose_on(self.full, 6), 5., 90.); self.assertEqual(name, 'common'); self.assertEqual(routes[name]['labels'][i], 6)
        mid = Pose((L[40].x + R[40].x) / 2, (L[40].y + R[40].y) / 2, qr.path_heading_at(L, 40))  # between the lanes: refuse
        with self.assertRaises(ValueError): qf.choose_route(routes, mid, 5., 90.)
        only_left = qf.build_routes(self.full, self.spec('bend', **{'--lane': 'left'}), lanes); self.assertEqual(set(only_left), {'left'})
        self.assertEqual(qf.choose_route(only_left, mid, 5., 90.)[0], 'left')
        rev = Pose(L[40].x, L[40].y, qr.path_heading_at(L, 40) + math.pi)  # facing the wrong way: nothing
        with self.assertRaises(ValueError): qf.choose_route(routes, rev, 5., 90.)
        with self.assertRaises(ValueError): self.spec('s', **{'--lane': 'left'})
    def test_resume_index_single_segment(self):
        spec = self.spec('s', **{'--resume-index': 170}); pts, fidx = qf.slice_course(self.full, spec)
        self.assertEqual([fidx[i] for i in qf.allowed_entry_indices(fidx, spec, 170)], [170])
    def test_invalid_specs(self):
        for kw in ({'--course-start': 190}, {'--entry-to': 182}, {'--resume-index': 50}, {'--lead-in-m': -1}):
            with self.assertRaises(ValueError): self.spec('s', **kw)
    def test_entry_refuses_other_course(self):
        spec = self.spec('s'); pts, fidx = qf.slice_course(self.full, spec); allowed = qf.allowed_entry_indices(fidx, spec)
        bend = self.full[5]  # vehicle standing on the bend course, aligned with it
        with self.assertRaises(ValueError):
            qf.select_entry(pts, allowed, Pose(bend.x, bend.y, qr.path_heading_at(self.full, 5)), 5., 90.)
        # standing 1 m left of S file idx 120 with the local path heading -> that segment (or its neighbour), not idx 114
        ph = qr.path_heading_at(self.full, 120); p = self.full[120]
        i, d, err = qf.select_entry(pts, allowed, Pose(p.x - math.sin(ph), p.y + math.cos(ph), ph), 5., 90.)
        self.assertIn(fidx[i], (119, 120)); self.assertAlmostEqual(d, 1., delta=.15); self.assertLess(math.degrees(err), 15)
        # mid-course (file idx 160) is not an allowed entry without --resume-index
        p = self.full[160]; ph = qr.path_heading_at(self.full, 160)
        with self.assertRaises(ValueError): qf.select_entry(pts, allowed, Pose(p.x, p.y, ph), 5., 90.)
        self.assertEqual(fidx[qf.select_entry(pts, qf.allowed_entry_indices(fidx, spec, 160), Pose(p.x, p.y, ph), 5., 90.)[0]], 160)


class BootstrapHeading(unittest.TestCase):
    def test_forward_window_only(self):
        fixes = [(i * .2, 0.05 * math.sin(i), 100 + i * .2) for i in range(9)]  # 1.6 m east, small scatter
        yaw, dist, dev = qf.bootstrap_heading(fixes, 1., .3)
        self.assertAlmostEqual(math.degrees(yaw), 0., delta=3); self.assertAlmostEqual(dist, 1.6, delta=.05); self.assertLess(dev, .1)
    def test_reverse_gives_reverse_heading(self):
        yaw, _, _ = qf.bootstrap_heading([(-i * .3, 0, i) for i in range(6)], 1., .3); self.assertAlmostEqual(abs(math.degrees(yaw)), 180., delta=1)
    def test_insufficient_or_crooked(self):
        with self.assertRaises(ValueError): qf.bootstrap_heading([(0, 0, 0), (.2, 0, 1), (.4, 0, 2)], 1., .3)
        with self.assertRaises(ValueError): qf.bootstrap_heading([(0, 0, 0), (.6, .8, 1), (1.5, 0, 2)], 1., .3)
        with self.assertRaises(ValueError): qf.bootstrap_heading([(0, 0, 0), (1.5, 0, 1)], 1., .3)


class PlannedStop(unittest.TestCase):
    def test_trim_end_and_along_track(self):
        full, meta = load_points(); pts = full[0:69]; labels = list(range(69)); s = qr.cumulative_s(pts)
        tp, tl = qf.trim_end(pts, labels, 1.0)
        self.assertAlmostEqual(qr.cumulative_s(tp)[-1], s[-1] - 1.0, places=6); self.assertEqual(tl[-1], '68-1.00m'); self.assertEqual(tp[-1].source, 'end_offset')
        self.assertEqual(qf.trim_end(pts, labels, 0.)[0], pts)
        with self.assertRaises(ValueError): qf.trim_end(pts, labels, 1000.)
        mid = Pose((pts[10].x + pts[11].x) / 2, (pts[10].y + pts[11].y) / 2, 0.)
        self.assertAlmostEqual(qf.along_track(pts, 10, mid), (s[10] + s[11]) / 2, places=6)
    def test_control_letters(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / 'ctl'; f.write_text('c'); self.assertEqual(qf.control_file_poll(str(f)), ['c']); self.assertFalse(f.exists())
    def test_stop_at_requires_control_file_and_check_only(self):
        with self.assertRaises(ValueError):
            qf.main(['--course', 'bend', '--lane', 'left', '--course-end', '68', '--stop-at', 'left:52', '--rtk-session', 'x', '--output-dir', 'y', '--check-only'])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = qf.main(['--course', 'bend', '--lane', 'left', '--course-end', '68', '--end-offset-m', '1.0', '--stop-at', 'left:52', '--control-file', 'ctl',
                            '--rtk-session', 'x', '--output-dir', 'y', '--check-only'])
        out = json.loads(buf.getvalue().splitlines()[0]); self.assertEqual(code, 0)
        self.assertEqual(out['file_index_range'], [0, 68]); self.assertAlmostEqual(out['routes']['left']['length_m'], 89.15 + 0.65 + 50.9 - 1.0, delta=.3)
    def test_dry_run_planned_stop_hold_continue(self):
        """Synthetic 5 Hz GGA along S idx 168..182 at 2 m/s; planned stop at label 175 with 1 s min hold, 'c' written
        every second from t=3 s. Expects: hold entered, continue accepted, arrival. (No plant: the synthetic car keeps moving.)"""
        full, meta = load_points(); lat0, lon0 = meta['origin_lat'], meta['origin_lon']
        pts = full[168:183]; s = qr.cumulative_s(pts); ph0 = qr.path_heading_at(full, 168)
        def at(dist):
            i = max(k for k in range(len(s)) if s[k] <= dist); i = min(i, len(pts) - 2); t = (dist - s[i]) / max(1e-9, s[i + 1] - s[i])
            return pts[i].x + t * (pts[i + 1].x - pts[i].x), pts[i].y + t * (pts[i + 1].y - pts[i].y)
        with tempfile.TemporaryDirectory() as d:
            rtk = Path(d, 'rtk'); rtk.mkdir(); gga = rtk / 'gga.jsonl'; ctl = Path(d) / 'ctl'; stop = threading.Event()
            def writer():
                k = 0
                while not stop.is_set():
                    x, y = at(min(s[-1] + .6, k * .4))
                    row = {'host_utc': datetime.now(timezone.utc).isoformat(), 'fix_status': 'rtk_fixed', 'checksum_valid': True,
                           'differential_age_s': 1.0, 'latitude_deg': lat0 + y / 111320, 'longitude_deg': lon0 + x / (111320 * math.cos(math.radians(lat0)))}
                    with gga.open('a') as f: f.write(json.dumps(row) + '\n')
                    if k >= 15 and k % 5 == 0: ctl.write_text('c')
                    k += 1; time.sleep(.2)
            th = threading.Thread(target=writer, daemon=True); th.start(); time.sleep(.5)
            try:
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    code = qf.main(['--course', 's', '--resume-index', '168', '--rtk-session', str(rtk), '--output-dir', str(Path(d) / 'out'), '--dry-run',
                                    '--heading-mode', 'manual', '--initial-heading-deg', str(math.degrees(ph0)), '--max-seconds', '30',
                                    '--stop-at', '175', '--stop-offset-m', '0.5', '--stop-lead-m', '0.3', '--hold-min-s', '1', '--control-file', str(ctl)])
            finally: stop.set(); th.join()
            text = buf.getvalue(); final = json.loads([l for l in text.splitlines() if l.startswith('{')][-1])
        self.assertIn('PLANNED STOP', text); self.assertIn('CONTINUE', text)
        self.assertEqual(final['status'], 'arrived', final); ps = final['stats']['planned_stop']
        self.assertEqual(ps['label'], '175'); self.assertGreaterEqual(ps['hold_s'], 1.); self.assertIn('settle_overshoot_m', ps)


class Guards(unittest.TestCase):
    def test_straight_reached_and_control_file(self):
        self.assertTrue(qf.straight_reached(477, 477, 25)); self.assertTrue(qf.straight_reached(500, 477, 25)); self.assertFalse(qf.straight_reached(503, 477, 25))
        self.assertFalse(qf.straight_reached(None, 477, 25))
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / 'ctl'; self.assertEqual(qf.control_file_poll(str(f)), []); self.assertEqual(qf.control_file_poll(None), [])
            f.write_text('p\n'); self.assertEqual(qf.control_file_poll(str(f)), ['p']); self.assertFalse(f.exists())  # consumed once
            f.write_text('s'); self.assertEqual(qf.control_file_poll(str(f)), ['s']); self.assertTrue(f.exists())
            f.write_text('go'); self.assertEqual(qf.control_file_poll(str(f)), [])
    def test_heading_test_only_flag(self):
        a = qf.build_parser().parse_args(['--course', 'bend', '--lane', 'left', '--rtk-session', 'x', '--output-dir', 'y', '--heading-test-only'])
        self.assertTrue(a.heading_test_only); self.assertEqual(a.lane, 'left')
        with self.assertRaises(ValueError):
            qf.main(['--course', 'bend', '--rtk-session', 'x', '--output-dir', 'y', '--check-only', '--heading-test-only', '--heading-mode', 'manual', '--initial-heading-deg', '0'])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf): code = qf.main(['--course', 'bend', '--lane', 'left', '--course-end', '59', '--rtk-session', 'x', '--output-dir', 'y', '--check-only', '--heading-test-only'])
        out = json.loads(buf.getvalue().splitlines()[0]); self.assertEqual(code, 0); self.assertTrue(out['heading_test_only'])
        self.assertEqual(set(out['routes']), {'left'}); self.assertEqual(out['file_index_range'], [0, 59]); self.assertEqual(out['routes']['left']['points'], 122 + 60)
    def test_adaptive_lookahead(self):
        self.assertEqual(qf.adaptive_lookahead(2., 1.5, 6., .2), 2.); self.assertEqual(qf.adaptive_lookahead(2., 1.5, 6., -1.), 2.)
        self.assertAlmostEqual(qf.adaptive_lookahead(2., 1.5, 6., 2.), 3.); self.assertEqual(qf.adaptive_lookahead(2., 1.5, 6., 4.3), 6.)
    def test_encoder_freeze(self):
        class S: encoder_count = 100
        w = qf.EncoderWatch(3.); s = S()
        self.assertIsNone(w.check(s, True, 0.)); s.encoder_count = 150; self.assertIsNone(w.check(s, True, 1.))
        self.assertIsNone(w.check(s, True, 3.5)); self.assertIn('encoder frozen', w.check(s, True, 4.2) or '')
        self.assertIsNone(w.check(s, False, 10.)); self.assertIsNone(w.check(s, True, 11.))  # not driving resets; a new window starts
    def test_gga_tail_age(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(qf.gga_tail_age(d, time.time()), (None, None))
            now = time.time(); row = {'host_utc': datetime.fromtimestamp(now - 30, timezone.utc).isoformat(), 'fix_status': 'rtk_fixed'}
            (Path(d) / 'gga.jsonl').write_text(json.dumps(row) + '\n')
            age, fix = qf.gga_tail_age(d, now); self.assertAlmostEqual(age, 30, delta=.5); self.assertEqual(fix, 'rtk_fixed')
    def test_defaults_and_mode_checks(self):
        a = qf.build_parser().parse_args(['--course', 's', '--rtk-session', 'x', '--output-dir', 'y'])
        self.assertEqual((a.float_grace_s, a.heading_mode, a.drive_pwm, a.lookahead_m, a.heading_max_age_s, a.lane), (0., 'bootstrap', 0, 2., 15., 'auto'))
        self.assertEqual((a.max_steer_deg, a.max_steer_right_deg, a.start_distance_m, a.max_lateral_error_m), (25., 19., 5., 5.))
        for extra in (['--heading-mode', 'manual'], ['--initial-heading-deg', '10'], ['--dry-run'], ['--imu-session', 'z', '--imu-yaw-sign', '1']):
            with self.assertRaises(ValueError): qf.main(['--course', 's', '--rtk-session', 'x', '--output-dir', 'y', '--check-only'] + extra)


class Cli(unittest.TestCase):
    def run_main(self, argv):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf): code = qf.main(argv)
        return code, [json.loads(l) for l in buf.getvalue().splitlines() if l.startswith('{')]
    def test_check_only_both_courses(self):
        for course, rng, n in (('s', [105, 182], 78), ('bend', [0, 50], 51)):
            code, out = self.run_main(['--course', course, '--rtk-session', 'nope', '--output-dir', 'nope', '--check-only'])
            self.assertEqual(code, 0); self.assertEqual(out[0]['status'], 'COURSE_SELECTED'); self.assertEqual(out[0]['file_index_range'], rng)
            self.assertEqual(out[0]['routes']['common']['points'], n); self.assertFalse(out[0]['field_verified']); self.assertFalse(out[0]['official_finish_confirmed']); self.assertIn('HYPER', out[0]['basis'])
    def test_stale_rtk_session_refused(self):
        with tempfile.TemporaryDirectory() as d:
            row = {'host_utc': '2026-09-18T20:50:34.375+00:00', 'fix_status': 'dgps_fix'}
            Path(d, 'rtk').mkdir(); (Path(d) / 'rtk/gga.jsonl').write_text(json.dumps(row) + '\n')
            code, out = self.run_main(['--course', 's', '--rtk-session', str(Path(d) / 'rtk'), '--output-dir', str(Path(d) / 'out'),
                                       '--dry-run', '--heading-mode', 'manual', '--initial-heading-deg', '0'])
            self.assertNotEqual(code, 0); self.assertEqual(out[-1]['status'], 'RTK_SESSION_NOT_LIVE')
    def test_dry_run_manual_heading_resume_to_end(self):
        """Synthetic live GGA at 5 Hz moving 2 m/s along S file idx 168..182 starting 2 m left of the path.
        Dry-run (no board): expects entry at the resume segment, adaptive lookahead > 2 m at first, arrival."""
        full, meta = load_points(); lat0, lon0 = meta['origin_lat'], meta['origin_lon']
        pts = full[168:183]; s = qr.cumulative_s(pts); ph0 = qr.path_heading_at(full, 168)
        def at(dist):
            i = max(k for k in range(len(s)) if s[k] <= dist); i = min(i, len(pts) - 2); t = (dist - s[i]) / max(1e-9, s[i + 1] - s[i])
            return pts[i].x + t * (pts[i + 1].x - pts[i].x), pts[i].y + t * (pts[i + 1].y - pts[i].y)
        with tempfile.TemporaryDirectory() as d:
            rtk = Path(d, 'rtk'); rtk.mkdir(); gga = rtk / 'gga.jsonl'; stop = threading.Event()
            def writer():
                k = 0
                while not stop.is_set():
                    dist = min(s[-1] + .6, k * .4); x, y = at(dist); off = max(0., 2. - dist / 3.)   # converge from 2 m left
                    x += -math.sin(ph0) * off; y += math.cos(ph0) * off
                    row = {'host_utc': datetime.now(timezone.utc).isoformat(), 'fix_status': 'rtk_fixed', 'checksum_valid': True,
                           'differential_age_s': 1.0, 'latitude_deg': lat0 + y / 111320, 'longitude_deg': lon0 + x / (111320 * math.cos(math.radians(lat0)))}
                    with gga.open('a') as f: f.write(json.dumps(row) + '\n')
                    k += 1; time.sleep(.2)
            th = threading.Thread(target=writer, daemon=True); th.start(); time.sleep(.5)
            try:
                code, out = self.run_main(['--course', 's', '--resume-index', '168', '--rtk-session', str(rtk), '--output-dir', str(Path(d) / 'out'),
                                           '--dry-run', '--heading-mode', 'manual', '--initial-heading-deg', str(math.degrees(ph0)), '--max-seconds', '30'])
            finally: stop.set(); th.join()
        final = out[-1]; self.assertEqual(final['status'], 'arrived', final); self.assertEqual(code, 0)
        self.assertEqual(final['stats']['entry']['label'], 168); self.assertEqual(final['stats']['route'], 'common'); self.assertGreater(final['stats']['max_lookahead_m'], 2.)
        self.assertEqual(final['stats']['float_grace_rows'], 0); self.assertGreater(final['stats']['calculated_targets'], 10)
        ev = [json.loads(l) for l in (Path(d) / 'out/events.jsonl').read_text().splitlines()] if False else None  # folder removed with tempdir


if __name__ == '__main__': unittest.main()
