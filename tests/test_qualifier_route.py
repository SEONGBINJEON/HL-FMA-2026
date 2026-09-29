"""Q1 synthetic checks. Real pinned CSVs are read-only inputs for loader/quality;
selection/state tests use small synthetic routes and SYNTHETIC_PARAMS only."""
import copy, json, math, sys, tempfile, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
import qualifier_route as qr

ROOT = Path(__file__).parents[1]


def straight(name, n=20, step=1.0, x0=0.0, y0=0.0, yaw=0.0):
    pts = [qr.Point(i, x0 + math.cos(yaw) * step * i, y0 + math.sin(yaw) * step * i, yaw, name) for i in range(n)]
    return qr.Track(name, f"synthetic/{name}.csv", "0" * 64, ["idx", "x", "y", "yaw", "frame_id"], pts)


def route_of(*tracks, mode="s_only"):
    pts = [p for t in tracks for p in t.points]
    return qr.ModeRoute(mode, [qr.Segment(t.name, 0, len(t) - 1, "synthetic") for t in tracks], pts)


P = qr.SYNTHETIC_PARAMS


class RealFiles(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cands = qr.load_candidates()
        cls.tracks = qr.load_all_tracks(cls.cands)

    def test_hashes_match_and_sources_unmodified(self):
        for name, src in self.cands['sources'].items():
            self.assertEqual(self.tracks[name].sha256, src['sha256'], name)
        self.assertFalse(self.cands['runtime_approved'])

    def test_hash_mismatch_and_bad_files_are_rejected(self):
        src = self.cands['sources']['common_1']
        with self.assertRaises(qr.RouteError):
            qr.load_track_csv('common_1', ROOT / src['path'], 'f' * 64)
        with tempfile.TemporaryDirectory() as tmp:
            def write(name, body):
                p = Path(tmp) / name; p.write_text(body); return p
            with self.assertRaises(qr.RouteError):  # NaN
                qr.load_track_csv('nan', write('a.csv', 'idx,x,y,yaw,frame_id\n0,0,0,0,map\n1,nan,1,0,map\n'))
            with self.assertRaises(qr.RouteError):  # non-sequential idx
                qr.load_track_csv('idx', write('b.csv', 'idx,x,y,yaw,frame_id\n0,0,0,0,map\n2,1,1,0,map\n'))
            with self.assertRaises(qr.RouteError):  # wrong frame
                qr.load_track_csv('frame', write('c.csv', 'idx,x,y,yaw,frame_id\n0,0,0,0,map\n1,1,1,0,odom\n'))
            with self.assertRaises(qr.RouteError):  # missing column
                qr.load_track_csv('cols', write('d.csv', 'idx,x,y\n0,0,0\n1,1,1\n'))
            with self.assertRaises(qr.RouteError):  # missing file
                qr.load_track_csv('missing', Path(tmp) / 'none.csv')

    def test_quality_report_reproduces_known_facts(self):
        rep = qr.full_report(self.cands, self.tracks)
        c1 = rep['tracks']['common_1']
        self.assertEqual(c1['rows'], 288)
        self.assertEqual([g['from_idx'] for g in c1['gaps_gt_warn']], [172])
        self.assertEqual([m['idx'] for m in c1['yaw_mismatch_gt_deg']], [171])  # yaw flip inside S candidate
        self.assertTrue(c1['self_crossings'], 'common_1 crosses itself')
        self.assertAlmostEqual(rep['seams']['start_left_wp120_to_common_1']['gap_m'], 1.165, places=3)
        self.assertAlmostEqual(rep['seams']['start_right_wp121_to_common_1']['gap_m'], 1.793, places=3)
        obs = {o['topic']: o for o in rep['observations']}
        self.assertAlmostEqual(obs['18.8m gap']['value_m'], 18.802, places=3)
        self.assertAlmostEqual(obs['start_left wp121 yaw']['csv_at_pinned_commit']['wp121_yaw'], -0.578, places=3)
        self.assertEqual(obs['evidence s_curve_right_angle on common_1']['start_idx'], 0)
        self.assertGreater(obs['evidence s_curve_right_angle on common_1']['start_dist_m'], 5)

    def test_four_modes_build_as_unapproved_candidates(self):
        for m in qr.MODES:
            r = qr.build_mode(m, self.cands, self.tracks)
            self.assertFalse(r.runtime_approved); self.assertFalse(r.complete); self.assertTrue(r.unresolved, m)
        s = qr.build_mode('s_only', self.cands, self.tracks)
        self.assertEqual((s.segments[0].start_idx, s.segments[0].end_idx, len(s.points)), (136, 183, 48))
        b = qr.build_mode('bend_only', self.cands, self.tracks)
        self.assertEqual(b.points, [])  # unresolved on common_1 alone; never guessed
        l = qr.build_mode('start_left', self.cands, self.tracks)
        self.assertEqual(l.segments[0].end_idx, 120); self.assertEqual(l.points[0].source, 'start_left')
        self.assertEqual(l.points[121].source, 'common_1'); self.assertEqual(len(l.seams), 1)
        r = qr.build_mode('start_right', self.cands, self.tracks)
        self.assertEqual(r.segments[0].end_idx, 121)
        # left and right are separate alternatives, never averaged
        self.assertNotEqual(l.points[0], r.points[0])
        with self.assertRaises(qr.RouteError):
            qr.build_mode('all_start', self.cands, self.tracks)
        # explicit indices outside the track are rejected
        bad = copy.deepcopy(self.cands); bad['profiles']['s_only'].update(start_index=100, end_index=999)
        with self.assertRaises(qr.RouteError):
            qr.build_mode('s_only', bad, self.tracks)

    def test_selection_on_real_s_candidate_needs_synthetic_params(self):
        s = qr.build_mode('s_only', self.cands, self.tracks)
        p0 = s.points[0]; h = qr.path_heading_at(s.points, 0)
        pose = qr.Pose(p0.x + 0.2, p0.y - 0.1, h)
        self.assertFalse(qr.select_start(s, pose, qr.OPERATIONAL_PARAMS).accepted)  # unset => reject
        sel = qr.select_start(s, pose, P)
        self.assertTrue(sel.accepted, sel.reason); self.assertEqual(sel.index, 0)
        # standing at the S END facing backwards must not start
        pe = s.points[-1]
        self.assertFalse(qr.select_start(s, qr.Pose(pe.x, pe.y, h + math.pi), P).accepted)


class Selection(unittest.TestCase):
    def test_accept_each_mode_shape_and_reject_no_mode(self):
        t = straight('seg'); r = route_of(t)
        self.assertTrue(qr.select_start(r, qr.Pose(0.3, 0.2, 0.05), P).accepted)
        self.assertEqual(qr.select_start(None, qr.Pose(0, 0, 0), P).reason, 'mode not selected')
        empty = qr.ModeRoute('bend_only', [qr.Segment('common_1', 0, 0, 'x')], [])
        self.assertEqual(qr.select_start(empty, qr.Pose(0, 0, 0), P).reason, 'mode has no resolved points')

    def test_reject_reverse_heading_mismatch_wrong_place_and_stale(self):
        r = route_of(straight('seg'))
        self.assertIn('reverse', qr.select_start(r, qr.Pose(0.2, 0.0, math.pi), P).reason)
        self.assertIn('heading mismatch', qr.select_start(r, qr.Pose(0.2, 0.0, math.radians(40)), P).reason)
        self.assertIn('away', qr.select_start(r, qr.Pose(0.0, 5.0, 0.0), P).reason)
        self.assertIn('stale', qr.select_start(r, qr.Pose(0.2, 0.0, 0.0, age_s=2.0), P).reason)
        self.assertIn('position invalid', qr.select_start(r, qr.Pose(0.2, 0.0, 0.0, position_valid=False), P).reason)
        self.assertIn('heading invalid', qr.select_start(r, qr.Pose(0.2, 0.0, 0.0, heading_valid=False), P).reason)
        self.assertIn('unset', qr.select_start(r, qr.Pose(0.2, 0, 0), None).reason)

    def test_no_jump_into_middle_or_end(self):
        r = route_of(straight('seg', n=30))
        sel = qr.select_start(r, qr.Pose(15.0, 0.1, 0.0), P)
        self.assertFalse(sel.accepted); self.assertIn('no jump into the middle', sel.reason)
        sel = qr.select_start(r, qr.Pose(29.0, 0.0, 0.0), P)
        self.assertFalse(sel.accepted)

    def test_reject_crossing_and_parallel_ambiguity(self):
        # Two synthetic legs crossing at the origin: a pose at the crossing is ambiguous.
        a = straight('a', n=21, x0=-10, y0=0, yaw=0.0)
        b = straight('b', n=21, x0=0, y0=-10, yaw=math.pi / 2)
        r = route_of(a, b)
        r.points = a.points + [qr.Point(21 + i, p.x, p.y, p.yaw, 'b') for i, p in enumerate(b.points)]
        # entry of leg a is at x=-10; put the pose there to pass the entry window, but add a parallel lane
        par = straight('par', n=21, x0=-10, y0=0.9, yaw=0.0)
        r2 = route_of(a, par)
        sel = qr.select_start(r2, qr.Pose(-9.7, 0.45, 0.0), P)
        self.assertFalse(sel.accepted); self.assertIn('ambiguous', sel.reason)
        # same pose with only one lane is fine
        self.assertTrue(qr.select_start(route_of(a), qr.Pose(-9.7, 0.45, 0.0), P).accepted)
        # crossing: pose at the origin lies mid-segment of a and at the entry of b => both far into/ambiguous
        sel = qr.select_start(r, qr.Pose(0.1, 0.0, 0.0), P)
        self.assertFalse(sel.accepted)

    def test_synthetic_params_are_labelled_and_operational_unset(self):
        self.assertEqual(P.source, 'synthetic'); self.assertIsNone(qr.OPERATIONAL_PARAMS)


class State(unittest.TestCase):
    def setUp(self):
        self.t = straight('seg', n=12); self.r = route_of(self.t)

    def run_to_end(self, st):
        outs = []
        for i in range(1, 40):
            out = st.advance(qr.Pose(min(i, 11) + 0.05, 0.0, 0.0)); outs.append(out)
            if not out['drive']:
                break
        return outs

    def test_flow_mode_stop_pose_start_end_no_wrap(self):
        st = qr.RunState(P)
        self.assertFalse(st.arm(qr.Pose(0, 0, 0)))                    # no mode
        self.assertFalse(st.select_mode(self.r, vehicle_stopped=False))  # must be stopped
        self.assertTrue(st.select_mode(self.r, vehicle_stopped=True))
        self.assertFalse(st.start())                                    # not armed
        self.assertFalse(st.arm(qr.Pose(0.2, 0, math.pi)))               # reverse => reject
        self.assertTrue(st.arm(qr.Pose(0.2, 0.0, 0.0))); self.assertEqual(st.state, st.READY)
        self.assertEqual(st.advance(qr.Pose(0.2, 0, 0))['drive'], False)  # ready but not started
        self.assertTrue(st.start())
        outs = self.run_to_end(st)
        self.assertTrue(all(o['drive'] for o in outs[:-1])); self.assertFalse(outs[-1]['drive'])
        self.assertIn('segment end', outs[-1]['reason']); self.assertEqual(st.state, st.END)
        idx = [o['target_idx'] for o in outs]
        self.assertEqual(idx, sorted(idx))                              # monotonic
        for _ in range(5):                                              # no wrap / no restart from the end
            out = st.advance(qr.Pose(0.0, 0.0, 0.0)); self.assertFalse(out['drive'])
        self.assertFalse(st.start())

    def test_relocation_requires_reset_and_clears_progress(self):
        st = qr.RunState(P); st.select_mode(self.r, True); st.arm(qr.Pose(0.1, 0, 0)); st.start()
        for i in range(1, 6): st.advance(qr.Pose(i + 0.05, 0, 0))
        self.assertEqual(st.progress, 5)
        # Q1.1: relocating the car back to the entry without reset => STOPPED, no drive, progress kept
        out = st.advance(qr.Pose(0.0, 0.0, 0.0))
        self.assertFalse(out['drive']); self.assertEqual(st.state, st.STOPPED); self.assertIn('discontinuity', out['reason'])
        self.assertEqual(st.progress, 5)
        self.assertFalse(st.start()); self.assertFalse(st.advance(qr.Pose(0.0, 0.0, 0.0))['drive'])
        self.assertFalse(st.arm(qr.Pose(0.1, 0, 0)))                     # arm without reset is rejected
        st.reset(); self.assertEqual((st.state, st.progress, st.route), (st.IDLE, None, None))
        self.assertTrue(st.select_mode(self.r, True)); self.assertTrue(st.arm(qr.Pose(0.1, 0, 0)))
        self.assertEqual(st.progress, 0)
        # switching to another mode replaces route and progress
        other = route_of(straight('other', n=8, x0=50))
        st.start(); st.advance(qr.Pose(2.0, 0, 0))
        self.assertTrue(st.select_mode(other, True)); self.assertIs(st.route, other); self.assertIsNone(st.progress)

    def test_q1_1_nonfinite_pose_and_bad_params_are_rejected(self):
        r = self.r
        for field in ('x', 'y', 'yaw', 'age_s'):
            for bad in (math.nan, math.inf, -math.inf):
                kw = {'x': 0.2, 'y': 0.0, 'yaw': 0.0, 'age_s': 0.0}; kw[field] = bad
                sel = qr.select_start(r, qr.Pose(**kw), P)
                self.assertFalse(sel.accepted, (field, bad)); self.assertIn('non-finite', sel.reason)
        sel = qr.select_start(r, qr.Pose(0.2, 0, 0, age_s=-1.0), P)
        self.assertFalse(sel.accepted); self.assertIn('negative', sel.reason)
        good = qr.SYNTHETIC_PARAMS
        for kw in ({'max_distance_m': math.nan}, {'max_distance_m': 0.0}, {'max_distance_m': -1.0},
                   {'max_heading_deg': math.inf}, {'max_heading_deg': 0.0}, {'max_heading_deg': 181.0},
                   {'entry_window_m': -0.1}, {'ambiguity_ratio': 0.5}, {'pose_max_age_s': 0.0},
                   {'pose_max_age_s': math.nan}, {'source': ''}):
            bad = qr.SelectionParams(**{**good.__dict__, **kw})
            sel = qr.select_start(r, qr.Pose(0.2, 0, 0), bad)
            self.assertFalse(sel.accepted, kw); self.assertIn('selection parameters', sel.reason)
            self.assertFalse(qr.RunState(bad).select_mode(r, True), kw)
        # running: non-finite pose holds (no drive), never advances or ends
        st = qr.RunState(P); st.select_mode(r, True); st.arm(qr.Pose(0.1, 0, 0)); st.start()
        for field in ('x', 'y', 'yaw', 'age_s'):
            kw = {'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'age_s': 0.0}; kw[field] = math.nan
            out = st.advance(qr.Pose(**kw)); self.assertFalse(out['drive']); self.assertIn('non-finite', out['reason'])
        self.assertEqual((st.state, st.progress), (st.RUNNING, 0))
        self.assertFalse(st.advance(qr.Pose(1.0, 0, 0, age_s=-0.5))['drive'])

    def test_q1_1_far_pose_reverse_and_recovery_only_via_reset(self):
        def running():
            st = qr.RunState(P); self.assertTrue(st.select_mode(self.r, True))
            self.assertTrue(st.arm(qr.Pose(0.0, 0.0, 0.0))); self.assertTrue(st.start()); return st
        # far stationary pose: stop, not END, progress unchanged
        st = running(); outs = [st.advance(qr.Pose(100.0, 100.0, 0.0)) for _ in range(3)]
        self.assertFalse(any(o['drive'] for o in outs)); self.assertEqual(st.state, st.STOPPED)
        self.assertIn('off route', outs[0]['reason']); self.assertEqual(st.progress, 0)
        # lateral offset just beyond the envelope also stops; inside it follows
        st = running(); self.assertTrue(st.advance(qr.Pose(1.0, 0.9, 0.0))['drive'])
        st = running(); self.assertFalse(st.advance(qr.Pose(1.0, 1.1, 0.0))['drive']); self.assertEqual(st.state, st.STOPPED)
        # reverse heading during run: stop
        st = running(); out = st.advance(qr.Pose(1.0, 0.0, math.pi))
        self.assertFalse(out['drive']); self.assertIn('reverse', out['reason']); self.assertEqual(st.state, st.STOPPED)
        st = running(); out = st.advance(qr.Pose(1.0, 0.0, math.radians(40)))
        self.assertFalse(out['drive']); self.assertIn('heading mismatch', out['reason'])
        # END requires arriving near the last point; a jump from idx1 to far beyond is off-route, not arrival
        st = running(); st.advance(qr.Pose(1.0, 0, 0)); out = st.advance(qr.Pose(50.0, 0.0, 0.0))
        self.assertFalse(out['drive']); self.assertEqual(st.state, st.STOPPED)
        # recovery: only reset → select_mode → arm; start/arm/advance on STOPPED are rejected
        st = running(); st.advance(qr.Pose(100.0, 100.0, 0.0))
        self.assertFalse(st.start()); self.assertFalse(st.arm(qr.Pose(0, 0, 0)))
        self.assertFalse(st.select_mode(self.r, vehicle_stopped=False))
        self.assertTrue(st.select_mode(self.r, vehicle_stopped=True)); self.assertEqual(st.state, st.MODE_SELECTED)
        self.assertTrue(st.arm(qr.Pose(0.1, 0.0, 0.0))); self.assertTrue(st.start())
        self.assertTrue(st.advance(qr.Pose(1.0, 0.0, 0.0))['drive'])
        # normal progression to arrival still works and never restarts
        st = running(); outs = self.run_to_end(st)
        self.assertEqual(st.state, st.END); self.assertTrue(all(o['drive'] for o in outs[:-1]))
        self.assertFalse(st.advance(qr.Pose(0.0, 0.0, 0.0))['drive']); self.assertFalse(st.start())

    def test_c3_1_final_approach_end_only_at_or_beyond_last_point(self):
        # C3.1 change for every RunState user: when the forward candidate is the last index, advance() keeps
        # RUNNING ("final approach", drive True, target_idx = last) while the last point is still ahead along the
        # final segment direction; END only at/after the point. Distance/heading/continuity checks are unchanged.
        st = qr.RunState(P); st.select_mode(self.r, True); st.arm(qr.Pose(0.1, 0, 0)); st.start()
        for i in range(1, 11): self.assertTrue(st.advance(qr.Pose(i + 0.05, 0, 0))['drive'])
        out = st.advance(qr.Pose(10.6, 0.0, 0.0))                  # nearest is idx 11 (0.4 m) but 0.4 m still ahead
        self.assertEqual((st.progress, st.state, out['drive'], out['target_idx']), (11, st.RUNNING, True, 11)); self.assertIn('final approach', out['reason'])
        out = st.advance(qr.Pose(10.9, 0.5, 0.0)); self.assertTrue(out['drive']); self.assertEqual(st.state, st.RUNNING)
        out = st.advance(qr.Pose(11.0, 0.0, 0.0))                  # exactly at the point => END
        self.assertFalse(out['drive']); self.assertEqual(st.state, st.END); self.assertIn('segment end', out['reason'])
        # off route / heading rules still stop on the final approach (no END)
        st = qr.RunState(P); st.select_mode(self.r, True); st.arm(qr.Pose(0.1, 0, 0)); st.start()
        for i in range(1, 11): st.advance(qr.Pose(i + 0.05, 0, 0))
        out = st.advance(qr.Pose(10.6, 1.2, 0.0)); self.assertFalse(out['drive']); self.assertEqual(st.state, st.STOPPED)
        # finish(): the follower's explicit end criterion may end the run earlier; only from RUNNING; terminal
        st = qr.RunState(P); st.select_mode(self.r, True); st.arm(qr.Pose(0.1, 0, 0)); st.start(); st.advance(qr.Pose(1.0, 0, 0))
        self.assertFalse(st.finish('x')['drive']); self.assertEqual(st.state, st.END)
        self.assertFalse(st.advance(qr.Pose(2.0, 0, 0))['drive']); self.assertFalse(st.start())
        idle = qr.RunState(P); self.assertIn('ignored', idle.finish('x')['reason']); self.assertEqual(idle.state, idle.IDLE)

    def test_running_holds_on_invalid_or_stale_pose(self):
        st = qr.RunState(P); st.select_mode(self.r, True); st.arm(qr.Pose(0.1, 0, 0)); st.start()
        self.assertFalse(st.advance(qr.Pose(1, 0, 0, position_valid=False))['drive'])
        self.assertFalse(st.advance(qr.Pose(1, 0, 0, age_s=5))['drive'])
        self.assertTrue(st.advance(qr.Pose(1, 0, 0))['drive'])


if __name__ == '__main__':
    unittest.main()
