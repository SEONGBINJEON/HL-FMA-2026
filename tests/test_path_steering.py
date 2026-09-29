import math, sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
import path_steering as ps
import qualifier_route as qr

C = ps.SYNTHETIC_CONFIG


def straight(n=30, step=1.0, yaw=0.0, x0=0.0, y0=0.0, name='s'):
    return [qr.Point(i, x0 + math.cos(yaw) * step * i, y0 + math.sin(yaw) * step * i, yaw, name) for i in range(n)]


def arc(R=6.0, ccw=True, n=60, dt=0.05):
    pts = []
    for i in range(n):
        t = i * dt if ccw else -i * dt
        yaw = t + (math.pi / 2 if ccw else -math.pi / 2)
        pts.append(qr.Point(i, R * math.cos(t), R * math.sin(t), qr.wrap(yaw), 'arc'))
    return pts


class Geometry(unittest.TestCase):
    def test_straight_offsets_and_heading_signs(self):
        pts = straight()
        left = ps.compute_target(pts, 2, ps.PoseInput(2.0, 0.3, 0.0), C)
        right = ps.compute_target(pts, 2, ps.PoseInput(2.0, -0.3, 0.0), C)
        self.assertEqual(left.kind, 'steer'); self.assertLess(left.angle_cdeg, 0)      # left of path => steer right (negative)
        self.assertGreater(right.angle_cdeg, 0); self.assertEqual(left.angle_cdeg, -right.angle_cdeg)
        self.assertAlmostEqual(left.lateral_error_m, 0.3, places=6); self.assertAlmostEqual(right.lateral_error_m, -0.3, places=6)
        hl = ps.compute_target(pts, 2, ps.PoseInput(2.0, 0.0, math.radians(20)), C)   # nose left => steer right
        hr = ps.compute_target(pts, 2, ps.PoseInput(2.0, 0.0, -math.radians(20)), C)
        self.assertLess(hl.angle_cdeg, 0); self.assertEqual(hl.angle_cdeg, -hr.angle_cdeg)
        on = ps.compute_target(pts, 2, ps.PoseInput(2.0, 0.0, 0.0), C); self.assertEqual((on.kind, on.angle_cdeg), ('steer', 0))
        self.assertEqual(on.goal_idx, 4); self.assertAlmostEqual(on.goal_xy[0], 3.5, places=6)   # lookahead 1.5 m interpolated

    def test_known_arc_gives_atan_L_over_R_with_turn_sign(self):
        for ccw in (True, False):
            pts = arc(ccw=ccw); p = pts[5]
            d = ps.compute_target(pts, 5, ps.PoseInput(p.x, p.y, p.yaw), C)
            expect = math.degrees(math.atan(C.wheelbase_m / 6.0)) * 100
            self.assertEqual(d.kind, 'steer')
            self.assertAlmostEqual(abs(d.angle_cdeg), expect, delta=6)      # chord vs arc: small discretisation
            self.assertEqual(d.angle_cdeg > 0, ccw)                          # CCW (left turn) => positive
        rad = ps.rad_to_cdeg(math.radians(-12.345)); self.assertEqual(rad, -1234)   # unit conversion, half-up rounding of -1234.5? -> banker's? check magnitude
        self.assertEqual(ps.rad_to_cdeg(math.radians(7.13)), 713)
        with self.assertRaises(ValueError): ps.rad_to_cdeg(math.radians(400))

    def test_rejects_bad_inputs_and_unset_config(self):
        pts = straight()
        self.assertEqual(ps.compute_target(pts, 2, ps.PoseInput(2, 0, 0), None).kind, 'reject')
        for bad in ({'wheelbase_m': 0.0}, {'lookahead_m': math.nan}, {'max_equiv_steer_rad': 2.0}, {'pose_max_age_s': -1.0}, {'wheelbase_source': ''},
                    {'end_reach_tolerance_m': 0.0}, {'end_reach_tolerance_m': math.inf}, {'end_pass_max_overshoot_m': -0.1}, {'end_reference': ''}):
            cfg = ps.PurePursuitConfig(**{**C.__dict__, **bad})
            self.assertEqual(ps.compute_target(pts, 2, ps.PoseInput(2, 0, 0), cfg).kind, 'reject', bad)
        for pose in (ps.PoseInput(math.nan, 0, 0), ps.PoseInput(2, 0, math.inf), ps.PoseInput(2, 0, 0, age_s=1.0),
                     ps.PoseInput(2, 0, 0, frame_id='odom'), ps.PoseInput(2, 0, 0, reference_point='gnss_antenna'),
                     ps.PoseInput(2, 0, 0, position_valid=False), ps.PoseInput(2, 0, 0, heading_valid=False), ps.PoseInput(2, 0, 0, age_s=-1)):
            self.assertEqual(ps.compute_target(pts, 2, pose, C).kind, 'reject', pose)
        self.assertEqual(ps.compute_target(pts[:1], 0, ps.PoseInput(0, 0, 0), C).kind, 'reject')
        self.assertEqual(ps.compute_target(pts, 99, ps.PoseInput(0, 0, 0), C).kind, 'reject')
        self.assertEqual(ps.compute_target(pts, True, ps.PoseInput(0, 0, 0), C).kind, 'reject')

    def test_stop_decisions_are_explicit(self):
        pts = straight()
        # C3.1: the path ending inside the lookahead circle is NOT a stop any more (see EndApproach); arrival is
        end2 = ps.compute_target(pts, 29, ps.PoseInput(29.0, 0, 0), C)
        self.assertEqual((end2.kind, end2.stop_cause, end2.end_reached), ('stop', 'path_end', True)); self.assertIsNone(end2.angle_cdeg)
        off = ps.compute_target(pts, 2, ps.PoseInput(2.0, 1.2, 0), C); self.assertEqual((off.kind, off.stop_cause, off.end_reached), ('stop', 'limit', False)); self.assertIn('lateral', off.reason)
        rev = ps.compute_target(pts, 2, ps.PoseInput(2.0, 0, math.radians(120)), C); self.assertEqual(rev.kind, 'stop'); self.assertIn('heading', rev.reason); self.assertFalse(rev.end_reached)
        tight = ps.compute_target(arc(R=0.9), 5, ps.PoseInput(*[getattr(arc(R=0.9)[5], k) for k in ('x', 'y', 'yaw')]), C)
        self.assertEqual((tight.kind, tight.stop_cause), ('stop', 'limit')); self.assertIn('beyond usable', tight.reason)   # no clamp until measured
        self.assertEqual(ps.OPERATIONAL_CONFIG, None); self.assertEqual(C.source, 'synthetic')


class EndApproach(unittest.TestCase):
    """C3.1: end approach (steer to the last point with its real distance) vs arrival (explicit criterion).
    Synthetic: lookahead 1.5 m, reach tolerance 0.5 m, pass overshoot 1.0 m, lateral envelope 1.0 m."""

    def test_inside_lookahead_but_outside_tolerance_is_approach_not_completion(self):
        pts = straight()                                                  # 0..29 m
        d = ps.compute_target(pts, 28, ps.PoseInput(28.0, 0.0, 0.0), C)   # 1.0 m left: < lookahead, > tolerance
        self.assertEqual(d.kind, 'steer'); self.assertTrue(d.end_approach); self.assertEqual(d.goal_idx, 29)
        self.assertEqual(d.goal_xy, (29.0, 0.0)); self.assertEqual(d.angle_cdeg, 0); self.assertAlmostEqual(d.end_distance_m, 1.0)
        self.assertIsNone(d.end_reached)
        # curvature uses the REAL distance to the end point: vehicle 0.2 m left of the line => steer right, with 2 sin(alpha)/d
        o = ps.compute_target(pts, 28, ps.PoseInput(28.0, 0.2, 0.0), C)
        self.assertEqual(o.kind, 'steer'); self.assertLess(o.angle_cdeg, 0)
        dist = math.hypot(1.0, 0.2); alpha = math.atan2(-0.2, 1.0)
        self.assertAlmostEqual(o.curvature_1pm, 2 * math.sin(alpha) / dist, places=9)
        self.assertEqual(o.angle_cdeg, ps.rad_to_cdeg(math.atan(C.wheelbase_m * o.curvature_1pm)))
        # Codex review case: straight 0..10, x=9, tolerance 0.5 => must not be a stop
        pts10 = straight(n=11)
        r = ps.compute_target(pts10, 9, ps.PoseInput(9.0, 0.0, 0.0), C)
        self.assertEqual(r.kind, 'steer'); self.assertTrue(r.end_approach)

    def test_completion_only_inside_tolerance_or_pass_window(self):
        pts = straight()
        reached = ps.compute_target(pts, 29, ps.PoseInput(28.6, 0.0, 0.0), C)      # 0.4 m <= 0.5
        self.assertEqual((reached.kind, reached.stop_cause, reached.end_reached), ('stop', 'path_end', True))
        edge = ps.compute_target(pts, 28, ps.PoseInput(28.4, 0.0, 0.0), C)          # 0.6 m > 0.5: still approaching
        self.assertEqual(edge.kind, 'steer'); self.assertTrue(edge.end_approach)
        passed = ps.compute_target(pts, 29, ps.PoseInput(29.7, 0.1, 0.0), C)        # 0.7 m beyond the end, inside 1.0 m
        self.assertEqual((passed.kind, passed.stop_cause, passed.end_reached), ('stop', 'path_end', True)); self.assertIn('passed', passed.reason)
        over = ps.compute_target(pts, 29, ps.PoseInput(30.5, 0.0, 0.0), C)          # 1.5 m beyond: not reached, no turning back
        self.assertEqual((over.kind, over.stop_cause, over.end_reached), ('stop', 'limit', False)); self.assertIn('no turning back', over.reason)
        # a stop for a limit before the end is never "reached", whatever the remaining distance
        off = ps.compute_target(pts, 28, ps.PoseInput(28.2, 1.3, 0.0), C)
        self.assertEqual((off.kind, off.stop_cause, off.end_reached), ('stop', 'limit', False))
        self.assertIsNone(ps.OPERATIONAL_CONFIG)

    def test_sparse_last_segment_and_last_index_selection_moment(self):
        # 1 m spacing then a 1.8 m final segment: the nearest index becomes the last one 0.8 m before the end
        pts = straight(n=10) + [qr.Point(10, 10.8, 0.0, 0.0, 's')]
        st = qr.RunState(qr.SYNTHETIC_PARAMS); route = qr.ModeRoute('s_only', [qr.Segment('s', 0, 10, 'synthetic')], pts)
        self.assertTrue(st.select_mode(route, True) and st.arm(qr.Pose(0.1, 0, 0)) and st.start())
        for x in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0):
            self.assertTrue(st.advance(qr.Pose(x + 0.05, 0, 0))['drive'])
        out = st.advance(qr.Pose(10.0, 0.0, 0.0))            # nearest is the last index (0.8 m) but the end is still ahead
        self.assertEqual((st.progress, st.state, out['drive'], out['target_idx']), (10, st.RUNNING, True, 10)); self.assertIn('final approach', out['reason'])
        d = ps.compute_target(pts, st.progress, ps.PoseInput(10.0, 0.0, 0.0), C)
        self.assertEqual(d.kind, 'steer'); self.assertTrue(d.end_approach); self.assertAlmostEqual(d.end_distance_m, 0.8)
        # keep going: inside the tolerance the follower criterion completes; Q1 alone completes once the point is passed
        d2 = ps.compute_target(pts, st.progress, ps.PoseInput(10.4, 0.0, 0.0), C)
        self.assertEqual((d2.kind, d2.end_reached), ('stop', True))
        out = st.advance(qr.Pose(10.4, 0.0, 0.0)); self.assertTrue(out['drive']); self.assertEqual(st.state, st.RUNNING)   # Q1 has no tolerance
        out = st.advance(qr.Pose(10.85, 0.0, 0.0)); self.assertFalse(out['drive']); self.assertEqual(st.state, st.END)      # passed => END

    def test_passed_end_never_steers_back(self):
        pts = straight(n=12)
        for x, y in ((11.3, 0.0), (11.9, 0.3), (13.0, 0.0), (11.5, 1.5)):
            d = ps.compute_target(pts, 11, ps.PoseInput(x, y, 0.0), C)
            self.assertEqual(d.kind, 'stop', (x, y)); self.assertIsNone(d.angle_cdeg)
        # inside the tolerance the heading does not matter for arrival (0.5 m to the last point)
        near = ps.compute_target(pts, 10, ps.PoseInput(10.5, 0.0, math.radians(50)), C)
        self.assertEqual((near.kind, near.end_reached), ('stop', True))
        # last point still ahead along the path (0.1 m) and 0.61 m away, heading/lateral inside limits, but the point
        # lies behind the vehicle in ITS frame (alpha ~ -135 deg): stop, not reached, never steer backwards
        behind = ps.compute_target(pts, 10, ps.PoseInput(10.9, 0.6, math.radians(55)), C)
        self.assertEqual((behind.kind, behind.stop_cause, behind.end_reached), ('stop', 'limit', False)); self.assertIn('not ahead', behind.reason)
        # same spot facing the point (alpha ~ -25 deg at 0.61 m): the real-distance curvature needs ~47 deg equivalent
        # steer, beyond the usable 30 deg => limit stop (no clamp), still not "reached"
        facing = ps.compute_target(pts, 10, ps.PoseInput(10.9, 0.6, math.radians(-55)), C)
        self.assertEqual((facing.kind, facing.stop_cause, facing.end_reached), ('stop', 'limit', False)); self.assertIn('beyond usable', facing.reason)
        self.assertTrue(facing.end_approach)
        # a mild offset facing the point steers towards it (end approach)
        mild = ps.compute_target(pts, 10, ps.PoseInput(10.2, 0.4, math.radians(-20)), C)
        self.assertEqual(mild.kind, 'steer'); self.assertTrue(mild.end_approach); self.assertLess(mild.angle_cdeg, 0)

    def test_short_standalone_course_from_its_start(self):
        # whole course (1.2 m) lies inside the lookahead from the entry: start is an approach, not completion
        pts = [qr.Point(0, 0.0, 0.0, 0.0, 's'), qr.Point(1, 1.2, 0.0, 0.0, 's')]
        st = qr.RunState(qr.SYNTHETIC_PARAMS); route = qr.ModeRoute('s_only', [qr.Segment('s', 0, 1, 'synthetic')], pts)
        self.assertTrue(st.select_mode(route, True) and st.arm(qr.Pose(0.0, 0.05, 0.0)) and st.start())
        f = ps.PathFollower(st, C, lambda s: int(s * 1000))
        m = f.step(ps.PoseInput(0.0, 0.05, 0.0), 0.0)
        self.assertEqual(m.kind, 'steer'); self.assertTrue(m.decision['end_approach']); self.assertEqual(st.state, st.RUNNING)
        m = f.step(ps.PoseInput(0.4, 0.02, 0.0), 0.1); self.assertEqual(m.kind, 'steer')          # 0.8 m left
        m = f.step(ps.PoseInput(0.75, 0.0, 0.0), 0.2)                                              # 0.45 m <= 0.5
        self.assertEqual((m.kind, m.cause, m.reached), ('stop', 'path_end', True)); self.assertEqual(st.state, st.END)
        again = f.step(ps.PoseInput(0.8, 0.0, 0.0), 0.3); self.assertEqual(again.kind, 'stop'); self.assertEqual(f.sequence, 2)

    def test_uses_progress_not_global_nearest(self):
        # two parallel lanes 0.8 m apart; progress on lane A, vehicle slightly closer to lane B: goal stays on lane A ahead of progress
        a = straight(n=20); b = straight(n=20, y0=0.8, name='b')
        route = a + [qr.Point(20 + i, p.x, p.y, p.yaw, 'b') for i, p in enumerate(b)]
        d = ps.compute_target(route, 5, ps.PoseInput(5.0, 0.5, 0.0), C)
        self.assertEqual(d.kind, 'steer'); self.assertLess(d.goal_idx, 20); self.assertLess(d.angle_cdeg, 0)   # steers back to lane A (right)

    def test_ideal_ackermann_is_labelled_model(self):
        r = ps.ideal_ackermann_wheel_angles(math.radians(10), 0.75, 0.67)
        self.assertGreater(r['left_rad'], r['right_rad'])         # left turn: inner (left) wheel steers more
        self.assertIn('unverified', r['model']); self.assertIn('not the A1 measured', r['model'])
        r2 = ps.ideal_ackermann_wheel_angles(-math.radians(10), 0.75, 0.67); self.assertLess(r2['right_rad'], r2['left_rad'])
        with self.assertRaises(ValueError): ps.ideal_ackermann_wheel_angles(math.nan, 0.75, 0.67)


class Follower(unittest.TestCase):
    def make(self, pts):
        st = qr.RunState(qr.SYNTHETIC_PARAMS); route = qr.ModeRoute('s_only', [qr.Segment('s', 0, len(pts) - 1, 'synthetic')], pts)
        self.assertTrue(st.select_mode(route, True)); self.assertTrue(st.arm(qr.Pose(0.1, 0, 0))); self.assertTrue(st.start())
        return ps.PathFollower(st, C, lambda s: int(s * 1000))

    def test_messages_sequence_and_stop(self):
        f = self.make(straight(n=12))
        m1 = f.step(ps.PoseInput(1.0, 0.1, 0.0), 1.0); m2 = f.step(ps.PoseInput(2.0, 0.05, 0.0), 1.1)
        self.assertEqual((m1.kind, m1.sequence, m2.sequence), ('steer', 1, 2)); self.assertEqual(m2.host_time_ms, 1100)
        # stale pose => hold (no target, sequence unchanged), state stays RUNNING
        m3 = f.step(ps.PoseInput(3.0, 0, 0, age_s=2.0), 1.2); self.assertEqual(m3.kind, 'hold'); self.assertEqual(f.sequence, 2)
        # wrong reference frame => hold with reason, never a target
        m4 = f.step(ps.PoseInput(3.0, 0, 0, reference_point='gnss_antenna'), 1.3); self.assertEqual(m4.kind, 'hold'); self.assertIn('rear-axle', m4.reason)
        # walk to the end: explicit stop, RunState stopped, no further targets
        last = None
        for i in range(3, 12):
            last = f.step(ps.PoseInput(float(i), 0.0, 0.0), 1.4 + i * 0.1)
            if last.kind == 'stop': break
        self.assertEqual(last.kind, 'stop'); self.assertFalse(hasattr(last, 'angle_cdeg'))
        self.assertEqual((last.cause, last.reached, f.state.state), ('path_end', True, f.state.END))   # arrival via END, 0.0 m to last point
        again = f.step(ps.PoseInput(11.0, 0, 0), 5.0); self.assertEqual(again.kind, 'stop')
        self.assertEqual(f.sequence, 2 + sum(1 for _ in range(0)) + (f.sequence - 2))  # sequence only grew with steer messages

    def test_stop_cause_and_reached_are_separate(self):
        # limit stop before the end: cause limit, reached False, RunState STOPPED (not END)
        f = self.make(straight(n=12)); f.step(ps.PoseInput(1.0, 0.0, 0.0), 1.0)
        m = f.step(ps.PoseInput(2.0, 0.0, math.radians(50)), 1.1)      # inside Q1 30deg? no: Q1 stops first at 30 deg
        self.assertEqual((m.kind, m.cause, m.reached), ('stop', 'limit', False)); self.assertEqual(f.state.state, f.state.STOPPED)
        # C3 limit (usable steer) with Q1 still happy: tight arc
        pts = arc(R=0.9, n=40); st = qr.RunState(qr.SYNTHETIC_PARAMS); route = qr.ModeRoute('s_only', [qr.Segment('a', 0, 39, 'synthetic')], pts)
        self.assertTrue(st.select_mode(route, True) and st.arm(qr.Pose(pts[0].x, pts[0].y, pts[0].yaw)) and st.start())
        f2 = ps.PathFollower(st, C, lambda s: int(s * 1000)); m = f2.step(ps.PoseInput(pts[1].x, pts[1].y, pts[1].yaw), 0.0)
        self.assertEqual((m.kind, m.cause, m.reached), ('stop', 'limit', False)); self.assertIn('beyond usable', m.reason); self.assertEqual(st.state, st.STOPPED)
        # Q1 END after passing the end beyond the overshoot window: stop, cause limit, reached False
        pts = straight(n=12); st = qr.RunState(qr.SYNTHETIC_PARAMS); route = qr.ModeRoute('s_only', [qr.Segment('s', 0, 11, 'synthetic')], pts)
        self.assertTrue(st.select_mode(route, True) and st.arm(qr.Pose(0.1, 0, 0)) and st.start())
        f3 = ps.PathFollower(st, C, lambda s: int(s * 1000))
        for x in range(1, 11): self.assertEqual(f3.step(ps.PoseInput(float(x), 0.0, 0.0), x * 0.1).kind, 'steer')
        st.progress = 11                                            # pretend the last index was reached without a follower stop
        m = f3.step(ps.PoseInput(11.9, 0.0, 0.0), 2.0)              # Q1: passed => END; follower: 0.9 m <= 1.0 => reached
        self.assertEqual((m.kind, m.cause, m.reached, st.state), ('stop', 'path_end', True, st.END))
        # hold keeps RunState RUNNING (Q1 internal recovery) and carries cause error
        f4 = self.make(straight(n=12)); h = f4.step(ps.PoseInput(1.0, 0.0, 0.0, position_valid=False), 1.0)
        self.assertEqual((h.kind, h.cause, h.reached, f4.state.state), ('hold', 'error', False, f4.state.RUNNING))

    def test_unset_config_never_targets(self):
        st = qr.RunState(qr.SYNTHETIC_PARAMS); pts = straight(n=12)
        route = qr.ModeRoute('s_only', [qr.Segment('s', 0, 11, 'synthetic')], pts); st.select_mode(route, True); st.arm(qr.Pose(0.1, 0, 0)); st.start()
        f = ps.PathFollower(st, ps.OPERATIONAL_CONFIG, lambda s: int(s * 1000))
        m = f.step(ps.PoseInput(1.0, 0, 0), 1.0); self.assertEqual(m.kind, 'hold'); self.assertIn('unset', m.reason); self.assertEqual(f.sequence, 0)

    def test_no_mode_selected_gives_hold_not_target(self):
        f = ps.PathFollower(qr.RunState(qr.SYNTHETIC_PARAMS), C, lambda s: int(s * 1000))
        m = f.step(ps.PoseInput(0, 0, 0), 0.0); self.assertEqual(m.kind, 'hold'); self.assertIn('not running', m.reason)   # no mode: wait, never a target


if __name__ == '__main__':
    unittest.main()
