"""C3.1: follower hold/stop -> C2 lock/DISARM bridge, and the synthetic integration scenarios.
Synthetic calibration/board MODEL only; no serial, no vehicle. Builds the shared library if missing."""
import subprocess, sys, unittest
from dataclasses import asdict
from pathlib import Path
ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import path_steering as ps
import path_steering_bridge as pb
import path_steering_sim as psim
import steer_target_host as h
import steer_target_sim as sim
from joystick_protocol import Parser, JoystickStatus
from step_manual_control import Client
from step_manual_protocol import CONTROL, DISARM, ARM, ARMED, DISARMED


def status(boot, req, state, reason, req_steer=0, act_steer=0, adc=600, flags=38):
    return JoystickStatus(boot, req, 1000 + req * 50, 0, adc, state, reason, 0, req_steer, 0, act_steer, flags,
                          999, 999, 999, 0, 0, 0, 0, 0, 0, 0, 0)


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not h.LIB_PATH.is_file():
            subprocess.run(['bash', str(ROOT / 'firmware/steer_target/host/build.sh')], check=True, capture_output=True)
        cls.lib = h.load_library()
        cls.cfg, _, syn = h.load_calibration(ROOT / 'tests/fixtures/steering_calibration_synthetic.json', allow_synthetic=True)
        assert cls.cfg is not None and syn

    def ready_adapter(self, client=False):
        core = h.TargetCore(self.lib, self.cfg); ad = h.HostSteerAdapter(core)
        ad.identified = True; ad.phase = 'ready'; ad.locked_by_fault = False
        ad.status = status(57, 40, ARMED, 9); ad.last_status_at = 2.0; ad.last_tick_at = 2.0; ad.last_reply = (57, 40)
        core.set_ready(True, 2000)
        if client:
            tr = sim.MockTransport(); cl = Client(tr, ad, None, encoder=h.encode_zero_pwm); cl.parser = Parser()
            return ad, tr, cl
        return ad

    def hold(self, t=2.1, reason='pose rejected: position invalid'):
        return ps.StopMessage('hold', reason, int(t * 1000), cause='error')

    def steer(self, seq, angle=400, t=2.0):
        return ps.TargetMessage('steer', seq, angle, int(t * 1000), {})


class Bridge(Base):
    def test_hold_locks_c2_disarms_and_blocks_targets_until_explicit_reprepare(self):
        ad = self.ready_adapter(); br = pb.FollowerC2Bridge(ad)
        self.assertEqual(br.apply(self.steer(1), 2.0).action, 'target_accepted'); self.assertTrue(ad.have_target_since_prepare)
        r = br.apply(self.hold(2.1), 2.1)
        self.assertEqual(r.action, 'lock_disarm'); self.assertTrue(ad.locked_by_fault); self.assertEqual(ad.phase, 'locked')
        self.assertEqual((r.disarm_command['boot_id'], r.disarm_command['request_id'], r.disarm_command['action']), (57, 40, DISARM))
        self.assertFalse(r.disarm_sent)                       # no Client here: nothing was transmitted, and we say so
        self.assertFalse(ad.have_target_since_prepare); self.assertIn('path follower hold', ad.first_fault)
        # the next board request is answered with DISARM (never CONTROL steer 0 as a "stop")
        cmd = ad.step(2.15, [status(57, 41, ARMED, 10, -1, -1, 610, 102)])
        self.assertEqual(cmd.action, DISARM)
        # a valid pose alone => the follower emits new targets, but C2 refuses them; nothing resumes
        for seq in (2, 3):
            r = br.apply(self.steer(seq, t=2.2), 2.2); self.assertEqual(r.action, 'not_prepared'); self.assertIn('reprepare', r.reason)
        self.assertEqual(br.refused_targets, 2); self.assertEqual(ad.core.last_sequence(), 1)
        # repeated holds while locked are no-ops (no DISARM spam, no new lock events)
        r = br.apply(self.hold(2.3), 2.3); self.assertEqual(r.action, 'already_locked'); self.assertEqual(len(br.lock_events), 1)
        # explicit reprepare needs a fresh DISARMED status (C2 rule) and is the ONLY way back
        ad.status = status(57, 42, DISARMED, 1); ad.last_status_at = 2.35
        self.assertTrue(br.reprepare(2.36)); self.assertEqual(ad.phase, 'neutral_pending'); self.assertFalse(ad.locked_by_fault)
        r = br.apply(self.steer(4, t=2.37), 2.37); self.assertEqual(r.action, 'not_prepared')   # preparing, not ready yet
        ad.phase = 'ready'                                     # handshake completion is covered by the simulator/C2 tests
        r = br.apply(self.steer(5, t=2.5), 2.5); self.assertEqual(r.action, 'target_accepted'); self.assertEqual(ad.core.last_sequence(), 5)

    def test_stop_with_client_puts_disarm_on_the_wire(self):
        ad, tr, cl = self.ready_adapter(client=True); br = pb.FollowerC2Bridge(ad, cl)
        br.apply(self.steer(1), 2.0)
        msg = ps.StopMessage('stop', 'path end reached: 0.4m', 2100, cause='path_end', reached=True)
        r = br.apply(msg, 2.1)
        self.assertEqual((r.action, r.disarm_sent, r.cause, r.reached), ('lock_disarm', True, 'path_end', True))
        frames = [sim.decode_tx(f) for f in tr.tx_frames]
        self.assertEqual(len(frames), 1); self.assertEqual((frames[0]['action'], frames[0]['req'], frames[0]['drive'], frames[0]['pwm']), (DISARM, 40, 0, 0))
        self.assertEqual(br.last_stop, msg); self.assertTrue(ad.locked_by_fault)

    def test_hold_during_preparation_aborts_and_locked_without_target_is_noop(self):
        core = h.TargetCore(self.lib, self.cfg); ad = h.HostSteerAdapter(core); br = pb.FollowerC2Bridge(ad)
        ad.identified = True; ad.status = status(57, 5, DISARMED, 1); ad.last_status_at = 1.0
        self.assertTrue(ad.request_prepare(1.01)); self.assertEqual(ad.phase, 'neutral_pending')
        r = br.apply(self.hold(1.02), 1.02)
        self.assertEqual(r.action, 'lock_disarm'); self.assertEqual(ad.phase, 'locked'); self.assertTrue(ad.locked_by_fault)
        ad2 = h.HostSteerAdapter(h.TargetCore(self.lib, self.cfg)); br2 = pb.FollowerC2Bridge(ad2)   # never prepared
        r = br2.apply(self.hold(0.5), 0.5)
        self.assertEqual(r.action, 'already_locked'); self.assertIsNone(ad2.first_fault); self.assertEqual(br2.lock_events, [])
        r = br2.apply(self.steer(1, t=0.6), 0.6); self.assertEqual(r.action, 'not_prepared')   # bridge never prepares by itself
        self.assertEqual(ad2.phase, 'locked')
        with self.assertRaises(TypeError): br2.apply(object(), 0.7)


class Scenarios(Base):
    def test_all_synthetic_scenarios_hold_their_invariants(self):
        for name in psim.SCENARIOS:
            with self.subTest(name):
                r = psim.run(scenario=name)
                self.assertTrue(r['ok'], (name, r['checks'], r['summary']))

    def test_hold_scenario_details(self):
        r = psim.run(scenario='hold_transient'); s = r['summary']
        self.assertEqual(s['messages']['hold'], 1); self.assertEqual(s['bridge']['locks'], 2)          # hold lock + end stop lock
        self.assertFalse(any(s['hold']['board_actual_steer_first_50ms_after_hold']))
        self.assertGreater(s['hold']['targets_refused_before_reprepare'], 0)
        self.assertEqual((s['stop']['cause'], s['stop']['reached'], s['q1_state']), ('path_end', True, 'END'))
        self.assertLessEqual(s['end_geometry_at_stop']['distance_m'], ps.SYNTHETIC_CONFIG.end_reach_tolerance_m)
        r = psim.run(scenario='hold_no_reprepare'); s = r['summary']
        self.assertIsNone(s['stop']); self.assertEqual(s['q1_state'], 'RUNNING'); self.assertEqual(s['bridge']['locks'], 1)
        r = psim.run(scenario='nominal'); s = r['summary']
        self.assertEqual(s['messages']['hold'], 0); self.assertTrue(s['end_geometry_at_stop']['reached'])
        self.assertLess(s['end_geometry_at_stop']['distance_m'], 1.0)   # old demo stopped ~1.48 m short and called it reached


if __name__ == '__main__':
    unittest.main()
