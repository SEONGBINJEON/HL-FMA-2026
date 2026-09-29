"""C2 host adapter tests. Synthetic calibration only; no serial. Builds the shared
library once if missing (same command as firmware/steer_target/host/build.sh)."""
import ctypes, subprocess, sys, unittest
from pathlib import Path
ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import steer_target_host as h
import steer_target_sim as sim
import steering_calibration as sc
from joystick_protocol import Parser, encode_command
from step_manual_protocol import Command, CONTROL, DISARM, ARM


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not h.LIB_PATH.is_file():
            subprocess.run(['bash', str(ROOT / 'firmware/steer_target/host/build.sh')], check=True, capture_output=True)
        cls.lib = h.load_library()
        cls.cfg, reasons, syn = h.load_calibration(ROOT / 'tests/fixtures/steering_calibration_synthetic.json', allow_synthetic=True)
        assert cls.cfg is not None and syn


class Binding(Base):
    def test_cabi_matches_python_validator_vectors(self):
        pts = sc.validate(sc.load(ROOT / 'tests/fixtures/steering_calibration_synthetic.json'), allow_synthetic=True)[0]['points']
        v = ctypes.c_uint16(); a = ctypes.c_int16()
        for angle in (0, 400, -450, 1500, -1750, 1600, -45, 10, 45):
            self.assertEqual(self.lib.stc_angle_to_adc(ctypes.byref(self.cfg), angle, ctypes.byref(v)), 1)
            self.assertEqual(v.value, sc.angle_to_adc(pts, angle), angle)
        self.assertEqual(self.lib.stc_angle_to_adc(ctypes.byref(self.cfg), 1601, ctypes.byref(v)), 0)
        for adc in (660, 535, 700, 601):
            self.assertEqual(self.lib.stc_adc_to_angle(ctypes.byref(self.cfg), adc, ctypes.byref(a)), 1)
            self.assertEqual(a.value, sc.adc_to_angle(pts, adc), adc)
        self.assertEqual(self.lib.stc_config_valid(ctypes.byref(self.cfg)), 1)
        self.assertEqual(self.lib.stc_config_valid(ctypes.byref(h.StcConfig())), 0)

    def test_operational_record_gives_no_config_and_synthetic_is_refused_by_default(self):
        cfg, reasons, syn = h.load_calibration(ROOT / 'config/steering_calibration.json')
        self.assertIsNone(cfg); self.assertTrue(reasons)
        cfg, reasons, syn = h.load_calibration(ROOT / 'tests/fixtures/steering_calibration_synthetic.json')
        self.assertIsNone(cfg); self.assertIn('synthetic record refused for operation', reasons)
        core = h.TargetCore(self.lib, None)
        self.assertFalse(core.valid); core.set_ready(True, 0); self.assertFalse(core.set_target(1, 0, 0, 0))
        self.assertEqual(h.REASON_NAMES[core.update(10).reason], 'NO_CALIBRATION')

    def test_core_same_sequence_never_renews(self):
        core = h.TargetCore(self.lib, self.cfg); core.set_ready(True, 0)
        self.assertTrue(core.set_target(5, 400, 0, 0)); self.assertFalse(core.set_target(5, 400, 0, 200))
        core.set_measurement(600, 290); self.assertEqual(h.REASON_NAMES[core.update(290).reason], 'MOVING')
        core.set_measurement(600, 300); self.assertEqual(h.REASON_NAMES[core.update(300).reason], 'COMMAND_STALE')

    def test_zero_pwm_encoder(self):
        raw = h.encode_zero_pwm(Command(7, 3, CONTROL, 0, -1, 1))
        self.assertEqual(raw, encode_command(Command(7, 3, CONTROL, 0, -1, 1), 0)); self.assertEqual(raw[16], 0)


class Adapter(Base):
    def status_frame(self, board, ms):
        f = board.status(ms); self.assertIsNotNone(f); return f

    def test_prepare_is_explicit_and_target_before_ready_is_rejected(self):
        core = h.TargetCore(self.lib, self.cfg); ad = h.HostSteerAdapter(core)
        board = sim.BoardModel(self.lib, 9, 600, self.cfg.steer_sign_for_adc_increase)
        p = Parser()
        board.tick(1000); vals, _ = p.feed(self.status_frame(board, 1000), 1.0)
        cmd = ad.step(1.0, vals)                      # locked => DISARM reply, no auto-prepare
        self.assertEqual(cmd.action, DISARM)
        self.assertFalse(ad.set_target(1, 400, 0, 1.0))   # not prepared
        self.assertTrue(ad.request_prepare(1.01)); self.assertEqual(ad.phase, 'neutral_pending')
        board.feed(h.encode_zero_pwm(cmd), 1005)
        board.tick(1050); vals, _ = p.feed(self.status_frame(board, 1050), 1.05)
        cmd = ad.step(1.05, vals); self.assertEqual((cmd.action, cmd.steer, cmd.deadman), (CONTROL, 0, 0)); board.feed(h.encode_zero_pwm(cmd), 1055)
        board.tick(1100); vals, _ = p.feed(self.status_frame(board, 1100), 1.10)
        cmd = ad.step(1.10, vals); self.assertEqual(cmd.action, ARM); board.feed(h.encode_zero_pwm(cmd), 1105)
        board.tick(1150); vals, _ = p.feed(self.status_frame(board, 1150), 1.15)
        cmd = ad.step(1.15, vals); self.assertEqual(ad.phase, 'ready'); self.assertEqual((cmd.action, cmd.steer), (CONTROL, 0))
        self.assertTrue(ad.set_target(1, 400, 0, 1.15)); self.assertFalse(ad.set_target(1, 400, 0, 1.16))  # same seq
        board.destroy()

    def test_stop_locks_and_requires_prepare_and_new_target(self):
        core = h.TargetCore(self.lib, self.cfg); ad = h.HostSteerAdapter(core)
        ad.phase = 'ready'; ad.identified = True; ad.locked_by_fault = False
        self.assertTrue(ad.set_target(1, 400, 0, 0.0))            # adapter pushes readiness into the core itself
        ad.stop('explicit')
        self.assertTrue(ad.locked_by_fault); self.assertEqual(ad.phase, 'locked')
        self.assertFalse(ad.set_target(2, 400, 0, 0.1))
        self.assertEqual(ad.last_sent_sign, 0); self.assertIsNone(ad.neutral_accepted_at_ms)


class C21(Base):
    """C2.1 regression: fault propagation, input validation, no-calibration prepare, neutral gate."""

    def make_status(self, boot, req, state, reason, req_steer=0, act_steer=0, adc=600, flags=38):
        from joystick_protocol import JoystickStatus
        return JoystickStatus(boot, req, 1000 + req * 50, 0, adc, state, reason, 0, req_steer, 0, act_steer, flags,
                              999, 999, 999, 0, 0, 0, 0, 0, 0, 0, 0)

    def ready_adapter(self):
        core = h.TargetCore(self.lib, self.cfg); ad = h.HostSteerAdapter(core)
        ad.identified = True; ad.phase = 'ready'; ad.locked_by_fault = False
        ad.status = self.make_status(57, 40, 1, 9); ad.last_status_at = 2.0; ad.last_tick_at = 2.0; ad.last_reply = (57, 40)
        core.set_ready(True, 2000)
        return ad

    def test_input_validation_before_ctypes(self):
        for bad in ((0, 400, 0, 1.0), (2 ** 32, 400, 0, 1.0), (True, 400, 0, 1.0), (1, 65536 + 400, 0, 1.0), (1, -32769, 0, 1.0),
                    (1, True, 0, 1.0), (1, 400, -1, 1.0), (1, 400, 2 ** 32, 1.0), (1, 400, 0, float('nan')), (1, 400, 0, -1.0), (1, 4.0, 0, 1.0)):
            self.assertTrue(h.check_target_input(*bad), bad)
        self.assertEqual(h.check_target_input(1, 400, 0, 1.0), '')
        self.assertEqual(h.check_target_input(2 ** 32 - 1, -32768, 2 ** 32 - 1, 0.0), '')
        core = h.TargetCore(self.lib, self.cfg); core.set_ready(True, 0)
        with self.assertRaises(ValueError): core.set_target(1, 65536 + 400, 0, 0)
        with self.assertRaises(ValueError): core.set_measurement(1024, 0)
        self.assertEqual(core.last_sequence(), 0)          # nothing reached the C ABI

    def test_bad_target_locks_adapter_and_disarms_next_request(self):
        ad = self.ready_adapter()
        self.assertTrue(ad.set_target(1, 400, 0, 2.0))
        self.assertFalse(ad.set_target(2, 65536 + 400, 0, 2.05))
        self.assertTrue(ad.locked_by_fault); self.assertIn('invalid target input', ad.first_fault)
        cmd = ad.step(2.1, [self.make_status(57, 41, 1, 10, -1, -1, 610, 102)])
        self.assertEqual(cmd.action, DISARM); self.assertTrue(any(t.note.startswith('first fault:') for t in ad.trace))
        self.assertFalse(ad.set_target(3, 400, 0, 2.2))    # new sequence alone never resumes

    def test_rejected_target_is_a_fault_not_silent(self):
        ad = self.ready_adapter()
        self.assertFalse(ad.set_target(1, 1600, 0, 2.0))   # 830 > usable 820 under REJECT
        self.assertTrue(ad.locked_by_fault); self.assertIn('rejected', ad.first_fault)
        ad2 = self.ready_adapter(); self.assertTrue(ad2.set_target(1, 400, 0, 2.0))
        self.assertFalse(ad2.set_target(1, 400, 0, 2.1)); self.assertFalse(ad2.locked_by_fault)  # duplicate is not a fault

    def test_core_latch_after_target_is_fault_but_residual_latch_after_prepare_is_not(self):
        ad = self.ready_adapter(); self.assertTrue(ad.set_target(1, 400, 0, 2.0))
        # statuses every 50 ms; the single target goes stale at 2.0 + 0.3 s
        cmd = None
        for i in range(1, 8):
            cmd = ad.step(2.0 + 0.05 * i, [self.make_status(57, 40 + i, 1, 10, -1, -1, 600 + i * 3, 102)])
            if cmd.action == DISARM: break
        self.assertEqual(cmd.action, DISARM); self.assertIn('COMMAND_STALE', ad.first_fault); self.assertTrue(ad.locked_by_fault)
        self.assertAlmostEqual(2.0 + 0.05 * i, 2.3, places=2)
        # re-prepare: residual core latch must not re-lock
        ad.locked_by_fault = False; ad.phase = 'ready'; ad.first_fault = None; ad.have_target_since_prepare = False
        ad.last_status_at = 2.3
        cmd = ad.step(2.35, [self.make_status(57, 50, 1, 9, 0, 0, 620, 38)])
        self.assertEqual((cmd.action, cmd.steer), (CONTROL, 0)); self.assertFalse(ad.locked_by_fault)
        self.assertTrue(ad.set_target(2, 400, 0, 2.35))   # same core object, sequence memory kept
        self.assertEqual(ad.core.last_sequence(), 2)

    def test_no_calibration_refuses_prepare(self):
        core = h.TargetCore(self.lib, None); ad = h.HostSteerAdapter(core)
        ad.identified = True; ad.status = self.make_status(57, 5, 0, 1); ad.last_status_at = 1.0
        self.assertFalse(ad.request_prepare(1.01)); self.assertEqual(ad.phase, 'locked'); self.assertIn('보정 없음', ad.message)
        cmd = ad.step(1.05, [self.make_status(57, 6, 0, 1)]); self.assertEqual(cmd.action, DISARM)

    def test_neutral_confirmation_gate_rejects_wrong_statuses(self):
        ad = self.ready_adapter(); self.assertTrue(ad.set_target(1, 400, 0, 2.0))
        cmd = ad.step(2.05, [self.make_status(57, 41, 1, 9, 0, 0, 600, 38)]); self.assertEqual(cmd.steer, -1)   # moving
        # target reached: adapter sends neutral in reply to request 42
        cmd = ad.step(2.1, [self.make_status(57, 42, 1, 10, -1, -1, 660, 102)]); self.assertEqual(cmd.steer, 0)
        self.assertEqual(ad.neutral_pending_req, (57, 42))
        # keep the target fresh and ask for the opposite direction
        self.assertTrue(ad.set_target(2, -450, 0, 2.1))
        # (a) duplicate of request 42 (same id) => Session ignores it, no acceptance, no reversal
        cmd = ad.step(2.12, [self.make_status(57, 42, 1, 9, 0, 0, 660, 38)]); self.assertIsNone(cmd); self.assertIsNone(ad.neutral_accepted_at_ms)
        # (b) newer requests but board still ACTIVE with requested -1 => not accepted; once the core's own
        #     60 ms neutral time has passed the adapter gate is what holds the reversal
        cmd = ad.step(2.15, [self.make_status(57, 43, 1, 10, -1, -1, 660, 102)]); self.assertEqual(cmd.steer, 0)
        cmd = ad.step(2.2, [self.make_status(57, 44, 1, 10, -1, -1, 660, 102)]); self.assertEqual(cmd.steer, 0)
        self.assertTrue(any('reversal held' in t.note for t in ad.trace)); self.assertIsNone(ad.neutral_accepted_at_ms)
        # (c) proper confirmation: newer request, ARMED, requested/actual 0, READY => accepted; reversal after 60 ms
        cmd = ad.step(2.25, [self.make_status(57, 45, 1, 9, 0, 0, 660, 38)]); self.assertEqual(cmd.steer, 0)
        self.assertIsNotNone(ad.neutral_accepted_at_ms)
        cmd = ad.step(2.3, [self.make_status(57, 46, 1, 9, 0, 0, 660, 38)]); self.assertEqual(cmd.steer, 0)   # 50 ms < 60 ms
        cmd = ad.step(2.35, [self.make_status(57, 47, 1, 9, 0, 0, 660, 38)]); self.assertEqual(cmd.steer, 1)
        self.assertTrue(any(t.note.startswith('reversal allowed') for t in ad.trace))
        # (d) a different boot while pending => Session restart fault, lock, no reversal
        ad2 = self.ready_adapter(); self.assertTrue(ad2.set_target(1, 400, 0, 2.0))
        ad2.step(2.05, [self.make_status(57, 41, 1, 9, 0, 0, 600, 38)]); ad2.step(2.1, [self.make_status(57, 42, 1, 10, -1, -1, 660, 102)])
        self.assertTrue(ad2.set_target(2, -450, 0, 2.1))
        cmd = ad2.step(2.15, [self.make_status(58, 3, 1, 9, 0, 0, 660, 38)])
        self.assertEqual(cmd.action, DISARM); self.assertTrue(ad2.locked_by_fault); self.assertIn('재시작', ad2.first_fault)
        # (e) request regression while pending => lock, no reversal
        ad3 = self.ready_adapter(); self.assertTrue(ad3.set_target(1, 400, 0, 2.0))
        ad3.step(2.05, [self.make_status(57, 41, 1, 9, 0, 0, 600, 38)]); ad3.step(2.1, [self.make_status(57, 42, 1, 10, -1, -1, 660, 102)])
        self.assertTrue(ad3.set_target(2, -450, 0, 2.1))
        cmd = ad3.step(2.15, [self.make_status(57, 40, 1, 9, 0, 0, 660, 38)])
        self.assertTrue(ad3.locked_by_fault); self.assertIn('역행', ad3.first_fault)
        self.assertNotIn(1, [t.sent['steer'] for t in ad3.trace if t.sent and t.sent['action'] == CONTROL])


class Scenarios(Base):
    # C3 cleanup (C2.1 review): the helper was named run() and shadowed unittest.TestCase.run(),
    # so this class's tests never executed. Renamed; assertions below now really run.
    def run_scenario(self, name):
        r = sim.run_scenario(name, self.lib, self.cfg)
        self.assertTrue(r['ok'], (name, r['checks'], r['summary']))
        return r

    def test_all_scenarios_pass(self):
        for name in sim.SCENARIOS:
            with self.subTest(name):
                self.run_scenario(name)

    def test_reach_reverse_details(self):
        r = self.run_scenario('reach'); self.assertEqual(r['summary']['board_reasons_seen'], ['ACTIVE', 'NEUTRAL', 'READY', 'STOP'])
        r = self.run_scenario('reverse'); self.assertTrue(r['checks']['reversal_waited_for_board_neutral'])
        r = self.run_scenario('lost_neutral'); self.assertEqual(r['summary']['final_phase'], 'locked')
        r = self.run_scenario('no_calibration'); self.assertEqual(r['summary']['final_plant_adc'], 600); self.assertTrue(r['checks']['prepare_refused_no_arm_sent'])
        for name in ('stale_then_recover', 'sensor_invalid_then_recover', 'bad_target_input', 'wrong_status_no_reverse'):
            self.assertTrue(self.run_scenario(name)['ok'], name)
        r = self.run_scenario('unidentified_protocol'); self.assertEqual(r['summary']['tx_frames'], 0)


if __name__ == '__main__':
    unittest.main()
