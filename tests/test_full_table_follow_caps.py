"""Short bundle for the 2026-09-19 full-table calibration copy and asymmetric follow caps.
No device access. Checks interpolation at the new boundaries, preflight acceptance, and that the
follower cap clamps left/right separately. Validator is used as-is (no bypass)."""
import json, sys, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT / 'tools'))
import steering_calibration as sc, live_steering_trial as lst, live_path_follow as lpf
COPY = ROOT / 'data/claude3_follow_20260919_020600/drive_calibration_full_table.json'
ORIG = ROOT / 'data/stall_guard_trial_20260918_204436/drive_calibration_zero480.json'

class FullTableCopy(unittest.TestCase):
    def setUp(self):
        self.cfg, reasons = sc.validate(sc.load(COPY)); self.assertEqual(reasons, [])
        self.orig, r0 = sc.validate(sc.load(ORIG)); self.assertEqual(r0, [])
    def test_only_usable_limits_changed(self):
        self.assertEqual(self.cfg['points'], self.orig['points'])
        for k in ('steer_sign_for_adc_increase', 'stop_band_adc', 'restart_band_adc', 'command_stale_ms', 'sensor_stale_ms', 'reversal_neutral_ms', 'limit_policy'):
            self.assertEqual(self.cfg[k], self.orig[k], k)
        self.assertEqual((self.cfg['usable_adc_min'], self.cfg['usable_adc_max']), (55, 931))
        self.assertEqual((self.orig['usable_adc_min'], self.orig['usable_adc_max']), (155, 831))
        self.assertFalse(self.cfg['synthetic'])
    def test_boundary_interpolation(self):
        pts = self.cfg['points']
        self.assertEqual(sc.angle_to_adc(pts, 0), 477)
        self.assertEqual(sc.angle_to_adc(pts, 2500), 931); self.assertEqual(sc.angle_to_adc(pts, -1900), 55)
        self.assertEqual(sc.angle_to_adc(pts, -1000), 257); self.assertEqual(sc.angle_to_adc(pts, 1500), 824)
        self.assertIsNone(sc.angle_to_adc(pts, 2501)); self.assertIsNone(sc.angle_to_adc(pts, -1901))  # no extrapolation
        self.assertEqual(lst.angle_in_table(self.cfg, 2500), ''); self.assertEqual(lst.angle_in_table(self.cfg, -1900), '')
        self.assertIn('outside the measured table', lst.angle_in_table(self.cfg, -2000))
        # the original copy still refuses beyond 155..831
        self.assertIn('outside usable', lst.angle_in_table(self.orig, 2500))
    def test_preflight_left25_right19(self):
        self.assertTrue(lst.preflight(COPY, 25.0)['live_capable']); self.assertTrue(lst.preflight(COPY, -19.0)['live_capable'])
        self.assertFalse(lst.preflight(COPY, 26.0)['live_capable']); self.assertFalse(lst.preflight(COPY, -20.0)['live_capable'])
        self.assertFalse(lst.preflight(ORIG, 25.0)['live_capable'])
    def test_asymmetric_cap(self):
        self.assertEqual(lpf.cap_left_wheel(3000, 25, 19), 2500); self.assertEqual(lpf.cap_left_wheel(-3000, 25, 19), -1900)
        self.assertEqual(lpf.cap_left_wheel(-2000, 25, 19), -1900); self.assertEqual(lpf.cap_left_wheel(2000, 25, 19), 2000)
        self.assertEqual(lpf.cap_left_wheel(-500, 25, 19), -500); self.assertEqual(lpf.cap_left_wheel(700, 10, 10), 700)
    def test_cli_right_cap_defaults_and_validates(self):
        a = lpf.get_parser().parse_args(['--route', 'r', '--rtk-session', 's', '--calibration', 'c', '--output-dir', 'o'])
        self.assertIsNone(a.max_steer_right_deg)
        a = lpf.get_parser().parse_args(['--route', 'r', '--rtk-session', 's', '--calibration', 'c', '--output-dir', 'o', '--max-steer-deg', '25', '--max-steer-right-deg', '19'])
        self.assertEqual((a.max_steer_deg, a.max_steer_right_deg), (25.0, 19.0))

if __name__ == '__main__': unittest.main()
