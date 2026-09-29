import copy, io, json, sys, unittest
from contextlib import redirect_stdout
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
import steering_calibration as sc

ROOT = Path(__file__).parents[1]
# 미측정 기록의 동작을 고정 fixture로 검증한다. 운용 파일(config/steering_calibration.json)은
# 현장 상태에 따라 값이 채워지거나 차단될 수 있어 테스트 고정점이 될 수 없다
# (2026-09-18: 측정값이 채워진 뒤 blocked_angle_unverified로 차단됨).
NULL_RECORD = ROOT / 'tests' / 'fixtures' / 'steering_calibration_unmeasured.json'
OPERATIONAL_RECORD = ROOT / 'config' / 'steering_calibration.json'
SYNTHETIC = ROOT / 'tests' / 'fixtures' / 'steering_calibration_synthetic.json'


def synthetic():
    return json.loads(SYNTHETIC.read_text(encoding='utf-8'))


class Tests(unittest.TestCase):
    def test_unmeasured_record_is_not_ready_and_lists_every_gap(self):
        config, reasons = sc.validate(sc.load(NULL_RECORD))
        self.assertIsNone(config)
        joined = '\n'.join(reasons)
        for needle in ("status is 'unmeasured'", 'steer_command_sign_for_adc_increase', 'unmeasured',
                       'usable_limits.adc_min/adc_max', 'control.stop_band_adc unset', 'limit_policy', 'confirmed'):
            self.assertIn(needle, joined)
        # Straight has angle 0 but no adc: still counted as unmeasured.
        self.assertIn('straight', joined)

    def test_operational_record_is_not_ready_for_use(self):
        """운용 기록은 승인 전까지 어떤 사유로든 READY가 아니어야 한다(차단 여부와 무관한 안전 조건)."""
        config, reasons = sc.validate(sc.load(OPERATIONAL_RECORD))
        self.assertIsNone(config, '운용 보정이 검토 없이 사용 가능 상태다')
        self.assertTrue(reasons)

    def test_synthetic_record_refused_for_operation_but_usable_in_tests(self):
        config, reasons = sc.validate(synthetic())
        self.assertIsNone(config)
        self.assertEqual(reasons, ['synthetic record refused for operation'])
        config, reasons = sc.validate(synthetic(), allow_synthetic=True)
        self.assertEqual(reasons, [])
        self.assertTrue(config['synthetic'])
        self.assertEqual(config['points'], [(-1800, 300), (-900, 470), (0, 600), (800, 720), (1600, 830)])
        self.assertEqual((config['usable_adc_min'], config['usable_adc_max']), (320, 820))
        self.assertEqual(config['limit_policy'], 'LIMIT_REJECT')

    def test_table_vectors_match_cpp_core(self):
        # Same vectors as firmware/steer_target/tests/steer_target_test.cpp table_lookup_vectors().
        pts = sc.validate(synthetic(), allow_synthetic=True)[0]['points']
        self.assertEqual(sc.angle_to_adc(pts, 0), 600)
        self.assertEqual(sc.angle_to_adc(pts, 400), 660)
        self.assertEqual(sc.angle_to_adc(pts, -450), 535)
        self.assertEqual(sc.angle_to_adc(pts, 1500), 816)
        self.assertEqual(sc.angle_to_adc(pts, -1750), 309)
        self.assertEqual(sc.angle_to_adc(pts, 1600), 830)
        self.assertIsNone(sc.angle_to_adc(pts, 1601))
        self.assertIsNone(sc.angle_to_adc(pts, -1801))
        self.assertEqual(sc.adc_to_angle(pts, 660), 400)
        self.assertEqual(sc.adc_to_angle(pts, 535), -450)
        self.assertEqual(sc.adc_to_angle(pts, 700), 667)
        self.assertIsNone(sc.adc_to_angle(pts, 299))

    def test_decreasing_table_validates_and_matches_cpp_vectors(self):
        # C1.1 regression (Codex probe): angle decreasing with ADC, negative denominator.
        rec = synthetic()
        for p in rec['points']:
            p['angle_cdeg'] = -p['angle_cdeg']
        rec['sensor']['steer_command_sign_for_adc_increase'] = 1
        config, reasons = sc.validate(rec, allow_synthetic=True)
        self.assertEqual(reasons, [])
        pts = config['points']
        self.assertEqual(pts, [(1800, 300), (900, 470), (0, 600), (-800, 720), (-1600, 830)])
        expect = {899: 470, 450: 535, -400: 660, 10: 599, -10: 602, 45: 594, 900: 470, -1600: 830, 1800: 300, 0: 600}
        for angle, adc in expect.items():
            self.assertEqual(sc.angle_to_adc(pts, angle), adc, angle)
        self.assertIsNone(sc.angle_to_adc(pts, 1801)); self.assertIsNone(sc.angle_to_adc(pts, -1601))
        for adc, angle in {470: 900, 535: 450, 601: -7, 300: 1800, 830: -1600}.items():
            self.assertEqual(sc.adc_to_angle(pts, adc), angle, adc)
        # Increasing table half-way cases, same as the C++ test.
        inc = sc.validate(synthetic(), allow_synthetic=True)[0]['points']
        for angle, adc in {-45: 594, -10: 599, 10: 602, 45: 607}.items():
            self.assertEqual(sc.angle_to_adc(inc, angle), adc, angle)
        self.assertEqual(sc.adc_to_angle(inc, 601), 7)
        self.assertIn('cfg.steer_sign_for_adc_increase = 1;', sc.emit_cpp(config))

    def test_control_fields_respect_cpp_storage_and_semantic_ranges(self):
        # C1.1 regression (Codex probe): uint16/uint32 overflow used to pass as READY.
        def check(key, value, ok):
            rec = synthetic(); rec['control'][key] = value
            if key == 'stop_band_adc' and _int_ok(value) and value > rec['control']['restart_band_adc']:
                rec['control']['restart_band_adc'] = value
            config, reasons = sc.validate(rec, allow_synthetic=True)
            if ok:
                self.assertEqual(reasons, [], (key, value))
            else:
                self.assertIsNone(config, (key, value))
                self.assertTrue(any(f'control.{key}' in r for r in reasons), (key, value, reasons))
        def _int_ok(v): return type(v) is int
        for key in ('stop_band_adc', 'restart_band_adc', 'sensor_margin_adc'):
            check(key, 1023, True); check(key, 1024, False); check(key, 65535, False)
            check(key, 65540, False); check(key, -1, False); check(key, True, False); check(key, 4.0, False)
        for key in ('command_stale_ms', 'sensor_stale_ms', 'reversal_neutral_ms'):
            check(key, 4294967295, True); check(key, 4294967296, False); check(key, 4294967596, False)
            check(key, -1, False); check(key, True, False)
        check('reversal_neutral_ms', 0, True)
        rec = synthetic(); rec['control'].update(stop_band_adc=65540, restart_band_adc=65544)
        self.assertIsNone(sc.validate(rec, allow_synthetic=True)[0])
        rec = synthetic(); rec['points'][0]['angle_cdeg'] = -40000
        config, reasons = sc.validate(rec, allow_synthetic=True)
        self.assertIsNone(config); self.assertTrue(any('int16' in r for r in reasons))

    def test_each_broken_field_is_rejected_with_a_reason(self):
        def broken(mutate, needle):
            rec = synthetic(); mutate(rec)
            config, reasons = sc.validate(rec, allow_synthetic=True)
            self.assertIsNone(config, needle)
            self.assertTrue(any(needle in r for r in reasons), (needle, reasons))
        broken(lambda r: r['points'].pop(2), 'no straight')
        broken(lambda r: r['points'][3].__setitem__('adc', 470), 'not strictly increasing')  # duplicate adc
        broken(lambda r: r['points'][3].__setitem__('adc', 460), 'not strictly monotonic')  # sorted by adc => angle order breaks
        broken(lambda r: r['points'][3].__setitem__('angle_cdeg', -100), 'not strictly monotonic')
        broken(lambda r: r['points'][0].__setitem__('adc', 1), 'electrical rail')
        broken(lambda r: r['points'][4].__setitem__('adc', 1022), 'electrical rail')
        broken(lambda r: r['points'][1].__setitem__('adc', None), 'unmeasured')
        broken(lambda r: r.__setitem__('points', r['points'][2:4]), 'fewer than 3')
        broken(lambda r: r['usable_limits'].__setitem__('adc_max', 900), 'outside measured table')
        broken(lambda r: r['usable_limits'].__setitem__('adc_min', 650), 'both sides of the straight')
        broken(lambda r: r['usable_limits'].update(adc_min=820, adc_max=320), 'adc_min must be < adc_max')
        broken(lambda r: r['sensor'].__setitem__('steer_command_sign_for_adc_increase', 0), 'must be -1 or +1')
        broken(lambda r: r['sensor'].__setitem__('steer_command_sign_for_adc_increase', None), 'must be -1 or +1')
        broken(lambda r: r['sensor'].__setitem__('saturation_observed', True), 'saturation_observed')
        broken(lambda r: r['control'].__setitem__('stop_band_adc', 0), 'stop_band_adc must be > 0')
        broken(lambda r: r['control'].__setitem__('restart_band_adc', 3), 'restart_band_adc must be >=')
        broken(lambda r: r['control'].__setitem__('command_stale_ms', 0), 'stale timeouts')
        broken(lambda r: r['control'].__setitem__('limit_policy', 'ignore'), 'limit_policy')
        broken(lambda r: r['control'].__setitem__('confirmed', False), 'unconfirmed')
        broken(lambda r: r['measurement'].__setitem__('evidence_paths', []), 'evidence_paths empty')
        broken(lambda r: r['measurement'].__setitem__('measured_kst', None), 'measured_kst missing')
        broken(lambda r: r.__setitem__('status', 'partial'), "status is 'partial'")

    def test_emit_cpp_carries_the_validated_numbers_only(self):
        config = sc.validate(synthetic(), allow_synthetic=True)[0]
        text = sc.emit_cpp(config)
        for needle in ('cfg.calibrated = true;', 'cfg.points[2].angle_cdeg = 0; cfg.points[2].adc = 600;',
                       'cfg.point_count = 5;', 'cfg.usable_adc_min = 320;', 'cfg.usable_adc_max = 820;',
                       'cfg.steer_sign_for_adc_increase = -1;', 'cfg.stop_band_adc = 4;',
                       'cfg.restart_band_adc = 8;', 'cfg.limit_policy = hl_steer_target::LIMIT_REJECT;',
                       'synthetic: true'):
            self.assertIn(needle, text)
        self.assertNotIn('605', text)  # peer-draft numbers never appear

    def test_cli_exit_codes(self):
        out = io.StringIO()
        with redirect_stdout(out):
            self.assertEqual(sc.main(['check', str(NULL_RECORD)]), 1)
            self.assertEqual(sc.main(['check', str(SYNTHETIC)]), 1)
            self.assertEqual(sc.main(['check', str(SYNTHETIC), '--allow-synthetic']), 0)
            self.assertEqual(sc.main(['emit-cpp', str(SYNTHETIC), '--allow-synthetic']), 0)
            self.assertEqual(sc.main(['check', str(ROOT / 'config' / 'does_not_exist.json')]), 2)
        text = out.getvalue()
        self.assertIn('NOT READY', text)
        self.assertIn('READY (SYNTHETIC, tests only)', text)
        self.assertNotIn('ntrip', text.lower())


if __name__ == '__main__':
    unittest.main()
