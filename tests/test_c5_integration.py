"""C5 integration tests: fixture validation, angle contract, and real replay results (synthetic only).

Every replay here runs the actual C2 host core + BoardModel through integrated_replay.run_replay.
No device, no network. "ok" means replay invariants held, never a course pass.
"""
import copy, json, math, sys, tempfile, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import integrated_replay as ir  # noqa: E402
import steering_angle_contract as sac  # noqa: E402
import path_steering as ps  # noqa: E402

FIXTURE = ROOT / "tests/fixtures/c5_replay_synthetic.json"


def _fixture():
    return copy.deepcopy(_fixture.cache)


_fixture.cache = ir.load_input(FIXTURE)


def _stale_heading():
    d = _fixture(); d["records"][10][5] = 2.0   # t=2.0 heading age > max_age_s, later rows fresh
    return d


class InputValidation(unittest.TestCase):
    def test_fixture_loads_and_revalidates(self):
        d = _fixture()
        self.assertIs(ir.validate_input(d), d)
        self.assertEqual(d["angle_contract"]["source_kind"], sac.SOURCE_KIND)
        self.assertEqual(d["angle_contract"]["target_kind"], sac.TARGET_KIND)

    def test_contract_kinds_must_be_explicit_and_correct(self):
        d = _fixture(); d["angle_contract"]["source_kind"] = sac.TARGET_KIND
        with self.assertRaises(ValueError): ir.validate_input(d)
        d = _fixture(); del d["angle_contract"]["target_kind"]
        with self.assertRaises(ValueError): ir.validate_input(d)

    def test_absolute_time_must_stay_within_1_to_120_s(self):
        # span-only check would accept this and pump for ~1e9 s; validate_input must refuse it up front.
        d = _fixture(); d["records"] = [[r[0] + 1e9] + r[1:] for r in d["records"][:4]]
        with self.assertRaises(ValueError): ir.validate_input(d)
        d = _fixture(); d["records"][-1][0] = 120.5
        with self.assertRaises(ValueError): ir.validate_input(d)
        d = _fixture(); d["records"][0][0] = 0.9
        with self.assertRaises(ValueError): ir.validate_input(d)

    def test_bool_nan_inf_duplicate_reversal_refused(self):
        for mutate in (lambda d: d["records"][3].__setitem__(1, True),
                       lambda d: d["records"][3].__setitem__(2, float("nan")),
                       lambda d: d["records"][3].__setitem__(3, float("inf")),
                       lambda d: d["records"][3].__setitem__(0, d["records"][2][0]),
                       lambda d: d["records"][3].__setitem__(0, 1.05),
                       lambda d: d["records"][3].__setitem__(6, 1)):
            d = _fixture(); mutate(d)
            with self.assertRaises(ValueError): ir.validate_input(d)

    def test_unknown_key_and_missing_label_refused(self):
        d = _fixture(); d["extra"] = 1
        with self.assertRaises(ValueError): ir.validate_input(d)
        d = _fixture(); d["pose"]["source"] = "  "
        with self.assertRaises(ValueError): ir.validate_input(d)


class AngleContract(unittest.TestCase):
    def contract(self, L=0.75):
        return sac.AngleContract(sac.SYNTHETIC_MODE, sac.SOURCE_KIND, sac.TARGET_KIND, L, 0.67)

    def test_left_wheel_asymmetry_and_zero(self):
        kw = dict(contract=self.contract(), synthetic=True, min_cdeg=-1800, max_cdeg=1600)
        self.assertEqual(sac.convert_angle(0, **kw), 0)
        self.assertGreater(sac.convert_angle(800, **kw), 800)
        self.assertLess(abs(sac.convert_angle(-800, **kw)), 800)

    def test_huge_length_is_value_error_without_repr(self):
        with self.assertRaises(ValueError) as cm:
            sac.convert_angle(800, self.contract(10 ** 400), synthetic=True, min_cdeg=-1800, max_cdeg=1600)
        self.assertLess(len(str(cm.exception)), 200)
        msg = ps.TargetMessage("steer", 1, 800, 1000, {})
        out = sac.convert_message(msg, self.contract(10 ** 400), synthetic=True, min_cdeg=-1800, max_cdeg=1600)
        self.assertIsInstance(out, ps.StopMessage); self.assertEqual(out.kind, "hold")

    def test_wrong_kind_or_unconfigured_is_hold(self):
        msg = ps.TargetMessage("steer", 1, 300, 1000, {})
        bad = sac.AngleContract(sac.SYNTHETIC_MODE, sac.TARGET_KIND, sac.SOURCE_KIND, 0.75, 0.67)
        for c in (bad, sac.AngleContract(sac.UNCONFIGURED_MODE, sac.SOURCE_KIND, sac.TARGET_KIND, None, None)):
            out = sac.convert_message(msg, c, synthetic=True, min_cdeg=-1800, max_cdeg=1600)
            self.assertIsInstance(out, ps.StopMessage); self.assertEqual(out.kind, "hold")


class Replay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nominal = ir.run_replay(_fixture())
        cls.stale = ir.run_replay(_stale_heading())

    def test_nominal_reaches_path_end_by_input(self):
        r = self.nominal
        self.assertTrue(r["ok"], r["checks"])
        self.assertEqual(r["terminal_cause"], "path_end"); self.assertTrue(r["reached_by_input"])
        self.assertGreater(r["targets"]["accepted"], 0); self.assertIsNone(r["hold"]["cause"])
        self.assertTrue(r["board"]["nonzero_steer_observed"])
        self.assertTrue(any(a != 0 for a in r["angles_cdeg"]["left_front_wheel"]))
        self.assertNotEqual(r["angles_cdeg"]["equivalent"], r["angles_cdeg"]["left_front_wheel"])
        self.assertEqual(r["board"]["final"]["actual_steer"], 0)
        self.assertFalse(r["hardware_access"]); self.assertFalse(r["closed_loop"])

    def test_nominal_records_after_path_end_are_not_faults(self):
        r = self.nominal
        kinds = [e["event"] for e in r["events"]]
        self.assertNotIn("hold_lock", kinds)
        self.assertGreater(r["samples"]["ignored_after_stop"], 0)
        self.assertEqual(r["samples"]["ignored_locked"], 0)
        self.assertIsNone(r["hold"]["adapter_first_fault"])   # the path_end lock is intentional, not a fault
        self.assertEqual(r["targets"]["accepted_after_lock"], 0)

    def test_stale_heading_holds_and_fresh_input_never_resumes(self):
        r = self.stale
        self.assertTrue(r["ok"], r["checks"])
        self.assertEqual(r["terminal_cause"], "hold_lock"); self.assertFalse(r["reached_by_input"])
        self.assertIn("heading stale", r["hold"]["cause"]); self.assertEqual(r["hold"]["at_s"], 2.0)
        accepted_t = [e["t_s"] for e in r["events"] if e["event"] == "target" and e["bridge"] == "target_accepted"]
        self.assertTrue(accepted_t and max(accepted_t) < 2.0)
        self.assertGreater(r["samples"]["ignored_locked"], 0)
        self.assertEqual(r["targets"]["accepted_after_lock"], 0)
        self.assertIsNone(r["path_end"])
        self.assertEqual([e["event"] for e in r["events"] if e["t_s"] > 2.0 and e["event"] not in ("ignored_locked", "eof_stop")], [])

    def test_stale_heading_board_output_stops_after_hold_model_only(self):
        sr = self.stale["stop_response"]
        self.assertTrue(sr["model_only"]); self.assertTrue(sr["steer_nonzero_before_hold"])
        self.assertIsNotNone(sr["first_zero_after_hold_s"])
        self.assertLessEqual(sr["latency_ms"], sr["window_ms"]); self.assertFalse(sr["nonzero_after_first_zero"])

    def test_eof_midroute_is_not_path_end(self):
        d = _fixture(); d["records"] = [r for r in d["records"] if r[0] <= 2.0]
        r = ir.run_replay(d)
        self.assertTrue(r["ok"], r["checks"]); self.assertEqual(r["terminal_cause"], "eof")
        self.assertFalse(r["reached_by_input"]); self.assertIsNone(r["stop"]); self.assertIsNone(r["hold"]["cause"])
        self.assertGreater(r["targets"]["accepted"], 0); self.assertEqual(r["eof"]["bridge"], "lock_disarm")
        self.assertEqual(r["board"]["final"]["actual_steer"], 0)

    def test_input_gap_fault_time_comes_from_host_tick(self):
        d = _fixture(); d["records"] = [r for r in d["records"] if not (1.7 < r[0] < 2.8)]
        r = ir.run_replay(d)
        self.assertTrue(r["ok"], r["checks"]); self.assertEqual(r["terminal_cause"], "adapter_fault")
        self.assertIn("STALE", r["hold"]["adapter_first_fault"])
        self.assertIsNotNone(r["hold"]["fault_tick_s"]); self.assertTrue(1.7 < r["hold"]["fault_tick_s"] < 2.8)
        self.assertEqual(r["hold"]["at_s"], r["hold"]["fault_tick_s"])
        self.assertTrue(r["stop_response"]["steer_nonzero_before_hold"])
        self.assertFalse(r["stop_response"]["nonzero_after_first_zero"])
        self.assertEqual(r["targets"]["accepted_after_lock"], 0); self.assertGreater(r["samples"]["ignored_locked"], 0)

    def test_unconfigured_contract_holds_before_any_accept(self):
        d = _fixture(); d["angle_contract"].update(mode=sac.UNCONFIGURED_MODE, wheelbase_m=None, track_m=None)
        r = ir.run_replay(d)
        self.assertTrue(r["ok"], r["checks"]); self.assertEqual(r["targets"]["accepted"], 0)
        self.assertEqual(r["targets"]["conversion_holds"], 1); self.assertIn("conversion failed", r["hold"]["cause"])
        self.assertEqual(r["terminal_cause"], "hold_lock")


class CliExitSemantics(unittest.TestCase):
    def _run(self, data, name):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / f"{name}.json"; p.write_text(json.dumps(data))
            code = ir.main(["replay", "--input", str(p), "--out", str(Path(td) / "out")])
            summary = json.loads((Path(td) / "out/summary.json").read_text())
        return code, summary

    def test_nominal_replay_exits_zero_and_hold_exits_nonzero(self):
        code, s = self._run(_fixture(), "nominal")
        self.assertEqual(code, 0); self.assertEqual(s["terminal_cause"], "path_end")
        code, s = self._run(_stale_heading(), "stale")
        self.assertNotEqual(code, 0); self.assertTrue(s["ok"]); self.assertEqual(s["terminal_cause"], "hold_lock")

    def test_invalid_input_exits_2_without_dump(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "bad.json"; d = _fixture(); d["records"] = [[r[0] + 1e9] + r[1:] for r in d["records"]]
            p.write_text(json.dumps(d))
            self.assertEqual(ir.main(["replay", "--input", str(p), "--out", str(Path(td) / "out")]), 2)
            self.assertFalse((Path(td) / "out").exists())


if __name__ == "__main__":
    unittest.main()
