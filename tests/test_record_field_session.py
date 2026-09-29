"""One hardware-free orchestration batch using real, isolated fake processes."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("field", ROOT / "tools/record_field_session.py")
field = importlib.util.module_from_spec(spec)
spec.loader.exec_module(field)

FAKE = '''import json, pathlib, signal, sys, time
name, destination, mode = sys.argv[1:]
folder = pathlib.Path(destination)
folder.mkdir(parents=True)
print(("저장 폴더: " if name == "rtk" else "OUTPUT=") + str(folder), flush=True)
def write(status):
    data = {"status": status, "gga_records": 4, "rtcm_frames_written": 3,
            "rtk_fixed_observed": False, "summary": {"valid_pairs": 0 if mode == "empty" else 4},
            "capture": {"measurements": {"positive_distance_nodes": 7}}}
    (folder / ("report.json" if name == "imu" else "metadata.json")).write_text(json.dumps(data))
def stop(*_):
    write("interrupted")
    (folder / "cleanup.marker").write_text("SIGINT cleanup ran")
    raise SystemExit(130)
signal.signal(signal.SIGINT, stop)
if mode == "hang":
    while True: time.sleep(.05)
time.sleep(.12 if name == "lidar" else .04)
write("failed" if mode == "fail" else "completed")
raise SystemExit(5 if mode == "fail" else 0)
'''


class FieldBatch(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="field orchestration spaces ")
        self.root = Path(self.temp.name)
        self.old = field.ROOT
        field.ROOT = self.root
        (self.root / "data").mkdir()
        self.child = self.root / "fake child.py"
        self.child.write_text(FAKE)

    def tearDown(self):
        field.ROOT = self.old
        self.temp.cleanup()

    def run_fake(self, modes, stop=None, **kwargs):
        folder = self.root / "data/field_case"
        folder.mkdir()
        items = []
        for name, mode in modes.items():
            output = folder / name if name == "imu" else self.root / "data" / (name + "_fake")
            item = {"name": name, "duration_s": .3,
                    "argv": [sys.executable, str(self.child), name, str(output), mode]}
            if name == "imu":
                item["output"] = str(output)
            items.append(item)
        report = {}
        with contextlib.redirect_stdout(io.StringIO()):
            code = field.run_session(items, folder, report, stop or [False], prepare=lambda *_: None, **kwargs)
        self.assertEqual(json.loads((folder / "session.json").read_text()), report)
        return code, report, folder

    def test_normal_completion_and_external_links_without_fixed_claim(self):
        code, report, folder = self.run_fake({"rtk": "ok", "imu": "ok", "lidar": "ok"})
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "completed")
        self.assertFalse(report["sensors"][0]["evidence"]["rtk_fixed_observed"])
        self.assertTrue((folder / "rtk").is_symlink())
        self.assertTrue((folder / "lidar").is_symlink())

    def test_failure_and_zero_samples_do_not_stop_healthy_sibling(self):
        code, report, _ = self.run_fake({"rtk": "fail", "imu": "empty", "lidar": "ok"})
        self.assertEqual(code, 1)
        self.assertEqual([s["status"] for s in report["sensors"]], ["needs_review", "needs_review", "completed"])

    def test_stop_requests_cleanup_and_reaps_all_children(self):
        stop = [False]
        timer = threading.Timer(.3, lambda: stop.__setitem__(0, True))
        timer.start()
        try:
            code, report, folder = self.run_fake({"rtk": "hang", "imu": "hang", "lidar": "hang"}, stop=stop)
        finally:
            timer.join()
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "completed_by_operator")
        for sensor in report["sensors"]:
            self.assertEqual(sensor["returncode"], 130)
            self.assertEqual(sensor["signals"][0]["signal"], "SIGINT")
            self.assertTrue((folder / sensor["name"] / "cleanup.marker").is_file())

    def test_deadline_and_real_cli_plan_skip_rtk_camera_bounds(self):
        code, report, _ = self.run_fake({"imu": "hang"}, startup_margin=0)
        self.assertEqual(code, 1)
        self.assertEqual(report["sensors"][0]["signals"][0]["reason"], "overrun")
        run = subprocess.run(["bash", str(ROOT / "scripts/record_field_session.sh"), "--dry-run",
                              "--duration", "600", "--skip-rtk"], capture_output=True, text=True, check=True)
        sensors = json.loads(run.stdout)["sensors"]
        self.assertTrue(sensors[0]["disabled"])
        self.assertEqual([s["name"] for s in sensors], ["rtk", "imu", "lidar"])
        self.assertEqual(sensors[1]["duration_s"], 600)
        planned = field.plans(self.root, 600, "12345", 30)
        self.assertEqual(planned[-1]["duration_s"], 30)
        road = field.plans(self.root, 600, "12345", 600, skip_lidar=True)
        self.assertTrue(road[2]["disabled"])
        unlimited = field.plans(self.root, None)
        self.assertTrue(all(item["duration_s"] is None for item in unlimited))
        self.assertTrue(all("--until-interrupt" in item["argv"] for item in unlimited))

    def test_control_segment_keeps_encoder_and_mismatch_rows(self):
        source_dir = self.root / "data/step_manual_control_example"
        source_dir.mkdir()
        source = source_dir / "events.jsonl"
        prefix = '{"event":"old"}\n'
        source.write_text(prefix)
        snapshot = {"source": str(source), "offset": len(prefix.encode()),
                    "captured_at_kst": "test"}
        appended = [
            {"event": "status", "status": {"encoder_count": 10,
             "requested_drive": 1, "actual_drive": 0}},
            {"event": "status", "status": {"encoder_count": 17,
             "requested_drive": 0, "actual_drive": 0}},
        ]
        with source.open("a") as stream:
            for item in appended:
                stream.write(json.dumps(item) + "\n")
        folder = self.root / "data/field_control"
        folder.mkdir()
        report = {}
        field.attach_control_segment(folder, snapshot, report)
        self.assertEqual(report["control"]["status"], "completed")
        self.assertEqual(report["control"]["encoder_samples"], 2)
        self.assertEqual(report["control"]["encoder_delta"], 7)
        self.assertEqual(report["control"]["drive_mismatch_samples"], 1)
        self.assertEqual(len((folder / "control/events.jsonl").read_text().splitlines()), 2)


if __name__ == "__main__":
    unittest.main()
