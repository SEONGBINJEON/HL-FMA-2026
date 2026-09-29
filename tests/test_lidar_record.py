"""Only local temporary files, synthetic children and PTY; never real hardware."""
import fcntl
import io
import json
import os
from pathlib import Path
import pty
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import lidar_record as lidar


class LidarRecordingTests(unittest.TestCase):
    def test_parser_fragmentation_ranges_markers_and_invalid_lines(self):
        scans = io.StringIO()
        summary = lidar.NodeSummary(scans)
        summary.feed(b"SDK header\n   theta: 000.00 Dist: 00000.00 Q: 0 \nS  theta: 010.", .1)
        summary.feed(b"00 Dist: 00123.25 Q: 10 \n   theta: 350.00 Dist: 00456.00 Q: 63 \n", .3)
        summary.feed(b"S  theta: 010.00 Dist: 00200.00 Q: 5 \n   theta: 999.00 Dist: 00100.00 Q: 4 \n   theta: 010.00 Dist: 00100.00 Q: 99 \n", .7)
        result = summary.result()
        self.assertEqual(result["parsed_nodes"], 4)
        self.assertEqual(result["positive_distance_nodes"], 3)
        self.assertEqual(result["positive_distance_range_mm"], [123.25, 456])
        self.assertEqual(result["angle_range_deg"], [0, 350])
        self.assertEqual(result["malformed_measurement_lines"], 2)
        self.assertEqual(result["closed_marker_to_marker_candidates"], 1)
        self.assertEqual(result["nodes_before_first_marker"], 1)
        self.assertAlmostEqual(result["max_node_host_receipt_gap_s"], .4)
        self.assertEqual(json.loads(scans.getvalue())["nodes"], 2)
        self.assertEqual(result["trailing_unclosed_candidate"]["nodes"], 1)

    def fake_usb(self, root, name, serial, vendor="10c4", product="ea60"):
        by_id, tty = root / "by-id", root / "tty"
        by_id.mkdir(exist_ok=True)
        tty.mkdir(exist_ok=True)
        dev = root / "dev" / name
        dev.parent.mkdir(exist_ok=True)
        dev.touch()
        usb = root / "usb" / name
        child = usb / "interface" / "port"
        child.mkdir(parents=True)
        for key, value in {"idVendor": vendor, "idProduct": product, "serial": serial}.items():
            (usb / key).write_text(value)
        (tty / name).mkdir()
        (tty / name / "device").symlink_to(child)
        (by_id / f"usb-CP2102-{serial}-{name}").symlink_to(dev)
        return by_id, tty

    def test_identity_rejects_imu_and_wrong_vendor_even_with_lidar_serial(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            by_id, tty = self.fake_usb(root, "ttyUSB0", "0001")
            with self.assertRaisesRegex(RuntimeError, "found 0"):
                lidar.identity(by_id, tty)
            self.fake_usb(root, "ttyUSB1", lidar.SERIAL, vendor="2341")
            with self.assertRaisesRegex(RuntimeError, "found 0"):
                lidar.identity(by_id, tty)
            self.fake_usb(root, "ttyUSB2", lidar.SERIAL)
            self.assertTrue(lidar.identity(by_id, tty)["real_port"].endswith("ttyUSB2"))

    def test_identity_rejects_duplicate_matches(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            by_id, tty = self.fake_usb(root, "ttyUSB0", lidar.SERIAL)
            self.fake_usb(root, "ttyUSB1", lidar.SERIAL)
            with self.assertRaisesRegex(RuntimeError, "found 2"):
                lidar.identity(by_id, tty)

    def test_duration_sigint_preserves_raw_and_host_chunk_offsets(self):
        child = """import signal, time, sys
stop = False
def quit(*_):
    global stop
    stop = True
signal.signal(signal.SIGINT, quit)
sys.stdout.buffer.write(b'header\\xff\\nS  theta: 001.00 Dist: 00123.25 Q: 12 \\n')
sys.stdout.flush()
sys.stderr.write('diagnostic\\n'); sys.stderr.flush()
while not stop: time.sleep(.01)
print('cleanup reached', flush=True)
"""
        with tempfile.TemporaryDirectory() as td:
            folder = Path(td)
            result = lidar.capture([sys.executable, "-c", child], folder, .4, grace=1)
            self.assertEqual(result["status"], "completed")
            self.assertFalse(result["physical_motor_stop_verified"])
            self.assertTrue(result["sdk_cleanup_path_expected"])
            self.assertEqual([x["signal"] for x in result["signals"]], ["SIGINT"])
            raw = (folder / "stdout.bin").read_bytes()
            self.assertIn(b'header\xff\n', raw)
            self.assertTrue(raw.endswith(b"cleanup reached\n"))
            offsets = {"stdout": 0, "stderr": 0}
            for row in map(json.loads, (folder / "host_chunks.jsonl").read_text().splitlines()):
                self.assertEqual(row["offset"], offsets[row["stream"]])
                self.assertGreaterEqual(row["host_elapsed_s"], 0)
                offsets[row["stream"]] += row["bytes"]
            self.assertEqual(offsets, result["stream_bytes"])

    def test_ignored_signals_force_bounded_kill_and_never_claim_cleanup(self):
        child = "import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); print('S  theta: 000.00 Dist: 00010.00 Q: 1 ',flush=True); time.sleep(20)"
        with tempfile.TemporaryDirectory() as td:
            start = time.monotonic()
            result = lidar.capture([sys.executable, "-c", child], Path(td), .4, grace=.15, terminate_grace=.15)
            self.assertLess(time.monotonic() - start, 3)
            self.assertEqual([x["signal"] for x in result["signals"]], ["SIGINT", "SIGTERM", "SIGKILL"])
            self.assertEqual(result["status"], "needs_review")
            self.assertTrue(result["forced_termination"])
            self.assertFalse(result["sdk_cleanup_path_expected"])

    def test_early_zero_exit_without_measurements_is_not_success(self):
        with tempfile.TemporaryDirectory() as td:
            result = lidar.capture([sys.executable, "-c", "print('Error, cannot bind serial')"], Path(td), 2)
            self.assertEqual(result["returncode"], 0)
            self.assertEqual(result["status"], "needs_review")
            self.assertEqual(result["signals"], [])

    def test_same_pty_advisory_lock_is_retained_in_child(self):
        master, slave = pty.openpty()
        try:
            fcntl.flock(slave, fcntl.LOCK_EX | fcntl.LOCK_NB)
            child = """import os,fcntl,sys
fd=os.open(sys.argv[1],os.O_RDWR|os.O_NOCTTY)
try:
    fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    print('unexpectedly acquired',flush=True)
except BlockingIOError:
    print('lock retained',flush=True)
os.close(fd)
"""
            with tempfile.TemporaryDirectory() as td:
                folder = Path(td)
                lidar.capture([sys.executable, "-c", child, os.ttyname(slave)], folder, 2, pass_fds=(slave,))
                self.assertEqual((folder / "stdout.bin").read_bytes(), b"lock retained\n")
        finally:
            os.close(slave)
            os.close(master)


if __name__ == "__main__":
    unittest.main()
