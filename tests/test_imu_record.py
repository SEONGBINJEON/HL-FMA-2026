"""Private pseudo-terminal only; no IMU or other hardware is opened."""

import contextlib
import fcntl
import io
import json
import os
from pathlib import Path
import select
import sys
import tempfile
import threading
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import imu_record as recorder


class ImuRecordTests(unittest.TestCase):
    def run_pty(self, e=b"e=-0.281,0.136,-19.507\r\n", g=b"g=-0.313,0.331,0.029\r\n",
                version=b"INTH iAHRSv1\r\nH/W Ver 1.01\r\nS/W Ver 1.21\r\n", locked=False, wrong_port=False):
        master, slave = os.openpty()
        path = os.ttyname(slave)
        sent, failures = [], []
        stop = threading.Event()
        holder = None
        def respond():
            pending = b""
            try:
                while not stop.is_set():
                    ready, _, _ = select.select([master], [], [], .02)
                    if not ready:
                        continue
                    pending += os.read(master, 1024)
                    while b"\r\n" in pending:
                        command, pending = pending.split(b"\r\n", 1)
                        sent.append(command)
                        reply = {b"vr": version, b"e": e, b"g": g}[command]
                        if reply:
                            os.write(master, reply)
            except BaseException as exc:
                failures.append(exc)
        thread = threading.Thread(target=respond, daemon=True)
        try:
            if locked:
                holder = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                fcntl.flock(holder, fcntl.LOCK_EX | fcntl.LOCK_NB)
            thread.start()
            with tempfile.TemporaryDirectory() as temporary:
                output = Path(temporary) / "record"
                ident = {"port": path, "real_port": path, "serial": "0001", "idVendor": "10c4", "idProduct": "ea60"}
                args = ["--duration", "1", "--output-dir", str(output)]
                if wrong_port:
                    args += ["--port", "/dev/not-the-imu"]
                with mock.patch.object(recorder.probe, "identity", return_value=ident), contextlib.redirect_stdout(io.StringIO()):
                    code = recorder.main(args)
                report = json.loads((output / "report.json").read_text())
                sample_path = output / "samples.jsonl"
                samples = [json.loads(line) for line in sample_path.read_text().splitlines()] if sample_path.exists() else []
                raw_path = output / "imu/raw_rx.bin"
                raw = raw_path.read_bytes() if raw_path.exists() else b""
                self.assertIn("imu_record.py", report["source_sha256"])
            self.assertFalse(failures, failures)
            return code, report, samples, raw, sent
        finally:
            stop.set()
            thread.join(timeout=1)
            if holder is not None:
                os.close(holder)
            os.close(slave)
            os.close(master)

    def test_valid_pairs_units_timestamps_and_raw_are_preserved(self):
        code, report, samples, raw, sent = self.run_pty()
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "completed")
        self.assertGreaterEqual(report["summary"]["valid_pairs"], 8)
        self.assertEqual(len(samples), report["summary"]["valid_pairs"])
        self.assertEqual(samples[0]["euler_deg"], [-.281, .136, -19.507])
        self.assertEqual(samples[0]["gyro_deg_s"], [-.313, .331, .029])
        self.assertIn("degrees/second", report["units"]["g"])
        self.assertLessEqual(samples[0]["queries"]["e"]["received_monotonic_s"], samples[0]["queries"]["g"]["sent_monotonic_s"])
        self.assertIn(b"g=-0.313,0.331,0.029", raw)
        self.assertEqual(set(sent), {b"vr", b"e", b"g"})

    def test_malformed_response_fails_and_preserves_sample_and_raw(self):
        code, report, samples, raw, sent = self.run_pty(e=b"e=1,2,3\r\ne=4,5,6\r\n")
        self.assertNotEqual(code, 0)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["summary"]["valid_pairs"], 0)
        self.assertFalse(samples[0]["valid"])
        self.assertIn(b"e=4,5,6", raw)
        self.assertEqual(sent, [b"vr", b"e"])

    def test_missing_gyro_response_is_not_zero(self):
        code, report, samples, _, _ = self.run_pty(g=b"")
        self.assertNotEqual(code, 0)
        self.assertNotIn("gyro_deg_s", samples[0])
        self.assertFalse(samples[0]["valid"])

    def test_wrong_version_stops_before_data_queries(self):
        code, report, samples, _, sent = self.run_pty(version=b"not an IMU\r\n")
        self.assertNotEqual(code, 0)
        self.assertEqual(sent, [b"vr"])
        self.assertFalse(samples)

    def test_lock_conflict_has_no_tx(self):
        code, report, samples, raw, sent = self.run_pty(locked=True)
        self.assertNotEqual(code, 0)
        self.assertFalse(sent)
        self.assertFalse(raw)
        self.assertFalse(samples)

    def test_explicit_other_port_is_not_opened(self):
        code, report, samples, raw, sent = self.run_pty(wrong_port=True)
        self.assertNotEqual(code, 0)
        self.assertFalse(sent)
        self.assertFalse(samples)

    def test_strict_vector_and_exact_version_validation(self):
        for raw in (b"", b"e=1,2", b"e=1,2,3,4", b"g=1,2,3", b"e=nan,2,3", b"e=1e999,2,3", b"e=1,2,3\x00"):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                recorder.parse_vector(raw, "e")
        self.assertFalse(recorder.recognized_version(b"INTH iAHRSv1\r\nH/W Ver 1.01\r\nS/W Ver 1.22\r\n"))

    def test_query_whitelist_rejects_settings_before_any_fd_access(self):
        unopened = object.__new__(recorder.ReadPort)
        for command in (b"mv=0\r\n", b"sd=1\r\n", b"rd\r\n", b"ca\r\n", b"\xa5\x60"):
            with self.assertRaises(ValueError):
                unopened.query(command)


if __name__ == "__main__":
    unittest.main()
