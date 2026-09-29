"""IMU power recorder checks using private pseudo-terminals, never sensors."""
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
import imu_power_record as recorder


class ImuPowerRecordTests(unittest.TestCase):
    def run_pty(self, voltage=b"pv=4.2\r\n", status=b"s=0x0\r\n", locked=False):
        master, slave = os.openpty()
        path = os.ttyname(slave)
        sent = []
        stopped = threading.Event()
        errors = []
        holder = None

        def respond():
            pending = b""
            try:
                while not stopped.is_set():
                    ready, _, _ = select.select([master], [], [], 0.05)
                    if not ready:
                        continue
                    pending += os.read(master, 4096)
                    while b"\r\n" in pending:
                        command, pending = pending.split(b"\r\n", 1)
                        sent.append(command)
                        replies = {
                            b"vr": b"INTH iAHRSv1\r\nH/W Ver 1.01\r\nS/W Ver 1.21\r\n",
                            b"pv": voltage,
                            b"s": status,
                        }
                        if command not in replies:
                            raise AssertionError(f"Unexpected query: {command!r}")
                        if replies[command]:
                            os.write(master, replies[command])
            except BaseException as exc:
                errors.append(exc)

        thread = threading.Thread(target=respond, daemon=True)
        try:
            if locked:
                holder = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                fcntl.flock(holder, fcntl.LOCK_EX | fcntl.LOCK_NB)
            thread.start()
            with tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                (root / "data").mkdir()
                identity = {"port": path, "real_port": path, "serial": "0001",
                            "idVendor": "10c4", "idProduct": "ea60", "test_only": True}
                with mock.patch.object(recorder.probe, "ROOT", root), \
                        mock.patch.object(recorder.probe, "identity", return_value=identity) as identify, \
                        contextlib.redirect_stdout(io.StringIO()):
                    code = recorder.main(["--duration", "1"])
                identify.assert_called_once_with("0001")
                folder, = (root / "data").iterdir()
                report = json.loads((folder / "report.json").read_text())
                events = [json.loads(line) for line in (folder / "imu/events.jsonl").read_text().splitlines()]
                raw = (folder / "imu/raw_rx.bin").read_bytes()
                sample_file = folder / "samples.jsonl"
                samples = [json.loads(line) for line in sample_file.read_text().splitlines()] if sample_file.exists() else []
            self.assertFalse(errors, errors)
            return code, report, samples, events, raw, sent
        finally:
            stopped.set()
            if thread.ident is not None:
                thread.join(timeout=1)
            if holder is not None:
                os.close(holder)
            os.close(slave)
            os.close(master)

    def test_normal_response_and_raw_queries_are_preserved(self):
        code, report, samples, events, raw, sent = self.run_pty()
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["summary"]["valid_samples"], 1)
        self.assertEqual(report["summary"]["voltage_warning_samples"], 0)
        self.assertEqual(report["summary"]["voltage_min_v"], 4.2)
        self.assertEqual(samples, report["samples"])
        self.assertEqual(sent, [b"vr", b"pv", b"s"])
        self.assertIn(b"pv=4.2\r\n", raw)
        self.assertEqual([bytes.fromhex(e["hex"]) for e in events if e["kind"] == "tx_completed"],
                         [b"vr\r\n", b"pv\r\n", b"s\r\n"])

    def test_voltage_warning_is_preserved_in_sample_and_summary(self):
        code, report, samples, _, raw, _ = self.run_pty(b"pv=3.6\r\n", b"s=0x20000\r\n")
        self.assertEqual(code, 0)  # Completed measurement, not a healthy-power assertion.
        self.assertTrue(samples[0]["voltage_warning"])
        self.assertEqual(samples[0]["status_flags"], 0x20000)
        self.assertEqual(report["summary"]["voltage_warning_samples"], 1)
        self.assertEqual(report["summary"]["nonzero_status_samples"], 1)
        self.assertIn(b"s=0x20000\r\n", raw)

    def test_malformed_status_fails_and_preserves_response(self):
        code, report, samples, _, raw, _ = self.run_pty(status=b"s=0x20000\r\ns=0x0\r\n")
        self.assertNotEqual(code, 0)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["summary"]["valid_samples"], 0)
        self.assertIn("error", samples[0])
        self.assertIn("s=0x20000", samples[0]["s_response"])
        self.assertIn(b"s=0x20000\r\ns=0x0\r\n", raw)

    def test_exclusive_lock_conflict_sends_nothing(self):
        code, report, samples, events, raw, sent = self.run_pty(locked=True)
        self.assertNotEqual(code, 0)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["summary"]["valid_samples"], 0)
        self.assertEqual(samples, [])
        self.assertEqual(events, [])
        self.assertEqual(raw, b"")
        self.assertEqual(sent, [])

    def test_parser_rejects_missing_or_unrecognized_values(self):
        for pv, status in [("", "s=0x0"), ("pv=NaN", "s=0x0"),
                           ("pv=4.2", ""), ("pv=4.2", "s=0xg"),
                           ("pv=4.2\x00", "s=0x0"), ("pv=4.2", "s=0")]:
            with self.subTest(pv=pv, status=status):
                with self.assertRaises(ValueError):
                    recorder.parse_sample(pv, status)


if __name__ == "__main__":
    unittest.main()
