"""No hardware: checksum fixtures and local PTY query/lock checks."""
import fcntl
import json
import os
from pathlib import Path
import pty
import select
import struct
import sys
import tempfile
import termios
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import gnss_diagnostics as diag


def frame(name, payload):
    body = bytes(diag.POLL_IDS[name]) + len(payload).to_bytes(2, "little") + payload
    return b"\xb5\x62" + body + diag.checksum(body)


def pvt():
    p = bytearray(92)
    p[20], p[21], p[23] = 3, 0x83, 31
    p[40:44] = (19).to_bytes(4, "little")
    return frame("NAV-PVT", p)


class DiagnosticsTests(unittest.TestCase):
    def test_only_allowlisted_empty_polls_and_expected_checksums(self):
        self.assertEqual(diag.poll_bytes("MON-VER").hex(), "b5620a0400000e34")
        self.assertEqual(diag.poll_bytes("NAV-PVT").hex(), "b562010700000819")
        for name in diag.POLL_IDS:
            self.assertEqual(len(diag.poll_bytes(name)), 8)
            self.assertEqual(diag.poll_bytes(name)[4:6], b"\0\0")
        with self.assertRaises(KeyError):
            diag.poll_bytes("CFG-RST")

    def test_fragmented_corrupt_binary_cannot_inject_valid_frame(self):
        nested = bytearray(frame("MON-VER", pvt()))
        nested[-1] ^= 1
        stream = diag.UbxStream()
        found = []
        for b in b"$GNGGA,irrelevant\r\n" + nested + pvt():
            found.extend(stream.feed(bytes((b,))))
        self.assertEqual(found, [pvt()])
        self.assertEqual(stream.invalid_checksums, 1)
        self.assertIsNone(stream.incomplete())
        stream.feed(b"\xb5\x62\x01\x07\x5c\x00\x12")
        self.assertEqual(stream.incomplete(), {"buffered_bytes": 7, "expected_frame_bytes": 100})

    def test_decode_version_pvt_sat_signal_and_reject_unknown_versions(self):
        version = frame("MON-VER", b"EXT CORE 1.32".ljust(30, b"\0") + b"00190000".ljust(10, b"\0") + b"PROTVER=27.31".ljust(30, b"\0"))
        self.assertEqual(diag.decode_frame(version)["extensions"], ["PROTVER=27.31"])
        row = diag.decode_frame(pvt())
        self.assertEqual(row["carrier_solution"], "fixed")
        self.assertTrue(row["gnss_fix_ok"])
        self.assertEqual(row["satellites_used"], 31)
        self.assertEqual(row["horizontal_accuracy_estimate_m"], 0.019)
        self.assertNotIn("latitude", row)
        sat = struct.pack("<IBBH", 123, 1, 1, 0) + struct.pack("<BBBbhhI", 0, 22, 42, 55, 123, 0, 15)
        row = diag.decode_frame(frame("NAV-SAT", sat))
        self.assertEqual(row["satellites_used_count"], 1)
        self.assertEqual(row["observations"][0]["cno_dbhz"], 42)
        sig = struct.pack("<IBBH", 123, 0, 1, 0) + struct.pack("<BBBBhBBBBHI", 0, 22, 3, 0, 0, 43, 7, 2, 0, 0xD8, 0)
        row = diag.decode_frame(frame("NAV-SIG", sig))
        self.assertEqual(row["observations"][0]["signal_name"], "GPS L2 CL")
        self.assertTrue(row["observations"][0]["carrier_correction_used"])
        with self.assertRaises(ValueError):
            diag.decode_frame(frame("NAV-SIG", sig[:4] + b"\x01" + sig[5:]))
        with self.assertRaises(ValueError):
            diag.decode_frame(pvt()[:-1] + b"\0")

    def test_pty_only_queries_recorded_and_host_attributes_restored(self):
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        original = termios.tcgetattr(slave)
        observed = bytearray()
        stop = threading.Event()

        def receiver():
            while not stop.is_set():
                ready, _, _ = select.select([master], [], [], .05)
                if ready:
                    chunk = os.read(master, 1024)
                    observed.extend(chunk)
                    if diag.poll_bytes("NAV-PVT") in observed:
                        os.write(master, pvt())

        worker = threading.Thread(target=receiver, daemon=True)
        worker.start()
        try:
            with tempfile.TemporaryDirectory() as td:
                output = Path(td) / "session"
                result = diag.record(port, output, duration=2)
                self.assertEqual(result["status"], "completed")
                self.assertEqual(result["tx_bytes"], 32)
                self.assertEqual(result["latest"]["NAV-PVT"]["satellites_used"], 31)
                self.assertEqual(bytes(observed), b"".join(diag.poll_bytes(n) for n in diag.POLL_IDS))
                self.assertEqual(result["no_response_messages"], ["MON-VER", "NAV-SAT", "NAV-SIG"])
                self.assertTrue((output / "raw.bin").read_bytes())
                self.assertTrue((output / "tx.jsonl").read_text())
                self.assertTrue(result["host_serial_settings_restored"])
                self.assertEqual(termios.tcgetattr(slave), original)
        finally:
            stop.set()
            worker.join(1)
            os.close(master)
            os.close(slave)

    def test_lock_denied_before_host_settings_or_query_write(self):
        master, slave = pty.openpty()
        original = termios.tcgetattr(slave)
        fcntl.flock(slave, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            with tempfile.TemporaryDirectory() as td:
                result = diag.record(os.ttyname(slave), Path(td) / "session", duration=2)
                self.assertEqual(result["status"], "failed")
                self.assertEqual(result["tx_bytes"], 0)
                self.assertEqual(termios.tcgetattr(slave), original)
                self.assertFalse(select.select([master], [], [], .05)[0])
        finally:
            os.close(master)
            os.close(slave)

    def test_no_response_is_completed_observation_not_fault(self):
        master, slave = pty.openpty()
        try:
            with tempfile.TemporaryDirectory() as td:
                result = diag.record(os.ttyname(slave), Path(td) / "session", duration=2)
                self.assertEqual(result["status"], "completed")
                self.assertEqual(result["no_response_messages"], list(diag.POLL_IDS))
        finally:
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    unittest.main()
