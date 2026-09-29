"""No hardware needed: parser fixtures and an actual receive-only PTY session."""

import importlib.util
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import termios
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "gnss_record.py"
SPEC = importlib.util.spec_from_file_location("gnss_record", SCRIPT)
gnss = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gnss)


def nmea(payload):
    data = payload.encode("ascii")
    checksum = 0
    for byte in data:
        checksum ^= byte
    return b"$" + data + f"*{checksum:02X}\r\n".encode()


def gga(quality="4"):
    return nmea(f"GNGGA,123519.00,3728.0000,N,12706.0000,E,{quality},12,0.7,50.0,M,20.0,M,1.5,0001")


def ubx(payload):
    body = b"\x01\x07" + len(payload).to_bytes(2, "little") + payload
    a = b = 0
    for byte in body:
        a = (a + byte) & 255
        b = (b + a) & 255
    return b"\xb5\x62" + body + bytes((a, b))


class ParserTests(unittest.TestCase):
    def test_quality_is_explicit_and_coordinates_are_converted(self):
        for quality, status, rtk in (("4", "rtk_fixed", "fixed"), ("5", "rtk_float", "float"),
                                     ("2", "dgps_fix", "unknown"), ("9", "unknown", "unknown"),
                                     ("", "unknown", "unknown"), ("0", "invalid", "unknown")):
            with self.subTest(quality=quality):
                row = gnss.parse_gga(gga(quality), "host-time", 1.0)
                self.assertEqual(row["fix_status"], status)
                self.assertEqual(row["rtk_state"], rtk)
                self.assertAlmostEqual(row["latitude_deg"], 37 + 28 / 60)
                self.assertEqual(row["longitude_deg"], 127.1)
                self.assertEqual(row["receiver_utc_time"], "123519.00")
                self.assertEqual(row["host_utc"], "host-time")

    def test_invalid_and_missing_checksum_are_rejected(self):
        self.assertIsNone(gnss.parse_gga(gga().replace(b"123519", b"123518")))
        self.assertIsNone(gnss.parse_gga(b"$GNGGA,123519\r\n"))
        self.assertFalse(gnss.checksum_ok(b"$foo*+1\r\n"))

    def test_empty_no_fix_sentence_does_not_invent_coordinates(self):
        row = gnss.parse_gga(nmea("GNGGA,123519,,,,,0,00,99.99,,,,,,"))
        self.assertIsNone(row["latitude_deg"])
        self.assertIsNone(row["longitude_deg"])
        self.assertEqual(row["fix_status"], "invalid")

    def test_mixed_binary_and_fragmented_nmea(self):
        extractor = gnss.MixedNmeaStream()
        # A checksum-valid GGA hidden INSIDE UBX must not be emitted.
        data = b"noise\x00\xff" + ubx(gga("5")) + gga("4") + b"$broken" + gga("2")
        output = []
        for byte in data:
            output.extend(extractor.feed(bytes((byte,))))
        self.assertEqual(output, [gga("4"), gga("2")])
        self.assertEqual(extractor.ubx_frames, 1)

    def test_corrupt_binary_and_bad_nmea_resynchronize(self):
        extractor = gnss.MixedNmeaStream()
        bad_ubx = bytearray(ubx(b"test"))
        bad_ubx[-1] ^= 1
        bad_nmea = gga().replace(b"123519", b"123518")
        self.assertEqual(extractor.feed(bad_ubx + bad_nmea + gga()), [gga()])
        self.assertEqual(extractor.invalid_ubx_frames, 1)
        self.assertEqual(extractor.invalid_nmea_checksums, 1)

    def test_corrupt_ubx_payload_cannot_fabricate_rtk_fixed(self):
        extractor = gnss.MixedNmeaStream()
        corrupt = bytearray(ubx(gga("4")))
        corrupt[-1] ^= 1
        received = extractor.feed(corrupt + gga("2"))
        self.assertEqual(received, [gga("2")])
        self.assertEqual(gnss.parse_gga(received[0])["rtk_state"], "unknown")
        self.assertEqual(extractor.invalid_ubx_frames, 1)

    def test_long_incomplete_ubx_is_reported_without_trusting_its_payload(self):
        extractor = gnss.MixedNmeaStream()
        self.assertEqual(extractor.feed(b"\xb5\x62\x01\x07\xff\xff" + gga("4")), [])
        self.assertEqual(extractor.incomplete_ubx()["expected_frame_bytes"], 65543)
        self.assertEqual(extractor.incomplete_ubx()["buffered_bytes"], 6 + len(gga("4")))

    def test_invalid_coordinates_and_nonfinite_values(self):
        self.assertIsNone(gnss.coordinate("9060.0", "N", True))
        self.assertIsNone(gnss.coordinate("3728.0", "E", True))
        self.assertEqual(gnss.coordinate("1230", "S", True), -12.5)
        self.assertIsNone(gnss.optional_float("nan"))


@unittest.skipUnless(sys.platform.startswith("linux"), "Linux termios/PTY integration")
class RecorderIntegrationTests(unittest.TestCase):
    def test_second_reader_is_locked_before_termios_changes_and_close_releases(self):
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        first = reopened = None
        original = termios.tcgetattr(slave)
        try:
            first, _ = gnss.open_serial(port, 115200)
            with mock.patch.object(gnss.termios, "tcsetattr") as set_attributes:
                with self.assertRaisesRegex(BlockingIOError, "다른 GNSS/RTK 프로그램이 사용 중"):
                    gnss.open_serial(port, 9600)
                set_attributes.assert_not_called()
            os.close(first)
            first = None
            reopened, _ = gnss.open_serial(port, 115200)
            self.assertTrue(os.isatty(reopened))
        finally:
            termios.tcsetattr(slave, termios.TCSANOW, original)
            if first is not None:
                os.close(first)
            if reopened is not None:
                os.close(reopened)
            os.close(master)
            os.close(slave)

    def test_no_bytes_fails_but_ubx_only_is_preserved_with_unknown_quality(self):
        cases = [
            (b"", "no_data", 3, False),
            (ubx(b"binary only"), "completed", 0, False),
            (b"\xb5\x62\x01\x07\xff\xff" + gga("4"), "completed", 0, True),
        ]
        for payload, status, exit_code, blocked in cases:
            with self.subTest(status=status, blocked=blocked), tempfile.TemporaryDirectory() as temp:
                output = Path(temp) / "session"
                master, slave = pty.openpty()
                proc = subprocess.Popen(
                    [sys.executable, str(SCRIPT), "--port", os.ttyname(slave),
                     "--duration", "0.2", "--output-dir", str(output)],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                try:
                    self.assertTrue(select.select([proc.stdout], [], [], 3)[0])
                    self.assertIn("Recording receive-only", proc.stdout.readline())
                    if payload:
                        os.write(master, payload)
                    _, stderr = proc.communicate(timeout=3)
                    self.assertEqual(proc.returncode, exit_code, stderr)
                    metadata = json.loads((output / "metadata.json").read_text())
                    self.assertEqual(metadata["status"], status)
                    self.assertEqual(metadata["blocked_on_incomplete_ubx"], blocked)
                    self.assertEqual(metadata["gga_records"], 0)
                    self.assertEqual((output / "raw.bin").read_bytes(), payload)
                    self.assertEqual((output / "gga.jsonl").read_text(), "")
                    self.assertIn("RTK quality is unknown", stderr)
                finally:
                    if proc.poll() is None:
                        proc.kill()
                        proc.communicate()
                    os.close(master)
                    os.close(slave)

    def run_session(self, interrupted=False, disconnected=False):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "session"
            master, slave = pty.openpty()
            port = os.ttyname(slave)
            proc = subprocess.Popen(
                [sys.executable, str(SCRIPT), "--port", port, "--duration", "0.5",
                 "--output-dir", str(output)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                self.assertTrue(select.select([proc.stdout], [], [], 3)[0], "recorder did not start")
                self.assertIn("Recording receive-only", proc.stdout.readline())
                payload = b"\x00\xff" + ubx(b"GNSS binary") + gga("4") + gga("2")
                os.write(master, payload[:19])
                os.write(master, payload[19:])
                if interrupted or disconnected:
                    # Wait for flushed data before interrupting the process.
                    import time
                    deadline = time.monotonic() + 2
                    while (output / "raw.bin").stat().st_size != len(payload) and time.monotonic() < deadline:
                        time.sleep(0.01)
                    self.assertEqual((output / "raw.bin").stat().st_size, len(payload))
                    if interrupted:
                        proc.send_signal(signal.SIGINT)
                    else:
                        os.close(master)
                        master = None
                stdout, stderr = proc.communicate(timeout=3)
                self.assertEqual(proc.returncode, 130 if interrupted else (1 if disconnected else 0), stderr)
                self.assertEqual((output / "raw.bin").read_bytes(), payload)
                rows = [json.loads(line) for line in (output / "gga.jsonl").read_text().splitlines()]
                self.assertEqual([row["rtk_state"] for row in rows], ["fixed", "unknown"])
                metadata = json.loads((output / "metadata.json").read_text())
                self.assertEqual(metadata["status"], "interrupted" if interrupted else ("error" if disconnected else "completed"))
                self.assertEqual(metadata["bytes_received"], len(payload))
                self.assertEqual(metadata["valid_ubx_frames"], 1)
                self.assertEqual(metadata["gga_records"], 2)
                self.assertIn("ended_utc", metadata)
                # No receiver command is ever written to the other PTY end.
                if master is not None:
                    self.assertFalse(select.select([master], [], [], 0)[0])
                    # A normal finish or Ctrl-C must release the advisory lock.
                    reopened, attributes = gnss.open_serial(port, 115200)
                    try:
                        self.assertTrue(os.isatty(reopened))
                    finally:
                        termios.tcsetattr(reopened, termios.TCSANOW, attributes)
                        os.close(reopened)
                original = (output / "raw.bin").read_bytes()
                repeated = subprocess.run(
                    [sys.executable, str(SCRIPT), "--port", port, "--output-dir", str(output)],
                    capture_output=True, text=True, timeout=3)
                self.assertNotEqual(repeated.returncode, 0)
                self.assertIn("already exists", repeated.stderr)
                self.assertEqual((output / "raw.bin").read_bytes(), original)
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.communicate()
                if master is not None:
                    os.close(master)
                os.close(slave)

    def test_duration_raw_bytes_parsing_no_transmission_and_collision(self):
        self.run_session()

    def test_ctrl_c_finalizes_files(self):
        self.run_session(interrupted=True)

    def test_disconnection_finalizes_metadata(self):
        self.run_session(disconnected=True)

    def test_missing_device_is_clear_and_does_not_create_output(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "session"
            result = subprocess.run(
                [sys.executable, str(SCRIPT), "--port", str(Path(temp) / "no-device"),
                 "--output-dir", str(output)], capture_output=True, text=True, timeout=3)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("Cannot start GNSS recorder", result.stderr)
            self.assertIn("No such file", result.stderr)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
