import io, json, os, select, subprocess, sys, tempfile, termios, time, unittest
from contextlib import redirect_stderr
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
import steering_center_receiver as scr

LINE = "[MONITOR] ENC=0 TARGET=0 ERROR=0 MODE=HOLD | POT_RAW=519 POT=519.00 STEER_TARGET=519 STEER_CENTER=519"


def rows(values, t0=100.0, dt=0.2):
    return [{"monotonic_s": t0 + i * dt, "steering_adc": v} for i, v in enumerate(values)]


class ParseTests(unittest.TestCase):
    def test_exact_line_parses_pot_raw_as_measurement(self):
        p = scr.parse_monitor(LINE)
        self.assertEqual(p["steering_adc"], 519)
        self.assertEqual(p["steering_filtered"], 519.0)
        self.assertEqual((p["encoder"], p["hold_target"], p["hold_error"], p["mode"]), (0, 0, 0, "HOLD"))
        self.assertEqual((p["steer_target_adc"], p["steer_center_adc"]), (519, 519))
        self.assertEqual(scr.parse_monitor(LINE.encode() + b"\r\n"), p)
        drive = scr.parse_monitor(LINE.replace("ENC=0 TARGET=0 ERROR=0 MODE=HOLD", "ENC=-1234 TARGET=5 ERROR=1239 MODE=DRIVE"))
        self.assertEqual((drive["encoder"], drive["mode"]), (-1234, "DRIVE"))

    def test_other_protocol_and_banner_rejected(self):
        for bad in ("$GNGGA,000000.00,,,,,0,00,99.99,,,,,,*56", "F8 PID HILL HOLD READY",
                    "[HOLD ENTER] Target = 0", "", "MONITOR ENC=0", b"\xb5\x62\x01\x07\x00\x00",
                    LINE + " EXTRA=1", LINE.replace("MODE=HOLD", "MODE=IDLE")):
            self.assertIsNone(scr.parse_monitor(bad), bad)

    def test_ranges_and_non_finite_rejected(self):
        self.assertIsNone(scr.parse_monitor(LINE.replace("POT_RAW=519", "POT_RAW=1024")))
        self.assertIsNone(scr.parse_monitor(LINE.replace("STEER_CENTER=519", "STEER_CENTER=-1")))
        self.assertIsNone(scr.parse_monitor(LINE.replace("POT=519.00", "POT=nan")))
        self.assertIsNone(scr.parse_monitor(LINE.replace("POT=519.00", "POT=inf")))
        self.assertIsNone(scr.parse_monitor(LINE.replace("POT=519.00", "POT=1023.50")))
        self.assertIsNone(scr.parse_monitor(LINE.replace("ENC=0", "ENC=2147483648")))
        self.assertIsNotNone(scr.parse_monitor(LINE.replace("ENC=0", "ENC=-2147483648")))
        self.assertIsNotNone(scr.parse_monitor(LINE.replace("POT_RAW=519", "POT_RAW=0")))
        self.assertIsNotNone(scr.parse_monitor(LINE.replace("POT_RAW=519", "POT_RAW=1023")))


class SummarizeTests(unittest.TestCase):
    def test_median_from_real_window(self):
        r = rows([518, 519, 519, 520, 519, 519, 518, 519, 520, 519, 519, 519, 519, 519, 519])  # 15 @5Hz = 2.8s span
        s = scr.summarize_window(r, now=100.0 + 14 * 0.2 + 0.1)
        self.assertTrue(s["ok"], s)
        self.assertEqual((s["n"], s["median_adc"], s["min_adc"], s["max_adc"], s["range_adc"]), (15, 519, 518, 520, 2))
        self.assertAlmostEqual(s["span_s"], 2.8)
        self.assertFalse(s["rail"]); self.assertEqual(s["warnings"], [])
        self.assertIsNone(s["auto_still_judgement"])
        self.assertEqual(s["samples"], [x["steering_adc"] for x in r])

    def test_stale_rejected(self):
        r = rows([519] * 15)
        s = scr.summarize_window(r, now=100.0 + 14 * 0.2 + 0.7)
        self.assertEqual((s["ok"], s["reason"]), (False, "stale"))

    def test_too_few_samples_rejected(self):
        r = rows([519] * 9, dt=0.35)  # span 2.8s but only 9 rows
        s = scr.summarize_window(r, now=100.0 + 8 * 0.35 + 0.1)
        self.assertEqual((s["ok"], s["reason"]), (False, "too_few_samples"))

    def test_short_span_rejected(self):
        r = rows([519] * 12, dt=0.1)  # 1.1s span, 12 samples
        s = scr.summarize_window(r, now=100.0 + 11 * 0.1 + 0.1)
        self.assertEqual((s["ok"], s["reason"]), (False, "short_span"))

    def test_future_timestamp_rejected(self):
        r = rows([519] * 15)
        s = scr.summarize_window(r, now=100.0 + 14 * 0.2 - 0.05)
        self.assertEqual((s["ok"], s["reason"]), (False, "future_timestamp"))
        self.assertEqual(scr.summarize_window([], now=5.0)["reason"], "no_samples")

    def test_rail_and_wide_range_are_warnings_not_rejections(self):
        s = scr.summarize_window(rows([0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0]), now=100.0 + 14 * 0.2)
        self.assertTrue(s["ok"]); self.assertTrue(s["rail"]); self.assertIn("rail_0_1_or_1022_1023", s["warnings"])
        s = scr.summarize_window(rows([510, 530] * 8), now=100.0 + 15 * 0.2)
        self.assertTrue(s["ok"]); self.assertEqual(s["range_adc"], 20)
        self.assertTrue(any(w.startswith("wide_range") for w in s["warnings"]))

    def test_only_last_window_used(self):
        old = rows([100] * 15, t0=0.0)
        new = rows([519] * 15, t0=100.0)
        s = scr.summarize_window(old + new, now=100.0 + 14 * 0.2)
        self.assertEqual((s["n"], s["median_adc"]), (15, 519))


class MarkTests(unittest.TestCase):
    def setUp(self):
        self.ok = scr.summarize_window(rows([519] * 15), now=100.0 + 14 * 0.2)

    def test_mark_requires_confirmation_and_refuses_duplicates(self):
        rec, why = scr.make_mark("straight_reference", self.ok, 0, set(), confirmed=False)
        self.assertEqual((rec, why), (None, "operator_confirmation_unchecked"))
        rec, why = scr.make_mark("straight_reference", self.ok, 0, set(), confirmed=True,
                                 latest={"steer_center_adc": 519}, host_kst="2026-09-18T04:00:00+09:00")
        self.assertIsNone(why)
        self.assertTrue(rec["operator_asserted_straight"]); self.assertIsNone(rec["angle_deg"])
        self.assertFalse(rec["angle_measured"]); self.assertEqual(rec["approach_from"], None)
        self.assertEqual(rec["summary"]["median_adc"], 519); self.assertEqual(rec["steer_center_reported"], 519)
        self.assertEqual(rec["actual_steering_output"], "unverified_not_in_telemetry")
        rec2, why2 = scr.make_mark("straight_reference", self.ok, 0, {"straight_reference"}, confirmed=True)
        self.assertEqual((rec2, why2), (None, "duplicate_label_use_new_session"))
        rec3, _ = scr.make_mark("return_from_left_1", self.ok, 2, {"straight_reference"}, confirmed=True)
        self.assertEqual((rec3["approach_from"], rec3["epoch"], rec3["label_ko"]), ("left", 2, "왼쪽에서 직진 복귀 1회"))
        self.assertEqual(scr.make_mark("zero_deg", self.ok, 0, set(), True)[1], "unknown_label")
        bad = scr.summarize_window(rows([519] * 3), now=100.5)
        self.assertEqual(scr.make_mark("return_from_right_1", bad, 0, set(), True)[1], "too_few_samples")

    def test_labels_are_the_five_required(self):
        self.assertEqual(list(scr.LABELS), ["straight_reference", "return_from_left_1", "return_from_right_1",
                                            "return_from_left_2", "return_from_right_2"])


class FeedAndEpochTests(unittest.TestCase):
    def test_line_feed_handles_partial_and_junk(self):
        f = scr.LineFeed()
        self.assertEqual(f.feed(b"\xff\x00junk\r\n[MONITOR] ENC=0 TARGET=0 ERROR=0 MODE=HOLD | POT_RAW=519 POT=519.00 STEER_TARGET=519 STEER_CE"), ["�\x00junk"])
        self.assertEqual(f.feed(b"NTER=519\r\n"), [LINE])

    def test_banner_and_center_change_reset_epoch_without_serial(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "src.txt"; src.write_text("void setup(){}\n")
            s = scr.Session("/dev/null-not-opened", Path(tmp) / "out", src, True)
            # exercise file side without touching serial: replicate open() minus open_serial
            s.out.mkdir(); s.raw = (s.out / "raw_rx.bin").open("xb")
            s.samples_f = (s.out / "samples.jsonl").open("x"); s.marks_f = (s.out / "marks.jsonl").open("x")
            for i in range(15):
                s.handle_line(LINE, 100.0 + i * 0.2)
            self.assertEqual((s.epoch, len(s.rows)), (0, 15))
            s.handle_line(" F8 PID HILL HOLD READY", 103.0)
            self.assertEqual((s.epoch, len(s.rows), s.epoch_reset_flag), (1, 0, True))
            s.handle_line(LINE.replace("STEER_CENTER=519", "STEER_CENTER=530"), 103.2)
            self.assertEqual((s.epoch, len(s.rows)), (2, 1))  # center change resets, then the row itself is kept
            self.assertEqual(s.rows[0]["epoch"], 2)
            s.close()
            meta = json.loads((s.out / "session.json").read_text())
            self.assertEqual((meta["epochs"], meta["boot_banners"], meta["center_changes"]), (2, 1, 1))
            self.assertEqual(meta["transmitted_bytes"], 0); self.assertTrue(meta["os_serial_open_may_reset_board"])
            events = [json.loads(l) for l in (s.out / "samples.jsonl").read_text().splitlines()]
            self.assertEqual([e["why"] for e in events if e.get("event") == "epoch_reset"],
                             ["boot_banner", "steer_center_changed_519_to_530"])


class CliTests(unittest.TestCase):
    def test_import_does_not_load_tk_or_serial(self):
        code = ("import sys; sys.path.insert(0, 'tools'); import steering_center_receiver as m; "
                "assert 'tkinter' not in sys.modules, 'tkinter imported at module import'; "
                "assert 'serial' not in sys.modules; assert not hasattr(m, 'tk'); print('isolated')")
        r = subprocess.run([sys.executable, "-c", code], cwd=str(Path(__file__).parents[1]), capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr); self.assertIn("isolated", r.stdout)
        # Source-string check only guards against accidental write paths; it is NOT
        # proof of zero transmission. That is covered by PtyTests on a real tty.
        text = Path(scr.__file__).read_text()
        for forbidden in ("os.write(", "serial.Serial", "avrdude", "O_RDWR", "O_WRONLY"):
            self.assertNotIn(forbidden, text)

    def test_required_flags(self):
        for argv in ([], ["--receive"], ["--receive", "--battery-off-confirmed", "--reviewed-source", "x", "--output-dir", "y"]):
            with self.assertRaises(SystemExit) as cm, redirect_stderr(io.StringIO()):
                scr.main(argv)
            self.assertEqual(cm.exception.code, 2)

    def test_existing_output_dir_or_missing_source_stop_before_port_open(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "src.txt"; src.write_text("x")
            err = io.StringIO()
            with redirect_stderr(err):
                rc = scr.main(["--receive", "--battery-off-confirmed", "--reviewed-source", str(src),
                               "--output-dir", tmp, "--port", "/dev/does-not-exist"])
            self.assertEqual(rc, 1); self.assertIn("already exists", err.getvalue())
            err = io.StringIO()
            with redirect_stderr(err):
                rc = scr.main(["--receive", "--battery-off-confirmed", "--reviewed-source", str(Path(tmp) / "nope"),
                               "--output-dir", str(Path(tmp) / "new"), "--port", "/dev/does-not-exist"])
            self.assertEqual(rc, 1); self.assertIn("reviewed source not found", err.getvalue())
            self.assertFalse((Path(tmp) / "new").exists())

    def test_missing_device_records_error_and_copies_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "board.ino"; src.write_text("void loop(){}\n")
            out = Path(tmp) / "session"
            err = io.StringIO()
            with redirect_stderr(err):
                rc = scr.main(["--receive", "--battery-off-confirmed", "--reviewed-source", str(src),
                               "--output-dir", str(out), "--port", str(Path(tmp) / "no-such-tty")])
            self.assertEqual(rc, 1)
            meta = json.loads((out / "session.json").read_text())
            self.assertEqual(meta["status"], "error"); self.assertIn("no-such-tty", meta["error"])
            self.assertEqual(meta["reviewed_source"]["sha256"], scr.sha256_file(src))
            self.assertEqual(meta["reviewed_source"]["flash_identity"], "unverified")
            self.assertEqual((out / meta["reviewed_source"]["copy"]).read_text(), src.read_text())
            self.assertEqual((out / "raw_rx.bin").stat().st_size, 0)
            self.assertEqual(meta["bytes_received"], 0)


def open_pty_session(tmp, name, echo_on=True):
    """Fresh PTY standing in for the board port; slave starts with ECHO/ICANON/HUPCL ON."""
    master, slave = os.openpty()
    if echo_on:
        attrs = termios.tcgetattr(slave)
        attrs[3] |= termios.ECHO | termios.ICANON
        attrs[2] |= termios.HUPCL
        termios.tcsetattr(slave, termios.TCSANOW, attrs)
    src = Path(tmp) / "synthetic.ino"
    if not src.exists():
        src.write_text("// synthetic provenance, not uploaded firmware\n")
    return master, slave, scr.Session(os.ttyname(slave), Path(tmp) / name, src, True)


def flags(fd):
    a = termios.tcgetattr(fd)
    return {"echo": bool(a[3] & termios.ECHO), "icanon": bool(a[3] & termios.ICANON), "hupcl": bool(a[2] & termios.HUPCL)}


def readable(fd, timeout=0.2):
    r, _, _ = select.select([fd], [], [], timeout)
    return os.read(fd, 8192) if r else b""


class PtyTests(unittest.TestCase):
    """Regression for the review finding: close() must not restore ECHO/HUPCL.
    Only a PTY created here is touched; no vehicle device is opened."""

    def test_line_settings_stay_raw_no_echo_after_close_and_nothing_is_echoed(self):
        with tempfile.TemporaryDirectory() as tmp:
            master, slave, s = open_pty_session(tmp, "close")
            try:
                self.assertEqual(flags(slave), {"echo": True, "icanon": True, "hupcl": True})
                s.open()
                self.assertEqual(flags(slave), {"echo": False, "icanon": False, "hupcl": False})
                self.assertFalse(s.meta["termios_restore_on_close"])
                os.write(master, (LINE + "\r\n").encode())
                for _ in range(20):
                    s.poll()
                    if s.latest:
                        break
                    time.sleep(0.02)
                self.assertEqual(s.latest["steering_adc"], 519)
                self.assertEqual(readable(master, 0.1), b"", "board-side bytes echoed back while receiving")
                s.close()
                self.assertIsNone(s.fd)
                self.assertEqual(flags(slave), {"echo": False, "icanon": False, "hupcl": False})
                os.write(master, (LINE + "\n").encode())      # board keeps talking after we closed
                self.assertEqual(readable(master, 0.2), b"", "kernel echo re-enabled by close()")
                meta = json.loads((s.out / "session.json").read_text())
                self.assertEqual((meta["status"], meta["transmitted_bytes"]), ("completed", 0))
                self.assertGreater(meta["bytes_received"], 0)
            finally:
                os.close(slave); os.close(master)

    def test_mark_after_disconnect_is_refused_even_with_fresh_samples(self):
        with tempfile.TemporaryDirectory() as tmp:
            master, slave, s = open_pty_session(tmp, "disconnect", echo_on=False)
            try:
                s.open()
                now = time.monotonic()
                for i in range(15):
                    s.handle_line(LINE, now - 2.8 + i * 0.2)
                self.assertTrue(scr.summarize_window(s.rows, time.monotonic())["ok"])  # samples alone would pass
                self.assertTrue(s.receiving())
                s.fail("synthetic disconnect")
                self.assertFalse(s.receiving())
                before = (s.out / "marks.jsonl").stat().st_size
                rec, why = s.mark("straight_reference", True)
                self.assertEqual((rec, why), (None, "connection_error"))
                self.assertEqual(s.marks, [])
                self.assertEqual(s.meta["marks"], 0)
                self.assertEqual((s.out / "marks.jsonl").stat().st_size, before)
                s.close()
                meta = json.loads((s.out / "session.json").read_text())
                self.assertEqual((meta["status"], meta["marks"], meta["samples"]), ("error", 0, 15))
                self.assertEqual(len((s.out / "samples.jsonl").read_text().splitlines()), 15)  # evidence kept
            finally:
                os.close(slave); os.close(master)

    def test_mark_before_port_open_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "s.txt"; src.write_text("x")
            s = scr.Session("/dev/never-opened", Path(tmp) / "o", src, True)
            self.assertEqual(s.mark("straight_reference", True), (None, "not_receiving"))

    def test_eof_from_board_side_sets_error_without_reconnect(self):
        with tempfile.TemporaryDirectory() as tmp:
            master, slave, s = open_pty_session(tmp, "eof", echo_on=False)
            try:
                s.open()
                os.write(master, b"junk\n")
                self.assertTrue(s.poll())
                os.close(master); master = None
                # closing the master gives the slave EOF/EIO
                alive = True
                for _ in range(20):
                    alive = s.poll()
                    if not alive:
                        break
                    time.sleep(0.02)
                self.assertFalse(alive)
                self.assertIsNone(s.fd); self.assertEqual(s.meta["status"], "error")
                self.assertIn("disconnect", s.error + " " + s.meta["status"] + " serial")
                self.assertFalse(s.poll())   # stays dead; no reopen attempt
                s.close()
            finally:
                os.close(slave)
                if master is not None:
                    os.close(master)


if __name__ == "__main__":
    unittest.main()
