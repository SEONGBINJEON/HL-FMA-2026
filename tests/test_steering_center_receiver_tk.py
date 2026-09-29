"""Tk controller checks for steering_center_receiver (needs a display; skipped otherwise).

A pipe stands in for the serial fd. Synthetic monitor lines are NOT measurements.
No vehicle device is opened. tkinter is imported inside the test only.
"""
import json, os, sys, tempfile, time, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
import steering_center_receiver as scr

LINE = b"[MONITOR] ENC=0 TARGET=0 ERROR=0 MODE=HOLD | POT_RAW=%d POT=519.00 STEER_TARGET=519 STEER_CENTER=519\r\n"


@unittest.skipUnless(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"), "no display for Tk")
class TkControllerTests(unittest.TestCase):
    def setUp(self):
        import tkinter
        self.tmp = tempfile.TemporaryDirectory(prefix="scr_tk_")
        tmp = Path(self.tmp.name)
        src = tmp / "src.txt"; src.write_text("synthetic\n")
        s = scr.Session("pipe://synthetic", tmp / "out", src, True)
        s.out.mkdir(); s.raw = (s.out / "raw_rx.bin").open("xb")
        s.samples_f = (s.out / "samples.jsonl").open("x"); s.marks_f = (s.out / "marks.jsonl").open("x")
        self.r_fd, self.w_fd = os.pipe(); s.fd = self.r_fd; s.meta["status"] = "receiving"
        self.session = s
        try:
            self.root = tkinter.Tk()
        except tkinter.TclError as exc:
            self.skipTest(f"Tk unavailable: {exc}")
        self.ctrl = scr.WindowController(self.root, s)

    def tearDown(self):
        try:
            self.root.destroy()
        except Exception:
            pass
        for fd in (self.w_fd,):
            try:
                os.close(fd)
            except OSError:
                pass
        if self.session.raw is not None:      # closes fd + evidence files (no termios restore)
            self.session.close()
        self.tmp.cleanup()

    def feed_window(self, n=15):
        """Write n synthetic lines with spaced monotonic stamps (no real waiting)."""
        now = time.monotonic()
        for i in range(n):
            self.session.handle_line(LINE.decode().strip() % (519 + i % 2), now - 2.8 + i * 0.2)

    def states(self):
        return {k: str(b.cget("state")) for k, b in self.ctrl.buttons.items()}

    def test_mark_flow_then_disconnect_disables_marking(self):
        c, s = self.ctrl, self.session
        os.write(self.w_fd, b"\xff\x00junk\r\n F8 PID HILL HOLD READY\r\n" + LINE % 519)
        c.tick(); self.root.update()
        self.assertEqual((s.epoch, s.meta["boot_banners"]), (1, 1))
        self.assertIn("epoch=1", c.v_status.get())
        self.feed_window(); c.tick(); self.root.update()
        self.assertEqual(c.v_raw.get(), "519"); self.assertNotIn("STALE", c.v_age.get())
        c.buttons["straight_reference"].invoke()
        self.assertIn("operator_confirmation_unchecked", c.v_status.get()); self.assertEqual(s.marks, [])
        c.check.invoke()                                   # operator confirms
        c.buttons["straight_reference"].invoke()
        self.assertEqual(len(s.marks), 1); self.assertIn("기록됨", c.v_status.get())
        self.assertFalse(c.confirmed.get())                 # auto-cleared
        self.assertEqual(self.states()["straight_reference"], "disabled")
        self.assertEqual(s.marks[0]["summary"]["median_adc"], 519)
        # disconnect: writer closes -> EOF on next tick
        os.close(self.w_fd)
        alive = c.tick(); self.root.update()
        self.assertFalse(alive); self.assertIsNone(s.fd)
        self.assertIn("오류", c.v_status.get()); self.assertEqual(c.v_raw.get(), "ERR")
        self.assertFalse(c.confirmed.get())
        self.assertEqual(set(self.states().values()), {"disabled"})
        self.assertEqual(str(c.check.cget("state")), "disabled")
        # even forcing the callbacks after the error adds nothing
        c.confirmed.set(True); c.on_mark("return_from_left_1")
        self.assertIn("connection_error", c.v_status.get()); self.assertEqual(len(s.marks), 1)
        self.assertFalse(c.tick())                         # stays dead
        s.close()
        meta = json.loads((s.out / "session.json").read_text())
        self.assertEqual((meta["status"], meta["marks"], meta["transmitted_bytes"]), ("error", 1, 0))
        self.assertEqual(len((s.out / "marks.jsonl").read_text().splitlines()), 1)

    def test_stale_samples_are_shown_as_stale_and_not_marked(self):
        c, s = self.ctrl, self.session
        now = time.monotonic()
        for i in range(15):
            s.handle_line(LINE.decode().strip() % 519, now - 4.0 + i * 0.2)   # newest 1.2s old
        c.tick(); self.root.update()
        self.assertIn("STALE", c.v_raw.get()); self.assertIn("STALE", c.v_age.get())
        c.check.invoke(); c.buttons["straight_reference"].invoke()
        self.assertIn("stale", c.v_status.get()); self.assertEqual(s.marks, [])
        self.assertTrue(c.confirmed.get())                  # refused mark keeps the operator's check
        self.assertEqual(self.states()["straight_reference"], "normal")


if __name__ == "__main__":
    unittest.main()
