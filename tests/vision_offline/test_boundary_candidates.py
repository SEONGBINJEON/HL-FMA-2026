"""Minimal checks for tools/vision_offline/boundary_candidates.py (Claude 2).
Synthetic inputs only; no device access. Run: python3 -m unittest tests.vision_offline.test_boundary_candidates
"""
import json, sys, tempfile, unittest
from pathlib import Path
import cv2, numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/vision_offline"))
import boundary_candidates as bc  # noqa: E402


def synth_frame(w=1440, h=1080, top=135, bottom=135):
    """Gray 'road' inside black letterbox with a bright diagonal 'curb' line."""
    img = np.zeros((h, w, 3), np.uint8)
    img[top:h - bottom, :] = 110
    cv2.line(img, (900, 600), (1400, 900), (230, 230, 230), 6)
    return img


class BoundaryCandidatesTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.d = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_rejects_missing_empty_and_corrupt(self):
        with self.assertRaises(ValueError):
            bc.load_input(self.d / "nope.png", None, None)
        e = self.d / "empty.png"; e.write_bytes(b"")
        with self.assertRaises(ValueError):
            bc.load_input(e, None, None)
        c = self.d / "corrupt.png"; c.write_bytes(b"\x89PNG not really a png")
        with self.assertRaises(ValueError):
            bc.load_input(c, None, None)
        v = self.d / "corrupt.mp4"; v.write_bytes(b"\x00" * 100)
        with self.assertRaises(ValueError):
            bc.load_input(v, 0, None)

    def test_margin_detection_and_coordinate_restoration(self):
        img = synth_frame()
        res, _ = bc.analyse(img, dict(bc.DEFAULTS))
        t = res["transform"]
        self.assertEqual(t["black_margins"], {"top": 135, "bottom": 135, "left": 0, "right": 0})
        self.assertEqual(t["content_rect_original_xyxy"], [0, 135, 1440, 945])
        self.assertEqual(res["status"], "candidates", res)
        c = res["candidates"][0]
        pts = c["polyline_px_original"]
        # every restored point must lie on the drawn original line y = 600 + 0.6*(x-900), within tolerance
        for x, y in pts:
            self.assertLess(abs(y - (600 + 0.6 * (x - 900))), 12, (x, y))
            self.assertTrue(900 - 15 <= x <= 1400 + 15 and 135 <= y < 945)
        self.assertEqual(c["image_position_label"], "image_right")
        self.assertEqual(res["course_left_right"], "undetermined")

    def test_flat_frame_gives_no_candidate(self):
        img = np.full((1080, 1440, 3), 110, np.uint8)
        res, _ = bc.analyse(img, dict(bc.DEFAULTS))
        self.assertEqual(res["status"], "no_candidate")
        self.assertEqual(res["candidates"], [])

    def test_overexposed_and_black_frames_are_unreadable(self):
        white = np.full((1080, 1440, 3), 250, np.uint8)
        res, _ = bc.analyse(white, dict(bc.DEFAULTS))
        self.assertEqual(res["status"], "unreadable"); self.assertIn("overexposed", res["reason"])
        black = np.zeros((1080, 1440, 3), np.uint8)
        res, _ = bc.analyse(black, dict(bc.DEFAULTS))
        self.assertEqual(res["status"], "unreadable")

    def test_run_writes_json_and_overlay_and_clip_frame_range(self):
        p = self.d / "synth.png"; cv2.imwrite(str(p), synth_frame())
        res = bc.run(p, self.d / "out")
        j = json.loads((self.d / "out/synth.json").read_text())
        self.assertEqual(j["status"], "candidates"); self.assertTrue((self.d / "out/synth_overlay.png").is_file())
        self.assertEqual(j["source"]["kind"], "image"); self.assertEqual(len(j["source"]["sha256"]), 64)
        # tiny synthetic clip: 5 frames; frame 7 must be rejected, frame 2 accepted
        vp = self.d / "synth.mp4"
        w = cv2.VideoWriter(str(vp), cv2.VideoWriter_fourcc(*"mp4v"), 30, (1440, 1080))
        for _ in range(5):
            w.write(synth_frame())
        w.release()
        if vp.stat().st_size > 0:
            with self.assertRaises(ValueError):
                bc.load_input(vp, 7, None)
            img, src = bc.load_input(vp, 2, None)
            self.assertEqual(src["kind"], "clip_frame"); self.assertEqual(src["clip_frame_index"], 2)


if __name__ == "__main__":
    unittest.main()
