"""Offline antenna preview: coordinate axes and discontinuity preservation."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import route_preview as route
from gnss_record import parse_gga


def row(t, quality=4, latitude="3500.00000"):
    payload = f"GNGGA,120000.00,{latitude},N,12800.00000,E,{quality},12,0.7,60.0,M,20.0,M,1.0,0001"
    checksum = 0
    for byte in payload.encode("ascii"):
        checksum ^= byte
    sentence = f"${payload}*{checksum:02X}"
    return parse_gga(sentence.encode("ascii"), "2026-09-05T12:00:00+00:00", t)


def raw(rows):
    return ("\n".join(json.dumps(value) if not isinstance(value, str) else value for value in rows) + "\n").encode()


class RoutePreviewTests(unittest.TestCase):
    def test_wgs84_axes_and_enu_rotation(self):
        self.assertEqual(route.ecef(0, 0), (6378137.0, 0.0, 0.0))
        self.assertAlmostEqual(route.ecef(90, 0)[2], 6356752.314245179, places=6)
        self.assertEqual(route.enu(35, 128, (35, 128)), (0.0, 0.0, 0.0))
        east = route.enu(0, math.degrees(1 / route.A), (0, 0))
        self.assertAlmostEqual(east[0], 1.0, places=7)
        self.assertAlmostEqual(east[1], 0.0, places=7)
        north = route.enu(.00001, 0, (0, 0))
        self.assertAlmostEqual(north[1], 1.105742758, places=7)
        self.assertAlmostEqual(north[0], 0.0, places=7)
        self.assertLess(route.enu(0, -0.00001, (0, 0))[0], 0)
        self.assertLess(route.enu(-0.00001, 0, (0, 0))[1], 0)

    def test_quality_transitions_split_and_float_has_no_line(self):
        points, segments, summary = route.analyze(raw([row(0), row(1), row(2, 5), row(3, 5), row(4), row(5)]))
        self.assertEqual([s["samples"] for s in segments], [2, 2, 2])
        self.assertEqual([p["segment_id"] for p in points], [1, 1, 2, 2, 3, 3])
        image = route.svg(points, summary)
        self.assertEqual(image.count("<polyline"), 2)
        self.assertNotIn('stroke="#c56c00"', image)

    def test_gaps_reversals_duplicates_and_missing_time_break(self):
        bad = row(5)
        bad["elapsed_s"] = None
        points, segments, _ = route.analyze(raw([row(0), row(1), row(4), row(3), row(3), bad, row(6), row(7)]))
        self.assertEqual([s["samples"] for s in segments], [2, 1, 1, 1, 2])
        self.assertEqual(points[2]["break_reason"], "time_gap")
        self.assertEqual(points[3]["break_reason"], "time_not_increasing")
        self.assertEqual(points[4]["break_reason"], "time_not_increasing")
        self.assertFalse(points[5]["plotted"])
        self.assertTrue(points[6]["break_reason"].startswith("after_"))

    def test_bad_checksum_malformed_and_unmeasured_do_not_bridge(self):
        bad = row(1)
        bad["sentence"] += "junk"
        values = [row(0), bad, row(2), "{bad json", row(4), row(5, 6), row(6)]
        points, segments, summary = route.analyze(raw(values))
        self.assertEqual(summary["plotted_observations"], 4)
        self.assertEqual([s["samples"] for s in segments], [1, 1, 1, 1])
        self.assertFalse(points[1]["plotted"])
        self.assertFalse(points[5]["plotted"])
        self.assertNotIn("<polyline", route.svg(points, summary))

    def test_column_tampering_is_not_accepted_as_fixed_position(self):
        for key, value in (("quality_code", 4), ("latitude_deg", 36.0), ("checksum_valid", False)):
            changed = row(1, 5)
            changed[key] = value
            points, _, summary = route.analyze(raw([changed]))
            self.assertFalse(points[0]["plotted"])
            self.assertIsNone(summary["origin"])

    def test_stationary_is_declared_not_inferred_and_source_is_untouched(self):
        content = raw([row(0), row(1), row(2)])
        with tempfile.TemporaryDirectory() as temporary:
            source, output = Path(temporary) / "gga.jsonl", Path(temporary) / "preview"
            source.write_bytes(content)
            summary = route.create(source, output, motion="stationary")
            self.assertEqual(source.read_bytes(), content)
            self.assertEqual((output / "source_gga.jsonl").read_bytes(), content)
            self.assertEqual(summary["description"], "정지 안테나 관측 산포")
            self.assertNotIn("<polyline", (output / "antenna_preview.svg").read_text())
            self.assertTrue((output / "segments.csv").exists())
            with self.assertRaises(FileExistsError):
                route.create(source, output)
        _, _, unspecified = route.analyze(content)
        self.assertEqual(unspecified["motion_declaration"], "unknown")

    def test_invalid_gap_and_coordinates_are_rejected(self):
        for gap in (0, -1, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                route.analyze(raw([row(0)]), max_gap_s=gap)
        for coordinate in ((91, 0), (0, 181), (float("nan"), 0)):
            with self.assertRaises(ValueError):
                route.ecef(*coordinate)


if __name__ == "__main__":
    unittest.main()
