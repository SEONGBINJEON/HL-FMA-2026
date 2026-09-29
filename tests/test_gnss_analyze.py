import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location("gnss_analyze", Path(__file__).parents[1] / "tools" / "gnss_analyze.py")
analyzer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analyzer)


class AnalyzeTests(unittest.TestCase):
    def test_quality_integrity_malformed_and_metrics(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            rows = []
            for index, quality in enumerate((0, 1, 2, 3, 4, 5, 6, 7, 8, None, True)):
                rows.append({"quality_code": quality, "fix_status": "rtk_fixed",
                             "checksum_valid": True, "latitude_deg": 37.0, "longitude_deg": 127.0,
                             "elapsed_s": float(index), "satellites": 12, "hdop": 0.8,
                             "differential_age_s": float("nan")})
            rows.append({"quality_code": 4, "checksum_valid": False,
                         "latitude_deg": 37.0, "longitude_deg": 127.0})
            rows.append({"quality_code": 4, "checksum_valid": True,
                         "latitude_deg": float("inf"), "longitude_deg": 127.0})
            source = "\n".join(json.dumps(row) for row in rows) + "\n{bad\n[]\n"
            path = directory / "gga.jsonl"
            path.write_text(source)
            report = analyzer.analyze(directory)
            self.assertEqual(report["malformed_lines"], 2)
            self.assertEqual(report["measured_position_records"], 5)
            self.assertEqual(report["rtk_fixed_coordinate_records"], 1)
            self.assertEqual(report["quality_counts"]["dgps_fix"], 1)
            self.assertEqual(report["quality_counts"]["unknown"], 2)
            self.assertEqual(report["quality_counts"]["unverified_checksum"], 1)
            self.assertEqual(report["gga_receive_interval_s"]["mean"], 1)
            self.assertEqual(report["metrics"]["differential_age_s"]["samples"], 0)
            json.dumps(report, allow_nan=False)
            self.assertEqual(path.read_text(), source)

    def test_empty_and_existing_output_are_safe(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            (directory / "gga.jsonl").write_text("")
            (directory / "metadata.json").write_text(json.dumps({"status": "no_data", "bytes_received": 0}))
            report = analyzer.analyze(directory)
            self.assertEqual(report["records"], 0)
            self.assertEqual(report["quality_percent"], {})
            output = directory / "report.json"
            output.write_text("original")
            self.assertEqual(analyzer.main(["--session", temp, "--output", str(output)]), 1)
            self.assertEqual(output.read_text(), "original")


if __name__ == "__main__":
    unittest.main()
