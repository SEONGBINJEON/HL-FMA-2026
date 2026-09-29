"""Metadata fixtures: no real sensor connection, capture, or serial access."""

import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock


SPEC = importlib.util.spec_from_file_location(
    "check_devices", Path(__file__).resolve().parents[1] / "tools/check_devices.py")
devices = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(devices)


class DeviceInventoryTests(unittest.TestCase):
    def test_video_nodes_are_not_misidentified_as_realsense(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            node = root / "sys/class/video4linux/video0"
            node.mkdir(parents=True)
            (node / "name").write_text("Intel IPU6 ISYS Capture 0")
            report = devices.device_inventory(root / "sys", root / "dev")
            self.assertEqual(report["video_nodes"][0]["name"], "Intel IPU6 ISYS Capture 0")
            self.assertIsNone(report["video_nodes"][0]["usb_parent"])
            self.assertEqual(report["usb_devices"], [])
            summary = devices.format_summary({"devices": report})
            self.assertIn("USB 부모 미확인 영상 노드: 1개", summary)
            self.assertIn("USB 연결 영상 노드: 0개", summary)
            self.assertNotIn("Intel IPU6 ISYS Capture 0", summary)
            self.assertIn("미검출은 고장이나 기능 실패 판정이 아닙니다", summary)

    def test_usb_parent_and_stable_serial_paths_are_linked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            usb = root / "sys/bus/usb/devices/1-2"
            usb.mkdir(parents=True)
            for name, value in {"idVendor": "1546", "idProduct": "01a9",
                                "manufacturer": "u-blox", "product": "GNSS receiver"}.items():
                (usb / name).write_text(value)
            tty = root / "sys/class/tty"
            tty.mkdir(parents=True)
            (usb / "ttyACM0").mkdir()
            (tty / "ttyACM0").symlink_to(usb / "ttyACM0")
            dev = root / "dev"
            (dev / "serial/by-id").mkdir(parents=True)
            (dev / "ttyACM0").touch()
            link = dev / "serial/by-id/usb-fixture"
            link.symlink_to("../../ttyACM0")
            report = devices.device_inventory(root / "sys", dev)
            port = report["serial_ports"][0]
            self.assertEqual(port["usb_parent"], "1-2")
            self.assertEqual(port["stable_paths"], [str(link)])
            self.assertFalse(port["permissions"]["character_device"])
            self.assertEqual(report["usb_devices"][0]["product"], "GNSS receiver")
            self.assertNotIn("model", report["usb_devices"][0])
            summary = devices.format_summary({"devices": report})
            self.assertIn("USB에 연결된 시리얼 포트: 1개", summary)
            self.assertIn(str(link), summary)
            self.assertIn("1546:01a9", summary)

    def test_ros_setup_without_path_does_not_claim_runtime_works(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "humble").mkdir()
            (root / "humble/setup.bash").touch()
            report = devices.ros_inventory(root, executable_lookup=lambda name: None)
            self.assertEqual(report["status"], "setup_files_found_commands_not_on_path")
            self.assertEqual(report["runtime_working"], "unknown_not_executed")

    def test_output_refuses_to_overwrite_previous_report(self):
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / "report.json"
            report.write_text("previous report")
            with mock.patch.object(devices, "collect_report", return_value={}), \
                    mock.patch("sys.stderr"):
                self.assertEqual(devices.main(["--output", str(report)]), 1)
            self.assertEqual(report.read_text(), "previous report")

    def test_summary_option_still_saves_full_json(self):
        report = {"devices": {"video_nodes": [
            {"device": "/dev/video0", "name": "builtin fixture", "usb_parent": None}]}}
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "full.json"
            stdout = io.StringIO()
            with mock.patch.object(devices, "collect_report", return_value=report), \
                    mock.patch("sys.stdout", stdout):
                self.assertEqual(devices.main(["--summary", "--output", str(output)]), 0)
            self.assertEqual(json.loads(output.read_text()), report)
            self.assertIn("USB 부모 미확인 영상 노드: 1개", stdout.getvalue())
            self.assertNotIn("builtin fixture", stdout.getvalue())


if __name__ == "__main__":
    unittest.main()
