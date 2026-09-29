"""Selection and launch fixtures; no real serial port is opened."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock


TOOLS = Path(__file__).resolve().parents[1] / "tools"
with mock.patch.object(sys, "path", [str(TOOLS), *sys.path]):
    SPEC = importlib.util.spec_from_file_location("start_gnss_recording", TOOLS / "start_gnss_recording.py")
    starter = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(starter)


def inventory(count=1):
    return {"usb_devices": [{"sysfs_device": "usb-target", "vendor_id": "1546", "product_id": "01a9"}],
            "serial_ports": [{"usb_parent": "usb-target", "device": f"/dev/ttyACM{index + 7}",
                              "stable_paths": [f"/dev/serial/by-id/fixture{index}"],
                              "permissions": {"present": True, "character_device": True,
                                              "readable_by_current_process": True}}
                             for index in range(count)]}


class StarterTests(unittest.TestCase):
    def test_single_identified_receiver_uses_stable_path(self):
        self.assertEqual(starter.select_port(inventory()), "/dev/serial/by-id/fixture0")

    def test_none_multiple_wrong_id_or_regular_file_are_refused(self):
        wrong = inventory()
        wrong["usb_devices"][0]["vendor_id"] = "1234"
        regular = inventory()
        regular["serial_ports"][0]["permissions"]["character_device"] = False
        for fixture in [inventory(0), inventory(2), wrong, regular]:
            with self.subTest(fixture=fixture), self.assertRaises(ValueError):
                starter.select_port(fixture)

    def test_ambiguous_receiver_never_launches_recorder(self):
        with mock.patch.object(starter, "device_inventory", return_value=inventory(2)), \
                mock.patch.object(starter.subprocess, "run") as run, mock.patch("sys.stderr"):
            self.assertEqual(starter.main([]), 1)
        run.assert_not_called()

    def test_launch_uses_separate_arguments_and_unique_directory(self):
        with mock.patch.object(starter, "device_inventory", return_value=inventory()), \
                mock.patch.object(starter.subprocess, "run", return_value=SimpleNamespace(returncode=0)) as run, \
                mock.patch("sys.stdout"):
            self.assertEqual(starter.main(["2.5"]), 0)
        first = run.call_args_list[0].args[0]
        second = run.call_args_list[1].args[0]
        self.assertEqual(first[first.index("--duration") + 1], "2.5")
        self.assertEqual(first[first.index("--port") + 1], "/dev/serial/by-id/fixture0")
        output = first[first.index("--output-dir") + 1]
        self.assertRegex(Path(output).name, r"^gnss_\d{8}_\d{6}_[0-9a-f]{8}$")
        self.assertEqual(second[-1], output)
        self.assertNotIn("shell", run.call_args_list[0].kwargs)


if __name__ == "__main__":
    unittest.main()
