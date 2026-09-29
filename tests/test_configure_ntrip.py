import importlib.util
import json
from pathlib import Path
import stat
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("configure_ntrip", Path(__file__).resolve().parents[1] / "tools/configure_ntrip.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PrivateConfigTests(unittest.TestCase):
    def config(self):
        return dict(server="caster.example", port=2101, mountpoint="TEST", https=False,
                    version="2.0", ntripuser="fixture-user", ntrippassword="fixture-password", ggainterval=10)

    def test_private_mode_preserves_existing_and_explicit_replace(self):
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / "private/ntrip.local.json"
            module.save_private(self.config(), target)
            self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o600)
            self.assertEqual(json.loads(target.read_text()), self.config())
            newer = dict(self.config(), mountpoint="NEXT")
            with self.assertRaises(FileExistsError):
                module.save_private(newer, target)
            self.assertEqual(json.loads(target.read_text()), self.config())
            module.save_private(newer, target, replace=True)
            self.assertEqual(json.loads(target.read_text()), newer)
            self.assertEqual(list(target.parent.glob(".ntrip-*")), [])

    def test_invalid_settings_and_symlink_leave_target_unchanged(self):
        for change in (dict(ntripuser="user:bad"), dict(port=True), dict(port=0),
                       dict(server="https://caster.example"), dict(ntrippassword="bad\n"),
                       dict(mountpoint="/bad"), dict(https=1), dict(version="3.0")):
            with self.assertRaises(ValueError):
                module.validate(dict(self.config(), **change))
        with tempfile.TemporaryDirectory() as folder:
            original = Path(folder) / "original"
            original.write_text("keep")
            link = Path(folder) / "ntrip.local.json"
            link.symlink_to(original)
            with self.assertRaises(FileExistsError):
                module.save_private(self.config(), link, replace=True)
            self.assertEqual(original.read_text(), "keep")


if __name__ == "__main__":
    unittest.main()
