"""--initial-drive-pwm 0: left stick must not produce a drive key; steering still passes."""
import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
from joystick_control import Driver
from step_manual_control import Session


class Pad:
    def __init__(self): self.value = (0., 0.); self.pwm_steps = []; self.raw_events = []; self.buttons = [0] * 11
    def poll(self): return self.value


class Tests(unittest.TestCase):
    def test_zero_drive_pwm_blocks_drive_keys_but_not_steer(self):
        pad = Pad(); s = Session(); d = Driver(pad, s, None); d.drive_pwm = 0
        pad.value = (1.0, 0.0); d.input()
        self.assertEqual(d.drive, 1); self.assertFalse(s.keys & {'w', 's'})       # requested but PWM 0 => no drive key
        pad.value = (1.0, 1.0); d.input()
        self.assertIn('d', s.keys); self.assertFalse(s.keys & {'w', 's'})
        pad.pwm_steps = [1]; d.input()                                            # D-pad up raises to 5 => drive key appears
        self.assertEqual(d.drive_pwm, 5); self.assertIn('w', s.keys)

    def test_cli_option_is_parsed_and_bounded(self):
        import subprocess
        tool = Path(__file__).parents[1] / 'tools' / 'joystick_desktop.py'
        help_text = subprocess.run([sys.executable, str(tool), '--help'], capture_output=True, text=True).stdout
        self.assertIn('--initial-drive-pwm', help_text)
        bad = subprocess.run([sys.executable, str(tool), '--initial-drive-pwm', '300'], capture_output=True, text=True)
        self.assertNotEqual(bad.returncode, 0); self.assertIn('0..255', bad.stderr)


if __name__ == '__main__':
    unittest.main()
