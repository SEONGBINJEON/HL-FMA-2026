"""Headless checks for arrow aliases, IME-independent keys, and held-key stops."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import step_manual_control as pc
import step_manual_protocol as wire


def status(request=1, now=0., **kwargs):
    return wire.Status(1, request, int(now * 1000), steering_adc=600, **kwargs)


def ready_session():
    session = pc.Session()
    session.step(0., [status()])
    assert session.request_arm(.01)
    session.step(.05, [status(2, .05)])
    session.step(.10, [status(3, .10, reason=11)])
    session.step(.15, [status(4, .15, state=1, reason=9)])
    assert session.phase == "ready"
    return session


class KeyboardAliasTests(unittest.TestCase):
    def setUp(self):
        self.session = ready_session()
        self.callbacks = {}
        self.sequence = 0
        self.stops = []

        def schedule(callback):
            self.sequence += 1
            self.callbacks[self.sequence] = callback
            return self.sequence

        def stop(reason):
            self.stops.append(reason)
            self.session.stop(reason)

        self.keyboard = pc.KeyboardEvents(self.session, stop, schedule,
            lambda token: self.callbacks.pop(token, None), lambda *a, **kw: None)

    def test_normalization_is_identical_for_shifted_and_unshifted_keys(self):
        for source, target in (("Up", "w"), ("Down", "s"), ("Left", "a"),
                               ("Right", "d"), ("W", "w"), ("S", "s"),
                               ("A", "a"), ("D", "d"), ("Shift_L", "shift_l"),
                               ("Shift_R", "shift_r"), ("Escape", "escape")):
            with self.subTest(source=source):
                self.assertEqual(pc.normalize_key(source), target)
        for source in ("Hangul", "Hangul_J_SsangJieuj", "ㅉ", "space", "Return"):
            self.assertNotIn(pc.normalize_key(source), pc.KEYS)

    def test_arrows_alone_produce_the_four_single_axis_commands(self):
        for key, code, drive, steer in (("Up", 111, 1, 0), ("Down", 116, -1, 0),
                                       ("Left", 113, 0, -1), ("Right", 114, 0, 1)):
            with self.subTest(key=key):
                self.setUp()
                self.keyboard.press(key, 2, code)
                command = self.session.step(.20, [status(5, .20, state=1, reason=9)])
                self.assertEqual((command.action, command.drive, command.steer, command.deadman),
                                 (wire.CONTROL, drive, steer, 1))

    def test_arrow_alone_release_is_committed_before_control_response(self):
        self.keyboard.press("Up", 2, 111)
        self.keyboard.release("Up", 3, 111)
        self.keyboard.flush()
        command = self.session.step(.20, [status(5, .20, state=1, reason=9)])
        self.assertEqual(command, wire.Command(1, 5, wire.CONTROL))
        self.assertEqual(self.session.phase, "ready")
        self.assertNotIn("w", self.session.keys)

    def test_exact_arrow_autorepeat_only_can_cancel_pending_release(self):
        self.keyboard.press("Shift_L", 1, 50)
        self.keyboard.press("Up", 2, 111)
        self.keyboard.release("Up", 100, 111)
        self.keyboard.press("Up", 100, 111)
        self.assertEqual(self.session.phase, "active")
        self.assertFalse(self.callbacks)
        self.keyboard.release("Up", 110, 111)
        self.keyboard.press("Up", 111, 111)
        self.assertEqual(self.session.phase, "active")

    def test_shift_release_does_not_interrupt_arrow(self):
        self.keyboard.press("Shift_L", 1, 50)
        self.keyboard.press("Right", 2, 114)
        self.keyboard.release("Shift_L", 100, 50)
        self.assertEqual(self.session.phase, "active")
        self.keyboard.press("Right", 100, 114)
        self.assertEqual(self.session.phase, "active")

    def test_drive_and_steer_together_allowed_but_opposites_stop(self):
        self.keyboard.press("Up", 2, 111)
        self.keyboard.press("Left", 3, 113)
        command = self.session.step(.20, [status(5, .20, state=1)])
        self.assertEqual((command.drive, command.steer, command.deadman), (1, -1, 1))
        self.setUp()
        self.keyboard.press("Up", 2, 111)
        self.keyboard.press("s", 3, 39)
        self.assertEqual(self.session.phase, "locked")

    def test_releasing_one_alias_cannot_hide_another_held_physical_key(self):
        self.keyboard.press("Shift_L", 1, 50)
        self.keyboard.press("w", 2, 25)
        self.keyboard.press("Up", 3, 111)
        self.keyboard.release("w", 4, 25)
        self.keyboard.flush()
        self.keyboard.release("Shift_L", 5, 50)
        self.assertEqual(self.session.phase, "active")
        self.session.step(.20, [status(5, .20, state=1, reason=10,
                                                requested_drive=1, actual_drive=1, flags=102)])
        self.keyboard.release("Up", 6, 111)
        self.keyboard.flush()
        self.assertEqual(self.session.phase, "ready")

    def test_focus_restore_does_not_rearm_a_held_arrow(self):
        self.keyboard.press("Down", 2, 116)
        self.session.focus(False)
        self.session.focus(True)
        self.keyboard.press("Down", 3, 116)
        self.assertEqual(self.session.phase, "locked")
        self.assertFalse(self.session.request_arm(.16))

    def test_shift_alone_is_not_a_motion_permission(self):
        self.keyboard.press("Shift_L", 1, 50)
        command = self.session.step(.20, [status(5, .20, state=1, reason=9)])
        self.assertEqual(command, wire.Command(1, 5, wire.CONTROL, 0, 0, 0))
        self.assertEqual(self.session.phase, "ready")
        self.assertFalse(self.session.deadman_seen)

    def test_direction_held_before_arm_requires_release_and_fresh_press(self):
        self.session.stop()
        self.keyboard.press("Up", 1, 111)
        self.session.step(.20, [status(5, .20)])
        self.assertFalse(self.session.request_arm(.21))
        self.keyboard.press("Up", 2, 111)
        self.assertEqual(self.session.phase, "locked")
        self.keyboard.release("Up", 3, 111)
        self.keyboard.flush()
        self.assertTrue(self.session.request_arm(.22))
        self.assertEqual(self.session.step(.25, [status(6, .25)]).action, wire.CONTROL)
        self.assertEqual(self.session.step(.30, [status(7, .30, reason=11)]).action, wire.ARM)
        neutral = self.session.step(.35, [status(8, .35, state=1, reason=9)])
        self.assertEqual((neutral.drive, neutral.steer, neutral.deadman), (0, 0, 0))
        self.keyboard.press("Up", 4, 111)
        motion = self.session.step(.40, [status(9, .40, state=1, reason=9)])
        self.assertEqual((motion.drive, motion.steer, motion.deadman), (1, 0, 1))


if __name__ == "__main__":
    unittest.main()
