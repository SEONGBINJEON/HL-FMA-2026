"""Pulse preparation logic and fake transport only; never opens hardware."""

from dataclasses import replace
from pathlib import Path
import struct
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import steer_pulse_test as pulse
from manual_protocol import Command, ARM, CONTROL, DISARM, STATUS_STRUCT, crc16, decode_frame


def status(request=1, now=0., **changes):
    value = pulse.PulseStatus(7, request, int(now * 1000), 0, 739, 0, 0, 0, 0, 0, 0, 36)
    return replace(value, **changes)


def encoded(value, kind=133):
    data = STATUS_STRUCT.pack(*value.__dict__.values())
    body = bytes((kind, len(data))) + data
    return b"\xf1\xa5" + body + struct.pack("<H", crc16(body))


def fire_sequence():
    seq = pulse.Sequence(0.)
    assert seq.receive([status()], .0).action == DISARM
    seq.input("fire", .01)
    assert seq.receive([status(2, .05)], .05) == Command(7, 2, CONTROL)
    assert seq.receive([status(3, .1, reason=11)], .10) == Command(7, 3, ARM)
    command = seq.receive([status(4, .15, state=1, reason=9)], .15)
    assert command == Command(7, 4, CONTROL, 0, -1, 1)
    return seq, command


class FakeTransport:
    def __init__(self, partial=None, queued=0):
        self.partial = partial
        self.out_waiting = queued
        self.writes = []

    def write(self, raw):
        count = len(raw) if self.partial is None else self.partial
        self.writes.append(raw[:count])
        return count


class FakeArchive:
    def event(self, *args, **kwargs):
        pass
    def raw(self, *args, **kwargs):
        pass


class PulseTests(unittest.TestCase):
    def test_cross_protocol_types_cannot_identify(self):
        for kind in (129, 131, 132):
            frames, errors = pulse.Parser().feed(encoded(status(), kind=kind), 0.)
            self.assertFalse(frames)
            self.assertTrue(errors)
            seq = pulse.Sequence(0.)
            self.assertIsNone(seq.receive(frames, 0., errors))
            self.assertIsNone(seq.stop_command())
        import front_pulse_test as front
        import rear_pulse_test as rear
        for module in (front, rear):
            frames, errors = module.Parser().feed(encoded(status(), kind=133), 0.)
            self.assertFalse(frames)
            self.assertTrue(errors)

    def test_positive_steer_or_any_drive_is_rejected(self):
        for invalid in (status(state=1, requested_steer=1), status(state=1, actual_steer=1),
                        status(state=1, requested_drive=1), status(state=1, actual_drive=1)):
            with self.assertRaises(ValueError):
                invalid.validate()

    def test_fast_relative_adc_stop_can_complete_without_active_snapshot(self):
        seq, command = fire_sequence()
        stopped = status(5, .2, state=1, reason=10, requested_steer=-1, flags=100, steering_adc=751)
        self.assertEqual(seq.receive([stopped], .2).action, DISARM)
        self.assertFalse(seq.observed_active)
        seq.receive([status(6, .25, flags=100, reason=1, steering_adc=751)], .25)
        self.assertEqual(seq.phase, 'DONE')
        self.assertTrue(seq.observed_used_stopped)

    def test_wait_only_sends_disarm_and_requires_explicit_fire(self):
        seq = pulse.Sequence(0.)
        for request in range(1, 6):
            self.assertEqual(seq.receive([status(request, request * .01)], request * .01).action, DISARM)
        self.assertTrue(seq.ready)
        self.assertEqual(seq.phase, "WAIT")
        self.assertEqual(seq.pulse_commands, 0)

    def test_one_pulse_then_same_request_zero_observation_and_disarm_ack(self):
        seq, command = fire_sequence()
        transport = FakeTransport()
        pulse.send(transport, FakeArchive(), command, seq)
        self.assertEqual(seq.sent_pulses, 1)
        active = status(5, .2, state=1, reason=10, requested_steer=-1, actual_steer=-1, flags=100)
        self.assertIsNone(seq.receive([active], .2))
        stopped = replace(active, millis=250, actual_steer=0)
        self.assertEqual(seq.receive([stopped], .25).action, DISARM)
        self.assertEqual(seq.fresh_at, .2)  # Duplicate did not renew request age.
        self.assertIsNone(seq.receive([status(6, .3, flags=100, reason=1)], .3))
        self.assertTrue(seq.done)
        self.assertEqual(seq.phase, "DONE")
        self.assertTrue(seq.observed_active)
        self.assertTrue(seq.observed_used_stopped)
        self.assertTrue(seq.observed_disarmed)
        self.assertEqual(len(transport.writes), 1)

    def test_duplicate_never_replies_again_and_regression_aborts(self):
        seq = pulse.Sequence(0.)
        seq.receive([status(2)], 0.)
        self.assertIsNone(seq.receive([status(2, .05)], .05))
        self.assertEqual(seq.fresh_at, 0.)
        with self.assertRaises(ValueError):
            seq.receive([status(1, .06)], .06)

    def test_fire_before_ready_or_second_fire_is_rejected(self):
        seq = pulse.Sequence(0.)
        seq.input("fire", 0.)
        self.assertTrue(seq.done)
        seq, _ = fire_sequence()
        seq.input("fire", .16)
        self.assertTrue(seq.done)
        self.assertEqual(seq.pulse_commands, 1)

    def test_already_used_board_and_reboot_never_fire(self):
        seq = pulse.Sequence(0.)
        with self.assertRaises(ValueError):
            seq.receive([status(flags=100)], 0.)
        seq = pulse.Sequence(0.)
        seq.receive([status()], 0.)
        with self.assertRaises(ValueError):
            seq.receive([status(2, .05, boot_id=8)], .05)
        self.assertEqual(seq.pulse_commands, 0)

    def test_absent_ack_or_unsafe_status_cannot_progress(self):
        seq = pulse.Sequence(0.)
        seq.receive([status()], 0.)
        seq.input("fire", .01)
        seq.receive([status(2, .05)], .05)
        with self.assertRaises(ValueError):
            seq.receive([status(3, .10, reason=9)], .10)
        for unsafe in (status(actual_steer=-1), status(requested_drive=1),
                       status(flags=33), status(state=1, actual_steer=-1, flags=100)):
            with self.assertRaises(ValueError):
                unsafe.validate()

    def test_timeout_cancel_and_eof_are_terminal(self):
        for text in ("cancel", "", "something"):
            seq = pulse.Sequence(0.)
            seq.input(text, 0.)
            self.assertTrue(seq.done)
        seq = pulse.Sequence(0.)
        seq.timeout(600.)
        self.assertTrue(seq.done)
        seq, _ = fire_sequence()
        seq.timeout(.301)
        self.assertTrue(seq.done)
        seq, _ = fire_sequence()
        seq.fresh_at = 5.
        seq.timeout(5.011)
        self.assertTrue(seq.done)

    def test_preparation_allows_just_before_600_seconds_and_stops_at_deadline(self):
        seq = pulse.Sequence(0.)
        seq.receive([status(1, 599.95)], 599.95)
        seq.timeout(599.999)
        self.assertFalse(seq.done)
        self.assertEqual(seq.phase, "WAIT")
        seq.timeout(600.)
        self.assertTrue(seq.done)
        self.assertEqual(seq.error, "600-second preparation deadline")

    def test_type133_parser_crc_and_partial_expiry(self):
        parser = pulse.Parser()
        frame = encoded(status())
        values = []
        for byte in frame:
            got, errors = parser.feed(bytes((byte,)), 0.)
            values += got
            self.assertFalse(errors)
        self.assertEqual(values, [status()])
        bad = bytearray(frame)
        bad[-1] ^= 1
        self.assertIn("CRC", parser.feed(bad, .01)[1])
        parser.feed(frame[:10], .02)
        self.assertIn("partial frame expired", parser.feed(b"", .051)[1])
        self.assertFalse(parser.buffer)

    def test_original_and_bench_protocol_never_identify(self):
        parser = pulse.Parser()
        frames, errors = parser.feed(b"[MONITOR] Drive:STOP\r\n" + encoded(status(), 129), 0.)
        self.assertFalse(frames)
        seq = pulse.Sequence(0.)
        self.assertIsNone(seq.receive(frames, 0., errors))
        self.assertIsNone(seq.stop_command())

    def test_crc_error_with_good_status_prevents_control(self):
        seq, _ = fire_sequence()
        with self.assertRaises(ValueError):
            seq.receive([status(5, .2, state=1, flags=100, actual_steer=-1, requested_steer=-1)], .2, ["CRC"])

    def test_slow_send_is_replaced_by_disarm_and_never_counted_as_pulse(self):
        seq, command = fire_sequence()
        transport = FakeTransport()
        with mock.patch.object(pulse.time, "monotonic", return_value=.2):
            pulse.send(transport, FakeArchive(), command, seq, deadline=.19)
        self.assertEqual(decode_frame(transport.writes[0]).action, DISARM)
        self.assertEqual(seq.sent_pulses, 0)
        self.assertTrue(seq.done)

    def test_partial_tx_or_queued_tx_has_no_retry(self):
        for transport in (FakeTransport(partial=5), FakeTransport(queued=1)):
            seq, command = fire_sequence()
            with self.assertRaises(OSError):
                pulse.send(transport, FakeArchive(), command, seq)
            self.assertLessEqual(len(transport.writes), 1)
            self.assertEqual(seq.sent_pulses, 0)

    def test_missing_prepare_power_flags_cannot_open_port(self):
        with mock.patch.object(pulse, "MegaTransport") as opened:
            with self.assertRaises(SystemExit):
                pulse.main(["--prepare"])
            opened.assert_not_called()


if __name__ == "__main__":
    unittest.main()
