"""Hardware-free protocol, input state, freshness and fake-transport checks."""

from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import manual_control as pc
import manual_protocol as wire


def status(request=1, now=0., **kwargs):
    return wire.Status(1, request, int(now * 1000), steering_adc=774, **kwargs)


def ready_session():
    session = pc.Session()
    assert session.step(0., [status()]).action == wire.DISARM
    assert session.request_arm(.01)
    assert session.step(.05, [status(2, .05)]).action == wire.CONTROL
    assert session.step(.10, [status(3, .10, reason=11)]).action == wire.ARM
    command = session.step(.15, [status(4, .15, state=wire.ARMED, reason=9)])
    assert command == wire.Command(1, 4, wire.CONTROL)
    assert session.phase == "ready"
    return session


class FakeTransport:
    def __init__(self):
        self.rx = b""
        self.writes = []
        self.closed = False
        self.short = None
        self.queued = 0
        self.raise_write = False

    @property
    def in_waiting(self):
        return len(self.rx)

    @property
    def out_waiting(self):
        return self.queued

    def read(self, size):
        raw, self.rx = self.rx[:size], self.rx[size:]
        return raw

    def write(self, data):
        if self.raise_write:
            raise BlockingIOError("synthetic")
        count = len(data) if self.short is None else self.short
        self.writes.append(data[:count])
        return count

    def close(self):
        self.closed = True


class ProtocolTests(unittest.TestCase):
    def test_crc_reference_and_cpp_actual_command_vector(self):
        self.assertEqual(wire.crc16(b"123456789"), 0x29b1)
        value = wire.Command(0x12345678, 2, wire.CONTROL, -1, 1, 1)
        expected = bytes.fromhex("f1a5010c785634120200000002ff010172b8")
        self.assertEqual(wire.encode_command(value), expected)
        self.assertEqual(wire.decode_frame(expected), value)

    def test_cpp_actual_status_vector(self):
        expected = bytes.fromhex("f1a5811a78563412010000003412000085ffffff0603000000000000210010b1")
        value = wire.Status(0x12345678, 1, 0x1234, -123, 774)
        self.assertEqual(wire.encode_status(value), expected)
        self.assertEqual(wire.decode_frame(expected), value)

    def test_fragmented_frame_and_bounded_buffer(self):
        parser = wire.FrameParser()
        raw = wire.encode_status(status())
        received = []
        for byte in raw:
            values, errors = parser.feed(bytes((byte,)), .0)
            self.assertFalse(errors)
            received += values
            self.assertLessEqual(len(parser.buffer), wire.MAX_FRAME)
        self.assertEqual(received, [status()])

    def test_bad_crc_is_not_silently_recovered_as_valid(self):
        bad = bytearray(wire.encode_status(status()))
        bad[-1] ^= 1
        values, errors = wire.FrameParser().feed(bytes(bad) + wire.encode_status(status(2)), 0.)
        self.assertEqual(values, [status(2)])
        self.assertIn("CRC", errors)

    def test_partial_expires_without_more_bytes(self):
        parser = wire.FrameParser()
        parser.feed(wire.encode_status(status())[:12], 0.)
        values, errors = parser.feed(b"", .031)
        self.assertFalse(values)
        self.assertIn("partial frame expired", errors)
        self.assertFalse(parser.buffer)

    def test_overload_and_invalid_lengths_are_bounded(self):
        parser = wire.FrameParser()
        self.assertEqual(parser.feed(b"x" * 4097, 0.)[1], ["RX flood"])
        self.assertFalse(parser.buffer)
        values, errors = parser.feed(b"\xf1\xa5\x81\xff", .01)
        self.assertFalse(values)
        self.assertIn("type/length", errors)
        self.assertFalse(parser.buffer)

    def test_semantic_invalid_values_cannot_encode(self):
        for command in (wire.Command(1, 1, 7), wire.Command(1, 1, wire.ARM, 1),
                        wire.Command(1, 1, wire.CONTROL, 1, 0, 0),
                        wire.Command(-1, 1), wire.Command(1, 1, wire.CONTROL, 2, 0, 1)):
            with self.assertRaises(wire.ProtocolError):
                wire.encode_command(command)
        for invalid in (replace(status(), steering_adc=1024), replace(status(), flags=128),
                        replace(status(), state=2), replace(status(), reason=12)):
            with self.assertRaises(wire.ProtocolError):
                wire.encode_status(invalid)


class SessionTests(unittest.TestCase):
    def test_no_arm_before_identification_or_with_keys_held(self):
        session = pc.Session()
        self.assertFalse(session.request_arm(0.))
        session.step(0., [status()])
        session.key_down("w")
        self.assertFalse(session.request_arm(.01))
        session.key_up("w")
        self.assertTrue(session.request_arm(.01))

    def test_neutral_arm_fresh_shift_and_input_sequence(self):
        session = ready_session()
        self.assertEqual(session.step(.20, [status(5, .20, state=1, reason=11)]),
                         wire.Command(1, 5, wire.CONTROL))
        session.key_down("Shift_L")
        session.key_down("w")
        self.assertEqual(session.step(.25, [status(6, .25, state=1, reason=10)]),
                         wire.Command(1, 6, wire.CONTROL, 1, 0, 1))

    def test_key_pressed_during_arm_is_not_reused(self):
        session = pc.Session()
        session.step(0., [status()])
        session.request_arm(.01)
        session.key_down("shift_l")
        session.key_down("w")
        command = session.step(.05, [status(2, .05)])
        self.assertEqual(command.action, wire.DISARM)
        self.assertFalse(session.request_arm(.06))

    def test_missing_neutral_or_arm_ack_stops(self):
        for reason in (0, 1, 9):
            session = pc.Session()
            session.step(0., [status()])
            session.request_arm(.01)
            session.step(.05, [status(2, .05)])
            self.assertEqual(session.step(.1, [status(3, .1, reason=reason)]).action, wire.DISARM)
        session = pc.Session()
        session.step(0., [status()])
        session.request_arm(.01)
        session.step(.05, [status(2, .05)])
        session.step(.1, [status(3, .1, reason=11)])
        self.assertEqual(session.step(.15, [status(4, .15, reason=6)]).action, wire.DISARM)

    def test_any_control_key_release_stops_and_retains_other_held_keys(self):
        for key in ("w", "shift_l"):
            session = ready_session()
            session.key_down("shift_l")
            session.key_down("w")
            session.key_up(key)
            self.assertEqual(session.phase, "locked")
            self.assertTrue(session.keys)
            self.assertFalse(session.request_arm(.16))
            self.assertEqual(session.step(.20, [status(5, .20, state=1)]).action, wire.DISARM)

    def test_focus_loss_never_auto_rearms_or_forgets_held_keys(self):
        session = ready_session()
        session.key_down("shift_l")
        session.key_down("w")
        session.focus(False)
        session.focus(True)
        self.assertEqual(session.phase, "locked")
        self.assertEqual(session.keys, {"shift_l", "w"})
        self.assertFalse(session.request_arm(.16))

    def test_ui_stall_wins_over_fresh_status(self):
        session = ready_session()
        session.key_down("shift_l")
        session.key_down("w")
        command = session.step(.251, [status(5, .251, state=1)])
        self.assertEqual(command.action, wire.DISARM)
        self.assertEqual(session.phase, "locked")

    def test_status_timeout_while_ui_keeps_ticking(self):
        session = ready_session()
        session.key_down("shift_l")
        session.step(.20)
        session.step(.25)
        self.assertIsNone(session.step(.301))
        self.assertEqual(session.phase, "locked")
        self.assertIn("수신 만료", session.message)

    def test_latest_only_response_and_duplicate_no_freshness_extension(self):
        session = pc.Session()
        command = session.step(0., [status(1), status(2), status(3)])
        self.assertEqual(command.request_id, 3)
        self.assertIsNone(session.step(.05, [status(3, .05)]))
        self.assertEqual(session.last_status_at, 0.)
        self.assertIsNone(session.step(.10, [status(3, .10)]))

    def test_bad_order_in_discarded_batch_also_stops(self):
        session = ready_session()
        session.key_down("shift_l")
        command = session.step(.20, [status(5, .20, state=1), status(2, .1), status(6, .21, state=1)])
        self.assertEqual(command.action, wire.DISARM)
        self.assertEqual(command.request_id, 6)

    def test_boot_change_stops_and_held_key_blocks_rearm(self):
        session = ready_session()
        session.key_down("shift_l")
        command = session.step(.20, [replace(status(), boot_id=2)])
        self.assertEqual(command, wire.Command(2, 1))
        self.assertFalse(session.request_arm(.21))

    def test_clock_wrap_allowed_but_clock_regression_stops(self):
        session = pc.Session()
        session.step(0., [replace(status(), millis=0xfffffff0)])
        self.assertEqual(session.step(.05, [replace(status(2), millis=0x20)]).request_id, 2)
        session.request_arm(.06)
        command = session.step(.10, [replace(status(3), millis=0x10)])
        self.assertIsNone(command)
        self.assertEqual(session.phase, "locked")

    def test_live_flags_invalid_boot_and_reported_actual_output_lock(self):
        for invalid in (replace(status(), flags=wire.BOOT_ID_VALID),
                        replace(status(), boot_id=0), replace(status(), request_id=0),
                        replace(status(), flags=wire.BENCH | wire.BOOT_ID_VALID | wire.DRIVE_ENABLED),
                        replace(status(), flags=wire.BENCH), replace(status(), actual_drive=1)):
            session = pc.Session()
            self.assertEqual(session.step(0., [invalid]).action, wire.DISARM)
            self.assertFalse(session.request_arm(.01))

    def test_board_stop_or_opposing_input_forces_new_arm(self):
        session = ready_session()
        session.key_down("shift_l")
        session.key_down("w")
        session.key_down("s")
        self.assertEqual(session.phase, "locked")
        session = ready_session()
        self.assertEqual(session.step(.20, [status(5, .20, reason=8)]).action, wire.DISARM)
        self.assertEqual(session.phase, "locked")


class ClientTests(unittest.TestCase):
    def test_tick_gap_and_logging_delay_share_the_100ms_budget(self):
        # Use one clock domain for the UI argument and monotonic calls: the
        # previous tick was .15, current entry .24, actual write .33. Neither
        # 90ms segment alone reaches 100ms, but together they must block motion.
        for location in ("rx", "tx_attempt"):
            with self.subTest(location=location):
                clock = [.24]

                class SlowJournal:
                    def event(self, name, **fields):
                        if name == location:
                            clock[0] += .09

                    def bytes(self, direction, raw):
                        if direction == location:
                            clock[0] += .09

                transport, session = FakeTransport(), ready_session()
                self.assertEqual(session.last_tick_at, .15)
                session.key_down("shift_l")
                session.key_down("w")
                transport.rx = wire.encode_status(status(5, .24, state=wire.ARMED))
                client = pc.Client(transport, session, SlowJournal())
                with mock.patch.object(pc.time, "monotonic", side_effect=lambda: clock[0]):
                    client.tick(clock[0])
                self.assertAlmostEqual(clock[0], .33)
                self.assertEqual(len(transport.writes), 1)
                self.assertEqual(wire.decode_frame(transport.writes[0]).action, wire.DISARM)
                self.assertEqual(session.phase, "locked")

    def test_logging_stall_before_step_or_write_can_only_send_disarm(self):
        for location in ("rx", "tx_attempt"):
            clock = [0.]
            class SlowJournal:
                def event(self, name, **fields):
                    if name == location:
                        clock[0] += .101
                def bytes(self, direction, raw):
                    if direction == location:
                        clock[0] += .101
            transport, session = FakeTransport(), ready_session()
            session.key_down("shift_l")
            session.key_down("w")
            transport.rx = wire.encode_status(status(5, .20, state=1))
            client = pc.Client(transport, session, SlowJournal())
            with mock.patch.object(pc.time, "monotonic", side_effect=lambda: clock[0]):
                client.tick(.20)
            self.assertEqual(wire.decode_frame(transport.writes[0]).action, wire.DISARM)
            self.assertEqual(session.phase, "locked")

    def test_original_monitor_firmware_and_stop_button_send_zero_bytes(self):
        transport, session = FakeTransport(), pc.Session()
        client = pc.Client(transport, session)
        transport.rx = b"[MONITOR] Drive:STOP Encoder:0 Steering:774\r\n"
        client.tick(0.)
        client.stop("test stop")
        client.close()
        self.assertEqual(transport.writes, [])

    def test_valid_status_first_identifies_then_disarms(self):
        transport, session = FakeTransport(), pc.Session()
        client = pc.Client(transport, session)
        transport.rx = wire.encode_status(status())
        client.tick(0.)
        self.assertEqual([wire.decode_frame(raw) for raw in transport.writes], [wire.Command(1, 1)])
        client.stop("Escape")
        self.assertEqual(len(transport.writes), 2)  # Explicit emergency exception.

    def test_corrupt_frame_followed_by_good_frame_cannot_continue_input(self):
        transport, session = FakeTransport(), ready_session()
        session.key_down("shift_l")
        session.key_down("w")
        client = pc.Client(transport, session)
        bad = bytearray(wire.encode_status(status(5, .20, state=1)))
        bad[-1] ^= 1
        transport.rx = bytes(bad) + wire.encode_status(status(6, .21, state=1))
        client.tick(.20)
        self.assertEqual(wire.decode_frame(transport.writes[0]).action, wire.DISARM)

    def test_partial_write_closes_without_retry_or_append(self):
        transport, session = FakeTransport(), pc.Session()
        transport.short = 7
        transport.rx = wire.encode_status(status())
        client = pc.Client(transport, session)
        client.tick(0.)
        self.assertTrue(client.failed)
        self.assertTrue(transport.closed)
        client.tick(.05)
        client.stop("again")
        self.assertEqual(len(transport.writes), 1)
        self.assertEqual(len(transport.writes[0]), 7)

    def test_queued_tx_and_write_exception_close_without_retry(self):
        for mode in ("queue", "exception"):
            transport, session = FakeTransport(), pc.Session()
            transport.queued = 1 if mode == "queue" else 0
            transport.raise_write = mode == "exception"
            transport.rx = wire.encode_status(status())
            client = pc.Client(transport, session)
            client.tick(0.)
            self.assertTrue(client.failed)
            self.assertTrue(transport.closed)
            self.assertFalse(transport.writes)

    def test_receive_flood_is_bounded_and_closes(self):
        transport, session = FakeTransport(), ready_session()
        transport.rx = b"x" * 4097
        client = pc.Client(transport, session)
        client.tick(.20)
        self.assertTrue(client.failed)
        self.assertTrue(transport.closed)
        self.assertEqual(wire.decode_frame(transport.writes[0]).action, wire.DISARM)

    def test_timeout_without_rx_sends_one_emergency_stop(self):
        transport, session = FakeTransport(), ready_session()
        session.key_down("shift_l")
        client = pc.Client(transport, session)
        for now in (.20, .25, .301, .35):
            client.tick(now)
        self.assertEqual(len(transport.writes), 1)
        self.assertEqual(wire.decode_frame(transport.writes[0]).action, wire.DISARM)

    def test_journal_raw_bytes_events_and_exact_source_hashes(self):
        import hashlib
        with tempfile.TemporaryDirectory() as temporary:
            journal = pc.Journal("demo", Path(temporary) / "new")
            journal.bytes("rx", b"\x00hello")
            journal.event("focus", focused=False)
            journal.close()
            self.assertEqual((journal.path / "rx.bin").read_bytes(), b"\x00hello")
            meta = json.loads((journal.path / "metadata.json").read_text())
            for name, digest in meta["sources_sha256"].items():
                self.assertEqual(hashlib.sha256((journal.path / name).read_bytes()).hexdigest(), digest)
            self.assertFalse(meta["physical_vehicle_verified"])
            self.assertIsNone(meta["port"])
            with self.assertRaises(FileExistsError):
                pc.Journal("demo", journal.path)

    def test_default_is_demo_and_connect_must_be_explicit(self):
        with mock.patch.object(pc, "run_ui") as run:
            self.assertEqual(pc.main([]), 0)
            run.assert_called_once_with(False, None)
        with mock.patch.object(pc, "run_ui") as run:
            self.assertEqual(pc.main(["--connect"]), 0)
            run.assert_called_once_with(True, None)

    def test_mismatched_mega_never_opens_any_device(self):
        with mock.patch.object(Path, "is_symlink", return_value=True), \
             mock.patch.object(pc, "_device_identity", return_value=(1, ("2341", "0042", "wrong"))), \
             mock.patch.object(pc.os, "open") as opened:
            with self.assertRaises(RuntimeError):
                pc.MegaTransport()
            opened.assert_not_called()


class KeyboardTests(unittest.TestCase):
    def setUp(self):
        self.session = ready_session()
        self.stops = []
        self.scheduled = {}
        self.next_id = 0
        def schedule(callback):
            self.next_id += 1
            self.scheduled[self.next_id] = callback
            return self.next_id
        def stop(reason):
            self.session.stop(reason)
            self.stops.append(reason)
        self.keyboard = pc.KeyboardEvents(self.session, stop, schedule,
            lambda key: self.scheduled.pop(key, None), lambda *args, **kwargs: None)
        self.keyboard.press("Shift_L", 1, 50)
        self.keyboard.press("w", 2, 25)

    def test_shift_release_repress_in_same_event_batch_always_stops(self):
        self.keyboard.release("Shift_L", 100, 50)
        self.assertEqual(self.session.phase, "locked")
        self.keyboard.press("Shift_L", 101, 50)
        self.assertEqual(self.session.phase, "locked")
        self.assertEqual(len(self.stops), 1)
        self.assertFalse(self.scheduled)

    def test_shift_release_even_identical_timestamp_is_never_cancelled(self):
        self.keyboard.release("Shift_L", 100, 50)
        self.keyboard.press("Shift_L", 100, 50)
        self.assertEqual(self.session.phase, "locked")

    def test_only_exact_movement_repeat_pair_can_cancel_release(self):
        self.keyboard.release("w", 100, 25)
        self.keyboard.press("w", 100, 25)
        self.assertEqual(self.session.phase, "active")
        self.assertFalse(self.scheduled)
        self.assertFalse(self.stops)

    def test_real_release_repress_requires_new_arm(self):
        self.keyboard.release("w", 100, 25)
        self.keyboard.press("w", 101, 25)
        self.assertEqual(self.session.phase, "locked")
        self.assertEqual(len(self.stops), 1)

    def test_pending_release_flushes_before_response_even_without_idle(self):
        self.keyboard.release("w", 100, 25)
        self.assertEqual(self.session.phase, "active")
        self.keyboard.flush()
        self.assertEqual(self.session.phase, "locked")
        self.assertNotIn("w", self.session.keys)
        self.assertEqual(self.session.step(.20, [status(5, .20, state=1)]).action, wire.DISARM)

    def test_zero_delay_callback_commits_release(self):
        self.keyboard.release("w", 100, 25)
        callback = next(iter(self.scheduled.values()))
        callback()
        self.assertEqual(self.session.phase, "locked")
        self.assertFalse(self.scheduled)


if __name__ == "__main__":
    unittest.main()
