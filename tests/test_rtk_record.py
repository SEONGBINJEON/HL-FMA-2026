import contextlib
import importlib.util
import io
import json
import logging
import os
import pty
from pathlib import Path
import sys
import tempfile
import select
from threading import Event, Thread
import unittest
from unittest.mock import patch

TOOLS = Path(__file__).parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
import rtk_record as rtk
from gnss_record import parse_gga

try:
    from pyrtcm import calc_crc24q
    import pygnssutils
    HAS_RTK = True
except ImportError:
    HAS_RTK = False


def gga(quality=1, blank=False):
    fields = f"GNGGA,123519,3728.0000,N,12706.0000,E,{quality},12,0.8,50,M,20,M,,"
    if blank:
        fields = f"GNGGA,123519,3728.0000,N,12706.0000,E,{quality},,,,,,,,"
    data = fields.encode()
    checksum = 0
    for byte in data:
        checksum ^= byte
    return b"$" + data + f"*{checksum:02X}\r\n".encode()


def rtcm():
    payload = b"\x3e\xd0" + b"\0" * 17  # RTCM 1005 station position fixture
    prefix = b"\xd3\0" + bytes((len(payload),)) + payload
    return prefix + calc_crc24q(prefix).to_bytes(3, "big")


CONFIG = {"server": "example.invalid", "port": 2101, "mountpoint": "TEST",
          "https": True, "version": "2.0", "ggainterval": 10,
          "ntripuser": "fixture-user", "ntrippassword": "fixture-secret"}


def inventory():
    return {"usb_devices": [{"vendor_id": "1546", "product_id": "01a9", "sysfs_device": "ublox"},
                             {"vendor_id": "2341", "product_id": "0042", "sysfs_device": "mega"}],
            "serial_ports": [{"device": "FAKE_GNSS", "usb_parent": "ublox", "stable_paths": [],
                              "permissions": {"present": True, "character_device": True, "readable_by_current_process": True}},
                             {"device": "NEVER_ARDUINO", "usb_parent": "mega", "permissions": {"present": True}}]}


class ConfigTests(unittest.TestCase):
    def test_owner_only_config_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "config.json"
            path.write_text(json.dumps(CONFIG))
            path.chmod(0o600)
            self.assertEqual(rtk.load_config(path), CONFIG)
            path.chmod(0o644)
            with self.assertRaises(rtk.SafeError):
                rtk.load_config(path)
            link = Path(temp) / "link"
            link.symlink_to(path)
            with self.assertRaises(OSError):
                rtk.load_config(link)


@unittest.skipUnless(HAS_RTK, "Run with .venv-rtk/bin/python")
class RTKTests(unittest.TestCase):
    def tearDown(self):
        logging.disable(logging.NOTSET)

    def test_pinned_formatter_uses_exact_raw_and_stale_or_closed_stops(self):
        now = [0.0]
        app = rtk.LivePosition(lambda: now[0])
        for quality in (1, 2, 3, 4, 5):
            source = gga(quality, blank=True)
            app.update(parse_gga(source), source)
            client = rtk.make_client(app)
            raw, parsed = client._format_gga()
            self.assertEqual(raw, source)
            self.assertEqual(parsed.quality, quality)
            self.assertEqual(app.get_coordinates()["alt"], "")
            self.assertEqual(app.get_coordinates()["sip"], "")
        now[0] = 6
        self.assertEqual(client._format_gga(), (None, None))
        app.close()
        now[0] = 0
        self.assertEqual(client._format_gga(), (None, None))

    def test_only_crc_valid_rtcm_passes(self):
        valid = rtcm()
        self.assertEqual(rtk.validated_rtcm((valid, object())), valid)
        for bad in (gga(4), b"HTTP/1.1 401 Unauthorized", b"\xb5\x62UBX", valid[:-1],
                    valid + b"x", valid[:-1] + bytes((valid[-1] ^ 1,))):
            self.assertIsNone(rtk.validated_rtcm((bad, object())))

    def test_mock_connection_single_gnss_and_queue_filter(self):
        now = [0.0]
        written = []
        calls = []
        class Receiver:
            in_waiting = 1000
            def __init__(self, **kwargs):
                calls.append(kwargs)
                self.sent = False
            def read(self, amount):
                now[0] += .1
                if not self.sent:
                    self.sent = True
                    return gga(1)
                return b""
            def write(self, data):
                written.append(data)
                return len(data)
            def close(self): pass
        class Client:
            status = {"code": 200, "description": CONFIG["ntrippassword"]}
            connected = True
            def __init__(self, app): self.app = app
            def run(self, **kwargs):
                self.assert_fresh = self.app.fresh()
                kwargs["output"].put((gga(4), None))
                kwargs["output"].put((b"HTTP error", None))
                kwargs["output"].put((rtcm(), None))
            def stop(self): pass
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "session"
            code, meta = rtk.record(CONFIG, .5, output, inventory_fn=inventory, serial_factory=Receiver,
                                    client_factory=Client, clock=lambda: now[0])
            self.assertEqual(code, 0)
            self.assertEqual(written, [rtcm()])
            self.assertEqual(calls[0]["port"], "FAKE_GNSS")
            self.assertTrue(calls[0]["exclusive"])
            self.assertEqual(meta["quality_counts"], {"gps_fix": 1})
            self.assertFalse(meta["rtk_fixed_observed"])
            self.assertEqual(meta["http_status_codes"], [200])
            self.assertEqual(meta["rejected_queue_items"], 2)
            self.assertIsNotNone(meta["rtcm_last_written_elapsed_s"])
            self.assertGreater(meta["rtcm_last_write_age_s"], 0)
            self.assertTrue(meta["ntrip_connected_before_stop"])
            for file in output.iterdir():
                self.assertEqual(file.stat().st_mode & 0o777, 0o600)
                data = file.read_bytes()
                self.assertNotIn(CONFIG["ntrippassword"].encode(), data)
                self.assertNotIn(CONFIG["ntripuser"].encode(), data)

    def test_no_fresh_fix_never_connects(self):
        now = [0.0]
        class Receiver:
            in_waiting = 0
            def __init__(self, **kwargs): pass
            def read(self, amount):
                now[0] += 1
                return gga(0)
            def write(self, data): raise AssertionError("No receiver write allowed")
            def close(self): pass
        def forbidden(app):
            raise AssertionError("No NTRIP client without a fresh measured GGA")
        with tempfile.TemporaryDirectory() as temp:
            code, meta = rtk.record(CONFIG, 30, Path(temp) / "session", inventory_fn=inventory,
                                    serial_factory=Receiver, client_factory=forbidden, clock=lambda: now[0])
            self.assertEqual(code, 3)
            self.assertEqual(meta["status"], "no_fresh_gga")
            self.assertEqual(now[0], 30)  # GNSS/raw reception continues until requested end.
            self.assertFalse(meta["ntrip_started"])

    def test_stale_fix_stops_before_more_corrections_are_written(self):
        now = [0.0]
        written = []
        clients = []
        class Receiver:
            in_waiting = 0
            def __init__(self, **kwargs): pass
            def read(self, amount):
                now[0] += 1
                if now[0] == 1:
                    return gga(1)
                if clients:
                    clients[0].queue.put((rtcm(), None))
                return b""
            def write(self, raw):
                written.append(now[0])
                return len(raw)
            def close(self): pass
        class Client:
            status = {"code": 200}
            def __init__(self, app):
                self.app = app
                clients.append(self)
            def run(self, **kwargs): self.queue = kwargs["output"]
            def stop(self): self.stopped = True
        with tempfile.TemporaryDirectory() as temp:
            code, meta = rtk.record(CONFIG, 10, Path(temp) / "session", inventory_fn=inventory,
                                    serial_factory=Receiver, client_factory=Client, clock=lambda: now[0])
            self.assertEqual(code, 3)
            self.assertEqual(meta["status"], "no_fresh_gga")
            self.assertEqual(now[0], 10)  # No stale forwarding; recorder remains available for recovery.
            self.assertTrue(written)
            self.assertTrue(all(t <= 6 for t in written))
            self.assertFalse(clients[0].app.fresh())

    def test_pty_receives_only_rtcm_and_raw_gnss_is_preserved(self):
        from serial import Serial
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        opened = Event()
        received = []
        failures = []
        def serial_factory(**kwargs):
            self.assertEqual(kwargs["port"], port)
            self.assertTrue(kwargs["exclusive"])
            handle = Serial(**kwargs)
            opened.set()
            return handle
        def fake_inventory():
            result = inventory()
            result["serial_ports"][0]["device"] = port
            return result
        class Client:
            status = {"code": 200}
            def __init__(self, app): self.app = app
            def run(self, **kwargs):
                for raw in (gga(4), b"HTTP/1.1 200 OK", b"\xb5\x62command", rtcm()):
                    kwargs["output"].put((raw, None))
            def stop(self): pass
        def peer():
            try:
                if not opened.wait(2):
                    raise AssertionError("PTY serial did not open")
                os.write(master, gga(1))
                if not select.select([master], [], [], 2)[0]:
                    raise AssertionError("No RTCM returned to PTY")
                received.append(os.read(master, 4096))
            except BaseException as exc:
                failures.append(exc)
        thread = Thread(target=peer)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as temp:
                output = Path(temp) / "session"
                code, meta = rtk.record(CONFIG, .4, output, inventory_fn=fake_inventory,
                                        serial_factory=serial_factory, client_factory=Client)
                thread.join(3)
                self.assertFalse(thread.is_alive())
                self.assertFalse(failures)
                self.assertEqual(code, 0)
                self.assertEqual(b"".join(received), rtcm())
                self.assertEqual((output / "raw.bin").read_bytes(), gga(1))
                self.assertEqual(meta["rtcm_frames_written"], 1)
        finally:
            thread.join(3)
            os.close(master)
            os.close(slave)

    def test_library_exception_and_debug_log_cannot_leak_credentials(self):
        now = [0.0]
        class Receiver:
            in_waiting = 0
            def __init__(self, **kwargs): pass
            def read(self, amount):
                now[0] += .1
                return gga(1)
            def close(self): pass
        class Client:
            def __init__(self, app): pass
            def run(self, **kwargs):
                logging.getLogger("pygnssutils.gnssntripclient").critical(CONFIG["ntrippassword"])
                raise RuntimeError(CONFIG["ntripuser"] + CONFIG["ntrippassword"])
            def stop(self): pass
        captured = io.StringIO()
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(captured), contextlib.redirect_stderr(captured):
            output = Path(temp) / "session"
            code, meta = rtk.record(CONFIG, 1, output, inventory_fn=inventory,
                                    serial_factory=Receiver, client_factory=Client, clock=lambda: now[0])
            self.assertEqual(code, 4)  # retryable network failure, no RTCM by requested end
            all_text = captured.getvalue() + (output / "metadata.json").read_text()
            for key in ("ntripuser", "ntrippassword"):
                self.assertNotIn(CONFIG[key], all_text)

    def test_cleanup_closes_all_handles_despite_receiver_and_fsync_errors(self):
        now = [0.0]
        handles = []
        original_file = rtk.private_file
        original_fsync = os.fsync
        sync_calls = [0]
        def capture_file(*args):
            result = original_file(*args)
            handles.append(result)
            return result
        def failing_first_sync(fd):
            sync_calls[0] += 1
            if sync_calls[0] == 1:
                raise OSError(CONFIG["ntrippassword"])
            return original_fsync(fd)
        class Receiver:
            in_waiting = 0
            def __init__(self, **kwargs): pass
            def read(self, amount):
                now[0] += .1
                return b""
            def close(self): raise OSError(CONFIG["ntrippassword"])
        with tempfile.TemporaryDirectory() as temp, patch.object(rtk, "private_file", capture_file), patch.object(rtk.os, "fsync", failing_first_sync):
            output = Path(temp) / "session"
            code, meta = rtk.record(CONFIG, .2, output, inventory_fn=inventory,
                                    serial_factory=Receiver, clock=lambda: now[0])
            self.assertEqual(code, 1)
            self.assertEqual(len(handles), 4)
            self.assertTrue(all(handle.closed for handle in handles))
            saved = json.loads((output / "metadata.json").read_text())
            self.assertEqual(saved["status"], "error")
            self.assertIn("receiver_close_failed", saved["cleanup_error_codes"])
            self.assertIn("data_0_flush_failed", saved["cleanup_error_codes"])
            self.assertNotIn(CONFIG["ntrippassword"], json.dumps(saved))

    def test_reconnect_and_optional_rate_use_same_exclusive_receiver(self):
        now=[0.];writes=[];attempts=[];reads=[]
        class Receiver:
            in_waiting=0
            def __init__(self,**kw):self.kw=kw
            def read(self,n):now[0]+=.1;reads.append(now[0]);return gga(4)
            def write(self,data):writes.append(data);return len(data)
            def close(self):pass
        class Client:
            connected=True;status={'code':200}
            def __init__(self,app):attempts.append(now[0])
            def run(self,**kw):
                if len(attempts)==1:raise OSError('fixture network down')
                kw['output'].put((rtcm(),None))
            def stop(self):self.connected=False
        with tempfile.TemporaryDirectory() as temp:
            code,meta=rtk.record(CONFIG,2.,Path(temp)/'session',inventory_fn=inventory,
                                serial_factory=Receiver,client_factory=Client,clock=lambda:now[0],requested_hz=10)
            self.assertEqual(code,0);self.assertEqual(len(attempts),2);self.assertGreaterEqual(len(reads),20)
            self.assertTrue(writes[0].startswith(b'\xb5\x62\x06\x08'))
            self.assertEqual(writes[1:],[rtcm()])
            self.assertEqual(meta['rate_request']['requested_hz'],10)
            self.assertFalse(meta['rate_request']['ack_verified'])
            self.assertTrue(meta['receiver_configuration_commands_sent'])

    def test_user_interrupt_stops_retry_lifecycle(self):
        now=[0.];stops=[]
        class Receiver:
            in_waiting=0
            def __init__(self,**kw):pass
            def read(self,n):
                now[0]+=.1
                if now[0]>.5:raise KeyboardInterrupt
                return gga(4)
            def close(self):pass
        class Client:
            connected=True;status={'code':200}
            def __init__(self,app):pass
            def run(self,**kw):pass
            def stop(self):stops.append(now[0])
        with tempfile.TemporaryDirectory() as temp:
            code,meta=rtk.record(CONFIG,None,Path(temp)/'session',inventory_fn=inventory,
                                serial_factory=Receiver,client_factory=Client,clock=lambda:now[0])
            self.assertEqual(code,130);self.assertEqual(meta['status'],'interrupted');self.assertTrue(stops)
            self.assertEqual(meta['connection_at_stop']['state'],'stopped')

    def test_partial_file_creation_closes_previously_opened_handles(self):
        handles = []
        original_file = rtk.private_file
        def failing_third_file(*args):
            if len(handles) == 2:
                raise OSError("fixture creation failure")
            result = original_file(*args)
            handles.append(result)
            return result
        class Receiver:
            def __init__(self, **kwargs): pass
            def close(self): self.closed = True
        with tempfile.TemporaryDirectory() as temp, patch.object(rtk, "private_file", failing_third_file):
            code, meta = rtk.record(CONFIG, .1, Path(temp) / "session", inventory_fn=inventory,
                                    serial_factory=Receiver)
            self.assertEqual(code, 1)
            self.assertIsNone(meta)
            self.assertEqual(len(handles), 2)
            self.assertTrue(all(handle.closed for handle in handles))


if __name__ == "__main__":
    unittest.main()
