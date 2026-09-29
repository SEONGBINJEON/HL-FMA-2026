import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from ntrip_recovery import Recovery


class App:
    ok = True
    def fresh(self): return self.ok


class Client:
    connected = True
    status = {'code': 200}
    def run(self, **kw): self.queue = kw['output']
    def stop(self): self.connected = False


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        self.app = App()
        self.clients = []
        def factory(app):
            c = Client(); self.clients.append(c); return c
        self.r = Recovery(self.app, factory, {}, lambda: self.now)

    def test_disconnect_new_generation_and_late_old_output(self):
        self.r.tick(); old = self.clients[-1]
        old.connected = False; self.r.tick()
        old.queue.put(b'old')
        self.now = 1.; self.r.tick()
        self.assertEqual(list(self.r.items()), [])
        self.clients[-1].queue.put(b'new')
        self.assertEqual(list(self.r.items()), [b'new'])
        self.assertEqual(self.r.attempts, 2)

    def test_initial_failures_backoff_and_network_return(self):
        calls = []
        def factory(app):
            calls.append(self.now)
            if len(calls) < 3: raise OSError('do not log credential-bearing exceptions')
            return Client()
        self.r.factory = factory
        for t in [0., .5, 1., 2., 3.]:
            self.now = t; self.r.tick()
        self.assertEqual(calls, [0., 1., 3.])
        self.assertEqual(self.r.state, 'awaiting_valid_rtcm')
        self.r.valid_written()
        self.assertEqual(self.r.backoff, 1.)

    def test_gga_loss_waits_and_recovers_without_closing_app(self):
        self.r.tick(); self.app.ok = False; self.r.tick()
        self.assertIsNone(self.r.client)
        self.now = 20.; self.r.tick(); self.assertEqual(self.r.attempts, 1)
        self.app.ok = True; self.r.tick(); self.assertEqual(self.r.attempts, 2)

    def test_library_flag_without_rtcm_does_not_mean_ready(self):
        self.r.tick(); self.now = 9.; self.r.tick()
        self.assertEqual(self.r.state, 'retry_after_rtcm_silence')
        self.assertIsNone(self.r.snapshot()['valid_rtcm_age_s'])

    def test_queue_age_and_overflow_never_refresh_corrections(self):
        self.r.tick(); self.clients[-1].queue.put(b'old'); self.now = 2.
        self.assertEqual(list(self.r.items()), [])
        self.assertIsNone(self.r.last_valid)
        for _ in range(1100): self.r.queue.put(b'x')
        self.assertEqual(self.r.queue.qsize(), 1024)
        self.assertEqual(self.r.queue.dropped, 76)

    def test_shutdown_prevents_all_future_connections_and_forwarding(self):
        self.r.tick(); c = self.clients[-1]; self.r.close()
        c.queue.put(b'late'); self.now = 100.; self.r.tick()
        self.assertEqual(self.r.attempts, 1)
        self.assertEqual(list(self.r.items()), [])


if __name__ == '__main__': unittest.main()
