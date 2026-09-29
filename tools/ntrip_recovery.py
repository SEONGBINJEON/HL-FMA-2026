"""Nonblocking lifecycle for the pinned pygnssutils asynchronous client.

Only record() owns/writes the GNSS port. Each connection has its own bounded,
receipt-timestamped queue; abandoned workers cannot inject into a new generation.
No exception text, endpoint credentials, or synthetic freshness is exposed.
"""
from queue import Queue, Empty, Full
import time


class ReceiptQueue(Queue):
    def __init__(self, clock):
        super().__init__(maxsize=1024)
        self.clock = clock
        self.dropped = 0

    def put(self, item, block=True, timeout=None):
        try:
            super().put((self.clock(), item), block=False)
        except Full:
            self.dropped += 1


class Recovery:
    def __init__(self, app, factory, config, clock=time.monotonic,
                 retry_min=1., retry_max=15., silent_s=8., max_queue_age=1.):
        self.app, self.factory, self.config, self.clock = app, factory, config, clock
        self.retry_min, self.retry_max = retry_min, retry_max
        self.silent_s, self.max_queue_age = silent_s, max_queue_age
        self.client = None
        self.queue = ReceiptQueue(clock)
        self.next_attempt = 0.
        self.backoff = retry_min
        self.started_at = self.last_valid = None
        self.closed = False
        self.attempts = self.reconnects = self.stale_dropped = 0
        self.state = 'waiting_for_fresh_gga'
        self.http_codes = []
        self.last_connected = None

    def retire(self, reason):
        if self.client is not None:
            try:
                self.client.stop()  # pinned implementation sets Events; no join/network wait
            except Exception:
                pass
        self.client = None
        self.queue = ReceiptQueue(self.clock)
        self.state = reason
        self.next_attempt = self.clock() + self.backoff
        self.backoff = min(self.retry_max, self.backoff * 2)

    def tick(self):
        if self.closed:
            return
        now = self.clock()
        if not self.app.fresh():
            if self.client is not None:
                self.retire('waiting_for_fresh_gga')
            self.state = 'waiting_for_fresh_gga'
            return
        if self.client is not None:
            code = getattr(self.client, 'status', {}).get('code')
            if type(code) is int and 100 <= code <= 599 and code not in self.http_codes:
                self.http_codes.append(code)
            # connected is only the library lifecycle flag, NOT socket/auth/FIX proof.
            flag = getattr(self.client, 'connected', True)
            self.last_connected = flag if type(flag) is bool else None
            if flag is False:
                self.retire('retry_after_disconnect')
            elif now - (self.last_valid if self.last_valid is not None else self.started_at) > self.silent_s:
                self.retire('retry_after_rtcm_silence')
        if self.client is None and now >= self.next_attempt:
            self.attempts += 1
            self.reconnects = max(0, self.attempts - 1)
            self.queue = ReceiptQueue(self.clock)
            self.started_at, self.last_valid = now, None
            try:
                self.client = self.factory(self.app)
                self.client.run(**self.config, datatype='RTCM', ggamode=0, output=self.queue)
                self.state = 'awaiting_valid_rtcm'
            except Exception:
                self.retire('retry_after_start_failure')

    def items(self):
        for _ in range(128):
            try:
                received_at, item = self.queue.get_nowait()
            except Empty:
                return
            if not 0 <= self.clock() - received_at <= self.max_queue_age:
                self.stale_dropped += 1
                continue
            if self.closed or not self.app.fresh():
                continue
            yield item

    def valid_written(self):
        self.last_valid = self.clock()  # only after CRC validation + full receiver write
        self.backoff = self.retry_min
        self.state = 'receiving_valid_rtcm'

    def snapshot(self):
        return {'state': self.state, 'attempts': self.attempts, 'reconnects': self.reconnects,
                'library_connected': self.last_connected,
                'valid_rtcm_age_s': None if self.last_valid is None else self.clock()-self.last_valid,
                'queue_stale_dropped': self.stale_dropped, 'queue_overflow_dropped': self.queue.dropped,
                'http_status_codes': list(self.http_codes)}

    def close(self):
        self.closed = True
        self.retire('stopped')
