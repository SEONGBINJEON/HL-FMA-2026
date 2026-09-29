"""Local datagram CLI with run/epoch binding, expiry and replay rejection."""
import json
from pathlib import Path
import socket
import time
import uuid


class Commands:
    def __init__(self, session, run_id, epoch):
        self.run_id, self.epoch = run_id, epoch
        self.path = Path(session)/'control.sock'
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(str(self.path)); self.sock.setblocking(False)
        self.seen = set()

    def poll(self, phase):
        try: raw, peer = self.sock.recvfrom(4096)
        except BlockingIOError: return None
        cmd = None
        try:
            msg = json.loads(raw)
            if msg['run_id'] != self.run_id or msg['epoch'] != self.epoch:
                raise ValueError('foreign/stale session')
            if not 0 <= time.monotonic()-msg['sent_at'] <= 2.:
                raise ValueError('expired command')
            if msg['nonce'] in self.seen: raise ValueError('duplicate command')
            self.seen.add(msg['nonce'])
            if len(self.seen)>4096: raise ValueError('command flood')
            action = msg['action']
            if action == 'start' and phase != 'waiting': raise ValueError('start only from prepared waiting')
            if action in ('localize','choices') and phase not in ('waiting','not_ready'):raise ValueError('localize only before explicit start')
            if action not in ('start','stop','status','localize','choices'): raise ValueError('unknown command')
            cmd = {'action':action,'payload':msg.get('payload',{}),'nonce':msg['nonce']} if action in ('localize','choices') else action
            response = {'accepted':True,'action':action,'phase':phase,'nonce':msg['nonce']}
        except (ValueError,KeyError,TypeError):
            response = {'accepted':False,'reason':'invalid, stale, duplicate or phase-incompatible command','phase':phase}
        try: self.sock.sendto(json.dumps(response).encode(),peer)
        except OSError: pass
        return cmd

    def close(self):
        self.sock.close(); self.path.unlink(missing_ok=True)


def send(session, action, payload=None):
    session = Path(session)
    info = json.loads((session/'session.json').read_text())
    with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as sock:
        sock.bind('\0hl-fma-'+uuid.uuid4().hex)
        sock.settimeout(2.)
        msg = {'run_id':info['run_id'],'epoch':info['epoch'],'action':action,
               'sent_at':time.monotonic(),'nonce':uuid.uuid4().hex}
        if payload is not None:msg['payload']=payload
        sock.sendto(json.dumps(msg).encode(),str(session/'control.sock'))
        return json.loads(sock.recv(4096))
