#!/usr/bin/env python3
"""Two-stick control. Default: real gamepad preview, NO Arduino access.

--connect opens only the known Mega; no serial TX until CRC-valid type150 status.
Explicit neutral preparation in the active local page is required to drive.
"""
import argparse
from dataclasses import asdict
from datetime import datetime,timezone
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import os
from pathlib import Path
import queue
import secrets
import signal
import threading
import time
from gamepad_input import Gamepad,direction
from joystick_protocol import Parser,encode_command
from step_manual_control import Session,Client,MegaTransport


class Journal:
    def __init__(self,path):
        self.path=path;path.mkdir(parents=True,exist_ok=False)
        self.rows=(path/'events.jsonl').open('x',buffering=1)
        self.files={n:(path/(n+'.bin')).open('xb') for n in ('rx','tx','gamepad')}
        for name in ('joystick_control.py','joystick_control.html','joystick_protocol.py','gamepad_input.py','step_manual_control.py','step_manual_protocol.py'):
            (path/name).write_bytes(Path(__file__).with_name(name).read_bytes())
    def event(self,name,**fields):
        self.rows.write(json.dumps({'host_utc':datetime.now(timezone.utc).isoformat(),'event':name,**fields},ensure_ascii=False)+'\n')
    def bytes(self,direction,raw):self.files[direction].write(raw);self.files[direction].flush()
    def close(self):
        self.rows.close()
        for f in self.files.values():f.close()


class Driver:
    def __init__(self,pad,session,client):
        self.pad=pad;self.session=session;self.client=client
        self.drive=self.steer=0;self.drive_pwm=60;self.confirmed=False;self.last_ui=0.;self.focused=False
    def stop(self,reason):
        # The periodic Client.tick is the sole writer. An immediate extra
        # DISARM here can still be queued when tick tries to reply to a status.
        self.session.stop(reason)
    def input(self):
        a,b=self.pad.poll()
        for step in self.pad.pwm_steps:
            self.drive_pwm=max(0,min(255,self.drive_pwm+5*step))
        self.drive=direction(a,self.drive);self.steer=direction(b,self.steer)
        effective_drive=self.drive if self.drive_pwm else 0
        keys=({'w'} if effective_drive>0 else {'s'} if effective_drive<0 else set())|({'d'} if self.steer>0 else {'a'} if self.steer<0 else set())
        for key in self.session.keys-keys:self.session.key_up(key)
        for key in keys-self.session.keys:self.session.key_down(key)
        return a,b
    def action(self,msg,now):
        if msg.get('action')=='pulse':
            self.last_ui=now;self.focused=bool(msg.get('focused'));self.session.focus(self.focused)
            if not self.focused:self.stop('조작 창 비활성 · 정지')
        elif msg.get('action')=='stop':self.stop('사용자 정지')
        elif msg.get('action')=='confirm':
            if not self.drive and not self.steer:self.confirmed=True
        elif msg.get('action')=='arm':
            if self.confirmed and self.focused and now-self.last_ui<.25 and not self.drive and not self.steer and self.client:
                self.session.request_arm(now)
            else:
                self.session.message=('스틱을 모두 중립으로 놓으세요' if self.drive or self.steer else
                                      '조작 화면을 클릭한 뒤 다시 준비하세요' if not self.focused else
                                      '입력 방향 확인 또는 보드 연결 대기')


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--connect',action='store_true');ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--port',type=int,default=8767);ap.add_argument('--duration',type=float,default=0)
    ap.add_argument('--mapping-confirmed',action='store_true',help='Reuse the operator-confirmed normal two-stick mapping')
    args=ap.parse_args()
    if not (args.duration==0 or 1<=args.duration<=3600):ap.error('duration must be 0 (until stopped) or 1..3600')
    journal=Journal(args.output_dir);pad=Gamepad();session=Session();session.message='스틱 입력 확인 · 아직 주행 준비 전'
    # Keep echo disabled on close: no accidental telemetry-to-command feedback.
    transport=MegaTransport() if args.connect else None
    if transport:transport.saved=None
    client=Client(transport,session,journal) if transport else None
    if client:client.parser=Parser()
    driver=Driver(pad,session,client);actions=queue.Queue(maxsize=32);token=secrets.token_urlsafe(24)
    if client:client.encoder=lambda command:encode_command(command,driver.drive_pwm)
    driver.confirmed=args.mapping_confirmed
    stop=threading.Event();shared={'mode':'실차 연결' if args.connect else '입력 확인 (차량 연결 없음)'}
    html=Path(__file__).with_suffix('.html').read_text().replace('__TOKEN__',token).encode()
    origin=f'http://127.0.0.1:{args.port}'
    metadata={'started_utc':datetime.now(timezone.utc).isoformat(),'argv':__import__('sys').argv,'mode':shared['mode'],'gamepad_identity':pad.identity,'axis_codes':pad.axis_codes,'mapping':{'drive':'ABS_Y inverted (left vertical)','steer':'ABS_RX (right horizontal)'},'physical_direction_verified':False,'ground_test_authorized':True,'user_condition':'Motor ON, ground trial permitted, cutoff switch available per latest user; no OFF assumption','firmware_type':150,'ultrasonic_mode':'distance_only','drive_pwm_initial':60,'drive_pwm_range':[0,255],'dpad_step':5,'steer_pwm':60,'steering_mode':'continuous_manual','mechanical_endpoints_calibrated':False,'drive_trial_max_ms':None,'steer_trial_max_ms':None,'steer_relative_adc_limit':None,'wireless_loss_behavior_verified':False}
    (args.output_dir/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def answer(self,data,mime='application/json',status=200):
            self.send_response(status);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        def do_GET(self):
            if self.headers.get('Host')!=f'127.0.0.1:{args.port}':self.send_error(403);return
            if self.path=='/':self.answer(html,'text/html; charset=utf-8')
            elif self.path=='/state':self.answer(json.dumps(shared,ensure_ascii=False).encode())
            else:self.send_error(404)
        def do_POST(self):
            if self.path!='/action' or self.headers.get('Origin')!=origin or self.headers.get('X-Session')!=token:self.send_error(403);return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=512:raise ValueError()
                value=json.loads(self.rfile.read(length));value['received_at']=time.monotonic();actions.put_nowait(value)
                self.answer(b'{}')
            except (ValueError,queue.Full):self.send_error(400)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stop.set())
    started=time.monotonic();last_record=0.
    try:
        while not stop.is_set() and (args.duration==0 or time.monotonic()-started<args.duration):
            now=time.monotonic();a,b=driver.input()
            for raw in pad.raw_events:journal.bytes('gamepad',raw)
            while not actions.empty():
                msg=actions.get_nowait()
                if now-msg['received_at']<.25:driver.action(msg,now)
                else:driver.stop('오래된 화면 요청 · 정지')
            if now-driver.last_ui>=.25:driver.stop('조작 화면 응답 만료 · 정지')
            # Any face button is a stop; axes are the only motion input.
            if any(pad.buttons[:4]):driver.stop('컨트롤러 버튼 정지')
            if client:client.tick(time.monotonic())
            board_age=None if session.last_status_at is None else time.monotonic()-session.last_status_at
            shared={**shared,'updated_utc':datetime.now(timezone.utc).isoformat(),'drive_pwm':driver.drive_pwm,'drive':driver.drive,'steer':driver.steer,'drive_axis':a,'steer_axis':b,'axes':pad.axes[:],'buttons':pad.buttons[:],'phase':session.phase,'message':'보드 연결 종료 · 수신 로그 확인 필요' if client and client.failed else session.message,'confirmed':driver.confirmed,'board':asdict(session.status) if session.status else None,'board_age_s':board_age,'connected':bool(client and session.identified and not client.failed and board_age is not None and board_age<.15),'recording':str(args.output_dir)}
            if client and not session.identified and client.parser.unsupported_banner:
                shared['error']='보드 코드 불일치: HENES F8 문자 명령 코드 수신 · 두 스틱용 type150 필요'
            if now-last_record>.1:journal.event('input_state',**shared);last_record=now
            time.sleep(.01)
    except Exception as exc:
        shared={**shared,'error':str(exc)};journal.event('failed',error=str(exc));raise
    finally:
        if client:client.close()
        pad.close();server.shutdown();server.server_close();journal.close()
        metadata.update(ended_utc=datetime.now(timezone.utc).isoformat(),status='failed' if shared.get('error') else 'completed')
        (args.output_dir/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':main()
