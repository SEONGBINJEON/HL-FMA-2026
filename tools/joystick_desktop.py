#!/usr/bin/env python3
"""Native two-stick window; --connect explicitly opens the known Mega.

Default is gamepad preview only. No browser or HTTP server is used.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import time
import tkinter as tk

from joystick_control import Driver, Journal
from gamepad_input import Gamepad
from joystick_protocol import Parser, encode_command
from step_manual_control import Session, Client, MegaTransport


def connection_message(client, session, now):
    if client is None:
        return False, '입력 확인 모드 · 차량 연결 없음'
    if client.failed:
        return False, '보드 연결 종료 · 프로그램을 종료하고 연결을 확인하세요'
    if not session.identified and client.parser.unsupported_banner:
        return False, '보드 코드 불일치: HENES F8 문자 명령 코드 수신 · 두 스틱용 type150 필요'
    fresh = (session.identified and session.last_status_at is not None
             and now - session.last_status_at < .15)
    return fresh, '두 스틱용 보드 연결됨' if fresh else '두 스틱용 보드 응답 대기 · 준비 불가'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--connect', action='store_true')
    ap.add_argument('--mapping-confirmed', action='store_true')
    ap.add_argument('--output-dir', type=Path)
    ap.add_argument('--confirm-final-stop', action='store_true',
                    help='write same-boot stop/encoder confirmation for main-event handoff')
    ap.add_argument('--initial-drive-pwm', type=int, default=60,
                    help='starting drive PWM (0..255). 0 = drive stays braked unless the D-pad raises it; for steering-only measurement')
    args = ap.parse_args()
    if not 0 <= args.initial_drive_pwm <= 255: ap.error('--initial-drive-pwm must be 0..255')
    output = args.output_dir or Path(__file__).resolve().parents[1] / 'data' / (
        'joystick_desktop_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    # Establish the display before opening any hardware.
    root = tk.Tk(); root.title('HL FMA · 컨트롤러 주행'); root.geometry('820x540')
    journal = pad = client = transport = None
    metadata = {}; closing = False; failed = False
    try:
        journal = Journal(output)
        (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
        pad = Gamepad(); session = Session()
        if args.connect:
            transport = MegaTransport(); transport.saved = None
            client = Client(transport, session, journal); client.parser = Parser()
        driver = Driver(pad, session, client); driver.confirmed = args.mapping_confirmed
        driver.drive_pwm = args.initial_drive_pwm
        if client: client.encoder = lambda command: encode_command(command, driver.drive_pwm)
        metadata = {'started_utc': datetime.now(timezone.utc).isoformat(),
                    'argv': __import__('sys').argv, 'interface': 'tk', 'connect': args.connect,
                    'firmware_type': 150, 'steering_mode': 'continuous_manual',
                    'initial_drive_pwm': args.initial_drive_pwm,
                    'mechanical_endpoints_calibrated': False, 'gamepad_identity': pad.identity,
                    'axis_codes': pad.axis_codes, 'physical_motion_verified': False}
        (output / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
        connection = tk.StringVar(value='보드 응답 대기')
        message = tk.StringVar(value='스틱을 놓고 조작 준비를 누르세요')
        telemetry = tk.StringVar()
        root.configure(bg='#101824')
        def label(variable=None, text='', size=18):
            w=tk.Label(root, textvariable=variable, text=text, font=('sans', -size),
                       fg='white', bg='#101824', justify='left', anchor='w', wraplength=770)
            w.pack(fill='x', padx=22, pady=8); return w
        label(text='왼쪽 스틱: 전후진   |   오른쪽 스틱: 조향', size=25)
        label(connection); label(message); label(telemetry)
        buttons=tk.Frame(root, bg='#101824'); buttons.pack(fill='x', padx=22, pady=15)
        def arm():
            now=time.monotonic()
            driver.action({'action':'pulse', 'focused':root.focus_displayof() is not None}, now)
            driver.action({'action':'arm'}, now)
            journal.event('arm_click', phase=session.phase, message=session.message)
        def confirm(): driver.action({'action':'confirm'}, time.monotonic())
        tk.Button(buttons, text='입력 방향 확인', command=confirm).pack(side='left', padx=6)
        arm_button=tk.Button(buttons, text='조작 준비', command=arm, font=('sans', -22))
        arm_button.pack(side='left', padx=6)
        tk.Button(buttons, text='정지 · Esc', command=lambda:driver.stop('사용자 정지'),
                  bg='#b32f3a', fg='white', font=('sans', -22)).pack(side='right')
        label(text='방향키 위/아래: 구동 PWM ±5 · 두 스틱 동시 조작 가능\n'
                   '스틱을 놓으면 해당 모터 정지 · 다른 창 선택/Esc/앞면 버튼: 전체 정지\n'
                   '연속 조향 · 끝까지 꺾이면 스틱을 놓으세요 · 자동 중앙 복귀 없음\n'
                   '초음파 거리 표시 전용', size=16)
        label(text='기록: '+str(output), size=12)
        def stop_window(*_):
            nonlocal closing
            closing=True
            driver.stop('창 종료 · 정지')
        def focus_out(event):
            # Ignore focus changes between widgets of this same window.
            root.after_idle(lambda: driver.action({'action':'pulse', 'focused':False}, time.monotonic())
                            if not closing and root.focus_displayof() is None else None)
        root.bind('<FocusOut>', focus_out)
        root.bind('<Escape>', lambda _: driver.stop('Esc · 정지'))
        root.protocol('WM_DELETE_WINDOW', stop_window)
        for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, stop_window)
        last_record=last_render=0.
        def tick():
            nonlocal last_record, last_render, failed
            if closing: root.quit(); return
            try:
                now=time.monotonic()
                a,b=driver.input()
                for raw in pad.raw_events: journal.bytes('gamepad', raw)
                driver.action({'action':'pulse', 'focused':root.focus_displayof() is not None}, now)
                if any(pad.buttons[:4]): driver.stop('컨트롤러 버튼 정지')
                if client: client.tick(time.monotonic())
                now=time.monotonic(); connected,reason=connection_message(client, session, now)
                if now-last_record >= .1:
                    journal.event('input_state', phase=session.phase, connected=connected,
                                  drive=driver.drive, steer=driver.steer, drive_pwm=driver.drive_pwm,
                                  drive_axis=a, steer_axis=b, message=session.message,
                                  board=asdict(session.status) if session.status else None)
                    last_record=now
                if now-last_render >= .1:
                    connection.set(reason)
                    message.set('준비됨 · 스틱으로 조작' if session.phase=='ready' else
                                '스틱 조작 중' if session.phase=='active' else session.message)
                    arm_button.configure(state='normal' if connected and driver.confirmed else 'disabled')
                    value=f'구동 PWM {driver.drive_pwm} | 전후진 입력 {a:+.0%} | 조향 입력 {b:+.0%}'
                    s=session.status
                    if s:
                        value+=f'\n보드 명령: 구동 {s.actual_drive}, 조향 {s.actual_steer} | 조향 ADC {s.steering_adc}'
                        value+='\n초음파: '+' / '.join('NO ECHO' if x==999 else f'{x}cm' for x in (s.left_cm,s.center_cm,s.right_cm))
                    telemetry.set(value); last_render=now
                root.after(10, tick)
            except Exception as exc:
                failed=True; journal.event('failed', error=str(exc))
                driver.stop('장치/처리 오류 · 종료'); print(f'종료: {exc}', flush=True); root.quit()
        print('전용 조작 창 실행. 두 스틱 중립 → 조작 준비. 종료: 창 닫기 또는 Ctrl+C.', flush=True)
        print('기록: '+str(output), flush=True)
        root.after(10, tick); root.mainloop()
    except BaseException:
        failed=True
        raise
    finally:
        try:
            if client and args.confirm_final_stop:
                from main_event_manual import confirm_stop
                report=confirm_stop(client,session,journal)
                (output/'final_stop.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
                if not report['stop_ack'] or not report['stationary_observed']:failed=True
            elif client: client.close()
        finally:
            if transport: transport.close()
            if pad: pad.close()
            if journal:
                metadata.update(ended_utc=datetime.now(timezone.utc).isoformat(), status='failed' if failed else 'completed')
                (output/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
                journal.close()
            root.destroy()


if __name__=='__main__': main()
