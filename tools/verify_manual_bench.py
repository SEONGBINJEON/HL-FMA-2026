#!/usr/bin/env python3
"""Finite real-Mega protocol checks, ONLY after BENCH-only firmware is identified.

Tests send logical movement requests to firmware whose actual outputs are locked.
Motor power must remain isolated. No non-BENCH or legacy firmware receives TX.
"""
import argparse
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
import select
import sys
import time
import uuid
from zoneinfo import ZoneInfo

from manual_control import MegaTransport, Session, Client, Journal, ROOT
from manual_protocol import Command, Status, FrameParser, encode_command, ARM, CONTROL, DISARM, ARMED


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--motor-power-off', action='store_true')
    args = ap.parse_args()
    if not args.motor_power_off:
        ap.error('--motor-power-off is required; no port opened')
    folder = ROOT / 'data' / (datetime.now(ZoneInfo('Asia/Seoul')).strftime('manual_bench_check_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8])
    folder.mkdir()
    print('OUTPUT=' + str(folder), flush=True)
    report = {'status':'starting','checks':[], 'source_sha256':{},
              'started_utc':datetime.now().astimezone().isoformat(),
              'scope':'Real board BENCH protocol/telemetry only. No physical PWM, brake, stopping-distance or E-stop validation.',
              'operator_condition':'Latest user report: vehicle battery/motor power OFF; instructed to maintain isolation throughout continued BENCH work.'}
    for name in ('verify_manual_bench.py','manual_control.py','manual_protocol.py'):
        raw=Path(__file__).with_name(name).read_bytes()
        (folder/name).write_bytes(raw)
        report['source_sha256'][name]=hashlib.sha256(raw).hexdigest()
    transport=None
    pc_journal=None
    current=None
    history=[]
    phase='identify'
    parser=FrameParser()
    start=time.monotonic()
    code=1
    events=(folder/'events.jsonl').open('x',buffering=1)
    rx=(folder/'rx.bin').open('xb')
    tx=(folder/'tx.bin').open('xb')

    def event(kind, **kw):
        events.write(json.dumps({'host_elapsed_s':time.monotonic()-start,'host_utc':datetime.now().astimezone().isoformat(),
                                'phase':phase,'event':kind,**kw})+'\n')

    def receive(seconds=1.0, after=None, any_frame=False):
        nonlocal current
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            select.select([transport.fd],[],[],min(.01,max(0,end-time.monotonic())))
            data=transport.read(min(4096,transport.in_waiting)) if transport.in_waiting else b''
            if data: rx.write(data);rx.flush();event('rx',hex=data.hex())
            values,errors=parser.feed(data,time.monotonic())
            if errors and current is not None:
                raise RuntimeError('Status parse error: '+str(errors))
            eligible=[]
            for value in values:
                if not isinstance(value,Status):raise RuntimeError('Unexpected non-status frame')
                if value.flags != 33 or value.boot_id==0 or value.request_id==0 or value.actual_drive or value.actual_steer:
                    raise RuntimeError('Not the expected BENCH-only, zero-output firmware')
                if current is not None and value.boot_id != current.boot_id:
                    raise RuntimeError('Unexpected board restart during test')
                current=value;history.append(value);event('status',**asdict(value))
                if any_frame or after is None or value.request_id>after:eligible.append(value)
            if eligible:return eligible[-1]
        raise TimeoutError('No fresh BENCH response in bounded wait')

    def send(raw):
        if current is None or current.flags !=33:
            raise RuntimeError('No identified BENCH firmware; no transmission')
        if transport.out_waiting:raise RuntimeError('Unexpected pending transmit bytes')
        event('tx_attempt',hex=raw.hex())
        n=transport.write(raw);tx.write(raw[:n]);tx.flush();event('tx',bytes=n)
        if n!=len(raw):raise RuntimeError('Partial write; no retry')

    def exchange(action,drive=0,steer=0,deadman=0):
        previous=current
        raw=encode_command(Command(previous.boot_id,previous.request_id,action,drive,steer,deadman))
        send(raw)
        return receive(after=previous.request_id),raw,previous

    def check(name, predicate, **details):
        report['checks'].append({'name':name,'passed':bool(predicate),**details})
        event('check',**report['checks'][-1])
        if not predicate:raise AssertionError(name)

    def neutral_arm():
        value,_,_=exchange(DISARM)
        value,_,_=exchange(CONTROL)
        check(phase+'_neutral',value.state==0 and value.reason==11)
        value,_,_=exchange(ARM)
        check(phase+'_arm',value.state==1 and value.reason==9)
        return value

    try:
        transport=MegaTransport()
        first=receive(seconds=4)
        report['first_status']=asdict(first)
        phase='initial_arm_wait'
        neutral_arm()
        for _ in range(3):value,_,_=exchange(CONTROL)
        check('arm_wait_without_deadman_keeps_zero',value.state==1 and value.reason==9 and value.requested_drive==0)

        phase='communication_timeout'
        value,go,issued=exchange(CONTROL,drive=1,deadman=1)
        check('logical_forward_bench_output_zero',value.state==1 and value.requested_drive==1 and value.actual_drive==0)
        stop_seen=None
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline:
            value=receive(.3,any_frame=True)
            if value.state==0 and value.reason==2:
                stop_seen=value;break
        delay=((stop_seen.millis-issued.millis)&0xffffffff) if stop_seen else None
        check('no_commands_disarms_by_200ms_telemetry',stop_seen is not None and delay<=200,
              observed_mcu_ms_from_request_issue=delay,limit_note='150ms logic deadline plus 50ms telemetry interval; not physical stopping time')

        phase='stale_and_rearm'
        before=current.request_id;send(go);value=receive(after=before)
        check('old_request_rejected',value.state==0 and value.reason==4)
        value,_,_=exchange(ARM)
        check('rearm_without_new_neutral_rejected',value.state==0 and value.reason==6)

        phase='bad_crc'
        neutral_arm()
        before=current.request_id
        raw=bytearray(encode_command(Command(current.boot_id,current.request_id,CONTROL,1,0,1)));raw[-1]^=1
        send(bytes(raw));value=receive(after=before)
        check('bad_crc_disarms',value.state==0 and value.reason==3)

        phase='partial_frame'
        neutral_arm();before=current.request_id
        raw=encode_command(Command(current.boot_id,current.request_id,CONTROL,1,0,1))
        send(raw[:8]);value=receive(after=before)
        check('partial_frame_expires_to_stop',value.state==0 and value.reason==3)

        phase='legacy_flood'
        neutral_arm();before=current.request_id
        send(b'F'*80);value=receive(after=before)
        check('legacy_and_flood_do_not_move',value.state==0 and value.reason==3 and value.requested_drive==0)

        phase='deadman_release'
        neutral_arm();value,_,_=exchange(CONTROL,drive=1,deadman=1)
        value,_,_=exchange(CONTROL)
        check('deadman_release_disarms',value.state==0 and value.reason==1)

        phase='direction_reversal'
        neutral_arm();exchange(CONTROL,drive=1,deadman=1)
        exchange(CONTROL,deadman=1)
        value,_,_=exchange(CONTROL,drive=-1,deadman=1)
        check('reversal_after_zero_still_requires_rearm',value.state==0 and value.reason==7)

        phase='steering_duration'
        neutral_arm();value,_,issued=exchange(CONTROL,steer=-1,deadman=1)
        steering_stop=None
        for index in range(8):
            if value.state==0:
                steering_stop=value;break
            # Include one neutral steering sample: it must not restart the timer.
            value,_,_=exchange(CONTROL,steer=0 if index==1 else -1,deadman=1)
        delay=((steering_stop.millis-issued.millis)&0xffffffff) if steering_stop else None
        check('repeated_and_neutral_steering_cannot_extend_200ms',
              steering_stop is not None and steering_stop.reason==8 and delay<=300,
              observed_mcu_ms_from_first_request_issue=delay)

        phase='pc_session'
        exchange(DISARM)
        pc_journal=Journal('connect', folder/'pc_client')
        report['pc_client_recording']='pc_client/'
        session=Session();client=Client(transport,session,pc_journal)
        def spin(seconds):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                client.tick(time.monotonic())
                if client.failed:raise RuntimeError('PC Client failed: '+session.message)
                time.sleep(.01)
        spin(.2)
        check('pc_identifies_bench',session.identified and session.status is not None)
        check('pc_explicit_arm_request_accepted',session.request_arm(time.monotonic()))
        spin(.22)
        check('pc_neutral_arm_handshake',session.phase=='ready')
        session.key_down('shift_l');session.key_down('w');spin(.10)
        check('pc_new_keys_reach_requested_forward_only',session.status.requested_drive==1 and session.status.actual_drive==0)
        session.key_up('shift_l');client.stop(session.message);spin(.12)
        check('pc_key_release_stops_board',session.phase=='locked' and session.status.state==0 and session.status.requested_drive==0)
        pc_values,pc_errors=FrameParser().feed((pc_journal.path/'rx.bin').read_bytes(),time.monotonic())
        check('every_pc_test_status_has_zero_actual_output',
              bool(pc_values) and not pc_errors and all(isinstance(v,Status) and v.flags==33 and
                  v.actual_drive==0 and v.actual_steer==0 for v in pc_values),
              observed_status_frames=len(pc_values))
        report['pc_last_status']=asdict(session.status)
        current=session.status
        phase='final_stop'
        send(encode_command(Command(current.boot_id,0,DISARM)))
        final=receive(after=current.request_id)
        check('final_disarmed',final.state==0 and final.requested_drive==0 and final.requested_steer==0)
        check('every_direct_test_status_has_zero_actual_output',all(v.actual_drive==v.actual_steer==0 for v in history),observed_status_frames=len(history))
        report['status']='passed';code=0
    except (OSError,RuntimeError,ValueError,AssertionError,TimeoutError) as exc:
        report.update(status='failed',error=type(exc).__name__+': '+str(exc),failed_phase=phase)
    except KeyboardInterrupt:
        report.update(status='interrupted',failed_phase=phase);code=130
    finally:
        if transport is not None:
            if current is not None:
                try:send(encode_command(Command(current.boot_id,0,DISARM)))
                except (OSError,RuntimeError):pass
            transport.close()
        if pc_journal is not None:
            pc_journal.close()
        report['elapsed_s']=time.monotonic()-start
        report['ended_utc']=datetime.now().astimezone().isoformat()
        (folder/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        events.close();rx.close();tx.close()
        print(json.dumps({'status':report['status'],'passed_checks':sum(x['passed'] for x in report['checks']),
                          'error':report.get('error'),'elapsed_s':report['elapsed_s']},ensure_ascii=False),flush=True)
    return code


if __name__=='__main__':
    raise SystemExit(main())
