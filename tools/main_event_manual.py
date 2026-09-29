"""Explicit stopped autonomy -> existing joystick -> stopped manual handoff.

This module opens no device on import. Vehicle motion is only through the
operator's existing joystick window; recovery uses a new forward GNSS bootstrap.
"""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
from main_event_core import Stationary
from main_event_signal import atomic_json
from main_event_plan import ROOT,digest,NEXT_RECOVERY


def current_mission(saved):
    if saved.get('section','').startswith('parking_t_'):return 't_parking'
    if saved.get('section','').startswith('parking_p_'):return 'p_parking'
    gates=saved.get('gates',[]);i=saved.get('index',len(gates))
    return gates[i]['name'] if i<len(gates) else None


def stopped(saved):
    if not all(saved.get(k) is True for k in ('stop_ack','stationary_observed','process_released')):
        raise ValueError('confirmed stop and released device ownership required before manual handoff')


def wait_stop_tx_idle(client,journal,timeout=.04):
    """Do not stack DISARM frames on the USB TX queue; never flush/discard it."""
    begin=time.monotonic()
    pending=getattr(client.transport,'out_waiting',0)
    if not pending:return
    while getattr(client.transport,'out_waiting',0):
        if client.failed or time.monotonic()-begin>=timeout:
            raise ValueError('final DISARM TX queue did not drain within40ms; stop unconfirmed')
        time.sleep(.002)
    journal.event('stop_tx_drained',initial_bytes=pending,wait_s=time.monotonic()-begin)


def confirm_stop(client,session,journal,timeout=3.):
    """Same-boot newer DISARMED response + observed encoder settling; then close."""
    report={'stop_ack':False,'stationary_observed':False,'process_released':False,'boot':None}
    stable=Stationary();boot=req=None
    try:
        command=session.emergency_command()
        if command is None or client.failed:raise ValueError('manual board unavailable for final DISARM')
        boot,req=command.boot_id,command.request_id
        wait_stop_tx_idle(client,journal)
        client.stop('manual handoff final DISARM')
        expected_stop_message=getattr(session,'message',None)
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            wait_stop_tx_idle(client,journal)
            client.tick(time.monotonic());now=time.monotonic();s=session.status
            if client.failed:raise ValueError('manual board connection failed during final stop')
            if s is None:continue
            if getattr(session,'message',None)!=expected_stop_message:raise ValueError('new session fault during final stop')
            if hasattr(session,'_step_ok') and not session._step_ok(s):raise ValueError('invalid board status during final stop')
            if s.boot_id!=boot:raise ValueError('manual board reboot during final stop')
            fresh=session.last_status_at is not None and 0<=now-session.last_status_at<.15
            neutral=(s.state==0 and not any((s.requested_drive,s.actual_drive,s.requested_steer,s.actual_steer)))
            if fresh and neutral and s.request_id>req:report['stop_ack']=True
            board={'boot':s.boot_id,'request':s.request_id,'encoder':s.encoder_count,
                   'requested_drive':s.requested_drive,'actual_drive':s.actual_drive,
                   'received_at':session.last_status_at}
            if report['stop_ack'] and neutral and stable.update(board,now):
                report.update(stationary_observed=True,boot=boot,encoder=s.encoder_count);break
            if not neutral or not fresh:stable.reset()
            time.sleep(.01)
        if not report['stationary_observed']:report['reason']='manual_stop_unconfirmed'
    except Exception as exc:report['reason']=str(exc)
    finally:
        client.transport.close();report['process_released']=True
    report['finished_wall']=time.time()
    journal.event('manual_final_stop',**report)
    return report


def run_manual(old,mission,collision_recovery_confirmed=False):
    if mission!='s_curve' or collision_recovery_confirmed is not True:
        raise ValueError('수동 복구는 실제 S 충돌 후 심판 허용 예외만 가능; T/P/더미 일반 수동 통과 금지')
    old=Path(old);saved=json.loads((old/'state.json').read_text());info=json.loads((old/'session.json').read_text())
    stopped(saved)
    if mission not in NEXT_RECOVERY or current_mission(saved)!=mission:
        raise ValueError('collision recovery only for current S mission')
    token=uuid.uuid4().hex;folder=ROOT/'data'/('me_manual_'+token[:12])
    lock=os.open(ROOT/'data/main_event.lock',os.O_RDWR|os.O_CREAT,0o600)
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        previous=old/'manual_handoff.json'
        if previous.exists():
            previous.rename(old/('manual_handoff_superseded_'+token+'.json'))
        code=subprocess.run([sys.executable,'tools/joystick_desktop.py','--connect','--mapping-confirmed',
            '--initial-drive-pwm','60','--confirm-final-stop','--output-dir',str(folder)],cwd=ROOT).returncode
        report=json.loads((folder/'final_stop.json').read_text())
        metadata=json.loads((folder/'metadata.json').read_text())
        if code or metadata.get('status')!='completed':raise ValueError('manual window failed; inspect '+str(folder))
        stopped(report)
        record={'token':token,'mission':mission,'run_id':info['run_id'],'epoch':info['epoch'],
                'previous_state_sha256':digest(old/'state.json'),'manual_log_dir':str(folder),
                'stop_report_sha256':digest(folder/'final_stop.json'),'boot':report['boot'],
                'finished_wall':report['finished_wall'],'heading_mode':'forward_bootstrap',
                'collision_recovery_confirmed':True,'rule_basis':'operator_reports_actual_S_collision_and_judge_permission'}
        atomic_json(old/'manual_handoff.json',record)
    finally:os.close(lock)
    print(json.dumps({'manual_passage_stopped':True,'mission':mission,'log':str(folder),
        'next':'python3 tools/main_event.py recover --after-manual'},ensure_ascii=False))
    return 0


def load_handoff(old,info):
    old=Path(old);r=json.loads((old/'manual_handoff.json').read_text())
    if r.get('mission')!='s_curve' or r.get('collision_recovery_confirmed') is not True:
        raise ValueError('old/general manual passage receipt is not permitted under corrected rule')
    if r['run_id']!=info['run_id'] or r['epoch']!=info['epoch'] or r['previous_state_sha256']!=digest(old/'state.json'):
        raise ValueError('manual handoff does not match stopped autonomous session')
    path=Path(r['manual_log_dir'])/'final_stop.json'
    if digest(path)!=r['stop_report_sha256']:raise ValueError('manual stop report changed')
    report=json.loads(path.read_text());stopped(report)
    if report['boot']!=r['boot']:raise ValueError('manual boot mismatch')
    return r
