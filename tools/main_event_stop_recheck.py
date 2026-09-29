"""Fresh DISARM-only evidence for a failed, released preparation session.
Never edits the failed state. Archives its pointer only after fresh board proof.
"""
import fcntl,json,os,time
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
from main_event_signal import atomic_json
from main_event_plan import ROOT,digest
from main_event_manual import confirm_stop,stopped
from step_manual_control import Client,Session,MEGA_PATH
from joystick_protocol import Parser,encode_command
from live_steering_trial import PosixSerialTransport,TrialJournal


def recheck(active,vehicle_secured):
    if not vehicle_secured:raise ValueError('차량 기계적 고정·앞뒤 공간 확보 후 --vehicle-secured 필요; ARM/구동 없음')
    active=Path(active);pointer=json.loads(active.read_text());old=Path(pointer['session'])
    saved=json.loads((old/'state.json').read_text())
    if saved.get('process_released') is not True or saved.get('phase')!='stopped':
        raise ValueError('이전 실행기의 종료/포트 해제가 확인되지 않음; 재확인 연결 거부')
    lock=os.open(ROOT/'data/main_event.lock',os.O_RDWR|os.O_CREAT,0o600)
    journal=transport=None;report={};code=3
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        from main_event_release import verify
        release=verify()
        state_hash=digest(old/'state.json');pointer_hash=digest(active)
        folder=ROOT/'data'/('me_stop_recheck_'+datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S_%f'))
        journal=TrialJournal(folder,'measured',{'mode':'DISARM_only_stop_recheck','previous_session':str(old),
            'previous_state_sha256':state_hash,'release_id':release['release_id'],'vehicle_secured_operator_confirmed':True,'ARM_permitted':False})
        transport=PosixSerialTransport(MEGA_PATH,check_identity=True)
        session=Session();client=Client(transport,session,journal,encoder=lambda c:encode_command(c,0));client.parser=Parser()
        until=time.monotonic()+5.
        while time.monotonic()<until and not client.failed:
            client.tick(time.monotonic())
            if session.identified and session.status is not None:break
            time.sleep(.005)
        if not session.identified or client.failed or not session._step_ok(session.status):raise ValueError('새 보드 상태 식별/유효성 실패')
        report=confirm_stop(client,session,journal)
        atomic_json(folder/'stop_recheck_attempt.json',report)
        if not all(report.get(k) is True for k in ('stop_ack','stationary_observed','process_released')):
            raise ValueError('새 정지 확인 실패: '+str(report.get('reason','unknown'))+'; log='+str(folder))
        stopped(report)
        if digest(old/'state.json')!=state_hash or digest(active)!=pointer_hash:raise ValueError('이전 상태/포인터가 검사 도중 변경됨')
        report.update(previous_session=str(old),previous_state_sha256=state_hash,release_id=release['release_id'],new_prepare_only=True)
        atomic_json(folder/'stop_recheck.json',report)
        archived=active.with_name('main_event_active_before_recheck_'+str(time.time_ns())+'.json')
        active.rename(archived)  # preserve, never delete/rewrite the failed run or pointer
        journal.event('failed_pointer_archived',path=str(archived),report=report)
        code=0
        print(json.dumps({'stop_rechecked':True,'evidence':str(folder),'archived_pointer':str(archived),'current_boot':report['boot'],
            'old_failed_state_unchanged':True,'next':'python3 tools/main_event_menu.py ; 1 -> L/R -> I (do not press 2 during stationary checks)'},ensure_ascii=False))
        return 0
    except Exception as exc:
        if journal:journal.event('stop_recheck_failed',reason=str(exc))
        raise
    finally:
        if transport:transport.close()
        if journal:journal.close(status='completed' if code==0 else 'failed',exit_code=code,report=report)
        os.close(lock)
