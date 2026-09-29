#!/usr/bin/env python3
"""Fast terminal prepare/start/status/stop/recover entry. No AI in the run loop.

prepare --check is offline and reports complete-course blockers. prepare --live
starts one sensor/follower process after physical-input validation. This release
has explicit S/parking/finish barriers; no flag converts them into completion.
"""
import argparse
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
from zoneinfo import ZoneInfo

from main_event_control import send
from main_event_plan import ROOT, build_section, append_finish, blockers, NEXT_RECOVERY, digest, save_bundle, restore_bundle
from main_event_signal import atomic_json

ACTIVE=ROOT/'data/main_event_active.json'
CONFIG=ROOT/'config/main_event.json'


def load(a):
    config=json.loads(a.config.read_text())
    reference=config.get('candidate_reference_mode') is True
    if reference and not config.get('candidate_basis'):raise ValueError('candidate reference basis missing')
    if config.get('parking_bypass_approved'):
        raise ValueError('parking bypass has no implemented road-contained route')
    for enabled,names in ((config.get('s_avoidance_verified'),('s_curve',)),
                          (config.get('reverse_enabled'),('t_parking','p_parking'))):
        if enabled and not reference:
            for name in names:
                boundary=config.get('road_boundaries',{}).get(name,{})
                if (boundary.get('verified') is not True or not boundary.get('evidence') or
                        boundary.get('frame')!='map' or len(boundary.get('polygon',[]))<3):
                    raise ValueError(name+' needs a verified road boundary and evidence')
    if config.get('finish_rear_wheel_line_confirmed'):
        line=config.get('finish_line',{})
        if line.get('verified') is not True or not line.get('evidence'):
            raise ValueError('official rear-wheel finish line evidence missing')
    for key in ('drive_pwm','bootstrap_pwm'):
        if type(config.get(key)) is not int or not 0<config[key]<=110: raise ValueError('invalid '+key)
    reverse_ld=config.get('parking_reverse_lookahead_m',3.)
    if type(reverse_ld) not in (int,float) or not math.isfinite(reverse_ld) or not 0<reverse_ld<=6.:
        raise ValueError('invalid bounded P reverse lookahead')
    if config['bootstrap_pwm']>70:raise ValueError('bootstrap is bounded to existing PWM70 candidate')
    for key in ('light_1','light_2','light_3'):
        if not isinstance(config['signals'].get(key),list) or any(s not in ('green','left_arrow') for s in config['signals'][key]):
            raise ValueError('invalid permitted signal list')
    policy=config.get('speed_policy',{})
    if policy and (policy.get('cruise_pwm')!=100 or policy.get('cruise_observation_m')!=2.4 or
                   policy.get('approach_low_m')!=6. or not policy.get('basis') or config['drive_pwm']!=70):
        raise ValueError('unsupported unreviewed section speed policy')
    profile=json.loads((ROOT/config['lidar_profile']).read_text())
    bundle=build_section(a.section,a.lane,config)
    return config,profile,bundle


def check(a):
    config,profile,bundle=load(a)
    import qualifier_follow as q
    pre=q.lst.preflight(str(ROOT/q.DEFAULT_CAL),25.,port=None)
    neg=q.lst.preflight(str(ROOT/q.DEFAULT_CAL),-19.,port=None)
    report={'mode':'offline_check','hardware_access':False,'profile':config['profile_id'],
            'field_verified':False,'section':a.section,'lane':a.lane,
            'route_points':len(bundle['points']),'route_length_m':bundle['s'][-1],
            'gates':[vars(g) for g in bundle['gates']], 'route_hashes':bundle['hashes'],
            'complete_course_blockers':blockers(config,profile),
            'candidate_reference_mode':config.get('candidate_reference_mode',False),
            'candidate_basis':config.get('candidate_basis'),
            'finish_basis':config.get('finish_basis'),
            'mount_calibration_status':profile.get('calibration_status'),
            'steering_preflight':pre['missing']+neg['missing'],
            'live_commands_implemented':True,'race_ready':False}
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if pre['live_capable'] and neg['live_capable'] else 2


def active():
    return Path(json.loads(ACTIVE.read_text())['session'])


def prepare(a, recovery=None):
    if recovery is None and a.section != 'start':
        raise ValueError('non-start section requires a preserved judge-permitted recovery, not a new run')
    config,profile,bundle=load(a)
    if recovery and recovery.get('recovery_bundle'):bundle=restore_bundle(recovery['recovery_bundle'])
    from main_event_lidar import validate_profile,body_geometry_usable
    validate_profile(profile,profile['mount_id'])
    if not body_geometry_usable(profile):raise ValueError('body geometry input/basis missing')
    for key in ('body_width_m','clearance_m','stop_distance_m','front_overhang_from_rear_axle_m'):
        if type(profile.get(key)) not in (int,float) or not math.isfinite(profile[key]) or profile[key]<=0:
            raise ValueError('missing measured geometry/braking input '+key)
    from main_event_release import verify
    release=verify()
    if digest(a.config)!=release['sha256']['config/main_event.json']:
        raise ValueError('selected configuration differs from the frozen release')
    # Exclusion lock is inherited by exactly this worker; no stale PID-based takeover.
    lock=os.open(ROOT/'data/main_event.lock',os.O_RDWR|os.O_CREAT,0o600)
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if recovery is None and ACTIVE.exists():
            raise ValueError('기존 포인터 보존 필요: 정상 정지면 메뉴 N, 실패 종료면 차량 고정 후 recheck-stop --vehicle-secured')
        stamp=datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')
        session=ROOT/'data'/('me_'+stamp+'_'+uuid.uuid4().hex[:6])
        session.mkdir(mode=0o700)
        info={'session':str(session),'run_id':uuid.uuid4().hex,'epoch':0,'lane':a.lane,
              'section':a.section,'created_kst':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
              'completed':[],'skipped':[],'field_verified':False,'bundle_hashes':bundle['hashes']}
        if recovery:info.update(recovery)
        atomic_json(session/'session.json',info)
        atomic_json(session/'config.json',config);atomic_json(session/'lidar_profile.json',profile)
        info['config_sha256']=digest(session/'config.json')
        info['lidar_profile_sha256']=digest(session/'lidar_profile.json')
        info['release_id']=release['release_id']
        atomic_json(session/'session.json',info)
        atomic_json(session/'state.json',{'phase':'preparing','ready':False,'updated_at':time.monotonic(),
                    'stop_ack':False,'process_released':False,'reason':'workers not started yet'})
        out=(session/'runtime.stdout.log').open('xb');err=(session/'runtime.stderr.log').open('xb')
        proc=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'_serve','--session',str(session),
                               '--lock-fd',str(lock)],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=out,stderr=err,
                              start_new_session=True,pass_fds=(lock,))
        out.close();err.close()
        atomic_json(ACTIVE,{'session':str(session),'pid':proc.pid,'run_id':info['run_id'],'epoch':info['epoch']})
    finally:os.close(lock)
    print(json.dumps({'status':'PREPARING','session':str(session),'pid':proc.pid,
                      'next':'python3 tools/main_event.py status; when ready: python3 tools/main_event.py start'},ensure_ascii=False))
    return 0


def finish_recovery(a,session):
    """Warm sensors first; only then take/commit a fresh measured body yaw."""
    deadline=time.monotonic()+45.;last=None
    while time.monotonic()<deadline:
        state=json.loads((session/'state.json').read_text())
        if state.get('phase') in ('stopped','prepare_failed'):raise ValueError(state.get('reason','prepare failed'))
        fresh=0<=time.monotonic()-state.get('updated_at',0)<1.
        if fresh and state.get('sensors_ready') and state.get('phase')=='waiting':break
        if state.get('reason')!=last:
            print('복구 센서 준비 대기:',state.get('reason','starting'),flush=True);last=state.get('reason')
        time.sleep(.1)
    else:raise ValueError('sensors not ready within45s; inspect status; no drive was started')
    print(json.dumps({'GPS_재합류_후보':state.get('recovery_hint')},ensure_ascii=False),flush=True)
    heading=a.body_heading_deg
    if heading is None:
        if not sys.stdin.isatty():raise ValueError('run localize with a fresh measured --body-heading-deg after sensors ready')
        heading=float(input('지금 측정한 차체 ENU 방향 도(동0/북90; 경로 방향을 추정해 입력하지 마세요): ').strip())
        observed=time.time()
    else:
        observed=a.heading_observed_wall  # keep original command receipt time, not a refreshed old value
    reply=send(session,'localize',{'body_heading_deg':heading,'index':a.index,'observed_wall':observed})
    if not reply.get('accepted'):raise ValueError('recovery pose command rejected')
    until=time.monotonic()+2.
    while time.monotonic()<until:
        state=json.loads((session/'state.json').read_text())
        bound=state.get('localization_nonce')==reply.get('nonce') and reply.get('nonce') is not None
        if bound and state.get('localization_error'):raise ValueError(state['localization_error'])
        if bound and state.get('recovery_pose_ready'):
            print(json.dumps({'recovery_pose_ready':True,'candidate':state.get('recovery_hint'),
                              'next':'python3 tools/main_event.py start'},ensure_ascii=False));return 0
        time.sleep(.05)
    raise ValueError('pose commit not confirmed; inspect status before start')


def localize(a):
    a.heading_observed_wall=time.time()
    return finish_recovery(a,active())


def recover(a):
    if not getattr(a,'after_manual',False):
        raise ValueError('경기 복구는 실제 S 충돌·심판 허용을 기록한 after-manual만 지원; 임의 SKIP/KEEP 입력 금지')
    from main_event_rejoin import candidate_window,recovery_ceiling
    old=active();saved=json.loads((old/'state.json').read_text());info=json.loads((old/'session.json').read_text())
    manual=None
    if getattr(a,'after_manual',False):
        from main_event_manual import load_handoff
        manual=load_handoff(old,info)
        a.skip_mission=manual['mission'];a.judge_permitted=True
    if saved.get('stop_ack') is not True or saved.get('process_released') is not True or saved.get('stationary_observed') is not True:
        raise ValueError('stop_unconfirmed or previous device ownership not released; do not move/rearm')
    if saved.get('fault') in ('stop_unconfirmed','stop_motion_unconfirmed','board_reboot'):
        raise ValueError('board fault needs independent reconciliation; no recovery bypass')
    interactive=sys.stdin.isatty()
    a.heading_observed_wall=time.time()
    if not a.judge_permitted:
        if not interactive or input('심판이 이번 복구와 자율재개 입력을 허용했나요? YES/NO: ').strip()!='YES':
            raise ValueError('only judge-permitted recovery is supported')
    skipped=list(saved.get('skipped',[]))
    current=saved['gates'][saved['index']]['name'] if saved['index']<len(saved['gates']) else None
    if saved.get('section','').startswith('parking_t_'):current='t_parking'
    if saved.get('section','').startswith('parking_p_'):current='p_parking'
    if interactive and a.skip_mission is None and current in NEXT_RECOVERY:
        answer=input(f'{current}: 심판 허용 미수행 이동 뒤면 SKIP, 현재 미션 재개면 KEEP: ').strip()
        if answer not in ('SKIP','KEEP'):raise ValueError('choose explicit SKIP or KEEP')
        if answer=='SKIP':a.skip_mission=current
    a.lane=info['lane'];a.section=info['section'] if info['section'] in ('start','after_s','after_t','after_p') else 'start'
    config,_,baseline=load(a)
    if a.skip_mission:
        if a.skip_mission!=current or current not in NEXT_RECOVERY:raise ValueError('only current permitted S/parking mission may be skipped')
        a.section=NEXT_RECOVERY[current];skipped.append(current)
        bundle=build_section(a.section,a.lane,config,parking_choices=saved.get('session_choices'));direction=1;window=[0,len(bundle['points'])-1];progress=0
    elif saved.get('route_sha256'):
        if digest(old/'final_route.json')!=saved['route_sha256']:raise ValueError('saved final route changed')
        bundle=restore_bundle(json.loads((old/'final_route.json').read_text()))
        if bundle.get('config')!=config:raise ValueError('saved route configuration differs; no cross-configuration automatic rejoin')
        direction=saved.get('direction',1);window=saved.get('active_window',[0,len(bundle['points'])-1]);progress=saved['progress']
    else:
        if info.get('release_id'):raise ValueError('legacy session has no preserved route bundle; explicit permitted section skip required')
        bundle=baseline;direction=1;window=[0,len(bundle['points'])-1];progress=saved['progress']
        if saved.get('finish_lane'):bundle=append_finish(bundle,saved['finish_lane'])
    pending=[g for g in bundle['gates'] if g.name not in saved.get('completed',[])+skipped]
    if progress is None:
        # Cancelled preparation has no selected entry segment. Preserve the
        # previously bounded candidates, never invent progress=0 or a board boot.
        allowed=saved.get('entry_candidates')
        if (saved.get('entry_pending') is not True or not isinstance(allowed,list) or not allowed or
                any(type(i) is not int or not window[0]<=i<window[1] for i in allowed)):
            raise ValueError('cancelled preparation lacks preserved entry candidates; explicit reconciliation required')
        if type(saved.get('boot')) is not int:
            raise ValueError('cancelled preparation lacks same-boot observed stop')
    else:
        ceiling=recovery_ceiling(bundle,progress,pending)
        allowed=candidate_window(bundle,progress,window[1],ceiling,bool(a.skip_mission))
    allowed=[i for i in allowed if i>=window[0]]
    if not allowed:raise ValueError('no rejoin within preserved directional window')
    if a.index is not None and a.index not in allowed:raise ValueError('rejoin index crosses current mission window')
    bundle['allowed']=allowed
    recovery={'run_id':info['run_id'],'epoch':info['epoch']+1,'section':bundle['section'],
              'completed':saved['completed'],'skipped':skipped,'finish_lane':saved.get('finish_lane'),
              'recovery_bundle':save_bundle(bundle),'session_choices':saved.get('session_choices',{}),
              'recovery':{'judge_permitted':True,'previous_session':str(old),'boot':saved['boot'],
                          'pose_pending':True,'allowed_indices':allowed,'direction':direction,
                          'window_start':window[0],'window_end':window[1]},
              'previous_state_sha256':digest(old/'state.json')}
    if manual:
        recovery['recovery'].update(boot=manual['boot'],heading_mode='forward_bootstrap',
                                    boot_binding_pending=True,manual_handoff=manual)
    result=prepare(a,recovery)
    if result:return result
    if manual:
        print('수동 통과 재개 준비: status의 recovery_bootstrap_ready:true 확인 뒤 start. 기존 전진 GNSS 방향 취득(최대1.2m)을 수행합니다.',flush=True)
        return 0
    if getattr(a,'prepare_only',False):
        print('센서 ready 뒤: python3 tools/main_event.py localize ; 그 다음 start',flush=True);return 0
    return finish_recovery(a,active())


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    prep=sub.add_parser('prepare');mode=prep.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check',action='store_true');mode.add_argument('--live',action='store_true')
    for ap in (prep,):
        ap.add_argument('--config',type=Path,default=CONFIG)
        ap.add_argument('--lane',choices=('left','right'),required=True)
        ap.add_argument('--section',choices=('start','after_s','after_t','after_p'),default='start')
    for cmd in ('start','stop','status'):sub.add_parser(cmd)
    recheck=sub.add_parser('recheck-stop');recheck.add_argument('--vehicle-secured',action='store_true')
    manual=sub.add_parser('manual');manual.add_argument('--mission',choices=tuple(NEXT_RECOVERY))
    manual.add_argument('--collision-recovery-confirmed',action='store_true',help='operator confirms actual S collision and judge-authorized exception; never ordinary passage')
    loc=sub.add_parser('localize');loc.add_argument('--body-heading-deg',type=float);loc.add_argument('--index',type=int)
    rec=sub.add_parser('recover');rec.add_argument('--config',type=Path,default=CONFIG)
    rec.add_argument('--after-manual',action='store_true',help='verified judge-authorized S collision handoff only; fresh forward GNSS bootstrap')
    rec.add_argument('--judge-permitted',action='store_true')
    rec.add_argument('--prepare-only',action='store_true',help='warm sensors; take fresh heading later with localize')
    rec.add_argument('--body-heading-deg',type=float,help='fresh measured BODY yaw in ENU degrees: east0 north90; not reverse GPS course; omitted = terminal prompt')
    rec.add_argument('--index',type=int,help='LOCAL segment in the preserved current section; omitted = terminal prompt')
    rec.add_argument('--skip-mission',choices=tuple(NEXT_RECOVERY))
    server=sub.add_parser('_serve');server.add_argument('--session',type=Path,required=True);server.add_argument('--lock-fd',type=int,required=True)
    a=p.parse_args(argv)
    try:
        if a.command=='prepare':return check(a) if a.check else prepare(a)
        if a.command=='recheck-stop':
            from main_event_stop_recheck import recheck
            return recheck(ACTIVE,a.vehicle_secured)
        if a.command=='_serve':
            # FD must be inherited from prepare; do not invent another singleton lock.
            st=os.fstat(a.lock_fd);expected=os.stat(ROOT/'data/main_event.lock')
            if (st.st_dev,st.st_ino)!=(expected.st_dev,expected.st_ino):raise ValueError('invalid inherited ownership lock')
            fcntl.flock(a.lock_fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            from main_event_runtime import run
            try:return run(a.session)
            except Exception as exc:
                atomic_json(a.session/'state.json',{'phase':'prepare_failed','ready':False,'updated_at':time.monotonic(),
                           'stop_ack':False,'process_released':False,'reason':type(exc).__name__+': '+str(exc)})
                raise
        if a.command=='recover':return recover(a)
        if a.command=='localize':return localize(a)
        session=active()
        if a.command=='manual':
            from main_event_manual import run_manual,current_mission
            saved=json.loads((session/'state.json').read_text())
            return run_manual(session,a.mission or current_mission(saved),a.collision_recovery_confirmed)
        if a.command=='status':
            state=json.loads((session/'state.json').read_text())
            state['session']=str(session);state['status_age_s']=time.monotonic()-state['updated_at']
            state['heartbeat_fresh']=0<=state['status_age_s']<1.
            if not state['heartbeat_fresh']:state['ready']=False
            print(json.dumps(state,ensure_ascii=False,indent=2));return 0
        if a.command=='stop':
            saved=json.loads((session/'state.json').read_text())
            if saved.get('phase')=='stopped' and saved.get('process_released') is True:
                ok=all(saved.get(k) is True for k in ('stop_ack','stationary_observed','process_released'))
                print(json.dumps({'already_released':True,'stop_confirmed':ok,'reason':saved.get('reason'),
                    'next':None if ok else 'vehicle secured then recheck-stop --vehicle-secured; old failure remains'},ensure_ascii=False))
                return 0 if ok else 3
        result=send(session,a.command)
        print(json.dumps(result,ensure_ascii=False))
        if a.command=='stop' and result['accepted']:
            deadline=time.monotonic()+12.
            while time.monotonic()<deadline:
                state=json.loads((session/'state.json').read_text())
                if state.get('phase')=='stopped':
                    report={k:state.get(k) for k in ('stop_ack','stationary_observed','process_released','reason','exit_code')}
                    print(json.dumps(report,ensure_ascii=False))
                    return 0 if all(report.get(k) is True for k in ('stop_ack','stationary_observed','process_released')) else 3
                time.sleep(.1)
            print('STOP_UNCONFIRMED: timeout waiting for same-boot stop ACK/stationary observation',file=sys.stderr)
            return 3
        return 0 if result['accepted'] else 2
    except (ValueError,OSError,KeyError) as exc:
        print('BLOCKED: '+str(exc),file=sys.stderr);return 2


if __name__=='__main__':raise SystemExit(main())
