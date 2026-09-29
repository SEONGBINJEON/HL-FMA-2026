"""Single Mega owner reusing qualifier geometry, steering library and DriveGate.

Live mode is a candidate. Missing physical mount/profile values are refused before
opening devices. Sensor processes never open the Mega. Faults always end in the
existing same-boot DISARM acknowledgement protocol; no network-triggered rearm.
"""
import dataclasses
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone

import qualifier_follow as q
from main_event_core import Mission, Stationary, ForkVotes, finite
from main_event_control import Commands
from main_event_lidar import ScanAssembler, obstacle_state, sonar_state, validate_profile, path_corridor,body_geometry_usable
from main_event_plan import build_section, append_finish, digest, save_bundle, restore_bundle
from main_event_motion import DirectionalDriveGate, BodyHeading, travel_pose, physical_left_wheel
from main_event_sequence import Sequence
from main_event_rejoin import propose as rejoin_proposal, commit as rejoin_commit, valid_anchor, bootstrap_ready
from main_event_signal import atomic_json
from main_event_speed import SpeedPolicy
from main_event_lidar_scope import lidar_stop_scope
from main_event_autonomy import ProgressWatch
from main_event_finish import FinishChoice
from main_event_degraded import GapContinuity, normalize_choices, core_ready, parking_execution
from main_event_startup import enable_startup_sync, startup_status
from path_follow_inputs import LatestRows, Heading, utc_seconds

ROOT = Path(__file__).resolve().parents[1]


def preparation_vision_ready(inputs,now):
    # Preparation is camera-stream + completed-worker liveness, not a green
    # authorization. Mission SignalVotes still enforces capture age <=0.6s.
    ev=inputs.signal;frame=getattr(inputs,'camera_frame',None)
    cached=getattr(inputs,'prepare_last_valid_inference',None)
    if (ev and ev.get('valid') is True and finite(ev.get('decision_at')) and
            finite(ev.get('capture_at')) and 0<=ev['decision_at']-ev['capture_at']<=.6 and
            (cached is None or ev['decision_at']>cached['decision_at'])):
        inputs.prepare_last_valid_inference=dict(ev);cached=inputs.prepare_last_valid_inference
    return bool(cached and frame and frame.get('input_kind')=='live' and
        frame.get('source')==cached.get('source') and finite(frame.get('capture_at')) and
        0<=now-frame['capture_at']<=.6 and 0<=now-cached['decision_at']<1.)



def stop_fault_signature(trial):
    # first_fault is sticky: message also catches a later Session RX/ordering fault.
    ad=trial.adapter
    return (ad.locked_by_fault,ad.first_fault,getattr(ad,'message',None))


def final_stop_fault(trial,sample,stop_boot,expected_lock):
    if sample and (stop_boot is None or sample['boot']!=stop_boot):return 'board_reboot'
    if getattr(trial.client,'failed',False):return 'board_transport_after_stop'
    if stop_fault_signature(trial)!=expected_lock:return 'board_fault_after_stop'
    return None


def board_sample(trial):
    s = trial.adapter.status
    if s is None: return None
    return {'boot':s.boot_id,'request':s.request_id,'encoder':s.encoder_count,
            'state':'ARMED' if s.state == 1 else 'DISARMED',
            'requested_drive':s.requested_drive, 'actual_drive':s.actual_drive,
            'received_at':trial.adapter.last_status_at,'fault':trial.adapter.locked_by_fault,
            'left_cm':s.left_cm,'center_cm':s.center_cm,'right_cm':s.right_cm,
            'reported_drive_pwm':getattr(s,'drive_pwm',None),
            'first_fault':trial.adapter.first_fault,'adapter_phase':trial.adapter.phase,
            'locked_by_fault':trial.adapter.locked_by_fault}


class Inputs:
    def __init__(self, session, profile):
        self.session, self.profile = Path(session), profile
        self.gga = LatestRows(self.session/'rtk/gga.jsonl')
        self.chunks = LatestRows(self.session/'lidar/host_chunks.jsonl', budget=131072)
        self.assembler = ScanAssembler(profile,profile['mount_id'])
        self.last_chunk_end = None
        self.scan = None
        self.fix = None
        self.last_gga_at = None
        self.signal = None
        self.camera_frame = None
        self.gga_intervals = []
        self.rtcm_fresh = False
        self.perception_health = {}

    def poll(self, lat0, lon0):
        now,wall = time.monotonic(),time.time()
        new = []
        # During DISARM-only waiting, sensor parsing must not starve serial RX.
        service=getattr(self,'service_board',None)
        if service:service()
        for row in self.gga.poll():
            stamp = utc_seconds(row['host_utc'])
            if self.last_gga_at is not None:
                if stamp < self.last_gga_at: raise ValueError('GNSS timestamp regression')
                if stamp == self.last_gga_at: continue
                self.gga_intervals.append(stamp-self.last_gga_at)
                self.gga_intervals = self.gga_intervals[-50:]
            self.last_gga_at = stamp
            wall=time.time()  # evaluate a newly read row, not a pre-I/O timestamp
            age = row.get('differential_age_s')
            good = (row.get('fix_status') == 'rtk_fixed' and row.get('checksum_valid') is True and
                    finite(age) and 0 <= age <= 10. and 0 <= wall-stamp <= .5)
            lat,lon = row.get('latitude_deg'),row.get('longitude_deg')
            if not good or not finite(lat) or not finite(lon) or not -90<=lat<=90 or not -180<=lon<=180:
                self.fix = None; continue
            x = (lon-lon0)*111320*math.cos(math.radians(lat0)); y = (lat-lat0)*111320
            self.fix = (x,y,stamp)
            new.append(self.fix)
        if self.fix and not 0 <= wall-self.fix[2] <= .5: self.fix = None
        try:
            self.poll_lidar(service)
            self.perception_health['lidar']='receiving' if self.scan is not None else 'no_scan'
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.scan=None
            self.assembler=ScanAssembler(self.profile,self.profile['mount_id'])
            self.perception_health['lidar']=type(exc).__name__+': '+str(exc)
        self.rtcm_fresh=False
        try:
            health=json.loads((self.session/'rtk/health.json').read_text())
            age=time.time()-utc_seconds(health['host_utc'])
            correction_age=health['connection']['valid_rtcm_age_s']
            self.rtcm_fresh=(0<=age<=6. and finite(correction_age) and 0<=correction_age+age<=10.)
        except FileNotFoundError:pass
        self.signal=self.optional_json('signal.json','signal')
        self.camera_frame=self.optional_json('frames/latest.json','camera')
        if self.signal and self.signal.get('input_kind') != 'live':
            self.perception_health['signal']='non_live_result_rejected';self.signal=None
        return new

    def optional_json(self,path,name):
        try:
            value=json.loads((self.session/path).read_text())
            if not isinstance(value,dict):raise ValueError('expected object')
            if name=='signal':
                for key in ('capture_at','decision_at','confidence'):
                    if not finite(value.get(key)):raise ValueError('invalid '+key)
                for key in ('source','target','sign'):
                    if not isinstance(value.get(key),str):raise ValueError('invalid '+key)
                if type(value.get('frame_id')) is not int or type(value.get('epoch')) is not int:
                    raise ValueError('invalid frame/epoch identity')
            self.perception_health[name]='readable'
            return value
        except (OSError,ValueError,TypeError) as exc:
            self.perception_health[name]=type(exc).__name__+': '+str(exc)
            return None

    def poll_lidar(self,service):
        for c in self.chunks.poll():
            if service:service()
            if c['stream'] != 'stdout': continue
            # Tail skipping cannot splice an old partial scan into a new one.
            if self.last_chunk_end is not None and c['offset'] != self.last_chunk_end:
                self.assembler = ScanAssembler(self.profile,self.profile['mount_id']); self.scan = None
            stamp = utc_seconds(c['host_utc'])
            now,wall=time.monotonic(),time.time()
            age = wall-stamp
            self.last_chunk_end = c['offset']+c['bytes']
            if not 0 <= age <= .3:
                self.assembler = ScanAssembler(self.profile,self.profile['mount_id']); self.scan = None; continue
            metadata = json.loads((self.session/'lidar/metadata.json').read_text())
            if metadata.get('mount_id') != self.profile['mount_id'] or metadata.get('transform_count') != 0:
                raise ValueError('live LiDAR mounting/frame mismatch')
            with (self.session/'lidar/stdout.bin').open('rb') as f:
                f.seek(c['offset']); raw=f.read(c['bytes'])
            if len(raw) != c['bytes']: raise ValueError('incomplete lidar chunk')
            if service:
                # Same bytes, same original receipt timestamp, same assembler.
                for begin in range(0,len(raw),1024):
                    service()
                    for scan in self.assembler.feed(raw[begin:begin+1024],now-age):self.scan=scan
            else:
                for scan in self.assembler.feed(raw,now-age): self.scan=scan


class Workers:
    def __init__(self, session): self.session=Path(session);self.processes=[];self.handles=[];self.health={}
    def start(self,name,argv):
        out=(self.session/(name+'.stdout.log')).open('xb')
        err=(self.session/(name+'.stderr.log')).open('xb')
        self.handles += [out,err]
        try:p=subprocess.Popen(argv,cwd=ROOT,stdout=out,stderr=err,start_new_session=True)
        except OSError as exc:
            if name=='rtk':raise
            self.health[name]={'state':'start_failed','error':type(exc).__name__}
            return None
        self.processes.append((name,p))
        atomic_json(self.session/'workers.json',{n:{'pid':child.pid,'exit_code':child.poll()} for n,child in self.processes})
        return p
    def check(self):
        for name,p in self.processes:
            code=p.poll()
            self.health[name]={'state':'running' if code is None else 'ended','exit_code':code}
            if code is not None and name=='rtk':raise RuntimeError(name+' worker ended: '+str(code))
        return dict(self.health)
    def close(self):
        for _,p in self.processes:
            if p.poll() is None:
                try: p.send_signal(signal.SIGINT)
                except ProcessLookupError: pass
        end=time.monotonic()+6.
        while any(p.poll() is None for _,p in self.processes) and time.monotonic()<end:
            time.sleep(.05)
        result={}
        for name,p in self.processes:
            if p.poll() is None:
                p.terminate()
                try:p.wait(timeout=1.)
                except subprocess.TimeoutExpired:p.kill();p.wait(timeout=1.)
            result[name]=p.returncode
        for h in self.handles:h.close()
        return result


def run(session):
    session=Path(session)
    info=json.loads((session/'session.json').read_text())
    for name,key in (('config.json','config_sha256'),('lidar_profile.json','lidar_profile_sha256')):
        if info.get(key)!=digest(session/name):raise ValueError('prepared session configuration changed: '+name)
    config=json.loads((session/'config.json').read_text())
    profile=json.loads((session/'lidar_profile.json').read_text())
    validate_profile(profile,profile['mount_id'])
    if not body_geometry_usable(profile):raise ValueError('body geometry input/basis missing')
    # Actual consumer requires all geometry/stop-distance values, not only matrix entries.
    for key in ('body_width_m','clearance_m','stop_distance_m','front_overhang_from_rear_axle_m'):
        if not finite(profile.get(key)) or profile[key]<=0: raise ValueError('missing physical '+key)
    bundle=restore_bundle(info['recovery_bundle']) if info.get('recovery_bundle') else build_section(info['section'],info['lane'],config,parking_choices=info.get('session_choices'))
    if info.get('finish_lane') is not None and not info.get('recovery_bundle'):bundle=append_finish(bundle,info['finish_lane'])
    if info.get('resume_index') is not None:
        index=info['resume_index']
        if type(index) is not int or not 0<=index<len(bundle['points'])-1:raise ValueError('recovery segment outside preserved branch')
        bundle['allowed']=[index]
    gates=bundle['gates']
    mission=Mission(gates,info['run_id'],info['epoch'],info.get('completed',[]),info.get('skipped',[]))
    # Existing qualifier/calibration defaults, including asymmetric steering limits.
    args=q.build_parser().parse_args(['--course','track','--track','common_1','--course-end','287',
                                      '--rtk-session',str(session/'rtk'),'--output-dir',str(session/'vehicle')])
    args.drive_pwm=config['drive_pwm'];args.bootstrap_distance_m=1.2;args.bootstrap_max_s=2.5;args.bootstrap_min_m=.8
    meta=json.loads((ROOT/q.DEFAULT_ROUTE).with_suffix('.meta.json').read_text())
    lat0,lon0=meta['origin_lat'],meta['origin_lon']
    selection=q.qr.SelectionParams(args.start_distance_m,args.arm_heading_deg,bundle['s'][-1],1.5,args.pose_max_age_s,'current mission section only')
    state=q.EntryState(selection)
    state.allowed=bundle['allowed']
    route=q.qr.ModeRoute('manual',[q.qr.Segment(info['section'],0,len(bundle['points'])-1,'HYPER main')],bundle['points'])
    state.select_mode(route,True)
    cfg=q.ps.PurePursuitConfig(args.wheelbase_m,'existing qualifier dimensions',args.lookahead_m,
          args.max_lateral_error_m,math.radians(args.run_heading_deg),math.radians(85),.5,.3,.5,'rear_axle_center','existing field controller')
    follower=q.ps.PathFollower(state,cfg,lambda t:int(t*1000)&0xffffffff)
    pre=q.lst.preflight(args.calibration,25.,port=args.port)
    neg=q.lst.preflight(args.calibration,-19.,port=None)
    if not pre['live_capable'] or not neg['live_capable']:
        raise ValueError('steering/device preflight blocked: '+str(pre['missing']+neg['missing']))
    journal=q.lst.TrialJournal(session/'vehicle','live_vehicle',q.lst.build_metadata('main_event',args.calibration,pre,2500,
                   {'bundle_hashes':bundle['hashes'],'session':info,'config':config},sys.argv))
    commands=Commands(session,info['run_id'],info['epoch'])
    workers=Workers(session)
    transport=trial=drive=None
    inputs=Inputs(session,profile)
    heading=BodyHeading(args.heading_window_m,antenna_forward_m=args.antenna_offset_m)
    recovery_bootstrap=info.get('recovery',{}).get('heading_mode')=='forward_bootstrap'
    sequence=Sequence(bundle,state,follower,args,profile,config,journal)
    speed=SpeedPolicy(config);far_key=None;far_cached='unknown';speed_key=None
    progress_watch=ProgressWatch();progress_report=None;gap_continuity=GapContinuity()
    session_choices=normalize_choices(info['session_choices']) if info.get('session_choices') else {};choices_nonce=None;choices_error=None;last_health=None
    if bundle.get('preentry_policy'):journal.event('predeparture_parking_policy',**bundle['preentry_policy'])
    if info.get('recovery'):
        context=info['recovery']
        sequence.window.start=context.get('window_start',0)
        sequence.window.end=context.get('window_end',len(bundle['points'])-1)
        sequence.window.direction=context.get('direction',1)
    encoder=q.EncoderWatch(args.encoder_stall_s)
    phase='waiting'; reason='not_started';code=1
    stopping=[False]
    old_handlers={sig:signal.signal(sig,lambda *_:stopping.__setitem__(0,True)) for sig in (signal.SIGINT,signal.SIGTERM)}
    last_signal_detail=None;last_lidar_scope=None
    last_issue=None;last_target=0.;last_snapshot=0.;start_at=None;straight_at=None;last_decision=None
    last_position=None
    boot_fixes=[];boot_at=None;hold_anchor=None;hold_encoder=None;hold_pos=None;hold_changed=False
    observation_key=None;cached_observation='unknown'
    localization_error=None;localization_nonce=None;recovery_hint=None;recovery_stationary=Stationary()
    request_target=None;alive_started=time.monotonic();ready=False;finish_choice=FinishChoice(info['epoch'],config['finish_fallback_lane']);finish_lane=info.get('finish_lane')
    finish_choice.operator_lane=session_choices.get('finish')
    try:
        workers.start('rtk',[str(ROOT/'.venv-rtk/bin/python'),'tools/rtk_record.py','--config','config/ntrip.local.json','--until-interrupt','--rate-hz','10','--output-dir',str(session/'rtk')])
        workers.start('camera',[str(ROOT/'.venv-oak/bin/python'),'tools/main_event_camera.py','--output',str(session/'frames')])
        atomic_json(session/'signal_request.json',{'target':'light_1','epoch':info['epoch'],'requested_at':time.monotonic()})
        workers.start('signal',[str(ROOT/'.venv-yolo/bin/python'),'tools/main_event_signal.py','--frames',str(session/'frames'),
                      '--request',str(session/'signal_request.json'),'--output',str(session/'signal.json')])
        workers.start('lidar',[sys.executable,'tools/lidar_record.py','--until-interrupt','--mount-id',profile['mount_id'],
                      '--output-dir',str(session/'lidar'),'--context','stationary prepare / main event candidate; unchanged mount required'])
        transport=q.lst.PosixSerialTransport(args.port,check_identity=True)
        trial=q.lst.assemble(q.h.load_library(),args.calibration,1900,transport,journal,allow_synthetic=False)
        enable_startup_sync(trial.client)
        drive=DirectionalDriveGate(config['bootstrap_pwm'],trial.adapter,clock=time.monotonic);trial.client.encoder=drive
        while not stopping[0]:
            now,wall=time.monotonic(),time.time()
            loop_started=now
            worker_health=workers.check() or {}
            if not trial.tick(now):raise ValueError(trial.exit_reason or 'board transport ended')
            if phase=='waiting' and trial.adapter.locked_by_fault:
                raise ValueError('startup_board_locked: '+str(trial.adapter.first_fault or trial.adapter.message))
            board_tick_done=time.monotonic()
            inputs.service_board=lambda:trial.client.tick(time.monotonic())
            samples=inputs.poll(lat0,lon0)
            sensor_poll_done=time.monotonic()
            now,wall=time.monotonic(),time.time()
            # Worker/file faults invalidate their observations, never become green/clear.
            for name,health in worker_health.items():
                if health['state']!='running':
                    if name=='signal':inputs.signal=None
                    elif name=='camera':inputs.camera_frame=None;inputs.signal=None
                    elif name=='lidar':inputs.scan=None
            perception_health={'workers':worker_health,'inputs':getattr(inputs,'perception_health',{})}
            if perception_health!=last_health:
                journal.event('perception_health',**perception_health)
                last_health=json.loads(json.dumps(perception_health))
            b=board_sample(trial)
            speed.observe(inputs.fix,now)
            position_ok=inputs.fix is not None
            if position_ok:last_position=inputs.fix[:2]
            quality_ok=position_ok and inputs.rtcm_fresh
            if start_at is not None and now-start_at>900:raise ValueError('run exceeded bounded 900s')
            corridor=None
            if phase=='running' and inputs.fix is not None and heading.yaw is not None and state.progress is not None:
                rear=q.make_pose(args,*inputs.fix[:2],heading.yaw,wall,inputs.fix[2])
                corridor=path_corridor(bundle['points'][:sequence.window.end+1],state.progress,rear.x,rear.y,heading.yaw,
                                       profile['stop_distance_m'])
                # At an exact neutral segment endpoint there is no remaining sweep.
                # Mission may change direction while stopped; it may not drive an empty path.
            if inputs.scan is None or (phase=='running' and corridor==[]) or not 0<=now-inputs.scan['received_at']<=.3:
                obs='unknown'
            else:
                key=(id(inputs.scan),None if corridor is None else tuple(corridor))
                if key!=observation_key:
                    cached_observation=obstacle_state(inputs.scan,profile,now,corridor,sequence.window.direction)
                    observation_key=key
                obs=cached_observation
            raw_obs=obs
            far_obs='unknown'
            if phase=='running' and inputs.fix is not None and heading.yaw is not None and state.progress is not None:
                approximate_s=bundle['s'][state.progress]
                if speed.high_guard or speed.zone(phase,mission,bundle,approximate_s,sequence.window.direction)=='general_after_hill':
                    rear=q.make_pose(args,*inputs.fix[:2],heading.yaw,wall,inputs.fix[2])
                    far_corridor=path_corridor(bundle['points'][:sequence.window.end+1],state.progress,rear.x,rear.y,heading.yaw,speed.horizon)
                    covered=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in far_corridor)
                    if inputs.scan is not None and covered>=speed.horizon-1e-6:
                        key=(id(inputs.scan),tuple(far_corridor),sequence.window.direction)
                        if key!=far_key:
                            far_cached=obstacle_state(inputs.scan,profile,now,far_corridor,sequence.window.direction);far_key=key
                        far_obs=far_cached
            if speed.high_guard:obs=far_obs  # never shrink to70's horizon while still decelerating
            channels=config.get('sonar_confirmed_forward_channels',[])
            if channels and finite(config.get('sonar_stop_cm')) and b:
                if sonar_state(b,channels,now,config['sonar_stop_cm'])=='blocked':obs='blocked'
            # Input parsing/geometry work cannot make an earlier board snapshot
            # fresh at the explicit ARM boundary.
            now,wall=time.monotonic(),time.time()
            position_ok=inputs.fix is not None and 0<=wall-inputs.fix[2]<=.5
            quality_ok=position_ok and inputs.rtcm_fresh
            if inputs.scan is None or not 0<=now-inputs.scan['received_at']<=.3:obs=far_obs='unknown'
            # Optional perception health is reported separately from essential READY.
            ev=inputs.signal
            vision_ready=preparation_vision_ready(inputs,now)
            rate_ready=len(inputs.gga_intervals)>=3 and max(inputs.gga_intervals[-3:])<=.5
            ready=core_ready(b,now,quality_ok,rate_ready)
            sensors_ready=ready
            if phase=='waiting' and info.get('recovery'):
                recovery_stationary.update(b,now)
                r=info['recovery']
                recovery_hint=rejoin_proposal(bundle,r.get('allowed_indices',bundle['allowed']),*inputs.fix[:2]) if position_ok else None
                if recovery_bootstrap:
                    pending_binding=r.get('boot_binding_pending')
                    ready=ready and bootstrap_ready(r,b,inputs.fix,recovery_hint,recovery_stationary,wall)
                    if pending_binding and not r.get('boot_binding_pending'):
                        journal.event('manual_recovery_boot_bound',previous_boot=r['previous_boot'],boot=r['boot'],
                                      heading_source='pending fresh forward GNSS bootstrap')
                else:ready=ready and valid_anchor(r,b,inputs.fix,wall)
            action=commands.poll(phase if phase!='waiting' or ready else 'not_ready')
            if isinstance(action,dict) and action.get('action')=='choices':
                choices_nonce=action.get('nonce');choices_error=None
                try:
                    if phase!='waiting':raise ValueError('choices require waiting before start')
                    proposed={**session_choices,**normalize_choices(action['payload'])}
                    parking_changed=any(proposed.get(k)!=session_choices.get(k) for k in ('t','p'))
                    if parking_changed:
                        if info.get('recovery'):raise ValueError('parking choice is preserved across S recovery; cannot change it here')
                        # Only DISARM waiting: rebuild the future route, never jump a moving index.
                        replacement=build_section(info['section'],info['lane'],config,parking_choices=proposed)
                        sequence.install(replacement,mission);sequence.window.end=len(replacement['points'])-1
                        state.allowed=replacement['allowed'];bundle=replacement
                        journal.event('prestart_parking_route_bound',choices=proposed,execution=parking_execution(proposed))
                    session_choices=proposed
                    # Keep finish choice too when later sections inherit this bundle.
                    bundle['session_choices']=dict(session_choices)
                    finish_choice.operator_lane=session_choices.get('finish')
                    atomic_json(session/'choices.json',{'choices':session_choices,'nonce':choices_nonce,'at':now,
                        'parking_execution':parking_execution(session_choices)})
                    journal.event('session_choices',choices=session_choices,parking_execution=parking_execution(session_choices))
                except (ValueError,KeyError,TypeError) as exc:choices_error=str(exc)
                action=None
            if isinstance(action,dict) and action.get('action')=='localize':
                localization_nonce=action.get('nonce')
                try:
                    if not info.get('recovery') or not sensors_ready or recovery_stationary.stopped_since is None:
                        raise ValueError('recovery sensors and observed stationary DISARMED board required')
                    r=info['recovery'];payload=action['payload']
                    if b['boot']!=r['boot']:raise ValueError('board reboot; independent reconciliation required')
                    result=rejoin_commit(bundle,r['allowed_indices'],inputs.fix,payload.get('body_heading_deg'),
                                         payload.get('observed_wall'),wall,args,r.get('direction',1),payload.get('index'))
                    r.update(result,pose_pending=False,encoder=b['encoder'])
                    r['heading_deg']=payload['body_heading_deg'];state.allowed=[result['index']]
                    info['resume_index']=result['index'];localization_error=None
                    journal.event('recovery_pose_committed',**result,boot=b['boot'],encoder=b['encoder'])
                except ValueError as exc:
                    localization_error=str(exc);journal.event('recovery_pose_rejected',reason=localization_error)
                action=None
            if action=='stop':reason='operator_stop';code=130;break
            if action=='start':
                if not ready:raise ValueError('start readiness changed')
                if info.get('recovery'):
                    r=info['recovery']
                    valid=bootstrap_ready(r,b,inputs.fix,recovery_hint,recovery_stationary,wall) if recovery_bootstrap else valid_anchor(r,b,inputs.fix,wall)
                    if not valid:raise ValueError('recovery pose expired or moved after measurement')
                    if b['boot']!=r['boot']:raise ValueError('board reboot after stop; reconciliation required')
                    if not recovery_bootstrap:
                        heading.yaw=math.radians(r['heading_deg']);heading.anchor_at=r['observed_wall']
                    heading.change_direction(r.get('direction',1))
                    sequence.window.direction=r.get('direction',1)
                    sequence.window.start=r.get('window_start',0);sequence.window.end=r.get('window_end',len(bundle['points'])-1)
                if not trial.adapter.request_prepare(now):raise ValueError('board refused explicit start prepare')
                start_at=now;phase='preparing';journal.event('explicit_start',boot=b['boot'])
            if phase=='waiting':
                drive.close()
            else:
                if b is None or trial.adapter.locked_by_fault:raise ValueError(trial.adapter.first_fault or 'board fault')
                if phase in ('preparing','straighten','bootstrap') and not quality_ok:
                    raise ValueError('position quality lost during start')
                for fix in samples:
                    heading.measured_curvature=None
                    if sequence.window.direction<0 and bundle['section'].startswith('parking_p_'):
                        measured=q.sc.adc_to_angle(trial.validated['points'],trial.adapter.status.steering_adc)
                        if measured is None:raise ValueError('reverse BODY correction needs calibrated steering ADC')
                        tangent=math.tan(math.radians(measured/100.))
                        heading.measured_curvature=tangent/(args.wheelbase_m+args.track_m*.5*tangent)
                    if phase=='bootstrap':boot_fixes.append(fix)
                    elif phase=='running' and (not mission.holding or mission.stationary.stopped_since is None):heading.gps(*fix)
                if phase=='preparing' and trial.prepared() and b['state']=='ARMED':
                    if info.get('recovery') and not recovery_bootstrap:phase='entry'
                    else:
                        if not trial.command_target(0,now,trial.max_hold_s):raise ValueError('straight target refused')
                        straight_at=now;phase='straighten'
                if phase=='straighten':
                    drive.close()
                    target=q.sc.angle_to_adc(trial.validated['points'],0)
                    if q.straight_reached(trial.adapter.status.steering_adc,target,trial.validated['restart_band_adc']):
                        boot_fixes=[inputs.fix];boot_at=now;phase='bootstrap';encoder.reset()
                    elif now-straight_at>args.straighten_max_s:raise ValueError('straightening timeout')
                if phase=='bootstrap':
                    moved=math.hypot(inputs.fix[0]-boot_fixes[0][0],inputs.fix[1]-boot_fixes[0][1])
                    if moved>=1.2 or now-boot_at>=2.5:
                        drive.close()
                        heading.yaw,dist,dev=q.bootstrap_heading(boot_fixes,.8,.3)
                        heading.anchor_at=wall;heading.gps(*inputs.fix)
                        journal.event('bootstrap',moved_m=dist,scatter_m=dev,seconds=now-boot_at)
                        phase='entry'
                    else:drive.open(now)
                if phase in ('straighten','bootstrap') and now-last_target>=1.:
                    if not trial.command_target(0,now,trial.max_hold_s):raise ValueError('bootstrap target rejected')
                    last_target=now
                if phase=='entry':
                    pose=q.make_pose(args,*inputs.fix[:2],heading.yaw,wall,inputs.fix[2])
                    if not state.arm(travel_pose(pose,sequence.window.direction).as_route_pose()):raise ValueError(state.last_reason)
                    if sequence.window.direction<0:
                        drive.close();drive.change_direction(-1,recovery_stationary,now)
                    if not state.start():raise ValueError(state.last_reason)
                    drive.pwm=config['drive_pwm'];phase='running';encoder.reset()
                if phase=='running':
                    # No pose -> preserve the last section/progress, close drive; never infer a fresh fix.
                    s=bundle['s'][state.progress]
                    aligned=False
                    if position_ok and heading.yaw is not None:
                        pose=q.make_pose(args,*inputs.fix[:2],heading.yaw,wall,inputs.fix[2])
                        s=q.along_track(bundle['points'],state.progress,pose)
                        travel=travel_pose(pose,sequence.window.direction)
                        stage_points=bundle['points'][:sequence.window.end+1]
                        align_index=min(state.progress,len(stage_points)-2)
                        _,dist,err=q.select_entry(stage_points,[align_index],travel.as_route_pose(),args.max_lateral_error_m,args.run_heading_deg)
                        aligned=True
                    held_valid=False
                    if mission.holding and mission.stationary.stopped_since is not None and hold_anchor is None and heading.valid(wall,args.heading_max_age_s):
                        hold_anchor=heading.yaw;hold_encoder=b['encoder'];hold_pos=last_position
                    if mission.holding and hold_anchor is not None:
                        if b['encoder']!=hold_encoder:hold_changed=True
                        if position_ok and hold_pos and math.hypot(inputs.fix[0]-hold_pos[0],inputs.fix[1]-hold_pos[1])>.25:hold_changed=True
                        held_valid=not hold_changed
                        if hold_changed:mission.recovery_required=True;mission.recovery_cause='held_vehicle_moved'
                    heading_ok=not hold_changed and (held_valid or heading.valid(wall,args.heading_max_age_s))
                    continuity=gap_continuity.update(inputs.fix if position_ok else None,wall,s,b,
                        heading.yaw,heading.valid(wall,args.heading_max_age_s),aligned,
                        mission.stationary.stopped_since is not None,held_valid,hold_changed)
                    maneuver_ok=False
                    if quality_ok and heading_ok and aligned:
                        maneuver_ok=sequence.prepare(mission,pose,inputs.scan,now,s,held_valid,obs)
                        bundle=sequence.bundle
                        s=q.along_track(bundle['points'],state.progress,pose)
                        if sequence.reason and mission.gate and mission.gate.kind=='s_curve':obs='unknown'
                    gate=mission.gate
                    if gate and (gate.kind=='signal' or gate.name=='finish_fork') and gate.stop_s-s<=12. and request_target!=gate.name:
                        request_target=gate.name
                        atomic_json(session/'signal_request.json',{'target':gate.name,'epoch':info['epoch'],'requested_at':now})
                        ev=None  # never consume the previous target's buffered frame
                    if gate and gate.name=='finish_fork' and gate.stop_s-s<=12.:
                        choice=finish_choice.update(ev,now,s,gate)
                        if choice is not None:
                            bundle=append_finish(bundle,choice['lane']);finish_lane=choice['lane']
                            sequence.install(bundle,mission);sequence.window.end=len(bundle['points'])-1
                            journal.event('finish_branch',**choice,hashes=bundle['hashes'])
                    was_holding=mission.holding
                    observed_stop_s=mission.stationary.duration(now)
                    # Recheck against the real clock after geometry/planning work;
                    # an expensive calculation must not freshen old board/scan data.
                    now,wall=time.monotonic(),time.time()
                    if inputs.scan is None or not 0<=now-inputs.scan['received_at']<=.3:obs=far_obs='unknown'
                    if inputs.fix is not None and not 0<=wall-inputs.fix[2]<=.5:
                        position_ok=False;quality_ok=False
                    # A70 command is not evidence that100 has decelerated. If
                    # approaching a mission while the long envelope is retained,
                    # stop early rather than reaching its short70 stop margin fast.
                    lidar_required,lidar_scope=lidar_stop_scope(bundle,s,phase)
                    speed_approach_hold=bool(lidar_required and speed.high_guard and mission.gate and mission.gate.stop_s-s<=speed.horizon)
                    if speed_approach_hold:obs='unknown'
                    completed_before=len(mission.completed)
                    result=mission.step(s=s,now=now,board=b,quality_ok=quality_ok,heading_ok=heading_ok,
                                        aligned=aligned,signal=ev,obstacle=obs,sensor_required=lidar_required,position_ok=position_ok,
                                        position_observed_at=now-(wall-inputs.fix[2]) if position_ok else None,
                                        resume_ready=held_valid if was_holding else True,maneuver_ok=maneuver_ok,
                                        position_continuity_proven=continuity,
                                        finish_confirmed=sequence.finish(pose) if quality_ok and gate and gate.kind=='finish' else None)
                    if continuity and quality_ok and not mission.recovery_required:
                        journal.event('gps_continuity_reacquired',**gap_continuity.evidence)
                        gap_continuity.reset()
                    if speed_approach_hold and not result['drive'] and not mission.fault:
                        result['reason']='speed_deceleration_unconfirmed_before_mission'
                        mission.hold_reason=result['reason']
                    if len(mission.completed)>completed_before and gate and gate.kind in ('hill','signal','s_curve'):
                        # Finished missions are no longer geometric rejoin candidates.
                        # This preserves the original follower inside the CURRENT mission
                        # and avoids older crossing lanes triggering false relocation.
                        sequence.window.start=max(sequence.window.start,max(0,state.progress-1))
                        if gate.kind=='s_curve':bundle['section']='after_s'
                        journal.event('mission_window_advanced',mission_name=gate.name,window_start=sequence.window.start)
                        if gate.kind=='signal':journal.event('signal_crossing',**mission.last_signal_crossing)
                    if result.get('finished'):
                        drive.close()
                        reason='hyper_reference_goal_reached_official_finish_unconfirmed' if config.get('finish_mode')=='hyper_reference_goal' else 'official_rear_wheels_finished'
                        code=0;break
                    if result.get('action') in ('gear','transition'):
                        sequence.transition(mission,pose,drive,heading,now)
                        bundle=sequence.bundle;last_issue=None;encoder.reset()
                    decision_key=(result['reason'],mission.index,tuple(mission.completed),mission.obstacle_latched)
                    if decision_key!=last_decision:
                        journal.event('mission_decision',**result,progress=state.progress,s_m=s,
                                      observed_stop_s=observed_stop_s,completed=mission.completed,skipped=mission.skipped)
                        last_decision=decision_key
                    speed.permit(result['drive'],inputs.fix)
                    if not result['drive']:
                        drive.close();drive.pwm=speed.low;speed.selected=speed.low;speed.reason='stopped_or_transition'
                        if not was_holding:
                            hold_anchor=None  # acquire only after actual encoder settling, not at command0
                            hold_encoder=b['encoder'];hold_pos=last_position;hold_changed=False
                        if now-last_target>=1.:
                            if not trial.command_target(0,now,trial.max_hold_s):raise ValueError('hold target refused')
                            last_target=now
                        if mission.fault:raise ValueError(mission.fault)
                    else:
                        if lidar_required:
                            drive.pwm=speed.selected=speed.low;speed.reason='mission_lidar_scope_low'
                        else:drive.pwm=speed.choose(phase,mission,bundle,s,sequence.window.direction,inputs.fix,far_obs,now)
                        if was_holding:
                            # Renew only a stationary validated anchor; no second bootstrap across a stop line.
                            if not held_valid:raise ValueError('held heading invalid after movement; explicit recovery required')
                            heading.yaw=hold_anchor;heading.anchor_at=wall;heading.trail.clear();heading.gps(*inputs.fix)
                            encoder.reset();last_issue=None
                        if inputs.fix[2]!=last_issue:
                            ph=q.qr.path_heading_at(bundle['points'],state.progress)
                            lat=-math.sin(ph)*(pose.x-bundle['points'][state.progress].x)+math.cos(ph)*(pose.y-bundle['points'][state.progress].y)
                            base_lookahead=config.get('parking_reverse_lookahead_m',3.) if sequence.window.direction<0 and bundle['section'].startswith('parking_p_') else args.lookahead_m
                            follower.cfg=dataclasses.replace(cfg,lookahead_m=q.adaptive_lookahead(base_lookahead,1.5,6.,lat))
                            msg=sequence.window.step(pose,now)
                            if msg.kind!='steer':
                                journal.event('follower_stop',reason=msg.reason,cause=getattr(msg,'cause',None),
                                              reached=getattr(msg,'reached',False),decision=msg.decision,
                                              progress=state.progress,window=[sequence.window.start,sequence.window.end])
                                if not getattr(msg,'reached',False):raise ValueError('follower: '+msg.reason)
                                reason='route_end_not_official_finish';code=2;break
                            if sequence.window.direction<0 or bundle['section'].startswith('parking_'):
                                wheel=physical_left_wheel(msg.angle_cdeg,sequence.window.direction,args.wheelbase_m,args.track_m)
                            else:
                                wheel=q.lpf.cap_left_wheel(q.lpf.left_wheel_cdeg(msg.angle_cdeg,args.wheelbase_m,args.track_m),25.,19.)
                            if not trial.command_target(wheel,now,trial.max_hold_s):raise ValueError('steering target refused')
                            drive.open(now);last_issue=inputs.fix[2];last_target=now
                            journal.event('follow',progress=state.progress,label=bundle['labels'][state.progress],decision=msg.decision,target_cdeg=wheel,
                                          measured_heading_curvature=heading.measured_curvature)
                stall=encoder.check(trial.adapter.status,drive.allowed_until>now,now)
                if stall:raise ValueError(stall)
            if phase=='running' and state.progress is not None:
                progress_report=progress_watch.update(now,bundle['s'][state.progress],inputs.fix if position_ok else None,b,mission.hold_reason)
                if progress_report['changed']:journal.event('nonprogress_watch',**progress_report)
            current_speed_key=(speed.selected,speed.reason,drive.last_wire_pwm,drive.last_wire_drive,speed.high_guard)
            if current_speed_key!=speed_key:
                journal.event('speed_policy',selected_pwm=speed.selected,reason=speed.reason,
                    wire_pwm=drive.last_wire_pwm,wire_drive=drive.last_wire_drive,
                    gnss_speed_m_s=speed.speed,long_observation=far_obs,long_guard=speed.high_guard)
                speed_key=current_speed_key
            if now-last_snapshot>=.2:
                snap={**mission.snapshot(),'phase':phase,'ready':ready,'section':bundle['section'],'lane':info['lane'],
                      'direction':sequence.window.direction,'active_window':[sequence.window.start,sequence.window.end],
                      'maneuver_reason':sequence.reason,'sensors_ready':sensors_ready,'recovery_hint':recovery_hint,
                      'vision_ready':vision_ready,'gnss_rate_ready':rate_ready,
                      'core_ready':sensors_ready,'perception_health':perception_health,
                      'gps_continuity':gap_continuity.evidence,'session_choices':session_choices,
                      'choices_nonce':choices_nonce,'choices_error':choices_error,'parking_execution':parking_execution(session_choices),
                      'vision_preparation_evidence':{'frame_at':(getattr(inputs,'camera_frame',None) or {}).get('capture_at'),
                          'last_valid_decision_at':(getattr(inputs,'prepare_last_valid_inference',None) or {}).get('decision_at'),
                          'latest_result_valid':(inputs.signal or {}).get('valid')},
                      'nonprogress':progress_report,'parking_policy':config.get('parking_completion_policy'),
                      'selected_pwm':speed.selected,'speed_reason':speed.reason,'wire_pwm':drive.last_wire_pwm,
                      'wire_drive':drive.last_wire_drive,'gnss_speed_m_s':speed.speed,'long_observation':far_obs,'long_guard':speed.high_guard,
                      'localization_error':localization_error,'localization_nonce':localization_nonce,'recovery_pose_ready':bool(info.get('recovery') and valid_anchor(info['recovery'],b,inputs.fix,wall)),
                      'recovery_bootstrap_ready':bool(recovery_bootstrap and ready and phase=='waiting'),
                      'progress':state.progress,'label':None if state.progress is None else bundle['labels'][state.progress],
                      'lidar_raw_observation':raw_obs,
                      'lidar_stop_applied':lidar_stop_scope(bundle,None if state.progress is None else bundle['s'][state.progress],phase)[0],
                      'lidar_stop_scope':lidar_stop_scope(bundle,None if state.progress is None else bundle['s'][state.progress],phase)[1],
                      'signal_wait_detail':getattr(mission,'signal_wait_detail',None),
                      'startup_sync':startup_status(trial),'board_first_fault':trial.adapter.first_fault,
                      'board_adapter_phase':trial.adapter.phase,'board_locked':trial.adapter.locked_by_fault,
                      'pose_fresh':position_ok,'correction_fresh':inputs.rtcm_fresh,'obstacle':obs,'board':b,'finish_lane':finish_lane,'finish_choice':finish_choice.choice,
                      'gga_intervals_s':inputs.gga_intervals,'updated_at':now,
                      'stop_ack':False,'process_released':False}
                atomic_json(session/'state.json',snap);last_snapshot=now
            loop_done=time.monotonic()
            if loop_done-loop_started>=.025:
                journal.event('loop_processing_delay',board_tick_s=board_tick_done-loop_started,
                              input_poll_s=sensor_poll_done-board_tick_done,total_s=loop_done-loop_started)
            time.sleep(.005)
        if stopping[0]:reason='process_signal_stop';code=130
    except Exception as exc:
        reason=type(exc).__name__+': '+str(exc);code=1
        import traceback
        journal.event('runtime_exception',traceback=traceback.format_exc(),progress=state.progress,
                      body_heading_rad=heading.yaw,fix=inputs.fix,
                      heading_trail=list(heading.trail),heading_anchor_at=heading.anchor_at,
                      measured_heading_curvature=heading.measured_curvature,section=bundle['section'])
    finally:
        sequence.worker.close()
        if drive:drive.close()
        ack=False;stop_stationary=False;stop_fault=None;stop_boot=None
        if trial:
            try:
                prior_locked=trial.adapter.locked_by_fault
                prior_fault=trial.adapter.first_fault
                if prior_locked and code==0:
                    mission.fault='board_fault';reason=str(prior_fault);code=1
                trial.quit(time.monotonic())
                expected_stop_lock=stop_fault_signature(trial)
                deadline=time.monotonic()+trial.ack_wait_s+.3
                while time.monotonic()<deadline and trial.tick(time.monotonic()):time.sleep(.005)
                ack=trial.stop_ack_observed
                if ack:
                    stop_boot=(trial.current_stop or {}).get('boot')
                    observed=Stationary();until=time.monotonic()+3.
                    while time.monotonic()<until:
                        trial.client.tick(time.monotonic())
                        sample=board_sample(trial)
                        stop_fault=final_stop_fault(trial,sample,stop_boot,expected_stop_lock)
                        if stop_fault:break
                        if sample and sample['state']=='DISARMED':
                            stop_stationary=observed.update(sample,time.monotonic())
                        else:
                            observed.reset();stop_stationary=False
                        if stop_stationary:break
                        time.sleep(.005)
            except Exception as exc:
                stop_fault='stop_observation_exception'
                journal.event('stop_observation_exception',detail=str(exc))
        if stop_fault:
            stop_stationary=False;reason=stop_fault+'; '+reason;code=q.lst.EXIT_NO_ACK
        if not ack and trial and trial.adapter.identified:reason='stop_unconfirmed; '+reason;code=q.lst.EXIT_NO_ACK
        if ack and not stop_stationary:reason='stop_motion_unconfirmed; '+reason;code=q.lst.EXIT_NO_ACK
        if transport:transport.close()
        codes=workers.close();commands.close()
        for sig,old in old_handlers.items():signal.signal(sig,old)
        atomic_json(session/'final_route.json',save_bundle(bundle))
        final={**mission.snapshot(),'direction':sequence.window.direction,'active_window':[sequence.window.start,sequence.window.end],
               'boot':mission.boot if mission.boot is not None else (stop_boot if stop_stationary else None),
               'entry_pending':state.progress is None,
               'entry_candidates':list(info.get('recovery',{}).get('allowed_indices',state.allowed)),
               'route_sha256':digest(session/'final_route.json'),'phase':'stopped','section':bundle['section'],'lane':info['lane'],
               'progress':state.progress,'body_heading_rad':heading.yaw,'last_fix':inputs.fix,'finish_lane':finish_lane,'reason':reason,'exit_code':code,'stop_ack':ack,
               'process_released':True,'stationary_observed':stop_stationary,'stop_ack_boot':stop_boot,'worker_exit_codes':codes,'updated_at':time.monotonic(),
               'ended_utc':datetime.now(timezone.utc).isoformat(),'session_choices':session_choices,'parking_execution':parking_execution(session_choices),
               'startup_sync':startup_status(trial),'board':board_sample(trial) if trial else None,
               'board_first_fault':trial.adapter.first_fault if trial else None,
               'board_adapter_phase':trial.adapter.phase if trial else None,
               'board_locked':trial.adapter.locked_by_fault if trial else None,
               'fault':stop_fault or ('stop_unconfirmed' if not ack else ('stop_motion_unconfirmed' if not stop_stationary else mission.fault))}
        atomic_json(session/'state.json',final)
        journal.close(status='stopped' if ack else 'stop_unconfirmed',exit_code=code,reason=reason,final_stop_attempt=None if trial is None else trial.current_stop)
    print(json.dumps(final,ensure_ascii=False),flush=True)
    return code
