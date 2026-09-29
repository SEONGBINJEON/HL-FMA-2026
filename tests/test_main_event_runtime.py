"""Synthetic sensor/board transport only; runs the ACTUAL single-owner loop.
The vehicle plant is an ideal straight-line fixture, not a dynamics validation.
"""
import contextlib
import io
import json
import math
import csv
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import main_event_runtime as rt
import qualifier_route as qr
from main_event_core import Gate


class RuntimeIntegrationTests(unittest.TestCase):
    def test_changed_prepared_configuration_rejected_before_devices(self):
        with tempfile.TemporaryDirectory() as temp:
            d=Path(temp)
            (d/'session.json').write_text(json.dumps({'config_sha256':'not-the-current-file'}))
            (d/'config.json').write_text('{}')
            with patch.object(rt.q.lst,'PosixSerialTransport',side_effect=AssertionError('device must remain unopened')):
                with self.assertRaisesRegex(ValueError,'configuration changed'):
                    rt.run(d)

    def exercise(self,failure=None,diagnostic_true_body_yaw=False,gps_period_s=0.,session_choices=None,staging_choices=None):
        hyper=failure in ('hyper_left','hyper_right')
        full=failure in ('full_left','full_right','hyper_left','hyper_right','fallback_full_left','session_finish_right')
        warm=failure in ('warm_recovery','warm_stop','warm_reverse_stop','warm_delayed_start')
        manual=failure=='manual_bootstrap'
        clock=SimpleNamespace(t=1000.,x=0.,last=1000.,trial=None,started=False,stopped_at=None,commands=[],velocity=0.,pushed=False,y=0.,yaw=0.,wheel=0.,signed_travel=0.,dummy_at=None,reverse_frames=0,localized=False)
        points=[qr.Point(i,float(i),0.,0.,'synthetic','map') for i in range(45)]
        bundle={'points':points,'labels':[f'synthetic:{i}' for i in range(45)],'s':[float(i) for i in range(45)],
                'gates':[Gate('hill','hill',10.,10.3,3.),Gate('light_1','signal',20.,20.3,allowed=('green',)),
                         Gate('s_curve','unsupported',30.,30.3)],'allowed':list(range(8)),
                'hashes':{'synthetic':'synthetic'},'section':'start','lane':'left'}
        if full:
            points=[qr.Point(i,float(i),0.,0.,'synthetic','map') for i in range(41)]
            bundle.update(points=points,labels=[f'synthetic:{i}' for i in range(41)],s=list(map(float,range(41))),
                gates=[Gate('hill','hill',5.,5.3,3.),Gate('light_1','signal',10.,10.3,allowed=('green',)),
                       Gate('s_curve','s_curve',15.,27.),Gate('light_2','signal',32.,32.3,allowed=('green',)),
                       Gate('t_parking','parking',38.7,39.)],lane='left' if failure.endswith('_left') else 'right')
        real_angle_to_adc=rt.q.sc.angle_to_adc
        real_cal,_=rt.q.sc.validate(rt.q.sc.load(rt.q.DEFAULT_CAL))
        class Trial:
            max_hold_s=3.;ack_wait_s=.4;exit_reason=None;stop_ack_observed=False;current_stop=None
            def __init__(self):
                self.client=SimpleNamespace(encoder=None,failed=False,tick=lambda t:self.tick(t))
                self.validated=real_cal if full else {'points':[{'angle_cdeg':0,'adc':477}],'restart_band_adc':60}
                self.adapter=SimpleNamespace(status=None,phase='locked',locked_by_fault=False,
                    first_fault=None,identified=True,last_status_at=clock.t,have_target_since_prepare=True,
                    request_prepare=self.prepare)
                self.quitting=False;self.prepares=0;clock.trial=self
            def prepare(self,t):
                self.prepares+=1;self.adapter.phase='ready';return True
            def prepared(self):return self.adapter.phase=='ready'
            def command_target(self,wheel,t,hold):clock.commands.append((t,wheel));clock.wheel=wheel;return True
            def tick(self,t):
                gate=self.client.encoder
                encoded=None if gate is None else gate(rt.q.lpf.smp.Command(7,int(t*1000),rt.q.lpf.smp.CONTROL,0,0,0))
                axis=0 if encoded is None else (encoded[13] if encoded[13]<128 else encoded[13]-256)
                driving=axis!=0 and not self.quitting
                if axis<0:clock.reverse_frames+=1
                dt=t-clock.last;clock.last=t
                if full:
                    clock.velocity=1.0*axis if driving else math.copysign(max(0.,abs(clock.velocity)-min(dt,.1)*2.5),clock.velocity)
                    v=clock.velocity;delta=math.radians(clock.wheel/100)
                    clock.signed_travel+=min(dt,.1)*v
                    curvature=math.tan(delta)/(.75+.335*math.tan(delta))
                    clock.x+=min(dt,.1)*v*math.cos(clock.yaw);clock.y+=min(dt,.1)*v*math.sin(clock.yaw)
                    clock.yaw+=min(dt,.1)*v*curvature
                elif failure in ('short_unknown','rtcm_coast','gnss_coast','gnss_long','quick_green','held_push'):
                    clock.velocity=1.0 if driving else max(0.,clock.velocity-min(dt,.1)*2.5)
                    clock.x+=min(dt,.1)*clock.velocity
                elif driving:clock.x+=min(dt,.1)*1.0
                if failure=='held_push' and clock.x>9.5 and not driving and not clock.pushed:
                    if clock.stopped_at is None:clock.stopped_at=t
                    elif t-clock.stopped_at>1.:clock.x+=.3;clock.pushed=True
                self.adapter.last_status_at=t
                self.adapter.status=SimpleNamespace(boot_id=8 if failure=='reboot' and clock.x>3 else 7,
                    request_id=int(t*1000),encoder_count=int((clock.signed_travel if full else clock.x)*100),state=0 if self.quitting or not self.prepared() else 1,
                    requested_drive=axis if driving else 0,actual_drive=axis if driving else 0,steering_adc=real_angle_to_adc(self.validated['points'],clock.wheel) if full else 477,left_cm=999,center_cm=999,right_cm=999)
                if failure=='steer_stall' and clock.x>3:
                    self.adapter.locked_by_fault=True;self.adapter.first_fault='steering output held 1.5s ADC unchanged'
                if failure=='waiting_serial_fault' and not self.quitting:
                    self.adapter.locked_by_fault=True;self.adapter.first_fault='요청 번호 역행 synthetic waiting fault'
                if self.quitting and failure!='no_ack':self.stop_ack_observed=True
                if self.quitting and failure in ('post_ack_reboot','post_ack_fault'):
                    self.quit_ticks=getattr(self,'quit_ticks',0)+1
                    if self.quit_ticks>1:
                        if failure=='post_ack_reboot':self.adapter.status.boot_id=8
                        self.adapter.locked_by_fault=True
                        self.adapter.first_fault='new fault after DISARM ACK'
                return not self.quitting or failure=='no_ack'
            def quit(self,t):
                self.quitting=True
                if failure in ('operator_stop_lock','stop_before_start'):
                    self.adapter.locked_by_fault=True;self.adapter.first_fault='operator quit';self.adapter.message='operator quit'
                # Same as real emergency_command(): bind DISARM to latest status.
                self.current_stop={'sent':True,'boot':self.adapter.status.boot_id,'req':int(t*1000)}
        class Input:
            def __init__(self,*_):self.fix=None;self.scan=None;self.signal=None;self.gga_intervals=[.1]*5;self.rtcm_fresh=True;self.n=0
            def poll(self,*_):
                self.n+=1
                if failure=='service_probe' and clock.trial.prepared():
                    assert callable(getattr(self,'service_board',None))
                    self.service_board();clock.service_calls=getattr(clock,'service_calls',0)+1
                new_fix=self.fix is None or clock.t-self.fix[2]>=gps_period_s-1e-8
                if new_fix:self.fix=(clock.x+.315*math.cos(clock.yaw),clock.y+.315*math.sin(clock.yaw),clock.t)
                self.rtcm_fresh=not(failure in ('rtcm_gap','rtcm_coast') and 350<=self.n<550)
                if failure in ('gnss_coast','gnss_jump') and 350<=self.n<550:self.fix=None
                if failure=='gnss_long' and 350<=self.n<750:self.fix=None
                if failure=='gnss_jump' and self.n>=550:self.fix=(self.fix[0]+.4,self.fix[1],self.fix[2])
                if failure=='gnss_drop' and 3.<clock.x<3.5 and self.n<800:self.fix=None
                self.scan={'mount_id':'synthetic','frame':'rear_axle_FLU_m','transform_count':1,
                           'received_at':clock.t,'points':[(5,0,.28)],'complete_candidate':True,'forward_bins_covered':18}
                if failure=='short_unknown' and self.n in (350,351):self.scan=None
                self.signal={'source':'synthetic-camera','frame_id':self.n,'capture_at':clock.t-.01,
                    'decision_at':clock.t,'target':'light_1','epoch':0,'valid':True,'confidence':.9,'sign':'green','input_kind':'live'}
                if full:
                    request=json.loads((session/'signal_request.json').read_text())
                    self.signal.update(target=request['target'],sign=('allow_left' if failure.endswith('_left') else 'allow_right') if request['target']=='finish_fork' else 'green')
                    if failure=='fallback_full_left' and request['target']=='finish_fork':self.signal['sign']='none'
                    if not hasattr(self,'full_scan') or self.n%10==0:
                        self.full_scan={**self.scan,'rays':[(math.radians(i*.5),200.) for i in range(720)],
                                        'points':[(1+200*math.cos(math.radians(i*.5)),200*math.sin(math.radians(i*.5)),.28) for i in range(720)]}
                    self.scan=self.full_scan
                    if (math.hypot(clock.x-dummy_xy[0],clock.y-dummy_xy[1])<.8 if hyper else clock.x>62) and clock.dummy_at is None:clock.dummy_at=clock.t
                if failure=='boundary_late_red':
                    self.fix=(self.fix[0],self.fix[1],clock.t-.03)
                    if clock.x>=20.3:self.signal['sign']='red'
                if failure=='quick_green' and clock.x<19.5:self.signal['sign']='red'
                self.camera_frame={'input_kind':'live','source':self.signal['source'],'capture_at':self.signal['capture_at']}
                if failure=='optional_dead':self.signal=None;self.camera_frame=None
                if failure=='lift' and not clock.started:
                    clock.x=4. if clock.t>=1001. else 0.
                    self.fix=(clock.x+.315,0.,clock.t)
                if failure=='late_poll':clock.t+=.2
                return [self.fix] if self.fix and new_fix else []
        class Command:
            def __init__(self,*_):pass
            def poll(self,phase):
                if staging_choices and not clock.localized:
                    clock.localized=True;return {'action':'choices','payload':staging_choices,'nonce':'staging-choice'}
                if failure=='session_finish_right' and not clock.localized:
                    clock.localized=True;return {'action':'choices','payload':{'t':'left','p':'down','finish':'right'},'nonce':'synthetic-choice'}
                if failure=='stop_before_start' or (failure in ('warm_stop','warm_reverse_stop') and clock.localized):return 'stop'
                if warm and not clock.localized and clock.t>=1020.:
                    clock.localized=True
                    return {'action':'localize','payload':{'body_heading_deg':180. if failure=='warm_reverse_stop' else 0.,'observed_wall':clock.t,'index':None}}
                if failure=='warm_delayed_start' and clock.t<1025.:return None
                if failure=='lift' and clock.t<1002.:return None
                if phase=='waiting' and not clock.started:clock.started=True;clock.start_fix_x=clock.x;return 'start'
                if (not full and clock.x>=29.5) or clock.t>(1850. if hyper else (1250. if full else 1070.)):
                    if clock.stopped_at is None:clock.stopped_at=clock.t
                    elif clock.t-clock.stopped_at>.5:return 'stop'
                return None
            def close(self):pass
        class Workers:
            def __init__(self,*_):pass
            def start(self,*_):pass
            def check(self):return {n:{'state':'ended','exit_code':1} for n in ('camera','signal')} if failure=='optional_dead' else {}
            def close(self):return {'synthetic_only':0}
        class Transport:
            def __init__(self,*_,**kw):pass
            def close(self):pass
        def sleep(dt):clock.t+=max(dt,.01)
        config=json.loads((rt.ROOT/'config/main_event.json').read_text())
        profile={'mount_id':'synthetic','input_frame':'sdk_raw_polar','output_frame':'rear_axle_FLU_m',
            'driver_transform_count':0,'theta_sign':-1,'roll_deg':180,'yaw_deg':0.,'x_m':1.,'y_m':0.,'z_m':.28,
            'direction_verified':True,'direction_evidence':'synthetic only','body_outline_verified':True,
            'body_width_m':.9,'clearance_m':.1,'stop_distance_m':2.,'front_overhang_from_rear_axle_m':1.}
        if full:
            road={'verified':True,'evidence':'SYNTHETIC ROAD ONLY','frame':'map','polygon':[[-150,-150],[150,-150],[150,150],[-150,150]]}
            config.update(s_avoidance_verified=True,reverse_enabled=True,finish_rear_wheel_line_confirmed=True,
                road_boundaries={name:road for name in ('s_curve','t_parking','p_parking')},
                finish_line={'a':[98,-4],'b':[98,4],'forward_normal':[1,0],'verified':True,'evidence':'SYNTHETIC LINE ONLY','margin_m':.05})
            config['signals']={'light_1':['green'],'light_2':['green'],'light_3':['green']}
            bundle['config']=config
            if hyper:
                import main_event_plan as real_plan
                bundle=real_plan.build_section('start',bundle['lane'],config,parking_choices=session_choices)
                clock.x=bundle['points'][0].x;clock.y=bundle['points'][0].y
                clock.yaw=qr.path_heading_at(bundle['points'],0)
                end_name='end_'+bundle['lane']
                end=qr.load_track_csv(end_name,real_plan.TRACK/(end_name+'.csv')).points
                h=qr.path_heading_at(end,len(end)-1);nx,ny=math.cos(h),math.sin(h)
                centre=(end[-1].x-2*nx,end[-1].y-2*ny)
                config['finish_line'].update(a=[centre[0]+ny*5,centre[1]-nx*5],b=[centre[0]-ny*5,centre[1]+nx*5],forward_normal=[nx,ny])
                dummy=qr.load_track_csv('common_2',real_plan.TRACK/'common_2.csv').points[227]
                dummy_xy=(dummy.x,dummy.y)
        bundle.setdefault('config',config)
        real_pre=rt.q.lst.preflight
        with tempfile.TemporaryDirectory(dir=rt.ROOT/'data') as temp:
            session=Path(temp)
            (session/'session.json').write_text(json.dumps({'run_id':'R','epoch':0,'section':'start','lane':bundle['lane']}))
            (session/'config.json').write_text(json.dumps(config));(session/'lidar_profile.json').write_text(json.dumps(profile))
            info=json.loads((session/'session.json').read_text())
            if session_choices:info['session_choices']=session_choices
            if manual:
                info.update(run_id='manual_preserved',epoch=2,completed=['previous_mission'],skipped=['s_curve'])
                info['recovery']={'pose_pending':True,'judge_permitted':True,'boot':5,'boot_binding_pending':True,
                                  'heading_mode':'forward_bootstrap','allowed_indices':list(range(8)),
                                  'direction':1,'window_start':0,'window_end':44}
            if warm:
                info['recovery']={'pose_pending':True,'judge_permitted':True,'boot':7,'allowed_indices':list(range(8)),
                                  'direction':1,'window_start':0,'window_end':44}
                if failure=='warm_stop':info['completed']=['hill']
                if failure=='warm_reverse_stop':
                    info['completed']=['hill'];info['epoch']=4
                    info['recovery'].update(direction=-1,window_start=2,window_end=12,allowed_indices=list(range(2,8)))
                    clock.x=2.;clock.yaw=math.pi
            info.update(config_sha256=rt.digest(session/'config.json'),lidar_profile_sha256=rt.digest(session/'lidar_profile.json'))
            (session/'session.json').write_text(json.dumps(info))
            if full:
                import main_event_plan as plan
                import main_event_sequence as sequence
                track=session/'tracks';track.mkdir()
                def write_track(name,phases):
                    coords=[]
                    for start,end,n in phases:
                        row=[start+(end-start)*i/n for i in range(n+1)]
                        coords+=row if not coords else row[1:]
                    with (track/(name+'.csv')).open('w') as f:
                        writer=csv.writer(f);writer.writerow(['idx','x','y','yaw','frame_id'])
                        writer.writerows((i,x,0.,0.,'map') for i,x in enumerate(coords))
                for name,phases in {'t_left':[(40,34,23),(34,50,29)],'t_right':[(40,34,24),(34,50,29)],
                                    'p_up':[(70,74,8),(74,68,15),(68,80,18)],'p_down':[(70,67,9),(67,80,21)],
                                    'end_left':[(90,100,20)],'end_right':[(90,100,20)]}.items():write_track(name,phases)
                def next_section(section,lane,cfg,*args,**kwargs):
                    if section=='start':return bundle
                    begin,end=(50,70) if section=='after_t' else (80,90)
                    pts=[qr.Point(i,float(x),0.,0.,'synthetic','map') for i,x in enumerate(range(begin,end+1))]
                    gates=([Gate('light_3','signal',8.,8.3,allowed=('green',)),Gate('p_parking','parking',18.7,19.)]
                           if section=='after_t' else [Gate('finish_fork','unsupported',8.7,9.)])
                    return {'points':pts,'labels':[f'{section}:{i}' for i in range(len(pts))],'s':list(map(float,range(len(pts)))),
                            'gates':gates,'allowed':list(range(5)),'hashes':{},'section':section,'lane':lane,'config':cfg,
                            'front_offset_m':1.,'stop_margin_m':.275}
                bundle.update(front_offset_m=1.,stop_margin_m=.275)
            with contextlib.ExitStack() as stack:
                stack.enter_context(patch.object(rt,'enable_startup_sync',lambda client:None))
                if diagnostic_true_body_yaw:
                    class IdealBodyYaw(rt.BodyHeading):
                        # Diagnostic INPUT ONLY: unavailable perfect body sensor.
                        # Keep the real path planner/controller and steering limits.
                        def gps(self,x,y,stamp):self.yaw=qr.wrap(clock.yaw);self.anchor_at=stamp
                    stack.enter_context(patch.object(rt,'BodyHeading',IdealBodyYaw))
                if full:
                    class InlinePlanner:
                        # Deterministic scheduling only; calls the REAL planner.
                        # Async deadlines are covered separately with a real wall clock.
                        def __init__(self):self.result=None
                        def submit(self,kind,key,fn,*args):
                            if self.result is not None:return False
                            self.result={'kind':kind,'key':key,'value':fn(*args),'error':None,'compute_s':0.};return True
                        def poll(self):r=self.result;self.result=None;return r
                        def close(self):pass
                    stack.enter_context(patch.object(sequence,'PlanWorker',InlinePlanner))
                    if not hyper:
                        stack.enter_context(patch.object(plan,'TRACK',track));stack.enter_context(patch.object(sequence,'TRACK',track))
                        stack.enter_context(patch.object(plan,'build_section',side_effect=next_section))
                actual_build_section=rt.build_section
                patches=[patch.object(rt,'build_section',side_effect=lambda *a,**kw:actual_build_section(*a,**kw) if hyper and staging_choices else bundle),patch.object(rt,'Inputs',Input),
                         patch.object(rt,'Commands',Command),patch.object(rt,'Workers',Workers),
                         patch.object(rt,'obstacle_state',side_effect=lambda *a:'blocked' if failure=='nonmission_blocked' else 'unknown' if failure=='nonmission_unknown' else 'blocked' if full and clock.dummy_at is not None and clock.t-clock.dummy_at<.3 else 'clear'), # R1/R2 fixture; SDK pipeline tested separately
                         patch.object(rt.q.lst,'PosixSerialTransport',Transport),
                         patch.object(rt.q.lst,'assemble',side_effect=lambda *args,**kw:Trial()),
                         patch.object(rt.q.lst,'preflight',side_effect=lambda cal,angle,port:real_pre(cal,angle,port=None)),
                         patch.object(rt.q.sc,'angle_to_adc',return_value=477),
                         patch.object(rt.time,'monotonic',side_effect=lambda:clock.t),
                         patch.object(rt.time,'time',side_effect=lambda:clock.t),
                         patch.object(rt.time,'sleep',side_effect=sleep)]
                for p in patches:stack.enter_context(p)
                stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                code=rt.run(session)
            saved=json.loads((session/'state.json').read_text())
            clock.final_route=json.loads((session/'final_route.json').read_text())
            clock.session_info=info
            events=[json.loads(row) for row in (session/'vehicle/events.jsonl').read_text().splitlines()]
            decisions=[e for e in events if e['event']=='mission_decision']
            clock.transitions=[e for e in events if e['event']=='normal_mission_transition']
            clock.decision_reasons=[e['reason'] for e in decisions]
            print(json.dumps({'input_kind':'synthetic','gps_period_s':gps_period_s,'diagnostic_true_body_yaw':diagnostic_true_body_yaw,'planner_dispatch':'inline_actual_planner' if full else 'unused','failure':failure,'exit_code':code,'completed':saved['completed'],
                              'stop_ack':saved['stop_ack'],'reason':saved['reason'],'end_pose':[clock.x,clock.y,clock.yaw],
                              'exceptions':[e for e in events if e['event']=='runtime_exception'],
                              'last_follow':[e for e in events if e['event']=='follow'][-8:] if hyper else [],
                              'bootstrap_count':sum(e['event']=='bootstrap' for e in events),
                              'decisions':[{k:e.get(k) for k in ('reason','s_m','observed_stop_s')} for e in decisions]}))
            return code,saved,clock

    def test_real_loop_bootstrap_hill_green_then_s_barrier_no_manual_continue(self):
        code,saved,clock=self.exercise()
        self.assertEqual(code,130,saved['reason'])
        self.assertEqual(saved['completed'],['hill','light_1'],saved['reason'])
        self.assertEqual(saved['skipped'],[])
        self.assertTrue(saved['stop_ack']);self.assertTrue(saved['stationary_observed'])
        self.assertEqual(clock.trial.prepares,1)
        self.assertLess(clock.x,30.3)

    def test_reboot_after_disarm_ack_cannot_confirm_final_stationarity(self):
        code,saved,clock=self.exercise('post_ack_reboot')
        self.assertEqual(code,3)
        self.assertTrue(saved['stop_ack'])  # historical ACK belongs to old boot
        self.assertFalse(saved['stationary_observed'])
        self.assertEqual(saved['fault'],'board_reboot')

    def test_delayed_input_poll_does_not_arm_using_stale_board_readiness(self):
        code,saved,clock=self.exercise('late_poll')
        self.assertEqual(clock.trial.prepares,0)
        self.assertEqual(code,130,saved['reason'])

    def test_fault_after_disarm_ack_cannot_confirm_final_stationarity(self):
        code,saved,clock=self.exercise('post_ack_fault')
        self.assertEqual(code,3)
        self.assertTrue(saved['stop_ack'])
        self.assertFalse(saved['stationary_observed'])
        self.assertEqual(saved['fault'],'board_fault_after_stop')

    def test_missing_ack_reports_exit3_not_stopped_success(self):
        code,saved,_=self.exercise('no_ack')
        self.assertEqual(code,3);self.assertFalse(saved['stop_ack']);self.assertEqual(saved['fault'],'stop_unconfirmed')

    def test_reboot_never_arms_twice(self):
        code,saved,clock=self.exercise('reboot')
        self.assertEqual(code,1);self.assertEqual(saved['fault'],'board_reboot');self.assertEqual(clock.trial.prepares,1)

    def test_steering_stall_is_preserved_and_disarmed(self):
        code,saved,clock=self.exercise('steer_stall')
        self.assertEqual(code,1);self.assertIn('ADC unchanged',saved['reason']);self.assertTrue(saved['stop_ack'])
        self.assertEqual(clock.trial.prepares,1)

    def test_correction_gap_recovers_same_run_without_second_arm_or_bootstrap(self):
        code,saved,clock=self.exercise('rtcm_gap')
        self.assertEqual(code,130,saved['reason'])
        self.assertEqual(saved['completed'],['hill','light_1'],saved['reason'])
        self.assertEqual(clock.trial.prepares,1)

    def test_actual_hyper_map_with_synthetic_unbounded_visibility_and_road(self):
        # HYPER source path/labels/planning/controller are real; broad road,
        # instantaneous steering, exact GPS and all-clear beams are SYNTHETIC.
        self.full_assertions('hyper_left',gps_period_s=.1)

    def test_actual_hyper_map_right_start_and_finish(self):
        self.full_assertions('hyper_right',gps_period_s=.1)

    def test_full_actual_runtime_left_start_left_finish(self):
        self.full_assertions('full_left')

    def test_full_actual_runtime_right_start_right_finish(self):
        self.full_assertions('full_right')

    def full_assertions(self,kind,**kwargs):
        code,saved,clock=self.exercise(kind,**kwargs)
        self.assertEqual(code,0,saved['reason'])
        for name in ('hill','light_1','s_curve','light_2','t_parking','light_3','p_parking','finish'):
            self.assertIn(name,saved['completed'])
        self.assertGreater(clock.reverse_frames,0)
        self.assertEqual(clock.trial.prepares,1)
        self.assertGreaterEqual(len(clock.transitions),6)
        self.assertIn('obstacle_minimum_3s',clock.decision_reasons)
        self.assertTrue(saved['stop_ack']);self.assertTrue(saved['stationary_observed'])

    def test_actual_runtime_localizes_after_20_second_preparation_without_heading_expiry(self):
        code,saved,clock=self.exercise('warm_recovery')
        self.assertEqual(code,130,saved['reason'])
        self.assertTrue(clock.localized)
        self.assertEqual(saved['completed'],['hill','light_1'])
        self.assertEqual(clock.trial.prepares,1)

    def test_start_five_seconds_after_localize_preserves_stationary_anchor(self):
        code,saved,clock=self.exercise('warm_delayed_start')
        self.assertEqual(code,130,saved['reason'])
        self.assertEqual(saved['completed'],['hill','light_1'])
        self.assertEqual(clock.trial.prepares,1)

    def test_stop_before_entry_can_prepare_recovery_with_original_candidates(self):
        import main_event
        for kind in ('stop_before_start','warm_stop','warm_reverse_stop'):
            with self.subTest(kind=kind):
                code,saved,clock=self.exercise(kind)
                self.assertEqual(code,130)
                self.assertEqual(saved['boot'],7)
                self.assertIsNone(saved['progress'])
                self.assertTrue(saved['entry_pending'])
                expected=list(range(2,8)) if kind=='warm_reverse_stop' else list(range(8))
                self.assertEqual(saved['entry_candidates'],expected)
                with tempfile.TemporaryDirectory() as temp:
                    session=Path(temp)
                    (session/'state.json').write_text(json.dumps(saved))
                    (session/'session.json').write_text(json.dumps(clock.session_info))
                    (session/'final_route.json').write_text(json.dumps(clock.final_route))
                    saved['route_sha256']=rt.digest(session/'final_route.json')
                    (session/'state.json').write_text(json.dumps(saved))
                    a=SimpleNamespace(config=main_event.CONFIG,judge_permitted=True,body_heading_deg=None,
                                      index=None,skip_mission=None,prepare_only=True)
                    with patch.object(main_event,'active',return_value=session), \
                         patch.object(main_event.sys.stdin,'isatty',return_value=False), \
                         patch.object(main_event,'prepare',return_value=0) as prepare:
                        self.assertEqual(main_event.recover(a),0)
                    rec=prepare.call_args.args[1]
                    self.assertEqual(rec['run_id'],'R');self.assertEqual(rec['epoch'],clock.session_info['epoch']+1)
                    self.assertEqual(rec['completed'],saved['completed'])
                    self.assertTrue(rec['recovery']['pose_pending'])
                    self.assertEqual(rec['recovery']['allowed_indices'],expected)
                    self.assertEqual(rec['recovery']['direction'],-1 if kind=='warm_reverse_stop' else 1)

    def test_short_unknown_recovers_after_observed_coasting_stop(self):
        code,saved,clock=self.exercise('short_unknown')
        self.assertEqual(code,130,saved['reason'])
        self.assertEqual(saved['completed'],['hill','light_1'],saved['reason'])
        self.assertEqual(clock.trial.prepares,1)

    def test_manual_handoff_rebinds_boot_and_measures_heading_with_existing_bootstrap(self):
        code,saved,clock=self.exercise('manual_bootstrap')
        self.assertEqual(code,130,saved['reason'])
        self.assertIn('previous_mission',saved['completed'])
        self.assertIn('hill',saved['completed'])
        self.assertEqual(saved['skipped'],['s_curve'])
        self.assertEqual(saved['boot'],7)
        self.assertEqual(clock.trial.prepares,1)

    def test_green_before_settle_waits_then_resumes(self):
        code,saved,clock=self.exercise('quick_green')
        self.assertEqual(code,130,saved['reason'])
        self.assertEqual(saved['completed'],['hill','light_1'],saved['reason'])

    def test_correction_loss_with_observed_coasting_recovers(self):
        code,saved,clock=self.exercise('rtcm_coast')
        self.assertEqual(code,130,saved['reason'])
        self.assertEqual(saved['completed'],['hill','light_1'],saved['reason'])
        self.assertEqual(clock.trial.prepares,1)

    def test_position_jump_after_missing_fix_refuses_even_without_encoder_change(self):
        code,saved,clock=self.exercise('gnss_jump')
        self.assertNotIn('hill',saved['completed'])
        self.assertIn('pose_recovery_required',clock.decision_reasons)
        self.assertEqual(clock.trial.prepares,1)

    def test_long_unobserved_coasting_requires_recovery(self):
        code,saved,clock=self.exercise('gnss_long')
        self.assertNotIn('hill',saved['completed'])
        self.assertEqual(clock.trial.prepares,1)
        self.assertIn('pose_recovery_required',getattr(clock,'decision_reasons',[]))

    def test_bounded_gap_coasting_recovers_only_after_stationary_continuity(self):
        code,saved,clock=self.exercise('gnss_coast')
        self.assertEqual(code,130,saved['reason'])
        self.assertEqual(saved['completed'],['hill','light_1'],saved['reason'])
        self.assertEqual(clock.trial.prepares,1)
        self.assertIn('pose_recovery_required',clock.decision_reasons)

    def test_optional_workers_dead_allow_departure_but_hold_real_signal(self):
        code,saved,clock=self.exercise('optional_dead')
        self.assertEqual(code,130,saved['reason'])
        self.assertEqual(clock.trial.prepares,1)
        self.assertEqual(saved['completed'],['hill'])
        self.assertLess(clock.x,20.3)

    def test_session_finish_choice_runtime_overrides_model_and_is_preserved(self):
        code,saved,clock=self.exercise('session_finish_right')
        self.assertEqual(code,0,saved['reason'])
        self.assertEqual(saved['session_choices'],{'t':'left','p':'down','finish':'right'})
        self.assertEqual(saved['finish_lane'],'right')
        self.assertTrue(any('end_right' in s for s in clock.final_route['labels']))

    def test_running_connects_sensor_serial_service(self):
        code,saved,clock=self.exercise('service_probe')
        self.assertEqual(code,130,saved['reason'])
        self.assertGreater(clock.service_calls,100)
        self.assertEqual(saved['completed'],['hill','light_1'])

    def test_lift_waiting_uses_new_start_position_and_new_bootstrap(self):
        code,saved,clock=self.exercise('lift')
        self.assertEqual(code,130,saved['reason'])
        self.assertEqual(clock.start_fix_x,4.)
        self.assertEqual(clock.trial.prepares,1)
        self.assertEqual(saved['completed'],['hill','light_1'])

    def test_motion_after_stationary_anchor_refuses_resume(self):
        code,saved,clock=self.exercise('held_push')
        self.assertNotIn('light_1',saved['completed'])
        self.assertEqual(clock.trial.prepares,1)
        self.assertTrue(clock.pushed)


if __name__=='__main__':unittest.main()
