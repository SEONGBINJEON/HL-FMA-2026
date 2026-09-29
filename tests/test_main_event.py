import copy
import json
import math
from pathlib import Path
import socket
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from main_event_core import Gate, Mission, Stationary, SignalVotes, ForkVotes, validate_recovery
from main_event_lidar import transform_node, obstacle_state, ScanAssembler, sonar_state, path_corridor
from main_event_control import Commands
from main_event_plan import ROOT, build_section, append_finish
from main_event_runtime import board_sample, Inputs
import main_event
import qualifier_route as qr
from main_event_signal import fork_decision
from types import SimpleNamespace


def board(t,req=None,enc=100,drive=0,boot=7,state='ARMED'):
    return {'received_at':t,'request':round(t*1000) if req is None else req,'encoder':enc,
            'requested_drive':drive,'boot':boot,'state':state,'fault':False}


def event(t,n,sign='green',target='light_1',epoch=0,**extra):
    return {'capture_at':t-.05,'decision_at':t,'source':'camera','frame_id':n,
            'sign':sign,'target':target,'epoch':epoch,'valid':True,'confidence':.8,**extra}


PROFILE={'mount_id':'synthetic_inverted','input_frame':'sdk_raw_polar','output_frame':'rear_axle_FLU_m',
         'driver_transform_count':0,'theta_sign':-1,'roll_deg':180,'yaw_deg':0.,
         'x_m':1.,'y_m':0.,'z_m':.28,'direction_verified':True,'direction_evidence':'SYNTHETIC TEST ONLY',
         'body_outline_verified':True,'body_width_m':.9,'clearance_m':.1,'front_overhang_from_rear_axle_m':1.,'stop_distance_m':2.}


class StationaryTests(unittest.TestCase):
    def test_command_zero_not_actual_stop_timer(self):
        st=Stationary()
        for t,enc in ((0,100),(.2,103),(.4,110),(.8,111)):
            self.assertFalse(st.update(board(t,enc=enc),t))
        self.assertFalse(st.update(board(1.2,enc=111),1.2))
        self.assertTrue(st.update(board(1.4,enc=111),1.4))
        self.assertAlmostEqual(st.duration(4.4),3.)

    def test_duplicate_stale_and_requested_drive(self):
        st=Stationary();st.update(board(0,req=1),0)
        self.assertFalse(st.update(board(.6,req=1),.6))
        self.assertFalse(st.update(board(.7,drive=1),.7))
        self.assertFalse(st.update(board(.7),1.))

    def test_encoder_motion_restarts_minimum(self):
        st=Stationary();st.update(board(0),0);st.update(board(.6),.6)
        self.assertTrue(st.update(board(3.6),3.6))
        self.assertFalse(st.update(board(3.7,enc=101),3.7))
        self.assertEqual(st.duration(3.7),0)


class SignalTests(unittest.TestCase):
    def test_finish_two_panel_mapping_requires_three_visible_slots(self):
        def box(cls,x,conf=.9):
            return SimpleNamespace(cls=[cls],conf=[conf],xyxy=[SimpleNamespace(tolist=lambda:[x,20,x+20,40])])
        result=SimpleNamespace(names={0:'Allow',1:'Ban',2:'Blank'},boxes=[box(0,10),box(1,50),box(0,90)])
        self.assertEqual(fork_decision(result)['sign'],'allow_left')
        result.boxes=result.boxes[:2]
        self.assertIsNone(fork_decision(result))
        result.boxes=[box(1,10),box(0,50),box(2,90),box(2,11,.5)]
        self.assertEqual(fork_decision(result)['sign'],'allow_right')
    def test_finish_window_uses_distinct_fresh_frames_and_no_default(self):
        votes=ForkVotes(0)
        e=event(1.,0,'allow_left',target='finish_fork')
        for t in (1.,1.1,1.2):self.assertIsNone(votes.update(e,t))
        self.assertIsNone(votes.update(event(1.3,1,'allow_left',target='finish_fork'),1.3))
        self.assertEqual(votes.update(event(1.4,2,'allow_left',target='finish_fork'),1.4),'left')
        self.assertIsNone(votes.update(None,5.))
    def test_fresh_unique_three_and_red_revocation(self):
        v=SignalVotes('light_1',0)
        for i,t in enumerate((1.,1.1,1.2)):
            result=v.update(event(t,i),t,('green',))
            self.assertEqual(result,i==2)
        self.assertFalse(v.update(event(1.3,4,'red'),1.3,('green',)))

    def test_same_frame_never_counts_twice_or_refreshes_age(self):
        v=SignalVotes('light_1',0);e=event(1.,1)
        for t in (1.,1.1,1.2):self.assertFalse(v.update(e,t,('green',)))
        self.assertEqual(v.count,1)
        self.assertFalse(v.update(e,2.,('green',)))

    def test_old_future_wrong_target_epoch_and_unknown_permission(self):
        bad=[event(1.,1),event(4.,2),event(3.,3,target='light_2'),event(3.,4,epoch=2),
             event(3.,5,confidence=float('nan'))]
        for e in bad:
            v=SignalVotes('light_1',0)
            self.assertFalse(v.update(e,3.,('green',)))
        v=SignalVotes('light_3',0)
        for n in range(4):self.assertFalse(v.update(event(3+n*.1,n,target='light_3'),3+n*.1,()))

    def test_long_gap_breaks_continuity(self):
        v=SignalVotes('light_1',0)
        for n,t in enumerate((1.,1.1,2.)):v.update(event(t,n),t,('green',))
        self.assertEqual(v.count,1)


class MissionTests(unittest.TestCase):
    def make(self):return Mission([Gate('hill','hill',10,10.275,3),Gate('light_1','signal',20,20.275,allowed=('green',)),Gate('s_curve','unsupported',30,30.275)],'test')
    def tick(self,m,t,s,**kw):
        defaults=dict(s=s,now=t,board=board(t),quality_ok=True,heading_ok=True,aligned=True,obstacle='clear')
        defaults.update(kw);return m.step(**defaults)
    def test_no_input_hill_signal_then_explicit_s_barrier(self):
        m=self.make()
        self.assertTrue(self.tick(m,0,0)['drive'])
        for t in (1.,1.1,1.7,4.6):self.assertFalse(self.tick(m,t,9.5)['drive'])
        self.assertTrue(self.tick(m,4.8,9.5)['drive'])
        self.assertEqual(m.completed,['hill'])
        for i,t in enumerate((5.,5.1,5.2)):
            r=self.tick(m,t,19.5,signal=event(t,i))
            self.assertFalse(r['drive'])  # three greens do not bypass observed settling
        self.assertFalse(self.tick(m,5.3,20.1,signal=event(5.3,3,'red'))['drive'])
        for i,t in enumerate((5.4,5.5,5.6),10):self.tick(m,t,20.1,signal=event(t,i))
        self.assertTrue(self.tick(m,5.7,20.3)['drive'])
        self.assertEqual(m.completed,['hill','light_1'])
        self.assertEqual(self.tick(m,6.,29.6)['reason'],'unsupported:s_curve')
        self.assertEqual(m.completed,['hill','light_1'])
    def test_red_crossing_fault_not_completion(self):
        m=Mission([Gate('light_1','signal',20,20.275,allowed=('green',))],'test')
        r=self.tick(m,1,20.3,signal=event(1,1,'red'))
        self.assertFalse(r['drive']);self.assertIn('without_permission',m.fault);self.assertFalse(m.completed)
    def test_stop_timer_starts_after_settling_not_cut(self):
        m=self.make();self.tick(m,0,9.6)
        for t,e in ((.1,101),(.5,106),(.7,108),(1.3,108),(3.5,108)):
            self.assertFalse(self.tick(m,t,9.6,board=board(t,enc=e))['drive'])
        self.assertTrue(self.tick(m,4.4,9.6,board=board(4.4,enc=108))['drive'])
    def test_gnss_wait_autoresume_only_same_stationary_session(self):
        m=self.make();self.tick(m,1,2,quality_ok=False)
        for t in (1.1,1.7):self.assertFalse(self.tick(m,t,2,quality_ok=False)['drive'])
        self.assertTrue(self.tick(m,2.3,2)['drive'])
        m=self.make();self.tick(m,1,2,quality_ok=False)
        for t in (1.1,1.7,3.):
            self.assertFalse(self.tick(m,t,2,board=board(t,enc=101))['drive'])
        self.assertTrue(m.recovery_required)
    def test_reboot_disarm_fault_never_network_rearms(self):
        for mutation in ({'boot':8},{'state':'DISARMED'},{'fault':True},{'received_at':-1}):
            m=self.make();self.tick(m,1,2)
            b=board(2);b.update(mutation)
            self.assertFalse(self.tick(m,2,2,board=b)['drive'])
            self.assertFalse(self.tick(m,3,2)['drive'])
            self.assertIsNotNone(m.fault)
    def test_obstacle_requires_observed_3s_then_clear(self):
        m=self.make();self.tick(m,1,2,obstacle='blocked')
        for t in (1.1,1.7,4.6):self.assertFalse(self.tick(m,t,2,obstacle='blocked')['drive'])
        self.assertFalse(self.tick(m,4.8,2,obstacle='unknown')['drive'])
        self.assertTrue(self.tick(m,4.9,2,obstacle='clear')['drive'])
    def test_unsupported_parking_and_finish_do_not_autoskip(self):
        for kind,name in [('unsupported','t_parking'),('unsupported','p_parking'),('finish','finish')]:
            m=Mission([Gate(name,kind,10,11)],'test')
            for t in (0.,1.,4.,100.):self.assertFalse(self.tick(m,t,10)['drive'])
            self.assertEqual(m.completed,[])


class LidarTests(unittest.TestCase):
    def test_curved_path_obstacle_outside_straight_roi(self):
        pts=[qr.Point(i,x,y,0.,'synthetic','map') for i,(x,y) in enumerate([(0,0),(1,0),(2,1),(2,2)])]
        corridor=path_corridor(pts,0,0,0,0,4)
        scan={'mount_id':PROFILE['mount_id'],'frame':'rear_axle_FLU_m','transform_count':1,
              'received_at':1.,'points':[(2,1,.28)],'complete_candidate':True,'forward_bins_covered':18,
              'rays':[(math.radians(i*.5),5.) for i in range(720)]}
        self.assertEqual(obstacle_state(scan,PROFILE,1.1),'clear')
        self.assertEqual(obstacle_state(scan,PROFILE,1.1,corridor),'blocked')
    def test_inverted_front_left_right_single_transform(self):
        p=copy.deepcopy(PROFILE)
        front=transform_node(0,2000,10,p,p['mount_id'])
        left=transform_node(90,2000,10,p,p['mount_id'])
        right=transform_node(270,2000,10,p,p['mount_id'])
        self.assertAlmostEqual(front[0],3);self.assertAlmostEqual(front[1],0)
        self.assertGreater(left[1],0);self.assertLess(right[1],0)
    def test_planar_consumer_does_not_require_fabricated_exact_height(self):
        p={**PROFILE,'z_m':None,'planar_only':True}
        point=transform_node(0.,2000.,10,p,p['mount_id'])
        self.assertEqual(point,(3.,0.,None))
        with self.assertRaisesRegex(ValueError,'height'):
            transform_node(0.,2000.,10,{**p,'planar_only':False},p['mount_id'])

    def test_yaw_and_translation_are_not_replaced_by_180_yaw(self):
        p={**PROFILE,'yaw_deg':90.,'y_m':.2}
        v=transform_node(0,2000,10,p,p['mount_id'])
        self.assertAlmostEqual(v[0],1.);self.assertAlmostEqual(v[1],2.2)
    def test_old_mount_double_transform_and_unknown_direction_refused(self):
        for changes,mount,count in [({},'obs3_unknown',0),({},PROFILE['mount_id'],1),
                                    ({'driver_transform_count':1},PROFILE['mount_id'],0),
                                    ({'direction_verified':False},PROFILE['mount_id'],0),
                                    ({'yaw_deg':None},PROFILE['mount_id'],0)]:
            p={**PROFILE,**changes}
            with self.assertRaises(ValueError):transform_node(0,2000,10,p,mount,transform_count=count)
    def test_no_return_is_unknown_partial_scan_and_stale(self):
        scan={'mount_id':PROFILE['mount_id'],'frame':'rear_axle_FLU_m','transform_count':1,
              'received_at':1.,'points':[],'complete_candidate':True,'forward_bins_covered':0}
        self.assertEqual(obstacle_state(scan,PROFILE,1.1),'unknown')
        scan.update(points=[(2,0,.28)])
        self.assertEqual(obstacle_state(scan,PROFILE,1.1),'blocked')
        self.assertEqual(obstacle_state(scan,PROFILE,2.),'unknown')
        # Each received scan is a new object; cached visibility belongs to its rays.
        scan={k:v for k,v in scan.items() if not k.startswith('_')}
        scan.update(points=[(4,0,.28)],forward_bins_covered=18,
                    rays=[(math.radians(i*.5),5.) for i in range(720)])
        self.assertEqual(obstacle_state(scan,PROFILE,1.1),'clear')
        scan['complete_candidate']=False
        self.assertEqual(obstacle_state(scan,PROFILE,1.1),'unknown')
    def test_scan_clock_receipt_partial_dropped_no_fake_acquisition(self):
        a=ScanAssembler(PROFILE,PROFILE['mount_id'])
        raw=b'S theta: 0.00 Dist: 2000.00 Q: 10\n theta: 90.00 Dist: 0.00 Q: 0\n'
        self.assertEqual(a.feed(raw[:12],1.),[])
        self.assertEqual(a.feed(raw[12:],1.1),[])
        scans=a.feed(b'S theta: 0.00 Dist: 3000.00 Q: 10\n',1.2)
        self.assertEqual(len(scans),1);self.assertEqual(len(scans[0]['points']),1)
        self.assertEqual(scans[0]['received_at'],1.1)
        self.assertIn('not_sensor',scans[0]['time_basis'])
    def test_sonar_999_unknown_and_stop_only_when_channel_confirmed(self):
        b={**board(1.),'center_cm':999,'left_cm':10}
        self.assertEqual(sonar_state(b,['center_cm'],1.1,80),'unknown')
        self.assertEqual(sonar_state(b,[],1.1,80),'unknown')
        self.assertEqual(sonar_state(b,['left_cm'],1.1,80),'blocked')


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.saved={'stop_ack':True,'process_released':True,'fault':None,'run_id':'R','epoch':1,'completed':['hill']}
        self.req={'judge_permitted':True,'run_id':'R','epoch':2,'segment':20,'body_yaw':.5,
                  'heading_source':'measured_body_yaw','observed_at':1.,'stationary':True,'fresh_fix':True}
    def test_preserves_progress_and_completed(self):
        r=validate_recovery(self.saved,self.req,1.1,range(18,24))
        self.assertEqual(r['completed'],['hill']);self.assertFalse(r['prepared'])
    def test_no_ack_reboot_duplicate_stale_cross_segment_reverse_heading_refused(self):
        for change in ({'stop_ack':False},{'process_released':False},{'fault':'board_reboot'}):
            with self.assertRaises(ValueError):validate_recovery({**self.saved,**change},self.req,1.1,range(18,24))
        for change in ({'epoch':1},{'segment':200},{'observed_at':0},{'judge_permitted':False},
                       {'heading_source':'reverse_gps_course'},{'completed':['hill','light_1']},{'fresh_fix':False}):
            with self.assertRaises(ValueError):validate_recovery(self.saved,{**self.req,**change},1.1,range(18,24))


class PlanAndCLITests(unittest.TestCase):
    def test_interactive_recover_preserves_completed_and_marks_skip_separately(self):
        conf=json.loads((ROOT/'config/main_event.json').read_text())
        b=build_section('start','left',conf)
        with tempfile.TemporaryDirectory() as temp:
            d=Path(temp);pointer=d/'active.json';pointer.write_text(json.dumps({'session':str(d)}))
            (d/'session.json').write_text(json.dumps({'run_id':'R','epoch':4,'section':'start','lane':'left'}))
            saved={'stop_ack':True,'process_released':True,'stationary_observed':True,'fault':None,
                   'completed':['hill','light_1'],'skipped':[],'index':2,'gates':[vars(g) for g in b['gates']],
                   'progress':250,'boot':7}
            (d/'state.json').write_text(json.dumps(saved))
            a=SimpleNamespace(config=main_event.CONFIG,judge_permitted=False,body_heading_deg=None,index=None,skip_mission=None)
            with patch.object(main_event,'ACTIVE',pointer),patch.object(main_event.sys.stdin,'isatty',return_value=True), \
                    patch('builtins.input',side_effect=['YES','SKIP']),patch.object(main_event,'prepare',return_value=0) as launch, \
                    patch.object(main_event,'finish_recovery',return_value=0):
                self.assertEqual(main_event.recover(a),0)
            args,rec=launch.call_args.args
            self.assertEqual(rec['epoch'],5);self.assertEqual(rec['section'],'after_s')
            self.assertEqual(rec['completed'],['hill','light_1']);self.assertEqual(rec['skipped'],['s_curve'])
            self.assertIn(0,rec['recovery']['allowed_indices']);self.assertTrue(rec['recovery']['pose_pending'])

    def test_both_finish_branches_keep_common3_no_restart(self):
        conf=json.loads((ROOT/'config/main_event.json').read_text())
        b=build_section('after_p','left',conf)
        for lane in ('left','right'):
            extended=append_finish(b,lane)
            self.assertEqual(extended['points'][:len(b['points'])],b['points'])
            self.assertEqual(extended['gates'][0].kind,'finish')
            self.assertTrue(extended['labels'][-1].startswith('end_'+lane+':'))

    def test_stopline_reference_uses_rear_axle_plus_measured_antenna_bumper(self):
        conf=json.loads((ROOT/'config/main_event.json').read_text())
        b=build_section('start','left',conf);g=next(g for g in b['gates'] if g.name=='light_1')
        s=b['s'][b['labels'].index('common_1:68')]
        self.assertAlmostEqual(s-g.commit_s,1.040)
        self.assertAlmostEqual(s-g.stop_s,1.315)
    def test_main_raw_indices_left_right_all_sections(self):
        conf=json.loads((ROOT/'config/main_event.json').read_text())
        for lane in ('left','right'):
            for section in ('start','after_s','after_t','after_p'):
                b=build_section(section,lane,conf)
                self.assertTrue(b['allowed']);Mission(b['gates'],'test')
                if section=='start':
                    self.assertIn('common_1:287',b['labels'])
                    self.assertEqual([g.name for g in b['gates']],['hill','light_1','s_curve','light_2','t_parking'])
                if section=='start':self.assertNotIn('common_1:172',b['labels'])
                if section=='after_t':
                    self.assertNotIn('common_2:39',b['labels']);self.assertIn('common_2:98',b['labels'])
                if section=='after_s':self.assertEqual(b['labels'][0],'common_1:183')
    def test_unmeasured_profile_blocks_before_subprocess_or_device(self):
        config,profile,bundle=main_event.load(SimpleNamespace(config=main_event.CONFIG,section='start',lane='left'))
        profile={**profile,'theta_sign':None,'direction_verified':False}
        with patch.object(main_event,'load',return_value=(config,profile,bundle)), \
             patch('main_event.subprocess.Popen',side_effect=AssertionError('must not launch')):
            self.assertEqual(main_event.main(['prepare','--live','--lane','left']),2)
    def test_protocol_expiry_epoch_duplicate_and_no_start_after_running(self):
        with tempfile.TemporaryDirectory() as temp:
            server=Commands(temp,'R',1)
            with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as client:
                client.bind('\0test-main-event-'+str(time.monotonic()));client.settimeout(.1)
                msg={'run_id':'R','epoch':1,'action':'start','nonce':'a','sent_at':time.monotonic()}
                def attempt(payload,phase):
                    client.sendto(json.dumps(payload).encode(),str(server.path))
                    cmd=server.poll(phase);return cmd,json.loads(client.recv(4096))
                cmd,reply=attempt(msg,'waiting');self.assertEqual(cmd,'start');self.assertTrue(reply['accepted'])
                for change,phase in [({},'waiting'),({'nonce':'b'},'running'),({'nonce':'c','epoch':0},'waiting'),
                                      ({'nonce':'d','sent_at':time.monotonic()-3},'waiting')]:
                    cmd,reply=attempt({**msg,**change},phase);self.assertIsNone(cmd);self.assertFalse(reply['accepted'])
            server.close()


if __name__=='__main__':unittest.main()
