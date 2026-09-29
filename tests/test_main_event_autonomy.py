import json,math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import main_event_plan as plan
from main_event_autonomy import ProgressWatch
import main_event_manual as manual
from main_event_menu import Menu
import qualifier_route as qr
from main_event_maneuvers import Envelope,assess_path
CFG=json.loads((plan.ROOT/'config/main_event.json').read_text())

class AutonomyTests(unittest.TestCase):
 def test_preentry_routes_are_continuous_forward_and_preserve_prior_missions(self):
  data=json.loads((plan.ROOT/'data/main_event_autonomous_passages_20260920.json').read_text())
  profile=json.loads((plan.ROOT/'config/main_event_lidar_mount.json').read_text());env=Envelope(profile['front_overhang_from_rear_axle_m'],profile['rear_overhang_m'],profile['body_width_m'],profile['clearance_m'])
  for section,name in [('start','t_parking'),('after_t','p_parking')]:
   b=plan.build_section(section,'left',CFG);p=b['preentry_policy'];start=p['start_index'];spec=data['passages'][name]
   self.assertEqual(b['gates'][-1].name,name+'_not_entered');self.assertNotIn(name,[g.name for g in b['gates']])
   self.assertEqual(assess_path(b['points'][start-2:],env,spec['polygon'],[],lambda _:False,reference_candidate=True),'reference_candidate')
   self.assertLess(max(math.hypot(v.x-u.x,v.y-u.y) for u,v in zip(b['points'][start:],b['points'][start+1:])),1.5)
   following=plan.build_section(spec['next_section'],'left',CFG)
   self.assertLess(math.hypot(b['points'][-1].x-following['points'][0].x,b['points'][-1].y-following['points'][0].y),1e-7)
   _,base=plan.append_common(b,spec['next_section']);self.assertEqual(base,len(b['points']))
 def test_state_changes_and_encoder_noise_cannot_reset_nonprogress(self):
  w=ProgressWatch()
  for t in range(61):
   r=w.update(t,10,(0,0,t),{'received_at':t,'encoder':t},'replan_'+str(t))
  self.assertEqual(r['seconds_without_progress'],60);self.assertFalse(r['timeout_grants_drive'])
  r=w.update(61,10.6,(.5,0,61),{'received_at':61,'encoder':100},'follow')
  self.assertEqual(r['seconds_without_progress'],0)
  r=w.update(76,10.6,(.5,0,76),{'received_at':76,'encoder':101},'signal_wait:light_3')
  self.assertTrue(r['signal_wait']);self.assertEqual(r['seconds_without_progress'],15)
 def test_T_P_and_unconfirmed_S_manual_are_blocked_before_devices(self):
  for mission in ('s_curve','t_parking','p_parking'):
   with self.assertRaises(ValueError):manual.run_manual('nonexistent',mission)
  for name in ('t_parking','p_parking','s_curve'):
   menu=Menu(read=lambda:{'index':0,'gates':[{'name':name}]},execute=lambda _:self.fail('no command permitted'),ask=lambda _: '',emit=lambda *a:None)
   with self.assertRaises(ValueError):menu.action('m')

 def test_S_detour_remaps_following_nonentry_speed_approach(self):
  from dataclasses import replace
  b=plan.build_section('start','left',CFG);begin=20
  path=list(b['points'][begin:begin+5]);path[2]=replace(path[2],x=path[2].x+.4)
  out=plan.replace_local_path(b,begin,path);idx=out['preentry_policy']['start_index']
  self.assertAlmostEqual(out['preentry_policy']['start_s'],out['s'][idx])
  self.assertNotEqual(out['preentry_policy']['start_s'],b['preentry_policy']['start_s'])
 def test_nonentry_approach_cannot_use_cruise100(self):
  from main_event_speed import SpeedPolicy
  for section in ('start','after_t'):
   b=plan.build_section(section,'left',CFG);p=SpeedPolicy(CFG);p.speed=1.2;p.origin=(0,0)
   mission=NS(completed=['hill'],gate=b['gates'][-1]);begin=b['preentry_policy']['start_s']
   for pos in (begin-6,begin,b['s'][-1]):
    self.assertEqual(p.choose('running',mission,b,pos,1,(10,0,1),'clear',10),70)
 def test_confirmed_T_P_still_rejected_and_legacy_recover_blocked(self):
  import main_event
  for name in ('t_parking','p_parking'):
   with self.assertRaises(ValueError):manual.run_manual('nonexistent',name,collision_recovery_confirmed=True)
  with self.assertRaises(ValueError):main_event.recover(NS(after_manual=False))

class RuntimeAutonomyTests(unittest.TestCase):
 def test_HYPER_predeclared_nonentry_T_P_and_dummy_clear_no_operator_input(self):
  from test_main_event_runtime import RuntimeIntegrationTests
  import main_event_runtime as rt
  original=rt.q.lst.TrialJournal.event;events=[]
  def event(j,name,**kw):
   if name in ('autonomous_parking_nonentry_completed','bootstrap','mission_decision','normal_mission_transition'):events.append((name,kw))
   return original(j,name,**kw)
  with patch.object(rt.q.lst.TrialJournal,'event',event):
   code,saved,clock=RuntimeIntegrationTests().exercise('hyper_left',gps_period_s=.1)
  self.assertEqual(code,0,saved['reason']);self.assertEqual(saved['skipped'],['t_parking','p_parking'])
  self.assertEqual(clock.trial.prepares,1);self.assertEqual(sum(n=='bootstrap' for n,k in events),1)
  self.assertEqual(clock.reverse_frames,0)
  self.assertEqual(sum(n=='autonomous_parking_nonentry_completed' for n,k in events),2)
  stops=[k for n,k in events if n=='mission_decision' and k['reason']=='obstacle_minimum_3s']
  self.assertTrue(stops)
  releases=[k for n,k in events if n=='mission_decision' and k['reason']=='mission_permits' and k['observed_stop_s']>=3.]
  self.assertTrue(releases)
  print('NO M/N/R/second start; T/P road passages skipped at physical end; dummy clear resumed after >=3s; finish reached')
