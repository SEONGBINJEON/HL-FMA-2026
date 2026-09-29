import json,struct,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from main_event_speed import SpeedPolicy
from main_event_motion import DirectionalDriveGate
from step_manual_protocol import Command,CONTROL
from main_event_core import Gate,Mission
from main_event_maneuvers import Envelope,s_candidates
import qualifier_route as qr
CFG=json.loads((Path(__file__).resolve().parents[1]/'config/main_event.json').read_text())

class SpeedTests(unittest.TestCase):
 def setup_policy(self):
  p=SpeedPolicy(CFG);p.speed=1.2;p.origin=(0,0)
  m=Mission([Gate('light_1','signal',50,51)],'speed',completed=['hill'])
  return p,m,{'section':'start'}
 def wire(self,pwm,open=True):
  g=DirectionalDriveGate(pwm,NS(locked_by_fault=False,phase='ready',have_target_since_prepare=True),clock=lambda:10.)
  if open:g.open(10.)
  raw=g(Command(1,2,CONTROL));return struct.unpack('<IIBbbBB',raw[4:-2]),g
 def test_sections_and_wire_not_config_only(self):
  p,m,b=self.setup_policy()
  cases=[('bootstrap',m,b,10,1,70),('running',m,b,10,1,100),('running',m,b,45,1,70),
         ('running',m,{'section':'parking_p_up'},10,-1,70),('running',m,{'section':'parking_t_right'},10,1,70)]
  for phase,mission,bundle,s,d,want in cases:
   pwm=p.choose(phase,mission,bundle,s,d,(10,0,1),'clear',10)
   self.assertEqual(pwm,want);wire,g=self.wire(pwm);self.assertEqual(wire[-1],want);self.assertEqual(g.last_wire_pwm,want)
  m.completed=[];self.assertEqual(p.choose('running',m,b,10,1,(10,0,1),'clear',10),70)
  m.completed=['hill'];m.gates=[Gate('s_curve','s_curve',20,60)]
  for pos in (14,21,59):self.assertEqual(p.choose('running',m,b,pos,1,(10,0,1),'clear',10),70)
 def test_release_recovery_long_observation_and_stop_wire(self):
  p,m,b=self.setup_policy();p.permit(False,(10,0,1));p.permit(True,(10,0,1))
  for x in (10,11.9):self.assertEqual(p.choose('running',m,b,10,1,(x,0,1),'clear',10),70)
  self.assertEqual(p.choose('running',m,b,10,1,(12.1,0,1),'unknown',10),70)
  self.assertEqual(p.choose('running',m,b,10,1,(12.1,0,1),'blocked',10),70)
  self.assertEqual(p.choose('running',m,b,10,1,(12.1,0,1),'clear',10),100)
  self.assertTrue(p.high_guard)
  raw,g=self.wire(100,False);self.assertEqual((raw[3],raw[-1]),(0,0));self.assertEqual(g.last_wire_pwm,0)
 def test_deceleration_retains_long_horizon_until_low_speed_observed(self):
  p,m,b=self.setup_policy();p.choose('running',m,b,10,1,(10,0,1),'clear',10)
  p.choose('running',m,b,49,1,(10,0,1),'clear',10.1);self.assertTrue(p.high_guard)
  for i in range(8):p.observe((i*.16,0,100+i*.1),12+i*.1)
  self.assertTrue(p.high_guard)
  for i in range(12):p.observe((1.12+i*.08,0,101+i*.1),13+i*.1)
  self.assertFalse(p.high_guard)
 def test_intermediate_lattice_finds_valid_path_without_changing_envelope(self):
  env=Envelope(1.04,.24,.77,.15);ref=[qr.Point(i,i*.5,0,0,'test','map') for i in range(33)]
  road=[(-3,-1.7),(20,-1.7),(20,1.7),(-3,1.7)]
  args=(ref,env,road,[(8,0)],lambda p:True)
  old,why=s_candidates(*args,offsets=(0,.6,-.6,1,-1,1.4,-1.4));new,nwhy=s_candidates(*args)
  self.assertIsNone(old,why);self.assertIsNotNone(new,nwhy);self.assertEqual(abs(nwhy['offset_m']),.8)

if __name__=='__main__':unittest.main()

class RuntimeSpeedTests(unittest.TestCase):
 def test_actual_runtime_hill_release_and_wire_pwm(self):
  from unittest.mock import patch
  from test_main_event_runtime import RuntimeIntegrationTests
  import main_event_runtime as rt
  original=rt.q.lst.TrialJournal.event;events=[]
  def record(journal,name,**fields):
   if name=='speed_policy':events.append(fields)
   return original(journal,name,**fields)
  with patch.object(rt.q.lst.TrialJournal,'event',record):
   code,saved,clock=RuntimeIntegrationTests().exercise(gps_period_s=.1)
  self.assertEqual(code,130,saved['reason'])
  self.assertTrue(any(e['wire_pwm']==100 for e in events),events)
  first=next(i for i,e in enumerate(events) if e['wire_pwm']==100)
  self.assertTrue(all(e['wire_pwm'] in (0,70) for e in events[:first]))
  self.assertTrue(any(e['reason']=='mission_or_signal_approach' and e['wire_pwm']==70 for e in events))
  self.assertNotIn('speed_deceleration_unconfirmed_before_mission',clock.decision_reasons)  # fixture has no S/dummy lidar scope
  print('runtime speed events:',events)
 def test_extended_real_lidar_corridor_detects_far_positive_obstacle(self):
  import math,time
  from main_event_lidar import obstacle_state
  p=json.loads((Path(__file__).resolve().parents[1]/'config/main_event_lidar_mount.json').read_text())
  scan={'mount_id':p['mount_id'],'frame':'rear_axle_FLU_m','transform_count':1,
        'received_at':1.,'complete_candidate':True,'rays':[(math.radians(i*.5),50.) for i in range(720)],'points':[(2.7,0,.2)]}
  begin=time.perf_counter()
  short=obstacle_state(scan,p,1.,[((0,0),(.8,0))]);long=obstacle_state(scan,p,1.,[((0,0),(2.4,0))])
  self.assertEqual(short,'clear');self.assertEqual(long,'blocked')
  print('real scan corridor evaluation seconds',time.perf_counter()-begin)
