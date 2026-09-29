import sys,json,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from main_event_lidar_scope import lidar_stop_scope
from main_event_core import Mission,Gate
from main_event_speed import SpeedPolicy
import main_event_plan as plan
from test_main_event import board
CFG=json.loads((plan.ROOT/'config/main_event.json').read_text())
def event(t,i,target,sign='green'):
 return dict(source='camera',frame_id=i,target=target,epoch=0,sign=sign,confidence=.9,valid=True,capture_at=t,decision_at=t)
class ScopeTests(unittest.TestCase):
 def test_actual_HYPER_boundaries_and_departure(self):
  b=plan.build_section('start','left',CFG);g=next(g for g in b['gates'] if g.kind=='s_curve')
  for phase in ('waiting','preparing','straighten','bootstrap'):self.assertFalse(lidar_stop_scope(b,g.stop_s,phase)[0])
  self.assertFalse(lidar_stop_scope(b,g.stop_s-6.1,'running')[0]);self.assertTrue(lidar_stop_scope(b,g.stop_s-6.,'running')[0]);self.assertFalse(lidar_stop_scope(b,g.commit_s+.25,'running')[0])
  b=plan.build_section('after_t','left',CFG);a=b['s'][b['labels'].index('common_2:227')];z=b['s'][b['labels'].index('common_2:300')]
  for s,want in [(a-6.1,False),(a-6,True),(a,True),(z+.24,True),(z+.25,False)]:self.assertEqual(lidar_stop_scope(b,s,'running')[0],want)
 def test_latch_scope_exit_and_faults(self):
  m=Mission([Gate('s','s_curve',5,20)],'x')
  def step(t,obs,required,**kw):return m.step(s=10,now=t,board=board(t),quality_ok=True,heading_ok=True,aligned=True,obstacle=obs,sensor_required=required,maneuver_ok=True,**kw)
  self.assertFalse(step(0,'blocked',True)['drive']);self.assertTrue(m.obstacle_latched)
  for i in range(1,80):r=step(i*.01,'blocked',False)
  self.assertFalse(m.obstacle_latched);self.assertTrue(r['drive'])
  self.assertFalse(step(1,'unknown',True)['drive'])
  m=Mission([Gate('hill','hill',0,1,3)],'hill')
  self.assertFalse(m.step(s=0,now=0,board=board(0),quality_ok=True,heading_ok=True,aligned=True,obstacle='blocked',sensor_required=False)['drive'])
 def test_all_three_signals_red_green_loss_and_new_red(self):
  for name in ('light_1','light_2','light_3'):
   for red in (False,True):
    m=Mission([Gate(name,'signal',10,10.275,allowed=('green',))],'x')
    def step(t,s,e):return m.step(s=s,now=t,board=board(t),quality_ok=True,heading_ok=True,aligned=True,signal=e,obstacle='blocked',sensor_required=False)
    for i in range(70):r=step(i*.01,9.8,event(i*.01,i,name,'red'))
    self.assertFalse(r['drive']);self.assertEqual(m.signal_wait_detail,'red')
    for i,t in enumerate((.7,.8,.9)):r=step(t,9.8,event(t,100+i,name))
    self.assertTrue(r['drive'])
    r=step(1.7,10.3,event(1.7,110,name,'red') if red else None)
    self.assertEqual(bool(m.fault),red);self.assertEqual(name in m.completed,not red)
   m=Mission([Gate(name,'signal',10,10.275,allowed=('green',))],'none')
   self.assertFalse(m.step(s=10,now=5,board=board(5),quality_ok=True,heading_ok=True,aligned=True,signal=event(4,1,name),obstacle='unknown',sensor_required=False)['drive'])
   self.assertEqual(m.signal_wait_detail,'stale_frame')
 def test_actual_runtime_bootstrap_ignores_nonmission_lidar_only(self):
  from test_main_event_runtime import RuntimeIntegrationTests
  for kind in ('nonmission_blocked','nonmission_unknown'):
   code,saved,clock=RuntimeIntegrationTests().exercise(kind)
   self.assertEqual(clock.trial.prepares,1);self.assertIn('hill',saved['completed']);self.assertIn('light_1',saved['completed']);self.assertTrue(saved['stop_ack'])
 def test_no_observation_stays70_not100(self):
  from types import SimpleNamespace as N
  p=SpeedPolicy(CFG);p.origin=(0,0);p.speed=1.2;m=N(completed=['hill'],gate=None)
  for obs in ('blocked','unknown'):self.assertEqual(p.choose('running',m,{'section':'start'},0,1,(5,0,1),obs,1),70)
