"""Observed ordering only, all three gates; no hardware or fabricated permission."""
import unittest
from unittest.mock import patch
from test_main_event_lidar_scope import event
from test_main_event import board
from main_event_core import Mission,Gate

class BoundaryTests(unittest.TestCase):
 def start(self,name):
  m=Mission([Gate(name,'signal',10,10.275,allowed=('green',)),Gate('next','unsupported',30,31)],'boundary')
  for i,t in enumerate((1.,1.1,1.2)):self.step(m,t,9. if i<2 else 9.8,event(t,i,name))
  self.assertIsNotNone(m.signal_pass)
  return m
 def step(self,m,t,s,e,observed=None):
  return m.step(s=s,now=t,board=board(t),quality_ok=True,heading_ok=True,aligned=True,
                signal=e,sensor_required=False,position_observed_at=observed)
 def test_all_three_ordered_late_red_and_old_target(self):
  for name in ('light_1','light_2','light_3'):
   m=self.start(name)
   r=self.step(m,1.5,10.3,event(1.45,4,name,'red'),1.4)
   self.assertTrue(r['drive']);self.assertIn(name,m.completed)
   self.assertEqual(m.last_signal_crossing['basis'],'observed_crossing_before_red')
   self.assertTrue(self.step(m,1.6,11,event(1.6,5,name,'red'),1.55)['drive'])
   print(name,'green_permission=1.2 crossed_position=1.4 red_capture=1.45 -> completed; old target ignored')
 def test_all_three_preentry_red_revokes_and_ambiguous_faults(self):
  for name in ('light_1','light_2','light_3'):
   m=self.start(name)
   self.assertFalse(self.step(m,1.5,10.,event(1.45,4,name,'red'),1.4)['drive'])
   self.assertIsNone(m.signal_pass);self.assertNotIn(name,m.completed)
   for observed in (None,1.45,1.46,.9,1.6):
    m=self.start(name)
    self.assertFalse(self.step(m,1.5,10.3,event(1.45,4,name,'red'),observed)['drive'])
    self.assertTrue(m.fault);self.assertNotIn(name,m.completed)
 def test_all_three_no_permission_stale_and_red_to_green_resume(self):
  for name in ('light_1','light_2','light_3'):
   for e in (None,event(0,1,name)):
    m=Mission([Gate(name,'signal',10,10.275,allowed=('green',))],'missing')
    self.assertFalse(self.step(m,1.,9.8,e,.9)['drive']);self.assertIsNone(m.signal_pass)
   m=Mission([Gate(name,'signal',10,10.275,allowed=('green',))],'resume')
   for i in range(80):self.step(m,i*.01,9.8,event(i*.01,i,name,'red'))
   for i,t in enumerate((.8,.9,1.)):r=self.step(m,t,9.8,event(t,100+i,name))
   self.assertTrue(r['drive']);self.assertIsNotNone(m.signal_pass)
 def test_token_expired_or_revoked_cannot_authorize_later_red(self):
  m=self.start('light_1')
  self.step(m,1.3,9.8,event(1.3,4,'light_1','red'),1.25)
  self.assertFalse(self.step(m,1.5,10.3,event(1.45,5,'light_1','red'),1.4)['drive'])
  m=self.start('light_1')
  self.assertFalse(self.step(m,5.,10.3,event(4.95,5,'light_1','red'),4.9)['drive'])
 def test_runtime_timestamp_and_crossing_journal(self):
  from test_main_event_runtime import RuntimeIntegrationTests
  import main_event_runtime as rt
  events=[];original=rt.q.lst.TrialJournal.event
  def record(journal,name,**fields):
   if name=='signal_crossing':events.append(fields)
   return original(journal,name,**fields)
  with patch.object(rt.q.lst.TrialJournal,'event',record):
   code,saved,clock=RuntimeIntegrationTests().exercise('boundary_late_red')
  self.assertEqual(code,130,saved['reason']);self.assertIn('light_1',saved['completed'])
  self.assertEqual(events[0]['basis'],'observed_crossing_before_red')
  self.assertLess(events[0]['position_observed_at'],events[0]['red_capture_at'])
  print('actual runtime boundary journal:',events)
