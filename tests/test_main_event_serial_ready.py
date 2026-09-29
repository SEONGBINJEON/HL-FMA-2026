import json,sys,unittest
from pathlib import Path
from dataclasses import replace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from test_main_event_startup import StartupTests,frame,SESSIONS
from joystick_protocol import Parser
from main_event_serial import SerialFrames
from main_event_start_left import prepare_left
from main_event_menu import Menu
class SerialReadyTests(StartupTests):
 def replay(self,name,old=False):
  rows=self.records(name);c,t,ad,j=self.client(True)
  if old:c.parser.parser=Parser()
  start=rows[0]['host_monotonic'];clock=start
  for row in rows:
   if row['host_monotonic']-start>12.:break
   while clock+.005<row['host_monotonic']:
    clock+=.005;self.tick(c,t,clock)
   clock=row['host_monotonic'];self.tick(c,t,clock,bytes.fromhex(row['hex']))
   if ad.locked_by_fault:break
  return c,t,ad,j,clock-start
 def test_new_actual_failure_before_and_after(self):
  name='me_20260920_125339_8a5c80'
  c,t,ad,j,elapsed=self.replay(name,True)
  self.assertIn('Partial status expired',ad.first_fault);print('original125339 failure',elapsed,ad.first_fault)
  for name in SESSIONS+[name]:
   c,t,ad,j,elapsed=self.replay(name)
   self.assertFalse(ad.locked_by_fault,(name,elapsed,ad.first_fault))
   self.assertTrue(c.parser.done);self.assertTrue(all(b[12:17]==b'\x00'*5 for b in t.tx))
   print('new replay',name,elapsed,c.parser.snapshot(),'TX_DISARM_only',len(t.tx))
 def test_late_fragment_alone_never_refreshes_then_new_frame_releases_order(self):
  c,t,ad,v=self.acquire();stamp=ad.last_status_at
  a=replace(v,request_id=v.request_id+1,millis=v.millis+50)
  b=replace(a,request_id=a.request_id+1,millis=a.millis+50)
  self.tick(c,t,.11,frame(a)[:15]);self.tick(c,t,.17,frame(a)[15:])
  self.assertEqual(ad.last_status_at,stamp);self.assertFalse(ad.locked_by_fault)
  self.tick(c,t,.19,frame(b));self.assertEqual(ad.last_status_at,.19)
  self.assertEqual(ad.status.request_id,b.request_id);self.assertFalse(ad.locked_by_fault)
 def test_corrupt_late_frame_and_late_reverse_still_fault(self):
  for kind in ('crc','reverse','boot'):
   c,t,ad,v=self.acquire();a=replace(v,request_id=v.request_id+1,millis=v.millis+50)
   if kind=='reverse':a=replace(a,request_id=v.request_id-1)
   if kind=='boot':a=replace(a,boot_id=v.boot_id+1)
   raw=frame(a)
   if kind=='crc':raw=raw[:-1]+bytes([raw[-1]^1])
   b=replace(v,request_id=v.request_id+2,millis=v.millis+100)
   self.tick(c,t,.11,raw[:15]);self.tick(c,t,.17,raw[15:]+frame(b))
   self.assertTrue(ad.locked_by_fault,kind)
 def test_left_prepare_order_and_archive_failure_no_prepare(self):
  actions=[];menu=Menu(execute=lambda a:(actions.append(a) or (0,'')),emit=lambda x:None)
  with patch('main_event_start_left.verify',side_effect=lambda:actions.append('verify')),patch('main_event_start_left.archive_active',side_effect=lambda:actions.append('archive')):
   prepare_left(menu)
  self.assertEqual(actions,['verify','archive',['prepare','--live','--lane','left']])
  actions.clear()
  with patch('main_event_start_left.verify'),patch('main_event_start_left.archive_active',side_effect=ValueError('unconfirmed')):
   with self.assertRaises(ValueError):prepare_left(menu)
  self.assertEqual(actions,[])
