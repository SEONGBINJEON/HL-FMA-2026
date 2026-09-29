"""Captured RX replay + synthetic faults; no serial devices."""
import json,sys,struct,tempfile,unittest
from pathlib import Path
from dataclasses import replace,astuple
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from test_steer_target_host import Base
import steer_target_host as h
from step_manual_control import Client
from joystick_protocol import Parser,PAYLOAD,encode_command
from step_manual_protocol import DISARM,crc16
from main_event_startup import enable_startup_sync
from main_event_menu import Menu
ROOT=Path(__file__).resolve().parents[1]
SESSIONS=['me_20260920_121559_a85be4','me_20260920_122056_249e26']
def frame(v):
 body=bytes([150,PAYLOAD.size])+PAYLOAD.pack(*astuple(v));return b'\xf1\xa5'+body+struct.pack('<H',crc16(body))
class Transport:
 def __init__(self):self.rx=b'';self.tx=[];self.closed=False
 @property
 def in_waiting(self):return len(self.rx)
 out_waiting=0
 def read(self,n):v=self.rx[:n];self.rx=self.rx[n:];return v
 def write(self,b):self.tx.append(b);return len(b)
 def close(self):self.closed=True
class Journal:
 def __init__(self):self.rows=[]
 def event(self,n,**kw):self.rows.append((n,kw))
 def bytes(self,*a):pass
class StartupTests(Base):
 def client(self,sync):
  ad=h.HostSteerAdapter(h.TargetCore(self.lib,self.cfg));t=Transport();j=Journal();c=Client(t,ad,j,encoder=lambda cmd:encode_command(cmd,0));c.parser=Parser()
  if sync:enable_startup_sync(c)
  return c,t,ad,j
 def tick(self,c,t,now,raw=b''):
  t.rx+=raw
  with patch('step_manual_control.time.monotonic',return_value=now):c.tick(now)
 def records(self,name):return [r for r in map(json.loads,(ROOT/'data'/name/'vehicle/events.jsonl').read_text().splitlines()) if r['event']=='rx']
 def test_exact_first_rx_reproduces_original_lock_both(self):
  for name in SESSIONS:
   row=self.records(name)[0];c,t,ad,j=self.client(False)
   self.tick(c,t,row['host_monotonic'],bytes.fromhex(row['hex']))
   self.assertTrue(ad.locked_by_fault);self.assertIn('요청 번호 역행',ad.first_fault)
   print('original reproduced',name,'bytes',row['count'],'time',row['host_monotonic'],'fault',ad.first_fault)
 def test_actual_captured_rx_timing_new_disarm_sync_both(self):
  for name in SESSIONS:
   rows=self.records(name);c,t,ad,j=self.client(True);start=rows[0]['host_monotonic'];clock=start
   for row in rows:
    if row['host_monotonic']-start>1.:break
    while clock+.005<row['host_monotonic']:
     clock+=.005;self.tick(c,t,clock)
    clock=row['host_monotonic'];self.tick(c,t,clock,bytes.fromhex(row['hex']))
    if c.parser.done:break
   self.assertTrue(c.parser.done,(name,c.parser.snapshot(),ad.first_fault))
   self.assertFalse(ad.locked_by_fault);self.assertTrue(ad.identified);self.assertEqual(ad.status.state,0)
   self.assertTrue(all(b[12]==DISARM and b[13:17]==b'\x00'*4 for b in t.tx))
   print('captured sync',name,'seconds',clock-start,c.parser.snapshot(),'tx_DISARM_only',len(t.tx))
 def acquire(self):
  row=self.records(SESSIONS[0])[0];values,_=Parser().feed(bytes.fromhex(row['hex']),0);v=max(values,key=lambda x:x.request_id)
  c,t,ad,j=self.client(True);self.tick(c,t,0,bytes.fromhex(row['hex']))
  for i in (1,2):
   v=replace(v,request_id=v.request_id+1,millis=v.millis+50,state=0,reason=1)
   self.tick(c,t,i*.05,frame(v))
  self.assertTrue(c.parser.done);self.assertFalse(ad.locked_by_fault)
  return c,t,ad,v
 def test_post_sync_order_boot_crc_and_deadline_are_strict(self):
  for kind in ('reverse','boot','crc','partial','stale'):
   c,t,ad,v=self.acquire()
   if kind=='reverse':raw=frame(replace(v,request_id=v.request_id-1))
   elif kind=='boot':raw=frame(replace(v,boot_id=v.boot_id+1,request_id=v.request_id+1))
   elif kind=='crc':raw=frame(v)[:-1]+bytes([frame(v)[-1]^1])
   elif kind=='partial':
    self.tick(c,t,.11,frame(replace(v,request_id=v.request_id+1))[:10])
    for i in range(12,28):self.tick(c,t,i*.01)
    raw=b''
   else:
    for i in range(11,27):self.tick(c,t,i*.01)
    raw=b''
   self.tick(c,t,.27 if kind in ('stale','partial') else .15,raw)
   self.assertTrue(ad.locked_by_fault,kind);print('post_sync_guard',kind,ad.first_fault)
 def test_duplicate_cannot_complete_sync_or_renew_freshness(self):
  row=self.records(SESSIONS[0])[0];v=max(Parser().feed(bytes.fromhex(row['hex']),0)[0],key=lambda x:x.request_id)
  c,t,ad,j=self.client(True)
  for i in range(260):self.tick(c,t,i*.01,frame(v))
  self.assertFalse(c.parser.done);self.assertTrue(ad.locked_by_fault);self.assertIn('startup_sync_timeout',ad.first_fault)
 def test_partial_initial_then_fresh_disarm_is_required(self):
  row=self.records(SESSIONS[0])[0];v=max(Parser().feed(bytes.fromhex(row['hex']),0)[0],key=lambda x:x.request_id)
  c,t,ad,j=self.client(True);self.tick(c,t,0,frame(v)[:10]);self.tick(c,t,.04,frame(v)[10:]);self.assertFalse(c.parser.done)
  for i in range(3):self.tick(c,t,.05+i*.05,frame(replace(v,request_id=v.request_id+i+1,millis=v.millis+50*(i+1),reason=1)))
  self.assertTrue(c.parser.done)
 def test_menu_shows_lock_and_R_scope_without_ENOENT(self):
  with tempfile.TemporaryDirectory() as tmp:
   state=dict(session=tmp,phase='waiting',heartbeat_fresh=True,ready=False,board_locked=True,board_first_fault='요청 번호 역행',board_adapter_phase='locked',stop_ack=True,stationary_observed=True,process_released=True)
   lines=[];menu=Menu(read=lambda:state,emit=lines.append);menu.show()
   self.assertIn('요청 번호 역행',' '.join(lines))
   with self.assertRaisesRegex(ValueError,'R은 실제 S충돌'):menu.resume()

 def test_real_Client_Session_mock_board_prepare_Q_and_ARM_regression(self):
  from test_live_steering_trial import Harness
  for fault in (False,True):
   with tempfile.TemporaryDirectory() as temp:
    hs=Harness(self.lib,temp)
    try:
     enable_startup_sync(hs.trial.client)
     hs.run(.3);self.assertTrue(hs.trial.client.parser.done);self.assertFalse(hs.trial.adapter.locked_by_fault)
     self.assertTrue(all(d['action']==DISARM for d in hs.tx()))
     hs.run(.4,keys={hs.ms:'p'});self.assertTrue(hs.trial.prepared())
     self.assertTrue(all(not d.get('drive') and not d.get('steer') for d in hs.tx()))
     if fault:
      v=hs.trial.adapter.status
      hs.transport.rx+=frame(replace(v,request_id=v.request_id-1))
      hs.run(.01);self.assertTrue(hs.trial.adapter.locked_by_fault)
      self.assertIn('요청 번호 역행',hs.trial.adapter.first_fault)
     hs.run(.3,keys={hs.ms:'q'})
     self.assertTrue(hs.trial.stop_ack_observed)
     print('real Client/Session board model initial-sync/explicit prepare/Q', 'ARM regression protected' if fault else 'normal','stop_ack',hs.trial.stop_ack_observed)
    finally:hs.close()
 def test_captured_remaining_errors_still_fault_after_sync(self):
  for name in SESSIONS:
   rows=self.records(name);c,t,ad,j=self.client(True);start=rows[0]['host_monotonic'];clock=start;fault_at=None
   for row in rows:
    if row['host_monotonic']-start>10.:break
    while clock+.005<row['host_monotonic']:
     clock+=.005;self.tick(c,t,clock)
    clock=row['host_monotonic'];self.tick(c,t,clock,bytes.fromhex(row['hex']))
    if ad.locked_by_fault:fault_at=clock-start;break
   self.assertTrue(c.parser.done)
   print('captured post-sync first fault within10s:',name,fault_at,ad.first_fault)
   if fault_at is not None:self.assertTrue(ad.locked_by_fault)

 def test_runtime_waiting_lock_terminates_and_preserves_reason(self):
  from test_main_event_runtime import RuntimeIntegrationTests
  code,saved,clock=RuntimeIntegrationTests().exercise('waiting_serial_fault')
  self.assertEqual(code,1);self.assertFalse(clock.started);self.assertTrue(saved['stop_ack'])
  self.assertIn('startup_board_locked',saved['reason']);self.assertIn('요청 번호 역행',saved['board_first_fault'])
  self.assertTrue(saved['process_released']);self.assertEqual(saved['phase'],'stopped')
 def test_initial_boot_change_is_not_cleared(self):
  row=self.records(SESSIONS[0])[0];v=max(Parser().feed(bytes.fromhex(row['hex']),0)[0],key=lambda x:x.request_id)
  c,t,ad,j=self.client(True);self.tick(c,t,0,frame(v))
  self.tick(c,t,.05,frame(replace(v,boot_id=v.boot_id+1,request_id=v.request_id+1)))
  self.assertFalse(c.parser.done);self.assertTrue(ad.locked_by_fault);self.assertEqual(ad.first_fault,'startup_board_reboot')
