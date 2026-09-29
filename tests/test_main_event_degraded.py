import json,sys,tempfile,unittest,math,socket,time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from main_event_degraded import GapContinuity,normalize_choices,core_ready
from main_event_runtime import Inputs,Workers
from main_event_finish import FinishChoice
from main_event_core import Gate
from main_event_control import Commands
from test_main_event import PROFILE,board

class DegradedTests(unittest.TestCase):
 def test_optional_worker_failures_and_missing_executable_not_essential_rtk(self):
  with tempfile.TemporaryDirectory() as tmp:
   w=Workers(tmp)
   for n in ('camera','signal','lidar'):
    w.processes.append((n,SimpleNamespace(poll=lambda:1,returncode=1)))
   self.assertEqual(len(w.check()),3)
   self.assertTrue(all(v['state']=='ended' for v in w.check().values()))
   self.assertIsNone(w.start('camera_missing',['/nonexistent/optional-python']))
   with self.assertRaises(OSError):w.start('rtk',['/nonexistent/essential-python'])
   w.processes.append(('rtk',SimpleNamespace(poll=lambda:1,returncode=1)))
   with self.assertRaisesRegex(RuntimeError,'rtk worker'):w.check()
   for h in w.handles:h.close()
 def test_optional_missing_malformed_or_wrong_mount_never_fakes_good_data(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp);(p/'lidar').mkdir();(p/'frames').mkdir()
   obj=Inputs(p,PROFILE)
   for bad in ('{','[]','{"valid":true,"capture_at":"bad"}'):
    (p/'signal.json').write_text(bad);(p/'frames/latest.json').write_text(bad)
    with patch.object(obj,'poll_lidar',side_effect=ValueError('mount mismatch')):
     self.assertEqual(obj.poll(0,0),[])
    self.assertIsNone(obj.signal);self.assertIsNone(obj.scan)
    self.assertIn('mount mismatch',obj.perception_health['lidar'])
   self.assertTrue(core_ready(board(1,state='DISARMED'),1,True,True))
   self.assertFalse(core_ready(board(1,state='DISARMED'),1,False,True))
   self.assertFalse(core_ready(board(1,state='DISARMED'),1.2,True,True))
 def test_choices_validation_and_explicit_finish_remains_locked(self):
  self.assertEqual(normalize_choices({'t':'Lft','p':'Up','finish':'Right'}),{'t':'left','p':'up','finish':'right'})
  for v in ({'t':'up'},{'p':'left'},{'finish':'random'},{'unknown':'left'},{'t':None}):
   with self.assertRaises(ValueError):normalize_choices(v)
  g=Gate('finish_fork','unsupported',100,101)
  for lane in ('left','right'):
   c=FinishChoice(0,'left');c.operator_lane=lane
   self.assertIsNone(c.update(None,0,80,g))
   choice=c.update(None,1,89,g);self.assertEqual(choice['lane'],lane)
   c.operator_lane='right' if lane=='left' else 'left'
   self.assertIs(c.update(None,50,100,g),choice)
   self.assertEqual(choice['source'],'operator_session_choice')
 def test_session_choices_only_before_start(self):
  with tempfile.TemporaryDirectory() as tmp:
   c=Commands(tmp,'r',0)
   with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as sock:
    sock.bind('\0degraded-choice-test')
    for i,phase in enumerate(('waiting','not_ready','running','bootstrap')):
     sock.sendto(json.dumps(dict(run_id='r',epoch=0,nonce=str(i),sent_at=time.monotonic(),action='choices',payload={'finish':'right'})).encode(),str(c.path))
     action=c.poll(phase);response=json.loads(sock.recv(4096))
     self.assertEqual(response['accepted'],phase in ('waiting','not_ready'))
     self.assertEqual(action is not None,phase in ('waiting','not_ready'))
   c.close()
 def test_gap_proof_rejects_jump_boot_heading_long_and_cached_fix(self):
  for kind in ('valid','jump','boot','heading','long','cached','moved'):
   c=GapContinuity();b=board(0)
   c.update((0,0,0),0,0,b,0,True,True,False,False,False)
   c.update(None,.6,0,b,0,True,True,False,False,False)
   ret=False
   for i in range(4):
    t=(4 if kind=='long' else 1)+i*.1
    f=(.4 if kind=='jump' else .1,0,1 if kind=='cached' else t)
    b=board(t,boot=8 if kind=='boot' else 7)
    ret=c.update(f,t,.1,b,.2 if kind=='heading' else 0,True,True,True,True,kind=='moved')
   self.assertEqual(ret,kind=='valid',(kind,c.evidence))


from test_steer_target_host import Base
from test_live_steering_trial import Harness
from main_event_motion import DirectionalDriveGate
from main_event_startup import enable_startup_sync
from main_event_lidar import ScanAssembler
class RunningServiceTests(Base):
 def test_actual_Client_Session_board_serviced_during_sensor_poll_drive_expires(self):
  root=Path(__file__).resolve().parents[1]
  recorded=root/'data/me_20260920_130553_0bc5cf'
  profile=json.loads((recorded/'lidar_profile.json').read_text())
  raw=(recorded/'lidar/stdout.bin').read_bytes()[:131072]
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp);h=Harness(self.lib,p,angle_cdeg=0)
   try:
    enable_startup_sync(h.trial.client);h.run(.3);h.run(.4,keys={h.ms:'p'})
    self.assertTrue(h.trial.prepared())
    drive=DirectionalDriveGate(70,h.trial.adapter,clock=lambda:h.fake.now);h.trial.client.encoder=drive
    self.assertTrue(h.trial.command_target(0,h.fake.now,3));drive.open(h.fake.now)
    h.run(.05);self.assertTrue(any(d.get('drive') for d in h.tx()))
    d=p/'input';(d/'lidar').mkdir(parents=True)
    (d/'lidar/stdout.bin').write_bytes(raw)
    (d/'lidar/metadata.json').write_text(json.dumps({'mount_id':profile['mount_id'],'transform_count':0}))
    inp=Inputs(d,profile);calls=[]
    def service():
     h.run(.005);calls.append(h.fake.now)
    inp.service_board=service
    with patch.object(inp.gga,'poll',return_value=[]),patch.object(inp.chunks,'poll',return_value=[{'stream':'stdout','offset':0,'bytes':len(raw),'host_utc':'1970-01-01T00:00:01+00:00'}]),patch('main_event_runtime.time.time',return_value=1.):
     inp.poll(0,0)
    self.assertGreater(len(calls),100)
    self.assertFalse(h.trial.adapter.locked_by_fault,h.trial.adapter.first_fault)
    self.assertEqual(h.tx()[-1].get('drive',0),0) # service never renews the drive lease
    self.assertEqual(h.tx()[-1].get('pwm',0),0)
    drive.close();h.run(.5,keys={h.ms:'q'})
    self.assertTrue(h.trial.stop_ack_observed)
    print('actual Client/Session/BoardModel sensor checkpoints',len(calls),'expired drive PWM0; Q ACK true')
   finally:h.close()

class RemainingStopTests(unittest.TestCase):
 def test_gps_age_over_half_second_is_still_unusable(self):
  from datetime import datetime,timezone
  with tempfile.TemporaryDirectory() as tmp:
   obj=Inputs(tmp,PROFILE)
   row=dict(host_utc=datetime.fromtimestamp(1,timezone.utc).isoformat(),fix_status='rtk_fixed',checksum_valid=True,differential_age_s=1,latitude_deg=37.,longitude_deg=127.)
   with patch.object(obj.gga,'poll',return_value=[row]),patch('main_event_runtime.time.time',return_value=1.1):obj.poll(37,127)
   self.assertIsNotNone(obj.fix)
   with patch.object(obj.gga,'poll',return_value=[]),patch('main_event_runtime.time.time',return_value=1.501):obj.poll(37,127)
   self.assertIsNone(obj.fix)
 def test_no_progress_five_seconds_is_not_collision_or_M_permission(self):
  from main_event_autonomy import ProgressWatch
  w=ProgressWatch();w.update(0,0,(0,0,0),board(0))
  r=w.update(5,0,(0,0,5),board(5,enc=999))
  self.assertTrue(r['no_progress_5s']);self.assertTrue(r['encoder_changed'])
  self.assertEqual(r['gps_displacement_m'],0)
  self.assertFalse(r['collision_proven']);self.assertFalse(r['manual_permission_granted']);self.assertFalse(r['timeout_grants_drive'])
 def test_all_signals_none_ten_seconds_then_green_and_dummy_twenty_seconds(self):
  from main_event_core import Mission
  from test_main_event import event
  for name in ('light_1','light_2','light_3'):
   m=Mission([Gate(name,'signal',10,10.3,allowed=('green',))],'test')
   for i in range(120):
    t=i*.1;r=m.step(s=9.6,now=t,board=board(t),quality_ok=True,heading_ok=True,aligned=True,signal=None,sensor_required=False)
    self.assertFalse(r['drive'])
   for i in range(3):
    t=12+i*.1;r=m.step(s=9.6,now=t,board=board(t),quality_ok=True,heading_ok=True,aligned=True,signal=event(t,i,target=name),sensor_required=False)
   self.assertTrue(r['drive'],r)
  m=Mission([Gate('later','hill',100,101,3)],'dummy')
  for i in range(210):
   t=i*.1;r=m.step(s=10,now=t,board=board(t),quality_ok=True,heading_ok=True,aligned=True,obstacle='blocked')
   self.assertFalse(r['drive'])
  self.assertFalse(m.step(s=10,now=21.,board=board(21),quality_ok=True,heading_ok=True,aligned=True,obstacle='unknown')['drive'])
  self.assertTrue(m.step(s=10,now=21.1,board=board(21.1),quality_ok=True,heading_ok=True,aligned=True,obstacle='clear')['drive'])

if __name__=='__main__':unittest.main()
