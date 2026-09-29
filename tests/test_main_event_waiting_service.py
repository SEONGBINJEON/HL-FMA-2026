import json,sys,unittest,tempfile
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from main_event_runtime import Inputs
from main_event_plan import ROOT
from main_event_lidar import ScanAssembler
class WaitingServiceTests(unittest.TestCase):
 def test_sensor_bytes_and_times_identical_with_serial_service(self):
  recorded=ROOT/'data/me_20260920_130553_0bc5cf'
  profile=json.loads((recorded/'lidar_profile.json').read_text())
  raw=(recorded/'lidar/stdout.bin').read_bytes()[:262144]
  baseline=ScanAssembler(profile,profile['mount_id']).feed(raw,10.)
  self.assertTrue(baseline)
  with tempfile.TemporaryDirectory() as tmp:
   d=Path(tmp);(d/'lidar').mkdir();(d/'rtk').mkdir()
   (d/'lidar/stdout.bin').write_bytes(raw)
   (d/'lidar/metadata.json').write_text(json.dumps({'mount_id':profile['mount_id'],'transform_count':0}))
   obj=Inputs(d,profile);calls=[];obj.service_board=lambda:calls.append('serial')
   with patch.object(obj.gga,'poll',return_value=[]),patch.object(obj.chunks,'poll',return_value=[{'stream':'stdout','offset':0,'bytes':len(raw),'host_utc':'1970-01-01T00:00:10+00:00'}]),patch('main_event_runtime.time.monotonic',return_value=10.),patch('main_event_runtime.time.time',return_value=10.):
    obj.poll(0,0)
   self.assertEqual(obj.scan,baseline[-1]);self.assertGreaterEqual(len(calls),len(raw)//1024)
   print('same raw bytes/receipt/transform:',len(raw),'serial checkpoints',len(calls))

 def test_prepare_vision_separate_live_stream_never_grants_stale_signal(self):
  from types import SimpleNamespace
  from main_event_runtime import preparation_vision_ready
  from main_event_core import SignalVotes
  ev=dict(valid=True,source='camera',capture_at=9.3,decision_at=9.7,target='light_1',epoch=0,frame_id=1,sign='green',confidence=.9)
  inp=SimpleNamespace(signal=ev,camera_frame=dict(input_kind='live',source='camera',capture_at=9.9))
  self.assertTrue(preparation_vision_ready(inp,10.))
  votes=SignalVotes('light_1',0);self.assertFalse(votes.update(ev,10.,('green',)))
  for key,value in [('capture_at',9.),('source','other'),('input_kind','recorded')]:
   old=inp.camera_frame[key];inp.camera_frame[key]=value
   self.assertFalse(preparation_vision_ready(inp,10.));inp.camera_frame[key]=old
  ev['valid']=False
  self.assertTrue(preparation_vision_ready(inp,10.))  # original9.7 evidence, never refreshed
  inp.camera_frame['capture_at']=10.8
  self.assertFalse(preparation_vision_ready(inp,10.8))
  self.assertEqual(inp.prepare_last_valid_inference['decision_at'],9.7)
