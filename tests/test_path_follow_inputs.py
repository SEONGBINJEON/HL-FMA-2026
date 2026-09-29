import sys,unittest,tempfile,json,math
from pathlib import Path
from datetime import datetime,timezone
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tests')]
import path_follow_inputs as pi
import qualifier_route as qr
import live_path_follow as live
import live_steering_trial as lst
import steer_target_host as h
from test_live_steering_trial import Harness


def stamp(x):return datetime.fromtimestamp(x,timezone.utc).isoformat()
class Inputs(unittest.TestCase):
 def test_tail_bounded_partial_and_rotation(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'x';p.write_text(('{}\n'*10000)+'{"x":1}\n{"x":')
   s=pi.LatestRows(p,128);r=s.poll();self.assertEqual(r[-1],{'x':1});self.assertLess(len(r),50)
   with p.open('a') as f:f.write('2}\n')
   self.assertEqual(s.poll(),[{'x':2}]);p.write_text('{}\n')
   with self.assertRaises(ValueError):s.poll()
 def test_same_segment_middle_rejoin(self):
  pts=[qr.Point(i,i*.2,0,0,'t') for i in range(201)]
  i,d=pi.select_rejoin(pts,qr.Pose(22.15,.6,0),3,90)
  self.assertIn(i,(110,111));self.assertAlmostEqual(d,.6)
  with self.assertRaises(ValueError):pi.select_rejoin(pts,qr.Pose(22,.6,math.pi),3,90)
 def test_heading_stationary_does_not_refresh(self):
  h=pi.Heading(.5);h.gps(0,0,100);h.gps(1,0,101)
  self.assertTrue(h.valid(101,3));h.gps(1,0,110);self.assertFalse(h.valid(110,3))
 def test_relative_imu_and_wrap(self):
  h=pi.Heading(.5,0,100,1)
  h.imu({'host_utc':stamp(100),'valid':True,'euler_deg':[0,0,179]},100)
  h.imu({'host_utc':stamp(100.1),'valid':True,'euler_deg':[0,0,-179]},100.1)
  self.assertAlmostEqual(h.yaw,math.radians(2));self.assertTrue(h.valid(100.1,3))
  self.assertFalse(h.valid(101,3))
 def test_reference_and_asymmetric_angles(self):
  pts=[qr.Point(0,1,1,math.pi/2,'t')];out=pi.rear_axle_points(pts,'GNSS antenna position, NOT rear axle',.315)
  self.assertAlmostEqual(out[0].y,.685);self.assertEqual(pts[0].y,1)
  self.assertGreater(pi.left_wheel_cdeg(1000,.75,.67),1000)
  self.assertLess(abs(pi.left_wheel_cdeg(-1000,.75,.67)),1000)

class BoardConnection(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.lib=h.load_library()
 def test_real_adapter_gate_prepare_drive_and_latched_stop(self):
  with tempfile.TemporaryDirectory() as d:
   x=Harness(self.lib,d,adc0=600)
   try:
    gate=live.DriveGate(50,x.trial.adapter,clock=lambda:x.fake.now);x.trial.client.encoder=gate
    gate.open(0);x.run(.1);self.assertFalse(any(c.get('drive') for c in x.controls()))
    x.trial.key('p',x.ms/1000);x.run(.4);self.assertTrue(x.trial.prepared())
    self.assertTrue(x.trial.command_target(0,x.ms/1000,3));gate.open(x.ms/1000);x.run(.15)
    self.assertTrue(any(c.get('drive')==1 for c in x.controls()))
    gate.close();x.trial.stop_motion('test stop',x.ms/1000);x.run(.15)
    self.assertTrue(x.trial.stop_ack_observed)
    self.assertFalse(x.trial.command_target(0,x.ms/1000,3));gate.open(x.ms/1000);before=len(x.controls());x.run(.1)
    self.assertFalse(any(c.get('drive') for c in x.controls()[before:]))
   finally:x.close()
 def test_changing_target_stall(self):
  with tempfile.TemporaryDirectory() as d:
   x=Harness(self.lib,d,adc0=600,angle_cdeg=400)
   try:
    x.run(.5,keys={100:'p'});self.assertTrue(x.trial.prepared());x.freeze()
    for k in range(40):
     if x.trial.adapter.locked_by_fault:break
     x.trial.command_target(400+(k%2)*10,x.ms/1000,3);x.run(.05)
    self.assertIsNotNone(x.trial.stall_blocked);self.assertTrue(x.trial.adapter.locked_by_fault)
   finally:x.close()


class WholeExecutor(unittest.TestCase):
 def exercise(self, drop_quality=False, stale=False):
  from unittest.mock import patch
  import steer_target_sim as sim
  import step_manual_control as smc
  from contextlib import redirect_stdout
  import io
  with tempfile.TemporaryDirectory() as td:
   root=Path(td);route=root/'r.csv';route.write_text('idx,x,y,yaw,frame_id\n'+''.join(f'{i},{i*.25},0,0,map\n' for i in range(21)))
   route.with_suffix('.meta.json').write_text(json.dumps({'origin_lat':35.,'origin_lon':128.,'frame':'map','reference':'GNSS antenna position, NOT rear axle'}))
   stream=root/'gps';stream.mkdir();gga=stream/'gga.jsonl';gga.touch()
   lib=h.load_library();transport=sim.MockTransport();board=sim.BoardModel(lib,57,600,-1)
   class Clock:
    now=100.;last_gga=-1
    def monotonic(self):return self.now
    def time(self):return 1700000000.+self.now
    def sleep(self,dt):
     self.now+=dt;ms=int(self.now*1000)
     for raw in transport.pending_tx:board.feed(raw,ms)
     transport.pending_tx.clear();board.tick(ms);board.plant(dt)
     frame=board.status(ms)
     if frame:transport.rx+=frame
     if int(self.now*10)!=self.last_gga:
      self.last_gga=int(self.now*10);x=max(0.,self.now-100.5)
      row={'host_utc':stamp(self.time()-100 if stale else self.time()),'longitude_deg':128+x/(111320*math.cos(math.radians(35))),
           'latitude_deg':35.,'fix_status':'rtk_float' if drop_quality and self.now>101 else 'rtk_fixed',
           'checksum_valid':True,'differential_age_s':.1}
      with gga.open('a') as f:f.write(json.dumps(row)+'\n')
   clock=Clock()
   class Keys:
    sent=False
    def poll(self):
     if not self.sent and clock.now>100.3:self.sent=True;return ['p']
     return []
    def restore(self):pass
   # Real saved calibration is read only; plant is synthetic and follows scripted fixes.
   args=['--route',str(route),'--rtk-session',str(stream),'--calibration',str(ROOT/'data/stall_guard_trial_20260918_204436/drive_calibration_zero480.json'),
         '--output-dir',str(root/'out'),'--angle-model','ideal-ackermann','--initial-heading-deg','0','--max-seconds','8','--drive-pwm','50']
   try:
    with patch.object(live,'time',clock),patch.object(smc,'time',clock),patch.object(lst,'StdinKeys',Keys),patch.object(lst,'PosixSerialTransport',lambda *a,**k:transport),redirect_stdout(io.StringIO()):
     # The encoder clock must be the same injected clock as the runner.
     original_gate=live.DriveGate
     with patch.object(live,'DriveGate',lambda pwm,adapter:original_gate(pwm,adapter,clock=clock.monotonic)):
      code=live.main(args)
    metadata=json.loads((root/'out/metadata.json').read_text())
    if stale:
     self.assertNotEqual(code,0);self.assertEqual(metadata['stats']['drive_frames_sent'],0)
     self.assertEqual(metadata['stats']['accepted_targets'],0)
    else:self.assertGreater(metadata['stats']['drive_frames_sent'],0)
    if stale:pass
    elif drop_quality:
     self.assertNotEqual(code,0);self.assertEqual(metadata['status'],'failed');self.assertIn('quality',metadata['reason'])
    else:self.assertEqual(code,0,metadata);self.assertEqual(metadata['status'],'arrived')
    self.assertIsNotNone(metadata['final_stop_attempt']['ack_req'])
   finally:board.destroy()
 def test_full_loop_arrival(self):self.exercise()
 def test_full_loop_float_stops(self):self.exercise(True)
 def test_full_loop_old_gps_cannot_start(self):self.exercise(stale=True)



class ClosedLoopGeometry(unittest.TestCase):
 def test_middle_entry_with_steering_gain_error(self):
  import path_steering as ps
  for gain in (.65,1.3):
   with self.subTest(gain=gain):
    pts=[qr.Point(i,i*.2,0,0,'synthetic') for i in range(216)]
    route=qr.ModeRoute('synthetic',[qr.Segment('s',0,215,'synthetic')],pts)
    state=live.RejoinState(qr.SelectionParams(2,90,43,1.5,.5,'synthetic'));state.select_mode(route,True)
    x,y,yaw=15.,.6,0.;pose=ps.PoseInput(x,y,yaw);self.assertTrue(state.arm(pose.as_route_pose()));state.start()
    cfg=ps.PurePursuitConfig(.75,'synthetic',2,2,math.pi/2,math.radians(85),.5,.3,.5,'rear_axle_center','synthetic')
    follower=ps.PathFollower(state,cfg,lambda t:int(t*1000));arrived=False
    for step in range(1000):
     msg=follower.step(ps.PoseInput(x,y,yaw),step*.1)
     if msg.kind!='steer':arrived=msg.reached;break
     wheel=max(-1000,min(1000,pi.left_wheel_cdeg(msg.angle_cdeg,.75,.67)))
     left=math.radians(wheel/100)*gain
     curvature=math.tan(left)/(.75+.335*math.tan(left))
     x+=.06*math.cos(yaw);y+=.06*math.sin(yaw);yaw=qr.wrap(yaw+.06*curvature)
    self.assertTrue(arrived);self.assertLess(abs(y),.12)
if __name__=='__main__':unittest.main()
