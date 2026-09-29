import sys,json,math,time,unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import main_event_sequence as seq,main_event_plan as plan,qualifier_route as qr,qualifier_follow as q
from main_event_core import Mission
from test_main_event import board

class SStallTests(unittest.TestCase):
 def fixture(self):
  cfg=json.loads((plan.ROOT/'config/main_event.json').read_text());prof=json.loads((plan.ROOT/'config/main_event_lidar_mount.json').read_text());b=plan.build_section('start','left',cfg)
  begin=b['labels'].index('common_1:145');end=begin+1
  while b['s'][end]-b['s'][begin]<10:end+=1
  ref=b['points'][begin:end+1];ss=qr.cumulative_s(ref);j=min(range(len(ref)),key=lambda j:abs(ss[j]-3.5));c=ref[j];h=qr.path_heading_at(ref,j)
  obs=[(c.x-math.sin(h)*(.35+v/20),c.y+math.cos(h)*(.35+v/20)) for v in range(-10,11)]
  pose=q.ps.PoseInput(ref[0].x,ref[0].y,qr.path_heading_at(ref,0));co,si=math.cos(pose.yaw),math.sin(pose.yaw)
  body=[(co*(x-pose.x)+si*(y-pose.y),-si*(x-pose.x)+co*(y-pose.y),.28) for x,y in obs]
  args=q.build_parser().parse_args(['--course','track','--track','common_1','--course-end','287','--rtk-session','unused','--output-dir','unused'])
  ev=[];state=NS(progress=begin,route=None);o=seq.Sequence(b,state,NS(),args,prof,cfg,NS(event=lambda n,**k:ev.append((n,k))))
  m=Mission(b['gates'],'S');m.index=next(i for i,g in enumerate(m.gates) if g.kind=='s_curve')
  scan={'received_at':time.monotonic(),'mount_id':prof['mount_id'],'frame':'rear_axle_FLU_m','transform_count':1,'points':body,'rays':[],'complete_candidate':True}
  return o,m,pose,scan,ev
 def test_real_worker_old_failed_geometry_is_committed_and_Mission_resumes(self):
  o,m,pose,scan,ev=self.fixture();s=o.bundle['s'][o.state.progress];start=time.monotonic()
  self.assertFalse(o.prepare(m,pose,scan,start,s,True,'blocked'))
  until=start+.29;ok=False
  while time.monotonic()<until:
   time.sleep(.003);ok=o.prepare(m,pose,scan,time.monotonic(),s,True,'blocked')
   if ok:break
  self.assertTrue(ok,ev);self.assertTrue(any(n=='planner_result' and k['fresh'] for n,k in ev))
  self.assertTrue(any(n=='s_candidate' and k.get('shape')=='quintic_entry_peak_return' for n,k in ev))
  self.assertLessEqual(o.path_until_s,m.gate.commit_s)
  m.step(s=s,now=0.,board=board(0),quality_ok=True,heading_ok=True,aligned=True,obstacle='blocked',maneuver_ok=False)
  for i in range(1,402):
   t=i*.01;r=m.step(s=s,now=t,board=board(t),quality_ok=True,heading_ok=True,aligned=True,obstacle='clear',maneuver_ok=True)
  self.assertTrue(r['drive'],r)
  before=list(o.bundle['points']);self.assertTrue(o.prepare(m,pose,{**scan,'received_at':time.monotonic()},time.monotonic(),s,True,'clear'));self.assertEqual(before,o.bundle['points'])
  # New blocked committed path: no lock early-return; wait for stop before replan.
  m.stationary.reset();self.assertFalse(o.prepare(m,pose,scan,time.monotonic(),s,False,'blocked'))
  m.stationary.stopped_since=time.monotonic()-.6
  hit=o.bundle['points'][o.state.progress+5];dx=hit.x-pose.x;dy=hit.y-pose.y;c,si=math.cos(pose.yaw),math.sin(pose.yaw)
  newer={**scan,'received_at':time.monotonic(),'points':scan['points']+[(c*dx+si*dy,-si*dx+c*dy,.28)]}
  self.assertFalse(o.prepare(m,pose,newer,time.monotonic(),s,True,'blocked'))
  self.assertIsNotNone(o.pending);self.assertTrue(any(n=='s_committed_path_blocked_replan' for n,k in ev))
  until=time.monotonic()+.29
  while time.monotonic()<until and sum(n=='planner_result' for n,k in ev)<2:
   time.sleep(.003);o.prepare(m,pose,newer,time.monotonic(),s,True,'blocked')
  results=[k for n,k in ev if n=='planner_result'];self.assertEqual(len(results),2);self.assertTrue(results[-1]['fresh'])
  print('actual Sequence/PlanWorker: old rejected case selected; Mission release; committed-path new positive return replanned:',[k.get('reason') for n,k in ev if n=='s_candidate'])
  self.assertFalse(m.step(s=s,now=5.,board=board(5.),quality_ok=True,heading_ok=True,aligned=True,obstacle='blocked',maneuver_ok=True)['drive'])
  o.worker.close()
 def test_stale_result_and_unknown_never_authorize_movement(self):
  o,m,p,scan,ev=self.fixture();s=o.bundle['s'][o.state.progress];now=time.monotonic()
  o.prepare(m,p,scan,now,s,True,'blocked');time.sleep(.08)
  self.assertFalse(o.prepare(m,p,scan,now+.5,s,True,'blocked'))
  self.assertFalse(m.step(s=s,now=now+.5,board=board(now+.5),quality_ok=True,heading_ok=True,aligned=True,obstacle='unknown',maneuver_ok=True)['drive']);o.worker.close()
