import json
from pathlib import Path
import sys,time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from main_event_planner_worker import PlanWorker

class WorkerTests(unittest.TestCase):
    def test_cpu_work_does_not_block_caller_and_queue_is_bounded(self):
        worker=PlanWorker()
        def slow():
            until=time.perf_counter()+.25;n=0
            while time.perf_counter()<until:n+=1
            return n
        t=time.perf_counter();self.assertTrue(worker.submit('synthetic','one',slow))
        self.assertLess(time.perf_counter()-t,.1)
        self.assertFalse(worker.submit('synthetic','two',slow))
        polls=0;gaps=[];previous=time.perf_counter();result=None
        while time.perf_counter()-t<2.:
            result=worker.poll();now=time.perf_counter();gaps.append(now-previous);previous=now;polls+=1
            if result:break
            time.sleep(.005)
        self.assertIsNotNone(result);self.assertGreater(polls,5);self.assertLess(max(gaps),.15)
        self.assertIsNone(result['error']);self.assertGreater(result['value'],0)
        print(json.dumps({'input_kind':'synthetic_cpu','polls':polls,'max_poll_gap_s':max(gaps),'worker_compute_s':result['compute_s']}))
        worker.close();self.assertFalse(worker.submit('synthetic','three',slow))

    def test_worker_error_is_data_not_a_motion_command(self):
        w=PlanWorker()
        def fail():raise ValueError('synthetic failure')
        w.submit('s','x',fail);result=None;until=time.monotonic()+1.
        while result is None and time.monotonic()<until:result=w.poll();time.sleep(.001)
        self.assertIn('synthetic failure',result['error']);self.assertIsNone(result['value']);w.close()

if __name__=='__main__':unittest.main()

from unittest.mock import patch
from types import SimpleNamespace
import qualifier_follow as q
import qualifier_route as qr
import main_event_sequence as seq
from main_event_core import Gate,Mission
from test_main_event import PROFILE
import math


class DeferredPlanner:
    def __init__(self):self.job=None;self.result=None
    def submit(self,kind,key,fn,*args):
        if self.job is not None:return False
        self.job=(kind,key,fn,args);return True
    def complete(self):
        kind,key,fn,args=self.job;self.job=None
        self.result={'kind':kind,'key':key,'value':fn(*args),'error':None,'compute_s':.02}
    def poll(self):r=self.result;self.result=None;return r
    def close(self):pass


class ResultBindingTests(unittest.TestCase):
    def exercise(self,delay=0.,dx=0.,yaw=0.):
        points=[qr.Point(i,i*.5,0.,0.,'synthetic','map') for i in range(41)]
        config={'s_avoidance_verified':True,'road_boundaries':{'s_curve':{'verified':True,'evidence':'synthetic','frame':'map',
            'polygon':[[-10,-10],[50,-10],[50,10],[-10,10]]}}}
        gate=Gate('s_curve','s_curve',5.,15.)
        bundle={'points':points,'labels':[str(i) for i in range(41)],'s':qr.cumulative_s(points),'gates':[gate],
                'section':'start','config':config}
        state=SimpleNamespace(progress=0,route=None)
        args=q.build_parser().parse_args(['--course','track','--track','common_1','--course-end','287','--rtk-session','unused','--output-dir','unused'])
        events=[];journal=SimpleNamespace(event=lambda name,**kw:events.append((name,kw)))
        scan={'received_at':1.,'complete_candidate':True,'rays':[(math.radians(i*.5),50.) for i in range(720)],'points':[]}
        with patch.object(seq,'PlanWorker',DeferredPlanner):obj=seq.Sequence(bundle,state,SimpleNamespace(),args,PROFILE,config,journal)
        mission=Mission([gate],'test');pose=q.ps.PoseInput(0.,0.,0.)
        self.assertFalse(obj.prepare(mission,pose,scan,1.,0.,False))
        obj.worker.complete()
        allowed=obj.prepare(mission,q.ps.PoseInput(dx,0.,yaw),scan,1.+delay,dx,False)
        return allowed,events,obj

    def test_fresh_result_commits_actual_planner_output(self):
        allowed,events,obj=self.exercise(.1)
        self.assertTrue(allowed);self.assertTrue(obj.s_ok)
        self.assertTrue(next(data for name,data in events if name=='planner_result')['fresh'])

    def test_stale_result_never_moves_or_installs(self):
        allowed,events,obj=self.exercise(.31)
        self.assertFalse(allowed);self.assertFalse(obj.s_ok)
        self.assertFalse(next(data for name,data in events if name=='planner_result')['fresh'])

    def test_position_or_heading_change_discards_old_snapshot(self):
        for dx,yaw in [(.26,0.),(0.,math.radians(6.))]:
            allowed,events,obj=self.exercise(.1,dx,yaw)
            self.assertFalse(allowed);self.assertFalse(obj.s_ok)
