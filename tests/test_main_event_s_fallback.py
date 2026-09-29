import json,math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import main_event_sequence as seq
import qualifier_follow as q
import qualifier_route as qr
from main_event_core import Gate,Mission
from main_event_lidar import obstacle_state
from test_main_event import PROFILE,board
from test_main_event_planner_worker import DeferredPlanner


class SReferenceFallbackTests(unittest.TestCase):
    def make(self,side=1,boundary=True):
        points=[qr.Point(i,i*.5,0.,0.,'synthetic_reference','map') for i in range(41)]
        config={'candidate_reference_mode':True,'road_boundaries':{}}
        if boundary:config['road_boundaries']['s_curve']={'verified':False,'basis_kind':'hyper_aerial_reference',
            'evidence':'SYNTHETIC TEST POLYGON, not real aerial boundary','frame':'map',
            'polygon':[[-3,-4],[25,-4],[25,4],[-3,4]]}
        gate=Gate('s_curve','s_curve',5.,18.)
        bundle={'points':points,'labels':[str(i) for i in range(41)],'s':qr.cumulative_s(points),
                'gates':[gate],'section':'start','config':config}
        state=SimpleNamespace(progress=0,route=None)
        args=q.build_parser().parse_args(['--course','track','--track','common_1','--course-end','287','--rtk-session','unused','--output-dir','unused'])
        self.events=[];journal=SimpleNamespace(event=lambda name,**kw:self.events.append((name,kw)))
        with patch.object(seq,'PlanWorker',DeferredPlanner):obj=seq.Sequence(bundle,state,SimpleNamespace(),args,PROFILE,config,journal)
        scan={'received_at':1.,'mount_id':PROFILE['mount_id'],'frame':'rear_axle_FLU_m','transform_count':1,
              'complete_candidate':True,'rays':[],'points':[(5.,side*.35,.28)]}
        return obj,Mission([gate],'S-test'),q.ps.PoseInput(0.,0.,0.),scan

    def test_positive_ahead_obstacle_selects_opposite_side_and_keeps_committed_detour(self):
        for side in (-1,1):
            obj,mission,pose,scan=self.make(side)
            self.assertFalse(obj.prepare(mission,pose,scan,1.,0.,False,'clear'))
            obj.worker.complete()
            self.assertTrue(obj.prepare(mission,pose,scan,1.1,0.,False,'clear'))
            selected=next(v for n,v in self.events if n=='s_candidate')
            self.assertLess(selected['offset_m']*side,0)
            self.assertIsNotNone(obj.s_lock_end)
            committed=list(obj.bundle['points'])
            other={**scan,'received_at':1.2,'points':[(5.,-side*.35,.28)]}
            self.assertFalse(obj.prepare(mission,pose,other,1.2,0.,False,'blocked'))
            self.assertEqual(obj.bundle['points'],committed)
            self.assertFalse(mission.step(s=5.,now=1.2,board=board(1.2),quality_ok=True,heading_ok=True,
                             aligned=True,obstacle='blocked',maneuver_ok=True)['drive'])

    def test_selected_future_geometry_does_not_make_unobserved_motion_clear(self):
        obj,mission,pose,scan=self.make()
        obj.prepare(mission,pose,scan,1.,0.,False,'clear');obj.worker.complete()
        self.assertTrue(obj.prepare(mission,pose,scan,1.1,0.,False,'clear'))
        self.assertEqual(obstacle_state(scan,PROFILE,1.1),'unknown')

    def test_missing_road_reference_stops_for_manual_fallback(self):
        obj,mission,pose,scan=self.make(boundary=False)
        self.assertFalse(obj.prepare(mission,pose,scan,1.,0.,False,'blocked'))
        self.assertIn('no_autonomous_detour',obj.reason);self.assertIsNone(obj.worker.job)


if __name__=='__main__':unittest.main()
