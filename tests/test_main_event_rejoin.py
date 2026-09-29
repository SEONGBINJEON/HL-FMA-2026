import math
from pathlib import Path
import sys
import unittest
import json
from unittest.mock import patch
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import qualifier_route as qr
import qualifier_follow as q
from main_event_rejoin import candidate_window,recovery_ceiling,propose,commit,valid_anchor

class RejoinTests(unittest.TestCase):
    def test_actual_hyper_s_middle_keep_uses_completion_boundary(self):
        from main_event_plan import build_section
        root=Path(__file__).resolve().parents[1]
        config=json.loads((root/'config/main_event.json').read_text())
        bundle=build_section('start','left',config)
        gate=next(g for g in bundle['gates'] if g.name=='s_curve')
        progress=bundle['labels'].index('common_1:161')
        ceiling=recovery_ceiling(bundle,progress,[gate])
        self.assertEqual(ceiling,gate.commit_s)
        allowed=candidate_window(bundle,progress,len(bundle['points'])-1,ceiling)
        self.assertIn(progress,allowed)
        self.assertTrue(all(bundle['s'][i]<gate.commit_s for i in allowed))
        self.assertEqual(recovery_ceiling(bundle,0,[gate]),gate.stop_s)
        signal=next(g for g in bundle['gates'] if g.name=='light_1')
        self.assertEqual(recovery_ceiling(bundle,progress,[signal]),signal.stop_s)

    def setUp(self):
        self.bundle={'points':[qr.Point(i,float(i),0.,0.,'synthetic','map') for i in range(30)],
                     'labels':[str(i) for i in range(30)],'s':list(map(float,range(30)))}
        self.args=q.build_parser().parse_args(['--course','track','--track','common_1','--course-end','287','--rtk-session','unused','--output-dir','unused'])

    def test_candidate_is_local_and_cannot_cross_unfinished_mission(self):
        indices=candidate_window(self.bundle,10,29,15.)
        self.assertEqual(indices,list(range(7,15)))
        result=propose(self.bundle,indices,11.315,0.)
        self.assertIn(result['proposed_index'],indices)
        self.assertEqual(propose(self.bundle,indices,25.,0.)['proposed_index'],None)

    def test_last_point_of_directional_window_is_not_next_gear_segment(self):
        indices=candidate_window(self.bundle,9,10,29.)
        self.assertIn(9,indices)
        self.assertNotIn(10,indices)  # segment10 runs from point10 to point11

    def test_preparation_20_seconds_does_not_refresh_old_measurement(self):
        with self.assertRaisesRegex(ValueError,'expired'):
            commit(self.bundle,range(7,15),(11.315,0.,120.),0.,100.,120.,self.args,1)
        fresh=commit(self.bundle,range(7,15),(11.315,0.,120.),0.,120.,120.,self.args,1)
        self.assertIn(fresh['index'],range(7,15));self.assertAlmostEqual(fresh['body_yaw'],0.)

    def test_missing_body_yaw_never_uses_route_tangent(self):
        with self.assertRaisesRegex(ValueError,'BODY'):
            commit(self.bundle,range(7,15),(11.315,0.,120.),None,120.,120.,self.args,1)

    def test_position_fix_and_window_are_required(self):
        for fix,index in [(None,None),((11.315,0.,119.),None),((11.315,0.,120.),25)]:
            with self.assertRaises(ValueError):commit(self.bundle,range(7,15),fix,0.,120.,120.,self.args,1,index)

    def test_unchanged_encoder_boot_and_position_required_until_start(self):
        rec={'pose_pending':False,'observed_wall':100.,'boot':7,'encoder':20,'antenna_xy':[1.,0.]}
        b={'boot':7,'encoder':20}
        self.assertTrue(valid_anchor(rec,b,(1.,0.,100.),100.))
        for board,fix,t in [({'boot':8,'encoder':20},(1.,0.,100.),100.),({'boot':7,'encoder':21},(1.,0.,100.),100.),
                             (b,(1.3,0.,100.),100.),(b,(1.,0.,116.),116.)]:self.assertFalse(valid_anchor(rec,board,fix,t))

class LocalizationReplyTests(unittest.TestCase):
    def test_old_ready_or_error_cannot_acknowledge_new_localization(self):
        import main_event
        clock=SimpleNamespace(t=100.)
        states=[{'updated_at':100.,'sensors_ready':True,'phase':'waiting'},
                {'localization_nonce':'old','localization_error':'old failure','recovery_pose_ready':True},
                {'localization_nonce':'new','recovery_pose_ready':True}]
        args=SimpleNamespace(body_heading_deg=0.,index=None,heading_observed_wall=100.)
        with patch.object(Path,'read_text',side_effect=[json.dumps(s) for s in states]) as read, \
             patch.object(main_event,'send',return_value={'accepted':True,'nonce':'new'}), \
             patch.object(main_event.time,'monotonic',side_effect=lambda:clock.t), \
             patch.object(main_event.time,'sleep',side_effect=lambda dt:setattr(clock,'t',clock.t+dt)):
            self.assertEqual(main_event.finish_recovery(args,Path('synthetic')),0)
            self.assertEqual(read.call_count,3)

if __name__=='__main__':unittest.main()
