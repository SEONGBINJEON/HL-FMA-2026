import math,json,tempfile
from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from main_event_lidar_direction import angular_mapping,oblique_ordering,target_cluster,raw_bins

class DirectionTests(unittest.TestCase):
    def test_oblique_order_crossing_zero_does_not_invent_90deg_or_approve_yaw(self):
        data={k:{'raw_angle_deg':v} for k,v in [('front',345.),('left',20.),('right',310.)]}
        result=oblique_ordering(data,180)
        self.assertEqual(result['net_angle_gain_measured'],1)
        self.assertEqual(result['theta_sign_from_local_order'],-1)
        self.assertNotIn('yaw_deg',result)
        self.assertFalse(result['live_direction_approved'])
        data['right']['raw_angle_deg']=30.
        with self.assertRaisesRegex(ValueError,'opposite'):oblique_ordering(data,180)

    def test_inverted_mapping_is_derived_from_three_observed_directions(self):
        data={k:{'raw_angle_deg':v} for k,v in [('front',30.),('left',120.),('right',300.)]}
        result=angular_mapping(data,180)
        self.assertEqual(result['theta_sign'],-1);self.assertAlmostEqual(result['yaw_deg'],-30.)
        self.assertFalse(result['profile_updated']);self.assertFalse(result['live_direction_approved'])

    def test_mirrored_raw_axes_have_other_measured_sign(self):
        data={k:{'raw_angle_deg':v} for k,v in [('front',0.),('left',270.),('right',90.)]}
        self.assertEqual(angular_mapping(data,180)['theta_sign'],1)

    def test_ambiguous_target_is_not_guessed(self):
        baseline=[5.]*360;target=baseline.copy()
        for i in list(range(25,36))+list(range(115,126)):target[i]=1.5
        with self.assertRaisesRegex(ValueError,'ambiguous'):target_cluster(baseline,target)
        for i in range(115,126):target[i]=5.
        self.assertAlmostEqual(target_cluster(baseline,target)['raw_angle_deg'],30.)

    def test_old_mount_or_transformed_recording_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);(d/'metadata.json').write_text(json.dumps({'mount_id':'old','raw_frame':'sdk_raw_polar','transform_count':0}))
            with self.assertRaisesRegex(ValueError,'identity'):raw_bins(d,'new')
            (d/'metadata.json').write_text(json.dumps({'mount_id':'new','raw_frame':'sdk_raw_polar','transform_count':1}))
            with self.assertRaisesRegex(ValueError,'zero'):raw_bins(d,'new')

if __name__=='__main__':unittest.main()
