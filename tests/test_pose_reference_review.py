"""Independent Codex contract checks; all coordinates are synthetic."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import pose_reference as pr


def inputs():
    return dict(x=10., y=20., heading=90., heading_convention='north_cw_deg',
                frame_id='map', frame_alignment_verified=True, reference_point='gnss_antenna',
                position_valid=True, heading_valid=True, position_age_s=0., heading_age_s=0.,
                max_age_s=.5, planar_assumption_verified=True, offset_x_m=.5, offset_y_m=.2)


class PoseContractReview(unittest.TestCase):
    def test_map_rotation_must_be_explicit(self):
        with self.assertRaises(TypeError):
            pr.rear_axle_pose(**inputs())

    def test_enu_east_is_map_north_when_axes_rotated(self):
        p = pr.rear_axle_pose(**inputs(), map_yaw_from_enu_rad=math.pi / 2)
        self.assertAlmostEqual(p.yaw, math.pi / 2)
        self.assertAlmostEqual(p.x, 10.2)
        self.assertAlmostEqual(p.y, 19.5)

    def test_enu_east_is_map_south_when_axes_rotated(self):
        p = pr.rear_axle_pose(**inputs(), map_yaw_from_enu_rad=-math.pi / 2)
        self.assertAlmostEqual(p.yaw, -math.pi / 2)
        self.assertAlmostEqual(p.x, 9.8)
        self.assertAlmostEqual(p.y, 20.5)

    def test_finite_inputs_cannot_produce_infinite_position(self):
        with self.assertRaises(ValueError):
            pr.antenna_to_rear_axle(1e308, 0., 0., -1e308, 0.)

    def test_unrepresentable_int_uses_declared_error_contract(self):
        with self.assertRaises(ValueError):
            pr.heading_to_yaw(10 ** 400, 'enu_ccw_rad')


if __name__ == '__main__':
    unittest.main()
