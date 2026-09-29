#!/usr/bin/env python3
"""C4 tests for tools/pose_reference.py (revision 1). SYNTHETIC values only: offsets, ages,
thresholds and the map rotation angles here are simple geometry examples, not measurements of
the vehicle or of the HYPER map. map_yaw_from_enu_rad=0.0 in the synthetic kwargs is an explicit
test choice, not a claim that the real map axes are ENU. Expected numbers were worked out by
hand (rotate an FLU offset by 0/+90/-90/180 deg and subtract). No hardware, no file/network side
effects. Does not re-run the existing test suites.
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import pose_reference as pr  # noqa: E402
import path_steering as ps   # noqa: E402


def _good_kwargs(**over):
    """A valid synthetic call: antenna at (10, 20), heading east (compass 90 deg), antenna 0.5 m
    ahead of the rear axle, fresh ages, 0.5 s threshold, map axes declared equal to ENU (0.0,
    explicit synthetic choice)."""
    kw = dict(x=10.0, y=20.0, heading=90.0, heading_convention="north_cw_deg",
              map_yaw_from_enu_rad=0.0, frame_id="map",
              frame_alignment_verified=True, reference_point="gnss_antenna", position_valid=True,
              heading_valid=True, position_age_s=0.10, heading_age_s=0.30, max_age_s=0.5,
              planar_assumption_verified=True, offset_x_m=0.5, offset_y_m=0.0)
    kw.update(over)
    return kw


class TestHeadingToYaw(unittest.TestCase):
    def test_compass_cardinals(self):
        self.assertAlmostEqual(pr.heading_to_yaw(0.0, "north_cw_deg"), math.pi / 2)     # north -> +y
        self.assertAlmostEqual(pr.heading_to_yaw(90.0, "north_cw_deg"), 0.0)            # east  -> +x
        self.assertAlmostEqual(pr.heading_to_yaw(180.0, "north_cw_deg"), -math.pi / 2)  # south -> -y
        west = pr.heading_to_yaw(270.0, "north_cw_deg")                                 # west  -> -x
        self.assertTrue(-math.pi <= west < math.pi)
        self.assertAlmostEqual(math.cos(west), -1.0)
        self.assertAlmostEqual(math.sin(west), 0.0)

    def test_compass_wraps_and_ints(self):
        self.assertAlmostEqual(pr.heading_to_yaw(450, "north_cw_deg"), 0.0)     # 450 = 90 + 360
        # -90 compass = west = 180 deg ENU = pi, which the half-open interval maps to -pi
        y = pr.heading_to_yaw(-90, "north_cw_deg")
        self.assertAlmostEqual(y, -math.pi)
        self.assertTrue(-math.pi <= y < math.pi)
        self.assertAlmostEqual(math.cos(y), -1.0)
        self.assertAlmostEqual(math.sin(y), 0.0)

    def test_enu_passthrough_and_normalisation(self):
        self.assertAlmostEqual(pr.heading_to_yaw(0.3, "enu_ccw_rad"), 0.3)
        self.assertAlmostEqual(pr.heading_to_yaw(-math.pi / 2, "enu_ccw_rad"), -math.pi / 2)
        self.assertAlmostEqual(pr.heading_to_yaw(3 * math.pi / 2, "enu_ccw_rad"), -math.pi / 2)
        self.assertAlmostEqual(pr.heading_to_yaw(2 * math.pi + 0.1, "enu_ccw_rad"), 0.1)
        self.assertAlmostEqual(pr.heading_to_yaw(math.pi, "enu_ccw_rad"), -math.pi)   # half-open interval
        self.assertAlmostEqual(pr.heading_to_yaw(-math.pi, "enu_ccw_rad"), -math.pi)

    def test_range_always_half_open(self):
        for v in (-10.0, -7.0, -math.pi, -1e-12, 0.0, 1e-12, math.pi, 7.0, 100.0):
            y = pr.heading_to_yaw(v, "enu_ccw_rad")
            self.assertTrue(-math.pi <= y < math.pi, (v, y))

    def test_rejects_bad_values(self):
        for bad in (True, False, float("nan"), float("inf"), -float("inf"), "90", None, [90]):
            with self.assertRaises(ValueError, msg=repr(bad)):
                pr.heading_to_yaw(bad, "north_cw_deg")
            with self.assertRaises(ValueError, msg=repr(bad)):
                pr.heading_to_yaw(bad, "enu_ccw_rad")

    def test_rejects_unrepresentable_int_as_value_error(self):
        # an int too large for a float must follow the ValueError contract, not leak OverflowError
        for bad in (10 ** 400, -(10 ** 400)):
            with self.assertRaises(ValueError):
                pr.heading_to_yaw(bad, "enu_ccw_rad")
            with self.assertRaises(ValueError):
                pr.heading_to_yaw(bad, "north_cw_deg")

    def test_rejects_unsupported_convention(self):
        for conv in ("deg", "rad", "ENU_CCW_RAD", "north_ccw_deg", "enu_cw_deg", "", None, 0, True):
            with self.assertRaises(ValueError, msg=repr(conv)):
                pr.heading_to_yaw(90.0, conv)


class TestAntennaToRearAxle(unittest.TestCase):
    def test_yaw_zero_forward_offset(self):
        rx, ry = pr.antenna_to_rear_axle(10.0, 20.0, 0.0, 0.5, 0.0)
        self.assertAlmostEqual(rx, 9.5)
        self.assertAlmostEqual(ry, 20.0)

    def test_yaw_plus_90_forward_offset(self):
        # heading +y: antenna 0.5 m ahead means 0.5 m further in +y; rear axle is below it
        rx, ry = pr.antenna_to_rear_axle(10.0, 20.0, math.pi / 2, 0.5, 0.0)
        self.assertAlmostEqual(rx, 10.0)
        self.assertAlmostEqual(ry, 19.5)

    def test_yaw_minus_90_forward_offset(self):
        rx, ry = pr.antenna_to_rear_axle(10.0, 20.0, -math.pi / 2, 0.5, 0.0)
        self.assertAlmostEqual(rx, 10.0)
        self.assertAlmostEqual(ry, 20.5)

    def test_lateral_offset_yaw_zero(self):
        # antenna 0.5 m forward and 0.2 m left (+y) of the rear axle
        rx, ry = pr.antenna_to_rear_axle(10.0, 20.0, 0.0, 0.5, 0.2)
        self.assertAlmostEqual(rx, 9.5)
        self.assertAlmostEqual(ry, 19.8)

    def test_lateral_offset_yaw_plus_90(self):
        # heading +y: forward = +y, left = -x. antenna = (10 - 0.2, 20 + 0.5) -> rear = (10.2, 19.5)
        rx, ry = pr.antenna_to_rear_axle(10.0, 20.0, math.pi / 2, 0.5, 0.2)
        self.assertAlmostEqual(rx, 10.2)
        self.assertAlmostEqual(ry, 19.5)

    def test_yaw_180(self):
        rx, ry = pr.antenna_to_rear_axle(0.0, 0.0, math.pi, 0.5, 0.2)
        self.assertAlmostEqual(rx, 0.5)
        self.assertAlmostEqual(ry, 0.2)

    def test_zero_offset_is_identity(self):
        rx, ry = pr.antenna_to_rear_axle(3.0, -4.0, 1.234, 0.0, 0.0)
        self.assertAlmostEqual(rx, 3.0)
        self.assertAlmostEqual(ry, -4.0)

    def test_no_default_offset(self):
        with self.assertRaises(TypeError):
            pr.antenna_to_rear_axle(10.0, 20.0, 0.0)  # offsets are mandatory

    def test_rejects_bad_numbers(self):
        for bad in (float("nan"), float("inf"), True, False, "0.5", None):
            with self.assertRaises(ValueError, msg=repr(bad)):
                pr.antenna_to_rear_axle(bad, 20.0, 0.0, 0.5, 0.0)
            with self.assertRaises(ValueError, msg=repr(bad)):
                pr.antenna_to_rear_axle(10.0, 20.0, bad, 0.5, 0.0)
            with self.assertRaises(ValueError, msg=repr(bad)):
                pr.antenna_to_rear_axle(10.0, 20.0, 0.0, bad, 0.0)
            with self.assertRaises(ValueError, msg=repr(bad)):
                pr.antenna_to_rear_axle(10.0, 20.0, 0.0, 0.5, bad)

    def test_rejects_non_finite_result_from_finite_inputs(self):
        # corrupted-input contract, not a vehicle value: 1e308 - (-1e308) overflows to inf
        with self.assertRaises(ValueError):
            pr.antenna_to_rear_axle(1e308, 0.0, 0.0, -1e308, 0.0)
        with self.assertRaises(ValueError):
            pr.antenna_to_rear_axle(0.0, 1e308, 0.0, 0.0, -1e308)


class TestRearAxlePose(unittest.TestCase):
    def test_happy_path_east(self):
        p = pr.rear_axle_pose(**_good_kwargs())
        self.assertIsInstance(p, ps.PoseInput)
        self.assertAlmostEqual(p.x, 9.5)
        self.assertAlmostEqual(p.y, 20.0)
        self.assertAlmostEqual(p.yaw, 0.0)
        self.assertEqual(p.frame_id, "map")
        self.assertEqual(p.reference_point, "rear_axle_center")
        self.assertIs(p.position_valid, True)
        self.assertIs(p.heading_valid, True)
        self.assertAlmostEqual(p.age_s, 0.30)   # max(position_age, heading_age)
        self.assertEqual(ps.pose_input_problem(p), "")

    def test_happy_path_north_with_lateral_offset(self):
        # compass 0 = north -> yaw +90 deg; forward = +y, left = -x
        p = pr.rear_axle_pose(**_good_kwargs(heading=0.0, offset_x_m=0.5, offset_y_m=0.2))
        self.assertAlmostEqual(p.yaw, math.pi / 2)
        self.assertAlmostEqual(p.x, 10.2)
        self.assertAlmostEqual(p.y, 19.5)

    def test_happy_path_south_enu_convention(self):
        p = pr.rear_axle_pose(**_good_kwargs(heading=-math.pi / 2, heading_convention="enu_ccw_rad"))
        self.assertAlmostEqual(p.yaw, -math.pi / 2)
        self.assertAlmostEqual(p.x, 10.0)
        self.assertAlmostEqual(p.y, 20.5)

    # ---- map axes vs ENU (synthetic rotation angles, not a HYPER map property)
    def test_map_rotated_plus_90_rotates_yaw_and_offset(self):
        # ENU east (yaw 0) with an explicit ENU-to-map coordinate rotation of +90 deg -> map yaw +pi/2.
        # The offset (0.5 fwd, 0.2 left) is rotated with the MAP yaw: rear = (10.2, 19.5).
        p = pr.rear_axle_pose(**_good_kwargs(map_yaw_from_enu_rad=math.pi / 2, offset_y_m=0.2))
        self.assertAlmostEqual(p.yaw, math.pi / 2)
        self.assertAlmostEqual(p.x, 10.2)
        self.assertAlmostEqual(p.y, 19.5)

    def test_map_rotated_minus_90_rotates_yaw_and_offset(self):
        p = pr.rear_axle_pose(**_good_kwargs(map_yaw_from_enu_rad=-math.pi / 2, offset_y_m=0.2))
        self.assertAlmostEqual(p.yaw, -math.pi / 2)
        self.assertAlmostEqual(p.x, 9.8)
        self.assertAlmostEqual(p.y, 20.5)

    def test_map_rotated_180_wraps_to_half_open_interval(self):
        p = pr.rear_axle_pose(**_good_kwargs(map_yaw_from_enu_rad=math.pi, offset_y_m=0.2))
        self.assertAlmostEqual(p.yaw, -math.pi)
        self.assertAlmostEqual(p.x, 10.5)
        self.assertAlmostEqual(p.y, 20.2)

    def test_map_rotation_does_not_move_xy(self):
        # x/y are already map positions: with zero offset they must pass through unchanged
        p = pr.rear_axle_pose(**_good_kwargs(map_yaw_from_enu_rad=1.0, offset_x_m=0.0, offset_y_m=0.0))
        self.assertAlmostEqual(p.x, 10.0)
        self.assertAlmostEqual(p.y, 20.0)
        self.assertAlmostEqual(p.yaw, 1.0)

    def test_map_rotation_is_mandatory(self):
        kw = _good_kwargs()
        del kw["map_yaw_from_enu_rad"]
        with self.assertRaises(TypeError):
            pr.rear_axle_pose(**kw)

    def test_age_keeps_the_older_of_the_two(self):
        p = pr.rear_axle_pose(**_good_kwargs(position_age_s=0.05, heading_age_s=0.20))
        self.assertAlmostEqual(p.age_s, 0.20)
        p = pr.rear_axle_pose(**_good_kwargs(position_age_s=0.40, heading_age_s=0.05))
        self.assertAlmostEqual(p.age_s, 0.40)

    def test_age_equal_to_threshold_is_accepted(self):
        p = pr.rear_axle_pose(**_good_kwargs(position_age_s=0.5, heading_age_s=0.5, max_age_s=0.5))
        self.assertAlmostEqual(p.age_s, 0.5)

    def test_keyword_only_and_no_defaults(self):
        with self.assertRaises(TypeError):
            pr.rear_axle_pose(10.0, 20.0, 90.0, "north_cw_deg")  # positional refused
        kw = _good_kwargs()
        for missing in ("offset_x_m", "offset_y_m", "max_age_s", "planar_assumption_verified",
                        "frame_alignment_verified", "heading_convention", "reference_point",
                        "map_yaw_from_enu_rad"):
            partial = dict(kw)
            del partial[missing]
            with self.assertRaises(TypeError, msg=missing):
                pr.rear_axle_pose(**partial)

    # ---- frame / reference
    def test_rejects_wrong_or_unverified_frame(self):
        for fid in ("odom", "MAP", "utm", "", None, 1):
            with self.assertRaises(ValueError, msg=repr(fid)):
                pr.rear_axle_pose(**_good_kwargs(frame_id=fid))
        for fav in (False, "True", 1, None):
            with self.assertRaises(ValueError, msg=repr(fav)):
                pr.rear_axle_pose(**_good_kwargs(frame_alignment_verified=fav))

    def test_rejects_double_conversion(self):
        with self.assertRaisesRegex(ValueError, "double conversion"):
            pr.rear_axle_pose(**_good_kwargs(reference_point="rear_axle_center"))

    def test_rejects_other_reference_points(self):
        for ref in ("imu", "base_link", "GNSS_ANTENNA", "", None, 3):
            with self.assertRaises(ValueError, msg=repr(ref)):
                pr.rear_axle_pose(**_good_kwargs(reference_point=ref))

    # ---- validity flags
    def test_rejects_invalid_position_or_heading(self):
        with self.assertRaises(ValueError):
            pr.rear_axle_pose(**_good_kwargs(position_valid=False))
        with self.assertRaises(ValueError):
            pr.rear_axle_pose(**_good_kwargs(heading_valid=False))

    def test_rejects_non_bool_flags(self):
        for flag in ("position_valid", "heading_valid", "planar_assumption_verified", "frame_alignment_verified"):
            for bad in ("True", "yes", 1, 1.0, None, [True]):
                with self.assertRaises(ValueError, msg=f"{flag}={bad!r}"):
                    pr.rear_axle_pose(**_good_kwargs(**{flag: bad}))

    def test_rejects_unverified_planar_assumption(self):
        with self.assertRaisesRegex(ValueError, "planar"):
            pr.rear_axle_pose(**_good_kwargs(planar_assumption_verified=False))

    # ---- ages
    def test_rejects_stale_heading_with_fresh_position(self):
        with self.assertRaisesRegex(ValueError, "heading stale"):
            pr.rear_axle_pose(**_good_kwargs(position_age_s=0.0, heading_age_s=0.6, max_age_s=0.5))

    def test_rejects_stale_position_with_fresh_heading(self):
        with self.assertRaisesRegex(ValueError, "position stale"):
            pr.rear_axle_pose(**_good_kwargs(position_age_s=0.51, heading_age_s=0.0, max_age_s=0.5))

    def test_rejects_future_age(self):
        with self.assertRaisesRegex(ValueError, "negative"):
            pr.rear_axle_pose(**_good_kwargs(position_age_s=-0.01))
        with self.assertRaisesRegex(ValueError, "negative"):
            pr.rear_axle_pose(**_good_kwargs(heading_age_s=-1.0))

    def test_rejects_non_positive_or_bad_max_age(self):
        for bad in (0, 0.0, -0.5, float("nan"), float("inf"), True, "0.5", None):
            with self.assertRaises(ValueError, msg=repr(bad)):
                pr.rear_axle_pose(**_good_kwargs(max_age_s=bad))

    def test_rejects_bad_age_numbers(self):
        for bad in (float("nan"), float("inf"), True, "0.1", None):
            with self.assertRaises(ValueError, msg=repr(bad)):
                pr.rear_axle_pose(**_good_kwargs(position_age_s=bad))
            with self.assertRaises(ValueError, msg=repr(bad)):
                pr.rear_axle_pose(**_good_kwargs(heading_age_s=bad))

    # ---- numbers
    def test_rejects_bad_coordinates_heading_offsets(self):
        for field in ("x", "y", "heading", "offset_x_m", "offset_y_m", "map_yaw_from_enu_rad"):
            for bad in (float("nan"), float("inf"), -float("inf"), True, False, "1.0", None):
                with self.assertRaises(ValueError, msg=f"{field}={bad!r}"):
                    pr.rear_axle_pose(**_good_kwargs(**{field: bad}))

    def test_rejects_unrepresentable_int_fields_as_value_error(self):
        for field in ("x", "heading", "offset_x_m", "map_yaw_from_enu_rad"):
            with self.assertRaises(ValueError, msg=field):
                pr.rear_axle_pose(**_good_kwargs(**{field: 10 ** 400}))

    def test_rejects_unsupported_convention(self):
        for conv in ("deg", "NORTH_CW_DEG", "enu_cw_deg", None, ""):
            with self.assertRaises(ValueError, msg=repr(conv)):
                pr.rear_axle_pose(**_good_kwargs(heading_convention=conv))

    def test_no_substitution_on_bad_input(self):
        # a bad offset must raise, never silently become 0 (which would return the antenna pose)
        with self.assertRaises(ValueError):
            pr.rear_axle_pose(**_good_kwargs(offset_x_m=float("nan")))


if __name__ == "__main__":
    unittest.main()
