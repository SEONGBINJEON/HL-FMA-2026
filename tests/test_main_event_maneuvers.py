"""Pure geometry/controller contracts; synthetic road is NOT a surveyed course."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import qualifier_route as qr
from main_event_maneuvers import Envelope,s_candidates,assess_path,contained

ENV=Envelope(1.04,.24,.77,.1)
ROAD=[(-3.,-4.),(20.,-4.),(20.,4.),(-3.,4.)]
REF=[qr.Point(i,i*.5,0.,0.,'synthetic','map') for i in range(33)]

class CandidateTests(unittest.TestCase):
    def test_obstacle_left_selects_right_and_right_selects_left(self):
        for side in (-1,1):
            path,why=s_candidates(REF,ENV,ROAD,[(8.,side*.35)],lambda p:True)
            self.assertIsNotNone(path,why)
            self.assertLess(why['offset_m']*side,0)
            self.assertEqual((path[0].x,path[0].y),(REF[0].x,REF[0].y))
            self.assertAlmostEqual(path[-1].y,0.)

    def test_both_sides_blocked_no_fabricated_success(self):
        path,why=s_candidates(REF,ENV,ROAD,[(8.,i*.1) for i in range(-40,41)],lambda p:True)
        self.assertIsNone(path);self.assertIn('occupied',why['rejected'].values())

    def test_boundary_cannot_be_crossed_to_avoid_obstacle(self):
        narrow=[(-3.,-.8),(20.,-.8),(20.,.8),(-3.,.8)]
        path,why=s_candidates(REF,ENV,narrow,[(8.,0.)],lambda p:True)
        self.assertIsNone(path);self.assertIn('outside_boundary',why['rejected'].values())

    def test_sensor_loss_rejects_all_candidates(self):
        path,why=s_candidates(REF,ENV,ROAD,[],lambda p:False)
        self.assertIsNone(path);self.assertEqual(set(why['rejected'].values()),{'unobserved'})

    def test_curvature_checked_against_actual_left_wheel_asymmetric_limits(self):
        tight=[qr.Point(i,x,y,0.,'synthetic','map') for i,(x,y) in enumerate([(0.,0.),(.5,0.),(.5,.5)])]
        self.assertEqual(assess_path(tight,ENV,ROAD,[],lambda p:True),'steering_limit')

    def test_concave_road_not_just_four_corner_check(self):
        road=[(0.,0.),(4.,0.),(4.,4.),(2.5,4.),(2.5,1.),(1.5,1.),(1.5,4.),(0.,4.)]
        self.assertFalse(contained([(1.,2.),(3.,2.),(3.,3.),(1.,3.)],road))

from main_event_maneuvers import parking_paths,PARKING_SEGMENTS,finish_crossed
from main_event_plan import TRACK
from main_event_motion import DirectionalDriveGate,BodyHeading,travel_pose,physical_left_wheel
from main_event_core import Stationary
import path_steering as ps
import joystick_protocol as jp
import step_manual_protocol as smp
from types import SimpleNamespace


class DirectionContracts(unittest.TestCase):
    def test_straight_reverse_does_not_create_a_fake_180deg_corner(self):
        points=[qr.Point(i,-float(i),0.,0.,'synthetic_body_yaw','map') for i in range(3)]
        env=Envelope(1.04,.24,.77,.1)
        road=[(-3.,-.7),(2.,-.7),(2.,.7),(-3.,.7)]
        self.assertEqual(assess_path(points,env,road,[],lambda _:True,direction=-1),'clear')

    def test_pinned_main_parking_segments_and_body_yaw(self):
        for name,segments in PARKING_SEGMENTS.items():
            parts=parking_paths(name,TRACK)
            self.assertEqual([d for p,d in parts],[d for a,b,d in segments])
            self.assertTrue(any(d<0 for p,d in parts))
            self.assertEqual(parts[-1][1],1)

    def test_reverse_course_is_converted_to_body_yaw_once(self):
        h=BodyHeading(.5);h.change_direction(-1)
        h.gps(0.,0.,1.);h.gps(-1.,0.,2.)
        self.assertAlmostEqual(h.yaw,0.)
        h.gps(-1.,0.,2.1);self.assertAlmostEqual(h.yaw,0.)
        h.change_direction(1);self.assertEqual(len(h.trail),0)

    def test_reverse_chord_and_antenna_correction_from_measured_steering(self):
        for curvature in (-.4,.4):
            h=BodyHeading(.5);h.change_direction(-1);h.measured_curvature=curvature
            for i in range(31):
                yaw=-curvature*i*.04
                rear=(math.sin(yaw)/curvature,(1-math.cos(yaw))/curvature)
                h.gps(rear[0]+.315*math.cos(yaw),rear[1]+.315*math.sin(yaw),1+i*.1)
            self.assertLess(abs(qr.wrap(h.yaw-yaw)),.002)

    def test_body_reference_translation_precedes_virtual_yaw(self):
        p=ps.PoseInput(2.,3.,0.)
        virtual=travel_pose(p,-1)
        self.assertEqual((virtual.x,virtual.y),(2.,3.));self.assertAlmostEqual(abs(virtual.yaw),math.pi)
        self.assertGreater(physical_left_wheel(-1000,-1,.75,.67),0)
        self.assertLess(physical_left_wheel(1000,-1,.75,.67),0)
        with self.assertRaises(ValueError):physical_left_wheel(5000,1,.75,.67)

    def test_direction_change_needs_neutral_observed_stop_and_encodes_signed_axis(self):
        adapter=SimpleNamespace(locked_by_fault=False,phase='ready',have_target_since_prepare=True)
        gate=DirectionalDriveGate(70,adapter,clock=lambda:1.)
        stop=Stationary()
        with self.assertRaises(ValueError):gate.change_direction(-1,stop,1.)
        stop.stopped_since=.5;gate.open(1.)
        with self.assertRaises(ValueError):gate.change_direction(-1,stop,1.)
        gate.close();gate.change_direction(-1,stop,1.);gate.open(1.)
        cmd=smp.Command(1,2,smp.CONTROL,0,0,0)
        raw=gate(cmd)
        self.assertEqual(raw[13],255) # int8 -1, PWM still positive70
        self.assertEqual(raw[16],70)
        gate.close();self.assertEqual(gate(cmd)[13],0)

    def test_finish_both_rear_wheels_and_finite_official_segment(self):
        line={'a':[5.,-2.],'b':[5.,2.],'forward_normal':[1.,0.],'verified':True,'evidence':'synthetic only','margin_m':.05}
        self.assertIsNone(finish_crossed((6.,0.,0.),{},.67))
        self.assertFalse(finish_crossed((5.,0.,0.),line,.67))
        self.assertTrue(finish_crossed((5.1,0.,0.),line,.67))
        self.assertFalse(finish_crossed((5.1,0.,math.pi/4),line,.67))
        self.assertFalse(finish_crossed((6.,3.,0.),line,.67))

if __name__=='__main__':unittest.main()
