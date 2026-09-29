"""Arrival requires traversal to the final segment, not only spatial proximity."""
import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import path_steering as ps
import qualifier_route as qr

def points(xy):
 out=[]
 for i,(x,y) in enumerate(xy):
  a=xy[max(0,i-1)];b=xy[min(len(xy)-1,i+1)]
  out.append(qr.Point(i,x,y,math.atan2(b[1]-a[1],b[0]-a[0]),'synthetic'))
 return out

class EndProgress(unittest.TestCase):
 def test_near_finish_at_start_is_not_completion(self):
  p=points([(0.,0.),(4.,0.),(4.,4.),(0.,4.),(0.,.1)])
  d=ps.compute_target(p,0,ps.PoseInput(0.,0.,0.),ps.SYNTHETIC_CONFIG)
  self.assertEqual(d.kind,'steer');self.assertFalse(d.end_reached)
 def test_start_beyond_final_plane_is_not_completion(self):
  p=points([(0.,0.),(4.,0.),(4.,-4.),(0.,-4.),(0.,-.8)])
  d=ps.compute_target(p,0,ps.PoseInput(0.,0.,0.),ps.SYNTHETIC_CONFIG)
  self.assertEqual(d.kind,'steer');self.assertFalse(d.end_reached)
 def test_final_segment_can_complete(self):
  p=points([(0.,0.),(4.,0.),(4.,4.),(0.,4.),(0.,.1)])
  d=ps.compute_target(p,4,ps.PoseInput(0.,.2,-math.pi/2),ps.SYNTHETIC_CONFIG)
  self.assertEqual(d.kind,'stop');self.assertTrue(d.end_reached)

if __name__=='__main__':unittest.main()
