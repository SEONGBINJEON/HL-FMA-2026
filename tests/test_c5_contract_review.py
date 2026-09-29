"""Independent Codex angle-contract probes; no physical measurements."""
import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import steering_angle_contract as ac
import path_steering as ps

class ContractReview(unittest.TestCase):
 def contract(self,L=.75):return ac.AngleContract('synthetic_ideal_ackermann','bicycle_equivalent','left_front_wheel',L,.67)
 def test_left_wheel_is_not_bicycle_identity(self):
  kw=dict(contract=self.contract(),synthetic=True,min_cdeg=-1800,max_cdeg=1600)
  self.assertGreater(ac.convert_angle(800,**kw),800)
  self.assertLess(abs(ac.convert_angle(-800,**kw)),800)
 def test_large_length_uses_hold_error_contract(self):
  msg=ps.TargetMessage('steer',1,800,1000,{})
  out=ac.convert_message(msg,self.contract(10**400),synthetic=True,min_cdeg=-1800,max_cdeg=1600)
  self.assertIsInstance(out,ps.StopMessage)
  self.assertEqual(out.kind,'hold')
 def test_missing_contract_never_becomes_identity(self):
  msg=ps.TargetMessage('steer',1,0,1000,{})
  out=ac.convert_message(msg,None,synthetic=True,min_cdeg=-1800,max_cdeg=1600)
  self.assertIsInstance(out,ps.StopMessage)
 def test_singular_geometry_refused(self):
  with self.assertRaises(ValueError):ac.convert_angle(8900,self.contract(),synthetic=True,min_cdeg=-9000,max_cdeg=9000)

class ReplayBoundsReview(unittest.TestCase):
 def fixture(self):
  import json
  return json.loads((Path(__file__).resolve().parents[1]/'tests/fixtures/c5_replay_synthetic.json').read_text())
 def test_absolute_timestamp_bounded_before_pumping(self):
  import integrated_replay as ir
  d=self.fixture()
  for row in d['records']:row[0]+=1e6
  with self.assertRaises(ValueError):ir.validate_input(d)
 def test_larger_offset_is_a_real_limit_not_claimed_success(self):
  import integrated_replay as ir
  d=self.fixture()
  for row in d['records']:row[2]=.3  # rear y=.1, with explicit antenna left offset .2
  r=ir.run_replay(d)
  self.assertFalse(r['reached_by_input'])
  self.assertNotEqual(r['terminal_cause'],'path_end')
  self.assertEqual(r['board']['final']['actual_steer'],0)

 def test_huge_input_number_is_rejected_as_value_error(self):
  import integrated_replay as ir
  d=self.fixture();d['records'][0][1]=10**400
  with self.assertRaises(ValueError):ir.validate_input(d)
 def test_stale_heading_stops_an_active_output(self):
  import integrated_replay as ir
  d=self.fixture();d['records'][1][5]=2.0
  r=ir.run_replay(d);sr=r['stop_response']
  self.assertEqual(r['terminal_cause'],'hold_lock')
  self.assertAlmostEqual(sr['last_nonzero_s'],sr['hold_at_s']-.005,places=3)
  self.assertLessEqual(sr['latency_ms'],50)
  self.assertFalse(sr['nonzero_after_first_zero'])
  self.assertGreater(r['samples']['ignored_locked'],0)
  self.assertTrue(r['ok'])
  print('ACTIVE_STOP_EVIDENCE',r['terminal_cause'],sr)
 def test_gap_stops_an_active_output(self):
  import integrated_replay as ir
  d=self.fixture();d['records']=[r for r in d['records'] if r[0]==1.0 or r[0]>=1.8]
  d['records'][0][2]=.4  # synthetic rear y=.2; increased initial steering request
  r=ir.run_replay(d);sr=r['stop_response']
  self.assertEqual(r['terminal_cause'],'adapter_fault')
  self.assertAlmostEqual(sr['last_nonzero_s'],sr['hold_at_s']-.005,places=3)
  self.assertLessEqual(sr['latency_ms'],50)
  self.assertFalse(sr['nonzero_after_first_zero'])
  self.assertGreater(r['samples']['ignored_locked'],0)
  self.assertTrue(r['ok'])
  print('ACTIVE_STOP_EVIDENCE',r['terminal_cause'],sr)

if __name__=='__main__':unittest.main()

