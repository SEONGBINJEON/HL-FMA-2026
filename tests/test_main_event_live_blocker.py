import sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import main_event_runtime as rt
import steer_target_host as host

class StopLockTests(unittest.TestCase):
 def test_real_adapter_operator_stop_and_later_fault_are_distinct(self):
  core=NS(stop=lambda _:None)
  ad=host.HostSteerAdapter(core);trial=NS(adapter=ad,client=NS(failed=False))
  ad.stop('operator quit');expected=rt.stop_fault_signature(trial)
  self.assertTrue(ad.locked_by_fault)
  self.assertIsNone(rt.final_stop_fault(trial,{'boot':70},70,expected))
  ad.stop('보드 상태 수신 만료 · 다시 조작 준비 필요')
  self.assertEqual(ad.first_fault,'operator quit')
  self.assertEqual(rt.final_stop_fault(trial,{'boot':70},70,expected),'board_fault_after_stop')
 def test_actual_runtime_expected_quit_lock_does_not_fail_stationary(self):
  from test_main_event_runtime import RuntimeIntegrationTests
  code,saved,_=RuntimeIntegrationTests().exercise('operator_stop_lock')
  self.assertTrue(saved['stop_ack']);self.assertTrue(saved['stationary_observed'])
  self.assertNotEqual(code,3);self.assertNotEqual(saved['fault'],'board_fault_after_stop')
 def test_prepare_then_stop_without_start_never_arms(self):
  from test_main_event_runtime import RuntimeIntegrationTests
  code,saved,clock=RuntimeIntegrationTests().exercise('stop_before_start')
  self.assertEqual(code,130);self.assertEqual(saved['reason'],'operator_stop')
  self.assertEqual(clock.trial.prepares,0);self.assertEqual(clock.x,0.)
  self.assertTrue(saved['stop_ack']);self.assertTrue(saved['stationary_observed'])
 def test_fault_reboot_and_missing_ack_still_fail(self):
  from test_main_event_runtime import RuntimeIntegrationTests
  check=RuntimeIntegrationTests()
  check.test_reboot_after_disarm_ack_cannot_confirm_final_stationarity()
  check.test_fault_after_disarm_ack_cannot_confirm_final_stationarity()
  check.test_missing_ack_reports_exit3_not_stopped_success()

class RecheckTests(unittest.TestCase):
 def test_failed_record_preserved_and_real_protocol_disarm_only(self):
  import tempfile,json,hashlib
  from unittest.mock import patch
  import main_event_stop_recheck as r
  from test_main_event_manual_protocol import Board
  for mode in ('ok','reboot','moving'):
   with self.subTest(mode=mode),tempfile.TemporaryDirectory() as tmp:
    root=Path(tmp);(root/'data').mkdir();old=root/'old';old.mkdir();active=root/'data/active.json'
    saved={'phase':'stopped','stop_ack':True,'stationary_observed':False,'process_released':True,'fault':'board_fault_after_stop'}
    state=old/'state.json';state.write_text(json.dumps(saved));before=state.read_bytes()
    active.write_text(json.dumps({'session':str(old)}));pointer=active.read_bytes();boards=[]
    def connect(*a,**k):
     board=Board(root/'port',mode);boards.append(board);return board
    with patch.object(r,'ROOT',root),patch.object(r,'PosixSerialTransport',side_effect=connect),patch('main_event_release.verify',return_value={'release_id':'synthetic'}):
     with self.assertRaises(ValueError):r.recheck(active,False)
     self.assertEqual(boards,[])
     if mode=='ok':
      self.assertEqual(r.recheck(active,True),0);self.assertFalse(active.exists())
      self.assertEqual(next((root/'data').glob('main_event_active_before_recheck_*.json')).read_bytes(),pointer)
      report=json.loads(next((root/'data').glob('me_stop_recheck_*/stop_recheck.json')).read_text())
      self.assertTrue(report['stationary_observed']);self.assertEqual(report['boot'],8)
     else:
      with self.assertRaises(ValueError):r.recheck(active,True)
      self.assertEqual(active.read_bytes(),pointer)
    self.assertEqual(state.read_bytes(),before)
    self.assertTrue(boards[0].closed);self.assertTrue(all(c[2]==0 for c in boards[0].sent))
 def test_dead_server_stop_reports_old_failure_without_socket(self):
  import tempfile,json
  from unittest.mock import patch
  import main_event
  from main_event_menu import Menu
  with tempfile.TemporaryDirectory() as tmp:
   old=Path(tmp);s={'phase':'stopped','process_released':True,'stop_ack':True,'stationary_observed':False,'reason':'old failure'}
   (old/'state.json').write_text(json.dumps(s))
   with patch.object(main_event,'active',return_value=old),patch.object(main_event,'send',side_effect=AssertionError('dead socket must not be retried')):
    self.assertEqual(main_event.main(['stop']),3)
   m=Menu(read=lambda:s,execute=lambda _:self.fail('no dead server call'),emit=lambda *a:None)
   self.assertFalse(m.stop())

if __name__=='__main__':unittest.main()
