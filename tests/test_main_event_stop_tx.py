import sys,time,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from test_main_event_manual_protocol import Board
from step_manual_control import Client,Session
from joystick_protocol import Parser,encode_command
from main_event_manual import confirm_stop

class StopTxTests(unittest.TestCase):
 def test_real_client_with_nonempty_USB_tx_queue(self):
  class QueuedBoard(Board):
   until=0.
   @property
   def out_waiting(self):return 19 if time.monotonic()<self.until else 0
   def write(self,raw):
    self.until=time.monotonic()+.006
    return super().write(raw)
  with tempfile.TemporaryDirectory() as tmp:
   b=QueuedBoard(Path(tmp)/'port');session=Session();events=[]
   journal=NS(event=lambda n,**k:events.append((n,k)),bytes=lambda *a:None)
   c=Client(b,session,journal,encoder=lambda cmd:encode_command(cmd,0));c.parser=Parser()
   c.tick(time.monotonic());self.assertGreater(b.out_waiting,0)
   r=confirm_stop(c,session,journal)
   self.assertFalse(c.failed);self.assertTrue(r['stop_ack']);self.assertTrue(r['stationary_observed'])
   self.assertTrue(b.closed);self.assertTrue(all(x[2]==0 for x in b.sent))
   self.assertTrue(any(n=='stop_tx_drained' for n,k in events))
 def test_stuck_queue_still_refuses_stop_success(self):
  class StuckBoard(Board):out_waiting=19
  with tempfile.TemporaryDirectory() as tmp:
   b=StuckBoard(Path(tmp)/'port');session=Session()
   # Real parsed status, but no queued command may be overwritten.
   parser=Parser();values,errors=parser.feed(b.read(100),time.monotonic());session.step(time.monotonic(),values)
   c=Client(b,session,encoder=lambda cmd:encode_command(cmd,0))
   r=confirm_stop(c,session,NS(event=lambda *a,**k:None))
   self.assertFalse(r['stationary_observed']);self.assertFalse(r['stop_ack'])
   self.assertIn('did not drain',r['reason']);self.assertEqual(b.sent,[]);self.assertTrue(b.closed)

if __name__=='__main__':unittest.main()
