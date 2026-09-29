import unittest,json
from unittest.mock import patch
from test_main_event_lidar_scope import event
from main_event_core import Gate
from main_event_finish import FinishChoice
from main_event_plan import build_section,append_finish,ROOT
class FinishTests(unittest.TestCase):
 def test_valid_both_priority_at_deadline_and_latch(self):
  g=Gate('finish_fork','unsupported',20,21)
  for lane in ('left','right'):
   f=FinishChoice(0,'left')
   for i,t in enumerate((0,2.8,3.)):r=f.update(event(t,i,'finish_fork','allow_'+lane),t,10,g)
   self.assertEqual(r['lane'],lane);self.assertEqual(r['source'],'fresh_panel_window')
   self.assertEqual(f.update(None,9,19,g),r)
   self.assertEqual(f.update(event(10,5,'finish_fork','allow_left'),10,19,g),r)
 def test_missing_stale_two_panels_or_insufficient_votes(self):
  g=Gate('finish_fork','unsupported',20,21)
  for e in (None,event(-1,1,'finish_fork','allow_right'),event(0,1,'finish_fork','none'),event(0,1,'finish_fork','allow_right')):
   f=FinishChoice(0,'left');self.assertIsNone(f.update(e,0,9,g))
   r=f.update(e,3,10,g);self.assertEqual(r['source'],'fallback_penalty');self.assertEqual(r['lane'],'left')
  f=FinishChoice(0,'left');r=f.update(None,0,20-g.approach_m-1,g)
  self.assertEqual(r['elapsed_s'],0);self.assertEqual(r['lane'],'left')
 def test_real_HYPER_append_both_forward_existing_paths(self):
  c=json.loads((ROOT/'config/main_event.json').read_text());b=build_section('after_p','left',c)
  for lane in ('left','right'):
   e=append_finish(b,lane);self.assertEqual(e['points'][:len(b['points'])],b['points'])
   self.assertEqual(e['gates'][-1].kind,'finish');self.assertEqual(e['finish_lane'],lane)
   print('HYPER',lane,'old/new points',len(b['points']),len(e['points']),'omitted overlap',e['finish_seam_omitted_prefix'])
 def test_actual_runtime_no_detection_connected_to_finish(self):
  from test_main_event_runtime import RuntimeIntegrationTests
  import main_event_runtime as rt
  rows=[];original=rt.q.lst.TrialJournal.event
  def log(j,n,**kw):
   if n=='finish_branch':rows.append(kw)
   return original(j,n,**kw)
  with patch.object(rt.q.lst.TrialJournal,'event',log):code,saved,clock=RuntimeIntegrationTests().exercise('fallback_full_left')
  self.assertEqual(code,0,saved['reason']);self.assertEqual(saved['finish_lane'],'left')
  self.assertEqual(len(rows),1);self.assertEqual(rows[0]['source'],'fallback_penalty');self.assertTrue(saved['stop_ack'])
  print('runtime finish fallback',rows,saved['reason'])
