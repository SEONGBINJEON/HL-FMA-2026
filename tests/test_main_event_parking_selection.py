import sys,json,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from main_event_plan import build_section,ROOT,append_common,save_bundle,restore_bundle
from main_event_degraded import parking_execution
from main_event_lidar_scope import lidar_stop_scope

class ParkingSelectionTests(unittest.TestCase):
 def test_current_supported_choices_are_bound_before_start_without_frozen_config_edit(self):
  cfg=json.loads((ROOT/'config/main_event.json').read_text());original=json.dumps(cfg,sort_keys=True)
  for t in ('left','right'):
   for p in ('up','down'):
    choices={'t':t,'p':p,'finish':'right'}
    b=build_section('start','left',cfg,parking_choices=choices)
    self.assertEqual(b['gates'][-1].kind,'parking' if t=='right' else 'transition')
    self.assertEqual(b['session_choices'],choices)
    self.assertEqual(parking_execution(choices)['t']['mode'],'selected_path_attempt' if t=='right' else 'nonentry')
    following=build_section('after_t','left',cfg,parking_choices=choices)
    self.assertEqual(following['gates'][-1].kind,'parking')
    restored=restore_bundle(save_bundle(following));self.assertEqual(restored['session_choices'],choices)
  self.assertEqual(json.dumps(cfg,sort_keys=True),original)
 def test_parking_observation_scope_is_motion_only_and_directional(self):
  for name in ('parking_t_right','parking_p_up','parking_p_down'):
   b={'section':name,'labels':[],'gates':[]}
   self.assertFalse(lidar_stop_scope(b,1,'waiting')[0])
   self.assertTrue(lidar_stop_scope(b,1,'running')[0])
