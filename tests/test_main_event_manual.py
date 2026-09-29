import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import main_event
import main_event_manual as manual


class ManualHandoffTests(unittest.TestCase):
    def final_stop(self,mode):
        clock=SimpleNamespace(t=10.,closed=False)
        status=SimpleNamespace(boot_id=8,request_id=10,state=0,encoder_count=50,
                               requested_drive=0,actual_drive=0,requested_steer=0,actual_steer=0)
        session=SimpleNamespace(status=status,last_status_at=10.,
                                emergency_command=lambda:SimpleNamespace(boot_id=8,request_id=10))
        def tick(t):
            status.request_id+=1;session.last_status_at=t
            if mode=='reboot' and status.request_id>15:status.boot_id=9
            if mode=='moving':status.encoder_count+=1
        client=SimpleNamespace(failed=False,tick=tick,stop=lambda reason:None,
                               transport=SimpleNamespace(close=lambda:setattr(clock,'closed',True)))
        with patch.object(manual.time,'monotonic',side_effect=lambda:clock.t), \
             patch.object(manual.time,'time',side_effect=lambda:clock.t), \
             patch.object(manual.time,'sleep',side_effect=lambda dt:setattr(clock,'t',clock.t+dt)):
            result=manual.confirm_stop(client,session,SimpleNamespace(event=lambda *a,**k:None),timeout=1.)
        self.assertTrue(clock.closed)
        return result

    def test_manual_close_requires_newer_same_boot_stop_and_encoder_settling(self):
        result=self.final_stop('ok')
        self.assertTrue(result['stop_ack']);self.assertTrue(result['stationary_observed'])
        self.assertEqual(result['boot'],8)

    def test_manual_close_reboot_or_moving_is_not_handoff_success(self):
        for mode in ('reboot','moving'):
            self.assertFalse(self.final_stop(mode)['stationary_observed'])

    def test_confirmed_S_collision_window_then_actual_recovery_preserves_context(self):
        for name,next_section in [('s_curve','after_s')]:
            with self.subTest(name=name),tempfile.TemporaryDirectory() as temp:
                root=Path(temp);(root/'data').mkdir();old=root/'old';old.mkdir()
                saved={'stop_ack':True,'stationary_observed':True,'process_released':True,'boot':7,
                       'section':'start','gates':[{'name':name}],'index':0,'completed':['hill','light_1'],
                       'skipped':[],'progress':100,'fault':None,'finish_lane':None}
                info={'run_id':'same-run','epoch':3,'lane':'left','section':'start'}
                (old/'state.json').write_text(json.dumps(saved));(old/'session.json').write_text(json.dumps(info))
                def window(argv,**kwargs):
                    folder=Path(argv[-1]);folder.mkdir()
                    (folder/'metadata.json').write_text(json.dumps({'status':'completed'}))
                    (folder/'final_stop.json').write_text(json.dumps({'stop_ack':True,'stationary_observed':True,
                         'process_released':True,'boot':8,'encoder':300,'finished_wall':200.}))
                    return SimpleNamespace(returncode=0)
                with patch.object(manual,'ROOT',root),patch.object(manual.subprocess,'run',side_effect=window):
                    self.assertEqual(manual.run_manual(old,name,collision_recovery_confirmed=True),0)
                a=SimpleNamespace(config=main_event.CONFIG,after_manual=True,skip_mission=None,judge_permitted=False,
                                  body_heading_deg=None,index=None,prepare_only=False)
                with patch.object(main_event,'active',return_value=old), \
                     patch.object(main_event.sys.stdin,'isatty',return_value=False), \
                     patch.object(main_event,'prepare',return_value=0) as prep:
                    self.assertEqual(main_event.recover(a),0)
                recovery=prep.call_args.args[1]
                self.assertEqual(recovery['section'],next_section)
                self.assertEqual(recovery['run_id'],'same-run');self.assertEqual(recovery['epoch'],4)
                self.assertEqual(recovery['completed'],['hill','light_1'])
                self.assertEqual(recovery['skipped'],[name])
                self.assertEqual(recovery['recovery']['heading_mode'],'forward_bootstrap')
                self.assertEqual(recovery['recovery']['boot'],8)
                self.assertTrue(recovery['recovery']['boot_binding_pending'])
                (old/'state.json').write_text('{}')
                with self.assertRaisesRegex(ValueError,'does not match'):manual.load_handoff(old,info)


if __name__=='__main__':unittest.main()
