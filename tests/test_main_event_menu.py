import json,sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from main_event_menu import Menu

class MenuTests(unittest.TestCase):
    def setup_menu(self,state,results):
        calls=[]
        def execute(args):
            calls.append(args)
            result=results.pop(0)
            if args==['stop'] and result[0]==0:state.update(stop_ack=True,stationary_observed=True,process_released=True)
            return result
        return Menu(read=lambda:state,execute=execute,ask=lambda _: "C",emit=lambda *a:None),calls
    def test_failed_stop_cannot_open_manual(self):
        state={'gates':[{'name':'s_curve'}],'index':0}
        menu,calls=self.setup_menu(state,[(3,'')]);menu.action('m')
        self.assertEqual(calls,[['stop']])
    def test_manual_receipt_required_and_no_automatic_start(self):
        for output,ok in [('{}',False),('{"manual_passage_stopped":true}',True)]:
            state={'gates':[{'name':'s_curve'}],'index':0,'session':'unused'}
            menu,calls=self.setup_menu(state,[(0,''),(0,output)])
            with patch.object(menu,'resume') as resume:
                if ok:menu.action('m');resume.assert_called_once()
                else:
                    with self.assertRaisesRegex(ValueError,'manual_passage_stopped'):menu.action('m')
                    resume.assert_not_called()
            self.assertFalse(any(c[0]=='start' for c in calls))
    def test_unsupported_stale_and_quit_stop_failure(self):
        state={'gates':[{'name':'finish_fork'}],'index':0,'phase':'waiting','ready':True,'heartbeat_fresh':False}
        menu,calls=self.setup_menu(state,[(3,'')])
        for key in ('m','2'):
            with self.assertRaises(ValueError):menu.action(key)
        self.assertTrue(menu.action('q'));self.assertEqual(calls,[['stop']])
    def test_failed_manual_does_not_reuse_autonomous_stop_on_quit(self):
        s={'gates':[{'name':'s_curve'}],'index':0,'stop_ack':True,'stationary_observed':True,'process_released':True}
        menu,calls=self.setup_menu(s,[(2,'')])
        with self.assertRaises(ValueError):menu.action('m')
        self.assertTrue(menu.action('q'))
        self.assertEqual(calls,[['manual','--mission','s_curve','--collision-recovery-confirmed']])

    def test_camera_snapshot_rejects_stale_frame_without_opening_viewer(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'frames').mkdir()
            (root/'frames/latest.json').write_text(json.dumps({'capture_at':time.monotonic()-5}))
            menu,_=self.setup_menu({'session':str(root)},[])
            with patch('main_event_menu.subprocess.Popen') as popen:
                with self.assertRaisesRegex(ValueError,'오래됨'):menu.action('v')
                popen.assert_not_called()
    def test_sonar_unknown_is_display_only_and_new_run_cannot_bypass_stop(self):
        lines=[];s={'phase':'waiting','heartbeat_fresh':True,'board':{'received_at':time.monotonic(),
                    'left_cm':80,'center_cm':999,'right_cm':120}}
        menu,calls=self.setup_menu(s,[]);menu.emit=lines.append;menu.show()
        self.assertTrue(any('80 / -- / 120' in line for line in lines));self.assertEqual(calls,[])
        with patch('main_event_menu.archive_active') as archive:
            with self.assertRaisesRegex(ValueError,'정지/해제'):menu.action('n')
            archive.assert_not_called()
    def test_missing_or_changed_receipt_never_invokes_recovery(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'session.json').write_text('{}')
            s={'session':str(root),'stop_ack':True,'stationary_observed':True,'process_released':True}
            menu,calls=self.setup_menu(s,[])
            with self.assertRaisesRegex(ValueError,'R은 실제 S충돌'):menu.action('r')
            self.assertEqual(calls,[])

    def test_explicit_restart_only_after_ready(self):
        s={'phase':'waiting','ready':True,'heartbeat_fresh':True,'section':'after_t','recovery_bootstrap_ready':False}
        menu,calls=self.setup_menu(s,[(0,'')])
        with self.assertRaises(ValueError):menu.action('2')
        s['recovery_bootstrap_ready']=True;menu.action('2');self.assertEqual(calls,[['start']])

if __name__=='__main__':unittest.main()
