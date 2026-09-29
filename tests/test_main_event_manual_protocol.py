"""Real Client/Session/type150 parser/type2 encoder; simulated wire board only."""
import contextlib,dataclasses,fcntl,io,json,os,struct,sys,tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import main_event,main_event_manual as manual
from step_manual_control import Client,Session
from joystick_protocol import Parser,encode_command,JoystickStatus,PAYLOAD
from step_manual_protocol import crc16

class Board:
    out_waiting=0
    def __init__(self,port,mode='ok'):
        self.fd=os.open(port,os.O_RDWR|os.O_CREAT,0o600);fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.mode=mode;self.closed=False;self.sent=[];self.req=10;self.boot=8;self.enc=50;self.disarmed=False
    @property
    def in_waiting(self):return PAYLOAD.size+6
    def read(self,size):
        self.req+=1
        if self.mode=='moving':self.enc+=1
        if self.mode=='reboot' and self.req>15:self.boot=9
        s=JoystickStatus(boot_id=self.boot,request_id=self.req,millis=self.req*10,encoder_count=self.enc,
            steering_adc=477,state=0 if self.disarmed else 1,flags=102,
            requested_drive=0 if self.disarmed else 1,actual_drive=0 if self.disarmed else 1,
            left_cm=80,center_cm=999,right_cm=120)
        body=bytes([150,PAYLOAD.size])+PAYLOAD.pack(*dataclasses.astuple(s))
        return b'\xf1\xa5'+body+struct.pack('<H',crc16(body))
    def write(self,raw):
        assert raw[:3]==b'\xf1\xa5\x02'
        assert crc16(raw[2:-2])==int.from_bytes(raw[-2:],'little')
        command=struct.unpack('<IIBbbBB',raw[4:-2]);self.sent.append(command)
        if command[0]==self.boot and command[2]==0:self.disarmed=True
        return len(raw)
    def close(self):
        if not self.closed:os.close(self.fd);self.closed=True

class ProtocolHandoffTests(unittest.TestCase):
    def stop_board(self,port,mode='ok'):
        board=Board(port,mode);session=Session();events=[]
        journal=SimpleNamespace(event=lambda *a,**k:events.append((a,k)),bytes=lambda *a:None)
        client=Client(board,session,journal,encoder=lambda c:encode_command(c,60));client.parser=Parser()
        client.tick(time.monotonic())
        report=manual.confirm_stop(client,session,journal,timeout=.8)
        self.assertTrue(board.closed);self.assertTrue(board.sent)
        self.assertTrue(all(c[2]==0 for c in board.sent))
        self.assertTrue(events[-1][0][0]=='manual_final_stop')
        return report
    def test_real_wire_rejects_motion_and_reboot(self):
        with tempfile.TemporaryDirectory() as t:
            for mode in ('moving','reboot'):
                report=self.stop_board(Path(t)/'port',mode)
                self.assertFalse(report['stationary_observed'])
    def test_failed_new_manual_invalidates_previous_receipt(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);(root/'data').mkdir();old=root/'old';old.mkdir()
            (old/'state.json').write_text(json.dumps({'stop_ack':True,'stationary_observed':True,
                'process_released':True,'gates':[{'name':'s_curve'}],'index':0}))
            (old/'session.json').write_text('{}');(old/'manual_handoff.json').write_text('{"old":true}')
            with patch.object(manual,'ROOT',root),patch.object(manual.subprocess,'run',side_effect=OSError('simulated failure')):
                with self.assertRaises(OSError):manual.run_manual(old,'s_curve',collision_recovery_confirmed=True)
            self.assertFalse((old/'manual_handoff.json').exists())
            self.assertEqual(len(list(old.glob('manual_handoff_superseded_*.json'))),1)

    def test_real_wire_manual_cli_receipt_port_release_then_recover_cli(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);(root/'data').mkdir();old=root/'old';old.mkdir();port=root/'board_port.lock'
            saved={'stop_ack':True,'stationary_observed':True,'process_released':True,'boot':7,
                'section':'start','gates':[{'name':'s_curve'}],'index':0,'completed':['hill','light_1'],
                'skipped':[],'progress':100,'fault':None,'finish_lane':None}
            info={'run_id':'protocol-run','epoch':3,'lane':'left','section':'start'}
            (old/'state.json').write_text(json.dumps(saved));(old/'session.json').write_text(json.dumps(info))
            def window(argv,**kw):
                # Real protocol processing replaces only Tk/gamepad/physical serial.
                folder=Path(argv[-1]);folder.mkdir();report=self.stop_board(port)
                self.assertTrue(report['stop_ack']);self.assertTrue(report['stationary_observed'])
                (folder/'final_stop.json').write_text(json.dumps(report))
                (folder/'metadata.json').write_text('{"status":"completed"}')
                return SimpleNamespace(returncode=0)
            recoveries=[]
            def prepare(args,recovery):
                # Both the manual parent ownership lock and simulated serial lock must be released.
                fds=[]
                try:
                    for p in (root/'data/main_event.lock',port):
                        fd=os.open(p,os.O_RDWR);fds.append(fd);fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
                    recoveries.append(recovery)
                finally:
                    for fd in fds:os.close(fd)
                return 0
            output=io.StringIO()
            with patch.object(manual,'ROOT',root),patch.object(main_event,'active',return_value=old), \
                 patch.object(manual.subprocess,'run',side_effect=window),patch.object(main_event,'prepare',side_effect=prepare), \
                 contextlib.redirect_stdout(output):
                from main_event_menu import Menu
                calls=[]
                def cli(args):
                    calls.append(args);captured=io.StringIO()
                    with contextlib.redirect_stdout(captured):code=main_event.main(args)
                    return code,captured.getvalue()
                menu=Menu(read=lambda:{**saved,'session':str(old)},execute=cli,ask=lambda _: 'C',emit=lambda *a:None)
                menu.action('M')
                self.assertEqual(calls,[['manual','--mission','s_curve','--collision-recovery-confirmed'],['recover','--after-manual']])
            r=recoveries[0];self.assertEqual(r['section'],'after_s');self.assertEqual(r['epoch'],4)
            self.assertEqual(r['run_id'],'protocol-run');self.assertEqual(r['skipped'],['s_curve'])
            self.assertEqual(r['recovery']['heading_mode'],'forward_bootstrap')
            self.assertEqual(r['recovery']['boot'],8)
            print('REAL Client/Session type150/type2 CRC DISARM -> encoder stable -> receipt -> locks released -> after_s prepared; no ARM/start')

if __name__=='__main__':unittest.main()
