import sys
import struct
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from joystick_protocol import Parser,PAYLOAD,SIZE,encode_command
from step_manual_protocol import crc16,encode_status,Status,Command
from step_manual_control import Session,Client
from gamepad_input import direction,DpadPwm
from joystick_control import Driver
from joystick_desktop import connection_message

def frame():
    p=PAYLOAD.pack(1,1,50,0,500,0,0,0,0,0,0,38,20,999,90,0,60,60,129,3,7,0,0)
    b=bytes([150,len(p)])+p
    return b'\xf1\xa5'+b+struct.pack('<H',crc16(b))

class Pad:
    def __init__(self):self.value=(0.,0.);self.pwm_steps=[]
    def poll(self):return self.value

class Transport:
    def __init__(self,raw):self.raw=raw;self.tx=[]
    @property
    def in_waiting(self):return len(self.raw)
    @property
    def out_waiting(self):return 0
    def read(self,n):a,self.raw=self.raw,b'';return a
    def write(self,raw):self.tx.append(raw);return len(raw)
    def close(self):pass

class Tests(unittest.TestCase):
    def test_ascii_mismatch_is_visible_and_never_sends(self):
        t=Transport(b'HENES F8 DRI');s=Session();c=Client(t,s);c.parser=Parser()
        c.tick(__import__('time').monotonic())
        t.raw=b'VE TEST\r\n[MONITOR] Drive=STOP | Steering=1023\n'
        c.tick(__import__('time').monotonic())
        connected,message=connection_message(c,s,__import__('time').monotonic())
        self.assertFalse(connected);self.assertIn('코드 불일치',message)
        self.assertFalse(s.request_arm(__import__('time').monotonic()));self.assertEqual(t.tx,[])
    def test_native_connection_requires_fresh_board(self):
        t=Transport(frame());s=Session();c=Client(t,s);c.parser=Parser()
        now=__import__('time').monotonic();c.tick(now)
        self.assertTrue(connection_message(c,s,now+.01)[0])
        self.assertFalse(connection_message(c,s,now+.16)[0])
        c.failed=True
        self.assertFalse(connection_message(c,s,now+.02)[0])
    def test_type150_status_and_unknown_echo(self):
        p=Parser();values,errors=p.feed(frame(),0)
        self.assertFalse(errors);self.assertEqual(SIZE,47)
        self.assertEqual(values[0].center_cm,999)
        self.assertEqual(values[0].steer_compare,60)
        self.assertEqual(values[0].timer1a,129)
        self.assertEqual(values[0].steer_pin_modes,7)
        self.assertTrue(isinstance(values[0],Status))
        damaged=bytearray(frame());damaged[9]^=1
        self.assertTrue(p.feed(damaged,.01)[1])
    def test_old_and_ascii_firmware_receive_zero_transmission(self):
        prior=bytearray(frame());prior[2]=149;prior[-2:]=struct.pack('<H',crc16(prior[2:-2]))
        for raw in [bytes(prior),encode_status(Status(1,1)),b'HC-SR04 3 SENSOR TEST\r\nLEFT : 33.5 cm\n']:
            t=Transport(raw);s=Session();c=Client(t,s);c.parser=Parser();c.tick(__import__('time').monotonic())
            self.assertFalse(s.identified);self.assertEqual(t.tx,[])
    def test_both_sticks_and_independent_neutral(self):
        p=Pad();s=Session();s.phase='ready';d=Driver(p,s,None)
        p.value=(1.,-1.);d.input();self.assertEqual(s.keys,{'w','a'})
        p.value=(0.,-1.);d.input();self.assertEqual(s.keys,{'a'})
        p.value=(0.,0.);d.input();self.assertEqual(s.keys,set())
    def test_deadband_and_mapping_confirmation_requires_neutral(self):
        self.assertEqual(direction(.19),0);self.assertEqual(direction(.21),1)
        self.assertEqual(direction(.15,1),1);self.assertEqual(direction(.1,1),0)
        p=Pad();d=Driver(p,Session(),None);p.value=(1.,0.);d.input();d.action({'action':'confirm'},1)
        self.assertFalse(d.confirmed)
    def test_focus_loss_latches_stop(self):
        p=Pad();s=Session();s.phase='ready';d=Driver(p,s,None)
        d.action({'action':'pulse','focused':False},1)
        self.assertEqual(s.phase,'locked');self.assertFalse(s.focused)
    def test_stop_events_do_not_write_outside_periodic_sender(self):
        class ClientTrap:
            def stop(self,*_):raise AssertionError('Duplicate serial sender')
        p=Pad();s=Session();s.phase='ready';d=Driver(p,s,ClientTrap())
        d.action({'action':'stop'},1)
        d.action({'action':'pulse','focused':False},1.01)
        self.assertEqual(s.phase,'locked')

    def test_dpad_press_hold_release_and_initial_events(self):
        d=DpadPwm()
        self.assertEqual(d.event(-32767,True),0)
        self.assertEqual(d.event(-32767),0)
        self.assertEqual(d.event(0),0)
        self.assertEqual([d.event(v) for v in [-32767,-32767,0,32767,32767,0,-32767,0]],[1,0,0,-1,0,0,1,0])
    def test_pwm_steps_bounds_and_zero_drive_with_steering(self):
        p=Pad();s=Session();s.phase='ready';d=Driver(p,s,None)
        p.value=(1.,1.);p.pwm_steps=[1,1,-1];d.input()
        self.assertEqual(d.drive_pwm,65);self.assertEqual(s.keys,{'w','d'})
        p.pwm_steps=[-1]*100;d.input()
        self.assertEqual(d.drive_pwm,0);self.assertEqual(s.keys,{'d'})
        p.pwm_steps=[1]*100;d.input()
        self.assertEqual(d.drive_pwm,255);self.assertEqual(s.keys,{'w','d'})
    def test_versioned_pwm_command_and_crc(self):
        for pwm in [0,60,65,255]:
            b=encode_command(Command(1,2,2,1,-1,1),pwm)
            self.assertEqual(len(b),19);self.assertEqual(b[:4],bytes([241,165,2,13]))
            self.assertEqual(b[16],pwm);self.assertEqual(crc16(b[2:-2]),int.from_bytes(b[-2:],'little'))
        for pwm in [-1,256,1.5]:
            with self.assertRaises(ValueError):encode_command(Command(1,2),pwm)

if __name__=='__main__':unittest.main()
