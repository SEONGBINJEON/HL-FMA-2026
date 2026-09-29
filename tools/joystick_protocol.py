"""Only type150 (adjustable PWM + steering register diagnostics) status identifies the new joystick firmware."""
from dataclasses import dataclass
import struct
from step_manual_protocol import Status, crc16

PAYLOAD = struct.Struct('<IIIiHBBbbbbHHHHBBHBBBBB')
SIZE = PAYLOAD.size+6


@dataclass(frozen=True)
class JoystickStatus(Status):
    left_cm: int = 999
    center_cm: int = 999
    right_cm: int = 999
    sonar_flags: int = 0
    drive_pwm: int = 60
    steer_compare: int = 0
    timer1a: int = 0
    timer1b: int = 0
    steer_pin_modes: int = 0
    steer_output_latch: int = 0
    steer_input_levels: int = 0

    def validate(self):
        super().validate()
        if self.steer_pin_modes > 7 or self.steer_output_latch > 3 or self.steer_input_levels > 3:
            raise ValueError('Invalid steering diagnostic bits')
        if any(not 0<=n<=999 for n in (self.left_cm,self.center_cm,self.right_cm)) or self.sonar_flags != 0:
            raise ValueError('Invalid sonar status')


def decode(raw):
    if len(raw)!=SIZE or raw[:4]!=bytes([0xf1,0xa5,150,PAYLOAD.size]):
        raise ValueError('Wrong joystick firmware type/length')
    if crc16(raw[2:-2])!=int.from_bytes(raw[-2:],'little'):
        raise ValueError('CRC')
    value=JoystickStatus(*PAYLOAD.unpack(raw[4:-2]));value.validate();return value


class Parser:
    def __init__(self):
        self.pending=bytearray();self.since=None;self.identified=False
        self.diagnostic_tail=b'';self.unsupported_banner=False
    def feed(self,data,now):
        values=[];errors=[]
        if len(data)>4096:return [],['RX flood']
        if not self.identified:
            self.diagnostic_tail=(self.diagnostic_tail+data)[-8192:]
            if b'HENES F8 DRIVE TEST' in self.diagnostic_tail or b'[MONITOR] Drive=' in self.diagnostic_tail:
                self.unsupported_banner=True
        if self.pending and now-self.since>=.03:
            self.pending.clear();errors.append('Partial status expired')
        for b in data:
            if not self.pending:
                if b==0xf1:self.pending.append(b);self.since=now
                elif self.identified:errors.append('Unexpected byte')
                continue
            self.pending.append(b)
            if (len(self.pending)==2 and b!=0xa5) or (len(self.pending)==4 and self.pending[2:]!=bytes([150,PAYLOAD.size])):
                if self.identified:errors.append('Wrong frame')
                self.pending.clear()
            elif len(self.pending)==SIZE:
                try:values.append(decode(bytes(self.pending)));self.identified=True
                except ValueError as exc:errors.append(str(exc))
                self.pending.clear()
        return values,errors


def encode_command(command, drive_pwm):
    command.validate()
    if type(drive_pwm) is not int or not 0 <= drive_pwm <= 255:
        raise ValueError('PWM must be 0..255')
    payload=struct.pack('<IIBbbBB',command.boot_id,command.request_id,
                        command.action,command.drive,command.steer,command.deadman,drive_pwm)
    body=bytes([2,len(payload)])+payload
    return b'\xf1\xa5'+body+struct.pack('<H',crc16(body))
