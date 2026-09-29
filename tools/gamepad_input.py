"""Read only the identified Xbox-compatible Linux joystick. Never writes it."""
import array
import fcntl
import os
from pathlib import Path
import struct


def direction(value, previous=0):
    # Software neutral band (20% engage, 12% release); not steering calibration.
    threshold=.12 if previous else .20
    return (1 if value>threshold else -1 if value < -threshold else 0)


class DpadPwm:
    """Consume each hat event; init/hold/release are not new presses."""
    def __init__(self):self.previous=0
    def event(self,value,initial=False):
        current=(value>0)-(value<0)
        step=-current if current and current!=self.previous and not initial else 0
        self.previous=current
        return step


class Gamepad:
    def __init__(self):
        blocks=Path('/proc/bus/input/devices').read_text().split('\n\n')
        matches=[b for b in blocks if 'Vendor=045e Product=028e' in b and 'Microsoft X-Box 360 pad' in b]
        if len(matches)!=1:raise RuntimeError('Expected one identified controller')
        node=next(w for l in matches[0].splitlines() if l.startswith('H:') for w in l.split() if w.startswith('js'))
        self.path='/dev/input/'+node;self.fd=os.open(self.path,os.O_RDONLY|os.O_NONBLOCK)
        self.axes=[0.]*8;self.buttons=[0]*11;self.raw_events=[];self.pwm_steps=[];self.dpad=DpadPwm()
        codes=array.array('B',[0]*64);fcntl.ioctl(self.fd,0x80406a32,codes)
        if codes[:8].tolist()!=[0,1,2,3,4,5,16,17]:self.close();raise RuntimeError('Unexpected joystick axis map')
        self.identity=matches[0];self.axis_codes=codes[:8].tolist()
    def poll(self):
        self.raw_events=[];self.pwm_steps=[]
        for _ in range(8):
            try:raw=os.read(self.fd,1024)
            except BlockingIOError:break
            if not raw:raise OSError('Controller disconnected')
            if len(raw)%8:raise OSError('Malformed joystick events')
            self.raw_events.append(raw)
            for offset in range(0,len(raw),8):
                stamp,val,kind,idx=struct.unpack_from('IhBB',raw,offset)
                if kind&2 and idx==7:
                    step=self.dpad.event(val,bool(kind&0x80))
                    if step:self.pwm_steps.append(step)
                if kind&2 and idx<len(self.axes):self.axes[idx]=max(-1.,val/32767.)
                if kind&1 and idx<len(self.buttons):self.buttons[idx]=val
        else:raise OSError('Controller input flood')
        return -self.axes[1],self.axes[3]
    def close(self):
        if self.fd is not None:os.close(self.fd);self.fd=None
