"""Direction adapter around the unchanged forward follower and wire protocol.

The controller sees travel yaw; GNSS antenna translation always uses BODY yaw.
Reverse steering is the negative virtual-forward equivalent angle BEFORE the
existing asymmetric left-wheel Ackermann conversion. PWM remains a magnitude.
"""
import dataclasses
import math
import live_path_follow as base
from path_follow_inputs import Heading
from main_event_core import Stationary
import qualifier_route as qr


class DirectionalDriveGate(base.DriveGate):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.direction=1;self.last_wire_pwm=0;self.last_wire_drive=0

    def change_direction(self,direction,stationary,now):
        if direction not in (-1,1):raise ValueError('invalid drive direction')
        if self.allowed_until>now or stationary.stopped_since is None:
            raise ValueError('direction change requires closed gate and observed neutral stop')
        self.direction=direction

    def __call__(self,command):
        # Preserve the exact old forward path, including firmware protocol.
        if self.direction==1:
            raw=super().__call__(command)
            self.last_wire_pwm=raw[16];self.last_wire_drive=raw[13]
            return raw
        active=(self.clock()<self.allowed_until and not self.adapter.locked_by_fault
                and self.adapter.phase in ('ready','active')
                and self.adapter.have_target_since_prepare and command.action==base.smp.CONTROL)
        drive=-1 if active and self.pwm else 0
        if drive:self.sent_drive_frames+=1
        cmd=base.smp.Command(command.boot_id,command.request_id,command.action,drive,command.steer,
                             1 if drive or command.steer else 0)
        raw=base.jp.encode_command(cmd,self.pwm if drive else 0)
        self.last_wire_pwm=raw[16];self.last_wire_drive=drive
        return raw


class BodyHeading(Heading):
    def __init__(self,*args,antenna_forward_m=.315,**kw):
        super().__init__(*args,**kw);self.direction=1
        self.antenna_forward_m=antenna_forward_m;self.measured_curvature=None

    def gps(self,x,y,stamp):
        if self.trail and stamp<=self.trail[-1][2]:return
        before=self.anchor_at
        super().gps(x,y,stamp)
        if self.direction<0 and self.anchor_at!=before and self.anchor_at==stamp:
            self.yaw=qr.wrap(self.yaw+math.pi)
            if self.measured_curvature is not None:
                # Constant-steer local approximation from MEASURED steering ADC:
                # GNSS course is antenna velocity at the chord midpoint, not
                # current rear-axle BODY yaw. Preserve the forward estimator.
                k=self.measured_curvature;a=self.antenna_forward_m
                anchor=next(((px,py) for px,py,t in reversed(list(self.trail)[:-1])
                             if 0<stamp-t<=3. and math.hypot(x-px,y-py)>=self.window),None)
                if anchor is not None:
                    chord=math.hypot(x-anchor[0],y-anchor[1])/math.sqrt(1+(a*k)**2)
                    turn=2*math.asin(k*chord/2)
                    self.yaw=qr.wrap(self.yaw-math.atan(a*k)-turn/2)

    def change_direction(self,direction):
        if direction not in (-1,1):raise ValueError('invalid heading direction')
        self.direction=direction;self.trail.clear()
        self.measured_curvature=None


def travel_pose(body_pose,direction):
    if direction not in (-1,1):raise ValueError('invalid travel direction')
    return dataclasses.replace(body_pose,yaw=qr.wrap(body_pose.yaw+(math.pi if direction<0 else 0.)))


def physical_left_wheel(virtual_equivalent_cdeg,direction,wheelbase,track,left_limit=25.,right_limit=19.):
    if direction not in (-1,1):raise ValueError('invalid steering direction')
    wheel=base.left_wheel_cdeg(direction*virtual_equivalent_cdeg,wheelbase,track)
    if not -right_limit*100<=wheel<=left_limit*100:raise ValueError(f'maneuver left wheel {wheel/100:.2f}deg exceeds [-{right_limit},+{left_limit}] from virtual {virtual_equivalent_cdeg/100:.2f}deg direction {direction}')
    return wheel


class WindowFollower:
    """Run the unchanged follower on one directional window, exposing global indices.

Earlier forward lanes may overlap a reverse parking window; they are not valid
rejoin candidates during that window. No nearest-point search across missions.
"""
    def __init__(self,follower,start=0,end=None,direction=1):
        self.follower=follower;self.start=start;self.end=end;self.direction=direction

    def step(self,body_pose,now):
        state=self.follower.state;original=state.route
        end=len(original.points)-1 if self.end is None else self.end
        if not self.start<=state.progress<=end:raise ValueError('progress outside active directional window')
        state.route=qr.ModeRoute(original.mode,original.segments,original.points[self.start:end+1])
        state.progress-=self.start
        try:return self.follower.step(travel_pose(body_pose,self.direction),now)
        finally:
            state.progress+=self.start;state.route=original
