"""Section PWM policy. Distances are design margins, not new brake calibration."""
import math
from collections import deque

class SpeedPolicy:
    def __init__(self,config):
        self.spec=config.get('speed_policy',{})
        self.low=min(config['drive_pwm'],70);self.high=self.spec.get('cruise_pwm',self.low)
        self.horizon=self.spec.get('cruise_observation_m',2.4)
        self.approach=self.spec.get('approach_low_m',6.)
        self.origin=None;self.trail=deque();self.speed=None;self.high_guard=False;self.last_high=None
        self.selected=self.low;self.reason='initial_or_bootstrap'
    def observe(self,fix,now):
        if fix is None:return
        if self.trail and fix[2]<=self.trail[-1][2]:return
        self.trail.append(fix)
        while len(self.trail)>2 and fix[2]-self.trail[1][2]>.6:self.trail.popleft()
        span=fix[2]-self.trail[0][2]
        self.speed=None
        if .4<=span<=1.:
            self.speed=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(self.trail,list(self.trail)[1:]))/span
        if self.high_guard and now-self.last_high>=2. and self.speed is not None and self.speed<=.95:
            self.high_guard=False
    def permit(self,allowed,fix):
        if not allowed:self.origin=None
        elif self.origin is None and fix:self.origin=fix[:2]
    def zone(self,phase,mission,bundle,s,direction):
        if phase!='running':return 'initial_or_bootstrap'
        if 'hill' not in mission.completed:return 'hill_not_completed'
        if direction<0 or bundle['section'].startswith('parking_'):return 'parking_or_reverse'
        policy=bundle.get('preentry_policy')
        if policy and s>=policy['start_s']-self.approach:return 'autonomous_nonentry_road_approach'
        gate=mission.gate
        if gate and gate.kind=='s_curve' and s>=gate.stop_s-self.approach:return 'S_approach_or_entire_section'
        if gate and gate.stop_s-s<=self.approach:return 'mission_or_signal_approach'
        if bundle['section'].startswith('finish'):return 'finish_approach'
        return 'general_after_hill'
    def choose(self,phase,mission,bundle,s,direction,fix,far_observation,now):
        reason=self.zone(phase,mission,bundle,s,direction);pwm=self.low
        if reason=='general_after_hill':
            if self.origin is None or fix is None or math.hypot(fix[0]-self.origin[0],fix[1]-self.origin[1])<2.:
                reason='release_or_rejoin_first_2m'
            elif self.speed is None or self.speed>1.6:reason='speed_unobserved_or_above_recorded_envelope'
            elif far_observation!='clear':reason='long_corridor_'+str(far_observation)+'_keep70'
            else:pwm=self.high;reason='general_after_hill_long_corridor_clear'
        self.selected=pwm;self.reason=reason
        if pwm>70:self.high_guard=True;self.last_high=now
        return pwm
