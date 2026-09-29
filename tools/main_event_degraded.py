"""Small prestart-choice and stopped GNSS reacquisition policies. No estimated pose."""
import math


def core_ready(board,now,quality_ok,rate_ready):
    return bool(board and 0<=now-board['received_at']<.15 and
                board['state']=='DISARMED' and not board['fault'] and quality_ok and rate_ready)


def normalize_choices(values):
    if not isinstance(values,dict) or not values:raise ValueError('empty session choice')
    choices={}
    for name,value in values.items():
        if name not in ('t','p','finish') or not isinstance(value,str):raise ValueError('invalid choice key/value')
        v=value.lower().strip()
        if v=='lft':v='left'
        allowed=('up','down') if name=='p' else ('left','right')
        if v not in allowed:raise ValueError(name+' expects '+str(allowed))
        choices[name]=v
    return choices


class GapContinuity:
    """A stopped reacquisition, never permission to drive through missing fixes.

    Reuse the existing GNSS heading chord's 3s observation window and held-pose
    0.25m envelope. Larger/longer/ambiguous movement remains explicit recovery.
    Three separate fresh FIX receipts must agree after the gap. No new heading
    estimator, FLOAT grace, ARM, index jump, or timestamp renewal occurs here.
    """
    def __init__(self):self.reset()
    def reset(self):
        self.last=None;self.before=None;self.first_return=None;self.count=0;self.stamp=None
        self.invalid=None;self.evidence={'state':'continuous'}
    def update(self,fix,wall,s,board,yaw,heading_valid,aligned,stopped,held_valid,moved_while_held):
        if self.before is None and fix is None and self.last is not None:
            self.before=self.last;self.evidence={'state':'position_gap','last_fix_at':self.last['fix'][2]}
        if self.before is None:
            if fix is not None and heading_valid and aligned:
                self.last={'fix':fix,'s':s,'boot':board['boot'],'yaw':yaw}
            return False
        pre=self.before
        if moved_while_held:self.invalid='held_vehicle_moved'
        if board['boot']!=pre['boot']:self.invalid='board_boot_changed'
        if fix is None:return False
        if self.first_return is None:self.first_return=fix[2]
        # The return must fall inside the already-used 3s heading observation window.
        if not 0<self.first_return-pre['fix'][2]<=3.:self.invalid='gap_outside_heading_observation_window'
        if math.hypot(fix[0]-pre['fix'][0],fix[1]-pre['fix'][1])>.25:self.invalid='position_discontinuity'
        if not -.05<=s-pre['s']<=.25:self.invalid='route_progress_discontinuity'
        if yaw is None or abs(math.atan2(math.sin(yaw-pre['yaw']),math.cos(yaw-pre['yaw'])))>math.radians(5):
            self.invalid='heading_discontinuity'
        if self.stamp!=fix[2]:
            if self.stamp is not None and not 0<fix[2]-self.stamp<=.5:self.invalid='reacquisition_not_continuous'
            self.stamp=fix[2];self.count+=1
        proven=bool(not self.invalid and self.count>=3 and 0<=wall-fix[2]<=.5 and
                    heading_valid and aligned and stopped and held_valid)
        self.evidence={'state':'proven_stopped_continuity' if proven else 'reacquiring',
            'reject_reason':self.invalid,'fresh_fix_count':self.count,'gap_s':self.first_return-pre['fix'][2],
            'displacement_m':math.hypot(fix[0]-pre['fix'][0],fix[1]-pre['fix'][1]),
            'progress_m':s-pre['s'],'stationary':stopped,'heading_valid':heading_valid}
        return proven


def parking_execution(choices):
    """Current measured steering limits reject T-left; do not silently choose right."""
    result={}
    for key in ('t','p'):
        value=choices.get(key)
        feasible=(key=='t' and value=='right') or (key=='p' and value in ('up','down'))
        result[key]={'choice':value,'mode':'selected_path_attempt' if feasible else 'nonentry',
                     'reason':'existing_candidate_geometry' if feasible else 'steering_limit' if value else 'choice_not_supplied'}
    return result
