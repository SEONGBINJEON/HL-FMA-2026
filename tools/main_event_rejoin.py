"""Bounded current-window GPS proposal, with fresh measured body yaw at commit."""
import math
import qualifier_follow as q
from main_event_motion import travel_pose
from main_event_core import finite


def recovery_ceiling(bundle,progress,pending):
    """Stop gates remain uncrossable; an entered S ends at its completion gate."""
    if type(progress) is not int or not 0<=progress<len(bundle['points']):
        raise ValueError('saved progress outside preserved route')
    if not pending:return bundle['s'][-1]
    gate=pending[0]
    if gate.kind=='s_curve' and bundle['s'][progress]>=gate.stop_s:
        return gate.commit_s
    return gate.stop_s


def candidate_window(bundle,progress,end,ceiling,skipped=False):
    lower=0 if skipped else max(0,progress-3)
    # Indices identify segments [i,i+1]; the final point belongs to no segment
    # inside this gear window. Never propose the next direction's first segment.
    upper=min(len(bundle['points'])-2,end-1,lower+12)
    result=[i for i in range(lower,upper+1) if bundle['s'][i]<ceiling]
    if not result:raise ValueError('no rejoin segment before unfinished mission in current window')
    return result


def propose(bundle,indices,x,y,max_distance=1.5):
    """Position-only suggestion; route tangent is NEVER substituted for body yaw."""
    result=[]
    position=type('Position',(),{'x':x,'y':y})()
    for i in indices:
        p=q.project(bundle['points'],i,position)
        if p and p[0]<=max_distance+.315:result.append({'index':i,'label':bundle['labels'][i],'antenna_distance_m':p[0]})
    result.sort(key=lambda r:r['antenna_distance_m'])
    if not result:return {'proposed_index':None,'reason':'no_nearby_current_window_segment','candidates':[]}
    best=result[0]
    ambiguous=any(abs(bundle['s'][v['index']]-bundle['s'][best['index']])>3. and
                  abs(v['antenna_distance_m']-best['antenna_distance_m'])<.25 for v in result[1:])
    return {'proposed_index':None if ambiguous else best['index'],
            'reason':'ambiguous_current_window_branches' if ambiguous else 'position_only_requires_body_yaw',
            'candidates':result[:8]}


def commit(bundle,indices,fix,body_heading_deg,observed_wall,wall,args,direction,index=None):
    if not finite(body_heading_deg) or not -180<=body_heading_deg<=180:
        raise ValueError('fresh measured BODY yaw required; route tangent is not a measurement')
    if not finite(observed_wall) or not 0<=wall-observed_wall<=15.:
        raise ValueError('body yaw expired; measure after sensors are ready')
    if fix is None or not 0<=wall-fix[2]<=.5:raise ValueError('fresh FIX required for rejoin')
    pose=q.make_pose(args,*fix[:2],math.radians(body_heading_deg),wall,fix[2])
    virtual=travel_pose(pose,direction)
    allowed=indices
    if index is not None:
        if type(index) is not int or index not in indices:raise ValueError('index outside preserved current mission window')
        allowed=[index]
    elif propose(bundle,indices,*fix[:2])['proposed_index'] is None:
        raise ValueError('GPS rejoin missing/ambiguous; no global nearest-point fallback')
    selected,distance,err=q.select_entry(bundle['points'],allowed,virtual.as_route_pose(),
                                        min(args.start_distance_m,1.5),min(args.arm_heading_deg,45.))
    return {'index':selected,'body_yaw':pose.yaw,'distance_m':distance,'heading_error_deg':math.degrees(err),
            'observed_wall':observed_wall,'antenna_xy':list(fix[:2]),'heading_source':'operator_measured_body_yaw'}


def valid_anchor(recovery,board,fix,wall):
    if recovery.get('pose_pending') or not finite(recovery.get('observed_wall')):return False
    if not 0<=wall-recovery['observed_wall']<=15. or fix is None or board is None:return False
    if board['boot']!=recovery.get('boot') or board['encoder']!=recovery.get('encoder'):return False
    p=recovery.get('antenna_xy')
    return bool(p and math.hypot(fix[0]-p[0],fix[1]-p[1])<=.25)


def bootstrap_ready(recovery,board,fix,hint,stationary,wall):
    """Explicit manual SKIP starts a NEW pose, never reuses the old moving pose."""
    if not board or board['state']!='DISARMED' or board.get('fault') or stationary.stopped_since is None:return False
    if fix is None or not 0<=wall-fix[2]<=.5 or not hint or hint.get('proposed_index') is None:return False
    if recovery.get('boot_binding_pending'):
        recovery.update(previous_boot=recovery['boot'],boot=board['boot'],encoder=board['encoder'],
                        antenna_xy=list(fix[:2]),boot_binding_pending=False)
    if board['boot']!=recovery['boot']:raise ValueError('board reboot after manual recovery boot binding')
    return board['encoder']==recovery.get('encoder') and math.hypot(fix[0]-recovery['antenna_xy'][0],fix[1]-recovery['antenna_xy'][1])<=.25
