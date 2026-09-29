"""HYPER main-only forward sections and explicit unsupported mission barriers."""
import hashlib
import json
from pathlib import Path
import math
import qualifier_route as qr
from main_event_core import Gate

ROOT = Path(__file__).resolve().parents[1]
TRACK = ROOT/'reference/external/HYPER/src/planning/hyper_waypoint/waypoints/track'
MISSION = ROOT/'reference/external/HYPER/src/planning/hyper_planner/mission/mission_track.yaml'

# Original main CSV row indices (not common_1_hyper.csv with removed row 172).
LABELS = {'start_left': {'hill': 52}, 'start_right': {'hill': 41},
          'common_1': {'light_1':68,'s_curve':136,'s_end':183,'light_2':228,'t_parking':287},
          'common_2': {'light_3':98,'accel_start':227,'accel_end':300}}
SECTIONS = {'start':('common_1',0), 'after_s':('common_1',183),
            'after_t':('common_2',0), 'after_p':('common_3',0)}
NEXT_RECOVERY = {'s_curve':'after_s','t_parking':'after_t','p_parking':'after_p'}


def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_section(section, lane, config, resume_index=None, parking_choices=None):
    if config.get('map_bundle') != 'HYPER_main_f8537416':
        raise ValueError('unsupported mixed map bundle')
    if section not in SECTIONS or lane not in ('left','right'):
        raise ValueError('explicit section and start lane required')
    track_name, begin = SECTIONS[section]
    def read(name):
        return qr.load_track_csv(name, TRACK/f'{name}.csv').points
    points, labels, hashes = [], [], {str(MISSION.relative_to(ROOT)):digest(MISSION)}
    if section == 'start':
        start_name = 'start_'+lane
        points = list(read(start_name))
        labels = [f'{start_name}:{i}' for i in range(len(points))]
        hashes[str((TRACK/f'{start_name}.csv').relative_to(ROOT))] = digest(TRACK/f'{start_name}.csv')
    common = list(read(track_name))
    if points:
        seam = math.hypot(points[-1].x-common[0].x,points[-1].y-common[0].y)
        if seam > 2.: raise ValueError('HYPER main route seam exceeds 2m')
    # Reuse the exact previously validated raw-row172 exclusion without shifting
    # label identities or modifying the HYPER source CSV / proven follower.
    # common_2 row39 is a new CANDIDATE repair: single .525m backstep while
    # body yaw stays forward; main YAML orders FRPP, with no gear event.
    # Its actual-runtime failure and geometry are logged in this review round.
    omitted={172} if track_name=='common_1' else ({39} if track_name=='common_2' else set())
    source_indices=[i for i in range(begin,len(common)) if i not in omitted]
    points += [common[i] for i in source_indices]
    labels += [f'{track_name}:{i}' for i in source_indices]
    hashes[str((TRACK/f'{track_name}.csv').relative_to(ROOT))] = digest(TRACK/f'{track_name}.csv')
    ss = qr.cumulative_s(points)
    gates = []
    def gate(name,kind,track,idx,min_s=0):
        k = labels.index(f'{track}:{idx}')
        # Pose is REAR AXLE. The measured725mm is antenna->bumper, not rear axle->bumper.
        front=config['antenna_forward_of_rear_axle_m']+config['front_bumper_forward_of_antenna_m']
        margin=config['stop_margin_before_line_m']
        if not all(type(v) in (int,float) and math.isfinite(v) and v>0 for v in (front,margin)):
            raise ValueError('invalid measured stopline reference')
        gates.append(Gate(name,kind,ss[k]-front-margin,ss[k]-front,min_s,tuple(config['signals'].get(name,[]))))
    if section == 'start':
        gate('hill','hill','start_'+lane,LABELS['start_'+lane]['hill'],3.)
        gate('light_1','signal','common_1',68)
        gate('s_curve','s_curve','common_1',136)
        gates[-1].commit_s=ss[labels.index('common_1:183')]
    if section in ('start','after_s'):
        gate('light_2','signal','common_1',228)
        gate('t_parking','parking','common_1',287)
    elif section == 'after_t':
        gate('light_3','signal','common_2',98)
        gate('p_parking','parking','common_2',len(common)-1)
    elif section == 'after_p':
        gate('finish_fork','unsupported','common_3',len(common)-1)
    # Initial placement window MUST precede the first unfinished mission.
    first_gate_i = min(range(len(ss)),key=lambda i:abs(ss[i]-gates[0].stop_s))
    allowed = list(range(max(0,first_gate_i-1)))
    if resume_index is not None:
        if not 0 <= resume_index < len(points)-1: raise ValueError('resume outside section')
        allowed = [resume_index]
    if not allowed: raise ValueError('no valid entry window')
    bundle={'points':points,'labels':labels,'s':ss,'gates':gates,'allowed':allowed,
            'hashes':hashes,'section':section,'lane':lane,'config':config,
            'front_offset_m':config['antenna_forward_of_rear_axle_m']+config['front_bumper_forward_of_antenna_m'],
            'stop_margin_m':config['stop_margin_before_line_m'],
            'excluded_raw_rows':{track_name:sorted(omitted)} if omitted else {}}
    if parking_choices:bundle['session_choices']=dict(parking_choices)
    from main_event_autonomy import preentry_route
    return preentry_route(bundle)


def blockers(config, mount):
    problems = []
    from main_event_lidar import validate_profile,body_geometry_usable
    try: validate_profile(mount,mount.get('mount_id'))
    except ValueError as exc: problems.append(str(exc))
    for key in ('body_width_m','front_overhang_from_rear_axle_m','clearance_m','stop_distance_m'):
        if type(mount.get(key)) not in (int,float) or not math.isfinite(mount[key]) or mount[key]<=0:
            problems.append('lidar/obstacle measurement missing: '+key)
    if not body_geometry_usable(mount): problems.append('body geometry input/basis missing')
    
    if not config.get('s_avoidance_verified') and not config.get('candidate_reference_mode'):problems.append('S route mode/input missing')
    if not config.get('reverse_enabled'):problems.append('parking direction adapter implemented; rear observation/live validation missing')
    for light in ('light_2','light_3'):
        if not config['signals'].get(light): problems.append(light+' manoeuvre/allowed signal unconfirmed')
    if not config.get('finish_rear_wheel_line_confirmed') and config.get('finish_mode')!='hyper_reference_goal':problems.append('finish reference missing')
    return problems


def append_finish(bundle, lane):
    """Append the main branch without restarting heading or opening another Mega."""
    if lane not in ('left','right') or bundle['section']!='after_p':
        raise ValueError('finish branch requires after_p and a valid observed lane')
    track=qr.load_track_csv('end_'+lane,TRACK/f'end_{lane}.csv')
    a,b=bundle['points'][-1],track.points[0]
    if math.hypot(a.x-b.x,a.y-b.y)>2.:raise ValueError('finish seam exceeds main tolerance')
    result=dict(bundle)
    # Main end branches overlap the tail of common_3 (left first point is
    # ~1.04m behind its end). Keep common_3 and omit only the overlapping
    # branch prefix, within HYPER's existing 2m seam tolerance. Never issue a
    # backward segment while the forward gear/window remains active.
    yaw=qr.path_heading_at(bundle['points'],len(bundle['points'])-1)
    distances=qr.cumulative_s(track.points)
    begin=next((i for i,p in enumerate(track.points) if distances[i]<=2. and
                (p.x-a.x)*math.cos(yaw)+(p.y-a.y)*math.sin(yaw)>1e-6),None)
    if begin is None:raise ValueError('no forward finish seam within original2m tolerance')
    result['points']=bundle['points']+list(track.points[begin:])
    result['labels']=bundle['labels']+[f'end_{lane}:{i}' for i in range(begin,len(track.points))]
    result['s']=qr.cumulative_s(result['points'])
    result['gates']=bundle['gates'][:-1]+[Gate('finish','finish',result['s'][-1]-bundle['front_offset_m']-bundle['stop_margin_m'],result['s'][-1]-bundle['front_offset_m'])]
    result['hashes']={**bundle['hashes'],str((TRACK/f'end_{lane}.csv').relative_to(ROOT)):digest(TRACK/f'end_{lane}.csv')}
    result['finish_lane']=lane
    result['finish_seam_omitted_prefix']=list(range(begin))
    return result


def append_parking(bundle,name,branch,planned_points=None):
    """Replace only current parking gate with neutral gear windows and exit event."""
    from main_event_maneuvers import parking_paths,PARKING_SEGMENTS
    if (name,branch) not in [('t_parking','t_left'),('t_parking','t_right'),('p_parking','p_up'),('p_parking','p_down')]:
        raise ValueError('branch does not belong to current parking mission')
    if bundle['gates'][-1].name!=name:raise ValueError('parking is not the end of current section')
    pieces=parking_paths(branch,TRACK)
    pts=qr.load_track_csv(branch,TRACK/(branch+'.csv')).points
    if planned_points is not None:
        if len(planned_points)!=len(pts):raise ValueError('planned parking samples must retain segment boundary indices')
        for i in {v for a,b,d in PARKING_SEGMENTS[branch] for v in (a,b)}:
            if math.hypot(pts[i].x-planned_points[i].x,pts[i].y-planned_points[i].y)>1e-8:
                raise ValueError('generated parking candidate changed a neutral endpoint')
        pts=list(planned_points)
    if math.hypot(bundle['points'][-1].x-pts[0].x,bundle['points'][-1].y-pts[0].y)>2.:
        raise ValueError('parking seam outside pinned tolerance')
    result=dict(bundle);base=len(bundle['points'])
    result['points']=bundle['points']+pts
    result['labels']=bundle['labels']+[f'{branch}:candidate_sample:{i}' if planned_points is not None else f'{branch}:{i}' for i in range(len(pts))]
    result['s']=qr.cumulative_s(result['points']);ss=result['s']
    gates=list(bundle['gates'][:-1])
    specs=PARKING_SEGMENTS[branch]
    for k,(begin,end,direction) in enumerate(specs):
        stop_index=base-1 if k==0 else base+begin
        gates.append(Gate(f'{name}_gear_{k}','gear',ss[stop_index],ss[stop_index],
                          direction=direction,stage_start=base+begin,stage_end=base+end,approach_m=.35))
    end=base+specs[-1][1]
    gates.append(Gate(name,'transition',ss[end],ss[end],next_section=NEXT_RECOVERY[name],approach_m=.35))
    result['gates']=gates;result['section']='parking_'+branch
    result['hashes']={**bundle['hashes'],str((TRACK/(branch+'.csv')).relative_to(ROOT)):digest(TRACK/(branch+'.csv'))}
    result['parking_branch']=branch
    return result


def append_common(bundle,section):
    if section not in ('after_t','after_p'):raise ValueError('normal parking exit must name the following common section')
    following=build_section(section,bundle['lane'],bundle['config'],None,bundle.get('session_choices'))
    points=following['points'];base=len(bundle['points'])
    if math.hypot(bundle['points'][-1].x-points[0].x,bundle['points'][-1].y-points[0].y)>2.:
        raise ValueError('normal section seam outside pinned tolerance')
    result=dict(bundle);result['points']=bundle['points']+points
    result['labels']=bundle['labels']+following['labels'];result['s']=qr.cumulative_s(result['points'])
    offset=result['s'][base]
    from dataclasses import replace
    result['gates']=bundle['gates']+[replace(g,stop_s=g.stop_s+offset,commit_s=g.commit_s+offset) for g in following['gates']]
    result['hashes']={**bundle['hashes'],**following['hashes']};result['section']=section
    policy=following.get('preentry_policy')
    result['preentry_policy']=None if policy is None else {**policy,'start_index':policy['start_index']+base,'start_s':policy['start_s']+offset}
    return result,base


def replace_local_path(bundle,begin,path):
    """Keep raw label identity and remap all mission distances after a local detour."""
    import bisect
    from dataclasses import replace
    if len(path)<3 or begin<0 or begin+len(path)>len(bundle['points']):raise ValueError('local path window invalid')
    result=dict(bundle);points=list(bundle['points']);points[begin:begin+len(path)]=path
    old=bundle['s'];new=qr.cumulative_s(points)
    def remap(distance):
        if distance<=old[0]:return distance
        i=min(len(old)-2,bisect.bisect_right(old,distance)-1)
        if old[i+1]-old[i]<1e-8:return new[i]
        return new[i]+(distance-old[i])/(old[i+1]-old[i])*(new[i+1]-new[i])
    result['points']=points;result['s']=new
    result['gates']=[replace(g,stop_s=remap(g.stop_s),commit_s=remap(g.commit_s)) for g in bundle['gates']]
    if bundle.get('preentry_policy'):
        result['preentry_policy']={**bundle['preentry_policy'],'start_s':remap(bundle['preentry_policy']['start_s'])}
    return result


def save_bundle(bundle):
    from dataclasses import asdict
    return {**bundle,'points':[asdict(p) for p in bundle['points']],'gates':[asdict(g) for g in bundle['gates']]}


def restore_bundle(value):
    result=dict(value)
    result['points']=[qr.Point(**p) for p in value['points']]
    result['gates']=[Gate(**g) for g in value['gates']]
    if len(result['points'])!=len(result['labels']) or qr.cumulative_s(result['points'])!=result['s']:
        raise ValueError('saved route distances/labels inconsistent')
    return result
