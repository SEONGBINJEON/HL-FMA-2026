#!/usr/bin/env python3
"""Qualifier entry for HYPER courses: explicit course choice, lane-aware entry, straight-wheel check, short
forward heading bootstrap, adaptive-lookahead approach, then tracking to the chosen course end.

[2026-09-19 Claude5] Built on tools/live_path_follow.py (kept unmodified as the proven version). Reused from it:
DriveGate, cap_left_wheel, argument parser. Added here only:
  * --course bend|s  : course slice of common_1_hyper.csv (FILE indices). For 'bend' the recorded HYPER start
                       lanes start_left.csv / start_right.csv (sha-verified via route_candidates.json) are the
                       entry paths before the merge; after the merge the common section is used. --lane
                       auto|left|right|common; 'auto' refuses ambiguous lane placement instead of guessing.
                       Rejoin is limited to the entry range so no other lane/course can be picked and no
                       mandatory part is skipped. --resume-index N rejoins at exactly one common segment.
  * start sequence:  'p' (or --control-file containing p) -> board prepare -> steer target 0 (ADC 477) with
                       drive 0 -> WAIT until the board ADC is inside the calibration stop band (no wait if already
                       straight; timeout stops) -> straight low-speed forward for a bounded distance/time ->
                       heading from THIS forward window's fresh RTK FIXED rows only -> lane/segment entry -> run.
                       Too little displacement stops the vehicle; no heading is invented.
                       --heading-mode manual --initial-heading-deg X uses an operator-measured body direction,
                       anchored at 'p' (not at program start), so waiting cannot expire it.
  * adaptive lookahead max(lookahead_m, gain*|lateral|) capped at --lookahead-max-m, returning to lookahead_m
    near the path (Claude4 v2 formula, field-proven approach from 4.3 m).
  * --float-grace-s (default 0 = off): optional short RTK FLOAT tolerance from Claude4 v2 (not field-proven).
  * encoder-freeze stop: drive commanded but board encoder_count unchanged for --encoder-stall-s -> stop
    (physical cutoff seen on 2026-09-19 drive5: actual_drive stayed 1, encoder froze for 20 s).
  * RTK session freshness check at start: the gga.jsonl tail must be recent; an ended folder is refused.
No hardware is opened by imports, --check-only or --dry-run. Drive defaults to zero. NOT yet field-verified.
"""
import dataclasses
import json
import math
import sys
import time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import joystick_protocol as jp  # noqa: F401  (same import set as live_path_follow)
import steer_target_host as h
import live_steering_trial as lst
import path_steering as ps
import pose_reference as pr
import qualifier_route as qr
import steering_calibration as sc
import live_path_follow as lpf
from path_follow_inputs import LatestRows, Heading, utc_seconds, rear_axle_points

DEFAULT_ROUTE='data/daegu_scurve_hyper_20260919_045730/qual_route_candidate/common_1_hyper.csv'
DEFAULT_CAL='data/claude3_follow_20260919_020600/drive_calibration_full_table.json'

# File indices of common_1_hyper.csv (original common_1 idx 172 removed; idx<=171 identical, above = original-1).
COURSES={
 'bend':{'start':0,'end':50,'entry':(0,12),'lead_in_m':0.,'lanes':{'left':'start_left','right':'start_right'},
      'basis':'HYPER s_curve_right_angle.csv(50점,46.9m)이 common_1_hyper idx0~50 위(끝 0.32m). 진입은 HYPER 기록 차선 start_left.csv(89.2m,122점)/start_right.csv(100.0m,122점) 전체가 '
              'common_1 idx0으로 실제 연결(이음새 간격 0.65m/1.79m, 꺾임 좌 1.3°/5.0°·우 −3.7°/2.2°; end_hint120 절단은 좌 꺾임 11.6°라 미사용). 두 차선 간격 3.0~5.4m, 마지막 10m에서 합류. '
              '끝은 3번째 우회전(idx46~58) 중간이라 후보 45/50/59; 공식 판정선 미확인.',
      'candidates':{'start':[0],'end':[45,50,59]}},
 's':{'start':105,'end':182,'entry':(105,140),'lead_in_m':0.,'lanes':{},
      'basis':'HYPER s_curve.csv(65점,52.5m)이 common_1_hyper idx114~182 위에 중앙값0.20m/최대0.66m로 겹침. idx105~113은 같은 차선 접근 직선. 라벨 s_curve_start=136 후보, end=183(=파일182). [예선 굴절 확정으로 준비 중단]',
      'candidates':{'start':[114,136],'end':[182]}},
}
LANE_AMBIGUITY_M=1.5
TRACK_DIR=ROOT/'reference/external/HYPER/src/planning/hyper_waypoint/waypoints/track'
COURSES['track']={'start':0,'end':1,'entry':(0,0),'lead_in_m':0.,'lanes':{},'basis':'generic HYPER track csv chosen with --track; indices are that CSV row indices (no noise removal)','candidates':{}}


def build_parser():
    p=lpf.get_parser()
    p.description=__doc__
    for act in p._actions:  # proven 2026-09-19 field values become defaults; route/calibration get defaults
        if act.dest=='route':act.required=False;act.default=str(ROOT/DEFAULT_ROUTE)
        if act.dest=='calibration':act.required=False;act.default=str(ROOT/DEFAULT_CAL)
        if act.dest=='max_steer_deg':act.default=25.
        if act.dest=='max_steer_right_deg':act.default=19.
        if act.dest=='heading_max_age_s':act.default=15.
        if act.dest=='max_correction_age_s':act.default=10.
        if act.dest=='start_distance_m':act.default=5.
        if act.dest=='max_lateral_error_m':act.default=5.
        if act.dest=='max_seconds':act.default=300.
        if act.dest=='angle_model':act.default='ideal-ackermann'
    p.add_argument('--course',choices=tuple(COURSES),required=True,help='qualifier course slice of the route file; track = any HYPER csv via --track')
    p.add_argument('--track',help="HYPER track name for --course track (e.g. common_2, common_3, end_left); same map datum as common_1_hyper")
    p.add_argument('--lane',choices=('auto','left','right','common'),default='auto',help='entry lane before the merge; auto refuses ambiguity')
    p.add_argument('--course-start',type=int,help='override course start FILE index')
    p.add_argument('--course-end',type=int,help='override course end FILE index (bend stop candidates 45/50/59; official line unconfirmed)')
    p.add_argument('--entry-from',type=int,help='override entry range first FILE index')
    p.add_argument('--entry-to',type=int,help='override entry range last FILE index')
    p.add_argument('--lead-in-m',type=float,help='straight extension before the course start (default per course)')
    p.add_argument('--resume-index',type=int,help='explicit middle resume: rejoin only at this common FILE segment index')
    p.add_argument('--heading-mode',choices=('bootstrap','manual'),default='bootstrap')
    p.add_argument('--straighten-max-s',type=float,default=6.,help='max wait for the wheels to reach the straight target before moving')
    p.add_argument('--bootstrap-distance-m',type=float,default=1.5,help='forward travel target for the heading bootstrap')
    p.add_argument('--bootstrap-min-m',type=float,default=1.0,help='minimum displacement to accept a heading')
    p.add_argument('--bootstrap-max-s',type=float,default=8.,help='forward time limit; stop if not reached')
    p.add_argument('--bootstrap-max-dev-m',type=float,default=.3,help='max perpendicular scatter of the bootstrap fixes')
    p.add_argument('--lookahead-gain',type=float,default=1.5)
    p.add_argument('--lookahead-max-m',type=float,default=6.)
    p.add_argument('--float-grace-s',type=float,default=0.,help='0 = off (default): RTK FLOAT stops immediately as before')
    p.add_argument('--encoder-stall-s',type=float,default=3.,help='drive commanded but encoder frozen this long -> stop')
    p.add_argument('--gga-fresh-s',type=float,default=10.,help='rtk session gga.jsonl tail must be newer than this at start')
    p.add_argument('--control-file',help="optional file polled for a single letter: 'p' start (consumed), 's' stop. Keys still work.")
    p.add_argument('--end-offset-m',type=float,default=0.,help='trim the route so it ends this far BEFORE the last labelled point (antenna->front bumper 0.725 + margin)')
    p.add_argument('--stop-at',help="planned mission stop at a route label (e.g. left:52): drive 0 + brake, hold, wait for 'c' in the control file")
    p.add_argument('--stop-offset-m',type=float,default=0.,help='planned stop target this far BEFORE the label point along the path')
    p.add_argument('--stop-lead-m',type=float,default=.5,help='cut drive this far before the planned stop target (measured overshoot goes here)')
    p.add_argument('--hold-min-s',type=float,default=3.,help='minimum planned-stop hold before a continue is accepted')
    p.add_argument('--heading-test-only',action='store_true',help='TEST MODE: p -> straighten -> short forward -> heading -> STOP. No course proximity needed, no entry, no following.')
    return p


def course_spec(a):
    c=COURSES[a.course]
    if a.course=='track':
        if not a.track or a.course_end is None:raise ValueError('--course track needs --track NAME and --course-end (and usually --course-start)')
        if not (TRACK_DIR/f'{a.track}.csv').exists():raise ValueError(f'unknown track {a.track}')
        c=dict(c,start=0 if a.course_start is None else a.course_start,end=a.course_end)
        c['entry']=(c['start'],c['end']-1)
    elif a.track:raise ValueError('--track only with --course track')
    start=c['start'] if a.course_start is None else a.course_start
    end=c['end'] if a.course_end is None else a.course_end
    e0,e1=c['entry'];e0=e0 if a.entry_from is None else a.entry_from;e1=e1 if a.entry_to is None else a.entry_to
    lead=c['lead_in_m'] if a.lead_in_m is None else a.lead_in_m
    if not 0<=start<end:raise ValueError('course start must be before end')
    if not (start<=e0<=e1<end):raise ValueError('entry range must lie inside the course (before its last segment)')
    if not math.isfinite(lead) or lead<0 or lead>30:raise ValueError('invalid lead-in')
    if a.resume_index is not None and not start<=a.resume_index<end:raise ValueError('resume index outside the course')
    lanes=dict(c['lanes'])
    if a.lane in ('left','right') and a.lane not in lanes:raise ValueError(f'course {a.course} has no recorded lane {a.lane}')
    if not math.isfinite(a.end_offset_m) or a.end_offset_m<0 or a.end_offset_m>5:raise ValueError('invalid end offset')
    return {'course':a.course,'start':start,'end':end,'entry':(e0,e1),'lead_in_m':lead,'basis':c['basis'],'candidates':c['candidates'],
            'lanes':lanes,'lane_choice':a.lane,'end_offset_m':a.end_offset_m}


def load_lane_tracks(spec,antenna_offset):
    """HYPER start lanes from route_candidates.json (sha256 verified by qualifier_route). Same HYPER map frame as common_1."""
    if not spec['lanes']:return {}
    cands=qr.load_candidates();tracks=qr.load_all_tracks(cands);out={}
    for lane,name in spec['lanes'].items():
        t=tracks[name];out[lane]={'points':rear_axle_points(t.points,'rear_axle_center',antenna_offset),'sha256':t.sha256,'path':t.path,
                                  'end_hint':cands['profiles'][name].get('upstream_start_segment_end_index_hint',len(t.points)-1)}
    return out


def slice_course(points,spec,step_m=.5):
    """Common course points with optional straight lead-in. Returns (points, labels): labels are common FILE ints or 'lead:-k'."""
    seg=points[spec['start']:spec['end']+1];a,b=seg[0],seg[1]
    hd=math.atan2(b.y-a.y,b.x-a.x);lead=[]
    n=int(spec['lead_in_m']/step_m+1e-9)
    for k in range(n,0,-1):
        d=k*step_m
        lead.append(qr.Point(-k,a.x-d*math.cos(hd),a.y-d*math.sin(hd),hd,'lead_in',a.frame_id))
    return lead+list(seg),[f'lead:{-k}' for k in range(n,0,-1)]+list(range(spec['start'],spec['end']+1))


def allowed_entry_indices(labels,spec,resume_index=None):
    """Local segment indices where rejoin is permitted: lane points, lead-in, and the common entry range."""
    if resume_index is not None:
        return [labels.index(resume_index)]
    e0,e1=spec['entry']
    return [i for i,f in enumerate(labels[:-1]) if isinstance(f,str) or e0<=f<=e1]


def build_routes(common_points,spec,lanes,resume_index=None):
    """Candidate routes: each recorded lane + common course, and the common course alone (after the merge)."""
    cpts,clabels=slice_course(common_points,spec)
    routes={}
    names=['common'] if spec['lane_choice']=='common' else ([spec['lane_choice']] if spec['lane_choice'] in lanes else list(lanes)+['common'])
    if resume_index is not None:names=['common']
    for name in names:
        if name=='common':pts,labels=cpts,clabels
        else:
            lp=lanes[name]['points']   # full recorded lane: the seam to common_1 idx0 is smoother than at the mission end_hint cut
            pts=list(lp)+cpts;labels=[f'{name}:{i}' for i in range(len(lp))]+clabels
        pts,labels=trim_end(pts,labels,spec.get('end_offset_m',0.))
        allowed=[i for i in allowed_entry_indices(labels,spec,resume_index) if i<len(pts)-1]
        seam=None
        if name!='common':
            n=len(lp);a,b=pts[n-1],pts[n];h_seam=math.atan2(b.y-a.y,b.x-a.x)
            seam={'gap_m':round(math.hypot(b.x-a.x,b.y-a.y),2),'turn_lane_to_seam_deg':round(math.degrees(qr.wrap(h_seam-qr.path_heading_at(pts,n-2))),1),
                  'turn_seam_to_common_deg':round(math.degrees(qr.wrap(qr.path_heading_at(pts,n)-h_seam)),1)}
        routes[name]={'points':pts,'labels':labels,'allowed':allowed,'lane_points':0 if name=='common' else len(lp),'seam':seam,
                      'length_m':round(qr.cumulative_s(pts)[-1],2)}
    return routes


def trim_end(points,labels,offset_m):
    """Cut the route so it ends offset_m before its last point (interpolated on the path). offset 0 -> unchanged."""
    if offset_m<=0:return points,labels
    s=qr.cumulative_s(points);target=s[-1]-offset_m
    if target<=0:raise ValueError('end offset longer than the route')
    i=max(k for k in range(len(points)-1) if s[k]<=target)
    a,b=points[i],points[i+1];t=(target-s[i])/max(1e-9,s[i+1]-s[i])
    end=qr.Point(a.idx,a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),math.atan2(b.y-a.y,b.x-a.x),'end_offset',a.frame_id)
    return points[:i+1]+[end],labels[:i+1]+[f'{labels[-1]}-{offset_m:.2f}m']


def along_track(points,i,pose):
    """Cumulative path distance of the pose projected onto segment i."""
    a,b=points[i],points[min(i+1,len(points)-1)];dx,dy=b.x-a.x,b.y-a.y;ll=dx*dx+dy*dy
    t=0. if ll<1e-10 else max(0.,min(1.,((pose.x-a.x)*dx+(pose.y-a.y)*dy)/ll))
    return qr.cumulative_s(points)[i]+t*math.sqrt(ll)


def project(points,i,pose):
    a,b=points[i:i+2];dx,dy=b.x-a.x,b.y-a.y;ll=dx*dx+dy*dy
    if ll<1e-10:return None
    t=max(0.,min(1.,((pose.x-a.x)*dx+(pose.y-a.y)*dy)/ll))
    return math.hypot(pose.x-a.x-t*dx,pose.y-a.y-t*dy),math.atan2(dy,dx),t


def select_entry(points,allowed,pose,max_distance,max_heading_deg):
    """Nearest allowed same-direction segment (distance + heading penalty), as select_rejoin does on the full route."""
    cands=[]
    for i in allowed:
        pj=project(points,i,pose)
        if pj is None:continue
        d,ph,_=pj;err=abs(qr.wrap(pose.yaw-ph))
        if err>math.radians(max_heading_deg) or d>max_distance:continue
        cands.append((d+err*.25,d,i,err))
    if not cands:raise ValueError('no allowed entry segment within distance/heading; place the vehicle at the course entry or use --resume-index')
    cands.sort();return cands[0][2],cands[0][1],cands[0][3]


def choose_route(routes,pose,max_distance,max_heading_deg):
    """Pick the lane/common route whose allowed entry fits best. Ambiguous lane placement is refused, never guessed.
    Returns (name, local_index, distance, heading_err)."""
    found={}
    for name,r in routes.items():
        try:found[name]=select_entry(r['points'],r['allowed'],pose,max_distance,max_heading_deg)
        except ValueError:pass
    if not found:raise ValueError('no candidate lane/common segment within distance/heading; check placement or choose --lane')
    def on_lane(name):return name!='common' and found[name][0]<routes[name]['lane_points']
    lane_hits={n:v for n,v in found.items() if on_lane(n)}
    if 'common' in found and any(not on_lane(n) for n in found if n!='common'):
        for n in list(found):
            if n!='common' and not on_lane(n):del found[n]   # entry on the common part: use the common route itself
    if len(lane_hits)>=2:
        ds=sorted(v[1] for v in lane_hits.values())
        if ds[1]-ds[0]<LANE_AMBIGUITY_M:
            raise ValueError('lane ambiguous: '+', '.join(f'{n} {v[1]:.2f}m' for n,v in lane_hits.items())+f' (difference < {LANE_AMBIGUITY_M}m); rerun with --lane left|right')
    name=min(found,key=lambda n:found[n][1]+found[n][2]*.25);i,d,err=found[name]
    return name,i,d,err


def position_preview(points,allowed,x,y):
    """Position-only view of the entry candidates (before any heading exists)."""
    best=None
    for i in allowed:
        pj=project(points,i,type('P',(),{'x':x,'y':y})())
        if pj is None:continue
        if best is None or pj[0]<best[0]:best=(pj[0],i,pj[1])
    return best


def preview_all(routes,x,y):
    out={}
    for name,r in routes.items():
        pv=position_preview(r['points'],r['allowed'],x,y)
        if pv:out[name]={'lateral_m':round(pv[0],2),'label':r['labels'][pv[1]],'path_heading_deg':round(math.degrees(pv[2]),1),'on_lane':pv[1]<r['lane_points']}
    return out


class EntryState(qr.RunState):
    def __init__(self,params):
        super().__init__(params);self.allowed=[];self.entry=None
    def arm(self,pose):
        if self.state!=self.MODE_SELECTED:return False
        bad=qr.pose_problem(pose)
        if bad or pose.age_s>self.params.pose_max_age_s:
            self.last_reason=bad or 'pose stale';return False
        try:i,d,err=select_entry(self.route.points,self.allowed,pose,self.params.max_distance_m,self.params.max_heading_deg)
        except ValueError as exc:self.last_reason=str(exc);return False
        self.progress=i;self.state=self.READY;self.entry=(i,d,err)
        self.last_reason=f'entry segment {i}, distance {d:.2f}m, heading error {math.degrees(err):.0f}deg';return True


def bootstrap_heading(fixes,min_m,max_dev_m):
    """Heading from one straight forward window: fixes=[(x,y,stamp)] collected ONLY while drive was commanded.
    Returns (yaw_rad, moved_m, max_dev_m). Raises when displacement or straightness is insufficient."""
    if len(fixes)<3:raise ValueError(f'bootstrap: only {len(fixes)} fresh FIXED rows; no heading')
    (x0,y0,_),(x1,y1,_)=fixes[0],fixes[-1];dist=math.hypot(x1-x0,y1-y0)
    if dist<min_m:raise ValueError(f'bootstrap moved only {dist:.2f}m < {min_m}m; no heading, stop')
    ux,uy=(x1-x0)/dist,(y1-y0)/dist
    dev=max(abs(-(x-x0)*uy+(y-y0)*ux) for x,y,_ in fixes)
    if dev>max_dev_m:raise ValueError(f'bootstrap fixes not straight (scatter {dev:.2f}m > {max_dev_m}m); no heading, stop')
    return math.atan2(y1-y0,x1-x0),dist,dev


def straight_reached(adc,target_adc,band):
    """Board ADC inside the calibration stop band around the straight target: no waiting when already there."""
    return adc is not None and abs(int(adc)-int(target_adc))<=int(band)


def adaptive_lookahead(base,gain,cap,lateral):
    return max(base,min(cap,gain*abs(lateral)))


def gga_tail_age(path,now):
    """Age (s) of the newest gga.jsonl row and its fix_status; (None,None) when unreadable."""
    p=Path(path)/'gga.jsonl'
    if not p.exists() or p.stat().st_size==0:return None,None
    with p.open('rb') as f:
        f.seek(max(0,p.stat().st_size-8192));tail=f.read().split(b'\n')
    for line in reversed(tail):
        if line.strip():
            try:row=json.loads(line)
            except ValueError:continue
            return now-utc_seconds(row['host_utc']),row.get('fix_status')
    return None,None


def control_file_poll(path):
    """Return ['p'] / ['s'] from a one-letter control file ('p' is consumed so it cannot restart anything)."""
    if not path:return []
    p=Path(path)
    if not p.exists():return []
    try:text=p.read_text().strip().lower()
    except OSError:return []
    if text=='p':
        try:p.unlink()
        except OSError:pass
        return ['p']
    if text in ('s','stop','q'):return ['s']
    if text in ('c','continue'):
        try:p.unlink()
        except OSError:pass
        return ['c']
    return []


class EncoderWatch:
    """Stop condition: drive frames are being sent but the board encoder does not move."""
    def __init__(self,stall_s):self.stall_s=stall_s;self.value=None;self.changed_at=None
    def reset(self):self.value=None;self.changed_at=None
    def check(self,status,driving,now):
        if not driving or status is None:self.reset();return None
        enc=status.encoder_count
        if self.value is None or enc!=self.value:self.value=enc;self.changed_at=now;return None
        if now-self.changed_at>self.stall_s:return f'drive commanded but encoder frozen at {enc} for {now-self.changed_at:.1f}s (physical cutoff or stall): stop'
        return None


def make_pose(a,x,y,yaw,wall,stamp):
    return pr.rear_axle_pose(x=x,y=y,heading=yaw,heading_convention=pr.HEADING_ENU_CCW_RAD,map_yaw_from_enu_rad=0.,frame_id='map',
         frame_alignment_verified=True,reference_point='gnss_antenna',position_valid=True,heading_valid=True,position_age_s=wall-stamp,
         heading_age_s=0.,max_age_s=a.pose_max_age_s,planar_assumption_verified=True,offset_x_m=a.antenna_offset_m,offset_y_m=0.)


def main(argv=None):
    a=build_parser().parse_args(argv)
    for f in ('lookahead_m','max_lateral_error_m','max_steer_deg','pose_max_age_s','heading_max_age_s','heading_window_m',
              'max_correction_age_s','start_distance_m','wheelbase_m','track_m','max_seconds','bootstrap_distance_m','straighten_max_s',
              'bootstrap_min_m','bootstrap_max_s','bootstrap_max_dev_m','lookahead_gain','lookahead_max_m','encoder_stall_s','gga_fresh_s'):
        if not math.isfinite(getattr(a,f)) or getattr(a,f)<=0:raise ValueError('invalid '+f)
    if not math.isfinite(a.float_grace_s) or a.float_grace_s<0:raise ValueError('invalid float_grace_s')
    if a.bootstrap_min_m>a.bootstrap_distance_m:raise ValueError('bootstrap min must not exceed target distance')
    if a.lookahead_max_m<a.lookahead_m:raise ValueError('lookahead max below base')
    if (not 0<a.max_steer_deg<60 or not 0<a.arm_heading_deg<=90 or not 0<a.run_heading_deg<=90
            or not 0<=a.antenna_offset_m<2 or not 0<=a.drive_pwm<=255):raise ValueError('invalid physical/heading limits')
    if a.max_steer_right_deg is None:a.max_steer_right_deg=a.max_steer_deg
    if not 0<a.max_steer_right_deg<60:raise ValueError('invalid max_steer_right_deg')
    if a.heading_mode=='manual' and (a.initial_heading_deg is None or not math.isfinite(a.initial_heading_deg)):
        raise ValueError('--heading-mode manual needs --initial-heading-deg (measured actual body direction, ENU east0 north90)')
    if a.heading_mode=='bootstrap' and a.initial_heading_deg is not None:raise ValueError('bootstrap mode ignores no input: drop --initial-heading-deg or use --heading-mode manual')
    if a.heading_test_only and a.heading_mode!='bootstrap':raise ValueError('--heading-test-only is a bootstrap test')
    if a.heading_mode=='bootstrap' and a.dry_run:raise ValueError('dry-run cannot drive the bootstrap; use --heading-mode manual for a dry-run')
    if a.imu_session or a.imu_yaw_sign is not None:raise ValueError('IMU input is out of scope for this entry point')
    for f in ('stop_offset_m','stop_lead_m','hold_min_s'):
        if not math.isfinite(getattr(a,f)) or getattr(a,f)<0:raise ValueError('invalid '+f)
    if a.stop_at and not a.control_file:raise ValueError('--stop-at needs --control-file (continue is the letter c)')
    spec=course_spec(a)
    pre=lst.preflight(a.calibration,a.max_steer_deg,port=None if a.dry_run or a.check_only else a.port)
    negative=lst.preflight(a.calibration,-a.max_steer_right_deg,port=None)
    if not pre['live_capable'] or not negative['live_capable']:
        print(json.dumps({'positive':pre['missing'],'negative':negative['missing']},ensure_ascii=False));return lst.EXIT_BLOCKED
    meta=json.loads(Path(a.route).with_suffix('.meta.json').read_text())
    if a.course=='track':
        track=qr.load_track_csv(a.track,TRACK_DIR/f'{a.track}.csv');a.route=str(TRACK_DIR/f'{a.track}.csv')   # HYPER csv, same datum as the meta origin
        meta=dict(meta,reference='rear_axle_center')
    else:track=qr.load_track_csv('manual',Path(a.route))
    if meta.get('frame')!='map':raise ValueError('route frame mismatch')
    lat0,lon0=meta['origin_lat'],meta['origin_lon']
    if not (-90<lat0<90 and -180<=lon0<=180):raise ValueError('invalid local origin')
    if spec['end']>=len(track.points):raise ValueError('course end beyond route file')
    full=rear_axle_points(track.points,meta.get('reference'),a.antenna_offset_m)
    lanes=load_lane_tracks(spec,a.antenna_offset_m)
    routes=build_routes(full,spec,lanes,a.resume_index)
    summary={'course':spec['course'],'lane_choice':spec['lane_choice'],'file_index_range':[spec['start'],spec['end']],'entry_file_range':list(spec['entry']),
             'resume_index':a.resume_index,'lead_in_m':spec['lead_in_m'],
             'routes':{n:{'points':len(r['points']),'lane_points':r['lane_points'],'length_m':r['length_m'],'allowed_entry_segments':len(r['allowed']),'seam':r['seam']} for n,r in routes.items()},
             'lanes':{n:{'source':l['path'],'sha256_16':l['sha256'][:16],'end_hint':l['end_hint']} for n,l in lanes.items()},
             'basis':spec['basis'],'boundary_candidates_file_idx':spec['candidates'],'official_finish_confirmed':False,
             'route_reference_declared':meta.get('reference'),'heading_mode':a.heading_mode,
             'start_sequence':['p','board prepare','steer 0 -> wait ADC in stop band' if a.heading_mode=='bootstrap' else 'operator heading anchored at p',
                               f'forward <= {a.bootstrap_distance_m}m / {a.bootstrap_max_s}s at PWM {a.drive_pwm}' if a.heading_mode=='bootstrap' else '-',
                               'heading from that window' if a.heading_mode=='bootstrap' else '-','lane/segment entry','follow to course end'],
             'heading_test_only':a.heading_test_only,'field_verified':False}
    selection=qr.SelectionParams(a.start_distance_m,a.arm_heading_deg,max(r['length_m'] for r in routes.values()),1.5,a.pose_max_age_s,'explicit course entry range')
    state=EntryState(selection)
    cfg=ps.PurePursuitConfig(a.wheelbase_m,'user field dimensions; provisional',a.lookahead_m,a.max_lateral_error_m,
                           math.radians(a.run_heading_deg),math.radians(85),a.pose_max_age_s,.3,.5,'rear_axle_center','provisional field controller')
    follower=ps.PathFollower(state,cfg,lambda s:int(s*1000)&0xffffffff)
    print(json.dumps({'status':'COURSE_SELECTED',**summary},ensure_ascii=False),flush=True)
    if a.check_only:return 0
    age,fix=gga_tail_age(a.rtk_session,time.time())
    if age is None or age>a.gga_fresh_s:
        print(json.dumps({'status':'RTK_SESSION_NOT_LIVE','gga_tail_age_s':age,'fix_status':fix,'hint':'start a NEW rtk_record.py session folder and check fresh FIXED rows'},ensure_ascii=False));return lst.EXIT_BLOCKED
    print(f'RTK session live: newest GGA {age:.1f}s old, fix_status={fix}',flush=True)
    inputs=LatestRows(Path(a.rtk_session)/'gga.jsonl')
    heading=Heading(a.heading_window_m,None,None,None)   # anchored at 'p' (manual) or after bootstrap; never at program start
    details={'route':a.route,'route_sha256':lst.sha256_file(a.route),'route_meta_sha256':lst.sha256_file(Path(a.route).with_suffix('.meta.json')) if Path(a.route).with_suffix('.meta.json').exists() else None,
             'args':vars(a),'drive_pwm':a.drive_pwm,'steering_only':not bool(a.drive_pwm),'provisional_angle_model':a.angle_model,
             'qualifier':summary,'steer_cap_left_deg':a.max_steer_deg,'steer_cap_right_deg':a.max_steer_right_deg,
             'new_code_sha256':{f:lst.sha256_file(ROOT/f) for f in ('tools/qualifier_follow.py','tools/live_path_follow.py','tools/path_follow_inputs.py')}}
    journal=lst.TrialJournal(Path(a.output_dir),'model_calculation' if a.dry_run else 'live_vehicle',
                            lst.build_metadata('qualifier_follow',a.calibration,pre,int(a.max_steer_deg*100),details,argv))
    transport=trial=gate=keys=None;phase='waiting';last_stamp=None;last_fix=None;last_issued_stamp=None;bad_since=None
    boot_fixes=[];boot_t0=None;last_cmd=0.;straight_t0=None;encoder=EncoderWatch(a.encoder_stall_s)
    points=labels=None;chosen=None;target_adc=None;band=None
    stop_s=None;stop_done=False;hold_t0=None;hold_pos=None;hold_enc=None;hold_wheel=0;hold_last_cmd=0.;hold_max_drift=0.;stop_cut_pos=None
    stats={'fixes_seen':0,'calculated_targets':0,'accepted_targets':0,'max_lateral_error_m':None,'saturated_targets':0,
           'float_grace_rows':0,'max_lookahead_m':None,'straighten':None,'bootstrap':None,'entry':None,'planned_stop':None}
    outcome='timeout';reason='time budget expired without arrival';code=1;t0=time.monotonic();next_notice=0.;next_telemetry=0.
    try:
        keys=lst.StdinKeys()
        if not a.dry_run:
            transport=lst.PosixSerialTransport(a.port,check_identity=True)
            trial=lst.assemble(h.load_library(),a.calibration,int(min(a.max_steer_deg,a.max_steer_right_deg)*100),transport,journal,allow_synthetic=False)
            gate=lpf.DriveGate(a.drive_pwm,trial.adapter);trial.client.encoder=gate
            target_adc=sc.angle_to_adc(trial.validated['points'],0);band=trial.validated['restart_band_adc']  # same band the steering core uses to decide 'close enough'
        if a.heading_mode=='bootstrap':
            print(f'p: START = board prepare -> steer straight (ADC {target_adc}, wait until within {band}) -> forward <= {a.bootstrap_distance_m}m / {a.bootstrap_max_s}s at PWM {a.drive_pwm} '
                  'for heading -> lane/segment entry -> follow. Space ahead must be clear. s/space/q/Ctrl+C: STOP. No automatic restart.',flush=True)
        else:
            print(f'p: START with operator heading {a.initial_heading_deg}deg (anchored at p) -> entry -> follow. s/space/q/Ctrl+C: STOP.',flush=True)
        while time.monotonic()-t0<a.max_seconds:
            now=time.monotonic();wall=time.time();pressed=keys.poll()+control_file_poll(a.control_file)
            if any(k in ('s',' ','q') for k in pressed):outcome='operator_stop';reason='operator stop';code=130;break
            rows=inputs.poll();stats['fixes_seen']+=len(rows)
            for row in rows:
                stamp=utc_seconds(row['host_utc'])
                if last_stamp is not None and stamp<=last_stamp:
                    if stamp<last_stamp:raise ValueError('GNSS timestamp regressed')
                    continue
                last_stamp=stamp
                if not 0<=wall-stamp<=a.pose_max_age_s:last_fix=None;continue
                age=row.get('differential_age_s')
                age_ok=isinstance(age,(int,float)) and math.isfinite(age) and 0<=age<=a.max_correction_age_s
                good=row.get('fix_status')=='rtk_fixed' and row.get('checksum_valid') is True and age_ok
                if not good:
                    floaty=(a.float_grace_s>0 and row.get('fix_status')=='rtk_float' and row.get('checksum_valid') is True and age_ok)
                    if phase=='running' and floaty:
                        if bad_since is None:
                            bad_since=stamp;journal.event('gnss_grace',start_utc=row['host_utc'],fix_status=row.get('fix_status'))
                            print(f'GNSS FLOAT: grace up to {a.float_grace_s}s',flush=True)
                        if stamp-bad_since>a.float_grace_s:raise ValueError(f'GNSS FLOAT longer than {a.float_grace_s}s')
                        stats['float_grace_rows']+=1
                    elif phase!='waiting':raise ValueError('GNSS quality/correction age invalid')
                    else:last_fix=None;continue
                else:bad_since=None
                lat,lon=row['latitude_deg'],row['longitude_deg']
                if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (lat,lon)):raise ValueError('invalid GNSS coordinates')
                x=(lon-lon0)*111320*math.cos(math.radians(lat0));y=(lat-lat0)*111320
                last_fix=(x,y,stamp)
                if phase=='bootstrap':boot_fixes.append(last_fix)
                if phase=='running':heading.gps(x,y,stamp)
            pos_ok=last_fix is not None and 0<=wall-last_fix[2]<=a.pose_max_age_s
            if trial:
                if not trial.tick(time.monotonic()):raise ValueError(trial.exit_reason or 'board transport stopped')
                if phase!='waiting' and trial.adapter.locked_by_fault:raise ValueError(trial.adapter.first_fault or 'board fault')
                driving=gate is not None and gate.allowed_until>now and phase in ('bootstrap','running')
                frozen=encoder.check(trial.adapter.status,driving,now)
                if frozen:raise ValueError(frozen)
            if not pos_ok:
                if gate:gate.close()
                if phase!='waiting':raise ValueError('position invalid or stale; restart explicitly at current position')
                if now>=next_notice:print('Waiting: fresh RTK FIXED required.',flush=True);next_notice=now+2
                time.sleep(.01);continue
            x,y,stamp=last_fix
            if phase=='waiting':
                if now>=next_notice:
                    pv=preview_all(routes,x,y)
                    print('Entry preview: '+json.dumps(pv,ensure_ascii=False)+(f' | operator heading {a.initial_heading_deg}deg' if a.heading_mode=='manual' else '')+' | press p to start.',flush=True)
                    next_notice=now+2
                if 'p' in pressed or a.dry_run:
                    pv=preview_all(routes,x,y);reach=a.start_distance_m+(a.bootstrap_distance_m if a.heading_mode=='bootstrap' else 0.)
                    near=[v['lateral_m'] for v in pv.values() if v['lateral_m']<=reach]
                    if not near and not a.heading_test_only:
                        print(f'Entry refused: no candidate segment within {reach}m ({json.dumps(pv)}). Not starting.',flush=True);continue
                    if trial and not trial.adapter.request_prepare(time.monotonic()):
                        print('Board is not stopped/ready for prepare.',flush=True);continue
                    phase='preparing';journal.event('start_pressed',preview=pv)
                    print('Preparing board.',flush=True)
            if phase=='preparing' and (trial is None or trial.prepared()):
                if a.heading_mode=='manual':
                    heading.yaw=qr.wrap(math.radians(a.initial_heading_deg));heading.anchor_at=wall
                    heading.gps(x,y,stamp);phase='arming'
                else:
                    if not trial.command_target(0,time.monotonic(),trial.max_hold_s):raise ValueError('straight target rejected: '+(trial.adapter.first_fault or trial.adapter.message))
                    last_cmd=now;straight_t0=now;phase='straighten';journal.event('straighten_start',target_adc=target_adc,band=band)
                    print(f'Straightening: target ADC {target_adc} +/- {band}, drive 0.',flush=True)
            if phase=='straighten':
                st=trial.adapter.status;adc=None if st is None else st.steering_adc
                if straight_reached(adc,target_adc,band):
                    stats['straighten']={'adc':adc,'seconds':round(now-straight_t0,2)};journal.event('straight_reached',**stats['straighten'])
                    print(f'Straight reached: ADC {adc} in {now-straight_t0:.1f}s.',flush=True)
                    phase='bootstrap';boot_fixes=[last_fix];boot_t0=now;encoder.reset();journal.event('bootstrap_start',source_utc=stamp,x=x,y=y)
                    print(f'Bootstrap: straight forward up to {a.bootstrap_distance_m}m / {a.bootstrap_max_s}s.',flush=True)
                elif now-straight_t0>a.straighten_max_s:
                    raise ValueError(f'wheels did not reach straight (ADC {adc} vs {target_adc}+/-{band}) within {a.straighten_max_s}s: stop')
                else:
                    if now-last_cmd>=1.:
                        if not trial.command_target(0,time.monotonic(),trial.max_hold_s):raise ValueError('straight target rejected while straightening')
                        last_cmd=now
                    if now>=next_telemetry:print(f'STRAIGHTEN adc={adc} target={target_adc} t={now-straight_t0:.1f}s',flush=True);next_telemetry=now+1.
            if phase=='bootstrap':
                moved=math.hypot(x-boot_fixes[0][0],y-boot_fixes[0][1]);elapsed=now-boot_t0
                if moved>=a.bootstrap_distance_m or elapsed>=a.bootstrap_max_s:
                    gate.close()
                    yaw,dist,dev=bootstrap_heading(boot_fixes,a.bootstrap_min_m,a.bootstrap_max_dev_m)
                    heading.yaw=yaw;heading.anchor_at=wall;heading.gps(x,y,stamp)
                    stats['bootstrap']={'moved_m':round(dist,2),'seconds':round(elapsed,2),'fixes':len(boot_fixes),'scatter_m':round(dev,3),'heading_deg':round(math.degrees(yaw),1)}
                    journal.event('bootstrap_heading',**stats['bootstrap'],first=boot_fixes[0],last=boot_fixes[-1])
                    print(f"Bootstrap done: moved {dist:.2f}m in {elapsed:.1f}s ({len(boot_fixes)} fixes, scatter {dev:.2f}m) -> heading {math.degrees(yaw):.1f}deg",flush=True)
                    if a.heading_test_only:
                        outcome='heading_test_done';reason='heading test: stop after bootstrap (no entry, no following)';code=0;break
                    phase='arming'
                else:
                    gate.open(now)
                    if now-last_cmd>=1.:
                        if not trial.command_target(0,time.monotonic(),trial.max_hold_s):raise ValueError('straight target rejected during bootstrap')
                        last_cmd=now
                    if now>=next_telemetry:print(f'BOOTSTRAP moved={moved:.2f}m t={elapsed:.1f}s',flush=True);next_telemetry=now+1.
            if phase=='arming':
                pose=make_pose(a,x,y,heading.yaw,wall,stamp).as_route_pose()
                chosen,i,d,err=choose_route(routes,pose,a.start_distance_m,a.arm_heading_deg)
                r=routes[chosen];points,labels=r['points'],r['labels']
                route=qr.ModeRoute(mode='manual',segments=[qr.Segment(f'qualifier_{spec["course"]}_{chosen}',0,len(points)-1,spec['basis'])],points=points)
                state.select_mode(route,True);state.allowed=r['allowed']
                if not state.arm(pose):raise ValueError('entry rejected after heading: '+state.last_reason)
                i,d,err=state.entry;stats['entry']={'route':chosen,'local':i,'label':labels[i],'distance_m':round(d,2),'heading_error_deg':round(math.degrees(err),1)}
                journal.event('entry',route=chosen,index=i,label=labels[i],reason=state.last_reason)
                print(f'ENTRY route={chosen} label={labels[i]} lateral {d:.2f}m heading error {math.degrees(err):.0f}deg; following to common file idx {spec["end"]} ({r["length_m"]}m route).',flush=True)
                if not state.start():raise ValueError(state.last_reason)
                state.params=dataclasses.replace(state.params,max_distance_m=a.max_lateral_error_m,max_heading_deg=a.run_heading_deg)
                phase='running';encoder.reset()
                if a.stop_at:
                    lab_str=[str(l) for l in labels]
                    if a.stop_at not in lab_str:raise ValueError(f'--stop-at {a.stop_at} not on the chosen route')
                    s_all=qr.cumulative_s(points);k=lab_str.index(a.stop_at);stop_s=s_all[k]-a.stop_offset_m
                    if stop_s<=along_track(points,state.progress,pose):raise ValueError('planned stop target is behind the entry point')
                    print(f'Planned stop at {a.stop_at} - {a.stop_offset_m}m = s {stop_s:.1f}m (drive cut {a.stop_lead_m}m before); continue = c in control file after {a.hold_min_s}s.',flush=True)
            if phase=='hold':
                # planned mission stop: drive 0 + brake by the board, steering held, heading anchor frozen (vehicle is not moving).
                pose=make_pose(a,x,y,heading.yaw,wall,stamp)
                drift=math.hypot(pose.x-hold_pos[0],pose.y-hold_pos[1]);hold_max_drift=max(hold_max_drift,drift)
                enc=trial.adapter.status.encoder_count if trial and trial.adapter.status else None
                if now-hold_last_cmd>=1.:
                    if trial and not trial.command_target(hold_wheel,time.monotonic(),trial.max_hold_s):raise ValueError('steer hold target rejected during planned stop')
                    hold_last_cmd=now
                if now>=next_telemetry:
                    print(f'HOLD t={now-hold_t0:.1f}s drift={drift:.2f}m encoder_delta={None if enc is None or hold_enc is None else enc-hold_enc} (c to continue, s to stop)',flush=True)
                    journal.event('hold',t=round(now-hold_t0,2),drift_m=round(drift,3),encoder=enc,pose={'x':pose.x,'y':pose.y});next_telemetry=now+.5
                if 'c' in pressed:
                    if now-hold_t0<a.hold_min_s:print(f'continue ignored: hold {now-hold_t0:.1f}s < {a.hold_min_s}s',flush=True)
                    else:
                        stats['planned_stop']['hold_s']=round(now-hold_t0,2);stats['planned_stop']['max_drift_m']=round(hold_max_drift,3)
                        stats['planned_stop']['settle_overshoot_m']=round(along_track(points,state.progress,pose)-stop_s,2)
                        stats['planned_stop']['encoder_delta']=None if enc is None or hold_enc is None else enc-hold_enc
                        journal.event('continue',**stats['planned_stop']);heading.anchor_at=wall;stop_done=True;phase='running';encoder.reset();last_issued_stamp=None
                        print('CONTINUE: resuming to the route end with the held heading.',flush=True)
            if phase=='running':
                if not heading.valid(wall,a.heading_max_age_s):
                    if gate:gate.close()
                    raise ValueError('heading stale (no fresh GPS course); restart explicitly at current position')
                if stamp!=last_issued_stamp and stop_s is not None and not stop_done:
                    pose=make_pose(a,x,y,heading.yaw,wall,stamp);s_now=along_track(points,state.progress,pose)
                    if s_now>=stop_s-a.stop_lead_m:
                        if gate:gate.close()
                        phase='hold';hold_t0=now;hold_pos=(pose.x,pose.y);hold_enc=trial.adapter.status.encoder_count if trial and trial.adapter.status else None
                        hold_wheel=0;hold_last_cmd=now;stop_cut_pos=(pose.x,pose.y,s_now)
                        if trial:trial.command_target(0,time.monotonic(),trial.max_hold_s)
                        stats['planned_stop']={'label':a.stop_at,'target_s':round(stop_s,2),'cut_s':round(s_now,2),'cut_utc':stamp}
                        journal.event('planned_stop_cut',**stats['planned_stop'])
                        print(f'PLANNED STOP: drive cut at s={s_now:.2f}m (target {stop_s:.2f}m). Holding; watch for roll-back.',flush=True);continue
                    if now>=next_notice:print(f'to planned stop: {stop_s-s_now:.1f}m',flush=True);next_notice=now+1.
                if stamp!=last_issued_stamp:
                    pose=make_pose(a,x,y,heading.yaw,wall,stamp)
                    pi_=state.progress;ph_=qr.path_heading_at(points,pi_)
                    lat_=-math.sin(ph_)*(pose.x-points[pi_].x)+math.cos(ph_)*(pose.y-points[pi_].y)
                    ld_=adaptive_lookahead(a.lookahead_m,a.lookahead_gain,a.lookahead_max_m,lat_)
                    follower.cfg=dataclasses.replace(cfg,lookahead_m=ld_);stats['max_lookahead_m']=max(stats['max_lookahead_m'] or 0.,ld_)
                    msg=follower.step(pose,time.monotonic());dec=msg.decision or {};laterr=dec.get('lateral_error_m')
                    if isinstance(laterr,(int,float)):stats['max_lateral_error_m']=max(stats['max_lateral_error_m'] or 0.,abs(laterr))
                    if msg.kind!='steer':
                        outcome='arrived' if getattr(msg,'reached',False) else 'failed';reason=msg.reason;code=0 if outcome=='arrived' else 1;break
                    requested=lpf.left_wheel_cdeg(msg.angle_cdeg,a.wheelbase_m,a.track_m)
                    wheel=lpf.cap_left_wheel(requested,a.max_steer_deg,a.max_steer_right_deg);saturated=wheel!=requested
                    stats['calculated_targets']+=1;stats['saturated_targets']+=int(saturated)
                    ok=True if trial is None else trial.command_target(wheel,time.monotonic(),trial.max_hold_s)
                    if not ok:raise ValueError('target rejected: '+(trial.adapter.first_fault or trial.adapter.message))
                    stats['accepted_targets']+=int(trial is not None)
                    if gate:gate.open(time.monotonic())
                    last_issued_stamp=stamp
                    if now>=next_telemetry:
                        print(f'FOLLOW label={labels[state.progress]} error={laterr}m heading={math.degrees(pose.yaw):.1f}deg left={wheel/100:.2f}deg ld={ld_:.1f}m saturated={saturated}',flush=True)
                        next_telemetry=now+1.
                    journal.event('follow',kind=msg.kind,source_utc=stamp,pose={'x':pose.x,'y':pose.y,'yaw':pose.yaw},lookahead_m=ld_,
                                  heading_anchor_age_s=wall-heading.anchor_at,decision=dec,requested_left_cdeg=requested,
                                  target_left_cdeg=wheel,saturated=saturated,accepted=ok,progress=state.progress,label=labels[state.progress])
            time.sleep(.01)
    except KeyboardInterrupt:outcome='operator_stop';reason='Ctrl+C';code=130
    except Exception as exc:outcome='failed';reason=f'{type(exc).__name__}: {exc}';code=1
    finally:
        if gate:gate.close()
        if trial:
            try:
                trial.quit(time.monotonic());deadline=time.monotonic()+trial.ack_wait_s+.3
                while time.monotonic()<deadline and trial.tick(time.monotonic()):time.sleep(.01)
                if not trial.stop_ack_observed:outcome='stop_unconfirmed';reason+='; no stop ACK';code=lst.EXIT_NO_ACK
            except Exception as exc:outcome='stop_unconfirmed';reason+=f'; stop error {exc}';code=lst.EXIT_NO_ACK
        if transport:transport.close()
        if keys:keys.restore()
        stats['drive_frames_sent']=gate.sent_drive_frames if gate else 0;stats['phase_at_end']=phase;stats['route']=chosen
        journal.close(status=outcome,exit_code=code,reason=reason,stats=stats,final_stop_attempt=trial.current_stop if trial else None)
    print(json.dumps({'status':outcome,'reason':reason,'exit_code':code,'stats':stats},ensure_ascii=False));return code


if __name__=='__main__':raise SystemExit(main())
