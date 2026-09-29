"""Bounded local S candidates and directional parking contracts.

Uses HYPER's split forward/reverse strategy and candidate clearance selection,
not ROS/Nav2 controller parameters as a vehicle calibration. All geometry is
rear-axle FLU / ENU metres. Reference road polygons never authorize live use.
"""
from dataclasses import dataclass
import math
import qualifier_route as qr
from main_event_lidar import clip_polygon,polygon_area,free_triangles,outside_current_body
from path_follow_inputs import left_wheel_cdeg

S_OFFSETS=(0.,.3,-.3,.6,-.6,.8,-.8,1.,-1.,1.2,-1.2,1.4,-1.4)


@dataclass
class Envelope:
    front: float
    rear: float
    width: float
    margin: float
    wheelbase: float=.75
    track: float=.67
    left_deg: float=25.
    right_deg: float=19.

    def __post_init__(self):
        if any(not math.isfinite(v) or v<=0 for v in vars(self).values()):
            raise ValueError('positive finite envelope and steering measurements required')


def inside(point,polygon):
    x,y=point;odd=False
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:odd=not odd
    return odd


def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def contained(poly,boundary):
    if not all(inside(p,boundary) for p in poly):return False
    # Concave boundary may cut through a footprint despite all four corners inside.
    for a,b in zip(poly,poly[1:]+poly[:1]):
        for c,d in zip(boundary,boundary[1:]+boundary[:1]):
            if cross(a,b,c)*cross(a,b,d)<=0 and cross(c,d,a)*cross(c,d,b)<=0:return False
    return True


def footprint(x,y,yaw,env,pad=0.):
    c,s=math.cos(yaw),math.sin(yaw);m=env.margin+pad
    return [(x+c*u-s*v,y+s*u+c*v) for u,v in
            [(-env.rear-m,-env.width/2-m),(env.front+m,-env.width/2-m),
             (env.front+m,env.width/2+m),(-env.rear-m,env.width/2+m)]]


def body_points(points,pose):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    return [(c*(p[0]-x)+s*(p[1]-y),-s*(p[0]-x)+c*(p[1]-y)) for p in points]


def polygon_observed(poly,triangles):
    area=polygon_area(poly)
    if area<1e-10:return False
    covered=0.
    for tri in triangles:
        part=poly
        for a,b in zip(tri,tri[1:]+tri[:1]):
            part=clip_polygon(part,a,b)
            if not part:break
        covered+=polygon_area(part)
    return covered>=area-1e-7


def resample(points,step=.25):
    # This helper stores TRAVEL tangents at every sample. Original parking CSV
    # yaw is BODY yaw, which differs by pi in reverse; mixing the two makes a
    # straight reverse path appear to contain a spurious 180-degree corner.
    first=points[0]
    tangent=math.atan2(points[1].y-first.y,points[1].x-first.x)
    out=[qr.Point(first.idx,first.x,first.y,tangent,first.source,first.frame_id)]
    for a,b in zip(points,points[1:]):
        n=max(1,math.ceil(math.hypot(b.x-a.x,b.y-a.y)/step))
        yaw=math.atan2(b.y-a.y,b.x-a.x)
        for j in range(1,n+1):
            t=j/n;out.append(qr.Point(a.idx,a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),yaw,a.source,a.frame_id))
    return out


def assess_path(points,env,boundary,obstacles,observed,*,direction=1,reference_candidate=False):
    """Checks local curvature and continuous conservative footprint samples.

`observed` evaluates the full padded polygon, not only a path centre point.
The half-step plus rotational pad encloses linear interpolated body poses.
"""
    if direction not in (-1,1) or len(points)<3:return 'invalid_path'
    if (not boundary or len(boundary)<3) and not reference_candidate:return 'boundary_unknown'
    ss=qr.cumulative_s(points)
    headings=[math.atan2(b.y-a.y,b.x-a.x) for a,b in zip(points,points[1:])]
    for i in range(1,len(headings)):
        ds=(ss[i+1]-ss[i-1])*.5
        if ds<1e-6:return 'duplicate_path_point'
        k=qr.wrap(headings[i]-headings[i-1])/ds
        equiv=round(math.degrees(math.atan(direction*env.wheelbase*k))*100)
        left=left_wheel_cdeg(equiv,env.wheelbase,env.track)/100
        if not -env.right_deg<=left<=env.left_deg:return 'steering_limit'
    sampled=resample(points);footprints=[]
    for i,p in enumerate(sampled):
        travel=math.atan2(sampled[min(i+1,len(sampled)-1)].y-p.y,sampled[min(i+1,len(sampled)-1)].x-p.x) if i<len(sampled)-1 else sampled[-1].yaw
        yaw=qr.wrap(travel+(math.pi if direction<0 else 0.))
        # Constant .125 translational pad (half the <=.25 m sample interval).
        turn=0. if i==0 else abs(qr.wrap(travel-sampled[i-1].yaw))
        pad=.125+math.hypot(env.front,env.width/2)*min(1.,turn*.5)
        poly=footprint(p.x,p.y,yaw,env,pad)
        if boundary and not contained(poly,boundary):return 'outside_boundary'
        if any(inside(o[:2],poly) for o in obstacles):return 'occupied'
        footprints.append(poly)
    # Reject collision/boundary first: no expensive free-space work for a path
    # that is already geometrically impossible.
    if reference_candidate:return 'reference_candidate'  # motion still needs fresh local corridor observations
    for poly in footprints:
        if not observed(poly):return 'unobserved'
    return 'clear'


def _offset_candidates(reference,env,boundary,obstacles,observed,offsets=S_OFFSETS,reference_candidate=False):
    """Smooth local offsets, joining the same reference at both ends.

Unlike HYPER MPPI this bounded candidate lattice has no stochastic optimiser.
It reuses the candidate-clearance strategy; the proven follower tracks output.
"""
    ss=qr.cumulative_s(reference)
    if len(reference)<3 or ss[-1]<1.:return None,{'reason':'reference_too_short'}
    rejected={};valid=[]
    for offset in sorted(offsets,key=abs):
        candidate=[]
        for i,p in enumerate(reference):
            h=qr.path_heading_at(reference,min(i,len(reference)-2))
            u=ss[i]/ss[-1];d=offset*math.sin(math.pi*u)**4
            candidate.append(qr.Point(p.idx,p.x-math.sin(h)*d,p.y+math.cos(h)*d,h,p.source,p.frame_id))
        verdict=assess_path(candidate,env,boundary,obstacles,observed,reference_candidate=reference_candidate)
        if verdict in ('clear','reference_candidate'):
            valid.append((abs(offset),offset,candidate));break  # minimum-cost feasible candidate
        else:rejected[str(offset)]=verdict
    if not valid:return None,{'reason':'no_observed_road_contained_candidate','rejected':rejected}
    _,offset,path=min(valid,key=lambda v:v[0])
    return path,{'reason':'candidate_selected','offset_m':offset,'rejected':rejected}


def _s_symmetric_candidates(reference,env,boundary,obstacles,observed,offsets=S_OFFSETS,reference_candidate=False):
    if not boundary:return None,{'reason':'road_reference_missing'}
    path,why=_offset_candidates(reference,env,boundary,obstacles,observed,offsets,reference_candidate)
    why['global_visibility_required']=not reference_candidate
    why['motion_observation']='fresh_local_runtime_corridor_always_required'
    why['reference_geometry']='original_or_previous_accepted_local_path'
    if path is not None or 'steering_limit' not in why.get('rejected',{}).values():return path,why
    attempts=0
    for smooth,details in bezier_candidates(reference,1,max_deviation=.35):
        attempts+=1
        candidate,report=_offset_candidates(smooth,env,boundary,obstacles,observed,offsets,reference_candidate)
        if candidate is not None:
            report.update(reference_geometry=details,raw_rejections=why['rejected'])
            report.update(global_visibility_required=not reference_candidate,
                          motion_observation='fresh_local_runtime_corridor_always_required')
            return candidate,report
    why['generated_geometry_attempts']=attempts
    return None,why


def s_candidates(reference,env,boundary,obstacles,observed,offsets=S_OFFSETS,reference_candidate=False):
    path,why=_s_symmetric_candidates(reference,env,boundary,obstacles,observed,offsets,reference_candidate)
    if path is not None or not boundary or len(reference)<3 or not obstacles:return path,why
    ss=qr.cumulative_s(reference)
    if ss[-1]<1.:return path,why
    # Positive returns near this reference determine the longitudinal peak.
    stations=[]
    for x,y,*_ in obstacles:
        i=min(range(len(reference)),key=lambda k:math.hypot(reference[k].x-x,reference[k].y-y))
        if math.hypot(reference[i].x-x,reference[i].y-y)<=env.width/2+env.margin+1.4:
            if .1<ss[i]<ss[-1]-.1:stations.append(ss[i])
    if not stations:return path,why
    centre=sorted(stations)[len(stations)//2]/ss[-1]
    peak=max(.25,min(.75,round(centre/.05)*.05))
    peaks=list(dict.fromkeys(max(.25,min(.75,v)) for v in (peak,peak-.1,peak+.1)))
    rejected={};headings=[qr.path_heading_at(reference,min(i,len(reference)-2)) for i in range(len(reference))]
    for peak in peaks:
        for offset in sorted((o for o in offsets if o),key=abs):
            candidate=[]
            for i,p in enumerate(reference):
                u=ss[i]/ss[-1];v=u/peak if u<=peak else (1-u)/(1-peak)
                # Quintic has zero first/second derivatives at entry, peak, return.
                d=offset*v**3*(10+v*(-15+6*v));h=headings[i]
                candidate.append(qr.Point(p.idx,p.x-math.sin(h)*d,p.y+math.cos(h)*d,h,p.source,p.frame_id))
            verdict=assess_path(candidate,env,boundary,obstacles,observed,reference_candidate=reference_candidate)
            if verdict in ('clear','reference_candidate'):
                return candidate,{**why,'reason':'obstacle_anchored_candidate_selected','offset_m':offset,
                    'peak_fraction':peak,'shape':'quintic_entry_peak_return','anchored_rejected':rejected}
            rejected[str((peak,offset))]=verdict
    why['anchored_rejected']=rejected
    return None,why


class FreeSpace:
    """Angular index avoids scanning all 720+ triangles for every footprint."""
    def __init__(self,triangles,origin):
        self.origin=origin;self.triangles=triangles
        indexed=[]
        for i,t in enumerate(triangles):
            a=math.atan2(t[1][1]-origin[1],t[1][0]-origin[0])
            b=math.atan2(t[2][1]-origin[1],t[2][0]-origin[0])
            centre=(a+qr.wrap(b-a)*.5)%(2*math.pi)
            indexed.extend((centre+shift,i) for shift in (-2*math.pi,0.,2*math.pi))
        indexed.sort();self.angles=[a for a,i in indexed];self.indices=[i for a,i in indexed]

    def __call__(self,poly):
        if inside(self.origin,poly):return polygon_observed(poly,self.triangles)
        angles=sorted(math.atan2(y-self.origin[1],x-self.origin[0])%(2*math.pi) for x,y in poly)
        gaps=[(angles[(i+1)%len(angles)]-a)%(2*math.pi) for i,a in enumerate(angles)]
        k=max(range(len(gaps)),key=gaps.__getitem__)
        start=angles[(k+1)%len(angles)];end=start+2*math.pi-gaps[k]
        import bisect
        lo=bisect.bisect_left(self.angles,start-math.radians(1.))
        hi=bisect.bisect_right(self.angles,end+math.radians(1.))
        triangles=[self.triangles[i] for i in set(self.indices[lo:hi])]
        return polygon_observed(poly,triangles)


def scan_observer(scan,profile,pose,now):
    """A stale scan can never be reused as an observed free polygon."""
    if scan is None or not 0<=now-scan['received_at']<=.3 or not scan.get('complete_candidate'):
        return lambda poly:False
    if '_free_space' not in scan:
        scan['_free_space']=FreeSpace(free_triangles(scan,profile),(profile['x_m'],profile['y_m']))
    space=scan['_free_space']
    return lambda poly:all(space(part) for part in outside_current_body(body_points(poly,pose),profile))


PARKING_SEGMENTS={'t_left':((0,23,-1),(23,52,1)),
                  't_right':((0,24,-1),(24,53,1)),
                  'p_up':((0,8,1),(8,23,-1),(23,41,1)),
                  'p_down':((0,9,-1),(9,30,1))}


def parking_paths(name,track_dir):
    if name not in PARKING_SEGMENTS:raise ValueError('unknown HYPER main parking branch')
    points=qr.load_track_csv(name,track_dir/(name+'.csv')).points
    pieces=[]
    for begin,end,direction in PARKING_SEGMENTS[name]:
        if end>=len(points):raise ValueError('parking labels exceed pinned main CSV')
        part=points[begin:end+1]
        # CSV yaw is BODY yaw; reverse movement tangent must be opposite it.
        agreements=[]
        for a,b in zip(part,part[1:]):
            if math.hypot(b.x-a.x,b.y-a.y)<.05:continue
            travel=math.atan2(b.y-a.y,b.x-a.x)
            agreements.append(abs(qr.wrap(travel-a.yaw-(math.pi if direction<0 else 0.)))<math.pi/2)
        if not agreements or sum(agreements)/len(agreements)<.8:
            raise ValueError('parking body yaw / direction mismatch in pinned source')
        pieces.append((part,direction))
    return pieces


def bezier_candidates(points,direction,max_deviation=.75):
    """Generated alternatives, never a rewrite of measured/reference coordinates.

Exact endpoints and endpoint BODY yaw are retained. Every candidate is limited
in distance from the raw polyline, then independently checked against real road,
clearance, visibility and steering constraints by the caller.
"""
    a,b=points[0],points[-1];length=qr.cumulative_s(points)[-1]
    ha=a.yaw+(math.pi if direction<0 else 0.)
    hb=b.yaw+(math.pi if direction<0 else 0.)
    import qualifier_follow as q
    for f in (.2,.33,.5):
        for g in (.2,.33,.5):
            p1=(a.x+length*f*math.cos(ha),a.y+length*f*math.sin(ha))
            p2=(b.x-length*g*math.cos(hb),b.y-length*g*math.sin(hb))
            candidate=[];deviation=0.
            for i,original in enumerate(points):
                t=i/(len(points)-1);u=1-t
                x=u**3*a.x+3*u*u*t*p1[0]+3*u*t*t*p2[0]+t**3*b.x
                y=u**3*a.y+3*u*u*t*p1[1]+3*u*t*t*p2[1]+t**3*b.y
                dx=3*u*u*(p1[0]-a.x)+6*u*t*(p2[0]-p1[0])+3*t*t*(b.x-p2[0])
                dy=3*u*u*(p1[1]-a.y)+6*u*t*(p2[1]-p1[1])+3*t*t*(b.y-p2[1])
                yaw=qr.wrap(math.atan2(dy,dx)+(math.pi if direction<0 else 0.))
                point=qr.Point(original.idx,x,y,yaw,'generated_bezier_from_'+a.source,a.frame_id)
                distances=[q.project(points,k,point) for k in range(len(points)-1)]
                deviation=max(deviation,min(v[0] for v in distances if v is not None))
                candidate.append(point)
            if deviation<=max_deviation:
                yield candidate,{'kind':'generated_bezier','handles_fraction':[f,g],
                                 'max_deviation_from_raw_m':deviation,'raw_points_unchanged':True}


def parking_choice(branches,track_dir,env,boundary,obstacles,observed,reference_candidate=False):
    """Only feasible candidates; HYPER default p_up wins among eligible P branches."""
    accepted=[];rejected={};choices=[];selected_points=None;selected_meta=None
    for name in branches:
        try:pieces=parking_paths(name,track_dir)
        except ValueError as exc:rejected[name]=str(exc);continue
        chosen=[];meta=[];verdict='clear'
        for part,direction in pieces:
            alternatives=[(part,{'kind':'original_main_reference'})]
            found=None;attempts=[]
            # The original route is first; only generate alternatives if needed.
            import itertools
            for candidate,details in itertools.chain(alternatives,bezier_candidates(part,direction)):
                verdict=assess_path(candidate,env,boundary,obstacles,observed,direction=direction,reference_candidate=reference_candidate)
                attempts.append(verdict)
                if verdict in ('clear','reference_candidate'):found=candidate;meta.append(details);break
            if found is None:break
            chosen+=found if not chosen else found[1:]
        if verdict in ('clear','reference_candidate'):
            accepted.append(name)
            score=(0 if name=='p_up' else 1,sum(v['kind']!='original_main_reference' for v in meta),len(pieces),qr.cumulative_s(chosen)[-1])
            choices.append((score,name,chosen,meta))
        else:rejected[name]=verdict
    selected=None
    if choices:
        _,selected,selected_points,selected_meta=min(choices,key=lambda v:v[0])
    from dataclasses import asdict
    return selected,{'accepted':accepted,'rejected':rejected,
        'road_boundary_checked':bool(boundary),'global_visibility_required':not reference_candidate,
        'motion_observation':'fresh_local_runtime_corridor_always_required',
        'selection':'feasible_then_hyper_p_up_default_then_fewer_generated_curves_and_gears','selected_geometry':selected_meta,
        'planned_points':None if selected_points is None else [asdict(p) for p in selected_points]}


def finish_crossed(pose,contract,track_width):
    """Both rear wheel centres must cross the confirmed finite finish segment."""
    if not contract or contract.get('verified') is not True or not contract.get('evidence'):
        return None
    a,b=contract['a'],contract['b'];nx,ny=contract['forward_normal']
    vals=(*a,*b,nx,ny,track_width,contract.get('margin_m',0.))
    if not all(type(v) in (int,float) and math.isfinite(v) for v in vals):raise ValueError('nonfinite finish line')
    length=math.hypot(b[0]-a[0],b[1]-a[1]);nlen=math.hypot(nx,ny)
    if length<track_width or abs(nlen-1.)>.001 or abs(nx*(b[0]-a[0])+ny*(b[1]-a[1]))>length*.001:
        raise ValueError('finish segment/normal contract invalid')
    margin=contract.get('margin_m',0.)
    if margin<0:raise ValueError('negative finish margin')
    x,y,yaw=pose
    for sign in (-1,1):
        wx=x-sign*math.sin(yaw)*track_width/2;wy=y+sign*math.cos(yaw)*track_width/2
        along=((wx-a[0])*(b[0]-a[0])+(wy-a[1])*(b[1]-a[1]))/length
        if not 0<=along<=length:return False
        if (wx-a[0])*nx+(wy-a[1])*ny<=margin:return False
    return True
