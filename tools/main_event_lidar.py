"""One transform from SDK polar data to rear-axle FLU metres.

No ROS inverted flag or display transform is applied here. Profile theta_sign
defines the raw SDK convention; roll=180 is applied exactly once, then yaw and
translation. A profile ID must match the recording/live producer's mount ID.
"""
import argparse
import json
import math
from pathlib import Path
from main_event_core import finite
from lidar_record import NODE


def body_geometry_usable(p):
    return p.get('body_outline_verified') is True or (p.get('geometry_mode')=='candidate_reported' and bool(p.get('geometry_evidence')))


def validate_profile(p, mount_id):
    if not mount_id or mount_id != p.get('mount_id'):
        raise ValueError('mount_id missing/mismatch: never apply new mount to old recordings')
    if p.get('input_frame') != 'sdk_raw_polar' or p.get('output_frame') != 'rear_axle_FLU_m':
        raise ValueError('frame contract mismatch')
    if p.get('driver_transform_count') != 0:
        raise ValueError('double correction refused')
    if type(p.get('theta_sign')) is not int or p.get('theta_sign') not in (-1, 1):
        raise ValueError('raw SDK angular-axis correspondence unconfirmed')
    if type(p.get('roll_deg')) is not int or p.get('roll_deg') not in (0, 180):
        raise ValueError('mounting roll unconfirmed')
    for key in ('yaw_deg', 'x_m', 'y_m'):
        if not finite(p.get(key)):
            raise ValueError('unmeasured mount '+key)
    if not finite(p.get('z_m')) and not (p.get('planar_only') is True and p.get('z_m') is None):
        raise ValueError('height missing for non-planar consumer')
    if p.get('direction_verified') is not True or not p.get('direction_evidence'):
        raise ValueError('stationary front/left/right direction correspondence unverified')


def transform_node(theta_deg, distance_mm, quality, p, mount_id, *, transform_count=0):
    validate_profile(p, mount_id)
    if transform_count != 0:
        raise ValueError('already transformed data rejected')
    if (not finite(theta_deg) or not 0 <= theta_deg <= 360 or
            not finite(distance_mm) or distance_mm <= 0 or
            type(quality) is not int or not 0 < quality <= 63):
        return None
    theta = math.radians(theta_deg)*p['theta_sign']
    sx, sy = distance_mm*.001*math.cos(theta), distance_mm*.001*math.sin(theta)
    sy *= -1 if p['roll_deg'] == 180 else 1
    yaw = math.radians(p['yaw_deg'])
    return (p['x_m']+math.cos(yaw)*sx-math.sin(yaw)*sy,
            p['y_m']+math.sin(yaw)*sx+math.cos(yaw)*sy, p.get('z_m'))


def obstacle_state(scan, p, now, corridor=None, direction=1):
    """Occupied / observed free / unknown over the swept BODY, not a bin vote.

Body poses follow the supplied rear-axle polyline. Footprints include front/rear
extent and width; intermediate rotations are padded to cover their angular step.
This is a planar geometric model, not a dynamics or scan-height guarantee.
"""
    validate_profile(p, scan.get('mount_id'))
    if scan.get('frame') != 'rear_axle_FLU_m' or scan.get('transform_count') != 1:
        raise ValueError('consumer needs exactly one body transform')
    for key in ('body_width_m', 'clearance_m', 'stop_distance_m', 'front_overhang_from_rear_axle_m'):
        if not finite(p.get(key)) or p[key] <= 0:return 'unknown'
    if direction not in (-1,1):raise ValueError('invalid corridor direction')
    if not finite(scan.get('received_at')) or not 0 <= now-scan['received_at'] <= .3:return 'unknown'
    if not body_geometry_usable(p):return 'unknown'
    segments=corridor if corridor is not None else [((0.,0.),(direction*p['stop_distance_m'],0.))]
    if not segments:return 'unknown'
    polygons=body_sweep(segments,p,direction)
    front=p['front_overhang_from_rear_axle_m'];rear=p.get('rear_overhang_m',.24);half=p['body_width_m']/2
    for pt in scan.get('points',[]):
        x,y=pt[:2]
        if not finite(x) or not finite(y):return 'unknown'
        # The confirmed occupied body is the same exclusion used for visibility.
        # It is not expanded into an arbitrary sensor blind-zone mask.
        if -rear<=x<=front and abs(y)<=half:continue
        for poly in polygons:
            if all(_cross(a,b,(x,y))>=-1e-9 for a,b in zip(poly,poly[1:]+poly[:1])):return 'blocked'
    if scan.get('complete_candidate') is not True:return 'unknown'
    from main_event_maneuvers import FreeSpace
    if '_free_space' not in scan:scan['_free_space']=FreeSpace(free_triangles(scan,p),(p['x_m'],p['y_m']))
    space=scan['_free_space'];needed=False
    for poly in polygons:
        for part in outside_current_body(poly,p):
            needed=True
            if not space(part):return 'unknown'
    return 'clear' if needed else 'unknown'


def body_sweep(segments,p,direction=1):
    """Conservative union of translated rectangles and padded corner rotations."""
    front=p['front_overhang_from_rear_axle_m'];rear=p.get('rear_overhang_m',.24)
    width=p['body_width_m']/2+p['clearance_m'];out=[];previous=0.
    def rectangle(x,y,yaw,lo,hi,half):
        c,s=math.cos(yaw),math.sin(yaw)
        return [(x+c*u-s*v,y+s*u+c*v) for u,v in [(-lo,-half),(hi,-half),(hi,half),(-lo,half)]]
    for a,b in segments:
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        if length<1e-8:continue
        travel=math.atan2(dy,dx);yaw=travel+(math.pi if direction<0 else 0.)
        delta=(yaw-previous+math.pi)%(2*math.pi)-math.pi
        n=max(1,math.ceil(abs(delta)/math.radians(5.)))
        pad=math.hypot(max(front,rear),width)*math.sin(abs(delta)/(2*n))
        for k in range(n+1):out.append(rectangle(*a,previous+delta*k/n,rear+pad,front+pad,width+pad))
        lead=front if direction>0 else rear;trail=rear if direction>0 else front
        out.append(rectangle(*a,travel,trail,length+lead,width))
        previous=yaw
    return out


def _cross(a,b,c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def clip_polygon(poly,a,b):
    """Clip convex polygon to the left half-plane of directed edge a->b."""
    out=[]
    if not poly:return out
    prev=poly[-1];fp=_cross(a,b,prev)
    for cur in poly:
        fc=_cross(a,b,cur)
        if (fc>=0)!=(fp>=0):
            t=fp/(fp-fc)
            out.append((prev[0]+t*(cur[0]-prev[0]),prev[1]+t*(cur[1]-prev[1])))
        if fc>=0:out.append(cur)
        prev,fp=cur,fc
    return out


def polygon_area(poly):
    return abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(poly,poly[1:]+poly[:1])))*.5


def free_triangles(scan,p):
    """No-return rays are retained so interpolation cannot bridge them."""
    if '_free_triangles' in scan:return scan['_free_triangles']
    rays=sorted(scan.get('rays',[]),key=lambda r:r[0])
    origin=(p['x_m'],p['y_m']);triangles=[]
    max_gap=math.radians(1.)  # resolution contract, not a coverage-bin vote
    for i,a in enumerate(rays):
        b=rays[(i+1)%len(rays)]
        gap=(b[0]-a[0])%(2*math.pi)
        if a[1] is None or b[1] is None or not 1e-8<gap<=max_gap+1e-8:continue
        # Endpoints are occupied; keep free polygons 2 cm short of each return.
        ra,rb=a[1]-.02,b[1]-.02
        if min(ra,rb)<=0:continue
        tri=[origin,(origin[0]+ra*math.cos(a[0]),origin[1]+ra*math.sin(a[0])),
                    (origin[0]+rb*math.cos(b[0]),origin[1]+rb*math.sin(b[0]))]
        triangles.append(tri)
    if rays:scan['_free_triangles']=triangles
    return triangles


def outside_current_body(poly,p):
    """Exclude only currently occupied body, not margins or future footprints."""
    front=p['front_overhang_from_rear_axle_m'];half=p['body_width_m']/2
    rear=-p.get('rear_overhang_m',.24)
    external=[clip_polygon(poly,(front,1),(front,0))]
    behind=clip_polygon(poly,(front,0),(front,1))
    external.append(clip_polygon(behind,(rear,0),(rear,1)))
    middle=clip_polygon(behind,(rear,1),(rear,0))
    external.append(clip_polygon(middle,(0,half),(1,half)))
    external.append(clip_polygon(middle,(1,-half),(0,-half)))
    return [part for part in external if polygon_area(part)>1e-10]



class ScanAssembler:
    def __init__(self, profile, mount_id):
        validate_profile(profile, mount_id)
        self.profile, self.mount_id = profile, mount_id
        self.pending = b''
        self.points = None
        self.rays = []
        self.coverage = set()
        self.started = None
        self.scan_id = 0

    def feed(self, data, received_at):
        self.pending += data
        if len(self.pending) > 1024*1024:
            raise ValueError('unbounded SDK input')
        out = []
        while b'\n' in self.pending:
            line, self.pending = self.pending.split(b'\n', 1)
            match = NODE.fullmatch(line)
            if not match:
                continue
            sync, angle, dist, quality = match.groups()
            if sync:
                if self.points is not None:
                    self.scan_id += 1
                    out.append({'frame': 'rear_axle_FLU_m', 'transform_count': 1,
                                'mount_id': self.mount_id, 'scan_id': self.scan_id,
                                'received_at': self.started, 'closed_at': received_at,
                                'time_basis': 'host_pipe_receipt_not_sensor_capture',
                                'complete_candidate': 0 <= received_at-self.started <= .3,
                                'forward_bins_covered': len(self.coverage), 'points': self.points, 'rays': self.rays})
                self.points, self.coverage, self.started = [], set(), received_at
                self.rays = []
            point = transform_node(float(angle), float(dist), int(quality), self.profile, self.mount_id)
            if self.points is not None:
                if len(self.rays)>=8192:raise ValueError('scan has too many rays / missing sync')
                unit=transform_node(float(angle),1000.,1,self.profile,self.mount_id)
                if unit is None:raise ValueError('invalid raw scan angle')
                bearing=math.atan2(unit[1]-self.profile['y_m'],unit[0]-self.profile['x_m'])
                self.rays.append((bearing,None if point is None else float(dist)*.001))
            if self.points is not None and point is not None:
                if len(self.points) >= 8192:
                    raise ValueError('scan has too many points / missing sync')
                self.points.append(point)
                # Front half-plane in BODY axes, with original sensor origin removed.
                heading = math.degrees(math.atan2(point[1]-self.profile['y_m'], point[0]-self.profile['x_m']))
                if -90 <= heading < 90:
                    self.coverage.add(int((heading+90)//10))
        return out


def sonar_state(board, confirmed_channels, now, stop_cm):
    if not confirmed_channels or not finite(board.get('received_at')) or not 0 <= now-board['received_at'] < .15:
        return 'unknown'
    values = [board.get(ch) for ch in confirmed_channels]
    if any(type(v) is int and 0 < v < min(stop_cm, 999) for v in values):
        return 'blocked'
    # Firmware type150 has no independent sonar acquisition timestamp or validity.
    # Far/999 can NEVER supply a clear permission. This is supplemental stop only.
    return 'unknown'


def path_corridor(points, progress, x, y, yaw, horizon):
    """Rear-axle path in body FLU, clipped at the forward stopping horizon."""
    if not all(finite(v) for v in (x,y,yaw,horizon)) or horizon<=0:
        raise ValueError('invalid corridor pose/horizon')
    c,s=math.cos(yaw),math.sin(yaw)
    path=[(0.,0.)];travel=0.
    for p in points[min(progress+1,len(points)-1):]:
        dx,dy=p.x-x,p.y-y
        v=(c*dx+s*dy,-s*dx+c*dy)
        prev=path[-1];d=math.hypot(v[0]-prev[0],v[1]-prev[1])
        if d<1e-6:continue
        if travel+d>=horizon:
            t=(horizon-travel)/d
            path.append((prev[0]+t*(v[0]-prev[0]),prev[1]+t*(v[1]-prev[1])));break
        path.append(v);travel+=d
    return list(zip(path,path[1:]))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--profile', required=True, type=Path)
    ap.add_argument('--recorded-session', required=True, type=Path)
    ap.add_argument('--output', required=True, type=Path)
    args = ap.parse_args()
    p = json.loads(args.profile.read_text())
    meta = json.loads((args.recorded_session/'metadata.json').read_text())
    assembler = ScanAssembler(p, meta.get('mount_id'))
    # Host chunks restore ORIGINAL receipt times, not the replay wall time.
    with (args.recorded_session/'stdout.bin').open('rb') as raw, args.output.open('x') as out:
        for line in (args.recorded_session/'host_chunks.jsonl').read_text().splitlines():
            c = json.loads(line)
            if c['stream'] != 'stdout': continue
            raw.seek(c['offset'])
            for scan in assembler.feed(raw.read(c['bytes']), c['host_elapsed_s']):
                scan['input_kind'] = 'recorded'
                out.write(json.dumps(scan, allow_nan=False)+'\n')
    return 0


if __name__ == '__main__': raise SystemExit(main())
