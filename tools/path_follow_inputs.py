"""Bounded saved-stream inputs and explicit local-route geometry. No device access."""
import json
import math
from datetime import datetime
from pathlib import Path
from collections import deque
import qualifier_route as qr


def utc_seconds(value):
    dt = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if dt.tzinfo is None:
        raise ValueError('timestamp must have timezone')
    return dt.timestamp()


class LatestRows:
    """Read a bounded tail, retain incomplete lines, reject rotation during a run.

    Starting at the tail deliberately does not replay an old session into live control.
    A producer backlog larger than budget is skipped to its tail, never replayed as now.
    """
    def __init__(self, path, budget=16384):
        self.path = Path(path); self.budget = budget
        self.offset = None; self.inode = None; self.pending = b''

    def poll(self):
        if not self.path.exists():
            return []
        st = self.path.stat(); inode = (st.st_dev, st.st_ino)
        if self.inode is not None and (inode != self.inode or st.st_size < self.offset):
            raise ValueError('sensor log replaced/truncated; explicit restart required')
        self.inode = inode
        skip = self.offset is None or st.st_size - self.offset > self.budget
        if skip:
            self.offset = max(0, st.st_size - self.budget); self.pending = b''
        start = self.offset
        with self.path.open('rb') as f:
            f.seek(start); raw = f.read(self.budget)
        self.offset += len(raw)
        if skip and start:
            raw = raw.partition(b'\n')[2]
        lines = (self.pending + raw).split(b'\n'); self.pending = lines.pop()
        if len(self.pending) > self.budget:
            raise ValueError('oversized sensor row')
        rows = []
        for line in lines:
            if not line.strip(): continue
            try: rows.append(json.loads(line))
            except (ValueError, UnicodeError): raise ValueError('invalid complete sensor row')
        return rows


class Heading:
    """GPS course anchor plus optional *relative* IMU yaw (explicit sign required).

    Course assumes forward motion. IMU yaw is never taken as absolute north.
    A stationary GPS fix does not refresh the heading anchor's age.
    """
    def __init__(self, window_m=1., initial_rad=None, initial_stamp=None, imu_sign=None):
        self.window = window_m; self.trail = deque(maxlen=160)
        self.yaw = initial_rad; self.anchor_at = initial_stamp
        self.imu_sign = imu_sign; self.imu_raw = None; self.imu_at = None

    def imu(self, row, wall):
        stamp = utc_seconds(row.get('completed_host_utc', row.get('host_utc', '')))
        if not row.get('valid') or not 0 <= wall-stamp <= .5:
            return
        raw = math.radians(float(row['euler_deg'][2]))
        if not math.isfinite(raw): raise ValueError('invalid IMU yaw')
        if self.imu_at is not None and stamp <= self.imu_at: return
        if self.imu_sign not in (-1, 1): raise ValueError('IMU axis sign not specified')
        if self.yaw is not None and self.imu_raw is not None and stamp-self.imu_at <= .5:
            self.yaw = qr.wrap(self.yaw + self.imu_sign*qr.wrap(raw-self.imu_raw))
        elif self.imu_at is not None and stamp-self.imu_at > .5:
            # Missed motion cannot be recovered by applying a stale relative sample.
            self.yaw = None; self.anchor_at = None
        self.imu_raw, self.imu_at = raw, stamp

    def gps(self, x, y, stamp):
        if self.trail and stamp <= self.trail[-1][2]: return
        self.trail.append((x,y,stamp))
        anchor = next(((px,py) for px,py,t in reversed(list(self.trail)[:-1])
                       if 0 < stamp-t <= 3. and math.hypot(x-px,y-py) >= self.window), None)
        if anchor is not None:
            course = math.atan2(y-anchor[1], x-anchor[0])
            if self.imu_sign is not None and self.yaw is not None:
                self.yaw = qr.wrap(self.yaw + .2*qr.wrap(course-self.yaw))
            else: self.yaw = course
            self.anchor_at = stamp

    def valid(self, wall, max_age):
        if self.yaw is None or self.anchor_at is None: return False
        # Relative yaw bridges brief GPS course gaps, not indefinite unaided drift.
        if self.imu_sign is not None:
            return self.imu_at is not None and 0 <= wall-self.imu_at <= .5 and 0 <= wall-self.anchor_at <= 10.
        return 0 <= wall-self.anchor_at <= max_age


def rear_axle_points(points, reference, offset):
    if reference == 'rear_axle_center': return list(points)
    if reference != 'GNSS antenna position, NOT rear axle':
        raise ValueError('unknown route reference: declare antenna or rear axle in metadata')
    return [qr.Point(p.idx,p.x-offset*math.cos(p.yaw),p.y-offset*math.sin(p.yaw),p.yaw,p.source,p.frame_id)
            for p in points]


def select_rejoin(points, pose, max_distance, max_heading_deg, explicit_index=None):
    """Project on same-direction segments; adjacent samples are one continuous branch."""
    candidates=[]
    indices=range(len(points)-1) if explicit_index is None else [explicit_index]
    for i in indices:
        if not 0 <= i < len(points)-1: raise ValueError('start index outside route segments')
        a,b=points[i:i+2]; dx,dy=b.x-a.x,b.y-a.y; ll=dx*dx+dy*dy
        if ll < 1e-10: continue
        err=abs(qr.wrap(pose.yaw-math.atan2(dy,dx)))
        if err > math.radians(max_heading_deg): continue
        t=max(0.,min(1.,((pose.x-a.x)*dx+(pose.y-a.y)*dy)/ll))
        distance=math.hypot(pose.x-a.x-t*dx,pose.y-a.y-t*dy)
        if distance <= max_distance: candidates.append((distance+err*.25,distance,i,err))
    if not candidates: raise ValueError('no nearby same-direction segment; align vehicle or choose another entry')
    candidates.sort(); best=candidates[0]; s=qr.cumulative_s(points)
    for other in candidates[1:]:
        if (abs(s[other[2]]-s[best[2]]) > max(2.,2*max_distance)
                and abs(other[0]-best[0]) < .25):
            raise ValueError('two distinct route branches match; select --start-index explicitly')
    return best[2], best[1]


def left_wheel_cdeg(equiv_cdeg, wheelbase, track):
    d=math.radians(equiv_cdeg/100.)
    # Signed Ackermann: left wheel is inner on left turns, outer on right turns.
    return round(math.degrees(math.atan2(wheelbase*math.tan(d),wheelbase-track*.5*math.tan(d)))*100)
