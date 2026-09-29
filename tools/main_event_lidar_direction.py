#!/usr/bin/env python3
"""Offline stationary direction observation from four NEW-mount raw captures.
No device is opened; no profile is changed. A proposed angular mapping is not a
measured translation/body envelope/braking clearance or a live release approval.
"""
import argparse,hashlib,json,math,statistics
from pathlib import Path
from lidar_record import NODE


def wrap(deg):return (deg+180)%360-180


def raw_bins(session,mount_id):
    session=Path(session);meta=json.loads((session/'metadata.json').read_text())
    if meta.get('mount_id')!=mount_id or mount_id in ('unknown-legacy',''):
        raise ValueError('new mount identity required; never apply it to old recordings')
    if meta.get('raw_frame')!='sdk_raw_polar' or meta.get('transform_count')!=0:
        raise ValueError('raw SDK text with zero transforms required')
    bins=[[] for _ in range(360)]
    with (session/'stdout.bin').open('rb') as raw:
        for line in raw:
            match=NODE.fullmatch(line.rstrip(b'\r\n'))
            if not match:continue
            _,angle,dist,q=match.groups();a,r,quality=float(angle),float(dist)*.001,int(q)
            if quality>0 and 0<a+1<=361 and r>0:bins[round(a)%360].append(r)
    result=[statistics.median(v) if len(v)>=5 else None for v in bins]
    return result,{'session':str(session),'metadata':meta,
                   'raw_sha256':hashlib.sha256((session/'stdout.bin').read_bytes()).hexdigest()}


def target_cluster(baseline,target):
    changed={i for i,r in enumerate(target) if r is not None and .4<=r<=3. and
             (baseline[i] is None or baseline[i]-r>=.25)}
    groups=[]
    while changed:
        seed=changed.pop();group={seed};todo=[seed]
        while todo:
            i=todo.pop()
            for j in ((i-1)%360,(i+1)%360):
                if j in changed:changed.remove(j);group.add(j);todo.append(j)
        if 3<=len(group)<=60:groups.append(group)
    groups.sort(key=len,reverse=True)
    if not groups or (len(groups)>1 and len(groups[1])>=len(groups[0])*.5):
        raise ValueError('target absent/ambiguous; keep background stationary and repeat this pose only')
    group=groups[0]
    angle=math.degrees(math.atan2(sum(math.sin(math.radians(i)) for i in group),sum(math.cos(math.radians(i)) for i in group)))%360
    return {'raw_angle_deg':angle,'range_m':statistics.median(target[i] for i in group),'bins':sorted(group),
            'placement_assumption':'known object lies on requested body direction; not surveyed translation'}


def angular_mapping(observations,roll_deg=180):
    if roll_deg not in (0,180):raise ValueError('roll must be physically reported 0 or180')
    expected={'front':0.,'left':90.,'right':-90.};candidates=[]
    for gain in (-1,1):
        yaw=wrap(-gain*observations['front']['raw_angle_deg'])
        residual={k:wrap(gain*observations[k]['raw_angle_deg']+yaw-v) for k,v in expected.items()}
        candidates.append((max(abs(v) for v in residual.values()),gain,yaw,residual))
    candidates.sort();error,gain,yaw,residual=candidates[0]
    if error>15. or candidates[1][0]-error<30.:raise ValueError('front/left/right do not establish an unambiguous angular mapping')
    return {'theta_sign':gain*(-1 if roll_deg==180 else 1),'roll_deg':roll_deg,'yaw_deg':yaw,
            'net_angle_gain_measured':gain,'residual_deg':residual,
            'basis':'raw angular differences measured against user-positioned front/left/right; roll from physical user report',
            'yaw_precision':'limited by object placement; residual is not an absolute calibration error bound',
            'profile_updated':False,'live_direction_approved':False}


def oblique_ordering(observations,roll_deg=180):
    """Confirm local left/right ordering without inventing oblique target bearings."""
    if roll_deg not in (0,180):raise ValueError('roll must be physically reported 0 or180')
    front=observations['front']['raw_angle_deg']
    left=wrap(observations['left']['raw_angle_deg']-front)
    right=wrap(observations['right']['raw_angle_deg']-front)
    if not (10<=abs(left)<=90 and 10<=abs(right)<=90 and left*right<0):
        raise ValueError('forward-oblique targets do not establish opposite local angular ordering')
    gain=1 if left>0 else -1
    return {'theta_sign_from_local_order':gain*(-1 if roll_deg==180 else 1),
            'roll_deg':roll_deg,'net_angle_gain_measured':gain,
            'raw_offsets_from_front_deg':{'front_left':left,'front_right':right},
            'approximate_front_yaw_deg':wrap(-gain*front),
            'basis':'user placed front/forward-left/forward-right; oblique bearings NOT measured or assumed90deg',
            'scope':'local angular ordering only; yaw accuracy and translation remain unmeasured',
            'profile_updated':False,'live_direction_approved':False}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for role in ('baseline','front','left','right'):ap.add_argument('--'+role,type=Path,required=True)
    ap.add_argument('--mount-id',required=True);ap.add_argument('--roll-deg',type=int,choices=(0,180),required=True)
    ap.add_argument('--layout',choices=('cardinal','forward_oblique'),default='cardinal')
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    try:
        data={};sources={}
        for role in ('baseline','front','left','right'):data[role],sources[role]=raw_bins(getattr(args,role),args.mount_id)
        observed={role:target_cluster(data['baseline'],data[role]) for role in ('front','left','right')}
        mapping=oblique_ordering(observed,args.roll_deg) if args.layout=='forward_oblique' else angular_mapping(observed,args.roll_deg)
        result={'input_kind':'stationary_recorded_raw','mount_id':args.mount_id,'layout':args.layout,'observations':observed,
                'proposed_mapping':mapping,'sources':sources}
        with args.output.open('x') as out:json.dump(result,out,ensure_ascii=False,indent=2)
        print(json.dumps({'output':str(args.output),'observations':observed,'proposed_mapping':result['proposed_mapping']},ensure_ascii=False,indent=2))
        return 0
    except (OSError,ValueError) as exc:print('UNCONFIRMED: '+str(exc));return 2

if __name__=='__main__':raise SystemExit(main())
