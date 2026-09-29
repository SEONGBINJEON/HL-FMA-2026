#!/usr/bin/env python3
"""Local GNSS route following with explicit operator start, bounded input and fault latching.

Route middle re-entry uses nearby same-direction segments. This is a provisional
field controller, not a guarantee of convergence with an inaccurate steering map.
IMU relative yaw is optional and requires an explicitly checked mounting sign.
No hardware is opened by imports or --check-only. Drive defaults to zero.
"""
import argparse
import dataclasses
import json
import math
import sys
import time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import joystick_protocol as jp
import steer_target_host as h
import live_steering_trial as lst
import path_steering as ps
import pose_reference as pr
import qualifier_route as qr
import step_manual_protocol as smp
import steering_calibration as sc
from path_follow_inputs import LatestRows, Heading, utc_seconds, rear_axle_points, select_rejoin, left_wheel_cdeg


class DriveGate:
    def __init__(self,pwm,adapter,max_gap_s=.4,clock=time.monotonic):
        if type(pwm) is not int or not 0<=pwm<=255: raise ValueError('invalid PWM')
        self.pwm=pwm;self.adapter=adapter;self.max_gap_s=max_gap_s;self.clock=clock
        self.allowed_until=0.;self.sent_drive_frames=0
    def open(self,now): self.allowed_until=now+self.max_gap_s
    def close(self): self.allowed_until=0.
    def __call__(self,command):
        active=(self.clock()<self.allowed_until and not self.adapter.locked_by_fault
                and self.adapter.phase in ('ready','active')
                and self.adapter.have_target_since_prepare and command.action==smp.CONTROL)
        drive=1 if active and self.pwm else 0
        if drive:self.sent_drive_frames+=1
        cmd=smp.Command(command.boot_id,command.request_id,command.action,drive,command.steer,
                        1 if drive or command.steer else 0)
        return jp.encode_command(cmd,self.pwm if drive else 0)


def cap_left_wheel(requested_cdeg,left_cap_deg,right_cap_deg):
    """Asymmetric actuator cap on the left-front-wheel command: +left_cap (left turn) .. -right_cap (right turn)."""
    hi=round(left_cap_deg*100);lo=-round(right_cap_deg*100)
    return max(lo,min(hi,requested_cdeg))


class RejoinState(qr.RunState):
    def __init__(self,params,start_index=None):
        super().__init__(params);self.start_index=start_index
    def arm(self,pose):
        if self.state!=self.MODE_SELECTED:return False
        bad=qr.pose_problem(pose)
        if bad or pose.age_s>self.params.pose_max_age_s:
            self.last_reason=bad or 'pose stale';return False
        try:
            i,d=select_rejoin(self.route.points,pose,self.params.max_distance_m,
                              self.params.max_heading_deg,self.start_index)
        except ValueError as exc:self.last_reason=str(exc);return False
        self.progress=i;self.state=self.READY;self.last_reason=f'rejoin segment {i}, distance {d:.2f}m';return True


def get_parser():
    p=argparse.ArgumentParser(description=__doc__)
    for flag in ('route','rtk-session','calibration','output-dir'):p.add_argument('--'+flag,required=True)
    p.add_argument('--port',default=str(lst.smc.MEGA_PATH))
    p.add_argument('--lookahead-m',type=float,default=2.)
    p.add_argument('--max-lateral-error-m',type=float,default=2.)
    p.add_argument('--max-steer-deg',type=float,default=10.,help='left-front-wheel LEFT-turn cap (positive); also the right cap unless --max-steer-right-deg is given')
    p.add_argument('--max-steer-right-deg',type=float,help='separate right-turn cap magnitude (table right range may differ from left)')
    p.add_argument('--pose-max-age-s',type=float,default=.5)
    p.add_argument('--heading-max-age-s',type=float,default=3.)
    p.add_argument('--heading-window-m',type=float,default=.5)
    p.add_argument('--max-correction-age-s',type=float,default=5.)
    p.add_argument('--arm-heading-deg',type=float,default=90.)
    p.add_argument('--run-heading-deg',type=float,default=90.)
    p.add_argument('--start-distance-m',type=float,default=3.)
    p.add_argument('--start-index',type=int,help='optional explicit route segment if branches overlap')
    p.add_argument('--initial-heading-deg',type=float,help='operator supplied actual ENU body direction; east0 north90')
    p.add_argument('--imu-session',help='existing IMU recorder folder containing samples.jsonl; no device opened here')
    p.add_argument('--imu-yaw-sign',type=int,choices=(-1,1),help='confirmed sign to make left rotation positive')
    p.add_argument('--antenna-offset-m',type=float,default=.315)
    p.add_argument('--wheelbase-m',type=float,default=.75)
    p.add_argument('--track-m',type=float,default=.67)
    p.add_argument('--angle-model',choices=('ideal-ackermann',),help='explicit provisional model; not a measured linkage')
    p.add_argument('--max-seconds',type=float,default=180.)
    p.add_argument('--drive-pwm',type=int,default=0)
    p.add_argument('--dry-run',action='store_true',help='live file calculation only, no board; no physical validation')
    p.add_argument('--check-only',action='store_true',help='validate saved inputs only, no live files or board')
    return p


def main(argv=None):
    a=get_parser().parse_args(argv)
    for field in ('lookahead_m','max_lateral_error_m','max_steer_deg','pose_max_age_s','heading_max_age_s',
                  'heading_window_m','max_correction_age_s','start_distance_m','wheelbase_m','track_m','max_seconds'):
        if not math.isfinite(getattr(a,field)) or getattr(a,field)<=0:raise ValueError('invalid '+field)
    if (not 0<a.max_steer_deg<60 or not 0<a.arm_heading_deg<=90 or not 0<a.run_heading_deg<=90
            or not math.isfinite(a.antenna_offset_m) or not 0<=a.antenna_offset_m<2
            or not 0<=a.drive_pwm<=255):raise ValueError('invalid physical/heading limits')
    if a.initial_heading_deg is not None and not math.isfinite(a.initial_heading_deg):raise ValueError('invalid initial heading')
    if a.max_steer_right_deg is None:a.max_steer_right_deg=a.max_steer_deg
    if not math.isfinite(a.max_steer_right_deg) or not 0<a.max_steer_right_deg<60:raise ValueError('invalid max_steer_right_deg')
    if bool(a.imu_session)!=(a.imu_yaw_sign is not None):raise ValueError('IMU session and verified sign must be supplied together')
    if not a.angle_model:raise ValueError('specify --angle-model ideal-ackermann for a provisional field model')
    pre=lst.preflight(a.calibration,a.max_steer_deg,port=None if a.dry_run or a.check_only else a.port)
    negative=lst.preflight(a.calibration,-a.max_steer_right_deg,port=None)
    if not pre['live_capable'] or not negative['live_capable']:
        print(json.dumps({'positive':pre['missing'],'negative':negative['missing']},ensure_ascii=False));return lst.EXIT_BLOCKED
    track=qr.load_track_csv('manual',Path(a.route));meta=json.loads(Path(a.route).with_suffix('.meta.json').read_text())
    if meta.get('frame')!='map':raise ValueError('route frame mismatch')
    lat0,lon0=meta['origin_lat'],meta['origin_lon']
    if not all(math.isfinite(x) for x in (lat0,lon0)) or not (-90<lat0<90 and -180<=lon0<=180):raise ValueError('invalid local origin')
    points=rear_axle_points(track.points,meta.get('reference'),a.antenna_offset_m)
    route=qr.ModeRoute(mode='manual',segments=[qr.Segment('manual',0,len(points)-1,'local recorded route')],points=points)
    selection=qr.SelectionParams(a.start_distance_m,a.arm_heading_deg,qr.cumulative_s(points)[-1],1.5,a.pose_max_age_s,'explicit local rejoin')
    state=RejoinState(selection,a.start_index);state.select_mode(route,True)
    # Geometric controller calculates a requested angle; actuator cap is applied below with telemetry.
    cfg=ps.PurePursuitConfig(a.wheelbase_m,'user field dimensions; provisional',a.lookahead_m,
                           a.max_lateral_error_m,math.radians(a.run_heading_deg),math.radians(85),a.pose_max_age_s,
                           .3,.5,'rear_axle_center','provisional field controller')
    follower=ps.PathFollower(state,cfg,lambda s:int(s*1000)&0xffffffff)
    if a.check_only:
        print(json.dumps({'status':'INPUTS_VALID_ONLY','points':len(points),'route_reference':meta['reference'],
                          'tracking_reference':'rear_axle_center','model':a.angle_model,'hardware_access':False},ensure_ascii=False));return 0
    inputs=LatestRows(Path(a.rtk_session)/'gga.jsonl')
    imu=LatestRows(Path(a.imu_session)/'samples.jsonl') if a.imu_session else None
    heading=Heading(a.heading_window_m,math.radians(a.initial_heading_deg) if a.initial_heading_deg is not None else None,
                    time.time() if a.initial_heading_deg is not None else None,a.imu_yaw_sign)
    details={'route':a.route,'route_sha256':lst.sha256_file(a.route),'route_meta_sha256':lst.sha256_file(Path(a.route).with_suffix('.meta.json')),
             'args':vars(a),'drive_pwm':a.drive_pwm,'steering_only':not bool(a.drive_pwm),'provisional_angle_model':a.angle_model,
             'route_transform':'antenna route shifted by stored tangent heading; approximate on curves',
             'steer_cap_left_deg':a.max_steer_deg,'steer_cap_right_deg':a.max_steer_right_deg,
             'new_code_sha256':{f:lst.sha256_file(ROOT/f) for f in ('tools/live_path_follow.py','tools/path_follow_inputs.py')}}
    journal=lst.TrialJournal(Path(a.output_dir),'model_calculation' if a.dry_run else 'live_vehicle',
                            lst.build_metadata('path_follow',a.calibration,pre,int(a.max_steer_deg*100),details,argv))
    transport=trial=gate=keys=None;phase='waiting';last_stamp=None;last_fix=None;last_issued_stamp=None
    stats={'fixes_seen':0,'calculated_targets':0,'accepted_targets':0,'max_lateral_error_m':None,'saturated_targets':0}
    outcome='timeout';reason='time budget expired without arrival';code=1;t0=time.monotonic();next_notice=0.;next_telemetry=0.
    try:
        keys=lst.StdinKeys()
        if not a.dry_run:
            transport=lst.PosixSerialTransport(a.port,check_identity=True)
            trial=lst.assemble(h.load_library(),a.calibration,int(min(a.max_steer_deg,a.max_steer_right_deg)*100),transport,journal,allow_synthetic=False)
            gate=DriveGate(a.drive_pwm,trial.adapter);trial.client.encoder=gate
        print('p: select nearby forward route and prepare; s/space/q: STOP. Ctrl+C: STOP. No automatic restart.',flush=True)
        while time.monotonic()-t0<a.max_seconds:
            now=time.monotonic();wall=time.time();pressed=keys.poll()
            if any(k in ('s',' ','q') for k in pressed):outcome='operator_stop';reason='operator stop';code=130;break
            # Freshness/quality processed BEFORE another control response. All reads are bounded.
            if imu:
                for row in imu.poll():heading.imu(row,wall)
            rows=inputs.poll(); stats['fixes_seen']+=len(rows)
            for row in rows:
                stamp=utc_seconds(row['host_utc'])
                if last_stamp is not None and stamp<=last_stamp:
                    if stamp<last_stamp:raise ValueError('GNSS timestamp regressed')
                    continue
                last_stamp=stamp
                if not 0<=wall-stamp<=a.pose_max_age_s:
                    last_fix=None;continue
                age=row.get('differential_age_s')
                if (row.get('fix_status')!='rtk_fixed' or row.get('checksum_valid') is not True
                        or not isinstance(age,(int,float)) or not math.isfinite(age) or not 0<=age<=a.max_correction_age_s):
                    last_fix=None
                    if phase in ('preparing','running'):raise ValueError('GNSS quality/correction age invalid')
                    continue
                lat,lon=row['latitude_deg'],row['longitude_deg']
                if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (lat,lon)):raise ValueError('invalid GNSS coordinates')
                x=(lon-lon0)*111320*math.cos(math.radians(lat0));y=(lat-lat0)*111320
                heading.gps(x,y,stamp);last_fix=(x,y,stamp)
            valid=last_fix is not None and 0<=wall-last_fix[2]<=a.pose_max_age_s and heading.valid(wall,a.heading_max_age_s)
            if not valid:
                if gate:gate.close()
                if phase in ('preparing','running'):raise ValueError('position/heading invalid or stale; restart explicitly at current position')
            if trial:
                if not trial.tick(time.monotonic()):raise ValueError(trial.exit_reason or 'board transport stopped')
                if phase in ('preparing','running') and trial.adapter.locked_by_fault:raise ValueError(trial.adapter.first_fault or 'board fault')
            if not valid:
                if now>=next_notice:print('Waiting: fresh RTK and forward heading required.',flush=True);next_notice=now+2
                time.sleep(.01);continue
            x,y,stamp=last_fix
            pose=pr.rear_axle_pose(x=x,y=y,heading=heading.yaw,heading_convention=pr.HEADING_ENU_CCW_RAD,
                 map_yaw_from_enu_rad=0.,frame_id='map',frame_alignment_verified=True,reference_point='gnss_antenna',
                 position_valid=True,heading_valid=True,position_age_s=wall-stamp,heading_age_s=0.,max_age_s=a.pose_max_age_s,
                 planar_assumption_verified=True,offset_x_m=a.antenna_offset_m,offset_y_m=0.)
            if phase=='waiting' and ('p' in pressed or a.dry_run):
                if not state.arm(pose.as_route_pose()):
                    print('Entry: '+state.last_reason,flush=True);time.sleep(.01);continue
                if trial and not trial.adapter.request_prepare(time.monotonic()):
                    state.state=state.MODE_SELECTED;print('Board is not stopped/ready for prepare.',flush=True);continue
                phase='preparing';journal.event('entry',index=state.progress,reason=state.last_reason)
                print(f'Entry segment {state.progress}; preparing.',flush=True)
            if phase=='preparing' and (trial is None or trial.prepared()):
                if not state.start():raise ValueError(state.last_reason)
                # Rejoin distance only governs entry; tracking envelope is its own parameter.
                state.params=dataclasses.replace(state.params,max_distance_m=a.max_lateral_error_m,max_heading_deg=a.run_heading_deg)
                phase='running'
            if phase=='running' and stamp!=last_issued_stamp:
                msg=follower.step(pose,time.monotonic());dec=msg.decision or {};laterr=dec.get('lateral_error_m')
                if isinstance(laterr,(int,float)):stats['max_lateral_error_m']=max(stats['max_lateral_error_m'] or 0.,abs(laterr))
                if msg.kind!='steer':
                    outcome='arrived' if getattr(msg,'reached',False) else 'failed';reason=msg.reason;code=0 if outcome=='arrived' else 1;break
                requested=left_wheel_cdeg(msg.angle_cdeg,a.wheelbase_m,a.track_m)
                wheel=cap_left_wheel(requested,a.max_steer_deg,a.max_steer_right_deg);saturated=wheel!=requested
                stats['calculated_targets']+=1;stats['saturated_targets']+=int(saturated)
                ok=True if trial is None else trial.command_target(wheel,time.monotonic(),trial.max_hold_s)
                if not ok:raise ValueError('target rejected: '+(trial.adapter.first_fault or trial.adapter.message))
                stats['accepted_targets']+=int(trial is not None)
                if gate:gate.open(time.monotonic())
                last_issued_stamp=stamp
                if now>=next_telemetry:
                    print(f'FOLLOW idx={state.progress} error={laterr}m heading={math.degrees(pose.yaw):.1f}deg left={wheel/100:.2f}deg saturated={saturated}',flush=True)
                    next_telemetry=now+1.
                journal.event('follow',kind=msg.kind,source_utc=stamp,pose={'x':pose.x,'y':pose.y,'yaw':pose.yaw},
                              heading_anchor_age_s=wall-heading.anchor_at,decision=dec,requested_left_cdeg=requested,
                              target_left_cdeg=wheel,saturated=saturated,accepted=ok,progress=state.progress)
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
        stats['drive_frames_sent']=gate.sent_drive_frames if gate else 0
        journal.close(status=outcome,exit_code=code,reason=reason,stats=stats,
                      final_stop_attempt=trial.current_stop if trial else None)
    print(json.dumps({'status':outcome,'reason':reason,'exit_code':code,'stats':stats},ensure_ascii=False));return code

if __name__=='__main__':raise SystemExit(main())
