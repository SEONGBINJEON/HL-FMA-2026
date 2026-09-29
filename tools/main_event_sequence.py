"""Normal mission connections inside one runtime / board ownership session."""
import math
import qualifier_route as qr
from main_event_plan import TRACK,append_parking,append_common,replace_local_path
from main_event_maneuvers import S_OFFSETS,Envelope,s_candidates,parking_choice,scan_observer,finish_crossed
from main_event_motion import WindowFollower,travel_pose
from main_event_planner_worker import PlanWorker


class Sequence:
    def __init__(self,bundle,state,follower,args,profile,config,journal):
        self.bundle=bundle;self.state=state;self.args=args;self.profile=profile;self.config=config;self.journal=journal
        self.window=WindowFollower(follower,0,len(bundle['points'])-1)
        self.env=Envelope(profile['front_overhang_from_rear_axle_m'],profile.get('rear_overhang_m',.24),
                          profile['body_width_m'],profile['clearance_m'],args.wheelbase_m,args.track_m)
        self.reason=None;self.last_plan_key=None;self.s_ok=False
        self.worker=PlanWorker();self.pending=None;self.path_until_s=None
        self.s_lock_end=None
        self.replan_at=-math.inf

    def install(self,bundle,mission):
        self.bundle=bundle;mission.gates=bundle['gates']
        self.state.route=qr.ModeRoute('manual',[qr.Segment(bundle['section'],0,len(bundle['points'])-1,'HYPER main / generated local candidate')],bundle['points'])
        self.window.end=min(self.window.end,len(bundle['points'])-1)

    def boundary(self,name):
        spec=self.config.get('road_boundaries',{}).get(name,{})
        reference=(self.config.get('candidate_reference_mode') and spec.get('basis_kind')=='hyper_aerial_reference')
        if (spec.get('verified') is not True and not reference) or not spec.get('evidence') or spec.get('frame')!='map':return None
        return spec.get('polygon')

    def prepare(self,mission,pose,scan,now,s,held_valid,obstacle='unknown'):
        self.reason=None;gate=mission.gate
        if gate is None:return False
        reference=self.config.get('candidate_reference_mode') is True
        if reference and gate.kind=='s_curve':
            if self.s_lock_end is not None:
                if self.state.progress<self.s_lock_end:
                    if obstacle!='blocked':return True
                    if not (held_valid and mission.stationary.stopped_since is not None):
                        self.reason='s_blocked_wait_observed_stop';return False
                    if now-self.replan_at<.5:
                        self.reason='s_replan_cooldown';return False
                    self.replan_at=now;self.s_ok=False;self.path_until_s=None
                    self.journal.event('s_committed_path_blocked_replan',progress=self.state.progress,pose=[pose.x,pose.y,pose.yaw])
                self.s_lock_end=None;self.s_ok=False
            # Unobstructed reference keeps the existing follower. A blocked
            # corridor asks the existing bounded local planner for a detour.
            if obstacle!='blocked' and self.pending is None:
                # Detect a positive return on the upcoming reference early,
                # before the shorter braking horizon leaves too little room
                # for a bounded smooth detour. No-return is never free space.
                ahead=False
                if scan is not None and 0<=now-scan['received_at']<=.3 and gate.stop_s-s<=6.:
                    import qualifier_follow as q
                    begin=self.state.progress;end=begin
                    while end<len(self.bundle['points'])-1 and self.bundle['s'][end]-self.bundle['s'][begin]<14. and self.bundle['s'][end+1]<=gate.commit_s:end+=1
                    c,si=math.cos(pose.yaw),math.sin(pose.yaw)
                    for x,y,*_ in scan.get('points',[]):
                        if math.hypot(x,y)>12. or (-self.env.rear<=x<=self.env.front and abs(y)<=self.env.width/2):continue
                        p=type('Position',(),{'x':pose.x+c*x-si*y,'y':pose.y+si*x+c*y})()
                        if any((projection:=q.project(self.bundle['points'],i,p)) is not None and projection[0]<=self.env.width/2+self.env.margin+.125 for i in range(begin,end)):
                            ahead=True;break
                if not ahead:return True
        body=(pose.x,pose.y,pose.yaw)
        result=self.worker.poll()
        if result is not None:
            request=self.pending;self.pending=None
            fresh=(request is not None and request['gate']==gate.name and
                   0<=now-request['received_at']<=.3 and
                   math.hypot(pose.x-request['pose'][0],pose.y-request['pose'][1])<=.25 and
                   abs(qr.wrap(pose.yaw-request['pose'][2]))<=math.radians(5.))
            self.journal.event('planner_result',kind=result['kind'],fresh=fresh,
                               compute_s=result['compute_s'],error=result['error'])
            if fresh and result['error'] is None:
                selected,why=result['value']
                if result['kind']=='s':
                    self.s_ok=selected is not None
                    self.journal.event('s_candidate',**why,begin=request['begin'],end=request['end'],input_kind='current_scan')
                    if selected is not None:
                        import qualifier_follow as q
                        try:q.select_entry(selected,[0],pose.as_route_pose(),self.args.max_lateral_error_m,self.args.run_heading_deg)
                        except ValueError:
                            self.s_ok=False;self.reason='s_candidate_current_pose_entry_rejected';return False
                        self.install(replace_local_path(self.bundle,request['begin'],selected),mission)
                        self.path_until_s=self.bundle['s'][request['end']]
                        self.s_lock_end=request['end'] if why.get('offset_m',0.)!=0. else None
                        return True  # next tick observes the newly committed corridor
                else:
                    generated=why.pop('planned_points')
                    self.journal.event('parking_candidate',mission=gate.name,branch=selected,**why)
                    if selected is not None and held_valid:
                        end=len(self.bundle['points'])-1
                        self.install(append_parking(self.bundle,gate.name,selected,[qr.Point(**p) for p in generated]),mission)
                        self.window.end=end
                        return False
        if gate.kind=='s_curve' and gate.stop_s-s<=6.:
            if (not reference and not self.config.get('s_avoidance_verified')) or self.boundary('s_curve') is None:
                self.reason='s_curve_boundary_missing_no_autonomous_detour';return False
            valid=self.s_ok and self.path_until_s is not None and self.path_until_s-s>self.profile['stop_distance_m']+self.env.front
            if not valid:self.reason='s_curve_planning_or_no_observed_feasible_candidate'
            kind='s'
        elif gate.kind=='parking' and mission.holding and mission.stationary.stopped_since is not None and held_valid:
            if not self.config.get('reverse_enabled') or (self.boundary(gate.name) is None and not reference):
                self.reason='parking_boundary_reverse_validation_missing';return False
            kind='parking';valid=False
        else:return False
        if scan is None or not 0<=now-scan['received_at']<=.3:return False
        key=(gate.name,id(scan),self.state.progress)
        if self.pending is None and key!=self.last_plan_key:
            # A worker receives an immutable-by-contract snapshot, not the live cache.
            snapshot={k:v for k,v in scan.items() if not k.startswith('_')}
            observed=scan_observer(snapshot,self.profile,body,now)
            c,si=math.cos(pose.yaw),math.sin(pose.yaw)
            obstacles=[(pose.x+c*x-si*y,pose.y+si*x+c*y) for x,y,*_ in snapshot.get('points',[])
                       if not (-self.env.rear<=x<=self.env.front and abs(y)<=self.env.width/2)]
            request={'gate':gate.name,'received_at':scan['received_at'],'pose':body}
            if kind=='s':
                begin=self.state.progress;end=begin
                # Keep all original labels and cap the window at this S commit.
                horizon=10.
                for x,y in obstacles:
                    indices=range(begin,min(len(self.bundle['points']),begin+35))
                    j=min(indices,key=lambda j:math.hypot(self.bundle['points'][j].x-x,self.bundle['points'][j].y-y))
                    ahead=self.bundle['s'][j]-self.bundle['s'][begin]
                    if 0<ahead<10. and math.hypot(self.bundle['points'][j].x-x,self.bundle['points'][j].y-y)<2.:
                        horizon=max(horizon,min(14.,ahead+6.))
                while end<len(self.bundle['points'])-1 and self.bundle['s'][end]-self.bundle['s'][begin]<horizon and self.bundle['s'][end+1]<=gate.commit_s:end+=1
                request.update(begin=begin,end=end)
                submitted=self.worker.submit(kind,key,s_candidates,self.bundle['points'][begin:end+1],self.env,
                                             self.boundary('s_curve'),obstacles,observed,S_OFFSETS,reference)
            else:
                names=('t_left','t_right') if gate.name=='t_parking' else ('p_up','p_down')
                choice_key='t' if gate.name=='t_parking' else 'p'
                choice=self.bundle.get('session_choices',{}).get(choice_key)
                if choice is not None:names=(choice_key+'_'+choice,)
                submitted=self.worker.submit(kind,key,parking_choice,names,TRACK,self.env,self.boundary(gate.name),obstacles,observed,reference)
            if submitted:self.pending=request;self.last_plan_key=key
        return valid

    def transition(self,mission,pose,drive,heading,now):
        gate=mission.gate
        if gate.kind=='gear':
            start,end,direction=gate.stage_start,gate.stage_end,gate.direction
            drive.close();drive.change_direction(direction,mission.stationary,now)
            heading.change_direction(direction)
        elif gate.kind=='transition':
            bundle,start=append_common(self.bundle,gate.next_section)
            self.install(bundle,mission);end=len(bundle['points'])-1;direction=1
            drive.close();drive.change_direction(1,mission.stationary,now);heading.change_direction(1)
        else:raise ValueError('unexpected normal transition')
        # Entry validation is geometric; there is no second board ARM or bootstrap.
        import qualifier_follow as q
        virtual=travel_pose(pose,direction)
        local=self.bundle['points'][start:end+1]
        q.select_entry(local,list(range(min(3,len(local)-1))),virtual.as_route_pose(),
                       self.args.start_distance_m,self.args.arm_heading_deg)
        self.window.start=start;self.window.end=end;self.window.direction=direction
        self.state.progress=start;self.state.state=self.state.RUNNING
        if gate.name.endswith('_not_entered'):
            skipped=gate.name.removesuffix('_not_entered')
            if skipped not in mission.skipped:mission.skipped.append(skipped)
            self.journal.event('autonomous_parking_nonentry_completed',mission_name=skipped,
                               actual_pose=[pose.x,pose.y,pose.yaw],section=self.bundle['section'],
                               rule_basis='predeclared_nonentry_penalty; no manual handoff',skipped=mission.skipped)
        mission.advance()
        self.journal.event('normal_mission_transition',mission_name=gate.name,kind=gate.kind,direction=direction,
                           stage_start=start,stage_end=end,section=self.bundle['section'],completed=mission.completed)
        if self.bundle.get('preentry_policy'):self.journal.event('next_parking_preentry_policy',**self.bundle['preentry_policy'])
        self.last_plan_key=None;self.s_ok=False;self.path_until_s=None

    def finish(self,pose):
        if self.config.get('finish_mode')=='hyper_reference_goal':
            endpoint=self.bundle['points'][-1]
            return math.hypot(pose.x-endpoint.x,pose.y-endpoint.y)<=self.window.follower.cfg.end_reach_tolerance_m
        contract=self.config.get('finish_line')
        if not self.config.get('finish_rear_wheel_line_confirmed'):return None
        endpoint=self.bundle['points'][-1]
        tangent=qr.path_heading_at(self.bundle['points'],len(self.bundle['points'])-1)
        if finish_crossed((endpoint.x,endpoint.y,tangent),contract,self.args.track_m) is not True:
            raise ValueError('original road-contained route cannot prove rear-wheel finish; no arbitrary extension')
        return finish_crossed((pose.x,pose.y,pose.yaw),contract,self.args.track_m)
