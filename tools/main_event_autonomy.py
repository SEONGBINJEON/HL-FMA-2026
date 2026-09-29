"""Predeclared autonomous non-entry routes and non-progress observation.
No timeout grants drive permission or fabricates perception.
"""
import json,math
from pathlib import Path
from dataclasses import replace
import qualifier_route as qr
from main_event_core import Gate

ROOT=Path(__file__).resolve().parents[1]
PASSAGES=ROOT/'data/main_event_autonomous_passages_20260920.json'


def preentry_route(bundle):
    if bundle['config'].get('parking_completion_policy')!='preentry_road_passage':return bundle
    mission='t_parking' if bundle['section'] in ('start','after_s') else ('p_parking' if bundle['section']=='after_t' else None)
    if mission is None:return bundle
    from main_event_degraded import parking_execution
    choices=bundle.get('session_choices',{})
    if parking_execution(choices)['t' if mission=='t_parking' else 'p']['mode']=='selected_path_attempt':
        return bundle
    source=json.loads(PASSAGES.read_text());spec=source['passages'][mission]
    if not spec['feasible_geometry'] or source['frame']!='map' or not source['basis']:
        raise ValueError('autonomous passage geometry unavailable')
    label=spec['source_track']+':'+str(spec['source_raw_index']);begin=bundle['labels'].index(label)
    points=[qr.Point(i,*v,mission+'_nonentry_reference','map') for i,v in enumerate(spec['points'])]
    if math.hypot(points[0].x-bundle['points'][begin].x,points[0].y-bundle['points'][begin].y)>1e-7:
        raise ValueError('passage start differs from pinned pre-entry road')
    out=dict(bundle);out['points']=bundle['points'][:begin+1]+points[1:]
    out['labels']=bundle['labels'][:begin+1]+[mission+'_road_passage:'+str(i) for i in range(1,len(points))]
    out['s']=qr.cumulative_s(out['points'])
    if bundle['gates'][-1].name!=mission:raise ValueError('pre-entry passage must replace only final parking gate')
    gate=Gate(mission+'_not_entered','transition',out['s'][-1],out['s'][-1],next_section=spec['next_section'],approach_m=.35)
    out['gates']=bundle['gates'][:-1]+[gate]
    if any(g.commit_s>out['s'][begin] for g in out['gates'][:-1]):raise ValueError('passage cuts unfinished earlier mission')
    out['preentry_policy']={'mission':mission,'decision':'parking_not_entered_penalty_route',
        'start_label':label,'start_index':begin,'start_s':out['s'][begin],'length_m':spec['length_m'],'next_section':spec['next_section'],
        'evidence':str(PASSAGES.relative_to(ROOT)),'field_verified':False}
    import hashlib
    out['hashes']={**bundle['hashes'],str(PASSAGES.relative_to(ROOT)):hashlib.sha256(PASSAGES.read_bytes()).hexdigest()}
    return out


class ProgressWatch:
    """Reset only on meaningful physical AND along-route progress, never state names."""
    def __init__(self):self.s=None;self.xy=None;self.encoder=None;self.since=None;self.band=0;self.five=False
    def update(self,now,s,fix,board,reason=None):
        fresh=bool(fix and board and 0<=now-board.get('received_at',-1)<.15)
        if self.since is None:
            self.since=now;self.s=s;self.xy=None if not fix else fix[:2];self.encoder=None if not board else board['encoder']
        if (fresh and self.xy and s>=self.s+.5 and board['encoder']!=self.encoder and
            math.hypot(fix[0]-self.xy[0],fix[1]-self.xy[1])>=.4):
            self.since=now;self.s=s;self.xy=fix[:2];self.encoder=board['encoder']
        elapsed=max(0.,now-self.since);band=3 if elapsed>=50 else (2 if elapsed>=30 else (1 if elapsed>=15 else 0))
        five=elapsed>=5.
        changed=band!=self.band or five!=self.five;self.band=band;self.five=five
        return {'seconds_without_progress':elapsed,'band':band,'changed':changed,
                'no_progress_5s':five,'collision_proven':False,'manual_permission_granted':False,
                'encoder_changed':bool(board and board['encoder']!=self.encoder),
                'gps_displacement_m':None if not fix or not self.xy else math.hypot(fix[0]-self.xy[0],fix[1]-self.xy[1]),
                'along_route_progress_m':s-self.s,
                'signal_wait':bool(reason and reason.startswith('signal_wait:')),
                'reason':reason,'alternative':'predeclared_road_passage_for_T_P; other persistent blocks unresolved',
                'timeout_grants_drive':False}
