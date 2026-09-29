"""Read-only HYPER main geometry audit; no field signal/road confirmation."""
import json,math
import qualifier_route as qr
from main_event_plan import TRACK,LABELS,digest
from main_event_maneuvers import parking_paths,PARKING_SEGMENTS,assess_path,Envelope,parking_choice
from path_follow_inputs import left_wheel_cdeg


def main():
    report={'input_kind':'recorded_reference_not_field_measurement','signals':{},'parking':{}}
    for name,track,index in [('light_2','common_1',228),('light_3','common_2',98)]:
        path=TRACK/(track+'.csv');pts=qr.load_track_csv(track,path).points;ss=qr.cumulative_s(pts)
        before=max(i for i in range(index) if ss[index]-ss[i]>=5.)
        after=next(i for i in range(index+1,len(pts)) if ss[i]-ss[index]>=10.)
        incoming=math.atan2(pts[index].y-pts[before].y,pts[index].x-pts[before].x)
        outgoing=math.atan2(pts[after].y-pts[index].y,pts[after].x-pts[index].x)
        report['signals'][name]={'track':track,'raw_index':index,'window':[before,after],
            'signed_turn_deg':math.degrees(qr.wrap(outgoing-incoming)),'sha256':digest(path),
            'allowed_signal_confirmed':False,'basis':'path geometry only; no signal phase correspondence inferred'}
    env=Envelope(1.04,.24,.77,.1);wide=[(-100,-100),(100,-100),(100,100),(-100,100)]
    for name in PARKING_SEGMENTS:
        report['parking'][name]=[{'direction':d,'points':len(p),'geometry_only':assess_path(p,env,wide,[],lambda _:True,direction=d)} for p,d in parking_paths(name,TRACK)]
    for names in [('t_left','t_right'),('p_up','p_down')]:
        selected,result=parking_choice(names,TRACK,env,wide,[],lambda _:True)
        result.pop('planned_points',None)
        report['parking']['generated_'+names[0]]={'selected':selected,**result}
    pts=qr.load_track_csv('common_1',TRACK/'common_1.csv').points
    report['s_asset']={'raw_range':[130,190],'quality':qr.quality_report(qr.Track('S','',digest(TRACK/'common_1.csv'),[],pts[130:191]))}
    clean=[p for p in pts[130:191] if p.idx!=172]
    report['s_asset']['known_exclusion_geometry']=assess_path(clean,env,wide,[],lambda _:True)
    print(json.dumps(report,ensure_ascii=False,indent=2));return 0

if __name__=='__main__':raise SystemExit(main())
