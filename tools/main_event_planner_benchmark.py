"""Synthetic planner wall-time measurement, no devices and no field approval."""
import json,math,time
import qualifier_route as qr
from main_event_maneuvers import Envelope,s_candidates,scan_observer,parking_choice
from main_event_plan import TRACK

def main():
    profile={'x_m':1.12,'y_m':0.,'front_overhang_from_rear_axle_m':1.04,'body_width_m':.77,'rear_overhang_m':.24}
    scan={'received_at':1.,'complete_candidate':True,'rays':[(math.radians(i*.5),50.) for i in range(720)]}
    observer=scan_observer(scan,profile,(0.,0.,0.),1.)
    points=[qr.Point(i,i*.5,0.,0.,'synthetic','map') for i in range(33)]
    road=[(-100.,-100.),(100.,-100.),(100.,100.),(-100.,100.)];env=Envelope(1.04,.24,.77,.1)
    result={}
    for name,obstacles in [('clear',[]),('left',[(8.,.35)]),('both',[(8.,i*.1) for i in range(-30,31)])]:
        start=time.perf_counter();path,why=s_candidates(points,env,road,obstacles,observer)
        result[name]={'elapsed_s':time.perf_counter()-start,'path':path is not None,'reason':why}
    start=time.perf_counter();branch,why=parking_choice(('t_left','t_right'),TRACK,env,road,[],observer)
    why.pop('planned_points',None);result['parking']={'elapsed_s':time.perf_counter()-start,'branch':branch,**why}
    print(json.dumps({'input_kind':'synthetic','not_control_loop_deadline_proof':True,'result':result},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
