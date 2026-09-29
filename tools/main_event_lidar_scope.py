"""User-confirmed mission-only LiDAR stop scope; raw observations stay raw."""
def lidar_stop_scope(bundle,s,phase):
    if phase!='running':return False,'departure_lidar_stop_not_applied'
    if s is None:return False,'no_route_progress'
    if bundle.get('section','').startswith('parking_'):
        return True,'selected_parking_directional_motion'
    for g in bundle['gates']:
        if g.kind=='s_curve' and g.stop_s-6.<=s<=g.commit_s+.24:
            return True,'S_approach_performance_exit'
    # HYPER mission_track: common_2 accel_start #227 to accel_end #300,
    # obstacle_hold_s applies here. Existing dummy simulation also uses #227.
    labels=bundle['labels']
    if 'common_2:227' in labels and 'common_2:300' in labels:
        a=bundle['s'][labels.index('common_2:227')];z=bundle['s'][labels.index('common_2:300')]
        if a-6.<=s<=z+.24:return True,'dummy_accel_approach_performance_exit'
    return False,'general_lidar_stop_not_applied'
