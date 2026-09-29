"""Freeze/verify the candidate, excluding credentials and all firmware HEX files."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

ROOT=Path(__file__).resolve().parents[1]
DEFAULT=ROOT/'docs/handoff/main_event_release_20260920.json'
BASELINE={'tools/qualifier_follow.py':'620049531dcadf50324ef23b0fe9635ef9ca0249bac2a05bbbafe0e20bb7db1b',
          'firmware/joystick_manual/joystick_manual.ino':'bcdf895b6915427b428ac53b1c5441f81603d6a232490d06794c05a35849429c'}


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sources():
    paths=list((ROOT/'tools').glob('main_event*.py'))+list((ROOT/'tests').glob('test_main_event*.py'))
    paths+=list((ROOT/'reference/external/HYPER/src/planning/hyper_waypoint/waypoints/track').glob('*.csv'))
    names=['data/main_event_parking_recheck_20260920_1422.json','data/main_event_selected_parking_runtime_20260920.json','data/main_event_parking_staging_binding_20260920.json','data/main_event_parking_live_ready_20260920.json','tmp/codex1_parking_recheck/check.py','tmp/codex1_parking_recheck/selected_runtime.py','tmp/codex1_parking_recheck/dynamic.py','tmp/codex1_parking_recheck/observe_staging.py','tmp/codex1_parking_recheck/ready_evidence.py','tmp/codex1_degraded/audit.py','tmp/codex1_degraded/extra_tests.py','tmp/codex1_degraded/preserve_check.py','data/main_event_degraded_replay_20260920.json','data/main_event_degraded_network_20260920.json','data/main_event_degraded_live_evidence_20260920.json','tmp/codex1_degraded/live_check.py','tmp/codex1_degraded/replay_lead.py','tmp/codex1_degraded/network_check.py','tmp/codex1_degraded/targeted.py','data/main_event_final_left_ready_evidence_20260920.json','tmp/codex1_left_ready/live_check.py','tmp/codex1_left_ready/audit.py',
           'data/main_event_startup_serial_evidence_20260920.json','tmp/codex1_startup/evidence.py',
           'data/main_event_s_stall_probe_20260920.json','data/main_event_s_stall_comparison_20260920.json',
           'tmp/codex1_s_stall/probe.py','tmp/codex1_s_stall/compare.py',
           'data/main_event_live_camera_blocker_evidence_20260920.json','tmp/codex1_live/camera_inference.py',
           'tmp/codex1_live/camera_1080p15_diagnostic_preserved.py','data/codex1_live_camera_inference_20260920/report.json',
           'data/main_event_autonomous_passages_20260920.json','data/main_event_autonomous_passages_20260920.png',
           'data/main_event_autonomous_route_scope_20260920.png','data/main_event_finish_fallback_scope_20260920.json',
           'tmp/codex1_autonomy/build_passages.py','tmp/codex1_autonomy/route_scope.py','tmp/codex1_autonomy/finish_scope.py',
           'tools/rtk_record.py','tools/ntrip_recovery.py','tools/lidar_record.py','tools/gnss_set_rate.py',
           'tools/gnss_record.py','tools/start_gnss_recording.py','tools/hyper_signal_offline.py',
           'tools/qualifier_follow.py','tools/qualifier_route.py','tools/live_path_follow.py','tools/path_follow_inputs.py',
           'tools/joystick_desktop.py','tools/joystick_control.py','tools/gamepad_input.py',
           'tools/path_steering.py','tools/pose_reference.py','tools/steering_calibration.py','tools/steer_target_host.py',
           'tools/live_steering_trial.py','tools/joystick_protocol.py','tools/step_manual_protocol.py','tools/step_manual_control.py',
           'tests/test_ntrip_recovery.py','tests/test_rtk_record.py','tests/test_lidar_record.py',
           'config/main_event.json','config/main_event_lidar_mount.json',
           'config/main_event_s_reference_boundary.json',
           'data/main_event_recovery_map_20260920.png','data/main_event_reference_review_20260920.json',
           'data/main_event_cone_scope_20260920.json','data/main_event_s_segment_scope_20260920.json',
           'data/main_event_parking_photo_review_20260920.jpg','data/main_event_parking_branch_geometry_20260920.png',
           'tmp/codex1_menu/photo_review.py','tmp/codex1_menu/reference_review.py',
           'tmp/codex1_menu/cone_probe.py','tmp/codex1_menu/s_segment_check.py',
           'tmp/codex1_speed/practice_speed.py','tmp/codex1_speed/feasibility.py',
           'tmp/codex1_speed/final_evidence.py','data/main_event_speed_input_binding_20260920.json',
           'data/main_event_pwm100_practice_review_20260920.json','data/main_event_feasibility_retained_20260920.json',
           'data/claude3_follow_20260919_020600/drive_calibration_full_table.json',
           'data/daegu_scurve_hyper_20260919_045730/qual_route_candidate/common_1_hyper.csv',
           'data/daegu_scurve_hyper_20260919_045730/qual_route_candidate/common_1_hyper.meta.json',
           'reference/external/HYPER/src/planning/hyper_planner/mission/mission_track.yaml',
           'reference/external/HYPER/src/planning/hyper_planner/config/nav2_controller.yaml',
           'reference/external/HYPER/src/simulator/hyper_gazebo/worlds/models/driving_course/meshes/real_course.png',
           'reference/external/HYPER/src/simulator/hyper_gazebo/worlds/models/driving_course/meshes/real_course.align.yaml',
           'reference/external/HYPER/src/perception/hyper_object_detection/models/best_track.pt',
           '.setup/rplidar_sdk_built/ultra_simple','.setup/rplidar_sdk_built/build_provenance.json',
           'firmware/joystick_manual/joystick_manual.ino']
    paths += [ROOT/n for n in names]
    import steer_target_host as h
    paths += [h.LIB_PATH,h.LIB_PATH.parent/'build_info.json']
    build=json.loads((h.LIB_PATH.parent/'build_info.json').read_text())
    paths += [ROOT/n for n in build['sources_sha256']]
    return sorted(set(p.relative_to(ROOT).as_posix() for p in paths))


def verify(path=DEFAULT):
    manifest=json.loads(Path(path).read_text())
    mismatched=[]
    for name,expected in manifest['sha256'].items():
        if not (ROOT/name).is_file() or sha(ROOT/name)!=expected:mismatched.append(name)
    if mismatched:raise ValueError('release mismatch: '+', '.join(mismatched))
    return manifest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('freeze','verify','rollback-check'))
    p.add_argument('--manifest',type=Path,default=DEFAULT)
    a=p.parse_args()
    if a.action=='freeze':
        for name,expected in BASELINE.items():
            if sha(ROOT/name)!=expected:raise ValueError('proven baseline changed: '+name)
        live=json.loads((ROOT/'data/main_event_parking_live_ready_20260920.json').read_text())
        report={'release_id':datetime.now(ZoneInfo('Asia/Seoul')).strftime('main_event_candidate_%Y%m%d_%H%M%S'),
                'created_kst':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
                'race_ready':False,'hardware_validated_this_turn':live.get('passed',False),
                'hardware_validation_scope':'stationary_staging_READY_DISARM_only_no_start_left_running_for_user' if live.get('passed') else 'pending_stationary_check',
                'stationary_READY_evidence':'data/main_event_parking_live_ready_20260920.json',
                'stationary_lidar_evidence':'data/lidar_direction_oblique_20260920_0636.json',
                'optional_perception_separated_from_core_READY':True,'bounded_stopped_GPS_continuity_recovery':True,
                'prestart_session_choices':{'T':'right_selected_path_left_infeasible_nonentry','P':'up_or_down_selected_path','finish':'existing_HYPER_lane'},
                'USB_C_5G_validation':'pending_user_connection','vehicle_motion_validated_this_turn':False,'manual_handoff_live_validated':False,
                'sha256':{name:sha(ROOT/name) for name in sources()},
                'rollback':'backups/main_event_20260920_035401',
                'previous_candidate_preserved':'backups/main_event_preserved_141817_20260920_degraded',
                'previous_release_id':'main_event_candidate_20260920_141817',
                'S_stall_evidence':'data/main_event_s_stall_comparison_20260920.json','S_stall_field_validated':False,
                'camera_live_continuous_validated':True,'camera_hardware_only_this_turn':False,
                'stationary_prepare_shutdown_live_validated_this_turn':False,
                'camera_evidence':'data/main_event_live_camera_blocker_evidence_20260920.json',
                'parking_completion_policy':'session_selected_feasible_paths_else_nonentry','autonomous_T_P_field_validated':False,
                'autonomous_T_P_dummy_runtime_simulated':True,'finish_unrecognized_fallback_supported':True,
                'manual_rule':'actual_S_collision_judge_exception_only','manual_protocol_simulated':True,'cone_classifier_runtime_integrated':False,
                'section_pwm100_field_validated':False,'speed_policy_recorded_evidence':'data/main_event_pwm100_practice_review_20260920.json',
                'candidate_feasibility_evidence':'data/main_event_feasibility_retained_20260920.json',
                'firmware_upload_included':False,'credentials_included':False}
        with a.manifest.open('x') as out:json.dump(report,out,ensure_ascii=False,indent=2);out.write('\n')
        print(json.dumps({'release_id':report['release_id'],'files':len(report['sha256']),'manifest':str(a.manifest),'race_ready':False}))
    elif a.action=='verify':
        report=verify(a.manifest)
        print(json.dumps({'release_id':report['release_id'],'files_verified':len(report['sha256']),'race_ready':False}))
    else:
        backup=ROOT/'backups/main_event_20260920_035401'
        names=['tools/qualifier_follow.py','tools/rtk_record.py','tools/lidar_record.py']
        result={name:sha(backup/name) for name in names}
        if result['tools/qualifier_follow.py']!=BASELINE['tools/qualifier_follow.py']:raise ValueError('baseline backup mismatch')
        print(json.dumps({'backup':str(backup),'sha256':result,'restored':False,'firmware_restore':False,
                          'scope':'prior partial forward tracking only; not a complete main-event controller'},ensure_ascii=False,indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
