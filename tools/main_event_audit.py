"""Read-only handoff evidence checks. No device opens, model inference or secrets."""
import json
from pathlib import Path
from main_event_plan import ROOT, digest
from main_event_lidar import transform_node


def main():
    failures=[]
    for name in ('leg1_light2_163735','leg1_light2_163850','leg1_light2_163906','leg1_light2_163917'):
        path=ROOT/'data/qualifier_20260919'/name/'metadata.json'
        row=json.loads(path.read_text())
        stop=row.get('final_stop_attempt') or {}
        failures.append({'session':name,'metadata_sha256':digest(path),'status':row.get('status'),
                         'exit_code':row.get('exit_code'),'reason':row.get('reason'),
                         'stop_sent':stop.get('sent'),'stop_ack_request':stop.get('ack_req')})
    print(json.dumps({'prior_failure_evidence':failures,'cause':'battery/mechanical cause remains unconfirmed'},ensure_ascii=False,indent=2))
    profile=json.loads((ROOT/'config/main_event_lidar_mount.json').read_text())
    old=ROOT/'data/lidar_20260919_163147_c2671e8f/metadata.json'
    old_meta=json.loads(old.read_text())
    try:transform_node(0.,2000.,10,profile,old_meta.get('mount_id'))
    except ValueError as exc:
        print(json.dumps({'old_mount_new_transform':'REJECTED','reason':str(exc),'source':str(old.relative_to(ROOT)),
                          'raw_scan_read_or_changed':False},ensure_ascii=False))
    else:raise AssertionError('old mounting was incorrectly accepted')
    prior=json.loads((ROOT/'data/qualifier_20260919/hyper_signal_crop070/report.json').read_text())
    adapter=ROOT/'data/main_event_signal_smoke_20260920.jsonl'
    comparisons=[]
    for line in adapter.read_text().splitlines():
        row=json.loads(line);p=Path(row['file'])
        expected=next(x for x in prior['folders'][str(p.parent)]['rows'] if x['file']==p.name)
        assert expected['sign']==row['sign']
        assert row['capture_at'] is None and row['valid'] is False and row['input_kind']=='recorded'
        comparisons.append({'file':row['file'],'prior':expected['sign'],'adapter':row['sign'],'inference_s':row['inference_s']})
    print(json.dumps({'stored_adapter_matches':comparisons,'full_200_image_retest':False},ensure_ascii=False))
    evidence=sorted((ROOT/'data').glob('lidar_direction_oblique_*.json'))
    observation=None if not evidence else json.loads(evidence[-1].read_text())
    print(json.dumps({'serial_by_id':sorted(str(p) for p in Path('/dev/serial/by-id').glob('*')),
                      'device_access':'filesystem listing only',
                      'stationary_direction_evidence':None if not evidence else str(evidence[-1].relative_to(ROOT)),
                      'local_ordering_observation':None if observation is None else observation['proposed_mapping'],
                      'live_calibration_approved':False}))
    return 0


if __name__=='__main__':raise SystemExit(main())
