"""Synthetic SDK bytes -> real file Inputs -> ScanAssembler -> Mission.
No saved old-mount scans or injected coverage counters supply clear evidence.
"""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import main_event_runtime as rt
from main_event_lidar import obstacle_state
from main_event_core import Gate, Mission
from test_main_event import PROFILE,board


def sdk_scan(missing=(),occupied=None):
    lines=[]
    for i in range(720):
        a=i*.5;bad=any(lo<=a<=hi for lo,hi in missing)
        r=5000.
        if occupied is not None and abs(a-occupied[0])<=.25:r=occupied[1]*1000
        lines.append(f"{'S' if i==0 else ''} theta: {a:.2f} Dist: {0. if bad else r:.2f} Q: {0 if bad else 10}\n")
    lines.append('S theta: 0.00 Dist: 5000.00 Q: 10\n')
    return ''.join(lines).encode()


class PipelineTests(unittest.TestCase):
    def consume(self,missing=(),occupied=None,age=0.,corridor=None,direction=1):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);(d/'lidar').mkdir();(d/'rtk').mkdir()
            (d/'lidar/metadata.json').write_text(json.dumps({'mount_id':PROFILE['mount_id'],'transform_count':0}))
            raw=sdk_scan(missing,occupied);(d/'lidar/stdout.bin').write_bytes(raw)
            stamp=datetime.fromtimestamp(1000.-age,timezone.utc).isoformat()
            (d/'lidar/host_chunks.jsonl').write_text(json.dumps({'stream':'stdout','offset':0,'bytes':len(raw),'host_utc':stamp})+'\n')
            inp=rt.Inputs(d,PROFILE)
            with patch.object(rt.time,'monotonic',return_value=10.),patch.object(rt.time,'time',return_value=1000.):inp.poll(0,0)
            obs='unknown' if inp.scan is None else obstacle_state(inp.scan,PROFILE,10.,corridor,direction)
            m=Mission([Gate('later','unsupported',10.,11.)],'synthetic')
            decision=m.step(s=0.,now=10.,board=board(10.),quality_ok=True,heading_ok=True,aligned=True,obstacle=obs)
            return obs,decision

    def test_full_positive_rays_observe_straight_corridor(self):
        obs,result=self.consume();self.assertEqual(obs,'clear');self.assertTrue(result['drive'])

    def test_missing_only_behind_sensor_does_not_block_forward_corridor(self):
        obs,result=self.consume(missing=[(165,170)])
        self.assertEqual(obs,'clear');self.assertTrue(result['drive'])

    def test_missing_path_rays_remain_unknown(self):
        obs,result=self.consume(missing=[(0,2)])
        self.assertEqual(obs,'unknown');self.assertFalse(result['drive'])

    def test_small_invalid_ray_is_not_interpolated_as_clear(self):
        self.assertEqual(self.consume(missing=[(.5,.5)])[0],'unknown')

    def test_curve_requires_its_own_side_visibility(self):
        curve=[((1.,0.),(2.,0.)),((2.,0.),(2.,2.)),((2.,2.),(-2.,.5))]
        self.assertEqual(self.consume(missing=[(165,170)])[0],'clear')
        self.assertEqual(self.consume(missing=[(165,170)],corridor=curve)[0],'unknown')
        self.assertEqual(self.consume(corridor=curve)[0],'clear')

    def test_positive_return_in_curve_blocks(self):
        curve=[((1.,0.),(2.,0.)),((2.,0.),(2.,2.))]
        obs,result=self.consume(occupied=(45.,math.sqrt(2)),corridor=curve)
        self.assertEqual(obs,'blocked');self.assertFalse(result['drive'])

    def test_front_body_corner_beyond_rear_axle_path_is_occupied(self):
        curve=[((0.,0.),(1.,0.)),((1.,0.),(1.8,.8))]
        self.assertEqual(self.consume(occupied=(45.,math.sqrt(1.4**2+1.4**2)),corridor=curve)[0],'blocked')

    def test_reverse_requires_observed_rear_space(self):
        self.assertEqual(self.consume(missing=[(175,185)])[0],'clear')
        self.assertEqual(self.consume(missing=[(175,185)],direction=-1)[0],'unknown')
        self.assertEqual(self.consume(occupied=(180.,2.),direction=-1)[0],'blocked')

    def test_stale_receipt_is_discarded_before_mission(self):
        obs,result=self.consume(age=.31);self.assertEqual(obs,'unknown');self.assertFalse(result['drive'])

if __name__=='__main__':unittest.main()
