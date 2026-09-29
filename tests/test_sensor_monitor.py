import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import sys
from datetime import datetime, timedelta, timezone

sys.path.insert(0, str(Path(__file__).parents[1]/'tools'))
from ultrasonic_telemetry import parse_test_line
spec = importlib.util.spec_from_file_location('sensor_monitor', Path(__file__).parents[1]/'tools/sensor_monitor.py')
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


class SensorViewTests(unittest.TestCase):
    def test_no_echo_is_not_a_clear_distance(self):
        sample = parse_test_line('LEFT : 55.4 cm | CENTER : NO ECHO | RIGHT : 104.9 cm\r')
        self.assertEqual(sample['ultrasonic_cm'], [55.4, None, 104.9])
        self.assertEqual(sample['no_echo'], [False, True, False])
        self.assertIsNone(parse_test_line('LEFT : 55.4 cm | CENTER : NO ECHO | RIGHT : 10'))

    def test_ultrasonic_stops_being_fresh_when_raw_file_stops(self):
        import os
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder/'arduino').mkdir()
            raw = folder/'arduino/raw_rx.bin'
            raw.write_bytes(b'LEFT : 55.4 cm | CENTER : NO ECHO | RIGHT : 104.9 cm\n')
            os.utime(raw, (1,1))
            result = monitor.state(folder)['ultrasonic']
            self.assertGreater(result['age_s'], 2)
            self.assertIsNone(result['sample']['ultrasonic_cm'][1])

    def test_incomplete_newest_json_does_not_hide_last_complete_sample(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'sample.jsonl'
            path.write_bytes(b'{"valid":true,"value":3}\n{"valid":')
            self.assertEqual(monitor.last_json(path), {'valid':True, 'value':3})

    def test_stopped_lidar_never_plots_cached_points_as_live(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            lidar = folder/'lidar'
            lidar.mkdir()
            (folder/'session.json').write_text(json.dumps({'lidar_folder':str(lidar)}))
            (lidar/'stdout.bin').write_bytes(b'S theta: 90.00 Dist: 1200.00 Q: 15\n')
            old = (datetime.now(timezone.utc)-timedelta(seconds=10)).isoformat()
            (lidar/'host_chunks.jsonl').write_text(json.dumps({'host_utc':old})+'\n')
            result = monitor.state(folder)
            self.assertEqual(result['lidar']['points'], [])
            self.assertGreaterEqual(result['lidar']['age_s'], 10)
            fresh = datetime.now(timezone.utc).isoformat()
            (lidar/'host_chunks.jsonl').write_text(json.dumps({'host_utc':fresh})+'\n')
            self.assertEqual(monitor.state(folder)['lidar']['points'], [[90.,1.2]])

    def test_missing_time_is_unknown_not_fresh(self):
        self.assertIsNone(monitor.age_utc(None))
        self.assertIsNone(monitor.age_utc('bad'))


if __name__ == '__main__':
    unittest.main()
