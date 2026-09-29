import json, sys, tempfile, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
import dev_run_log


class Tests(unittest.TestCase):
    def run_tool(self, extra, code):
        with tempfile.TemporaryDirectory(prefix='devrunlog-') as tmp:
            argv = ['--name', 'unit_probe', '--log-root', tmp, '--source', 'tools/dev_run_log.py',
                    '--source', 'does/not/exist.txt', *extra, '--',
                    sys.executable, '-c', f'import sys; print("out-line"); print("err-line", file=sys.stderr); sys.exit({code})']
            rc = dev_run_log.main(argv)
            runs = list(Path(tmp).iterdir())
            self.assertEqual(len(runs), 1)
            d = runs[0]
            return rc, json.loads((d / 'run.json').read_text()), (d / 'stdout.log').read_text(), \
                (d / 'stderr.log').read_text(), (d / 'result.md').read_text()

    def test_child_exit_code_and_output_are_preserved(self):
        rc, run, out, err, result = self.run_tool([], 3)
        self.assertEqual(rc, 3)
        self.assertEqual(run['exit_code'], 3); self.assertEqual(run['status'], 'failed')
        self.assertEqual(out.strip(), 'out-line'); self.assertEqual(err.strip(), 'err-line')
        self.assertIn('종료코드 3', result)
        rc, run, *_ = self.run_tool([], 0)
        self.assertEqual((rc, run['status'], run['exit_code']), (0, 'completed', 0))

    def test_declared_metadata_defaults_to_unknown_not_none(self):
        rc, run, out, err, result = self.run_tool([], 0)
        self.assertEqual(run['operator_declared'], 'unspecified')
        self.assertEqual(run['stage_declared'], 'unspecified')
        self.assertEqual(run['hardware_access_declared'], 'unknown')
        self.assertNotIn('hardware_access', {k for k in run if k != 'hardware_access_declared'})
        self.assertIn('hardware_access_declared', run['declared_not_observed'])
        self.assertFalse(run['credential_scrubbing'])
        self.assertIn('미확인(선언 없음)', result); self.assertIn('선언한 것이며', result)
        self.assertNotIn('접근: 없음', result)

    def test_declared_metadata_is_recorded_consistently(self):
        rc, run, out, err, result = self.run_tool(
            ['--operator', 'user+electrical', '--stage', 'A1', '--hardware', 'used', '--input-kind', 'measured',
             '--note', 'declared for test only'], 0)
        self.assertEqual(run['operator_declared'], 'user+electrical'); self.assertEqual(run['stage_declared'], 'A1')
        self.assertEqual(run['hardware_access_declared'], 'used'); self.assertEqual(run['input_kind'], 'measured')
        self.assertIn('실행자(선언): user+electrical', result); self.assertIn('단계(선언): A1', result)
        self.assertIn('하드웨어 접근(선언): 사용(선언)', result); self.assertIn('입력(선언): measured', result)
        self.assertEqual(run['source_sha256']['does/not/exist.txt'], None)
        self.assertEqual(len(run['source_sha256']['tools/dev_run_log.py']), 64)

    def test_missing_command_is_a_failed_run_not_a_crash(self):
        with tempfile.TemporaryDirectory(prefix='devrunlog-') as tmp:
            rc = dev_run_log.main(['--name', 'missing', '--log-root', tmp, '--', 'definitely-not-a-command-xyz'])
            run = json.loads(next(Path(tmp).iterdir()).joinpath('run.json').read_text())
            self.assertEqual((rc, run['status'], run['exit_code']), (127, 'failed', 127))


if __name__ == '__main__':
    unittest.main()
