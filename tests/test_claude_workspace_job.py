"""Verify subprocess metadata logging without calling a model or any hardware."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

RUNNER = Path(__file__).resolve().parents[1] / 'tools/claude_workspace_job.py'


class WorkspaceJobLogging(unittest.TestCase):
    def test_private_stream_content_is_not_retained(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'TASK.md').write_text('Synthetic test only.')
            (root / 'manifest.json').write_text(json.dumps({
                'workspace': str(root), 'editable_files': ['tools/example.py']}))
            fake = root / 'claude'
            fake.write_text('''#!/usr/bin/python3
import json,sys
sys.stdin.read()
print(json.dumps({'type':'assistant','message':{'content':[
 {'type':'thinking','thinking':'PRIVATE_TEST_MARKER'},
 {'type':'text','text':'UNRETAINED_TEXT_MARKER'},
 {'type':'tool_use','name':'Write','input':{'file_path':'tools/example.py','content':'UNRETAINED_CODE_MARKER'}}]}}))
print(json.dumps({'type':'result','is_error':False,'result':'Final summary'}))
''')
            fake.chmod(0o755)
            env = dict(os.environ, PATH=str(root) + ':/usr/bin:/bin')
            proc = subprocess.run(['/usr/bin/python3', str(RUNNER), str(root / 'manifest.json')],
                                  env=env, capture_output=True, text=True, timeout=10)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            combined = proc.stdout + proc.stderr
            for marker in ('PRIVATE_TEST_MARKER', 'UNRETAINED_TEXT_MARKER', 'UNRETAINED_CODE_MARKER'):
                self.assertNotIn(marker, combined)
            self.assertEqual(json.loads(proc.stdout)['result'], 'Final summary')
            self.assertIn('tools/example.py', proc.stderr)


if __name__ == '__main__':
    unittest.main()
