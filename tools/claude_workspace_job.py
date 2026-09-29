#!/usr/bin/env python3
"""One Claude job in a prepared copy, with named edit files and a fixed test command.

The fixed test command must be reviewed by Codex and isolate its child process.
This launcher is a permission scope, not an OS sandbox for Claude itself.
No automatic merge, retries, model override, device work, or scheduler.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('manifest')
    args = ap.parse_args()
    manifest = json.loads(Path(args.manifest).read_text())
    cwd = Path(manifest['workspace']).resolve()
    prompt = (cwd / 'TASK.md').read_text()
    allowed = ['Read', 'Glob', 'Grep', 'Bash(python3 tools/c5_checks.py)']
    for rel in manifest['editable_files']:
        p = Path(rel)
        if p.is_absolute() or '..' in p.parts or any(c in rel for c in '()*?,\n'):
            raise ValueError('Invalid editable file path')
        allowed.extend([f'Write(./{rel})', f'Edit(./{rel})'])
    cmd = ['claude', '-p', '--safe-mode', '--tools', 'Read,Glob,Grep,Write,Edit,Bash',
           '--permission-mode', 'dontAsk', '--allowedTools', ','.join(allowed),
           '--strict-mcp-config', '--mcp-config', '{"mcpServers":{}}',
           '--no-session-persistence', '--output-format', 'stream-json', '--verbose']
    print(json.dumps({'workspace': str(cwd), 'editable_files': manifest['editable_files'],
                      'allowed_test_command': 'python3 tools/c5_checks.py',
                      'timeout_s': 900}), file=sys.stderr, flush=True)
    # Only tool names/paths and the final result are retained. Streaming model
    # text, reasoning, and tool-result bodies are deliberately not recorded.
    timed_out = threading.Event()
    result = None
    with tempfile.TemporaryFile(mode='w+t') as errors:
        p = subprocess.Popen(cmd, cwd=cwd, text=True, stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, stderr=errors, start_new_session=True)
        def terminate_job():
            if p.poll() is None:
                timed_out.set()
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        timer = threading.Timer(900, terminate_job)
        timer.daemon = True
        timer.start()
        try:
            p.stdin.write(prompt)
            p.stdin.close()
            for line in p.stdout:
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if event.get('type') == 'result':
                    result = event
                elif event.get('type') == 'assistant':
                    for block in event.get('message', {}).get('content', []):
                        if block.get('type') == 'tool_use':
                            progress = {'event': 'tool', 'name': block.get('name')}
                            path = block.get('input', {}).get('file_path')
                            if isinstance(path, str):
                                candidate = (cwd / path).resolve()
                                if candidate.is_relative_to(cwd):
                                    progress['file'] = str(candidate.relative_to(cwd))
                            print(json.dumps(progress), file=sys.stderr, flush=True)
            p.wait()
        except KeyboardInterrupt:
            terminate_job()
            p.wait()
            return 130
        finally:
            timer.cancel()
        errors.seek(0)
        sys.stderr.write(errors.read())
    if result is not None:
        print(json.dumps(result, ensure_ascii=False))
    if timed_out.is_set():
        print('Claude workspace job timed out; inspect logs, no automatic retry', file=sys.stderr)
        return 124
    if p.returncode:
        return p.returncode
    return 65 if result is None or result.get('is_error') else 0


if __name__ == '__main__':
    sys.exit(main())
