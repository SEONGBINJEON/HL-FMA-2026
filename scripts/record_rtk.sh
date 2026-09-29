#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$task_root/.venv-rtk/bin/python" ]]; then
  echo 'RTK 전용 실행 환경을 먼저 준비해야 합니다.' >&2
  exit 1
fi
exec "$task_root/.venv-rtk/bin/python" "$task_root/tools/start_rtk_recording.py" "$@"
