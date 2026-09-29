#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" != "--motor-power-off" || "$#" != 1 ]]; then
  echo '모터 전원 OFF와 확보된 시험 공간·차단 담당자 준비 후 --motor-power-off 인수로 실행하세요.' >&2
  exit 2
fi
STEP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
printf -v STEP_COMMAND '%q ' python3 "$STEP_ROOT/tools/step_manual_control.py" --connect --motor-power-off
exec sg dialout -c "$STEP_COMMAND"
