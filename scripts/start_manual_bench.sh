#!/usr/bin/env bash
set -euo pipefail

task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
printf '%s\n' '실제 모터 출력이 잠긴 보드 연결 창입니다. 차량 배터리와 모터 전원은 OFF로 유지하세요.'
printf -v launch_command '%q %q --connect' /usr/bin/python3 "$task_root/tools/manual_control.py"
exec sg dialout -c "$launch_command"
