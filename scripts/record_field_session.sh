#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
task_dry_run=false
task_has_dialout=false
for task_arg in "$@"; do
  [[ "$task_arg" == --dry-run || "$task_arg" == --help || "$task_arg" == -h ]] && task_dry_run=true
done
for task_group in $(id -nG); do
  [[ "$task_group" == dialout ]] && task_has_dialout=true
done
if ! "$task_dry_run" && ! "$task_has_dialout"; then
  # sg uses /bin/sh: POSIX single-quote every argument, including spaces/newlines.
  task_command="exec bash"
  for task_arg in "$task_root/scripts/record_field_session.sh" "$@"; do
    task_quoted=${task_arg//\'/\'\\\'\'}
    task_command+=" '$task_quoted'"
  done
  exec sg dialout -c "$task_command"
fi
exec python3 "$task_root/tools/record_field_session.py" "$@"
