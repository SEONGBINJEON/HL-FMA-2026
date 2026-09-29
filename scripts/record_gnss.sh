#!/usr/bin/env bash
# Record the single connected, identified u-blox receiver. Default: 60 seconds.
set -Eeuo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$task_dir/tools/start_gnss_recording.py" "$@"
