#!/usr/bin/env bash
set -euo pipefail

task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
gnss_path="/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"

if [[ ! -e "$gnss_path" ]]; then
  echo "GPS 보드를 찾을 수 없습니다. USB-C 연결을 확인하세요." >&2
  exit 1
fi

echo "GPS 걷기 기록을 시작합니다. 하늘이 트인 곳에서 Ctrl+C까지 계속 기록합니다."
exec bash "$task_root/scripts/record_rtk.sh" --until-interrupt
