#!/usr/bin/env bash
set -euo pipefail

PRACTICE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-}"

case "$MODE" in
  check)
    exec bash "$PRACTICE_ROOT/scripts/record_field_session.sh" \
      --duration 60 \
      --camera-serial 341522300519 \
      --camera-seconds 60 \
      --camera-fps 15 \
      --skip-lidar \
      --require-control-journal \
      --practice-mode check \
      --context "현장 도착 후 60초 전체 센서 수신 점검" \
      --mounting "차량 장착 상태; 정확한 외부파라미터는 별도 측정 필요"
    ;;
  first-lap)
    exec bash "$PRACTICE_ROOT/scripts/record_field_session.sh" \
      --duration 600 \
      --camera-serial 341522300519 \
      --camera-seconds 600 \
      --camera-fps 30 \
      --skip-lidar \
      --require-control-journal \
      --practice-mode first-lap \
      --auto-lap \
      --context "용인 첫 기준 주행: RTK/IMU/LiDAR/RGB/depth30fps/엔코더·조작 전 구간" \
      --mounting "차량 장착 상태; 주행 전 센서 위치 변경 여부 기록 필요"
    ;;
  route-lap)
    exec bash "$PRACTICE_ROOT/scripts/record_field_session.sh" \
      --duration 600 \
      --camera-serial 341522300519 \
      --camera-seconds 600 \
      --camera-fps 15 \
      --skip-lidar \
      --require-control-journal \
      --practice-mode route-lap \
      --auto-lap \
      --context "현장 반복 경로 주행: RTK/IMU/LiDAR/RGB/depth/엔코더·조작 전 구간" \
      --mounting "차량 장착 상태; 주행 전 센서 위치 변경 여부 기록 필요"
    ;;
  fallback-lap|phone-lap)
    exec bash "$PRACTICE_ROOT/scripts/record_field_session.sh" \
      --until-interrupt \
      --require-control-journal \
      --practice-mode route-lap \
      --auto-lap \
      --context "용인 휴대폰 전방영상 주행: RTK/IMU/LiDAR/엔코더·조작 전 구간; 휴대폰 영상 별도" \
      --mounting "차량 장착 상태; D455는 제거, 휴대폰을 기존 카메라 방향에 장착"
    ;;
  *)
    echo "사용법: bash scripts/practice_data.sh {check|first-lap|route-lap|phone-lap}" >&2
    echo "각 실행은 새 폴더에 전체 센서와 엔코더·조작 로그를 기록합니다." >&2
    exit 2
    ;;
esac
