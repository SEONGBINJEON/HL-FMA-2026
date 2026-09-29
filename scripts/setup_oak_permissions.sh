#!/usr/bin/env bash
set -euo pipefail
echo 'OAK 카메라(USB vendor 03e7)의 접근 권한만 설정합니다.'
echo 'Arduino/차량 구동 명령이나 USB 전체 재설정은 하지 않습니다.'
echo '비밀번호는 이 터미널의 sudo 프롬프트에 직접 입력하세요.'
task_rule_file=$(mktemp)
trap 'rm -f "$task_rule_file"' EXIT
printf '%s\n' 'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="03e7", MODE="0660", GROUP="dialout"' > "$task_rule_file"
sudo install -m 0644 "$task_rule_file" /etc/udev/rules.d/80-hl-fma-oak.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=03e7 --action=change
echo 'OAK 권한 설정 완료. 카메라 수신을 이어서 확인합니다.'
read -r -p 'Enter를 누르면 이 터미널을 닫습니다. ' || true
