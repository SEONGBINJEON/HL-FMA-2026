# 전달 소스의 핀·문법 수정 초안

**업로드하지 않은 초안. 현재 보드에서 실행 중인 코드로 확인되지 않았다.**

원본: `reference/vehicle/arduino_joystick_ultrasonic_2026-09-16.ino` (Downloads/아두이노.txt의 동일 사본).
사용자가 처음 알려준 핀을 재확인하여 중앙 TRIG 43→41, 오른쪽 TRIG 29→37, 오른쪽 ECHO 33→35를 수정했다. 나머지 핀은 유지했다. 원본은 loop() 닫는 중괄호가 없어 컴파일 실패; 초안 끝에 중괄호 하나를 추가했다.

Arduino CLI / arduino:avr:mega / AVR core 1.8.8로 컴파일 성공: flash 8302 bytes, RAM 662 bytes. 이는 문법·빌드 검사이며 실장치 제어·초음파 정지 검증이 아니다.

현재 수신은 `HC-SR04 3 SENSOR TEST` 및 `LEFT : ... | CENTER : ... | RIGHT : ...`다. 첨부 코드의 `F8 JOYSTICK CONTROL READY` / `[MONITOR] US=...`와 다르다. 사용자는 주행+초음파 정지 코드라고 설명했으나 차이는 미해소다. 원본 readback은 하지 않았다.

이 초안은 기능 검토를 마친 운용 펌웨어가 아니다. 기존 type144 호스트와 호환되지 않으며, 999 무응답의 정지 해제 처리와 반복 조향 명령의 한계 처리를 추가 검토해야 한다. 이번에는 제어 기능 변경·업로드·차량 명령을 수행하지 않았다.
