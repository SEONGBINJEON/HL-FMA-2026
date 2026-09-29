# 두 스틱 수동 주행 (type150)

현재 업로드: `data/joystick_upload_20260916_034145/metadata.json`. 이전 보드 원본 flash/EEPROM: `data/mega_before_joystick_20260916_015151/`. type144/145/146은 과거 기록이다.

- 왼쪽상하=전후진,오른쪽좌우=조향,동시가능. 구동PWM초기60,방향키위/아래각누름±5,범위0~255. 조향PWM60. 기울기비례출력이아니다. PWM0은구동브레이크,스틱유지중재증가시재구동.
- 주행/조향시간상한없음. 전기적ADC레일방향보호,축중립/반전전중립,150ms통신만료,CRC/boot/request,중립→준비절차유지. 자동중앙복귀없음.
- 초음파53/49,41/39,37/35. pulseIn상한7ms,12ms이상간격채널순환,거리표시/기록만하며NO ECHO=999. 거리기반자동정지없음.
- 명령type2/13bytepayload/총19bytes: boot4/request4/action1/drive1/steer1/deadman1/PWM1,CRC16. 상태type150/41bytepayload/총47bytes: 구동PWM 및 조향 레지스터 진단 포함. 이전유형호스트접속금지.
- 부팅출력0/브레이크HIGH,EEPROM16..27에boot카운터. 실제구동전원/물리비상정지입력감지없음.

## 과거 type147 검증

Python9+공유Client49,C++7그룹통과. 가상60초연속동시축/PWM경계/CRC·stale거부/ADC·lease·반전검사. Mega/AVR1.8.8 flash9126/RAM327bytes. build_provenance.json에소스/hex해시.

실업로드검증,부팅ID29/type147/PWM60/중립→ready/출력0확인. `data/joystick_drive_pwm_20260916_021623/verification_snapshot.json`. 이전판에서차량움직임사용자확인,새판방향키실입력/연속주행/무선단절은아직별도확인대상이다.

## type148 조향 진단 추가

출력동작은type147과같다. 상태47bytes에OCR1A/TCCR1A/TCCR1B/DDR출력모드/DIR·BRK출력래치와핀논리수준을추가. 직접레지스터읽기이며PWM핀digitalRead호출없음. 진단값은모터전압/전류/기계동작측정이아니다. Python9검사·Mega컴파일flash9278/RAM327·실업로드/중립준비확인. 양방향출력중진단과물리조향은사용자시험대기. 앞type147검증은과거이력이다.


## 과거 type149 조향 펄스 비교

성공했다고 전달받은 Arduino/Python의 명목 간격에 맞춰 스틱을 잡고 있는 동안 90ms PWM60/60ms PWM0·BRK HIGH를 반복한다. 총 유지 시간 상한은 없고 중립·오류 즉시 정지/상대ADC100·끝단 보호를 유지한다. 원본 blocking 초음파 루프의 지연까지 재현한 것은 아니다.

Python9/C++8그룹 통과, Mega flash9454/RAM331bytes. 업로드 검증 및 부팅ID33/type149 중립 수신 확인. `data/joystick_pulse_steering_20260916_025149/`에서 사용자 물리 시험 대기. 직전 type148은 보드 PWM/DIR/BRK 논리 정상 전환에도 사용자 관측상 조향 무반응이었다. 원본은 `reference/vehicle/working_joystick_pair_2026-09-16/`에 보존한다.


## 현재 type150 연속 조향

사용자 보고로 펄스 움직임과 기계적 끝단 전 중간 정지를 확인했다. 펄스와 상대ADC100 제한을 제거해 스틱 유지 동안 PWM60 연속 출력하고 놓으면 정지한다. 실제 최대각 자동 정지/직진 보정은 미구현이며 사용자가 끝단에서 해제해야 한다. 기존 전기적레일(좌>=1022, 우<=1) 방향차단/안쪽복귀·통신/중립보호 유지. Python11/C++8그룹 통과, flash9230/RAM325. 실보드 수신 완료, 새판 물리 연속 조향은 대기다.


## 2026-09-18 D50 판 (조향 DIR D6 → D50, 미업로드)

06:32 D6 DIR 래치/핀 불일치 이후 전장팀이 조향 DIR 배선을 D50으로 옮겼다는 사용자 보고에 맞춰 `STEER_DIR=50`으로 바꾸고, frame42/43/44 진단의 DIR 비트를 DDRH/PORTH/PINH bit3(D6)에서 DDRB/PORTB/PINB bit3(D50)으로 옮겼다. 출력 동작·프레임 크기·호스트 비트 의미(bit0 DIR, bit1 BRK)는 그대로다. D50은 Mega 변형표에서 PB3(SPI MISO, 타이머 없음)이며 이 스케치는 SPI를 쓰지 않고 다른 핀 상수와 겹치지 않는다(`tests/check_pin_map.py`). D6은 설정하지 않아 입력 상태로 남는다.

- 빌드: flash 9222/RAM 325, hex `5dfe041f…`, `logs/development/20260918_075548_d50_joystick_manual_compile/` exit0. 이전 D6 hex `3af234e3…`와 소스는 `archive/d6_build_20260916/`, 차이는 `archive/d6_to_d50_20260918.diff`.
- 검사: 정적 핀맵 검사 PASS(`…_075603_d50_joystick_manual_pin_map_check/`), 코어 소스 무변경으로 네이티브 게이트 8그룹 1회 재실행 PASS(`…_075608_…`). Python 호스트 검사는 프로토콜 무변경으로 재실행하지 않았다.
- **현재 보드에 올라간 것은 전장팀 업로드본이며 이 hex가 아니다.** 업로드는 현재 보드 백업·전원 OFF·조건 파일 확인 후 `tools/upload_joystick_manual.py`로만 한다. 이전 D6 hex는 재업로드하지 않는다.
