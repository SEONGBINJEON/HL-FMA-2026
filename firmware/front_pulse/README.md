# 앞 구동 모터 단발 공중 시험 전용

**실제 모터 출력이 있는 별도 펌웨어다.** 일반 수동 주행용이 아니며, 바퀴를 지면에서 띄운 안정된 받침과 전장 담당자의 전원 차단 대기 조건을 실제 확인한 뒤에만 사용한다. 구현 담당 작업에서는 보드 접근·업로드·구동을 하지 않았다. 현재 보드 상태와 사용자 조건은 프로젝트 `docs/STATUS.md`를 따른다.

## 고정 출력 범위

- 앞 구동: D12 PWM30/255, D8 LOW(원본의 전진 설정이며 실제 회전 방향 미측정), D22 BRK LOW. 한 부팅에서 단 한 번, 루프 시간 기준100ms까지만 허용한다.
- 뒤 구동: D44 PWM0 / D24 BRK HIGH만 적용한다.
- 조향: D11 PWM0 / D7 BRK HIGH만 적용한다. 조향 ADC A15는 원시값만 읽는다.
- 엔코더 D2/D3 ISR과 EEPROM0~11 검증 부팅 카운터는 기존 형식을 유지한다. 바퀴 거리 환산값은 없다.

명령은 새 중립 확인→별도ARM→CONTROL drive+1/steer0/deadman1 순서가 필요하다. 기존 검증 BENCH 코어는 수정하지 않고 그대로 복사했다. 코어가 수락한 논리 상태를 별도 `PulseGate`가 확인한 후 사용 래치를 먼저 소비하고 앞 출력만 켠다.100ms 만료나 논리 중립·정지·오류·통신 만료가 발생하면 끈다. 반복 명령이나 같은 부팅의 재ARM으로 다시 켜지지 않는다.

물리 E-stop/전원 상태 입력은 없다. 현재 버튼 방향과 하중 지지·차단 담당자의 준비를 코드가 확인하지 못한다. 루프가 실행되지 않는 CPU 정지나 부트로더/리셋 구간은100ms 소프트웨어 타이머로 보장하지 않으며 AVR watchdog도 사용하지 않는다. 사용 래치는 MCU 재부팅 때 초기화되므로 USB 재연결/재접속 뒤에는 새로운 시험으로 취급해야 한다.

## 상태 프레임

상태 TYPE은131, LENGTH26이다. 나머지 little-endian 배치/CRC는 기존 수동 프로토콜과 같다. 기존 BENCH GUI가 이 타입을 정상 BENCH 보드로 취급하지 않도록 분리한다.

- 정상 초기 flags34 = BOOT_ID_VALID32 | DRIVE_ENABLED2. BENCH/INTERLOCK 비트는 없다.
- 첫 펄스부터 flags98 = 위 값 | PULSE_USED64. 사용 비트는 펄스 종료 뒤에도 유지한다.
- `actual_drive=+1`은 앞 PWM 출력 명령이 현재 활성이라는 보고이며 물리 전압/회전 측정값은 아니다. 비활성은0, `actual_steer`는 항상0이다.
- state/reason/requested 방향은 코어 논리값을 보고한다. 논리ARMED/requested+1이어도 펄스가 끝났으면 actual0/PULSE_USED1이 된다. 사용 완료 후 PC는 DISARM을 보내 정리한다.
- EEPROM 검증 실패 등 유효 부팅 식별이 없으면 BOOT_ID_VALID/DRIVE_ENABLED가0이며 출력되지 않는다.

## 검증과 근거

`tests/run_tests.sh`는 실제 복사 코어와 `PulseGate`를 함께 실행한다. 최초 준비, 잘못된 방향/조향/미인증 입력 거부,100ms 제한, 반복/재ARM 불가, 중립·정지·오류·만료, millis wrap을 검사한다. 실제 모터 회전이나 제동 검증을 대신하지 않는다.

코어 복사 해시는 `core_copy_provenance.json`, 컴파일·검사 결과와 HEX 해시는 `build_provenance.json`에 기록한다. 기존 `firmware/safe_manual/`과 원본 펌웨어는 변경하지 않았다.
