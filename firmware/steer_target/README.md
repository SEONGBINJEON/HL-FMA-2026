# 목표 조향 후보 코어 (steer_target) — 미업로드 후보

작성: 2026-09-17 02:35 KST 안팎, Claude Code C1. C1.1(02:54 KST): 감소표 반올림 수정, ADC 필드 상한, 경로 A 반전 설명 정정. **현재 운용판은 `firmware/joystick_manual` type150 수동 제어이며 이 폴더는 실행/업로드되지 않은 후보 로직이다.** Arduino 헤더·직렬·핀 접근이 없고 g++ 로 호스트에서 검사한다. 실차 조향·자동주행 검증이 아니다.

## 파일

- `steer_target_core.h/.cpp` — 하드웨어 무관 목표 조향 로직. 입력: 검증된 보정(`Config`), 준비 상태(`setReady`), 정지(`stop`), 목표 명령(`setTarget`: sequence·각도·호스트시각), 조향 측정(`setMeasurement`: ADC·로컬시각). 출력 `Output`: 방향 -1/0/+1(수동 코어 steer 규약), 이유, 잠금 여부, 클램프 여부, 목표 ADC.
- `tests/steer_target_test.cpp`, `tests/run.sh` — 합성 표로 11 시나리오 그룹. 마지막 통과 기록 `logs/development/20260917_025239_c1_1_steer_target_cpp_compile/`(12그룹).
- 보정 입력은 `config/steering_calibration.json`(전부 null·미측정)과 `tools/steering_calibration.py`(검증·C++ 스니펫 생성)로 관리한다. 합성값은 `tests/fixtures/steering_calibration_synthetic.json`에만 있고 운용 파일로 자동 복사되지 않는다.

## 정책 (검사로 고정)

| 조건 | 출력 | 이유 코드 | 잠금 |
|---|---|---|---|
| `Config` 무효(기본값·표 3점 미만·직진점 없음·ADC/각도 비단조·레일값·한계가 표 밖/직진 한쪽·계수 0·정책 미설정) | 0 | `NO_CALIBRATION` | 해당 없음(영구) |
| 준비 전 | 0, 목표 명령 무시 | `NOT_READY` | 준비 하강 에지에서 잠금 |
| 준비됐지만 목표 없음(부팅 직후) | 0, 중앙 이동 없음 | `NO_TARGET` | 아니오 |
| 새 sequence 이후 `command_stale_ms` 경과 | 0 | `COMMAND_STALE` | 예 |
| 측정 없음/`sensor_stale_ms` 경과 | 0 | `SENSOR_STALE` | 예 |
| ADC ≤1, ≥1022, 또는 사용한계±여유 밖 | 0 | `SENSOR_INVALID` | 예 |
| 목표가 표/사용한계 밖 & 정책 REJECT | 0 | `TARGET_REJECTED` | 예 |
| 목표가 표/사용한계 밖 & 정책 CLAMP | 한계까지 이동, `target_clamped` | `MOVING` | 아니오 |
| \|목표−ADC\| ≤ stop_band (이동 중) / < restart_band (정지 중) | 0 | `AT_TARGET` | 아니오 |
| 이미 사용한계에서 바깥으로 요청 | 0 | `AT_LIMIT` | 아니오 |
| 방향 반전 요구 | 먼저 0을 ≥1회·`reversal_neutral_ms` 유지 | `REVERSAL_NEUTRAL` | 아니오 |
| 명시적 `stop()` | 0, 목표 삭제 | `STOPPED` | 예 |

- **잠금 해제 조건은 하나다:** 준비 상태에서 **이전보다 큰 sequence의 새 목표**가 수락될 때. 같은 sequence 재전송은 신선도를 갱신하지도, 재출력을 일으키지도 않는다(재송신 스레드가 상위 계산 정지를 숨기지 못하게 함).
- `host_time_ms`는 저장·진단용이며 보드 시각과 비교하지 않는다. 신선도는 새 sequence를 수락한 로컬 시각으로만 판단한다.
- 각도↔ADC는 측정점의 구간별 선형 보간이다. 좌우 대칭·선형을 가정하지 않으며 각 측정점이 그대로 표에 들어간다. 부호: 각도 +는 왼쪽(FLU yaw), `steer_sign_for_adc_increase`는 “ADC를 올리는 수동 코어 steer 값”으로 실측해야 한다.

## 현재 수동 코어와의 연결 (이번 C1에서는 켜지 않음)

- **경로 A — 호스트 측(펌웨어 무변경, 가장 빠른 실장치 검증):** 호스트가 type150 상태의 `steering_adc`(50ms)와 수신 시각을 `setMeasurement`로, 상위 경로/조작에서 오는 목표를 `setTarget`으로 넣고, `Output.steer`를 기존 `CONTROL(drive 0, steer, deadman 1)` 명령으로 보낸다. 준비=수동 코어 `ARMED`+lease 유효, 정지=기존 `DISARM`. **반전 규칙(C1.1 정정):** 이 코어가 0을 먼저 내보내고 `reversal_neutral_ms`를 기다리는 것만으로는 수동 코어와의 충돌이 보장되지 않는다. 명령 유실·지연·보드 재시작이 있으면 보드는 중립을 수락하지 않은 상태일 수 있다. 호스트 어댑터는 **보드가 그 중립 명령을 실제로 수락했다는 확인**(반전 전 최신 상태 프레임에서 같은 bootID·해당 request 처리 후 `requested_steer==0`, `state==ARMED`, `reason` READY/NEUTRAL)을 얻은 뒤에만 반대 부호를 보내야 하며, 확인이 없으면 0을 유지하거나 정지로 처리한다. 유실·지연·재시작 시나리오의 모의검사는 어댑터 구현 때 필요하다. 호스트 시각과 보드 `millis`는 각각 자기 시계로만 쓴다.
- **경로 B — 펌웨어 측(목표):** 새 명령 type(예: 3)으로 sequence+각도(cdeg)를 받고, `StepGate` 대신 이 코어의 `steer`를 `applyOutputs`에 준다. 상태 type도 새 번호로 바꿔 `joystick_protocol.py`가 다른 펌웨어를 식별하게 해야 한다. **충돌 없는 type 식별/검증 절차는 다음 통합 과제**로 남긴다.
- 어느 경로든 부팅ID·CRC·request lease·150ms 명령 만료는 기존 수동 코어가 그대로 담당하고, 이 코어는 그 위에서 “목표에 대한” 신선도·한계·반전을 추가로 판단한다. 이중 검사이며 하나가 다른 하나를 대체하지 않는다.

## 미완료·주의

- 실측 보정 없음. `config/steering_calibration.json`이 null인 동안 `steering_calibration.py check`는 NOT READY, 코어는 항상 0이다.
- 조향 PWM은 수동판 고정값 60을 전제로 방향만 낸다. 속도 비례 제어·PWM 조정은 실측 응답(A1) 뒤 판단.
- AVR 컴파일(arduino-cli)·flash/RAM 확인·실보드 업로드는 하지 않았다. 코어는 STL/float 없이 int32 정수 연산만 쓴다.

## C2 호스트 연결 (2026-09-17 09:33 KST, 오프라인)

경로 A를 구현했다. `host/steer_host_cabi.cpp`는 이 코어와 수동 type150 코어(`joystick_manual/control_core.cpp`+`step_gate.h`)를 C ABI로 감싸 `host/build/libsteer_host.so`로 빌드한다(`bash firmware/steer_target/host/build.sh`, 해시는 `build/build_info.json`). Python `tools/steer_target_host.py`가 ctypes로 같은 C++ 코어를 실행하며(복제 없음, 매 tick 컴파일 없음) `HostSteerAdapter`는 수동 `Session`을 상속해 준비 절차(명시 요청→보드 중립 확인→ARM 확인)와 프레임 순서·재시작·중복·만료 검사를 그대로 쓰고, 키 대신 코어의 방향 결정을 CONTROL(drive 0, PWM 0)에 넣는다. **반전은 실제로 보낸 중립 이후 같은 boot의 더 큰 request_id 상태가 ARMED·requested/actual 0·READY/NEUTRAL을 보인 뒤 `reversal_neutral_ms`가 지나야** 허용된다. 정지/오류/재시작은 잠금이며 새 `request_prepare()`와 새 목표 sequence가 필요하다. `tools/steer_target_sim.py`는 가짜 시계(보드 시각 오프셋)·모의 전송·보드 모형으로 11개 시나리오를 실행한다. 실제 직렬/차량 검증은 아니다. 상세는 `docs/handoff/CLAUDE_CODE_RESULT_C2.md`.

C2.1(10:05 KST): 목표 수락 후 코어 잠금은 어댑터 잠금·DISARM으로 전달(재준비 직후 잔여 잠금은 정상), 변환 전 입력 범위 검증(잘못된 목표=잠금), 무보정 준비 거절, 시나리오 15개. `docs/handoff/CLAUDE_CODE_RESULT_C2_1.md`.
