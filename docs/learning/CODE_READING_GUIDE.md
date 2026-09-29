# 코드 읽기 지도

저장소 루트 기준 경로다. 먼저 작은 순수 계산/상태 함수를 읽고 마지막에 실제 I/O를 연결하는 실행기를 읽는다. 아래 진입점은 읽기 안내이며 실행 권고가 아니다.

| 순서 | 파일·함수 | 확인할 질문 |
|---|---|---|
| 1 | `tools/qualifier_route.py`, `Point`, 경로 변환·heading 관련 함수 | 경로의 좌표와 단위는 무엇이고 진행 방향은 어떻게 정하는가? |
| 2 | `tools/path_steering.py`: `PoseInput`, `PurePursuitConfig`, `compute_target`, `PathFollower` | 현재 위치에서 목표점·곡률·각도를 어떻게 계산하며 어떤 입력을 거부하는가? |
| 3 | `tools/qualifier_follow.py`: `select_entry`, `bootstrap_heading`, `adaptive_lookahead` | 정지 출발·경로 진입·연속 추종을 어떻게 연결하는가? |
| 4 | `tools/steering_calibration.py`, `tools/steer_target_host.py`, `tools/main_event_serial.py`, `firmware/steer_target/` | 단위·부호·보정·요청/응답·보드 상태가 어떻게 연결되는가? |
| 5 | `tools/main_event_core.py`: `Gate`, `Stationary`, `SignalVotes`, `Mission.step` | 분기 순서와 다음 호출에도 남는 상태가 결과를 어떻게 바꾸는가? |
| 6 | `tools/main_event_signal.py`, `hyper_signal_offline.py`, `main_event_lidar_scope.py` | 인지 결과의 출처·장착·시각·대상 미션을 어디서 검사하는가? |
| 7 | `tools/main_event_degraded.py`, `GapContinuity` | 일시 단절과 복구 불가 잠금을 구분하는 기준은 무엇인가? |
| 8 | `tools/main_event_runtime.py`: `Inputs`, `Workers`, `run` | 위 모듈들이 실제 수신·구동 주기에서 어떤 순서로 호출되는가? |
| 9 | `tools/main_event.py`, `main_event_menu.py`, `main_event_start_left.py` | CLI 입력이 어떤 세션·프로세스·제어 요청으로 바뀌는가? |
| 10 | `tools/main_event_release.py`, `tools/dev_run_log.py` | 당시 소스와 실행 근거를 어떻게 식별하는가? |

## 입력을 따라갈 때 적을 표

| 자료 | 최소한 확인할 항목 | 혼동하기 쉬운 점 |
|---|---|---|
| 위치 | 좌표계, 단위, 수신/관측 시각, FIX 품질 | NTRIP 연결 성공을 신선한 위치로 착각 |
| heading | 기준축, rad/degree, 앞/뒤 진행, 생성 근거 | 정지 GPS 한 점을 방향으로 착각 |
| 조향 | 등가 조향각, cdeg, ADC, DIR 부호 | 알고리즘 각도를 모터 출력으로 착각 |
| 신호 | target/epoch, 촬영·판정 시각, frame_id, confidence | 같은 프레임 반복을 새 투표로 착각 |
| 보드 | boot, request, received_at, encoder, state/fault | ACK를 실제 무이동으로 착각 |
| 미션 | index, completed, holding, recovery_required/cause | 다음 루프에서 잠금이 자동 해제된다고 가정 |

시간 비교는 특히 중요하다. `time.time()` 계열의 벽시계와 `time.monotonic()` 계열의 경과 시계를 섞으면 정상 자료도 오래됐다고 판단하거나 미래 자료처럼 보일 수 있다. 각 필드의 생산자와 소비자를 함께 읽고 변환 지점을 확인한다.

## 첫 신호등 실패를 코드에서 따라가는 방법

1. [분석 기록](../retrospective/2026-09-29/LIGHT1_FAILURE.md)의 시간표를 읽는다.
2. `main_event_runtime.py`에서 `hold_encoder`, `hold_changed`, `held_vehicle_moved`를 찾는다.
3. `Mission.step()`에서 `recovery_required` 판단이 신호 투표보다 먼저 실행되는지 확인한다.
4. `GapContinuity.update()`가 어떤 상태를 회복시키고 어떤 원인은 유지하는지 읽는다.
5. [실습](LABS.md)의 합성 재현을 실행해 정상과 잠금 상태의 차이를 비교한다.

이 경로는 잠금 효과를 설명한다. 당시 실제 기준 엔코더/위치 값이 충분히 기록되지 않아 최초 트리거를 완전히 재생한 것은 아니다. 이 구분을 학습 노트에도 남긴다.

## 검색부터 시작하기

```bash
rg -n 'def compute_target|class PathFollower|def adaptive_lookahead' tools
rg -n 'hold_encoder|hold_changed|held_vehicle_moved|pose_recovery_required' tools
rg -n 'class SignalVotes|def step|signal_pass|recovery_required' tools/main_event_core.py
```

검색 결과를 읽은 뒤 호출자까지 따라간다. 함수 이름만 알고 끝내지 말고, 실제 실행기에서 그 함수를 쓰는지 확인한다. 초기 실험용·수동 도구·오프라인 분석 코드가 모두 같은 최종 경로에서 실행되는 것은 아니다.
