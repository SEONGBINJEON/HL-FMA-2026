# 차량 없이 하는 실습

GitHub 선별 사본 루트에서 시작한다. Python3와, 조향 라이브러리 빌드에는 g++가 필요하다. 아래 명령은 센서 포트·모터·인증파일에 접근하지 않는다. 실제 준비/주행 메뉴는 이 실습의 대상이 아니다.

## 1. 경로 추종의 수학을 직접 확인

```bash
python3 - <<'PY'
import math
# 설명용 가상 차량·목표점이다. 실제 차량 보정값이 아니다.
wheelbase = 0.5
for gx, gy in [(3., 0.), (3., 0.5), (6., 0.5), (3., -0.5)]:
    distance = math.hypot(gx, gy)
    alpha = math.atan2(gy, gx)
    curvature = 2 * math.sin(alpha) / distance
    steering = math.degrees(math.atan(wheelbase * curvature))
    print((gx, gy), '곡률=', round(curvature, 4), '조향각=', round(steering, 3))
PY
```

직진 목표의 각도는0이고 좌우가 바뀌면 부호가 바뀐다. 같은 옆 편차0.5m라도 앞쪽 목표가 멀면 요구 곡률이 줄어든다. 실제 코드의 끝점·조향 한계·진입 조건을 생략한 수식 실습이며 차량 동작을 검증하지 않는다.

## 2. 본선 실패와 연결되는 잠금 효과 재현

```bash
python3 docs/analysis/competition_archive_2026-09-29/reproduce_latched_hold.py
```

성공하면 `PASS: ordinary red->green resumes; ...`가 나오고 같은 폴더에 `latch_reproduction.json`이 생긴다. 정상과 잠금 시나리오는 동일하게 빨강 뒤 초록을 넣는다. 차이는 정지 기준 이후 엔코더 변화가 복구 잠금으로 이어지느냐이다.

확인할 질문:

- 초록인데도 `drive:false`인 행의 `reason`은 무엇인가?
- 왜 신호 모델 정확도를 높이는 것만으로 이 시나리오가 해결되지 않는가?
- 반복해서 함수를 부르면 잠금은 풀리는가? 누가 그 상태를 해제해야 하는가?
- 이 검사가 실제 당시 차량이 움직였다는 증거인가? 아니다. 합성 입력의 잠금 효과만 재현한다.

## 3. 순수 모듈의 기존 검사를 읽고 실행

```bash
PYTHONPATH=tools python3 -m unittest discover -s tests -p 'test_path_steering_end_progress.py'
```

검사 소스부터 읽는다. 왜 종점 근처에 있다는 사실만으로 경로 완료를 선언하지 않는지, 진행 인덱스와 종점 위치가 어떻게 다른지 설명한다. 2026-09-29 원본에서 이 학습용 검사는 exit0을 확인했다. 저장소의 모든 검사 일괄 통과를 의미하지 않는다.

## 4. Python과 C++의 경계 확인

```bash
bash firmware/steer_target/host/build.sh
```

`libsteer_host.so`가 생기는 위치와 입력 C++ 소스를 읽는다. Python이 실제 보드와 통신하는 것과 호스트에서 C++ 계산을 호출하는 것을 구분한다. 이 명령은 Arduino 펌웨어 업로드가 아니다.

## 학습을 개발로 이어갈 때

새 작업 사본/브랜치에서 합성 재현의 실패 원인을 먼저 설명하고, 수정의 원하는 전이 조건을 적는다. 정상 신호 재개, 실제 차량 이동, 위치 단절, 보드 재부팅을 하나의 “정지 해제”로 뭉뚱그리지 않는다. 실제 제어 수정과 차량 검증은 별도 작업으로 시작한다. 이번 아카이브 작업에서는 제어 코드를 수정하지 않았다.
