# planning — 판단 (하이레벨 담당)

위치·인지 정보를 받아 "어떻게 주행할지" 결정하는 두뇌 노드.

## 만들 것

### 1. Pure Pursuit 웨이포인트 추종 (최우선)
- 입력: `/localization/pose` (현재 위치·헤딩)
- 미리 저장한 코스 웨이포인트 목록을 따라감
- 전방주시거리(look-ahead) 밖의 점을 조준해 조향각 계산
- **작년 팀 교훈: look-ahead를 구간별로 동적 조절** (급커브는 짧게). 헤딩 PID는 지그재그 심해서 비추.
- 출력: `/cmd/drive` (`geometry_msgs/Twist`) — linear.x=속도, angular.z=조향각(deg)

### 2. 미션 상태머신 (2순위)
- 웨이포인트 구간(인덱스)별로 상태 전환:
  - 경사로 정지, 신호 정지(`/perception/sign`), T자·평행 주차, 돌발정지(`/sensor/ultrasonic`)
- **작년 팀 교훈: 신호 인식 실패 대비 타임아웃 강제주행** (30초 지나면 그냥 감)
- 후진 주차 트릭: 목표 웨이포인트 덮어쓰기 + heading 180도 반전 참고

### 3. 차선 안전벨트 (3순위, 여유 시)
- `/perception/lane` 받아서, GPS 경로가 차선을 크게 벗어나면 보정

## 지켜야 할 것
- 조향각 **도(degree)**, 좌회전 +. (`docs/interface.md`)
- 명령은 20Hz로 끊김없이 발행.

## 우선순위
Pure Pursuit 기본 주행 → 미션 상태 → 안전벨트.
