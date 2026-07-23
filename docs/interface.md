# 인터페이스 규약 v1.0

**이 문서가 노드 간 통신의 유일한 기준이다.**
여기 적힌 토픽 이름·타입·단위를 지키면, 서로 코드를 안 봐도 노드가 붙는다.
변경이 필요하면 반드시 총괄에게 먼저 알리고 이 문서를 함께 수정한다.

## 토픽 목록

| 토픽 이름 | 방향 | 타입 | 내용 · 단위 | 주기 |
|---|---|---|---|---|
| `/sensor/gps/fix` | GPS→측위 | `sensor_msgs/NavSatFix` | RTK 보정 위경도 | ~5Hz |
| `/sensor/imu` | IMU→측위 | `sensor_msgs/Imu` | 자세(쿼터니언)·각속도·가속도 | ~50Hz |
| `/sensor/odom` | 아두이노→측위 | `nav_msgs/Odometry` | 엔코더 오도메트리 (m, m/s) | ~20Hz |
| `/sensor/ultrasonic` | 아두이노→판단 | `sensor_msgs/Range` | 전·후방 근접 거리 (m) | ~10Hz |
| `/perception/sign` | 인지→판단 | `std_msgs/String` | `red`/`yellow`/`green`/`left`/`none` | ~10Hz |
| `/perception/lane` | 인지→판단 | `std_msgs/Float32MultiArray` | [오프셋(m), 조향각(deg), 유효성(0/1)] · 안전벨트용 | ~15Hz |
| `/localization/pose` | 측위→판단 | `geometry_msgs/Pose2D` | x(m), y(m), theta(rad) 지도좌표 | ~20Hz |
| `/cmd/drive` | 판단→아두이노 | `geometry_msgs/Twist` | linear.x=목표속도(m/s), angular.z=조향각(deg) | ~20Hz |
| `/sensor/scan` | (예약·비활성) | `sensor_msgs/LaserScan` | 라이다 추가 시 활성화 | — |

## 변경 불가 3원칙

이걸 어기면 차가 반대로 돌거나 엉뚱한 곳에서 멈춘다. 절대 통일.

1. **각도 단위**: 조향각은 도(degree). 헤딩(theta)은 라디안(radian).
2. **좌표계**: 원점 = 출발선. x축 = 차량 초기 전방. 반시계 방향이 +theta.
3. **조향 부호**: 좌회전 = +, 우회전 = −.

## 속도·조향 범위 (합의 필요 — 실측 후 채움)

| 항목 | 최소 | 최대 | 비고 |
|---|---|---|---|
| 목표 속도 (m/s) | 0 | (실측) | 후진은 음수 |
| 조향각 (deg) | (실측 −) | (실측 +) | 물리적 최대 조향각 |

## `/cmd/drive` 상세

- `linear.x`: 목표 전진 속도 (m/s). 음수 = 후진.
- `angular.z`: 목표 조향각 (deg). +좌 / −우.
- 나머지 필드는 0으로 채운다.
- 판단 노드는 20Hz로 계속 발행. 명령이 끊기면 아두이노가 안전정지.

## 안전 규약

- 아두이노는 `/cmd/drive`가 **0.5초 이상 안 오면 속도·조향 0으로 정지** (타임아웃 세이프티).
- 초음파가 임계거리 이내 감지 시 판단 노드가 즉시 정지 명령.
