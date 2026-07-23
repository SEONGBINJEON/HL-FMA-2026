# localization — 측위 (하이레벨 담당)

RTK GPS + IMU + 엔코더를 융합해 지도상 현재 위치·헤딩을 계산하는 노드.
**주행 정확도의 핵심.** 여기가 튼튼해야 웨이포인트 주행이 된다.

## 만들 것

### 1. NTRIP RTK GPS (최우선)
- u-blox ZED-F9P + ublox ROS 2 드라이버
- NTRIP 클라이언트로 국토지리정보원 보정(RTCM) 수신 → F9P에 주입
- **대회 코스 위치에 맞는 NMEA(GGA) 좌표를 서버에 보내야 함** (작년 팀 교훈: 지역 안 맞으면 보정 안 됨)
- 확인 지표: NavSatFix status = 2 (RTK Fix), 수평오차 cm급
- 출력: `/sensor/gps/fix` (`sensor_msgs/NavSatFix`)

### 2. IMU 헤딩 (최우선)
- RB-SDA-v1 → `/sensor/imu` (`sensor_msgs/Imu`)
- **IMU를 모터·배선에서 최대한 멀리 배치** (자기 간섭 → 헤딩 오차)

### 3. EKF 융합
- `robot_localization` 패키지 (dual EKF + navsat_transform)
- GPS(위치) + IMU(자세) + 엔코더 오도메트리(`/sensor/odom`) 융합
- GPS가 순간 튀어도 IMU·엔코더가 받쳐주도록 (HYPER 방식 참고)
- 출력: `/localization/pose` (`geometry_msgs/Pose2D`) = x(m), y(m), theta(rad)

## 지켜야 할 것
- theta는 **라디안**, 좌표 원점은 출발선, 반시계 +. (`docs/interface.md`)
- 엔코더 오도메트리는 아두이노 담당이 `/sensor/odom`으로 발행 → 협의 필요.

## 우선순위
RTK Fix 성공 → IMU 헤딩 → EKF 융합. RTK부터 확실히.
