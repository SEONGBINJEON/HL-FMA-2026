# F8 자율주행 (HL FMA 1/5 대회)

HL Future Mobility Award 1/5 스케일 자율주행 대회 출전 프로젝트.
F8 플랫폼 기반, ROS 2 + Arduino Mega 구조.

## 주행 철학

**GPS 웨이포인트 우선 + 카메라 보조.**
- 메인 주행: RTK GPS 웨이포인트를 Pure Pursuit로 추종
- 신호등·주차: 카메라(YOLO)로 인지
- 차선 안전벨트: 카메라 차선 인식으로 GPS 튐 보정 (우선순위 낮음, 여유 시)
- 근접 감지: 초음파 (돌발정지·주차)

작년 출전팀(검증된 완주 방식) + HYPER 팀(ROS 2 구조·카메라 인식)을 결합.

## 시스템 구조

```
[센서] → [인지] + [측위] → [판단] → [제어(아두이노)] → [구동]
 카메라    신호등    RTK+IMU   Pure     속도·조향 PID    모터
 초음파    (주차)    엔코더    Pursuit                   서보
```

- 연산(노트북, ROS 2): 인지 · 측위 · 판단
- 저수준(Arduino Mega): 속도 PID · 조향 PID · 엔코더
- 전원(24V): 주동력 / DC-DC / 센서 전용 필터 라인

## 폴더 구조

| 폴더 | 담당 | 내용 |
|---|---|---|
| `perception/` | 하이레벨 | 카메라 인지 (신호등·주차·차선) |
| `localization/` | 하이레벨 | RTK+IMU+엔코더 융합, 좌표 변환 |
| `planning/` | 하이레벨 | Pure Pursuit, 미션 상태머신 |
| `arduino/` | 로우레벨 | 메가 펌웨어 (속도·조향 PID, 시리얼) |
| `common/` | 공통 | 메시지 정의, 공용 유틸 |
| `docs/` | 총괄 | 인터페이스 규약, 설계 문서 |

## 하드웨어

- 메인컴퓨터: i5-9300HF, GTX 1650 4GB
- MCU: Arduino Mega 2560
- 모터드라이버: HENES aMAP PowerPack Driver
- 구동모터: F8 순정 12V 70W DC × 2
- 카메라: RealSense D455
- IMU: RB-SDA-v1
- GPS: u-blox ZED-F9P + ANN-MB-00 (NTRIP RTK)
- 엔코더: F8 순정
- 초음파: 전·후방
- 배터리: 24V (리튬 권장)

## 규칙

**개발 시작 전 반드시 [docs/interface.md](docs/interface.md) 를 읽을 것.**
토픽 이름·타입·단위는 이 문서가 유일한 기준이며, 변경은 총괄과 합의 후에만.

## 시작하기

각자 담당 폴더의 README를 참고. 상대 노드가 없어도
`common/`의 가짜 데이터 발행 노드로 단독 테스트 가능.
