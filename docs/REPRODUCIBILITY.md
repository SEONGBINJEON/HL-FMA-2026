# 장치 없이 확인하는 방법과 필요한 자료

이 저장소는 최종 차량 운용 환경 전체가 아닌 대회 코드·문서 아카이브다. 아래 두 명령은 하드웨어 포트를 열거나 구동 명령을 보내지 않는 오프라인 확인이다. 저장소 루트에서 실행한다.

## 첫 신호 정체의 잠금 효과 재현

```bash
python3 docs/analysis/competition_archive_2026-09-29/reproduce_latched_hold.py
```

Python 표준 라이브러리와 포함된 main_event_core만 사용한다. 정상 빨강→초록 조건은 재개하고, 정지 기준 이후 엔코더 변화로 복구 잠금이 설정된 경우에는 초록 입력에도 재개하지 않는 차이를 확인한다. `latch_reproduction.json`을 생성한다. 실제 경기 전체 재생이나 차량 시험은 아니다.

## 호스트 조향 라이브러리 빌드

```bash
bash firmware/steer_target/host/build.sh
```

Python3와C++11을 지원하는g++가 필요하다. 포함된 소스에서 `firmware/steer_target/host/build/libsteer_host.so`와 빌드 근거를 만든다. Arduino 업로드나 모터 구동을 하지 않는다. 바이너리는 Git에 포함하지 않는다.

## 전체 운용에 필요한 별도 자료

| 의존성 | 현재 처리 |
|---|---|
| 차량별 조향·치수 보정과 data 파일 | 실제 차량에서 사용한 원본은 로컬 보존. 임의 값으로 대체하지 않음 |
| HYPER 모델·경로·미션 파일 | 원 저장소/제공자로부터 권한과 버전을 확인해 별도 취득 |
| OAK 라이브 환경 | requirements-oak.txt는 당시 기록. YOLO 환경과 분리돼 있었음 |
| RTK/NTRIP | requirements-rtk.txt 및 별도 비공개 인증 설정 필요 |
| RealSense 환경 | requirements-camera.txt는 초기 자료 수집 경로. 최종 OAK 실행과 구분 |
| YOLO/Ultralytics | 기존 별도 .venv-yolo 환경이 필요했으며 이 아카이브에 완전한 잠금 파일은 없음 |
| 원시 주행·카메라·GNSS | analysis/audit.py의 실제 기록 재계산에 필요, 이 저장소에 포함하지 않음 |

여러 requirements를 무조건 하나의 환경에 합치는 설치법은 검증하지 않았다. 전체 테스트 중 실제 자료·외부 경로·빌드 산출물을 참조하는 항목은 그대로 실행하면 누락 오류가 날 수 있다. 전체 회귀가 이 사본만으로 통과한다고 주장하지 않는다.

`main_event_release.py verify`는 대회 당시 원본 환경을 검증하는 명령이다. 이 정리 사본의 검증 기준은 루트 PACKAGE_MANIFEST.json이며, 원본168파일 전체가 복제됐다는 뜻이 아니다.
