# HL FMA 2026 — 대회 준비와 자율주행 구현 기록

1/5 스케일 차량의 센서 연결, RTK GPS 경로 추종, 조향 제어, 신호 인식, 미션 통합과 현장 문제 해결을 기록한 프로젝트다. **대회는 종료됐으며, 예선 통과 후 본선에서는 경사로를 통과하고 첫 신호등에서 재출발하지 못했다.** 전체 완주 성공 프로젝트로 소개하지 않는다.

[전체 기록](docs/retrospective/2026-09-29/README.md) · [개발 과정](docs/retrospective/2026-09-29/TIMELINE.md) · [구성](docs/retrospective/2026-09-29/ARCHITECTURE.md) · [첫 신호등 실패 분석](docs/retrospective/2026-09-29/LIGHT1_FAILURE.md)

## 공부와 새 노트북 이전

[학습 순서](docs/learning/README.md)에서 시작해 [시스템 해설](docs/learning/SYSTEM_WALKTHROUGH.md) → [코드 읽기 지도](docs/learning/CODE_READING_GUIDE.md) → [차량 없는 실습](docs/learning/LABS.md) 순서로 읽는다. [개발 방식·면접 준비](docs/learning/DEVELOPMENT_AND_INTERVIEW.md)는 실제 성과와 한계를 설명하는 기준을 담는다.

[노트북 이전 안내](docs/migration/README.md)에 GitHub/USB의 역할, 분리된 Python 환경 기록, 복원·검증 절차와 별도 인증 이전 항목을 정리했다. USB의 원시 자료가 있어야 당시 자료를 이용한 전체 분석을 이어갈 수 있다.

## 구현 범위

- RTK GPS 기록과 경로 추종, 전진으로 초기 방향 취득, 적응형 lookahead와 경로 진입.
- 실측 조향 보정과 Arduino 목표각·구동 제어, 경사로 정지·버티기.
- HYPER 경로·미션·기존 신호 모델 재사용, OAK 입력, 뒤집힌 라이다 장착 변환.
- 미션 상태 연결, S 후보 재계획, 일부 선택 주차 경로, 종점 분기, 제한된 수동 복구.
- L 자동 준비·명시적 키2 출발, 시리얼 처리 개선, 보조 센서 장애 분리, 실행 기록·동결 해시.

## 실제 결과와 한계

| 구분 | 확인한 내용 |
|---|---|
| 직접 기록 경로 | 약111.8m 연속 추종. 기록 경로/GNSS 기준 최대 횡오차0.1283m |
| 예선 | 통과 사용자 보고 |
| 본선1차 | 초기 통신 준비 잠금으로 출발 실패 |
| 마지막 본선 주행 | 경사로 미션 완료 후 첫 신호 정지, 약7초 뒤 위치 복구 잠금으로 전환 |
| 신호 정체 원인 | 정지 중 관측 변화에 의한 잠금이 유력. 실제 초록 영상 존재, 역사적 모델 판정 일부 미기록 |
| 주차·후반부 | 일부 모의 수행. 실차 주차·본선 전체 완주 증거 없음 |
| 최종 코드 식별 | 동결143442/168파일, 9/29 해시 대조 일치 |

[검증 근거와 한계](docs/retrospective/2026-09-29/EVIDENCE_INDEX.md)를 함께 읽어야 한다. 원격 초기 설계의 ROS2/RealSense/센서융합 계획은 실제 최종 실행 구조와 구분했다.

## 오프라인 확인과 재사용 범위

[장치 없이 확인하는 방법](docs/REPRODUCIBILITY.md) · [외부 자료·출처](docs/THIRD_PARTY.md)

펌웨어·운영 도구의 과거 README는 작성 당시 상태를 담고 있다. 최종 본선 상태와 검증 범위는 위 회고 문서를 기준으로 읽는다.

## 폴더

- `tools/`, `scripts/`: 운영·제어·센서·분석 도구.
- `tests/`, `firmware/`: 검사와 보드 제어 소스.
- `config/`: 차량별 설정. 인증정보는 배포 대상에서 제외.
- `docs/retrospective/2026-09-29/`: 준비 과정·결과·원인 분석.
- `docs/analysis/competition_archive_2026-09-29/`: 저장 로그 추출과 분리 재현 근거.

원본 작업 폴더에는 `data/`, `logs/`, `backups/`, `reference/` 등도 보존돼 있다. GitHub 사본은 원시 자료·실행환경 전체 사본이 아니며, 일부 실행/검사는 로컬 보정·HYPER 자료·빌드 라이브러리·장치가 필요하다. clone만으로 주행 가능한 배포판이 아니다.

HYPER의 재사용 부분은 출처와 자체 작업을 구분한다. [GitHub 정리 범위와 외부 자료 처리](docs/retrospective/2026-09-29/GITHUB_MIGRATION_PLAN.md)에 보존·제외 기준을 기록했다.
