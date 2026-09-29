# 다른 노트북에서 공부하고 개발을 이어가기

2026-09-29 사용자 보고: 현재 노트북은 약1주 뒤 반납하며 개인 USB가 있다. 이번 이전의 목표는 코드만 남기는 것이 아니라 **코드·판단 근거·원본 자료·환경 기록을 함께 보존하는 것**이다.

## 두 자료의 역할

| 위치 | 담는 내용 | 용도 |
|---|---|---|
| 비공개 GitHub | 선별 코드, 회고·실패 분석, 학습 문서, 설치 환경의 패키지 이름/버전 | 코드 탐색·학습·새 변경의 이력 관리 |
| 개인 USB | 원본 작업 폴더의 코드·문서·주행 기록·보정·모델·경로·펌웨어 백업·기존 공유물, 백업 명세와 검증 결과 | 실제 입력을 이용한 재분석과 개발 의존성 복원 |

USB 백업에서는 인증파일, AI 대화 원문, 가상환경, Git/도구 내부 상태, 캐시와 해당 민감 항목을 포함한 기존 압축파일을 제외한다. 제외 목록은 USB의 `EXCLUSIONS.json`에 남긴다. 기존 자료를 삭제하거나 USB를 포맷하지 않는다. 패키지는 개인 보관용이며 외부 제공 자료의 재배포 허가를 뜻하지 않는다.

## USB를 받았을 때

새 폴더 `HL_FMA_BACKUP_날짜_시각` 안에서 `BACKUP_STATUS.json`의 상태가 `verified`인지 확인한다. `CHECKSUMS.sha256`은 분할 압축파일 검증용이다. FAT32의 파일 크기 제한 때문에 압축 스트림을2GiB 단위로 나눈다. 분할 파일은 모두 필요하며 개별적으로 압축을 풀지 않는다.

Linux/WSL에서 백업 폴더를 현재 디렉터리로 둔 뒤:

```bash
sha256sum -c CHECKSUMS.sha256
mkdir -p "$HOME/HL_FMA_restore_20260929"
set -o pipefail
cat workspace.tar.gz.part-* | tar -xzf - -C "$HOME/HL_FMA_restore_20260929"
python3 verify_restored.py FILE_MANIFEST.json "$HOME/HL_FMA_restore_20260929/workspace"
```

압축 해제는 빈 전용 폴더에서 한다. USB의 `START_HERE.txt`에도 같은 절차를 둔다. checksum 오류나 복원 검증 오류가 있으면 원본 노트북이 남아 있을 때 다시 복사한다. 기존 절대경로 심볼릭 링크는 원본대로 보존하며 검증 도구는 그 링크의 문자값을 검사할 뿐 연결 대상을 실행/열지 않는다. 예전 경로를 가리키는 링크는 복원 검증 후 작업 사본에서 상대경로로 고친다.

## 학습부터 시작하는 경우

GitHub에서 clone한 폴더와 USB 복원 원본은 별도로 둔다. GitHub 사본에서 [학습 순서](../learning/README.md)와 [오프라인 실습](../learning/LABS.md)을 진행한다. 큰 원본 자료를 Git에 모두 추가하지 않는다.

## 개발 환경 재구성

현재 관측 환경은 Ubuntu24.04.5 LTS, x86_64, Python3.12.3이다. [환경 기록](environment/platform.json)과 분리된 네 환경의 `*-installed.txt`를 남겼다. 이 파일은 설치된 배포 패키지의 이름·버전 목록이며, 패키지 취득 URL·wheel·OS 라이브러리를 전부 고정한 재현 lockfile은 아니다.

```bash
# 복원 원본을 보존한 별도 작업 사본 루트에서, Python3.12가 설치된 경우
python3.12 -m venv .venv-rtk
.venv-rtk/bin/python -m pip install -r docs/migration/environment/venv-rtk-installed.txt
python3.12 -m venv .venv-oak
.venv-oak/bin/python -m pip install -r docs/migration/environment/venv-oak-installed.txt
python3.12 -m venv .venv-yolo
.venv-yolo/bin/python -m pip install -r docs/migration/environment/venv-yolo-installed.txt
```

이는 환경별 복원 출발점이며 새 노트북에서 설치 성공을 아직 검증하지 않았다. YOLO 환경은 PyTorch·GPU 관련 패키지가 크고 다른 CPU/GPU/OS에서 같은 패키지가 설치되지 않을 수 있다. 실패한 패키지와 오류를 기록해 해당 환경만 조정한다. 세 환경의 의존성을 무조건 한 환경에 합치지 않는다. 기존 `.venv*`를 복사해 사용하는 방법은 경로·바이너리 의존성 때문에 채택하지 않았다.

호스트 조향 C++ 라이브러리는 포함된 `firmware/steer_target/host/build.sh`로 새 환경에서 빌드한다. `.setup`에는 당시 Arduino/RPLIDAR SDK 자료가 있지만 다른 기계에서 바이너리가 바로 동작한다고 보장하지 않는다. USB 권한 설정은 `scripts/setup_oak_permissions.sh`와 기존 장치 문서를 읽고 해당 장치 사용 단계에서 적용한다. 학습 단계에 장치 연결은 필요하지 않다.

## 별도로 다룰 것

- **NTRIP 인증:** `config/ntrip.local.json`은 이번 작업에서 열거나 해시하거나 백업하지 않는다. 서비스 정보를 본인이 안전한 별도 방법으로 이전하거나 새 환경에서 재발급·재입력해야 한다. GitHub·공유 문서·명령 출력에 비밀번호를 넣지 않는다.
- **경로:** 원본의 `/home/jeonseongbin/HL FMA v2`와 세션 절대경로는 역사적 기록이다. 작업 사본의 설정·심볼릭 링크·SDK 위치는 새 경로에 맞춰 확인한다. 원본 로그 전체를 문자열 치환하지 않는다.
- **이전 세션:** `data/main_event_active*`, lock, UNIX socket, PID는 이식 가능한 실행 상태가 아니다. 원본을 기록으로 보존하고 새 작업에서는 이전 세션을 그대로 재개하지 않는다.
- **펌웨어:** 백업된 hex/EEPROM·배선·보정은 당시 차량용이다. 새 장치에 자동 업로드하지 않는다.
- **제출본:** `main_event_release.py verify`는 당시 원본 파일 집합의 검증이다. 새 환경 빌드나 경로 수정 후에는 새 개발 버전으로 검사·기록하며 이전 동결을 억지로 통과시키지 않는다.

## 반납 전 완료 기준

- USB 조각 checksum, 압축 스트림 읽기 및 내부 파일 해시 검증이 성공한다.
- 별도 빈 폴더에 학습에 필요한 코드·문서를 복원해 오프라인 재현이 된다.
- GitHub에서 README·학습·이전 문서에 접근할 수 있다.
- 새 노트북에서 전체 개발 환경 복구는 별도 확인한다. USB 백업 성공을 장치 연결·실차 재현 성공으로 표현하지 않는다.
- 노트북 자료 삭제/초기화는 백업과 필요한 계정·인증 이전이 끝난 뒤 사용자가 결정한다. 이번 작업은 원본을 삭제하지 않는다.

[이번 USB 보존·학습 사본 복원 결과](VALIDATION_2026-09-29.md)에서 실제 확인 범위와 제외 항목을 확인한다.
