#!/usr/bin/env python3
"""Collect NTRIP settings privately in the user's own terminal, without connecting."""

import argparse
import getpass
import json
import os
from pathlib import Path
import sys
import tempfile


DEFAULT_PATH = Path(__file__).resolve().parents[1] / "config/ntrip.local.json"


def validate(config):
    for key in ("server", "mountpoint", "ntripuser", "ntrippassword"):
        value = config.get(key)
        if not isinstance(value, str) or not value or any(ord(c) < 32 for c in value):
            raise ValueError("서버·마운트포인트·계정 정보는 비어 있거나 제어문자를 포함할 수 없습니다.")
    if any(c in config["server"] for c in "/:@ "):
        raise ValueError("서버에는 주소만 입력하세요. http://, 포트, 경로는 제외합니다.")
    if any(c in config["mountpoint"] for c in "/?# "):
        raise ValueError("마운트포인트에는 서비스에서 알려준 신호 이름만 입력하세요.")
    if ":" in config["ntripuser"]:
        raise ValueError("보정서비스 아이디에는 콜론을 사용할 수 없습니다.")
    port = config.get("port")
    if isinstance(port, bool) or not isinstance(port, int) or not 2 <= port <= 65535:
        raise ValueError("포트는 2~65535 사이 숫자로 입력하세요.")
    if config.get("version") not in ("1.0", "2.0") or not isinstance(config.get("https"), bool):
        raise ValueError("NTRIP 버전과 TLS 설정을 확인하세요.")
    interval = config.get("ggainterval")
    if isinstance(interval, bool) or not isinstance(interval, int) or not 1 <= interval <= 60:
        raise ValueError("GGA 전송 간격은 1~60초 사이 정수여야 합니다.")
    return config


def save_private(config, target, replace=False):
    validate(config)
    target = Path(target)
    if target.exists() or target.is_symlink():
        if not replace or target.is_symlink():
            raise FileExistsError("기존 설정은 변경하지 않았습니다. 수정하려면 --replace를 사용하세요.")
    target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".ntrip-", dir=target.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            os.fchmod(handle.fileno(), 0o600)
            json.dump(config, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        if replace:
            os.replace(temporary, target)
        else:
            # A concurrent setup must not overwrite another complete file.
            os.link(temporary, target)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main(argv=None):
    parser = argparse.ArgumentParser(description="보정서비스 접속 정보를 이 PC에 비공개 저장합니다. 네트워크 접속은 하지 않습니다.")
    parser.add_argument("--replace", action="store_true", help="기존 로컬 설정을 새 입력으로 교체")
    args = parser.parse_args(argv)
    if not sys.stdin.isatty():
        print("사용자 Ubuntu 터미널에서 실행하세요. 숨김 입력이 가능한 터미널이 필요합니다.", file=sys.stderr)
        return 1
    if DEFAULT_PATH.exists() and not args.replace:
        print("이미 비공개 설정이 있습니다. 수정할 때만 --replace로 실행하세요.")
        return 1
    print("서비스 제공자가 알려준 접속 정보를 입력하세요. 아이디·비밀번호는 화면에 표시되지 않습니다.")
    print("이 단계는 저장만 합니다. 이후 RTK 실행 시 지정한 서비스에 현재 GNSS 위치(GGA)를 전송합니다.")
    try:
        server = input("서버 주소 (http://와 포트 제외): ").strip()
        port = int(input("포트 번호: ").strip())
        mountpoint = input("마운트포인트 (보정 신호 이름): ").strip()
        tls = input("서비스가 TLS/HTTPS 연결을 제공하나요? [y/N]: ").strip().lower()
        if tls not in ("", "n", "no", "y", "yes"):
            raise ValueError("TLS 설정은 y 또는 n으로 입력하세요.")
        version = input("NTRIP 버전 [기본 2.0, 필요하면 1.0]: ").strip() or "2.0"
        username = getpass.getpass("보정서비스 아이디 (숨김 입력): ")
        password = getpass.getpass("보정서비스 비밀번호 (숨김 입력): ")
        if password != getpass.getpass("비밀번호 한 번 더 (숨김 입력): "):
            raise ValueError("두 비밀번호가 일치하지 않습니다. 저장하지 않았습니다.")
        config = {"server": server, "port": port, "mountpoint": mountpoint,
                  "https": tls in ("y", "yes"), "version": version,
                  "ntripuser": username, "ntrippassword": password, "ggainterval": 10}
        save_private(config, DEFAULT_PATH, args.replace)
    except (KeyboardInterrupt, EOFError):
        print("\n입력을 중단했습니다.")
        return 130
    except ValueError:
        print("입력 형식 또는 비밀번호 확인이 맞지 않습니다. 저장하지 않았습니다.", file=sys.stderr)
        return 1
    except OSError:
        print("설정을 저장하지 못했습니다. 파일 존재 여부와 접근 권한을 확인하세요.", file=sys.stderr)
        return 1
    print("비공개 설정을 저장했습니다. 파일 내용은 채팅에 붙여넣지 말고 '입력 완료'라고 알려주세요.")
    print("아직 보정서비스 접속이나 RTK FIX를 검증한 것은 아닙니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
