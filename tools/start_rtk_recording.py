#!/usr/bin/env python3
"""Start an RTK recording in a unique folder without exposing credentials."""

import argparse
from datetime import datetime
import math
from pathlib import Path
import sys
import uuid

from rtk_record import main as record_main
from gnss_analyze import main as analyze_main


def main(argv=None):
    parser = argparse.ArgumentParser(description="보정서비스 접속과 GNSS 기록. 기본 180초, 날짜별 새 폴더 사용")
    parser.add_argument("seconds", nargs="?", type=float, default=180.0)
    parser.add_argument("--until-interrupt", action="store_true", help="Ctrl+C까지 기록")
    args = parser.parse_args(argv)
    if not args.until_interrupt and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("기록 시간은 0보다 큰 유한한 초 단위 값이어야 합니다.")
    root = Path(__file__).resolve().parents[1]
    config = root / "config/ntrip.local.json"
    if not config.is_file():
        print("먼저 configure_ntrip.py로 접속 정보를 비공개 입력하세요.", file=sys.stderr)
        return 1
    output = root / "data" / ("rtk_" + datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8])
    timing = ["--until-interrupt"] if args.until_interrupt else ["--duration", str(args.seconds)]
    label = "Ctrl+C까지" if args.until_interrupt else f"{args.seconds:g}초"
    print(f"RTK 기록 시간: {label}\n저장 폴더: {output}", flush=True)
    result = record_main(["--config", str(config), *timing, "--output-dir", str(output)])
    if (output / "gga.jsonl").is_file():
        analyze_main(["--session", str(output)])
    return result


if __name__ == "__main__":
    raise SystemExit(main())
