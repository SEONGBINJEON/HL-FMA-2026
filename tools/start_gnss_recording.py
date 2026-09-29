#!/usr/bin/env python3
"""Identify one connected u-blox USB tty, then invoke the existing recorder."""

import argparse
from datetime import datetime
import math
from pathlib import Path
import subprocess
import sys
import uuid

from check_devices import device_inventory


def select_port(inventory):
    usb_ids = {item["sysfs_device"] for item in inventory["usb_devices"]
               if (item["vendor_id"], item["product_id"]) == ("1546", "01a9")}
    candidates = [item for item in inventory["serial_ports"]
                  if item.get("usb_parent") in usb_ids
                  and item.get("permissions", {}).get("present")
                  and item.get("permissions", {}).get("character_device")]
    if len(candidates) != 1:
        raise ValueError(f"대상 u-blox USB 시리얼 포트가 {len(candidates)}개입니다. "
                         "기록할 수신기 한 대만 연결한 뒤 다시 실행하세요. 포트는 열지 않았습니다.")
    chosen = candidates[0]
    if not chosen["permissions"].get("readable_by_current_process"):
        raise ValueError("선택한 u-blox 포트의 읽기 권한이 없습니다. 장치 권한 설정을 확인하세요. 포트는 열지 않았습니다.")
    stable = [path for path in chosen.get("stable_paths", []) if "/serial/by-id/" in path]
    return sorted(stable)[0] if stable else chosen["device"]


def main(argv=None):
    parser = argparse.ArgumentParser(description="u-blox 수신기 한 대를 자동 선택해 GNSS를 기록하고 요약합니다.")
    parser.add_argument("seconds", nargs="?", type=float, default=60.0,
                        help="기록 시간(초), 기본 60")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("기록 시간은 0보다 큰 유한한 값이어야 합니다.")
    root = Path(__file__).resolve().parents[1]
    try:
        port = select_port(device_inventory())
        output = root / "data" / ("gnss_" + datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8])
        print(f"기록 포트: {port}\n기록 시간: {args.seconds:g}초\n저장 폴더: {output}", flush=True)
        result = subprocess.run([sys.executable, str(root / "tools/gnss_record.py"),
                                 "--port", port, "--duration", str(args.seconds),
                                 "--output-dir", str(output)], cwd=root, check=False)
        if result.returncode:
            return result.returncode
        return subprocess.run([sys.executable, str(root / "tools/gnss_analyze.py"),
                               "--session", str(output)], cwd=root, check=False).returncode
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("GNSS 기록을 중단했습니다. 생성된 기록 폴더를 확인하세요.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
