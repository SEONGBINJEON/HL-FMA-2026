#!/usr/bin/env python3
"""Record finite, read-only iAHRS power/status queries on the identified IMU."""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import sys
import time
import uuid
from zoneinfo import ZoneInfo

import probe_imu_lidar_once as probe


def parse_sample(voltage, status):
    pv = re.fullmatch(r"\s*pv=(\d+(?:\.\d+)?)\s*", voltage)
    state = re.fullmatch(r"\s*s=(0x[0-9a-fA-F]+)\s*", status)
    if not pv or not state:
        raise ValueError("Incomplete/unrecognized IMU voltage or status response")
    return float(pv[1]), int(state[1], 16)


def main(argv=None):
    parser = argparse.ArgumentParser(description="식별된 IMU의 전압/경고를 약1Hz로 유한 시간 기록. 설정 변경·차량 제어 없음.")
    parser.add_argument("--duration", type=float, default=60)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 600:
        parser.error("duration must be finite and between 1 and 600 seconds")
    output = probe.ROOT / "data" / (datetime.now(ZoneInfo("Asia/Seoul")).strftime("imu_power_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
    output.mkdir()
    report = {"started_utc": probe.now(), "requested_duration_s": args.duration,
              "status": "starting", "samples": [], "source_sha256": {},
              "execution": [sys.executable, str(Path(__file__).resolve()), *(sys.argv[1:] if argv is None else argv)],
              "scope": "CP2102 serial0001 only; vr, pv, s read queries. No settings, motor control, reset, or other sensor ports.",
              "voltage_note": "IMU-reported internal voltage, not a direct measurement at the hub input.",
              "time_note": "Host timestamps for sequential queries; voltage and status are not simultaneous sensor samples."}
    for source in (Path(__file__), Path(probe.__file__)):
        raw = source.read_bytes()
        (output / source.name).write_bytes(raw)
        report["source_sha256"][source.name] = hashlib.sha256(raw).hexdigest()
    print("OUTPUT=" + str(output), flush=True)
    port = None
    code = 1
    try:
        ident = probe.identity(probe.IMU_SERIAL)
        report["usb_identity"] = ident
        port = probe.Port(ident, 115200, output / "imu")
        report["passive_bytes"] = len(port.receive(1))
        version = port.query(b"vr\r\n", 1.5).decode("ascii", "replace")
        report["version_response"] = version
        if not (re.search(r"(?m)^\s*INTH iAHRSv1\s*$", version) and "H/W Ver" in version and "S/W Ver" in version):
            raise ValueError("Expected iAHRS version response was not received")
        start = time.monotonic()
        report["sampling_started_utc"] = probe.now()
        report["status"] = "recording"
        with (output / "samples.jsonl").open("x", encoding="utf-8", buffering=1) as stream:
            while time.monotonic() - start < args.duration:
                sample_start = time.monotonic()
                sample = {"host_utc": probe.now(), "elapsed_s": sample_start - start}
                voltage = port.query(b"pv\r\n", 0.4).decode("ascii", "replace")
                status = port.query(b"s\r\n", 0.4).decode("ascii", "replace")
                sample.update(pv_response=voltage, s_response=status, completed_host_utc=probe.now())
                try:
                    volts, flags = parse_sample(voltage, status)
                    sample.update(imu_reported_voltage_v=volts, status_flags=flags,
                                  voltage_warning=bool(flags & 0x20000))
                except ValueError as exc:
                    sample["error"] = str(exc)
                    stream.write(json.dumps(sample) + "\n")
                    report["samples"].append(sample)
                    raise
                stream.write(json.dumps(sample) + "\n")
                report["samples"].append(sample)
                remaining = min(sample_start + 1, start + args.duration) - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
        report["sampling_elapsed_s"] = time.monotonic() - start
        report["status"] = "completed"
        code = 0
    except KeyboardInterrupt:
        report["status"] = "interrupted"
        code = 130
    except (OSError, ValueError, RuntimeError) as exc:
        report.update(status="failed", error=str(exc))
    finally:
        if port is not None:
            try:
                port.close()
            except OSError as exc:
                report.update(status="failed", close_error=str(exc))
                code = 1
        valid = [s for s in report["samples"] if "status_flags" in s]
        report["summary"] = {"valid_samples": len(valid),
            "voltage_min_v": min((s["imu_reported_voltage_v"] for s in valid), default=None),
            "voltage_max_v": max((s["imu_reported_voltage_v"] for s in valid), default=None),
            "voltage_warning_samples": sum(s["voltage_warning"] for s in valid),
            "nonzero_status_samples": sum(s["status_flags"] != 0 for s in valid)}
        report["ended_utc"] = probe.now()
        (output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        print(json.dumps({"status": report["status"], **report["summary"], "error": report.get("error")}), flush=True)
    return code


if __name__ == "__main__":
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    raise SystemExit(main())
