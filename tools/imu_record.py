#!/usr/bin/env python3
"""Finite read-only iAHRS Euler/gyroscope recorder; no sensor configuration."""

import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import select
import signal
import sys
import time
import uuid
from zoneinfo import ZoneInfo

import probe_imu_lidar_once as probe

READ_COMMANDS = {b"vr\r\n", b"e\r\n", b"g\r\n"}
NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
MANUAL = probe.ROOT / "reference/sensors/IAHRS_manual_rb_sda_ob_v_dn_2.0.1_210607.pdf"


def parse_vector(raw, name):
    if name not in ("e", "g"):
        raise ValueError("Only e/g read responses are supported")
    text = raw.decode("ascii", "strict")
    matched = re.fullmatch(r"\s*" + name + "=(" + NUMBER + "),(" + NUMBER + "),(" + NUMBER + r")\s*", text)
    if matched is None:
        raise ValueError("Missing, duplicate, or malformed " + name + " response")
    values = [float(value) for value in matched.groups()]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Non-finite IMU value")
    return values


def recognized_version(raw):
    return bool(re.fullmatch(rb"\s*INTH iAHRSv1\s+H/W Ver 1\.01\s+S/W Ver 1\.21\s*", raw))


class ReadPort(probe.Port):
    """Reuse the established exact-device/flock/raw-log host serial setup."""

    def query(self, command, seconds=.12):
        if command not in READ_COMMANDS:
            raise ValueError("Only vr/e/g read queries are permitted")
        if command == b"vr\r\n":
            return super().query(command, seconds)
        # Unsolicited/late data must not be assigned to a new query. Preserve
        # it through the existing raw receiver, then end this recording.
        ready, _, _ = select.select([self.fd], [], [], 0)
        if ready:
            self.receive(.01)
            raise ValueError("Unexpected pending IMU data before query")
        self.log("tx_attempt", hex=command.hex())
        written = os.write(self.fd, command)
        self.log("tx_completed", hex=command[:written].hex())
        if written != len(command):
            raise OSError("Partial command write; no retry")
        deadline = time.monotonic() + seconds
        result = bytearray()
        while time.monotonic() < deadline:
            ready, _, _ = select.select([self.fd], [], [], max(0., deadline - time.monotonic()))
            if not ready:
                break
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                raise OSError("IMU disconnected")
            self.raw.write(chunk)
            self.raw.flush()
            self.log("rx", hex=chunk.hex())
            result.extend(chunk)
            if len(result) > 256:
                raise ValueError("IMU response exceeds bounded line size")
            if b"\n" in result:
                break
        return bytes(result)


def main(argv=None):
    parser = argparse.ArgumentParser(description="IMU 자세 e/각속도 g를 설정 변경 없이 기록합니다. yaw는 절대방위가 아닙니다.")
    timing = parser.add_mutually_exclusive_group()
    timing.add_argument("--duration", type=float, default=60.)
    timing.add_argument("--until-interrupt", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--port", type=Path, help="Optional; must exactly match the identified serial0001 by-id path")
    parser.add_argument("--rate", type=float, default=10., help="Requested pair polling rate, 1..10 Hz (not guaranteed device sample rate)")
    args = parser.parse_args(argv)
    duration = None if args.until_interrupt else args.duration
    if duration is not None and (not math.isfinite(duration) or not 1 <= duration <= 3600):
        parser.error("duration must be 1..3600 seconds")
    if not math.isfinite(args.rate) or not 1 <= args.rate <= 10:
        parser.error("rate must be 1..10 Hz")
    output = args.output_dir or probe.ROOT / "data" / (
        datetime.now(ZoneInfo("Asia/Seoul")).strftime("imu_attitude_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = {"started_utc": probe.now(), "status": "starting", "requested_duration_s": duration,
              "requested_poll_rate_hz": args.rate, "source_sha256": {},
              "execution": [sys.executable, str(Path(__file__).resolve()), *(sys.argv[1:] if argv is None else argv)],
              "scope": "Only identified CP2102 serial0001; vr/e/g read queries; no settings/reset/calibration/auto-output changes",
              "units": {"e": "degrees: roll,pitch,yaw; Euler Z-Y-X", "g": "degrees/second: sensor-frame x,y,z"},
              "limitations": ["Host timestamps; e and g queried sequentially, not synchronized device samples",
                  "No sensor timestamp or sequence counter in these ASCII replies; no loss-free claim",
                  "ASCII replies have no checksum; strict syntax alone does not establish measurement accuracy",
                  "Last confirmed mv=0 and automatic output off; not queried or changed in this run",
                  "Yaw is not verified north heading; mounting axes/extrinsics are unmeasured",
                  "Target rate is polling demand; actual timestamps and intervals must be used"],
              "source_manual": {"path": str(MANUAL), "page": 20,
                                "source_url": "https://blog.naver.com/robor21/222214954034"}}
    for source in (Path(__file__), Path(probe.__file__)):
        raw = source.read_bytes()
        (output / source.name).write_bytes(raw)
        report["source_sha256"][source.name] = hashlib.sha256(raw).hexdigest()
    if MANUAL.is_file():
        report["source_manual"]["sha256"] = hashlib.sha256(MANUAL.read_bytes()).hexdigest()
    print("OUTPUT=" + str(output), flush=True)
    port = None
    code = 1
    count, attempted = 0, 0
    first_at = last_at = None
    max_gap = 0.
    extrema = {name: {"min": None, "max": None} for name in ("euler_deg", "gyro_deg_s")}
    try:
        ident = probe.identity(probe.IMU_SERIAL)
        report["usb_identity"] = ident
        if args.port is not None and str(args.port) != ident["port"]:
            raise ValueError("--port must be the exact identified IMU by-id path")
        port = ReadPort(ident, 115200, output / "imu")
        check = probe.identity(probe.IMU_SERIAL)
        if check != ident or os.fstat(port.fd).st_rdev != os.stat(ident["port"]).st_rdev:
            raise RuntimeError("IMU identity changed while opening port")
        report["passive_bytes"] = len(port.receive(.15))
        version = port.query(b"vr\r\n", .6)
        report["version_response"] = version.decode("ascii", "replace")
        if not recognized_version(version):
            raise ValueError("Expected iAHRSv1 HW1.01/SW1.21 was not received")
        started = time.monotonic()
        deadline = None if duration is None else started + duration
        report.update(status="recording", sampling_started_utc=probe.now())
        (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        with (output / "samples.jsonl").open("x", encoding="utf-8", buffering=1) as stream:
            while deadline is None or time.monotonic() < deadline:
                sample_started = time.monotonic()
                sample = {"index": attempted, "host_utc": probe.now(), "host_monotonic_s": sample_started,
                          "elapsed_s": sample_started - started, "valid": False, "queries": {}}
                attempted += 1
                try:
                    for command, field in (("e", "euler_deg"), ("g", "gyro_deg_s")):
                        query = {"sent_host_utc": probe.now(), "sent_monotonic_s": time.monotonic()}
                        sample["queries"][command] = query
                        raw = port.query((command + "\r\n").encode("ascii"))
                        query.update(received_host_utc=probe.now(), received_monotonic_s=time.monotonic(),
                                     response=raw.decode("ascii", "replace"), response_hex=raw.hex())
                        sample[field] = parse_vector(raw, command)
                    sample["valid"] = True
                    count += 1
                    if first_at is None:
                        first_at = sample_started
                    if last_at is not None:
                        max_gap = max(max_gap, sample_started - last_at)
                    last_at = sample_started
                    for field, stat in extrema.items():
                        stat["min"] = [min(a, b) for a, b in zip(stat["min"], sample[field])] if stat["min"] else sample[field][:]
                        stat["max"] = [max(a, b) for a, b in zip(stat["max"], sample[field])] if stat["max"] else sample[field][:]
                except (OSError, ValueError) as exc:
                    sample.update(valid=False, error=str(exc))
                    raise
                finally:
                    sample["completed_host_utc"] = probe.now()
                    stream.write(json.dumps(sample, ensure_ascii=False) + "\n")
                target = sample_started + 1 / args.rate
                remaining = (target if deadline is None else min(target, deadline)) - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
        report.update(status="completed", sampling_elapsed_s=time.monotonic() - started)
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
        span = last_at - first_at if count > 1 else None
        report["summary"] = {"attempted_pairs": attempted, "valid_pairs": count,
            "observed_pair_rate_hz": (count - 1) / span if span and span > 0 else None,
            "maximum_pair_start_gap_s": max_gap if count > 1 else None,
            "value_extrema": extrema}
        report["ended_utc"] = probe.now()
        (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps({"status": report["status"], **report["summary"], "error": report.get("error")}), flush=True)
    return code


if __name__ == "__main__":
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    raise SystemExit(main())
