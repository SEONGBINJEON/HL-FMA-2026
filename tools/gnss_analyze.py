#!/usr/bin/env python3
"""Summarize existing GNSS recordings without changing them or opening devices."""

import argparse
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path
import sys


QUALITY = {0: "invalid", 1: "gps_fix", 2: "dgps_fix", 3: "pps_fix",
           4: "rtk_fixed", 5: "rtk_float", 6: "estimated", 7: "manual", 8: "simulation"}
MEASURED_QUALITY = {1, 2, 3, 4, 5}


def finite(value, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        if not math.isfinite(value):
            return None
    except OverflowError:
        return None
    if minimum is not None and value < minimum:
        return None
    if maximum is not None and value > maximum:
        return None
    return value


def stats(values):
    return {"samples": len(values), "min": min(values), "mean": sum(v / len(values) for v in values),
            "max": max(values)} if values else {"samples": 0, "min": None, "mean": None, "max": None}


def host_time(value):
    if not isinstance(value, str):
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
        return parsed if parsed.tzinfo is not None else None
    except ValueError:
        return None


def analyze(session):
    directory = Path(session).expanduser().resolve()
    counts = Counter()
    malformed = records = coordinates = measured = fixed_coordinates = 0
    intervals = []
    elapsed = []
    times = []
    metrics = {"satellites": [], "hdop": [], "differential_age_s": []}
    reversals = 0
    previous = None
    with (directory / "gga.jsonl").open(encoding="utf-8") as source:
        for line in source:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("record must be an object")
            except (ValueError, TypeError):
                malformed += 1
                continue
            records += 1
            quality = row.get("quality_code")
            if isinstance(quality, bool) or not isinstance(quality, int):
                quality = None
            # The raw quality code, never fix_status text, establishes RTK state.
            verified = row.get("checksum_valid") is True
            label = QUALITY.get(quality, "unknown") if verified else "unverified_checksum"
            counts[label] += 1
            lat = finite(row.get("latitude_deg"), -90, 90)
            lon = finite(row.get("longitude_deg"), -180, 180)
            if lat is not None and lon is not None:
                coordinates += 1
                if verified and quality in MEASURED_QUALITY:
                    measured += 1
                    fixed_coordinates += int(quality == 4)
            value = finite(row.get("elapsed_s"), 0)
            if value is not None:
                elapsed.append(value)
                if previous is not None:
                    if value >= previous:
                        intervals.append(value - previous)
                    else:
                        reversals += 1
                previous = value
            else:
                previous = None
            timestamp = host_time(row.get("host_utc"))
            if timestamp is not None:
                times.append(timestamp)
            for key in metrics:
                value = finite(row.get(key), 0, 999 if key == "satellites" else None)
                if value is not None:
                    metrics[key].append(value)
    metadata = {}
    warnings = []
    try:
        candidate = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
        if not isinstance(candidate, dict):
            raise ValueError("metadata must be an object")
        for key in ("status", "started_utc", "ended_utc"):
            metadata[key] = candidate.get(key) if isinstance(candidate.get(key), str) else None
        for key in ("elapsed_s", "bytes_received"):
            metadata[key] = finite(candidate.get(key), 0)
        metadata["blocked_on_incomplete_ubx"] = candidate.get("blocked_on_incomplete_ubx") is True
    except (OSError, ValueError):
        warnings.append("metadata.json이 없거나 읽을 수 없습니다. GGA 파일만 요약했습니다.")
    if records == 0:
        warnings.append("분석 가능한 GGA 기록이 없습니다. 위치·RTK 상태를 판단할 수 없습니다.")
    if metadata.get("blocked_on_incomplete_ubx"):
        warnings.append("미완성 UBX로 문장 분석이 막힌 기록입니다. 원시 파일을 보존하세요.")
    return {
        "schema_version": 1, "session": str(directory), "records": records,
        "malformed_lines": malformed, "recording_metadata": metadata,
        "first_host_utc": min(times).isoformat() if times else None,
        "last_host_utc": max(times).isoformat() if times else None,
        "observed_gga_span_s": max(elapsed) - min(elapsed) if elapsed else None,
        "quality_counts": dict(counts),
        "quality_percent": {key: round(100 * value / records, 3) for key, value in counts.items()},
        "finite_coordinate_records": coordinates,
        "measured_position_records": measured,
        "rtk_fixed_coordinate_records": fixed_coordinates,
        "gga_receive_interval_s": stats(intervals), "elapsed_time_reversals": reversals,
        "metrics": {key: stats(values) for key, values in metrics.items()},
        "warnings": warnings,
        "limitations": [
            "기존 수신 기록의 요약입니다. 절대 위치 정확도나 자율주행 준비 완료를 판정하지 않습니다.",
            "정지 시험 여부를 알 수 없으며 좌표 산포를 위치 오차로 계산하지 않습니다.",
            "품질 4만 RTK fixed, 5만 RTK float입니다. quality 0/6/7/8/unknown은 실측 유효 위치 수에서 제외합니다.",
            "측정 위치 개수는 경로 사용 승인이 아닙니다. 일반 GNSS와 DGPS도 포함되며 반복 검증이 필요합니다.",
            "수신 간격은 PC의 GGA elapsed_s 기준이며 센서 전체 주기·동기화를 보증하지 않습니다.",
        ],
    }


def short_summary(report):
    quality = ", ".join(f"{key} {value}개 ({report['quality_percent'][key]:g}%)"
                        for key, value in report["quality_counts"].items()) or "알 수 없음"
    print(f"GGA {report['records']}개 / 읽기 실패 줄 {report['malformed_lines']}개")
    print("품질: " + quality)
    span = report["observed_gga_span_s"]
    print("관측 GGA 시간 범위: " + (f"{span:.3f}초" if span is not None else "알 수 없음"))
    print(f"실측 품질+좌표 기록: {report['measured_position_records']}개 / 그중 RTK fixed 좌표 {report['rtk_fixed_coordinate_records']}개")
    for title, values in [("GGA 수신 간격(초)", report["gga_receive_interval_s"]),
                          ("위성 수", report["metrics"]["satellites"]),
                          ("HDOP", report["metrics"]["hdop"]),
                          ("보정 나이(초)", report["metrics"]["differential_age_s"])]:
        print(f"{title}: 평균 {values['mean']:.3f}, 최소 {values['min']:.3f}, 최대 {values['max']:.3f}"
              if values["samples"] else f"{title}: 알 수 없음")
    for warning in report["warnings"]:
        print("확인 필요: " + warning)
    print("이 결과는 위치 정확도·RTK 주행 준비 완료 판정이 아닙니다.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path, help="기존 GNSS 기록 폴더")
    parser.add_argument("--output", type=Path, help="새 JSON 보고서 경로; 기존 파일은 거부")
    args = parser.parse_args(argv)
    try:
        report = analyze(args.session)
        if args.output is not None:
            with args.output.expanduser().open("x", encoding="utf-8") as target:
                json.dump(report, target, ensure_ascii=False, indent=2, allow_nan=False)
                target.write("\n")
            print("분석 보고서 저장: " + str(args.output))
        else:
            short_summary(report)
        return 0
    except (OSError, ValueError) as exc:
        print("GNSS 분석 실패: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
