#!/usr/bin/env python3
"""Offline GNSS antenna observations; never a validated vehicle driving route."""
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
from html import escape
import json
import math
from pathlib import Path
import sys

try:
    from .gnss_analyze import finite, QUALITY, MEASURED_QUALITY
    from .gnss_record import parse_gga
except ImportError:
    from gnss_analyze import finite, QUALITY, MEASURED_QUALITY
    from gnss_record import parse_gga

A = 6378137.0
F = 1 / 298.257223563
SOURCES = [
    "https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84",
    "https://gssc.esa.int/navipedia/index.php/Ellipsoidal_and_Cartesian_Coordinates_Conversion",
    "https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates",
]
COLORS = {"FIX": "#087f5b", "FLOAT": "#c56c00", "OTHER": "#65758b"}
FIELDS = ["source_line", "segment_id", "break_reason", "elapsed_s", "host_utc",
          "quality_code", "quality_label", "group", "latitude_deg", "longitude_deg",
          "east_m", "north_m", "plotted", "error"]


def ecef(latitude, longitude, height=0):
    """WGS84 geodetic degrees/ellipsoid metres -> ECEF metres (ESA eq.1)."""
    if (finite(latitude, -90, 90) is None or finite(longitude, -180, 180) is None
            or finite(height) is None):
        raise ValueError("Invalid geodetic coordinate")
    lat, lon = math.radians(latitude), math.radians(longitude)
    e2 = F * (2 - F)
    n = A / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    return ((n + height) * math.cos(lat) * math.cos(lon),
            (n + height) * math.cos(lat) * math.sin(lon),
            (n * (1 - e2) + height) * math.sin(lat))


def enu(latitude, longitude, origin):
    """Local E/N/U from ECEF differences; both heights fixed to zero explicitly."""
    base = ecef(*origin)
    dx, dy, dz = (value - ref for value, ref in zip(ecef(latitude, longitude), base))
    lat, lon = map(math.radians, origin)
    return (-math.sin(lon) * dx + math.cos(lon) * dy,
            -math.sin(lat) * math.cos(lon) * dx - math.sin(lat) * math.sin(lon) * dy + math.cos(lat) * dz,
            math.cos(lat) * math.cos(lon) * dx + math.cos(lat) * math.sin(lon) * dy + math.sin(lat) * dz)


def analyze(raw, max_gap_s=1.5, motion="unknown"):
    if finite(max_gap_s, 0) is None or max_gap_s == 0:
        raise ValueError("max_gap_s must be positive and finite")
    if motion not in ("stationary", "moving", "unknown"):
        raise ValueError("Invalid motion declaration")
    points, segments = [], []
    counts, errors = Counter(), Counter()
    origin = previous = None
    pending_break = "start"
    for number, line in enumerate(raw.decode("utf-8").splitlines(), 1):
        if not line.strip():
            previous = None
            pending_break = "blank_line"
            errors[pending_break] += 1
            continue
        p = dict.fromkeys(FIELDS)
        p.update(source_line=number, plotted=False)
        try:
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError("not_object")
            t = finite(row.get("elapsed_s"), 0)
            p.update(elapsed_s=t, host_utc=row.get("host_utc") if isinstance(row.get("host_utc"), str) else None)
            q = row.get("quality_code")
            q = q if type(q) is int else None
            label = QUALITY.get(q, "unknown") if row.get("checksum_valid") is True else "unverified_checksum"
            counts[label] += 1
            p.update(quality_code=q, quality_label=label,
                     group="FIX" if q == 4 else "FLOAT" if q == 5 else "OTHER")
            lat, lon = finite(row.get("latitude_deg"), -90, 90), finite(row.get("longitude_deg"), -180, 180)
            p.update(latitude_deg=lat, longitude_deg=lon)
            # Recheck the retained NMEA sentence, not just a JSON boolean.
            sentence = row.get("sentence")
            checked = parse_gga(sentence.encode("ascii")) if isinstance(sentence, str) else None
            if row.get("checksum_valid") is not True or checked is None:
                raise ValueError("unverified_sentence")
            if any(checked.get(key) != row.get(key) for key in ("quality_code", "latitude_deg", "longitude_deg")):
                raise ValueError("sentence_columns_disagree")
            if q not in MEASURED_QUALITY:
                raise ValueError("non_measured_quality")
            if lat is None or lon is None:
                raise ValueError("invalid_coordinates")
            if t is None:
                raise ValueError("missing_or_invalid_time")
            if origin is None:
                origin = (lat, lon)
            east, north, _ = enu(lat, lon, origin)
            p.update(east_m=east, north_m=north, plotted=True)
            reason = pending_break if previous is None else ""
            if previous is not None:
                dt = t - previous["elapsed_s"]
                reasons = []
                if dt <= 0:
                    reasons.append("time_not_increasing")
                elif dt > max_gap_s:
                    reasons.append("time_gap")
                if q != previous["quality_code"]:
                    reasons.append("quality_transition")
                reason = "+".join(reasons)
            if reason or not segments:
                segments.append({"segment_id": len(segments) + 1, "quality_code": q,
                    "quality_label": label, "group": p["group"], "break_reason": reason or "start",
                    "start_line": number, "end_line": number, "start_elapsed_s": t,
                    "end_elapsed_s": t, "samples": 0, "duration_s": 0.0})
            s = segments[-1]
            p.update(segment_id=s["segment_id"], break_reason=reason)
            s.update(end_line=number, end_elapsed_s=t, samples=s["samples"] + 1,
                     duration_s=t - s["start_elapsed_s"])
            previous, pending_break = p, ""
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            reason = str(exc) if str(exc) in {"not_object", "unverified_sentence", "sentence_columns_disagree",
                "non_measured_quality", "invalid_coordinates", "missing_or_invalid_time"} else "malformed_record"
            p.update(error=reason, break_reason=reason, plotted=False)
            errors[reason] += 1
            previous, pending_break = None, "after_" + reason
        points.append(p)
    valid = [p for p in points if p["plotted"]]
    summary = {"schema_version": 1, "motion_declaration": motion,
        "description": "정지 안테나 관측 산포" if motion == "stationary" else "GNSS 안테나 관측 궤적 (차량 경로 검증 아님)",
        "input_records": len(points), "plotted_observations": len(valid), "quality_counts": dict(counts),
        "excluded_or_missing": dict(errors), "segments": len(segments), "max_gap_s": max_gap_s,
        "origin": {"latitude_deg": origin[0], "longitude_deg": origin[1], "ellipsoid_height_m": 0,
                   "selection": "first checksum-verified measured coordinate with valid elapsed_s"} if origin else None,
        "coordinates": "WGS84 geodetic lat/lon -> ECEF differences -> ENU; east right, north up; metres; antenna observations",
        "height_policy": "All lat/lon projected to WGS84 ellipsoid height 0 for a horizontal preview. GGA altitude/geoid are not used; Up is not exported as a measured height.",
        "extent_m": {axis: [min(p[axis] for p in valid), max(p[axis] for p in valid)] for axis in ("east_m", "north_m")} if valid else {},
        "limitations": ["WGS84 is assumed; receiver datum/antenna lever arm and vehicle orientation are unverified.",
            "Only FIX observations in one uninterrupted same-quality segment may be connected; FLOAT/other qualities are points only.",
            "Stationary mode never draws path lines. Unknown motion is not inferred from coordinate spread.",
            "No interpolation across gaps, exclusions, non-increasing time or quality transitions. No path length, speed or accuracy claim.",
            "This is not a drivable vehicle path, surveyed map, heading estimate or autonomous-readiness assessment.",
            "Time continuity uses recorder elapsed_s, not receiver time or a sensor synchronization guarantee."],
        "sources": SOURCES}
    return points, segments, summary


def svg(points, summary):
    valid = [p for p in points if p["plotted"]]
    lines = ['<svg xmlns="http://www.w3.org/2000/svg" width="960" height="830" viewBox="0 0 960 830">',
             '<rect width="960" height="830" fill="#f8fafc"/>',
             '<g font-family="sans-serif" fill="#15283b">',
             f'<text x="50" y="38" font-size="22">{escape(summary["description"])}</text>',
             '<text x="50" y="65" font-size="13">GNSS antenna only · equal-scale East/North [m] · no geographic basemap</text>']
    for i, (group, color) in enumerate(COLORS.items()):
        lines.append(f'<circle cx="{65+i*180}" cy="91" r="5" fill="{color}"/><text x="{80+i*180}" y="96" font-size="13">{group}</text>')
    if valid:
        emin, emax = summary["extent_m"]["east_m"]
        nmin, nmax = summary["extent_m"]["north_m"]
        span = max(emax - emin, nmax - nmin, 1.0) * 1.15
        ec, nc = (emin + emax) / 2, (nmin + nmax) / 2
        def xy(p):
            return (480 + (p["east_m"] - ec) * 560 / span, 410 - (p["north_m"] - nc) * 560 / span)
        for i in range(6):
            pos = 130 + i * 112
            value = nc + span / 2 - i * span / 5
            lines += [f'<path d="M200 {pos}H760" stroke="#d9e2ea"/>',
                      f'<text x="188" y="{pos+4}" text-anchor="end" font-size="12">{value:.3f}</text>']
            x = 200 + i * 112
            value = ec - span / 2 + i * span / 5
            lines += [f'<path d="M{x} 130V690" stroke="#d9e2ea"/>',
                      f'<text x="{x}" y="711" text-anchor="middle" font-size="12">{value:.3f}</text>']
        if summary["motion_declaration"] != "stationary":
            groups = {}
            for p in valid:
                if p["group"] == "FIX":
                    groups.setdefault(p["segment_id"], []).append(p)
            for segment in groups.values():
                if len(segment) > 1:
                    coords = " ".join(f"{x:.3f},{y:.3f}" for x, y in map(xy, segment))
                    lines.append(f'<polyline points="{coords}" fill="none" stroke="{COLORS["FIX"]}" stroke-width="1.3" opacity="0.6"/>')
        for p in valid:
            x, y = xy(p)
            lines.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="2.6" fill="{COLORS[p["group"]]}" opacity="0.65"><title>line {p["source_line"]}; {p["group"]}; elapsed {p["elapsed_s"]} s</title></circle>')
        origin = summary["origin"]
        lines.append(f'<text x="50" y="765" font-size="12">Origin: {origin["latitude_deg"]:.9f}°, {origin["longitude_deg"]:.9f}°; ellipsoid h=0 m projection</text>')
    else:
        lines.append('<text x="80" y="200" font-size="20">No verified coordinates with valid recorder time.</text>')
    lines += ['<text x="480" y="740" text-anchor="middle" font-size="15">East [m] →</text>',
              '<text transform="translate(80 430) rotate(-90)" text-anchor="middle" font-size="15">North [m] →</text>',
              '<text x="50" y="790" font-size="12">Points show observations, not position accuracy. Gaps/quality changes are never bridged.</text>',
              '<text x="50" y="812" font-size="12">FIX 4 / FLOAT 5 / OTHER measured qualities 1–3. See summary.json and segments.csv.</text></g></svg>']
    return "\n".join(lines)


def create(input_path, output_dir, max_gap_s=1.5, motion="unknown"):
    source = Path(input_path).expanduser().resolve()
    if source.is_dir():
        source /= "gga.jsonl"
    if not source.is_file():
        raise ValueError("input must be an existing regular GGA JSONL file or recording directory")
    raw = source.read_bytes()
    points, segments, summary = analyze(raw, max_gap_s, motion)
    output = Path(output_dir).expanduser()
    output.mkdir(parents=True, exist_ok=False)
    (output / "source_gga.jsonl").write_bytes(raw)
    summary.update(source=str(source), source_sha256=hashlib.sha256(raw).hexdigest(),
                   tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   generated_utc=datetime.now(timezone.utc).isoformat())
    for name, rows, fields in (("points.csv", points, FIELDS), ("segments.csv", segments,
            ["segment_id", "quality_code", "quality_label", "group", "break_reason", "start_line", "end_line",
             "start_elapsed_s", "end_elapsed_s", "samples", "duration_s"])):
        with (output / name).open("x", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (output / "antenna_preview.svg").write_text(svg(points, summary), encoding="utf-8")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="기록 폴더 또는 gga.jsonl")
    parser.add_argument("--output-dir", required=True, type=Path, help="새 결과 폴더, 덮어쓰기 금지")
    parser.add_argument("--max-gap-s", type=float, default=1.5, help="연결 가능한 최대 수신 간격(기본 1Hz 기록용 1.5초)")
    parser.add_argument("--motion", choices=("stationary", "moving", "unknown"), default="unknown", help="사용자가 확인한 정지/이동 조건; 자동 추정 없음")
    args = parser.parse_args(argv)
    try:
        report = create(args.input, args.output_dir, args.max_gap_s, args.motion)
        print(json.dumps({"output": str(args.output_dir), "description": report["description"],
                          "observations": report["plotted_observations"], "segments": report["segments"],
                          "quality_counts": report["quality_counts"]}, ensure_ascii=False))
        return 0 if report["plotted_observations"] else 2
    except (OSError, ValueError, TypeError) as exc:
        print("오프라인 궤적 요약 실패: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
