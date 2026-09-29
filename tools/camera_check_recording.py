#!/usr/bin/env python3
"""Sample a RealSense recording (.db3 with SDK 2.58) via file playback only.

Only pyrealsense2 is external; PNG previews use the Python standard library.
SDK reference: https://realsenseai.github.io/librealsense/python_docs/
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import struct
import sys
import time
import zlib


def positive_int(value):
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("프레임 수는 양수여야 합니다.")
    return result


def timeout_value(value):
    result = float(value)
    if not math.isfinite(result) or result < 15:
        raise argparse.ArgumentTypeError("시간 제한은 15초 이상의 유한한 값이어야 합니다.")
    return result


def profile_info(profile):
    result = {"stream": str(profile.stream_type()), "index": profile.stream_index(),
              "format": str(profile.format()), "fps": profile.fps()}
    if profile.is_video_stream_profile():
        video = profile.as_video_stream_profile()
        result.update(width=video.width(), height=video.height())
    return result


def rgb_pixels(frame):
    """Copy RGB/BGR bytes from a recorded video frame, respecting row stride."""
    video = frame.as_video_frame()
    width, height = video.get_width(), video.get_height()
    pixel_format = str(frame.get_profile().format()).split(".")[-1].lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(pixel_format)
    if channels is None:
        raise ValueError("PNG 미리보기를 지원하지 않는 color 형식: " + pixel_format)
    stride = video.get_stride_in_bytes()
    raw = bytes(video.get_data())
    if width <= 0 or height <= 0 or stride < width * channels or len(raw) < stride * height:
        raise ValueError("영상 버퍼 크기와 프로필이 일치하지 않습니다.")
    rows = []
    for y in range(height):
        row = raw[y * stride:y * stride + width * channels]
        if pixel_format == "rgb8":
            rows.append(row)
        else:
            output = bytearray(width * 3)
            for x in range(width):
                source = row[x * channels:x * channels + 3]
                output[x * 3:x * 3 + 3] = source[::-1] if pixel_format.startswith("bgr") else source
            rows.append(bytes(output))
    return width, height, b"".join(rows)


def write_png(path, width, height, pixels):
    if len(pixels) != width * height * 3:
        raise ValueError("RGB 이미지 크기가 맞지 않습니다.")
    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data +
                struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff))
    rows = b"".join(b"\x00" + pixels[y * width * 3:(y + 1) * width * 3]
                    for y in range(height))
    encoded = (b"\x89PNG\r\n\x1a\n" +
               chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) +
               chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(encoded)


def check_recording(rs, *, bag, preview=None, max_frames=90, timeout=15.0,
                    monotonic=time.monotonic):
    bag = Path(bag).expanduser().absolute()
    preview = Path(preview).expanduser().absolute() if preview is not None else None
    if not bag.is_file() or bag.stat().st_size == 0:
        raise ValueError("--bag에는 비어 있지 않은 기존 RealSense 녹화 파일이 필요합니다.")
    if max_frames <= 0 or not math.isfinite(timeout) or timeout < 15:
        raise ValueError("max_frames는 양수, timeout은 15초 이상이어야 합니다.")
    if preview is not None and (preview.exists() or preview.is_symlink()):
        raise ValueError("기존 미리보기 파일은 덮어쓰지 않습니다: " + str(preview))

    report = {
        "schema_version": 1, "status": "checking",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "bag": str(bag), "bag_bytes": bag.stat().st_size,
        "source": "native_realsense_file_playback_only",
        "repeat_playback": False, "real_time": False,
        "requested_max_color_frames": max_frames, "timeout_seconds": timeout,
        "verification_scope": "sampled", "end_of_file_observed": False,
        "profiles": [], "observed_streams": {}, "color_frames_sampled": 0,
        "preview": None,
        "scope_note": "sampled는 읽은 구간만 검사합니다. complete는 파일 재생 종료까지 도달했다는 뜻이며, 모든 원본 프레임의 보존·영상 품질·센서 동기화를 보증하지 않습니다.",
    }
    pipeline = rs.pipeline()
    config = rs.config()
    # SDK resolve considers only this recorded device. There is no live fallback.
    config.enable_device_from_file(str(bag), repeat_playback=False)
    config.enable_all_streams()
    attempted = started = False
    playback = None
    begin = monotonic()
    selected_preview = None
    preview_after_two_seconds = False
    first_color_timestamp = None
    last_numbers = {}
    frames = frame = None
    try:
        # Configure non-real-time playback before stream start when supported.
        resolved = config.resolve(rs.pipeline_wrapper(pipeline))
        device = resolved.get_device()
        if not device.is_playback():
            raise RuntimeError("SDK가 파일 재생 장치를 선택하지 않았습니다. 시작하지 않습니다.")
        device.as_playback().set_real_time(False)
        attempted = True
        active = pipeline.start(config)
        started = True
        playback = active.get_device().as_playback()
        playback.set_real_time(False)
        if Path(playback.file_name()).resolve() != bag.resolve():
            raise RuntimeError("재생 중인 파일이 요청 파일과 다릅니다.")
        report["file_duration_seconds"] = playback.get_duration().total_seconds()
        report["profiles"] = [profile_info(p) for p in active.get_streams()]
        if not any(p.stream_type() == rs.stream.color for p in active.get_streams()):
            raise RuntimeError("이 파일에는 RGB/color 스트림이 없습니다.")
        deadline = begin + timeout
        while report["color_frames_sampled"] < max_frames:
            remaining = deadline - monotonic()
            if remaining <= 0:
                raise TimeoutError("지정한 시간 안에 샘플 읽기 또는 파일 종료 확인을 마치지 못했습니다.")
            received, frames = pipeline.try_wait_for_frames(max(1, min(250, math.ceil(remaining * 1000))))
            if not received:
                if playback.current_status() == rs.playback_status.stopped:
                    report["end_of_file_observed"] = True
                    report["verification_scope"] = "complete"
                    report["stop_reason"] = "end_of_file"
                    break
                continue
            for frame in frames:
                profile = frame.get_profile()
                key = str(profile.stream_type()) + ":" + str(profile.stream_index())
                number = frame.get_frame_number()
                timestamp = frame.get_timestamp()
                counters = report["observed_streams"].setdefault(key, {
                    "unique_frames": 0, "duplicate_frame_numbers": 0,
                    "first_timestamp_ms": timestamp, "last_timestamp_ms": timestamp,
                    "timestamp_domain": str(frame.get_frame_timestamp_domain()),
                })
                counters["last_timestamp_ms"] = timestamp
                if last_numbers.get(key) == number:
                    counters["duplicate_frame_numbers"] += 1
                    continue
                last_numbers[key] = number
                counters["unique_frames"] += 1
                if profile.stream_type() == rs.stream.color:
                    report["color_frames_sampled"] += 1
                    if first_color_timestamp is None:
                        first_color_timestamp = timestamp
                    after_two = timestamp - first_color_timestamp >= 2000
                    if preview is not None and (selected_preview is None or
                                                (after_two and not preview_after_two_seconds)):
                        selected_preview = (rgb_pixels(frame), number, timestamp)
                        preview_after_two_seconds = after_two
            frames = frame = None
        if report["color_frames_sampled"] == 0:
            raise RuntimeError("읽을 수 있는 RGB/color 프레임이 없습니다.")
        report.setdefault("stop_reason", "sample_limit")
        if preview is not None:
            if selected_preview is None:
                raise RuntimeError("미리보기를 생성할 RGB 프레임이 없습니다.")
            (width, height, pixels), number, timestamp = selected_preview
            write_png(preview, width, height, pixels)
            report["preview"] = {"path": str(preview), "width": width, "height": height,
                                 "frame_number": number, "timestamp_ms": timestamp,
                                 "at_least_two_seconds_after_first_color": preview_after_two_seconds}
        report["status"] = "passed"
    except KeyboardInterrupt:
        report["status"] = "interrupted"
        report["error"] = "사용자가 재생 확인을 중단했습니다."
    except Exception as exc:
        report["status"] = "failed"
        report["error"] = str(exc)
        report["error_type"] = type(exc).__name__
    finally:
        frames = frame = selected_preview = None
        if playback is not None:
            try:
                report["last_playback_position_ns"] = playback.get_position()
            except RuntimeError:
                pass
        if attempted:
            try:
                pipeline.stop()
            except RuntimeError as exc:
                if started:
                    report["status"] = "failed"
                    report["stop_error"] = str(exc)
        report["elapsed_seconds"] = max(0.0, monotonic() - begin)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="기존 RealSense 녹화 파일(.db3)의 RGB 프레임을 재생 확인합니다. 실제 카메라는 열지 않습니다.")
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--preview", type=Path, help="새 PNG 경로. 가능하면 첫 color 프레임에서 2초 이후를 선택")
    parser.add_argument("--max-frames", type=positive_int, default=90, help="최대 RGB 샘플 수, 기본 90")
    parser.add_argument("--timeout", type=timeout_value, default=15.0, help="확인 제한 시간(초), 15 이상")
    args = parser.parse_args(argv)
    try:
        import pyrealsense2 as rs
        report = check_recording(rs, bag=args.bag, preview=args.preview,
                                 max_frames=args.max_frames, timeout=args.timeout)
    except (ImportError, OSError, ValueError, RuntimeError) as exc:
        report = {"status": "failed", "source": "native_realsense_file_playback_only", "error": str(exc)}
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "passed" else 130 if report["status"] == "interrupted" else 1


if __name__ == "__main__":
    raise SystemExit(main())
