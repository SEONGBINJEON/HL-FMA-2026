#!/usr/bin/env python3
"""Record an explicitly selected RealSense D455; no vehicle control or GUI.

SDK APIs: https://realsenseai.github.io/librealsense/python_docs/
The pinned SDK 2.58 records a single .db3 file in ROS 2 rosbag2 storage format.
Reference: https://github.com/realsenseai/librealsense/blob/master/doc/record-and-playback.md
"""

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time


class RecordingError(RuntimeError):
    pass


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("녹화 시간은 0보다 큰 유한한 초 단위 값이어야 합니다.")
    return number


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("0보다 큰 정수를 입력하세요.")
    return number


def device_info(device, rs):
    result = {}
    for key in ("name", "serial_number", "firmware_version", "usb_type_descriptor"):
        enum = getattr(rs.camera_info, key)
        result[key] = device.get_info(enum) if device.supports(enum) else None
    result["eligible_d455"] = (result["name"] or "").split()[-1:] == ["D455"]
    return result


def select_device(context, rs, serial):
    for device in context.query_devices():
        info = device_info(device, rs)
        if info["serial_number"] == serial:
            if not info["eligible_d455"]:
                raise RecordingError(f"선택한 장치는 D455가 아닙니다: {info['name']}")
            return device, info
    raise RecordingError("해당 일련번호의 D455를 찾지 못했습니다. --list-devices와 USB 연결·권한을 확인하세요.")


def profile_info(profile):
    video = profile.as_video_stream_profile()
    intrinsics = video.get_intrinsics()
    return {
        "stream": str(profile.stream_type()), "index": profile.stream_index(),
        "format": str(profile.format()), "fps": profile.fps(),
        "width": video.width(), "height": video.height(),
        "intrinsics": {
            "width": intrinsics.width, "height": intrinsics.height,
            "fx": intrinsics.fx, "fy": intrinsics.fy,
            "ppx": intrinsics.ppx, "ppy": intrinsics.ppy,
            "model": str(intrinsics.model), "coeffs": list(intrinsics.coeffs),
        },
    }


def available_color_profiles(device, rs):
    profiles = set()
    for sensor in device.query_sensors():
        for profile in sensor.get_stream_profiles():
            if profile.stream_type() == rs.stream.color and profile.is_video_stream_profile():
                video = profile.as_video_stream_profile()
                profiles.add(f"{video.width()}x{video.height()}@{video.fps()} {profile.format()}")
    return sorted(profiles)


def write_metadata(path, metadata):
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def record(rs, *, serial, output_dir, duration=60.0, width=640, height=480,
           fps=30, with_depth=False, max_frame_gap=2.0,
           monotonic=time.monotonic, wall_ns=time.time_ns):
    """Return a session summary; injectable clocks/SDK allow hardware-free tests."""
    if not math.isfinite(duration) or duration <= 0:
        raise RecordingError("녹화 시간은 0보다 큰 유한한 값이어야 합니다.")
    if fps <= 0 or not math.isfinite(max_frame_gap) or max_frame_gap < 2.0 / fps:
        raise RecordingError("--max-frame-gap은 유한한 값이며 최소 두 프레임 주기(2 / fps) 이상이어야 합니다.")
    output = Path(output_dir).expanduser().absolute()
    if output.exists() or output.is_symlink():
        raise RecordingError(f"기존 폴더/파일을 덮어쓰지 않습니다. 새 경로를 선택하세요: {output}")
    context = rs.context()
    device, info = select_device(context, rs, serial)
    pipeline = rs.pipeline(context)
    config = rs.config()
    config.enable_device(serial)
    config.disable_all_streams()
    config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
    if with_depth:
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    if not config.can_resolve(rs.pipeline_wrapper(pipeline)):
        choices = ", ".join(available_color_profiles(device, rs))
        raise RecordingError(
            f"요청한 RGB {width}x{height}@{fps} (depth={with_depth}) 조합을 열 수 없습니다. "
            f"USB3 연결·다른 카메라 프로그램을 확인하세요. 사용 가능한 RGB: {choices}")
    output.mkdir(parents=True, exist_ok=False)
    bag = output / "camera.db3"
    metadata_path = output / "metadata.json"
    try:
        sdk_version = importlib.metadata.version("pyrealsense2")
    except importlib.metadata.PackageNotFoundError:
        sdk_version = "unavailable"
    metadata = {
        "schema_version": 1, "status": "starting", "device": info,
        "sdk_package_version": sdk_version, "bag_file": bag.name,
        "recording_format": "realsense_sdk_rosbag2_db3",
        "requested": {"duration_seconds": duration, "width": width, "height": height,
                      "fps": fps, "with_depth": with_depth, "color_format": "rgb8",
                      "max_frame_gap_seconds": max_frame_gap},
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "profiles": [], "observed_streams": {},
        "counts_note": "프레임 수·번호 간격은 이 프로그램이 관측한 값입니다. bag 내부 전체 프레임 수나 실제 손실량을 보증하지 않습니다.",
        "timestamp_note": "device_timestamp_ms는 timestamp_domain 기준입니다. host_unix_ns는 수신 시각이며 센서 동기화를 보증하지 않습니다.",
    }
    started = False
    start_attempted = False
    begin = None
    failure = None
    last_numbers = {}
    last_seen = {}
    try:
        write_metadata(metadata_path, metadata)
        config.enable_record_to_file(str(bag))
        start_attempted = True
        active = pipeline.start(config)
        started = True
        begin = monotonic()
        actual = device_info(active.get_device(), rs)
        if actual["serial_number"] != serial or not actual["eligible_d455"]:
            raise RecordingError("SDK가 요청과 다른 장치를 열었습니다. 녹화를 중단합니다.")
        metadata["device"] = actual
        active_profiles = list(active.get_streams())
        metadata["profiles"] = [profile_info(p) for p in active_profiles]
        expected = {rs.stream.color: rs.format.rgb8}
        if with_depth:
            expected[rs.stream.depth] = rs.format.z16
        if len(active_profiles) != len(expected):
            raise RecordingError("SDK가 요청과 다른 수의 스트림을 열었습니다.")
        for profile in active_profiles:
            video = profile.as_video_stream_profile()
            if (expected.get(profile.stream_type()) != profile.format() or
                    (video.width(), video.height(), profile.fps()) != (width, height, fps)):
                raise RecordingError("SDK가 요청과 다른 영상 형식을 열었습니다.")
        if with_depth:
            metadata["depth_scale_meters"] = active.get_device().first_depth_sensor().get_depth_scale()
            color = active.get_stream(rs.stream.color)
            depth = active.get_stream(rs.stream.depth)
            extrinsics = depth.get_extrinsics_to(color)
            metadata["depth_to_color_extrinsics"] = {
                "rotation": list(extrinsics.rotation), "translation_meters": list(extrinsics.translation)}
        metadata["status"] = "recording"
        for stream in expected:
            metadata["observed_streams"][str(stream)] = {
                "unique_frames": 0, "duplicate_frame_numbers": 0,
                "frame_number_gaps": 0, "frame_number_resets": 0,
                "last_seen_elapsed_seconds": None, "max_observed_gap_seconds": 0.0,
            }
        write_metadata(metadata_path, metadata)
        with (output / "frames.jsonl").open("x", encoding="utf-8", buffering=1) as log:
            while True:
                elapsed = monotonic() - begin
                if elapsed >= duration:
                    break
                timeout_ms = max(1, min(250, math.ceil((duration - elapsed) * 1000)))
                received, frames = pipeline.try_wait_for_frames(timeout_ms)
                host_stamp = wall_ns()
                host_elapsed = monotonic() - begin
                for frame in frames if received else ():
                    stream = str(frame.get_profile().stream_type())
                    number = frame.get_frame_number()
                    counters = metadata["observed_streams"][stream]
                    previous = last_numbers.get(stream)
                    if previous == number:
                        counters["duplicate_frame_numbers"] += 1
                    else:
                        counters["unique_frames"] += 1
                        counters["max_observed_gap_seconds"] = max(
                            counters["max_observed_gap_seconds"], host_elapsed - last_seen.get(stream, 0.0))
                        last_seen[stream] = host_elapsed
                        counters["last_seen_elapsed_seconds"] = host_elapsed
                        if previous is not None:
                            if number > previous:
                                counters["frame_number_gaps"] += max(0, number - previous - 1)
                            else:
                                counters["frame_number_resets"] += 1
                    last_numbers[stream] = number
                    log.write(json.dumps({
                        "stream": stream, "frame_number": number,
                        "device_timestamp_ms": frame.get_timestamp(),
                        "timestamp_domain": str(frame.get_frame_timestamp_domain()),
                        "host_unix_ns": host_stamp, "host_elapsed_seconds": host_elapsed,
                    }) + "\n")
                del frames
                for stream, counters in metadata["observed_streams"].items():
                    gap = host_elapsed - last_seen.get(stream, 0.0)
                    counters["max_observed_gap_seconds"] = max(counters["max_observed_gap_seconds"], gap)
                    if counters["max_observed_gap_seconds"] >= max_frame_gap:
                        metadata["stalled_stream"] = stream
                        raise RecordingError(f"영상 수신 중단: {stream}의 새 프레임 수신 간격이 "
                                             f"{counters['max_observed_gap_seconds']:.2f}초에 도달했습니다.")
        if any(metadata["observed_streams"].get(str(stream), {}).get("unique_frames", 0) == 0
               for stream in expected):
            raise RecordingError("요청한 스트림의 영상 프레임을 받지 못했습니다. USB3·전원·권한을 확인하세요.")
        metadata["status"] = "completed"
    except KeyboardInterrupt:
        metadata["status"] = "interrupted"
    except Exception as exc:
        failure = exc
        metadata["status"] = "failed"
        metadata["error"] = str(exc)
    finally:
        # Release the last SDK frame references before stopping its stream.
        frames = frame = None
        if start_attempted:
            try:
                pipeline.stop()
            except Exception as exc:
                if started:
                    metadata["stop_error"] = str(exc)
                    metadata["status"] = "failed"
                    failure = failure or exc
        metadata["elapsed_seconds"] = max(0.0, monotonic() - begin) if begin is not None else 0.0
        for counters in metadata["observed_streams"].values():
            counters["effective_fps"] = (counters["unique_frames"] / metadata["elapsed_seconds"]
                                         if metadata["elapsed_seconds"] else 0.0)
        metadata["ended_utc"] = datetime.now(timezone.utc).isoformat()
        metadata["bag_bytes"] = bag.stat().st_size if bag.exists() else 0
        if metadata["status"] == "completed" and metadata["bag_bytes"] == 0:
            failure = RecordingError("camera.db3 파일이 없거나 비어 있습니다.")
            metadata["status"] = "failed"
            metadata["error"] = str(failure)
        write_metadata(metadata_path, metadata)
    if failure:
        raise RecordingError(f"녹화 실패: {failure}. 남은 기록과 오류 정보: {output}") from failure
    return metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description="명시한 RealSense D455를 유한 시간 동안 기록합니다. 차량 제어 기능은 없습니다.")
    parser.add_argument("--list-devices", action="store_true", help="SDK가 찾은 장치와 D455 여부 출력")
    parser.add_argument("--serial", help="--list-devices에서 확인한 D455 일련번호 (녹화 시 필수)")
    parser.add_argument("--output-dir", type=Path, help="새 녹화 폴더. 이미 존재하면 거절")
    parser.add_argument("--duration", type=positive_seconds, default=60.0, help="녹화 초, 기본 60. Ctrl+C로 조기 종료")
    parser.add_argument("--width", type=positive_int, default=640)
    parser.add_argument("--height", type=positive_int, default=480)
    parser.add_argument("--fps", type=positive_int, default=30)
    parser.add_argument("--with-depth", action="store_true", help="같은 해상도/FPS의 depth도 기록 (기본 RGB만)")
    parser.add_argument("--max-frame-gap", type=positive_seconds, default=2.0,
                        help="스트림별 새 프레임 수신 중단 허용 초(기본 2). 최소 2/fps 이상")
    args = parser.parse_args(argv)
    if not args.list_devices and (not args.serial or args.output_dir is None):
        parser.error("녹화에는 --serial 및 --output-dir이 필요합니다. 먼저 --list-devices를 실행하세요.")
    if args.max_frame_gap < 2.0 / args.fps:
        parser.error("--max-frame-gap은 두 프레임 주기(2 / fps) 이상이어야 합니다.")
    try:
        import pyrealsense2 as rs
    except (ImportError, OSError) as exc:
        print("RealSense SDK를 불러올 수 없습니다. 가상환경에 requirements-camera.txt를 설치한 뒤 다시 실행하세요.\n"
              f"상세: {exc}", file=sys.stderr)
        return 2
    try:
        if args.list_devices:
            devices = [device_info(d, rs) for d in rs.context().query_devices()]
            print(json.dumps(devices, indent=2, ensure_ascii=False))
            if not devices:
                print("RealSense 장치가 없습니다. D455 USB3 케이블·전원·접근 권한을 확인하세요.", file=sys.stderr)
                return 1
            return 0
        summary = record(rs, serial=args.serial, output_dir=args.output_dir,
                         duration=args.duration, width=args.width, height=args.height,
                         fps=args.fps, with_depth=args.with_depth, max_frame_gap=args.max_frame_gap)
        print(json.dumps({"status": summary["status"], "output_dir": str(args.output_dir.absolute()),
                          "elapsed_seconds": summary["elapsed_seconds"],
                          "observed_streams": summary["observed_streams"]}, indent=2, ensure_ascii=False))
        return 130 if summary["status"] == "interrupted" else 0
    except (RecordingError, RuntimeError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
