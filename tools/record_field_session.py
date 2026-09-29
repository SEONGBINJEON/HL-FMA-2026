#!/usr/bin/env python3
"""Run existing bounded sensor recorders; never access the vehicle controller."""
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import subprocess
import sys
import time
import uuid
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ("scripts/record_field_session.sh", "scripts/practice_data.sh", "tools/record_field_session.py",
           "scripts/record_rtk.sh", "tools/start_rtk_recording.py", "tools/rtk_record.py",
           "tools/start_gnss_recording.py", "tools/check_devices.py", "tools/gnss_record.py",
           "tools/gnss_analyze.py", "tools/imu_record.py", "tools/probe_imu_lidar_once.py",
           "tools/lidar_record.py", "tools/camera_record.py")


def now():
    return datetime.now(ZoneInfo("Asia/Seoul")).isoformat(timespec="seconds")


def save(path, value):
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    temp.replace(path)


def plans(folder, duration, camera_serial=None, camera_seconds=30, skip_rtk=False,
          camera_width=640, camera_height=480, camera_fps=30, skip_lidar=False):
    py = str(ROOT / ".venv/bin/python")
    duration_args = ["--until-interrupt"] if duration is None else ["--duration", str(duration)]
    rtk_args = ["--until-interrupt"] if duration is None else [str(duration)]
    items = [
        {"name": "rtk", "duration_s": duration,
         "argv": ["bash", str(ROOT / "scripts/record_rtk.sh"), *rtk_args]},
        {"name": "imu", "duration_s": duration, "output": str(folder / "imu"),
         "argv": [py, str(ROOT / "tools/imu_record.py"), *duration_args,
                  "--output-dir", str(folder / "imu")]},
        {"name": "lidar", "duration_s": duration,
         "argv": [py, str(ROOT / "tools/lidar_record.py"), *duration_args,
                  "--context", "Field session: " + str(folder) + "; see session.json for conditions"]},
    ]
    if skip_rtk:
        items[0]["disabled"] = True
    if skip_lidar:
        items[2]["disabled"] = True
    if camera_serial:
        seconds = camera_seconds if duration is None else min(camera_seconds, duration)
        items.append({"name": "camera", "duration_s": seconds, "output": str(folder / "camera"),
                      "camera_serial": camera_serial,
                      "argv": [py, str(ROOT / "tools/camera_record.py"), "--serial", camera_serial,
                               "--output-dir", str(folder / "camera"), "--duration", str(seconds),
                               "--width", str(camera_width), "--height", str(camera_height),
                               "--fps", str(camera_fps), "--with-depth"]})
    return items


def snapshot_control_journal(require=False, settle=.15):
    """Locate a currently updating manual-control journal without opening the Mega."""
    candidates = sorted((ROOT / "data").glob("step_manual_control_*/events.jsonl"),
                        key=lambda path: path.stat().st_mtime, reverse=True)
    if not candidates:
        if require:
            raise RuntimeError("조작 프로그램 기록을 찾지 못했습니다. 먼저 수동 조작 창을 실행하세요.")
        return None
    path = candidates[0]
    first = path.stat().st_size
    time.sleep(settle)
    second = path.stat().st_size
    if second <= first:
        if require:
            raise RuntimeError("조작 기록이 갱신되지 않습니다. 수동 조작 창 연결 상태를 확인하세요.")
        return None
    return {"source": str(path.resolve()), "offset": second,
            "captured_at_kst": now()}


def attach_control_segment(folder, snapshot, report):
    """Copy the concurrent control/status rows, including encoder values, into this run."""
    if snapshot is None:
        report["control"] = {"status": "not_requested"}
        return
    source = Path(snapshot["source"])
    control = folder / "control"
    control.mkdir()
    destination = control / "events.jsonl"
    rows, statuses, mismatches, encoders = 0, 0, 0, []
    try:
        with source.open("rb") as stream:
            stream.seek(snapshot["offset"])
            payload = stream.read()
        destination.write_bytes(payload)
        for raw in payload.splitlines():
            if not raw:
                continue
            rows += 1
            try:
                item = json.loads(raw)
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if item.get("event") == "status" and isinstance(item.get("status"), dict):
                statuses += 1
                value = item["status"]
                if isinstance(value.get("encoder_count"), int):
                    encoders.append(value["encoder_count"])
                if (value.get("requested_drive") and
                        value.get("actual_drive") != value.get("requested_drive")):
                    mismatches += 1
        source_link = control / "full_controller_journal"
        source_link.symlink_to(os.path.relpath(source.parent, control), target_is_directory=True)
        summary = {"status": "completed" if statuses else "needs_review",
                   "source": str(source), "start_offset": snapshot["offset"],
                   "events": rows, "status_samples": statuses,
                   "drive_mismatch_samples": mismatches,
                   "recording_continues_on_drive_mismatch": True,
                   "encoder_samples": len(encoders),
                   "encoder_first": encoders[0] if encoders else None,
                   "encoder_last": encoders[-1] if encoders else None,
                   "encoder_min": min(encoders) if encoders else None,
                   "encoder_max": max(encoders) if encoders else None,
                   "encoder_delta": encoders[-1] - encoders[0] if encoders else None}
    except (OSError, ValueError) as exc:
        summary = {"status": "needs_review", "source": str(source),
                   "error_type": type(exc).__name__}
    save(control / "report.json", summary)
    report["control"] = summary


def preflight(item, folder):
    """Read identity only; port opening and existing locks remain in each recorder."""
    name = item["name"]
    if name == "rtk":
        from start_gnss_recording import device_inventory, select_port
        port = select_port(device_inventory())
        if not port.startswith("/dev/serial/by-id/"):
            raise RuntimeError("GNSS by-id path required")
        item["selected_by_id"] = port
    elif name == "imu":
        from probe_imu_lidar_once import identity, IMU_SERIAL
        item["selected_by_id"] = identity(IMU_SERIAL)["port"]
    elif name == "lidar":
        from lidar_record import identity
        item["selected_by_id"] = identity()["port"]
    if name == "camera":
        # A launcher-to-launcher lock; SDK also rejects already occupied streams.
        lockdir = ROOT / "data/.field_locks"
        lockdir.mkdir(exist_ok=True, mode=0o700)
        fd = os.open(lockdir / ("camera_" + item["camera_serial"] + ".lock"),
                     os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(fd)
            raise
        return fd
    return None


def evidence(item, folder):
    name = item["name"]
    target = Path(item["output"]) if item.get("output") else None
    if target is None:
        with (folder / (name + ".log")).open("rb") as stream:
            lines = stream.read(65536).decode("utf-8", "replace").splitlines()
        prefix = "저장 폴더: " if name == "rtk" else "OUTPUT="
        found = [line[len(prefix):] for line in lines if line.startswith(prefix)]
        if len(found) == 1:
            candidate = Path(found[0]).resolve()
            # Only link the unique data folder announced by this child.
            if candidate.parent == (ROOT / "data").resolve() and candidate.name.startswith(name + "_"):
                target = candidate
    if target is None or not target.is_dir():
        return {"status": "missing_output", "has_samples": False}
    link = folder / name
    if target != link and not link.exists() and not link.is_symlink():
        link.symlink_to(os.path.relpath(target, folder), target_is_directory=True)
    item["output"] = str(target)
    path = target / ("report.json" if name == "imu" else "metadata.json")
    try:
        report = json.loads(path.read_text())
        result = {"status": report.get("status"), "report": str(path)}
        if name == "rtk":
            result.update(gga_records=report.get("gga_records", 0),
                          rtcm_frames_written=report.get("rtcm_frames_written", 0),
                          rtk_fixed_observed=report.get("rtk_fixed_observed", False))
            result["has_samples"] = result["gga_records"] > 0 and result["rtcm_frames_written"] > 0
        elif name == "imu":
            result["valid_pairs"] = report.get("summary", {}).get("valid_pairs", 0)
            result["has_samples"] = result["valid_pairs"] > 0
        elif name == "lidar":
            result["positive_distance_nodes"] = report.get("capture", {}).get("measurements", {}).get("positive_distance_nodes", 0)
            result["has_samples"] = result["positive_distance_nodes"] > 0
        else:
            streams = report.get("observed_streams", {})
            result["bag_bytes"] = report.get("bag_bytes", 0)
            result["has_samples"] = (result["bag_bytes"] > 0 and len(streams) == 2
                                     and all(v.get("unique_frames", 0) > 0 for v in streams.values()))
        return result
    except (OSError, ValueError, TypeError, AttributeError):
        return {"status": "missing_or_invalid_report", "has_samples": False}


def run_session(items, folder, report, stop, prepare=preflight, startup_margin=25, grace=12):
    """Isolate child failures; SIGINT first so existing finally blocks run."""
    active = []
    report["sensors"] = items
    path = folder / "session.json"
    try:
        for item in items:
            item.update(status="not_started", signals=[])
            if item.get("disabled"):
                item["status"] = "skipped"
                print(item["name"] + ": 요청에 따라 건너뜀", flush=True)
                continue
            if stop[0]:
                continue
            stream, lock = None, None
            try:
                lock = prepare(item, folder)
                stream = (folder / (item["name"] + ".log")).open("xb", buffering=0)
                proc = subprocess.Popen(item["argv"], cwd=ROOT, stdin=subprocess.DEVNULL,
                                        stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                item.update(status="running", pid=proc.pid, started_kst=now())
                deadline = None if item["duration_s"] is None else time.monotonic() + item["duration_s"] + startup_margin
                active.append({"item": item, "proc": proc, "stream": stream, "lock": lock,
                               "deadline": deadline, "stage": 0})
                print(item["name"] + ": 기록 시작", flush=True)
            except Exception as exc:
                item.update(status="failed_to_start", error_type=type(exc).__name__)
                if stream:
                    stream.close()
                if lock is not None:
                    os.close(lock)
                print(item["name"] + ": 시작 실패. 다른 센서는 계속 기록합니다.", flush=True)
            save(path, report)
        while active:
            for child in active[:]:
                item, proc = child["item"], child["proc"]
                code = proc.poll()
                if code is not None:
                    child["stream"].close()
                    if child["lock"] is not None:
                        os.close(child["lock"])
                    active.remove(child)
                    item.update(returncode=code, ended_kst=now(), evidence=evidence(item, folder))
                    good = code == 0 and item["evidence"]["status"] == "completed" and item["evidence"]["has_samples"]
                    operator_stop = (stop[0] and item["signals"] and
                        item["signals"][0].get("reason") == "operator_stop" and
                        code in (0, 130) and item["evidence"]["status"] in ("completed", "interrupted") and
                        item["evidence"]["has_samples"])
                    item["status"] = "completed_by_operator" if operator_stop else (
                        "completed" if good and not item["signals"] else "needs_review")
                    print(item["name"] + ": " + ("기록 완료" if item["status"] in ("completed", "completed_by_operator") else "확인 필요"), flush=True)
                    save(path, report)
                elif (stop[0] and child["stage"] == 0) or (
                        child["deadline"] is not None and time.monotonic() >= child["deadline"]):
                    sig = (signal.SIGINT, signal.SIGTERM, signal.SIGKILL)[min(child["stage"], 2)]
                    try:
                        # Only the process group created above, never unrelated apps.
                        os.killpg(proc.pid, sig)
                    except ProcessLookupError:
                        pass
                    item["signals"].append({"signal": sig.name, "at_kst": now(),
                                            "reason": "operator_stop" if stop[0] else "overrun"})
                    child["stage"] += 1
                    child["deadline"] = time.monotonic() + (grace if sig == signal.SIGINT else 2)
                    save(path, report)
            if active:
                time.sleep(.1)
    finally:
        # Also clean up if metadata writing or orchestration itself fails.
        for child in active:
            try:
                os.killpg(child["proc"].pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for child in active:
            for sig, timeout in ((None, grace), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
                try:
                    if sig is not None:
                        os.killpg(child["proc"].pid, sig)
                    child["proc"].wait(timeout=timeout)
                    break
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    continue
            child["stream"].close()
            if child["lock"] is not None:
                os.close(child["lock"])
    report["ended_kst"] = now()
    complete = all(i["status"] in ("completed", "completed_by_operator", "skipped") for i in items)
    report["status"] = "completed_by_operator" if stop[0] and complete else ("completed" if complete else "needs_review")
    save(path, report)
    return 0 if complete else 1


def main():
    parser = argparse.ArgumentParser(description="기존 GNSS RTK·IMU·라이다 기록기를 함께 실행합니다. 차량 제어 없음.")
    timing = parser.add_mutually_exclusive_group()
    timing.add_argument("--duration", type=float, default=180, help="센서 기록 시간: 2~900초, 기본180")
    timing.add_argument("--until-interrupt", action="store_true", help="Ctrl+C를 누를 때까지 시간 제한 없이 기록")
    parser.add_argument("--camera-serial", help="지정할 때만 D455 RGB/depth 추가; 기본 꺼짐")
    parser.add_argument("--camera-seconds", type=float, default=30, help="카메라 2~30초, 전체 기록 시간 이내")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--context", default="장소·날씨·차량 이동 여부 미기록")
    parser.add_argument("--mounting", default="이번 세션 센서 위치·방향·장착 치수 미기록")
    parser.add_argument("--skip-rtk", action="store_true", help="이미 별도 RTK 기록 중이면 GNSS를 건너뜀")
    parser.add_argument("--skip-lidar", action="store_true", help="LiDAR가 연결되지 않은 도로 기록에서는 건너뜀")
    parser.add_argument("--dry-run", action="store_true", help="장치·계정·네트워크 접근 없이 실행 계획만 출력")
    parser.add_argument("--require-control-journal", action="store_true",
                        help="현재 조작 창의 엔코더·명령·상태 기록이 없으면 시작 거부")
    parser.add_argument("--practice-mode", choices=("check", "first-lap", "route-lap"))
    parser.add_argument("--auto-lap", action="store_true", help="기존 현장 기록 다음 회차 번호 자동 부여")
    args = parser.parse_args()
    duration = None if args.until_interrupt else args.duration
    if duration is not None and (not math.isfinite(duration) or not 2 <= duration <= 900):
        parser.error("--duration은 2~900초여야 합니다.")
    if not math.isfinite(args.camera_seconds) or not 2 <= args.camera_seconds or (
            args.camera_serial and duration is not None and args.camera_seconds > duration):
        parser.error("--camera-seconds는 2초 이상이며 전체 기록 시간을 넘을 수 없습니다.")
    if min(args.camera_width, args.camera_height, args.camera_fps) <= 0:
        parser.error("카메라 너비·높이·FPS는 양수여야 합니다.")
    if args.camera_serial and not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", args.camera_serial):
        parser.error("카메라 일련번호 형식을 확인하세요.")
    folder = ROOT / "data" / (datetime.now(ZoneInfo("Asia/Seoul")).strftime("field_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
    items = plans(folder, duration, args.camera_serial, args.camera_seconds, args.skip_rtk,
                  args.camera_width, args.camera_height, args.camera_fps, args.skip_lidar)
    if args.dry_run:
        print(json.dumps({"dry_run": True, "folder": str(folder), "sensors": items}, ensure_ascii=False, indent=2))
        return 0
    os.umask(0o077)
    try:
        control_snapshot = snapshot_control_journal(args.require_control_journal)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "code").mkdir()
    hashes = {}
    for relative in SOURCES:
        source = ROOT / relative
        dest = folder / "code" / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, dest)
        hashes[relative] = hashlib.sha256(source.read_bytes()).hexdigest()
    lap_number = None
    if args.auto_lap:
        previous = []
        for candidate in (ROOT / "data").glob("field_*/session.json"):
            try:
                value = json.loads(candidate.read_text()).get("lap_number")
                if isinstance(value, int):
                    previous.append(value)
            except (OSError, ValueError):
                pass
        lap_number = max(previous, default=0) + 1
    report = {"started_kst": now(), "status": "recording", "context": args.context, "mounting": args.mounting,
              "practice_mode": args.practice_mode, "lap_number": lap_number,
              "execution": [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]],
              "platform": platform.platform(), "python": sys.version, "source_sha256": hashes,
              "ntrip_config": "config/ntrip.local.json — path only; content not copied or hashed",
              "camera_enabled": bool(args.camera_serial),
              "limitations": ["Concurrent launch, not sensor timestamp synchronization",
                              "Completion means recorded samples; not accuracy, lossless capture, or driving validation",
                              "RTK fixed observation is separate from recorder completion",
                              "LiDAR is SDK rounded text, not raw serial packets",
                              "IMU yaw is not calibrated north heading",
                              "If forced termination was needed, inspect SDK cleanup and camera replay"]}
    stop = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    print("현장 세션: " + str(folder) + "\nCtrl+C로 전체 기록을 마무리합니다.", flush=True)
    code = run_session(items, folder, report, stop)
    attach_control_segment(folder, control_snapshot, report)
    if args.require_control_journal and report["control"]["status"] != "completed":
        report["status"] = "needs_review"
        code = 1 if code == 0 else code
    save(folder / "session.json", report)
    print("세션 결과: " + report["status"] + " / " + str(folder / "session.json"), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
