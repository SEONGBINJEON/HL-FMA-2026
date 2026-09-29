#!/usr/bin/env python3
"""Bounded A2M12 SDK text capture; starts/stops the LiDAR's own scan motor.

The CLI has no arbitrary serial-port/binary override. It opens only the previously
identified CP2102 and never accesses a vehicle controller. SDK text is decoded,
rounded measurement output, NOT the original serial packets. Host receipt times
are NOT sensor acquisition timestamps or cross-sensor synchronization.
"""
import argparse
import array
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import selectors
import shutil
import signal
import stat
import subprocess
import sys
import termios
import time
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
SERIAL = "c5f4b14d0dc8fd43882b1e37a62d3c64"
BINARY = ROOT / ".setup/rplidar_sdk_built/ultra_simple"
SDK_SOURCE = ROOT / ".setup/rplidar_sdk/app/ultra_simple/main.cpp"
BUILD_PROVENANCE = ROOT / ".setup/rplidar_sdk_built/build_provenance.json"
TCGETS2, TCSETS2 = 0x802C542A, 0x402C542B
NODE = re.compile(rb"^(S)?\s+theta:\s*([0-9]+\.[0-9]+) Dist:\s*([0-9]+\.[0-9]+) Q:\s*([0-9]+)\s*$")


def now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identity(by_id=Path("/dev/serial/by-id"), tty_root=Path("/sys/class/tty")):
    matches = []
    for path in by_id.glob("*CP2102*"):
        real = path.resolve()
        if not real.name.startswith("ttyUSB"):
            continue
        device = (tty_root / real.name / "device").resolve()
        for parent in (device, *device.parents):
            if not (parent / "idVendor").exists():
                continue
            fields = {}
            for key in ("idVendor", "idProduct", "serial", "product"):
                f = parent / key
                fields[key] = f.read_text().strip() if f.exists() else ""
            if (fields["idVendor"], fields["idProduct"], fields["serial"]) == ("10c4", "ea60", SERIAL):
                matches.append({**fields, "port": str(path), "real_port": str(real), "sysfs": str(parent)})
            break
    if len(matches) != 1:
        raise RuntimeError(f"Expected exactly one LiDAR CP2102 {SERIAL}; found {len(matches)}")
    return matches[0]


class NodeSummary:
    def __init__(self, scan_file):
        self.scan_file = scan_file
        self.pending = b""
        self.count = self.positive = self.quality_positive = self.malformed = 0
        self.marker_count = self.closed_candidates = self.prefix = 0
        self.angle_min = self.distance_min = None
        self.angle_max = self.distance_max = None
        self.first_time = self.last_time = None
        self.max_gap = 0.0
        self.scan = None

    def feed(self, chunk, elapsed):
        self.pending += chunk
        while b"\n" in self.pending:
            line, self.pending = self.pending.split(b"\n", 1)
            self.line(line, elapsed)
        if len(self.pending) > 65536:
            raise RuntimeError("SDK output line exceeds 64 KiB")

    def line(self, line, elapsed):
        match = NODE.fullmatch(line)
        if not match:
            if b"theta:" in line:
                self.malformed += 1
            return
        sync, angle, distance, quality = match.groups()
        angle, distance, quality = float(angle), float(distance), int(quality)
        if not 0 <= angle <= 360 or not 0 <= quality <= 63:
            self.malformed += 1
            return
        self.count += 1
        self.positive += distance > 0
        self.quality_positive += quality > 0
        self.angle_min = angle if self.angle_min is None else min(self.angle_min, angle)
        self.angle_max = angle if self.angle_max is None else max(self.angle_max, angle)
        if distance > 0:
            self.distance_min = distance if self.distance_min is None else min(self.distance_min, distance)
            self.distance_max = distance if self.distance_max is None else max(self.distance_max, distance)
        if self.last_time is not None:
            self.max_gap = max(self.max_gap, elapsed - self.last_time)
        else:
            self.first_time = elapsed
        self.last_time = elapsed
        if sync:
            self.marker_count += 1
            if self.scan is not None:
                self.closed_candidates += 1
                self.scan_file.write(json.dumps({**self.scan, "next_marker_host_elapsed_s": elapsed}) + "\n")
            self.scan = {"candidate": self.marker_count, "marker_host_elapsed_s": elapsed, "nodes": 0, "positive_distance_nodes": 0}
        if self.scan is None:
            self.prefix += 1
        else:
            self.scan["nodes"] += 1
            self.scan["positive_distance_nodes"] += distance > 0

    def result(self):
        return {"parsed_nodes": self.count, "positive_distance_nodes": self.positive,
                "positive_quality_nodes": self.quality_positive, "malformed_measurement_lines": self.malformed,
                "angle_range_deg": [self.angle_min, self.angle_max],
                "positive_distance_range_mm": [self.distance_min, self.distance_max],
                "sync_markers": self.marker_count, "closed_marker_to_marker_candidates": self.closed_candidates,
                "nodes_before_first_marker": self.prefix, "trailing_unclosed_candidate": self.scan,
                "first_node_host_elapsed_s": self.first_time, "last_node_host_elapsed_s": self.last_time,
                "max_node_host_receipt_gap_s": self.max_gap if self.count > 1 else None,
                "unterminated_stdout_bytes": len(self.pending),
                "time_meaning": "Host pipe-read time shared by every node in that chunk; not sensor acquisition time",
                "scan_meaning": "S-marker-to-S-marker candidates in SDK angle-sorted text; not proof of complete revolutions or lossless data"}


def capture(command, folder, duration, grace=5.0, terminate_grace=1.0, stop_requested=None, pass_fds=()):
    """Process runner, also testable with a local synthetic child and no devices."""
    stop_requested = stop_requested or (lambda: False)
    start = time.monotonic()
    events = []
    offsets = {"stdout": 0, "stderr": 0}
    proc = None
    result = {"command": command, "started_utc": now(), "requested_duration_s": duration,
              "signals": events, "error": None, "forced_termination": False}
    with (folder / "stdout.bin").open("xb") as stdout, (folder / "stderr.bin").open("xb") as stderr, \
            (folder / "host_chunks.jsonl").open("x") as chunks, \
            (folder / "scan_candidates.jsonl").open("x") as scans:
        files = {"stdout": stdout, "stderr": stderr}
        summary = NodeSummary(scans)
        selector = selectors.DefaultSelector()
        try:
            proc = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, start_new_session=True, pass_fds=pass_fds)
            for name in files:
                pipe = getattr(proc, name)
                os.set_blocking(pipe.fileno(), False)
                selector.register(pipe, selectors.EVENT_READ, name)
            stage = 0
            deadline = None if duration is None else start + duration
            exited_at = None
            while True:
                current = time.monotonic()
                alive = proc.poll() is None
                if alive and ((stage == 0 and stop_requested()) or
                              (deadline is not None and current >= deadline)):
                    sig = (signal.SIGINT, signal.SIGTERM, signal.SIGKILL)[min(stage, 2)]
                    try:
                        os.killpg(proc.pid, sig)
                    except ProcessLookupError:
                        pass
                    events.append({"host_utc": now(), "elapsed_s": current-start, "signal": sig.name,
                                   "reason": "caller_stop" if stage == 0 and stop_requested() else "duration_or_grace_deadline"})
                    if stage >= 1:
                        result["forced_termination"] = True
                    stage += 1
                    deadline = current + (grace if stage == 1 else terminate_grace)
                    if stage > 3:
                        result["error"] = "Child did not reap after SIGKILL"
                        break
                if not alive:
                    if exited_at is None:
                        exited_at = current
                    if not selector.get_map() or current - exited_at > 1:
                        break
                for key, _ in selector.select(0.05):
                    data = os.read(key.fileobj.fileno(), 65536)
                    if not data:
                        selector.unregister(key.fileobj)
                        continue
                    name = key.data
                    received = time.monotonic() - start
                    files[name].write(data)
                    chunks.write(json.dumps({"stream": name, "offset": offsets[name], "bytes": len(data),
                                             "host_utc": now(), "host_elapsed_s": received}) + "\n")
                    files[name].flush()
                    chunks.flush()
                    offsets[name] += len(data)
                    if name == "stdout":
                        summary.feed(data, received)
        except BaseException as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            # Includes disk/parser/caller failures: request SDK cleanup before force.
            if proc is not None and proc.poll() is None:
                for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, terminate_grace), (signal.SIGKILL, terminate_grace)):
                    try:
                        os.killpg(proc.pid, sig)
                        events.append({"host_utc": now(), "elapsed_s": time.monotonic()-start,
                                       "signal": sig.name, "reason": "exception_or_final_cleanup"})
                        result["forced_termination"] |= sig != signal.SIGINT
                        proc.wait(timeout=wait)
                        break
                    except ProcessLookupError:
                        break
                    except subprocess.TimeoutExpired:
                        continue
            selector.close()
            if proc is not None:
                result["returncode"] = proc.poll()
                for pipe in (proc.stdout, proc.stderr):
                    if pipe:
                        pipe.close()
            result.update({"ended_utc": now(), "elapsed_s": time.monotonic()-start,
                           "stream_bytes": offsets, "measurements": summary.result()})
            result["measurements"]["startup_until_first_node_s"] = summary.first_time
            result["measurements"]["last_node_to_capture_end_s"] = (
                result["elapsed_s"] - summary.last_time if summary.last_time is not None else None)
            result["measurements"]["node_observation_span_s"] = (
                summary.last_time - summary.first_time if summary.last_time is not None else None)
    result["sdk_cleanup_path_expected"] = bool(result.get("returncode") == 0 and events
        and events[0]["signal"] == "SIGINT" and not result["forced_termination"]
        and result["measurements"]["parsed_nodes"] > 0 and not result["error"])
    result["physical_motor_stop_verified"] = False
    result["status"] = "completed" if result["sdk_cleanup_path_expected"] else "needs_review"
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    timing = parser.add_mutually_exclusive_group()
    timing.add_argument("--duration", type=float, default=60, help="total SDK process seconds, including startup (2..900)")
    timing.add_argument("--until-interrupt", action="store_true")
    parser.add_argument("--context", default="Physical conditions and sensor mounting not recorded", help="current test conditions, no credentials")
    parser.add_argument("--mount-id", default="unknown-legacy", help="physical mounting ID, never inferred from a later profile")
    parser.add_argument("--output-dir", type=Path, help="new explicit output folder; never overwrite")
    args = parser.parse_args()
    duration = None if args.until_interrupt else args.duration
    if duration is not None and not 2 <= duration <= 900:
        parser.error("duration must be 2..900 seconds")
    ident = identity()
    expected = json.loads(BUILD_PROVENANCE.read_text())["sha256"]["ultra_simple"]
    if sha256(BINARY) != expected:
        raise RuntimeError("SDK binary differs from recorded build provenance")
    stdbuf = shutil.which("stdbuf")
    if not stdbuf:
        raise RuntimeError("stdbuf is required for line-buffered SDK output")
    folder = ROOT / "data" / (datetime.now(ZoneInfo("Asia/Seoul")).strftime("lidar_%Y%m%d_%H%M%S_") + os.urandom(4).hex())
    if args.output_dir is not None: folder = args.output_dir.absolute()
    folder.mkdir(parents=True, exist_ok=False)
    report = {"mount_id": args.mount_id, "raw_frame": "sdk_raw_polar", "transform_count": 0, "started_utc": now(), "identity": ident, "host_baud": 256000, "context": args.context,
              "execution_argv": [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]],
              "python": sys.version, "platform": list(os.uname()), "status": "preparing",
              "source_sha256": sha256(Path(__file__)), "sdk_binary_sha256": expected,
              "sdk_source_sha256": sha256(SDK_SOURCE), "stdbuf_sha256": sha256(Path(stdbuf)),
              "scope": "Only specified LiDAR USB; SDK starts scan/motor. No vehicle-controller commands.",
              "limitations": "SDK text only, rounded angle/distance; not raw packets, acquisition timestamps, synchronization, accuracy or lossless verification"}
    for source, name in ((Path(__file__), "executed_lidar_record.py"), (SDK_SOURCE, "upstream_main.cpp"),
                         (BUILD_PROVENANCE, "sdk_build_provenance.json")):
        shutil.copyfile(source, folder / name)
    print("OUTPUT=" + str(folder), flush=True)
    (folder / "metadata.json").write_text(json.dumps(report, indent=2) + "\n")
    fd = None
    old2 = None
    old_handlers = {}
    stop = [False]
    try:
        fd = os.open(ident["port"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if identity() != ident or not stat.S_ISCHR(os.fstat(fd).st_mode) or os.fstat(fd).st_rdev != os.stat(ident["port"]).st_rdev:
            raise RuntimeError("USB identity changed while acquiring lock")
        snapshot = array.array("i", [0] * 64)
        fcntl.ioctl(fd, TCGETS2, snapshot)
        old2 = snapshot
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, lambda *_: stop.__setitem__(0, True))
        # Keep the advisory lock in both parent and SDK child until capture ends.
        command = [stdbuf, "-oL", "-eL", str(BINARY), "--channel", "--serial", ident["port"], "256000"]
        report["capture"] = capture(command, folder, duration, stop_requested=lambda: stop[0], pass_fds=(fd,))
        report["status"] = report["capture"]["status"]
    except (OSError, ValueError, RuntimeError) as exc:
        report["status"] = "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if fd is not None:
            try:
                if old2 is not None:
                    fcntl.ioctl(fd, TCSETS2, old2)
                    report["host_serial_attributes_restored"] = True
            except OSError as exc:
                report["host_serial_restore_error"] = str(exc)
            os.close(fd)
        for sig, previous in old_handlers.items():
            signal.signal(sig, previous)
        report["ended_utc"] = now()
        (folder / "metadata.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    return 0 if report["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
