#!/usr/bin/env python3
"""Bounded identity/read-value probe for the two explicitly identified CP2102s.

Never opens GNSS, Arduino, or camera. No sensor configuration writes, motor,
scan, reset, flash, or explicit modem-line ioctls. Linux host baud is restored.
"""
import array
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import select
import shutil
import sys
import termios
import time
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
IMU_SERIAL = "0001"
LIDAR_SERIAL = "c5f4b14d0dc8fd43882b1e37a62d3c64"
TCGETS2, TCSETS2, BOTHER = 0x802C542A, 0x402C542B, 0o010000
QUERIES = ("mv", "b2", "so", "sp", "sd", "a", "g", "e", "q", "s")


def now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def identity(serial):
    matches = []
    for p in Path("/dev/serial/by-id").glob("*CP2102*"):
        real = p.resolve()
        for parent in (Path("/sys/class/tty") / real.name / "device").resolve().parents:
            if (parent / "idVendor").exists():
                values = {k: (parent / k).read_text().strip()
                          for k in ("idVendor", "idProduct", "serial", "product")}
                if (values["idVendor"], values["idProduct"], values["serial"]) == ("10c4", "ea60", serial):
                    matches.append({**values, "port": str(p), "real_port": str(real), "sysfs": str(parent)})
                break
    if len(matches) != 1:
        raise RuntimeError("Expected exactly one matching CP2102: " + serial)
    return matches[0]


class Port:
    def __init__(self, ident, baud, folder):
        self.fd = None
        self.old2 = None
        self.folder = folder
        folder.mkdir()
        self.events = (folder / "events.jsonl").open("x")
        self.raw = (folder / "raw_rx.bin").open("xb")
        self.start = time.monotonic()
        try:
            self.fd = os.open(ident["port"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.old2 = array.array("i", [0] * 64)
            fcntl.ioctl(self.fd, TCGETS2, self.old2)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            current = array.array("i", [0] * 64)
            fcntl.ioctl(self.fd, TCGETS2, current)
            current[2] = (current[2] & ~termios.CBAUD) | BOTHER
            current[9] = current[10] = baud
            fcntl.ioctl(self.fd, TCSETS2, current)
            self.log("opened", identity=ident, host_baud=baud)
        except BaseException:
            self.close()
            raise

    def log(self, kind, **values):
        self.events.write(json.dumps({"host_utc": now(), "elapsed_s": time.monotonic()-self.start,
                                     "kind": kind, **values}) + "\n")
        self.events.flush()

    def receive(self, seconds):
        end = time.monotonic() + seconds
        received = bytearray()
        while time.monotonic() < end:
            ready, _, _ = select.select([self.fd], [], [], min(0.1, max(0, end-time.monotonic())))
            if ready:
                try:
                    b = os.read(self.fd, 8192)
                except BlockingIOError:
                    continue
                if not b:
                    raise OSError("Serial device disconnected")
                received.extend(b)
                self.raw.write(b)
                self.raw.flush()
                self.log("rx", hex=b.hex())
        return bytes(received)

    def query(self, command, seconds=0.7):
        allowed = {b"vr\r\n", b"pv\r\n", b"\xa5\x50", b"\xa5\x52"}
        allowed.update((q + "\r\n").encode("ascii") for q in QUERIES)
        if command not in allowed:
            raise ValueError("Query not permitted")
        self.log("tx_attempt", hex=command.hex())
        n = os.write(self.fd, command)
        self.log("tx_completed", hex=command[:n].hex())
        if n != len(command):
            raise OSError("Partial command write; no retry")
        return self.receive(seconds)

    def close(self):
        if self.fd is not None:
            try:
                if self.old2 is not None:
                    fcntl.ioctl(self.fd, TCSETS2, self.old2)
            finally:
                os.close(self.fd)
                self.fd = None
        self.events.close()
        self.raw.close()


def lidar_response(raw, kind):
    size = {4: 20, 6: 3}[kind]
    header = b"\xa5\x5a" + size.to_bytes(4, "little") + bytes([kind])
    offset = raw.find(header)
    if offset < 0 or len(raw) < offset+7+size:
        return {"valid_descriptor_and_payload": False, "response_hex": raw.hex()}
    p = raw[offset+7:offset+7+size]
    if kind == 4:
        fields = {"model_byte": p[0], "firmware_major": p[2], "firmware_minor": p[1],
                  "hardware": p[3], "lidar_serial_hex": p[4:].hex()}
    else:
        fields = {"status": p[0], "error_code": int.from_bytes(p[1:3], "little")}
    return {"valid_descriptor_and_payload": True, **fields}


def main():
    output = ROOT / "data" / (datetime.now(ZoneInfo("Asia/Seoul")).strftime("sensor_identity_%Y%m%d_%H%M%S_") + os.urandom(4).hex())
    output.mkdir()
    source = Path(__file__).read_bytes()
    (output / "executed_probe.py").write_bytes(source)
    report = {"started_utc": now(), "source_sha256": hashlib.sha256(source).hexdigest(),
              "execution": sys.executable + " " + str(Path(__file__).resolve()) + " " + " ".join(sys.argv[1:]),
              "python": sys.version, "platform": list(os.uname()),
              "conditions": "User confirmed outdoors and vehicle motor power OFF; manual repositioning may occur; communication check only, not stationary accuracy/drift measurement; exact sensor mounting unrecorded",
              "scope": "Two CP2102s only; ASCII IMU read queries and LiDAR INFO/HEALTH; no sensor configuration, scan, motor, reset, flash or explicit DTR/RTS ioctl",
              "sources": ["https://blog.naver.com/robor21/222214954034",
                          "https://github.com/Slamtec/rplidar_ros/blob/master/launch/rplidar_a2m12.launch",
                          "https://github.com/Slamtec/rplidar_sdk/blob/master/sdk/include/sl_lidar_cmd.h",
                          "https://github.com/Slamtec/rplidar_sdk/blob/master/sdk/include/sl_lidar_protocol.h"]}
    print("OUTPUT=" + str(output), flush=True)
    devices = (("imu_candidate", IMU_SERIAL, 115200), ("lidar_candidate", LIDAR_SERIAL, 256000))
    if sys.argv[1:] in (["--imu-only"], ["--imu-power-only"]):
        devices = devices[:1]
    elif sys.argv[1:]:
        raise ValueError("Only optional --imu-only or --imu-power-only is supported")
    for role, serial, baud in devices:
        port = None
        result = {}
        try:
            ident = identity(serial)
            result["usb_identity"] = ident
            port = Port(ident, baud, output / role)
            initial = port.receive(5 if role == "imu_candidate" else 1)
            result["passive_bytes"] = len(initial)
            if role == "imu_candidate":
                vr = port.query(b"vr\r\n", 1.5).decode("ascii", "replace")
                result["version_response"] = vr
                recognized = bool(re.search(r"(?m)^\s*(?:AHRS|INTH iAHRSv1)\s*$", vr) and "H/W Ver" in vr and "S/W Ver" in vr)
                result["identified_ahrs"] = recognized
                if recognized and sys.argv[1:] == ["--imu-power-only"]:
                    result["power_samples"] = []
                    for _ in range(4):
                        sample_start = time.monotonic()
                        sample = {"host_utc": now()}
                        sample["pv_response"] = port.query(b"pv\r\n", 0.4).decode("ascii", "replace")
                        sample["s_response"] = port.query(b"s\r\n", 0.4).decode("ascii", "replace")
                        result["power_samples"].append(sample)
                        select.select([], [], [], max(0, sample_start+1-time.monotonic()))
                elif recognized:
                    result["read_queries"] = {q: port.query((q+"\r\n").encode()).decode("ascii", "replace") for q in QUERIES}
            else:
                info = lidar_response(port.query(b"\xa5\x50", 2), 4)
                result["device_info"] = info
                if info["valid_descriptor_and_payload"]:
                    result["health"] = lidar_response(port.query(b"\xa5\x52", 2), 6)
        except (OSError, ValueError, RuntimeError) as exc:
            result["error"] = str(exc)
        finally:
            if port is not None:
                try:
                    port.close()
                except OSError as exc:
                    result["close_error"] = str(exc)
            report[role] = result
            print(role + "=" + json.dumps(result), flush=True)
            report["updated_utc"] = now()
            (output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    report["ended_utc"] = now()
    (output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    main()
