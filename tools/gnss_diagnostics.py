#!/usr/bin/env python3
"""Bounded u-blox status polls only; no CFG, reset, flash or network access.

Protocol reference: F9 HPG 1.32, UBX-22008968 R01, sections 3.5.2,
3.14.15, 3.15.13, 3.15.16, 3.15.18. Runtime firmware may differ;
no response means unavailable in this observation, not a receiver fault.
"""

import argparse
from collections import Counter
from contextlib import ExitStack
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import select
import struct
import sys
import termios
import time
import uuid

from check_devices import device_inventory
from start_gnss_recording import select_port


POLL_IDS = {"MON-VER": (10, 4), "NAV-PVT": (1, 7),
            "NAV-SAT": (1, 53), "NAV-SIG": (1, 67)}
SOURCE_URL = "https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf"
# Explicit names from the cited signal table; unmapped IDs remain numeric.
SIGNALS = {(0, 0): "GPS L1 C/A", (0, 3): "GPS L2 CL", (0, 4): "GPS L2 CM",
           (2, 0): "Galileo E1 C", (2, 1): "Galileo E1 B",
           (2, 5): "Galileo E5b I", (2, 6): "Galileo E5b Q",
           (3, 0): "BeiDou B1I D1", (3, 1): "BeiDou B1I D2",
           (3, 2): "BeiDou B2I D1", (3, 3): "BeiDou B2I D2",
           (5, 0): "QZSS L1 C/A", (5, 4): "QZSS L2 CM", (5, 5): "QZSS L2 CL",
           (6, 0): "GLONASS L1 OF", (6, 2): "GLONASS L2 OF"}


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def checksum(body):
    a = b = 0
    for value in body:
        a = (a + value) & 255
        b = (b + a) & 255
    return bytes((a, b))


def poll_bytes(name):
    """Only these four zero-payload messages can be transmitted."""
    cls, mid = POLL_IDS[name]
    body = bytes((cls, mid, 0, 0))
    return b"\xb5\x62" + body + checksum(body)


class UbxStream:
    def __init__(self):
        self.buffer = bytearray()
        self.invalid_checksums = 0
        self.valid_frames = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            start = self.buffer.find(b"\xb5\x62")
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 181 else b""
                break
            del self.buffer[:start]
            if len(self.buffer) < 6:
                break
            size = int.from_bytes(self.buffer[4:6], "little") + 8
            if len(self.buffer) < size:
                break
            frame = bytes(self.buffer[:size])
            del self.buffer[:size]
            if checksum(frame[2:-2]) != frame[-2:]:
                self.invalid_checksums += 1
                continue
            self.valid_frames += 1
            frames.append(frame)
        return frames

    def incomplete(self):
        if not self.buffer:
            return None
        return {"buffered_bytes": len(self.buffer),
                "expected_frame_bytes": int.from_bytes(self.buffer[4:6], "little") + 8
                if len(self.buffer) >= 6 else None}


def decode_frame(frame):
    key = tuple(frame[2:4])
    name = next((n for n, value in POLL_IDS.items() if value == key), None)
    if name is None:
        return None
    if len(frame) < 8 or checksum(frame[2:-2]) != frame[-2:]:
        raise ValueError("Invalid UBX checksum")
    p = frame[6:-2]
    if int.from_bytes(frame[4:6], "little") != len(p):
        raise ValueError("Invalid UBX length")
    out = {"message": name, "checksum_valid": True}
    if name == "MON-VER":
        if len(p) < 40 or (len(p) - 40) % 30:
            raise ValueError("Unexpected MON-VER length")
        txt = lambda raw: raw.split(b"\0", 1)[0].decode("ascii", errors="replace")
        out.update(sw_version=txt(p[:30]), hw_version=txt(p[30:40]),
                   extensions=[txt(p[i:i + 30]) for i in range(40, len(p), 30)])
    elif name == "NAV-PVT":
        if len(p) != 92:
            raise ValueError("Unsupported NAV-PVT length")
        flags = p[21]
        carrier = (flags >> 6) & 3
        out.update(itow_ms=int.from_bytes(p[:4], "little"), fix_type=p[20],
                   gnss_fix_ok=bool(flags & 1), differential_solution=bool(flags & 2),
                   carrier_solution_code=carrier,
                   carrier_solution={0: "none", 1: "float", 2: "fixed"}.get(carrier, "reserved"),
                   satellites_used=p[23],
                   horizontal_accuracy_estimate_m=int.from_bytes(p[40:44], "little") / 1000,
                   accuracy_note="Receiver estimate, not measured ground-truth accuracy")
    else:
        step = 12 if name == "NAV-SAT" else 16
        version = 1 if name == "NAV-SAT" else 0
        if len(p) < 8 or p[4] != version or len(p) != 8 + p[5] * step:
            raise ValueError("Unsupported satellite/signal version or length")
        rows = []
        for i in range(8, len(p), step):
            q = p[i:i + step]
            row = {"gnss_id": q[0], "satellite_id": q[1]}
            if name == "NAV-SAT":
                flags = int.from_bytes(q[8:12], "little")
                row.update(cno_dbhz=q[2], elevation_deg=struct.unpack("b", q[3:4])[0],
                           azimuth_deg=struct.unpack("<h", q[4:6])[0],
                           quality_indicator=flags & 7, used_in_solution=bool(flags & 8),
                           differential_correction_available=bool(flags & 64))
            else:
                flags = int.from_bytes(q[10:12], "little")
                row.update(signal_id=q[2], frequency_id=q[3],
                           signal_name=SIGNALS.get((q[0], q[2]), "unmapped"),
                           cno_dbhz=q[6], quality_indicator=q[7], correction_source=q[8],
                           pseudorange_used=bool(flags & 8), carrier_used=bool(flags & 16),
                           pseudorange_correction_used=bool(flags & 64),
                           carrier_correction_used=bool(flags & 128))
            rows.append(row)
        out.update(itow_ms=int.from_bytes(p[:4], "little"), version=p[4],
                   reported_count=p[5], observations=rows)
        if name == "NAV-SAT":
            out["satellites_used_count"] = sum(r["used_in_solution"] for r in rows)
        else:
            out["signal_name_counts"] = dict(Counter(r["signal_name"] for r in rows))
    return out


def open_locked(port):
    """Same flock as existing recorders; save/restore host serial attributes."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    try:
        if not os.isatty(fd):
            raise ValueError("Selected device is not a TTY")
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        original = termios.tcgetattr(fd)
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd, original
    except BaseException:
        try:
            if original is not None:
                termios.tcsetattr(fd, termios.TCSANOW, original)
        finally:
            os.close(fd)
        raise


def record(port, output_dir, duration=15.0):
    if not math.isfinite(duration) or not 2 <= duration <= 60:
        raise ValueError("Duration must be 2..60 seconds")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=False)
    source = Path(__file__).read_bytes()
    (output / "gnss_diagnostics_source.py").write_bytes(source)
    started = time.monotonic()
    metadata = {"started_utc": utc_now(), "port": port, "duration_requested_s": duration,
                "source_sha256": hashlib.sha256(source).hexdigest(),
                "protocol_reference": SOURCE_URL, "poll_allowlist": list(POLL_IDS),
                "network_used": False, "receiver_configuration_written": False,
                "receiver_firmware_uploaded": False, "host_serial_settings_restored": False,
                "status": "starting", "rx_bytes": 0, "tx_bytes": 0,
                "note": "No RTCM forwarding during this short diagnostic; RTK state may age after previous correction session.",
                "latest": {}, "message_counts": {}, "decode_errors": []}
    stream = UbxStream()
    fd = original = None
    try:
        fd, original = open_locked(port)
        metadata["status"] = "recording"
        with ExitStack() as stack:
            raw = stack.enter_context((output / "raw.bin").open("xb"))
            tx = stack.enter_context((output / "tx.jsonl").open("x"))
            parsed = stack.enter_context((output / "observations.jsonl").open("x"))
            polls = list(POLL_IDS)
            index = 0
            while time.monotonic() - started < duration:
                elapsed = time.monotonic() - started
                if index < len(polls) and elapsed >= index * 0.4:
                    name = polls[index]
                    query = poll_bytes(name)
                    tx.write(json.dumps({"utc": utc_now(), "elapsed_s": elapsed,
                                         "message": name, "intended_hex": query.hex()}) + "\n")
                    tx.flush()
                    sent = os.write(fd, query)
                    metadata["tx_bytes"] += sent
                    tx.write(json.dumps({"utc": utc_now(), "message": name,
                                         "bytes_written": sent, "written_hex": query[:sent].hex()}) + "\n")
                    tx.flush()
                    if sent != len(query):
                        raise OSError("Incomplete status query write; no retry")
                    index += 1
                ready, _, _ = select.select([fd], [], [], min(0.1, max(0, duration - elapsed)))
                if not ready:
                    continue
                data = os.read(fd, 8192)
                if not data:
                    raise OSError("Device disconnected or reached EOF")
                raw.write(data)
                metadata["rx_bytes"] += len(data)
                for frame in stream.feed(data):
                    try:
                        row = decode_frame(frame)
                    except ValueError as exc:
                        metadata["decode_errors"].append(str(exc))
                        continue
                    if row:
                        row.update(host_utc=utc_now(), elapsed_s=time.monotonic() - started)
                        parsed.write(json.dumps(row, ensure_ascii=False) + "\n")
                        name = row["message"]
                        metadata["latest"][name] = row
                        metadata["message_counts"][name] = metadata["message_counts"].get(name, 0) + 1
            metadata["status"] = "completed"
    except KeyboardInterrupt:
        metadata["status"] = "interrupted"
    except (OSError, ValueError, termios.error) as exc:
        metadata["status"] = "failed"
        metadata["error"] = str(exc)
    finally:
        if fd is not None:
            try:
                termios.tcsetattr(fd, termios.TCSANOW, original)
                metadata["host_serial_settings_restored"] = True
            except (OSError, termios.error):
                metadata["restore_warning"] = "Device unavailable during host serial restore"
            finally:
                os.close(fd)
        metadata.update(ended_utc=utc_now(), elapsed_s=time.monotonic() - started,
                        incomplete_ubx=stream.incomplete(), valid_ubx_frames=stream.valid_frames,
                        invalid_ubx_checksums=stream.invalid_checksums,
                        no_response_messages=[n for n in POLL_IDS if n not in metadata["latest"]],
                        no_response_note="No decodable response observed; not proof of hardware failure")
        (output / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
        lines = ["# GNSS 상태 조회", "", f"상태: {metadata['status']}",
                 "수신기 설정 변경 없음. 위치 좌표는 요약에 표시하지 않음.",
                 "이 조회는 보정을 전달하지 않으므로 앞선 RTK 연속 기록과 구분한다."]
        for name in POLL_IDS:
            value = metadata["latest"].get(name)
            if value is None:
                lines.append(f"- {name}: 해석 가능한 응답 없음(고장 판정 아님)")
            elif name == "MON-VER":
                lines.append(f"- {name}: {value['sw_version']} / {value['hw_version']} / {', '.join(value['extensions'])}")
            elif name == "NAV-PVT":
                lines.append(f"- {name}: carrier={value['carrier_solution']}, fix OK={value['gnss_fix_ok']}, 사용 위성={value['satellites_used']}, 수신기 수평 오차 추정={value['horizontal_accuracy_estimate_m']}m")
            elif name == "NAV-SAT":
                lines.append(f"- {name}: 보고 위성={value['reported_count']}, 사용 위성={value['satellites_used_count']}")
            else:
                lines.append(f"- {name}: 보고 신호={value['reported_count']}, 종류={value['signal_name_counts']}")
        (output / "summary.md").write_text("\n".join(lines) + "\n")
    return metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description="u-blox 수신기 상태 4종 조회만 수행합니다. 다른 GNSS 기록기를 먼저 종료하세요.")
    parser.add_argument("--duration", type=float, default=15)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[1]
    try:
        port = select_port(device_inventory())
        if "/serial/by-id/" not in port:
            raise ValueError("Stable by-id identity is unavailable; no device opened")
        output = args.output_dir or root / "data" / ("gnss_diagnostics_" + datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8])
        result = record(port, output, args.duration)
        print((output / "summary.md").read_text())
        print(f"기록: {output}")
        return 0 if result["status"] == "completed" else 1
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
