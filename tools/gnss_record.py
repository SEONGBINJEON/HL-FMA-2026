#!/usr/bin/env python3
"""Receive-only Linux GNSS recorder; standard library only, no device commands.

The output directory must not already exist. raw.bin preserves every received
byte. gga.jsonl contains checksum-verified NMEA GGA observations. Receiver UTC
in GGA has no date, so host UTC and receiver time-of-day are kept separate.
"""

import argparse
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import select
import sys
import termios
import time


QUALITY = {
    0: "invalid", 1: "gps_fix", 2: "dgps_fix", 3: "pps_fix",
    4: "rtk_fixed", 5: "rtk_float", 6: "estimated",
    7: "manual", 8: "simulation",
}


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def checksum_ok(sentence):
    """Check the exact NMEA payload; never treat a missing checksum as valid."""
    sentence = sentence.rstrip(b"\r\n")
    if not sentence.startswith(b"$") or sentence.count(b"*") != 1:
        return False
    payload, expected = sentence[1:].split(b"*")
    if len(expected) != 2 or any(c not in b"0123456789abcdefABCDEF" for c in expected):
        return False
    value = 0
    for byte in payload:
        value ^= byte
    return value == int(expected, 16)


def optional_float(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError):
        return None


def optional_int(value):
    try:
        return int(value) if value else None
    except (ValueError, TypeError):
        return None


def coordinate(value, hemisphere, latitude):
    """Convert ddmm.mmmm/dddmm.mmmm without manufacturing missing positions."""
    number = optional_float(value)
    allowed = ("N", "S") if latitude else ("E", "W")
    if number is None or number < 0 or hemisphere not in allowed:
        return None
    degrees = int(number // 100)
    minutes = number - degrees * 100
    limit = 90 if latitude else 180
    if minutes >= 60 or degrees > limit or (degrees == limit and minutes != 0):
        return None
    result = degrees + minutes / 60
    return -result if hemisphere in ("S", "W") else result


def parse_gga(sentence, host_utc=None, elapsed_s=None):
    if not checksum_ok(sentence):
        return None
    try:
        text = sentence.rstrip(b"\r\n").decode("ascii")
    except UnicodeDecodeError:
        return None
    fields = text[1:text.index("*")].split(",")
    if len(fields) < 15 or len(fields[0]) != 5 or fields[0][2:] != "GGA":
        return None
    quality = optional_int(fields[6])
    status = QUALITY.get(quality, "unknown")
    # RTK state comes solely from the receiver's GGA quality field. DGPS(2),
    # satellite count and HDOP do not establish either RTK fixed or RTK float.
    rtk_state = {4: "fixed", 5: "float"}.get(quality, "unknown")
    return {
        "host_utc": host_utc or utc_now(),
        "elapsed_s": elapsed_s,
        "talker": fields[0][:2],
        "receiver_utc_time": fields[1] or None,
        "latitude_deg": coordinate(fields[2], fields[3], True),
        "longitude_deg": coordinate(fields[4], fields[5], False),
        "quality_code": quality,
        "quality_raw": fields[6],
        "fix_status": status,
        "rtk_state": rtk_state,
        "satellites": optional_int(fields[7]),
        "hdop": optional_float(fields[8]),
        "altitude_m": optional_float(fields[9]) if fields[10] == "M" else None,
        "geoid_separation_m": optional_float(fields[11]) if fields[12] == "M" else None,
        "differential_age_s": optional_float(fields[13]),
        "reference_station_id": fields[14] or None,
        "checksum_valid": True,
        "sentence": text,
    }


class MixedNmeaStream:
    """Incremental NMEA extraction; validated UBX frames are skipped whole.

    This avoids treating text-like bytes inside a binary UBX payload as NMEA.
    Corrupt UBX frames are also skipped by their declared length: their binary
    payload must never be promoted to trusted NMEA. A truncated or implausibly
    long frame can therefore block parsing until the session ends; this is
    reported explicitly and all unfinished data remains available in raw.bin.
    """

    def __init__(self):
        self.buffer = bytearray()
        self.invalid_nmea_checksums = 0
        self.ubx_frames = 0
        self.invalid_ubx_frames = 0

    def incomplete_ubx(self):
        if not self.buffer.startswith(b"\xb5\x62"):
            return None
        expected = None
        if len(self.buffer) >= 6:
            expected = (self.buffer[4] | (self.buffer[5] << 8)) + 8
        return {"buffered_bytes": len(self.buffer), "expected_frame_bytes": expected}

    def feed(self, data):
        self.buffer.extend(data)
        sentences = []
        while self.buffer:
            if self.buffer[0] == 0xB5:
                if len(self.buffer) < 2:
                    break
                if self.buffer[1] == 0x62:
                    if len(self.buffer) < 6:
                        break
                    length = self.buffer[4] | (self.buffer[5] << 8)
                    frame_size = length + 8
                    if len(self.buffer) < frame_size:
                        break
                    a = b = 0
                    for byte in self.buffer[2:frame_size - 2]:
                        a = (a + byte) & 255
                        b = (b + a) & 255
                    if self.buffer[frame_size - 2:frame_size] == bytes((a, b)):
                        del self.buffer[:frame_size]
                        self.ubx_frames += 1
                        continue
                    self.invalid_ubx_frames += 1
                    # Even a checksum-invalid payload may contain a complete,
                    # checksum-valid GGA string. Do not parse that payload as
                    # text: skip the bounded declared frame (at most 65543
                    # bytes) and only resume parsing at the next boundary.
                    del self.buffer[:frame_size]
                    continue
                del self.buffer[0]
                continue
            if self.buffer[0] != ord("$"):
                del self.buffer[0]
                continue
            newline = self.buffer.find(b"\n")
            next_start = self.buffer.find(b"$", 1)
            binary_start = self.buffer.find(b"\xb5\x62", 1)
            restart = min((p for p in (next_start, binary_start) if p >= 0), default=-1)
            if restart >= 0 and (newline < 0 or restart < newline):
                del self.buffer[:restart]
                continue
            if newline < 0:
                if len(self.buffer) > 1024:
                    del self.buffer[0]
                    continue
                break
            sentence = bytes(self.buffer[:newline + 1])
            del self.buffer[:newline + 1]
            if checksum_ok(sentence):
                sentences.append(sentence)
            else:
                self.invalid_nmea_checksums += 1
        return sentences


def open_serial(port, baud):
    speed = getattr(termios, "B" + str(baud), None)
    if speed is None:
        raise ValueError(f"Unsupported baud rate on this system: {baud}")
    # O_RDONLY is deliberate: there is no command, configuration or RTCM
    # write path. termios only configures the host serial line.
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    try:
        if not os.isatty(fd):
            raise ValueError(f"Not a serial/TTY device: {port}")
        # Match pyserial's exclusive=True advisory lock. Acquire before changing
        # host termios settings so another GNSS/RTK reader is left untouched.
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise BlockingIOError(exc.errno,
                                  f"다른 GNSS/RTK 프로그램이 사용 중인 포트입니다: {port}") from exc
        original = termios.tcgetattr(fd)
        settings = termios.tcgetattr(fd)
        settings[0] = 0
        settings[1] = 0
        settings[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        settings[3] = 0
        settings[4] = settings[5] = speed
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        return fd, original
    except BaseException:
        if original is not None:
            try:
                termios.tcsetattr(fd, termios.TCSANOW, original)
            except (OSError, termios.error):
                pass
        os.close(fd)
        raise


def record(port, baud, duration, output_dir):
    output = Path(output_dir)
    if output.exists():
        raise FileExistsError(f"Output directory already exists; choose a new directory: {output}")
    fd, original = open_serial(port, baud)
    raw = parsed = meta_file = None
    metadata = None
    started = time.monotonic()
    stream = MixedNmeaStream()
    result = 0
    try:
        output.mkdir(parents=True, exist_ok=False)
        raw = (output / "raw.bin").open("xb")
        parsed = (output / "gga.jsonl").open("x", encoding="utf-8")
        meta_file = (output / "metadata.json").open("x", encoding="utf-8")
        metadata = {
            "schema_version": 1, "port": str(port), "baud": baud,
            "started_utc": utc_now(), "requested_duration_s": duration,
            "read_only": True, "device_commands_sent": False,
            "files": {"raw": "raw.bin", "gga": "gga.jsonl"},
            "status": "recording", "bytes_received": 0,
            "valid_nmea_sentences": 0, "gga_records": 0,
            "quality_counts": {},
        }
        json.dump(metadata, meta_file, ensure_ascii=False, indent=2)
        meta_file.flush()
        print(f"Recording receive-only GNSS data to {output.resolve()}", flush=True)
        while time.monotonic() - started < duration:
            remaining = duration - (time.monotonic() - started)
            ready, _, _ = select.select([fd], [], [], max(0, min(0.2, remaining)))
            if not ready:
                continue
            try:
                data = os.read(fd, 65536)
            except BlockingIOError:
                continue
            if not data:
                raise OSError("Serial device disconnected or reached end of stream")
            host_utc = utc_now()
            elapsed = round(time.monotonic() - started, 6)
            raw.write(data)
            raw.flush()
            metadata["bytes_received"] += len(data)
            for sentence in stream.feed(data):
                metadata["valid_nmea_sentences"] += 1
                observation = parse_gga(sentence, host_utc, elapsed)
                if observation is not None:
                    parsed.write(json.dumps(observation, ensure_ascii=False, allow_nan=False) + "\n")
                    metadata["gga_records"] += 1
                    status = observation["fix_status"]
                    counts = metadata["quality_counts"]
                    counts[status] = counts.get(status, 0) + 1
            parsed.flush()
        if metadata["bytes_received"] == 0:
            metadata["status"] = "no_data"
            result = 3
            print("No bytes received. Check the selected port, baud rate and receiver output configuration.", file=sys.stderr)
        else:
            metadata["status"] = "completed"
    except KeyboardInterrupt:
        if metadata is not None:
            metadata["status"] = "interrupted"
        result = 130
    except (OSError, ValueError) as exc:
        if metadata is not None:
            metadata["status"] = "error"
            metadata["error"] = str(exc)
        print(f"GNSS recording error: {exc}", file=sys.stderr)
        result = 1
    finally:
        if metadata is not None and meta_file is not None:
            metadata.update({
                "ended_utc": utc_now(), "elapsed_s": round(time.monotonic() - started, 6),
                "invalid_nmea_checksums": stream.invalid_nmea_checksums,
                "valid_ubx_frames": stream.ubx_frames,
                "invalid_ubx_frames": stream.invalid_ubx_frames,
                "unparsed_tail_bytes": len(stream.buffer),
                "blocked_on_incomplete_ubx": stream.incomplete_ubx() is not None,
                "incomplete_ubx": stream.incomplete_ubx(),
            })
            meta_file.seek(0)
            json.dump(metadata, meta_file, ensure_ascii=False, indent=2, allow_nan=False)
            meta_file.write("\n")
            meta_file.truncate()
        for handle in (raw, parsed, meta_file):
            if handle is not None:
                try:
                    handle.flush()
                    os.fsync(handle.fileno())
                finally:
                    handle.close()
        try:
            termios.tcsetattr(fd, termios.TCSANOW, original)
        except (OSError, termios.error):
            pass
        os.close(fd)
    if metadata is not None:
        print(f"Finished: {metadata['bytes_received']} bytes, {metadata['gga_records']} valid GGA records; {metadata['status']}")
        if metadata["gga_records"] == 0:
            print("No valid GGA received. Raw bytes are preserved; RTK quality is unknown.", file=sys.stderr)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Existing serial device, preferably /dev/serial/by-id/...")
    parser.add_argument("--baud", type=int, default=115200, help="Host baud rate (default: 115200); receiver is not reconfigured")
    parser.add_argument("--duration", type=float, default=60.0, help="Seconds to receive; Ctrl-C also finishes files (default: 60)")
    parser.add_argument("--output-dir", required=True, help="New session directory; existing directories are rejected")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be a finite number greater than zero")
    try:
        return record(args.port, args.baud, args.duration, args.output_dir)
    except (OSError, ValueError, termios.error) as exc:
        print(f"Cannot start GNSS recorder: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
