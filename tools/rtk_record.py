#!/usr/bin/env python3
"""Record one u-blox GNSS and forward only CRC-validated RTCM corrections.

Credentials are read only from an owner-only configuration file. Optional explicit --rate-hz sends RAM-only CFG-RATE through the same GNSS owner.
No motor commands, ROS integration or fixed reference positions.
"""

import argparse
import json
import logging
import math
import os
from pathlib import Path
import re
import stat
import sys
from threading import Lock
import time

from gnss_record import MixedNmeaStream, parse_gga, utc_now
from start_gnss_recording import device_inventory, select_port
from ntrip_recovery import Recovery


class SafeError(ValueError):
    """Only predefined, non-sensitive descriptions may be supplied here."""


def load_config(path):
    fd = os.open(Path(path).expanduser(), os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o600:
            raise SafeError("설정 파일은 현재 사용자 소유의 일반 파일이며 권한이 600이어야 합니다.")
        if info.st_size > 65536:
            raise SafeError("설정 파일 크기가 허용 범위를 넘었습니다.")
        with os.fdopen(fd, encoding="utf-8") as source:
            fd = None
            config = json.load(source)
        if not isinstance(config, dict):
            raise SafeError("설정 형식이 올바르지 않습니다.")
        if not isinstance(config.get("server"), str) or not re.fullmatch(r"[A-Za-z0-9.-]+", config["server"]):
            raise SafeError("서버에는 인증정보나 URL 경로 없는 호스트 이름을 입력해야 합니다.")
        if not isinstance(config.get("mountpoint"), str) or not re.fullmatch(r"[A-Za-z0-9_.~/-]+", config["mountpoint"]):
            raise SafeError("마운트포인트 형식이 올바르지 않습니다.")
        if type(config.get("port")) is not int or not 1 < config["port"] < 65536:
            raise SafeError("포트 번호가 올바르지 않습니다.")
        if type(config.get("https")) is not bool or config.get("version") not in ("1.0", "2.0"):
            raise SafeError("HTTPS 또는 NTRIP 버전 설정이 올바르지 않습니다.")
        if type(config.get("ggainterval")) is not int or not 1 <= config["ggainterval"] <= 60:
            raise SafeError("GGA 전송 간격은 1~60초여야 합니다.")
        for key in ("ntripuser", "ntrippassword"):
            value = config.get(key)
            if not isinstance(value, str) or not value or any(ord(c) < 32 for c in value):
                raise SafeError("NTRIP 인증정보 형식이 올바르지 않습니다.")
        return {key: config[key] for key in ("server", "port", "mountpoint", "https", "version",
                                             "ntripuser", "ntrippassword", "ggainterval")}
    finally:
        if fd is not None:
            os.close(fd)


class LivePosition:
    def __init__(self, clock=time.monotonic, stale_seconds=5.0):
        self.clock, self.stale_seconds = clock, stale_seconds
        self.lock = Lock()
        self.latest = None
        self.active = True

    def update(self, row, raw):
        valid = row.get("quality_code") in (1, 2, 3, 4, 5) and row.get("checksum_valid") is True
        for key, bound in (("latitude_deg", 90), ("longitude_deg", 180)):
            value = row.get(key)
            valid = valid and isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value) and abs(value) <= bound
        with self.lock:
            self.latest = (dict(row), bytes(raw), self.clock()) if valid and self.active else None

    def snapshot(self):
        with self.lock:
            if not self.active or self.latest is None or self.clock() - self.latest[2] > self.stale_seconds:
                raise ValueError("No fresh measured GGA")
            return self.latest

    def fresh(self):
        try:
            self.snapshot()
            return True
        except ValueError:
            return False

    def close(self):
        with self.lock:
            self.active = False
            self.latest = None

    def get_coordinates(self):
        row, _, _ = self.snapshot()
        # Every key is explicit. Missing optional data stays blank. The pinned
        # client's GGA formatter is overridden below to avoid its zero/default
        # substitution for alt/sep and its incomplete DGPS/PPS quality mapping.
        optional = lambda key: row.get(key) if row.get(key) is not None else ""
        return {"lat": row["latitude_deg"], "lon": row["longitude_deg"],
                "alt": optional("altitude_m"), "sep": optional("geoid_separation_m"),
                "fix": {1: "3D", 2: "DGPS", 3: "PPS", 4: "RTK FIXED", 5: "RTK FLOAT"}[row["quality_code"]],
                "sip": optional("satellites"), "hdop": optional("hdop"),
                "diffage": optional("differential_age_s"), "diffstation": optional("reference_station_id")}


def make_client(app):
    import importlib.metadata
    if importlib.metadata.version("pygnssutils") != "1.2.7":
        raise SafeError("이 도구는 검증한 pygnssutils 1.2.7 버전이 필요합니다.")
    from pygnssutils import GNSSNTRIPClient
    from pynmeagps import NMEAReader

    class ReceiverGGAClient(GNSSNTRIPClient):
        def _format_gga(self):
            try:
                _, raw, _ = app.snapshot()
                return raw, NMEAReader.parse(raw, validate=1)
            except (ValueError, TypeError):
                self.stop()
                return None, None

    return ReceiverGGAClient(app=app, retries=0, timeout=5, verbosity=-1)


def validated_rtcm(item):
    """An output Queue also contains transmitted GGA: never forward it."""
    raw = item[0] if isinstance(item, tuple) and len(item) == 2 else item
    if not isinstance(raw, bytes) or len(raw) < 8 or raw[0] != 0xD3 or raw[1] & 0xFC:
        return None
    length = ((raw[1] & 3) << 8) | raw[2]
    if len(raw) != length + 6:
        return None
    try:
        from pyrtcm import RTCMReader
        RTCMReader.parse(raw, validate=1)
        return raw
    except Exception:
        return None


def private_file(path, binary=False):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    return os.fdopen(fd, "wb" if binary else "w", **({} if binary else {"encoding": "utf-8"}))


def record(config, duration, output_dir, *, inventory_fn=device_inventory,
           serial_factory=None, client_factory=make_client, clock=time.monotonic,
           initial_wait=20.0, stale_seconds=5.0, requested_hz=None):
    if requested_hz is not None and (type(requested_hz) is not int or requested_hz not in (1, 5, 10)):
        raise SafeError('요청 측위 주기는 1/5/10Hz만 지원합니다.')
    if duration is not None and (not math.isfinite(duration) or duration <= 0):
        raise SafeError("기록 시간은 0보다 큰 유한한 값이어야 합니다.")
    output = Path(output_dir).expanduser().absolute()
    if output.exists() or output.is_symlink():
        raise SafeError("기존 출력 경로는 덮어쓰지 않습니다. 새 폴더를 선택하세요.")
    # The upstream DEBUG logger includes Basic Authorization headers. Never
    # enable logging in this dedicated process, including after worker stop.
    logging.disable(sys.maxsize)
    port = select_port(inventory_fn())
    if serial_factory is None:
        from serial import Serial
        serial_factory = Serial
    receiver = serial_factory(port=port, baudrate=115200, timeout=0.1,
                              write_timeout=1.0, exclusive=True)
    app = LivePosition(clock, stale_seconds)
    client = None
    recovery = Recovery(app, client_factory, config, clock)
    stream = MixedNmeaStream()
    handles = []
    start = clock()
    next_progress = 5.0
    last_quality = "미수신"
    metadata = None
    exit_code = 0
    try:
        output.mkdir(parents=True, exist_ok=False, mode=0o700)
        for name, binary in (("raw.bin", True), ("gga.jsonl", False),
                             ("corrections.rtcm3", True), ("metadata.json", False)):
            handles.append(private_file(output / name, binary))
        raw, parsed, corrections, meta_file = handles
        metadata = {"schema_version": 1, "status": "waiting_for_fresh_gga", "port": port,
                    "baud": 115200, "started_utc": utc_now(), "requested_duration_s": duration,
                    "bytes_received": 0, "gga_records": 0, "quality_counts": {},
                    "rtcm_frames_written": 0, "rtcm_bytes_written": 0, "rejected_queue_items": 0,
                    "rtcm_first_written_elapsed_s": None, "rtcm_last_written_elapsed_s": None,
                    "rtcm_max_interframe_gap_s": 0.0, "rtcm_last_write_age_s": None,
                    "http_status_codes": [], "ntrip_started": False,
                    "ntrip_connected_before_stop": None,
                    "connected_note": "라이브러리 connected 플래그는 인증 성공·RTCM 수신·RTK fixed를 보증하지 않습니다.",
                    "endpoint": {key: config[key] for key in ("server", "port", "mountpoint", "https", "version")},
                    "gga_source": "receiver_checksum_valid_gga_unchanged",
                    "receiver_configuration_commands_sent": False,
                    "files": {"raw": "raw.bin", "gga": "gga.jsonl", "corrections": "corrections.rtcm3"},
                    "rtk_fixed_observed": False,
                    "fixed_valid_coordinates_count": 0,
                    "limitations": "RTCM 전달과 RTK fixed 관측은 별개이며, 위치 정확도나 주행 준비를 보증하지 않습니다."}
        json.dump(metadata, meta_file, ensure_ascii=False, indent=2)
        meta_file.flush()
        if requested_hz is not None:
            # Same identified exclusive GNSS owner, before NTRIP; no competing rate tool.
            import struct
            from gnss_set_rate import ubx
            frame = ubx(0x06, 0x08, struct.pack('<HHH', round(1000/requested_hz), 1, 1))
            if receiver.write(frame) != len(frame):
                raise SafeError('측위 주기 RAM 설정 송신이 일부만 완료되었습니다.')
            metadata['receiver_configuration_commands_sent'] = True
            metadata['rate_request'] = {'requested_hz': requested_hz, 'ram_only': True,
                                        'frame_hex': frame.hex(), 'ack_verified': False,
                                        'actual_gga_rate_note': 'measure gga.jsonl intervals; request is not observed rate'}
        print("GNSS 원시 기록 시작. 새 측정 GGA를 받은 뒤에만 NTRIP 연결합니다.", flush=True)
        while duration is None or clock() - start < duration:
            data = receiver.read(max(1, min(getattr(receiver, "in_waiting", 0), 65536)))
            if data:
                raw.write(data)
                raw.flush()
                metadata["bytes_received"] += len(data)
                for sentence in stream.feed(data):
                    row = parse_gga(sentence, utc_now(), round(clock() - start, 6))
                    if row is None:
                        continue
                    parsed.write(json.dumps(row, allow_nan=False) + "\n")
                    parsed.flush()
                    metadata["gga_records"] += 1
                    counts = metadata["quality_counts"]
                    counts[row["fix_status"]] = counts.get(row["fix_status"], 0) + 1
                    last_quality = row["fix_status"]
                    metadata["rtk_fixed_observed"] |= row["quality_code"] == 4
                    app.update(row, sentence)
                    if row["quality_code"] == 4 and app.fresh():
                        metadata["fixed_valid_coordinates_count"] += 1
            recovery.tick()
            client = recovery.client
            metadata["ntrip_started"] = recovery.attempts > 0
            metadata["connection"] = recovery.snapshot()
            metadata["http_status_codes"] = list(recovery.http_codes)
            metadata["status"] = "recording" if app.fresh() else "waiting_for_fresh_gga"
            if client is not None:
                for item in recovery.items():
                    frame = validated_rtcm(item)
                    if frame is None:
                        metadata["rejected_queue_items"] += 1
                        continue
                    if not app.fresh():
                        break
                    written = receiver.write(frame)
                    if written != len(frame):
                        raise SafeError("RTCM 전송이 일부만 완료되어 중단했습니다.")
                    recovery.valid_written()
                    corrections.write(frame)
                    corrections.flush()
                    metadata["rtcm_frames_written"] += 1
                    metadata["rtcm_bytes_written"] += written
                    elapsed = round(clock() - start, 6)
                    previous = metadata["rtcm_last_written_elapsed_s"]
                    if previous is None:
                        metadata["rtcm_first_written_elapsed_s"] = elapsed
                    else:
                        metadata["rtcm_max_interframe_gap_s"] = max(metadata["rtcm_max_interframe_gap_s"], elapsed - previous)
                    metadata["rtcm_last_written_elapsed_s"] = elapsed
            elapsed = clock() - start
            if elapsed >= next_progress:
                metadata["connection"] = recovery.snapshot()
                live = {"host_utc": utc_now(), "connection": recovery.snapshot(),
                        "fresh_gga": app.fresh(), "last_fix_status": last_quality,
                        "rtcm_last_written_elapsed_s": metadata["rtcm_last_written_elapsed_s"],
                        "recorder_elapsed_s": elapsed}
                temp = output / ".health.tmp"
                with temp.open("w") as health:
                    json.dump(live, health, allow_nan=False)
                temp.chmod(0o600)
                temp.replace(output / "health.json")
                last_correction = metadata["rtcm_last_written_elapsed_s"]
                age = "미수신" if last_correction is None else f"{elapsed - last_correction:.1f}초 전"
                print(f"[{elapsed:.0f}초] GPS={last_quality} | 보정 {metadata['rtcm_frames_written']}개 전달 | 최근 보정 {age}", flush=True)
                next_progress = elapsed + 5.0
        if metadata["status"] in ("waiting_for_fresh_gga", "recording"):
            if metadata["bytes_received"] == 0:
                metadata["status"], exit_code = "no_data", 3
            elif not app.fresh():
                metadata["status"], exit_code = "no_fresh_gga", 3
            elif not metadata["ntrip_started"]:
                metadata["status"], exit_code = "no_fresh_gga", 3
            elif metadata["rtcm_frames_written"] == 0:
                metadata["status"], exit_code = "no_rtcm_received", 4
            else:
                metadata["status"] = "completed"
    except KeyboardInterrupt:
        if metadata is not None:
            metadata["status"] = "interrupted"
        exit_code = 130
    except Exception:
        # Exception strings from a network library can include credentials.
        if metadata is not None:
            metadata["status"] = "error"
            metadata["error_code"] = "recording_or_connection_failed"
        exit_code = 1
    finally:
        app.close()
        if client is not None:
            try:
                flag = client.connected
                if metadata is not None and type(flag) is bool:
                    metadata["ntrip_connected_before_stop"] = flag
            except Exception:
                pass
            try:
                client.stop()
            except Exception:
                pass
        recovery.close()
        if metadata is not None:
            metadata["connection_at_stop"] = recovery.snapshot()
        cleanup_errors = []
        try:
            receiver.close()
        except Exception:
            cleanup_errors.append("receiver_close_failed")
        # Each handle gets its own cleanup attempt, even if a previous flush,
        # fsync or close failed. Never include exception strings in diagnostics.
        def close_handle(handle, label):
            try:
                handle.flush()
                os.fsync(handle.fileno())
            except Exception:
                cleanup_errors.append(label + "_flush_failed")
            finally:
                try:
                    handle.close()
                except Exception:
                    cleanup_errors.append(label + "_close_failed")
        data_handles = handles[:-1] if metadata is not None else handles
        for index, handle in enumerate(data_handles):
            close_handle(handle, "data_" + str(index))
        if metadata is not None:
            metadata.update(ended_utc=utc_now(), elapsed_s=round(clock() - start, 6),
                            blocked_on_incomplete_ubx=stream.incomplete_ubx() is not None,
                            incomplete_ubx=stream.incomplete_ubx(),
                            invalid_nmea_checksums=stream.invalid_nmea_checksums)
            last = metadata["rtcm_last_written_elapsed_s"]
            if last is not None:
                metadata["rtcm_last_write_age_s"] = max(0.0, metadata["elapsed_s"] - last)
            metadata["cleanup_error_codes"] = cleanup_errors
            if cleanup_errors:
                metadata["status"], exit_code = "error", 1
            try:
                meta_file.seek(0)
                json.dump(metadata, meta_file, ensure_ascii=False, indent=2, allow_nan=False)
                meta_file.write("\n")
                meta_file.truncate()
            except Exception:
                cleanup_errors.append("metadata_write_failed")
            finally:
                close_handle(meta_file, "metadata")
        if cleanup_errors:
            exit_code = 1
            if metadata is not None:
                metadata["status"] = "error"
    return exit_code, metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    timing = parser.add_mutually_exclusive_group()
    timing.add_argument("--duration", type=float, default=60.0)
    timing.add_argument("--until-interrupt", action="store_true")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument('--rate-hz', type=int, choices=(1,5,10), help='optional RAM-only CFG-RATE before corrections; same exclusive GNSS owner')
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        code, report = record(config, None if args.until_interrupt else args.duration, args.output_dir, requested_hz=args.rate_hz)
        if report is not None:
            print(f"종료: {report['status']} / GGA {report['gga_records']}개 / RTCM {report['rtcm_frames_written']}개 전달 / RTK fixed 관측 {report['rtk_fixed_observed']}")
        return code
    except SafeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except Exception:
        print("RTK 기록을 시작하거나 마무리하지 못했습니다. 설정 권한·전용 환경·u-blox 연결을 확인하세요. 인증정보는 출력하지 않았습니다.", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
