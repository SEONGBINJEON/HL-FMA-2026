#!/usr/bin/env python3
"""Korean keyboard BENCH panel. Default demo never opens a hardware device.

Live connection is opt-in --connect, for the exact recorded Mega only. Opening
the USB port can reset a Mega. This release refuses non-BENCH firmware. No TX
occurs until a valid new-protocol status has identified the firmware.
"""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import struct
import sys
import termios
import time
import uuid

try:
    from .manual_protocol import (ARM, ARMED, BENCH, BOOT_ID_VALID, CONTROL,
        DISARM, DISARMED, Command, Status, FrameParser, encode_command,
        encode_status, decode_frame)
except ImportError:
    from manual_protocol import (ARM, ARMED, BENCH, BOOT_ID_VALID, CONTROL,
        DISARM, DISARMED, Command, Status, FrameParser, encode_command,
        encode_status, decode_frame)

ROOT = Path(__file__).resolve().parents[1]
MEGA_PATH = Path("/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_95431353338351A01190-if00")
MEGA_ID = ("2341", "0042", "95431353338351A01190")
KEYS = {"w", "s", "a", "d", "shift_l", "shift_r"}
SHIFTS = {"shift_l", "shift_r"}
REASONS = {0: "부팅", 1: "정지", 2: "통신 만료", 3: "통신 오류",
           4: "오래된 명령", 5: "전원 허용 조건", 6: "준비 거부",
           7: "방향 반전", 8: "조향 제한", 9: "준비", 10: "입력 중", 11: "중립"}


class Session:
    """Pure UI/session state. Only the GUI tick calls response generation."""

    def __init__(self):
        self.keys = set()
        self.focused = True
        self.phase = "locked"
        self.message = "벤치 보드 상태 대기 · 모터 전원 OFF 유지"
        self.status = None
        self.last_status_at = None
        self.last_tick_at = None
        self.last_reply = None
        self.identified = False
        self.deadman_seen = False
        self.drive_sign = 0
        self.steer_sign = 0

    def stop(self, message="정지 · 키를 모두 놓고 다시 조작 준비"):
        # Keep observed held keys: focus loss or a timeout must not pretend a
        # physically held key has been released and permit a fresh ARM.
        self.phase = "locked"
        self.deadman_seen = False
        self.drive_sign = self.steer_sign = 0
        self.message = message

    def focus(self, focused):
        self.focused = focused
        if not focused:
            self.stop("창 포커스 상실 · 키를 놓고 다시 조작 준비")

    def key_down(self, key):
        key = key.lower()
        if key not in KEYS or key in self.keys:
            return
        self.keys.add(key)
        if not self.focused or self.phase not in ("ready", "active"):
            self.stop("키를 모두 놓은 뒤 조작 준비를 누르세요")
            return
        if {"w", "s"} <= self.keys or {"a", "d"} <= self.keys:
            self.stop("서로 반대인 키 입력 · 다시 조작 준비 필요")
            return
        if self.keys & SHIFTS:
            self.phase = "active"
            self.deadman_seen = True

    def key_up(self, key):
        key = key.lower()
        if key in KEYS:
            self.keys.discard(key)
            self.stop("키 해제 · 정지, 다시 조작 준비 필요")

    def request_arm(self, now):
        if self.keys or not self.focused:
            self.stop("W/S/A/D와 Shift를 모두 놓고 창을 선택하세요")
            return False
        if (not self.identified or self.status is None or
                self.last_status_at is None or now - self.last_status_at >= .150):
            self.stop("새 벤치 보드 상태가 없어 준비할 수 없습니다")
            return False
        if not self._bench_ok(self.status) or self.status.state != DISARMED:
            self.stop("보드가 출력 잠금·정지 상태인지 확인 중입니다")
            return False
        if self.phase != "locked":
            return False
        self.phase = "neutral_pending"
        self.message = "중립 확인 중 · 아직 키를 누르지 마세요"
        return True

    @staticmethod
    def _bench_ok(value):
        return (value.boot_id != 0 and value.request_id != 0 and
                value.flags & (BENCH | BOOT_ID_VALID) == (BENCH | BOOT_ID_VALID)
                and not value.flags & 6  # DRIVE_ENABLED / STEER_ENABLED
                and value.actual_drive == 0 and value.actual_steer == 0)

    def emergency_command(self):
        if not self.identified or self.status is None:
            return None
        return Command(self.status.boot_id, self.status.request_id)

    def step(self, now, statuses=(), error=None):
        """Consume a bounded RX batch; reply only to its latest new request.

        All frames are inspected for invalid ordering, including earlier frames
        in the same batch. A UI stall takes precedence over otherwise good RX.
        """
        fault = error
        if self.last_tick_at is not None and now - self.last_tick_at >= .100:
            fault = "화면 처리 지연 · 다시 조작 준비 필요"
        self.last_tick_at = now
        if self.last_status_at is not None and now - self.last_status_at >= .150:
            fault = "보드 상태 수신 만료 · 다시 조작 준비 필요"
        latest = None
        for value in statuses:
            if not isinstance(value, Status):
                fault = "예상하지 않은 프레임"
                continue
            self.identified = True
            if not self._bench_ok(value):
                fault = "벤치 출력 잠금/부팅 식별 확인 실패 · 접속 중단 필요"
            previous = latest or self.status
            if previous is not None:
                if value.boot_id != previous.boot_id:
                    fault = "보드 재시작 · 다시 조작 준비 필요"
                    self.last_reply = None
                elif value.request_id < previous.request_id:
                    fault = "요청 번호 역행 · 다시 조작 준비 필요"
                    continue
                elif value.request_id == previous.request_id:
                    # A retry never renews freshness or earns a second response.
                    # millis/sensor telemetry may change on a retransmission.
                    if value.state == DISARMED and self.phase in ("ready", "active"):
                        fault = "보드 정지 상태 수신"
                    continue
                elif ((value.millis - previous.millis) & 0xffffffff) >= 0x80000000:
                    fault = "보드 시각 역행"
                    continue
            latest = value
            if not self._bench_ok(value):
                fault = "벤치 출력 잠금/부팅 식별 확인 실패 · 접속 중단 필요"
        if latest is not None:
            self.status = latest
            self.last_status_at = now
        if fault:
            self.stop(str(fault))
        if latest is None:
            return None
        token = (latest.boot_id, latest.request_id)
        if token == self.last_reply:
            return None
        self.last_reply = token
        base = (latest.boot_id, latest.request_id)
        if fault or not self.focused or not self._bench_ok(latest):
            return Command(*base)
        if self.phase == "neutral_pending":
            if latest.state != DISARMED or self.keys:
                self.stop("중립 확인 조건 해제")
                return Command(*base)
            self.phase = "neutral_sent"
            return Command(*base, CONTROL)
        if self.phase == "neutral_sent":
            if latest.state == DISARMED and latest.reason == 11 and not self.keys:
                self.phase = "arm_sent"
                self.message = "보드 조작 준비 응답 대기 · 키를 놓아두세요"
                return Command(*base, ARM)
            self.stop("중립 확인 응답 없음 · 다시 조작 준비 필요")
            return Command(*base)
        if self.phase == "arm_sent":
            if latest.state == ARMED and latest.reason == 9 and not self.keys:
                self.phase = "ready"
                self.message = "준비됨 · 새로 Shift를 누르고 W/S/A/D 입력"
            else:
                self.stop("보드가 조작 준비를 확인하지 않았습니다")
                return Command(*base)
        if self.phase in ("ready", "active"):
            if latest.state != ARMED:
                self.stop("보드 정지: " + REASONS.get(latest.reason, "상태 확인"))
                return Command(*base)
            deadman = int(bool(self.keys & SHIFTS))
            if self.deadman_seen and not deadman:
                self.stop("Shift 해제 · 다시 조작 준비 필요")
                return Command(*base)
            drive = int("w" in self.keys) - int("s" in self.keys) if deadman else 0
            steer = int("d" in self.keys) - int("a" in self.keys) if deadman else 0
            if ((drive and self.drive_sign and drive != self.drive_sign) or
                    (steer and self.steer_sign and steer != self.steer_sign)):
                self.stop("방향 반전 · 정지 후 다시 조작 준비 필요")
                return Command(*base)
            self.drive_sign = drive or self.drive_sign
            self.steer_sign = steer or self.steer_sign
            return Command(*base, CONTROL, drive, steer, deadman)
        return Command(*base)


class Journal:
    def __init__(self, mode, output=None):
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        self.path = Path(output) if output else ROOT / "data" / f"manual_control_{stamp}_{uuid.uuid4().hex[:8]}"
        self.path.mkdir(parents=True, exist_ok=False)
        self.events = (self.path / "events.jsonl").open("x", encoding="utf-8")
        self.rx = (self.path / "rx.bin").open("xb")
        self.tx = (self.path / "tx.bin").open("xb")
        sources = {}
        for name in ("manual_control.py", "manual_protocol.py"):
            source = Path(__file__).with_name(name).read_bytes()
            (self.path / name).write_bytes(source)
            sources[name] = hashlib.sha256(source).hexdigest()
        metadata = {"mode": mode, "started_utc": datetime.now(timezone.utc).isoformat(),
                    "sources_sha256": sources, "python": sys.version,
                    "physical_vehicle_verified": False,
                    "conditions": "Demo: no hardware; connect: user must confirm motor supply OFF before launch.",
                    "port": str(MEGA_PATH) if mode == "connect" else None}
        (self.path / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")

    def event(self, name, **fields):
        row = {"host_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
               "host_monotonic": time.monotonic(), "event": name, **fields}
        self.events.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.events.flush()

    def bytes(self, direction, data):
        stream = self.rx if direction == "rx" else self.tx
        stream.write(data)
        stream.flush()
        self.event(direction, count=len(data), hex=data.hex())

    def close(self):
        for stream in (self.events, self.rx, self.tx):
            stream.close()


def _device_identity(path):
    info = path.stat()
    if not stat.S_ISCHR(info.st_mode):
        raise RuntimeError("Mega 경로가 문자 장치가 아닙니다")
    node = Path(f"/sys/dev/char/{os.major(info.st_rdev)}:{os.minor(info.st_rdev)}/device").resolve(strict=True)
    for item in (node, *node.parents):
        if (item / "idVendor").exists():
            return info.st_rdev, tuple((item / name).read_text().strip() for name in
                                     ("idVendor", "idProduct", "serial"))
    raise RuntimeError("USB 식별 정보를 찾지 못했습니다")


class MegaTransport:
    """Nonblocking POSIX serial, flock acquired before changing host termios."""

    def __init__(self):
        self.fd = None
        self.saved = None
        if not MEGA_PATH.is_symlink():
            raise RuntimeError("지정된 Mega by-id 경로가 없습니다")
        expected_rdev, identity = _device_identity(MEGA_PATH)
        if identity != MEGA_ID:
            raise RuntimeError("Mega VID/PID/일련번호 불일치")
        try:
            self.fd = os.open(MEGA_PATH, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            current_rdev, current_identity = _device_identity(MEGA_PATH)
            if (os.fstat(self.fd).st_rdev != expected_rdev or
                    current_rdev != expected_rdev or current_identity != MEGA_ID):
                raise RuntimeError("열기 중 Mega 식별이 변경됐습니다")
            self.saved = termios.tcgetattr(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except BaseException:
            self.close()
            raise

    def _queue(self, code):
        return struct.unpack("I", fcntl.ioctl(self.fd, code, struct.pack("I", 0)))[0]

    @property
    def in_waiting(self):
        return self._queue(termios.FIONREAD)

    @property
    def out_waiting(self):
        return self._queue(termios.TIOCOUTQ)

    def read(self, size):
        try:
            return os.read(self.fd, size)
        except BlockingIOError:
            return b""

    def write(self, data):
        return os.write(self.fd, data)

    def close(self):
        if self.fd is not None:
            try:
                if self.saved is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            except OSError:
                pass
            finally:
                os.close(self.fd)
                self.fd = None


class Client:
    """Bounded transport work performed exclusively in the UI main thread."""

    def __init__(self, transport, session, journal=None):
        self.transport, self.session, self.journal = transport, session, journal
        self.parser = FrameParser()
        self.failed = False

    def _event(self, name, **fields):
        if self.journal:
            self.journal.event(name, **fields)

    def fail(self, reason):
        self.failed = True
        self.session.stop(reason)
        try:
            self._event("connection_failed", reason=reason)
        finally:
            self.transport.close()

    def send(self, command, dispatch_started=None, dispatch_deadline=None):
        if self.failed or command is None:
            return
        try:
            if self.transport.out_waiting:
                self.fail("송신 대기 발견 · 연결 종료, 보드 만료 정지 대기")
                return
            raw = encode_command(command)
            self._event("tx_attempt", command=asdict(command), count=len(raw))
            # Logging/flush or parser work can stall after entry into the UI
            # tick. Check actual wall time immediately before the system write.
            before_write = time.monotonic()
            if (command.action != DISARM and (
                    (dispatch_started is not None and before_write - dispatch_started >= .100) or
                    (dispatch_deadline is not None and before_write >= dispatch_deadline))):
                self.session.stop("송신 전 화면 처리 지연 · 다시 조작 준비 필요")
                command = self.session.emergency_command()
                raw = encode_command(command)
                self._event("tx_replaced_by_disarm", command=asdict(command))
            written = self.transport.write(raw)
            if written and self.journal:
                self.journal.bytes("tx", raw[:written])
            if written != len(raw):
                self.fail("부분 송신 · 재전송 없이 연결 종료")
        except (OSError, ValueError) as exc:
            self.fail("송신/기록 오류: " + type(exc).__name__)

    def stop(self, reason):
        self.session.stop(reason)
        self._event("stop", reason=reason)
        self.send(self.session.emergency_command())

    def tick(self, now):
        if self.failed:
            return
        dispatch_started = time.monotonic()
        # The gap before this tick and time spent inside it share one budget.
        # Two individually short delays must not permit an overdue command.
        dispatch_deadline = dispatch_started + .100
        if self.session.last_tick_at is not None:
            dispatch_deadline = min(dispatch_deadline, self.session.last_tick_at + .100)
        try:
            size = self.transport.in_waiting
            if size > 4096:
                self.stop("수신 과부하 · 연결 종료")
                self.fail("수신 과부하")
                return
            raw = self.transport.read(size) if size else b""
            if raw and self.journal:
                self.journal.bytes("rx", raw)
            values, errors = self.parser.feed(raw, now)
            if time.monotonic() - dispatch_started >= .100:
                errors.append("수신/기록 처리 지연 · 다시 조작 준비 필요")
            was_locked = self.session.phase == "locked"
            command = self.session.step(now, values, "; ".join(errors) if errors else None)
            if values:
                self._event("status", status=asdict(self.session.status) if self.session.status else None,
                            batch_frames=len(values), errors=errors)
            self.send(command, dispatch_started, dispatch_deadline)
            if command is None and not was_locked and self.session.phase == "locked":
                self.send(self.session.emergency_command())
        except (OSError, ValueError) as exc:
            self.fail("수신/기록 오류: " + type(exc).__name__)

    def close(self):
        try:
            if not self.failed:
                self.stop("창 닫기 · 정지")
        finally:
            self.transport.close()


class KeyboardEvents:
    """Tk adapter with deterministic, headless-testable key release handling.

    Shift release is immediate. Movement-key release can be paired only with
    an equal X11 timestamp/keycode repeat; pending releases are also committed
    before every response-generating tick, so idle starvation cannot retain a
    motion key. A fast real release/repress therefore requires a fresh ARM.
    """

    def __init__(self, session, stop, schedule, cancel, event):
        self.session, self.stop = session, stop
        self.schedule, self.cancel, self.event = schedule, cancel, event
        self.pending = {}

    def _commit(self, key):
        pending = self.pending.pop(key, None)
        if pending:
            self.cancel(pending[2])
        self.session.key_up(key)
        self.stop(self.session.message)
        self.event("key_up", key=key)

    def press(self, key, timestamp, keycode):
        key = key.lower()
        if key == "escape":
            self.flush()
            self.stop("Escape · 정지")
            return
        if key not in KEYS:
            return
        pending = self.pending.get(key)
        if pending:
            if key not in SHIFTS and pending[:2] == (timestamp, keycode):
                self.cancel(pending[2])
                self.pending.pop(key)
                return  # Precisely paired X11 repeat, not a new press.
            self._commit(key)
        self.session.key_down(key)
        self.event("key_down", key=key)

    def release(self, key, timestamp, keycode):
        key = key.lower()
        if key not in KEYS:
            return
        if key in SHIFTS:
            self._commit(key)
            return
        if key in self.pending:
            self._commit(key)
        callback_id = self.schedule(lambda: self._commit(key))
        self.pending[key] = (timestamp, keycode, callback_id)

    def flush(self):
        for key in tuple(self.pending):
            self._commit(key)


class DemoTransport:
    """UI-only synthetic data. Not the C++ firmware validation emulator."""

    def __init__(self):
        self.start = self.issue = self.last_emit = time.monotonic()
        self.request = 0
        self.state, self.reason = DISARMED, 0
        self.drive = self.steer = 0
        self.neutral_at = None
        self.deadman_seen = False
        self.consumed = True
        self.pending = b""
        self.closed = False

    def _stop(self, reason):
        self.state, self.reason = DISARMED, reason
        self.drive = self.steer = 0
        self.neutral_at = None
        self.deadman_seen = False

    @property
    def in_waiting(self):
        now = time.monotonic()
        if now - self.issue >= .150:
            self._stop(2)
            self.consumed = True
        if not self.closed and now - self.last_emit >= .050:
            if self.consumed:
                self.request += 1
                self.issue = now
                self.consumed = False
            self.last_emit = now
            self.pending = encode_status(Status(1, self.request, int((self.issue - self.start) * 1000),
                steering_adc=0, state=self.state, reason=self.reason,
                requested_drive=self.drive, requested_steer=self.steer))
        return len(self.pending)

    @property
    def out_waiting(self):
        return 0

    def read(self, size):
        result, self.pending = self.pending[:size], self.pending[size:]
        return result

    def write(self, raw):
        value = decode_frame(raw)
        now = time.monotonic()
        if value.action == DISARM:
            self._stop(1)
            self.consumed = True
        elif (value.boot_id != 1 or value.request_id != self.request or
              self.consumed or now - self.issue >= .150):
            self._stop(4)
            self.consumed = True
        else:
            self.consumed = True
            if value.action == ARM:
                if self.state == DISARMED and self.neutral_at is not None and now - self.neutral_at < .500:
                    self.state, self.reason = ARMED, 9
                    self.deadman_seen = False
                else:
                    self._stop(6)
            elif self.state == DISARMED and not (value.drive or value.steer or value.deadman):
                self.neutral_at, self.reason = now, 11
            elif self.state == ARMED and (value.deadman or not self.deadman_seen):
                self.drive, self.steer = value.drive, value.steer
                self.deadman_seen |= bool(value.deadman)
                self.reason = 10 if value.drive or value.steer else 11
            else:
                self._stop(1)
        return len(raw)

    def close(self):
        self.closed = True


def run_ui(connect=False, output=None):
    import tkinter as tk
    from tkinter import ttk

    journal = Journal("connect" if connect else "demo", output)
    transport = None
    root = None
    try:
        # Create the window before opening hardware: an absent display must
        # never reset a board or consume a serial port.
        root = tk.Tk()
        root.title("HL FMA · 벤치 수동 조작")
        root.geometry("790x660")
        root.minsize(680, 600)
        root.configure(bg="#101926")
        transport = MegaTransport() if connect else DemoTransport()
        session = Session()
        client = Client(transport, session, journal)
        frame = tk.Frame(root, bg="#101926", padx=28, pady=22)
        frame.pack(fill="both", expand=True)
        mode = "보드 연결 · 벤치 시험" if connect else "화면 연습 · 장치 연결 없음"
        # Explicit pixel fonts keep the compact panel usable when Tk inherits
        # a very large desktop points-to-pixels scale (observed on this laptop).
        tk.Label(frame, text=mode, fg="#92bedc", bg="#101926", font=("sans", -17)).pack(anchor="w")
        tk.Label(frame, text="실제 모터 출력 잠금", fg="#f6d17a", bg="#101926",
                 font=("sans", -30, "bold")).pack(anchor="w", pady=(10, 8))
        tk.Label(frame, text="모터 구동 전원과 차량 배터리는 OFF로 유지하세요.",
                 fg="#e5edf4", bg="#101926", wraplength=700, justify="left",
                 font=("sans", -15)).pack(anchor="w")
        message = tk.StringVar(value=session.message)
        tk.Label(frame, textvariable=message, wraplength=700, justify="left", anchor="w",
                 fg="#ffffff", bg="#203248", padx=14, pady=16, font=("sans", -16)).pack(fill="x", pady=20)
        tk.Label(frame, text="① 키를 모두 놓기 → ② 조작 준비 → ③ 새로 Shift + W/S/A/D\n"
                 "W 전진 · S 후진 · A 왼쪽 · D 오른쪽 (방향 실측 전)\n"
                 "키 해제 / 다른 창 선택 / Escape → 정지, 다시 조작 준비 필요",
                 fg="#cedbea", bg="#101926", justify="left", wraplength=700,
                 font=("sans", -14)).pack(anchor="w")
        buttons = tk.Frame(frame, bg="#101926")
        buttons.pack(fill="x", pady=18)

        def arm():
            accepted = session.request_arm(time.monotonic())
            journal.event("arm_click", accepted=accepted)

        def stop(reason="정지 버튼 · 키를 모두 놓고 다시 조작 준비"):
            client.stop(reason)

        arm_button = tk.Button(buttons, text="조작 준비", command=arm, font=("sans", -18),
                               bg="#297d9b", fg="white", padx=20, pady=12)
        arm_button.pack(side="left")
        tk.Button(buttons, text="정지  ·  Esc", command=stop, font=("sans", -20, "bold"),
                  bg="#b52e36", fg="white", padx=40, pady=11).pack(side="right")
        telemetry = tk.StringVar(value="보드 응답 대기")
        tk.Label(frame, textvariable=telemetry, fg="#c9dae8", bg="#101926", justify="left",
                 anchor="w", wraplength=700, font=("monospace", -15)).pack(fill="x", pady=(0, 10))
        tk.Label(frame, text="센서값은 원시값입니다. 조향 중앙·속도·실제 정지 거리를 뜻하지 않습니다.\n"
                 "이 화면의 정지 기능은 차량의 물리 정지 수단을 대신하지 않습니다.",
                 fg="#91a7ba", bg="#101926", justify="left", wraplength=700, font=("sans", -12)).pack(anchor="w")
        tk.Label(frame, text="기록: " + str(journal.path), fg="#91a7ba", bg="#101926",
                 wraplength=700, justify="left", font=("sans", -11)).pack(anchor="w", pady=(12, 0))
        closing = False
        keyboard = KeyboardEvents(session, client.stop,
            lambda callback: root.after(0, callback), root.after_cancel, journal.event)

        def press(event):
            key = event.keysym.lower()
            keyboard.press(key, event.time, event.keycode)
            return "break" if key in KEYS or key == "escape" else None

        def release(event):
            keyboard.release(event.keysym, event.time, event.keycode)

        def focus_change(focused):
            if closing:
                return
            # Latch the event now, even if focus returns before the next tick.
            # Internal widget focus changes conservatively stop as well.
            session.focus(focused)
            journal.event("focus", focused=focused)
            if not focused:
                keyboard.flush()
                client.stop(session.message)

        def close():
            nonlocal closing
            closing = True
            client.close()
            root.destroy()

        def callback_error(exc_type, value, traceback):
            client.fail("화면 처리 오류: " + exc_type.__name__)
            message.set(session.message)

        def tick():
            if closing:
                return
            keyboard.flush()
            client.tick(time.monotonic())
            message.set(session.message)
            arm_button.configure(state="disabled" if client.failed else "normal")
            value = session.status
            if value:
                source = "화면용 가상값" if not connect else "보드 보고값"
                telemetry.set(f"{source} | 상태: {session.phase} | {REASONS[value.reason]}\n"
                    f"요청 구동/조향: {value.requested_drive:+d} / {value.requested_steer:+d}    "
                    f"실제 출력: {value.actual_drive} / {value.actual_steer}\n"
                    f"엔코더: {value.encoder_count}    조향 ADC: {value.steering_adc}\n"
                    f"부팅 번호: {value.boot_id}    요청 번호: {value.request_id}")
            root.after(10, tick)

        root.report_callback_exception = callback_error
        root.bind_all("<KeyPress>", press)
        root.bind_all("<KeyRelease>", release)
        root.bind("<FocusOut>", lambda event: focus_change(False))
        root.bind("<FocusIn>", lambda event: focus_change(True))
        root.protocol("WM_DELETE_WINDOW", close)
        journal.event("window_opened")
        root.after(10, tick)
        root.mainloop()
    finally:
        if transport is not None:
            transport.close()
        journal.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--demo", action="store_true", help="장치에 접근하지 않는 화면 연습 (기본)")
    modes.add_argument("--connect", action="store_true", help="모터 OFF 확인 후 지정 Mega 벤치 보드에 접속; USB 열기 시 리셋 가능")
    parser.add_argument("--output", type=Path, help="새 기록 폴더 (기존 폴더는 거부)")
    args = parser.parse_args(argv)
    try:
        run_ui(args.connect, args.output)
    except Exception as exc:
        print(f"수동 조작 창을 시작하지 못했습니다: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
