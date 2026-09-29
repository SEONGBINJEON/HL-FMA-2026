#!/usr/bin/env python3
"""Receive-only steering ADC window for the hand-aligned straight-return check.

Battery OFF. The operator moves the wheels by hand to the same straight
reference and marks the current 3 s window of POT_RAW values. This tool only
reads the current `F8 PID HILL HOLD` ASCII monitor line. It never writes to the
serial port (no heartbeat, motor command, upload or reconnect). Opening the
port with the OS may still reset the board; that is outside our control.

The `--reviewed-source` file is copied and hashed as evidence of which source
the operator says is on the board. It is NOT a verification of flash contents.
Actual drive/steer motor output is not in this telemetry and stays UNVERIFIED;
ENC=0 / MODE=HOLD is not treated as "output 0".

  python3 tools/steering_center_receiver.py --receive --battery-off-confirmed \
      --reviewed-source PATH --output-dir NEW_DIR --port /dev/serial/by-id/...

Standard library + tools/gnss_record.open_serial only.
"""
import argparse
import hashlib
import json
import os
import re
import select
import shutil
import statistics
import sys
import time
from collections import deque
from datetime import datetime, timedelta, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gnss_record  # noqa: E402  (open_serial: O_RDONLY, raw, no echo)

KST = timezone(timedelta(hours=9), "KST")
BAUD = 115200
BOOT_BANNER = "F8 PID HILL HOLD READY"
WINDOW_S = 3.0
MAX_AGE_S = 0.6
MIN_SAMPLES = 10
MIN_SPAN_S = 2.5
# Display warning only. Not a measured stop/motion threshold, not a calibration.
RANGE_WARN_ADC = 4
INT32 = (-2 ** 31, 2 ** 31 - 1)

# label -> (Korean button text, approach side). Left/right = side the wheel
# was returned FROM by hand; not a steering command sign.
LABELS = {
    "straight_reference": ("직진 기준 (손 정렬)", None),
    "return_from_left_1": ("왼쪽에서 직진 복귀 1회", "left"),
    "return_from_right_1": ("오른쪽에서 직진 복귀 1회", "right"),
    "return_from_left_2": ("왼쪽에서 직진 복귀 2회", "left"),
    "return_from_right_2": ("오른쪽에서 직진 복귀 2회", "right"),
}

_MONITOR = re.compile(
    r"^\[MONITOR\] ENC=(-?\d+) TARGET=(-?\d+) ERROR=(-?\d+) MODE=(HOLD|DRIVE)"
    r" \| POT_RAW=(\d+) POT=(-?\d+(?:\.\d+)?) STEER_TARGET=(\d+) STEER_CENTER=(\d+)$")


def kst_now():
    return datetime.now(KST).isoformat(timespec="milliseconds")


def _adc(text):
    value = int(text)
    return value if 0 <= value <= 1023 else None


def _i32(text):
    value = int(text)
    return value if INT32[0] <= value <= INT32[1] else None


def parse_monitor(line):
    """Parse one exact monitor line; anything else (other protocol, out of
    range, non-finite) returns None. Measurement value is POT_RAW."""
    if isinstance(line, bytes):
        try:
            line = line.decode("ascii")
        except UnicodeDecodeError:
            return None
    m = _MONITOR.match(line.strip("\r\n"))
    if not m:
        return None
    enc, tgt, err = (_i32(m.group(i)) for i in (1, 2, 3))
    raw, steer_t, center = (_adc(m.group(i)) for i in (5, 7, 8))
    filtered = float(m.group(6))
    if None in (enc, tgt, err, raw, steer_t, center) or not 0 <= filtered <= 1023:
        return None
    return {"encoder": enc, "hold_target": tgt, "hold_error": err, "mode": m.group(4),
            "steering_adc": raw, "steering_filtered": filtered,
            "steer_target_adc": steer_t, "steer_center_adc": center}


def summarize_window(rows, now, window_s=WINDOW_S, max_age_s=MAX_AGE_S,
                     min_samples=MIN_SAMPLES, min_span_s=MIN_SPAN_S, range_warn_adc=RANGE_WARN_ADC):
    """Describe the last `window_s` seconds of rows ({monotonic_s, steering_adc}).

    Returns {"ok": False, "reason": ...} when the window is unusable. On success
    returns raw samples, median/min/max/range, span, n and rail flag. It does
    not decide whether the wheel was still; that stays an operator assertion.
    """
    rows = list(rows)
    if any(r["monotonic_s"] > now for r in rows):
        return {"ok": False, "reason": "future_timestamp"}
    window = [r for r in rows if now - window_s <= r["monotonic_s"] <= now]
    if not window:
        return {"ok": False, "reason": "no_samples"}
    times = [r["monotonic_s"] for r in window]
    age = now - max(times)
    if age > max_age_s:
        return {"ok": False, "reason": "stale", "latest_age_s": round(age, 3)}
    if len(window) < min_samples:
        return {"ok": False, "reason": "too_few_samples", "n": len(window)}
    span = max(times) - min(times)
    if span < min_span_s:
        return {"ok": False, "reason": "short_span", "span_s": round(span, 3)}
    values = [int(r["steering_adc"]) for r in window]
    lo, hi = min(values), max(values)
    rail = lo <= 1 or hi >= 1022
    warnings = []
    if rail:
        warnings.append("rail_0_1_or_1022_1023")
    if hi - lo > range_warn_adc:
        warnings.append(f"wide_range_gt_{range_warn_adc}_adc")
    return {"ok": True, "n": len(values), "samples": values, "median_adc": statistics.median(values),
            "min_adc": lo, "max_adc": hi, "range_adc": hi - lo, "span_s": round(span, 3),
            "latest_age_s": round(age, 3), "rail": rail, "warnings": warnings,
            "window_s": window_s, "auto_still_judgement": None}


def make_mark(label, summary, epoch, existing_labels, confirmed, latest=None, host_kst=None):
    """Build one mark record or return (None, reason). Pure; no I/O."""
    if label not in LABELS:
        return None, "unknown_label"
    if not confirmed:
        return None, "operator_confirmation_unchecked"
    if label in existing_labels:
        return None, "duplicate_label_use_new_session"
    if not summary.get("ok"):
        return None, summary.get("reason", "window_not_ok")
    text, side = LABELS[label]
    return {
        "label": label, "label_ko": text, "approach_from": side, "epoch": epoch,
        "host_kst": host_kst or kst_now(),
        "operator_asserted_straight": True, "operator_asserted_still_3s": True,
        "angle_deg": None, "angle_measured": False,
        "steer_center_reported": None if latest is None else latest.get("steer_center_adc"),
        "actual_steering_output": "unverified_not_in_telemetry",
        "summary": summary, "warnings": summary["warnings"],
    }, None


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def write_json_atomic(path, data):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    os.replace(tmp, path)


class LineFeed:
    """Split incoming bytes into text lines; drops undecodable/overlong junk
    (raw bytes are preserved separately by the caller)."""

    def __init__(self, limit=4096):
        self.buffer = bytearray()
        self.limit = limit

    def feed(self, data):
        self.buffer.extend(data)
        lines = []
        while True:
            nl = self.buffer.find(b"\n")
            if nl < 0:
                if len(self.buffer) > self.limit:
                    del self.buffer[:-self.limit]
                return lines
            chunk = bytes(self.buffer[:nl])
            del self.buffer[:nl + 1]
            lines.append(chunk.decode("ascii", "replace").rstrip("\r"))


class Session:
    """Owns the fd and the evidence files. Serial is read-only; nothing is sent."""

    def __init__(self, port, output_dir, source_path, battery_off_confirmed):
        self.port, self.out = str(port), Path(output_dir)
        self.source = Path(source_path)
        self.fd = self.original = None
        self.raw = self.samples_f = self.marks_f = None
        self.feed = LineFeed()
        self.rows = deque()
        self.epoch = 0
        self.center_seen = None
        self.latest = None
        self.marks = []
        self.error = None
        self.epoch_reset_flag = False
        self.meta = {
            "schema_version": 1, "tool": "tools/steering_center_receiver.py", "status": "starting",
            "started_kst": kst_now(), "ended_kst": None, "port": self.port, "baud": BAUD,
            "battery_off_confirmed_by_operator": bool(battery_off_confirmed),
            "wheel_positioning": "by hand; no motor steering used",
            "read_only": True, "transmitted_bytes": 0,
            "os_serial_open_may_reset_board": True, "auto_reconnect": False,
            "reviewed_source": {"path": str(self.source), "sha256": None, "bytes": None,
                                "copy": None, "flash_identity": "unverified"},
            "actual_drive_steer_output": "unverified (not present in telemetry)",
            "window": {"window_s": WINDOW_S, "max_age_s": MAX_AGE_S, "min_samples": MIN_SAMPLES,
                       "min_span_s": MIN_SPAN_S, "range_warn_adc_display_only": RANGE_WARN_ADC},
            "bytes_received": 0, "samples": 0, "marks": 0, "epochs": 0, "boot_banners": 0,
            "center_changes": 0, "files": {"raw": "raw_rx.bin", "samples": "samples.jsonl",
                                           "marks": "marks.jsonl", "session": "session.json"},
        }

    # -- lifecycle -------------------------------------------------------
    def open(self):
        if self.out.exists():
            raise FileExistsError(f"output dir already exists; choose a NEW dir: {self.out}")
        if not self.source.is_file():
            raise FileNotFoundError(f"reviewed source not found: {self.source}")
        self.out.mkdir(parents=True, exist_ok=False)
        copy = "reviewed_source_copy" + (self.source.suffix or ".txt")
        shutil.copyfile(self.source, self.out / copy)
        self.meta["reviewed_source"].update(sha256=sha256_file(self.source),
                                            bytes=self.source.stat().st_size, copy=copy)
        self.raw = (self.out / "raw_rx.bin").open("xb")
        self.samples_f = (self.out / "samples.jsonl").open("x", encoding="utf-8")
        self.marks_f = (self.out / "marks.jsonl").open("x", encoding="utf-8")
        self.write_meta()
        self.fd, self.original = gnss_record.open_serial(self.port, BAUD)
        self.meta["termios_restore_on_close"] = False
        self.meta["status"] = "receiving"
        self.meta["port_opened_kst"] = kst_now()
        self.write_meta()

    def write_meta(self):
        write_json_atomic(self.out / "session.json", self.meta)

    def close(self, status=None):
        # Deliberately NO termios restore: the raw/no-echo/HUPCL-off line
        # settings from open_serial stay in place until the fd is closed, so
        # the kernel never echoes board output back as command bytes and the
        # close itself does not drop DTR. `self.original` is kept only as
        # evidence and is never applied.
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        for h in (self.raw, self.samples_f, self.marks_f):
            if h is not None:
                try:
                    h.flush()
                    os.fsync(h.fileno())
                finally:
                    h.close()
        self.raw = self.samples_f = self.marks_f = None
        if status:
            self.meta["status"] = status
        elif self.meta["bytes_received"] == 0 and self.meta["status"] != "error":
            self.meta["status"] = "no_data"
        elif self.meta["status"] == "receiving":
            self.meta["status"] = "completed"
        if self.error:
            self.meta["error"] = self.error
        self.meta["ended_kst"] = kst_now()
        if self.out.exists():
            self.write_meta()

    # -- receive ----------------------------------------------------------
    def poll(self):
        """Non-blocking read; returns True while the port is usable."""
        if self.fd is None:
            return False
        ready, _, _ = select.select([self.fd], [], [], 0)
        if not ready:
            return True
        try:
            data = os.read(self.fd, 65536)
        except BlockingIOError:
            return True
        except OSError as exc:
            return self.fail(f"serial read error: {exc}")
        if not data:
            return self.fail("serial device disconnected (EOF); no auto reconnect")
        now = time.monotonic()
        self.raw.write(data)
        self.raw.flush()
        self.meta["bytes_received"] += len(data)
        for line in self.feed.feed(data):
            self.handle_line(line, now)
        self.samples_f.flush()
        return True

    def fail(self, message):
        self.error = message
        self.meta["status"] = "error"
        try:
            os.close(self.fd)
        except OSError:
            pass
        self.fd = None
        return False

    def reset_epoch(self, why):
        self.epoch += 1
        self.rows.clear()
        self.epoch_reset_flag = True
        self.meta["epochs"] = self.epoch
        self.samples_f.write(json.dumps({"event": "epoch_reset", "epoch": self.epoch, "why": why,
                                         "host_kst": kst_now()}, ensure_ascii=False) + "\n")

    def handle_line(self, line, now):
        if BOOT_BANNER in line:
            self.meta["boot_banners"] += 1
            self.reset_epoch("boot_banner")
            return
        parsed = parse_monitor(line)
        if parsed is None:
            return
        if self.center_seen is not None and parsed["steer_center_adc"] != self.center_seen:
            self.meta["center_changes"] += 1
            self.reset_epoch(f"steer_center_changed_{self.center_seen}_to_{parsed['steer_center_adc']}")
        self.center_seen = parsed["steer_center_adc"]
        row = {"monotonic_s": now, "host_kst": kst_now(), "epoch": self.epoch, **parsed}
        self.latest = row
        self.rows.append(row)
        while self.rows and now - self.rows[0]["monotonic_s"] > WINDOW_S + 1.0:
            self.rows.popleft()
        self.meta["samples"] += 1
        self.samples_f.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")

    def age_s(self):
        return None if self.latest is None else time.monotonic() - self.latest["monotonic_s"]

    # -- marks --------------------------------------------------------------
    def receiving(self):
        return self.fd is not None and self.error is None and self.meta["status"] == "receiving"

    def mark(self, label, confirmed):
        if not self.receiving():
            # Connection state first: fresh-looking samples after a
            # disconnect/error must never become a record.
            return None, "connection_error" if self.error else "not_receiving"
        summary = summarize_window(self.rows, time.monotonic())
        record, reason = make_mark(label, summary, self.epoch, {m["label"] for m in self.marks},
                                   confirmed, self.latest)
        if record is None:
            return None, reason
        self.marks.append(record)
        self.meta["marks"] = len(self.marks)
        self.marks_f.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self.marks_f.flush()
        self.write_meta()
        return record, None


class WindowController:
    """Tk widgets + one non-blocking tick. Built by run_window; tests build it
    on their own root and call tick() directly (no timers needed)."""

    def __init__(self, root, session):
        import tkinter as tk  # imported here only: module import never touches Tk
        self.root, self.session, self.alive = root, session, True
        root.title("조향 ADC 수신 창 — 배터리 OFF, 수신 전용 (송신 0)")
        big, mid, small = ("TkDefaultFont", 44, "bold"), ("TkDefaultFont", 14), ("TkDefaultFont", 10)
        self.v_raw = tk.StringVar(value="--")
        self.v_filt = tk.StringVar(value="POT(필터) --")
        self.v_age = tk.StringVar(value="마지막 수신 --")
        self.v_status = tk.StringVar(value="포트 열림. 수신 대기 (OS 포트 열기로 보드 리셋 가능)")
        self.v_marks = tk.StringVar(value="기록 0/5")
        self.confirmed = tk.BooleanVar(value=False)
        tk.Label(root, text="배터리 OFF, 손으로 위치 조절. 같은 직진 기준에 맞춘 뒤 3초 정지하고 기록.",
                 font=mid, wraplength=640, justify="left").pack(padx=12, pady=(10, 2), anchor="w")
        tk.Label(root, text="좌/우 = 직진에 접근해 온 방향 (조향각 명령 아님). 각도는 독립 측정 없음, "
                 "직진은 작업자 주장(operator_asserted_straight)으로만 기록.",
                 font=small, wraplength=640, justify="left", fg="#555").pack(padx=12, anchor="w")
        tk.Label(root, text="POT_RAW (ADC)", font=mid).pack(pady=(8, 0))
        self.l_raw = tk.Label(root, textvariable=self.v_raw, font=big, fg="#003366")
        self.l_raw.pack()
        tk.Label(root, textvariable=self.v_filt, font=mid).pack()
        self.l_age = tk.Label(root, textvariable=self.v_age, font=mid)
        self.l_age.pack()
        tk.Label(root, textvariable=self.v_status, font=mid, fg="#8a4b00", wraplength=640).pack(pady=(4, 6))
        self.check = tk.Checkbutton(root, text="같은 직진 기준에 맞췄고 3초 이상 정지했음 (기록마다 다시 체크)",
                                    variable=self.confirmed, font=mid)
        self.check.pack(pady=4)
        frame = tk.Frame(root)
        frame.pack(padx=12, pady=6)
        self.buttons = {}
        for i, (label, (text, _side)) in enumerate(LABELS.items()):
            b = tk.Button(frame, text=text, font=mid, width=28, command=lambda l=label: self.on_mark(l))
            if i == 0:
                b.grid(row=0, column=0, columnspan=2, padx=4, pady=3)
            else:
                b.grid(row=(i + 1) // 2, column=(i - 1) % 2, padx=4, pady=3)
            self.buttons[label] = b
        tk.Label(root, textvariable=self.v_marks, font=mid).pack()
        tk.Label(root, text=f"출력: {session.out}", font=("TkDefaultFont", 9), fg="#555").pack(pady=(0, 8))
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_mark(self, label):
        record, reason = self.session.mark(label, self.confirmed.get())
        if record is None:
            self.v_status.set(f"기록 거부 [{LABELS[label][0]}]: {reason}")
            return
        self.confirmed.set(False)
        s = record["summary"]
        warn = f" 경고={','.join(s['warnings'])}" if s["warnings"] else ""
        self.v_status.set(f"기록됨 [{record['label_ko']}] epoch{record['epoch']} n={s['n']} "
                          f"median={s['median_adc']} 범위={s['min_adc']}~{s['max_adc']} ({s['range_adc']} ADC){warn}")
        self.v_marks.set(f"기록 {len(self.session.marks)}/5")
        self.buttons[label].config(state="disabled")

    def disable_marking(self):
        self.confirmed.set(False)
        self.check.config(state="disabled")
        for b in self.buttons.values():
            b.config(state="disabled")

    def tick(self):
        """One poll + display refresh. Returns False once the port is gone."""
        session = self.session
        alive = session.poll()
        if session.epoch_reset_flag:
            session.epoch_reset_flag = False
            self.confirmed.set(False)
            self.v_status.set(f"부팅 문구/STEER_CENTER 변경 감지 → 버퍼 초기화, epoch={session.epoch}. 3초 다시 정지 후 기록.")
        latest, age = session.latest, session.age_s()
        if latest is not None:
            stale = age > MAX_AGE_S
            self.v_raw.set(str(latest["steering_adc"]) + (" (STALE)" if stale else ""))
            self.l_raw.config(fg="#888888" if stale else "#003366")
            self.v_filt.set(f"POT(필터) {latest['steering_filtered']:.2f}  MODE={latest['mode']}  "
                            f"STEER_CENTER={latest['steer_center_adc']}  epoch={latest['epoch']}")
            self.v_age.set(f"마지막 수신 {age:.2f}초 전" + (f" — STALE ({MAX_AGE_S}초 초과, 현재값 아님)" if stale else "")
                           + f"  표본(창) {len(session.rows)}  수신 {session.meta['bytes_received']}B")
            self.l_age.config(fg="#b00020" if stale else "black")
        elif session.meta["bytes_received"] == 0:
            self.v_age.set("수신 0바이트 — 포트/보드 전원(USB)/케이블 확인")
        else:
            self.v_age.set(f"수신 {session.meta['bytes_received']}B, 아직 모니터 행 없음")
        if not alive and self.alive:
            self.alive = False
            self.disable_marking()
            self.v_status.set(f"오류: {session.error} — 기록 불가(재접속 없음). 창을 닫으면 지금까지의 기록이 저장됩니다.")
            self.v_raw.set("ERR")
        return self.alive

    def on_close(self):
        try:
            self.session.close()
        finally:
            self.root.destroy()


def run_window(session):
    import tkinter as tk  # imported here only: module import never touches Tk

    root = tk.Tk()
    ctrl = WindowController(root, session)

    def loop():
        if ctrl.tick():
            root.after(50, loop)

    root.after(50, loop)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        ctrl.on_close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--receive", action="store_true", required=True, help="receive-only mode (the only mode)")
    ap.add_argument("--battery-off-confirmed", action="store_true", required=True,
                    help="operator declares vehicle battery is OFF (declared, not measured)")
    ap.add_argument("--reviewed-source", required=True, help="source file the operator says is on the board; copied+hashed only")
    ap.add_argument("--output-dir", required=True, help="NEW directory for raw_rx.bin/samples.jsonl/marks.jsonl/session.json")
    ap.add_argument("--port", required=True, help="serial device, preferably /dev/serial/by-id/...")
    args = ap.parse_args(argv)
    session = Session(args.port, args.output_dir, args.reviewed_source, args.battery_off_confirmed)
    try:
        session.open()
    except (OSError, ValueError) as exc:
        print(f"cannot start receiver: {exc}", file=sys.stderr)
        if session.out.exists() and session.raw is not None:
            session.error = str(exc)
            session.close("error")
        return 1
    print(f"receive-only window; port={args.port} out={session.out} tx=0 (OS open may have reset the board)", flush=True)
    try:
        run_window(session)
    finally:
        if session.raw is not None:
            session.close()
    print(f"finished: {session.meta['status']} bytes={session.meta['bytes_received']} "
          f"samples={session.meta['samples']} marks={session.meta['marks']} -> {session.out / 'session.json'}")
    return 0 if session.meta["status"] in ("completed",) else 1


if __name__ == "__main__":
    sys.exit(main())
