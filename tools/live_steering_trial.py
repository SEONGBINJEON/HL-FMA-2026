#!/usr/bin/env python3
"""Thin live executor: C2.1 target-angle host -> real type150 board. STEERING ONLY.

Reuses, without re-implementing: steer_target_host.TargetCore/HostSteerAdapter
(C1.1 core via ctypes), joystick_protocol.Parser (type150 status), step_manual_control.Client
(bounded TX/RX, stall checks) and encode_zero_pwm (type2 command with drive PWM 0).
Every command has drive=0 and PWM=0; there is no drive, gamepad or GPS path here.

Modes
  (default) / --preflight   no device: calibration, library, angle range, port presence.
                            Null/unmeasured or synthetic calibration => BLOCKED, exit 2.
  --model                   same executor on the board MODEL + synthetic calibration
                            (steer_target_sim). Writes under logs/, never data/.
  --identify --port P --output-dir NEW [--seconds 3]
                            receive-only: reports type150 frames/boot/outputs or ASCII markers.
                            No Client is built, so nothing can be transmitted (post-upload check).
  --live --port P --output-dir NEW
                            real board. Refused before the port is opened unless preflight
                            is READY. Nothing is sent until a CRC-valid type150 status
                            identifies the board; ASCII/other/unidentified => exit 4, TX 0.

Operator keys (terminal, non-blocking): p prepare (neutral->ARM handshake),
l/r target +/-ANGLE deg (left front wheel, + left), 0 target 0 deg of the TABLE (not a
verified straight), s/space stop (DISARM, lock), q quit (stop, wait for DISARMED ack).
A target is a FINITE hold: it is re-issued with new sequences only for --max-hold-s after
the last key press for it; then the executor stops explicitly. Any fault, reboot, stale
status, disconnect or stop locks the adapter; only 'p' then a new key moves again.

Serial line settings (raw, no echo, HUPCL off) are kept until the fd closes; the previous
termios is NOT restored on close (receiver review 2026-09-18).
"""
import argparse, fcntl, hashlib, json, math, os, select, struct, sys, termios, time
from dataclasses import asdict
from datetime import datetime, timezone, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import steer_target_host as h  # noqa: E402
import steering_calibration as sc  # noqa: E402
import step_manual_control as smc  # noqa: E402
from step_manual_control import Client  # noqa: E402
from step_manual_protocol import DISARM, ARM, CONTROL, DISARMED, ARMED  # noqa: E402

KST = timezone(timedelta(hours=9), "KST")
DEFAULT_CAL = ROOT / "config/steering_calibration.json"
SYNTH_CAL = ROOT / "tests/fixtures/steering_calibration_synthetic.json"
ASCII_MARKERS = (b"F8 PID HILL HOLD READY", b"[MONITOR] ENC=", b"HENES F8", b"[MONITOR] Drive=")
EXIT_OK, EXIT_CONN, EXIT_BLOCKED, EXIT_NO_ACK, EXIT_UNIDENTIFIED = 0, 1, 2, 3, 4
CODE_FILES = ("tools/live_steering_trial.py", "tools/steer_target_host.py", "tools/step_manual_control.py",
              "tools/step_manual_protocol.py", "tools/joystick_protocol.py", "tools/steering_calibration.py",
              "firmware/steer_target/host/build/libsteer_host.so")


def kst_now():
    return datetime.now(KST).isoformat(timespec="milliseconds")


def sha256_file(path):
    hh = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            hh.update(chunk)
    return hh.hexdigest()


def zero_drive_encoder(command):
    """type150 command with drive PWM 0; refuses any non-zero drive request (Client then fails safe)."""
    if command.drive != 0:
        raise ValueError("drive must be 0 in the steering-only executor")
    return h.encode_zero_pwm(command)


# ----------------------------------------------------------------------------- units
def deg_to_cdeg(angle_deg):
    """Left-front-wheel degrees (+ left) -> int centidegrees; ValueError when not finite/int16."""
    if isinstance(angle_deg, bool) or not isinstance(angle_deg, (int, float)) or not math.isfinite(angle_deg):
        raise ValueError("angle must be a finite number of degrees")
    cdeg = int(round(angle_deg * 100))
    if not (-32768 <= cdeg <= 32767):
        raise ValueError("angle out of int16 centidegrees")
    return cdeg


def angle_in_table(validated_cfg, cdeg):
    """'' when the validated calibration maps the angle inside the usable ADC range, else the reason."""
    adc = sc.angle_to_adc(validated_cfg["points"], cdeg)
    if adc is None:
        return f"angle {cdeg} cdeg outside the measured table"
    if not (validated_cfg["usable_adc_min"] <= adc <= validated_cfg["usable_adc_max"]):
        return f"angle {cdeg} cdeg -> adc {adc} outside usable {validated_cfg['usable_adc_min']}..{validated_cfg['usable_adc_max']}"
    return ""


# ----------------------------------------------------------------------------- preflight
def preflight(cal_path, angle_deg, port=None, allow_synthetic=False):
    """Device-free readiness report. Returns dict with status READY|BLOCKED and concrete 'missing'."""
    missing, info = [], {"calibration": str(cal_path), "hardware_access": False}
    try:
        record = sc.load(cal_path)
        vcfg, reasons = sc.validate(record, allow_synthetic=allow_synthetic)
    except (OSError, ValueError) as exc:
        record, vcfg, reasons = None, None, [f"cannot load calibration: {exc}"]
    info["calibration_status"] = (record or {}).get("status") if isinstance(record, dict) else None
    info["calibration_synthetic"] = bool((record or {}).get("synthetic")) if isinstance(record, dict) else None
    missing += [f"calibration: {r}" for r in reasons]
    if vcfg is not None and vcfg.get("synthetic") and not allow_synthetic:
        missing.append("calibration: synthetic record is never accepted for the live path")
    try:
        cdeg = deg_to_cdeg(angle_deg); info["angle_cdeg"] = cdeg
        if vcfg is not None:
            bad = angle_in_table(vcfg, cdeg)
            if bad: missing.append("angle: " + bad)
            bad0 = angle_in_table(vcfg, 0)
            if bad0: missing.append("angle: table straight (0 cdeg) " + bad0)
    except ValueError as exc:
        missing.append(f"angle: {exc}")
    if not h.LIB_PATH.is_file():
        missing.append(f"library: {h.LIB_PATH.relative_to(ROOT)} missing; build with `{h.BUILD_CMD}`")
    else:
        try:
            binfo = json.loads((h.LIB_PATH.parent / "build_info.json").read_text())
            stale = [p for p, hs in binfo["sources_sha256"].items() if not (ROOT / p).is_file() or sha256_file(ROOT / p) != hs]
            if stale: missing.append("library: built from different sources: " + ", ".join(stale))
            actual = sha256_file(h.LIB_PATH); info["library_sha256"] = actual
            if actual != binfo.get("library_sha256"):
                missing.append(f"library: {h.LIB_PATH.name} sha256 {actual[:12]}… differs from build_info.library_sha256 "
                               f"{str(binfo.get('library_sha256'))[:12]}…; rebuild with `{h.BUILD_CMD}`")
        except (OSError, ValueError, KeyError) as exc:
            missing.append(f"library: build_info unreadable ({exc})")
    if port is not None:
        p = Path(port); info["port"] = str(p)
        if not (p.is_symlink() or p.exists()):
            missing.append(f"port: {p} not present (stat only; device not opened)")
    info["board_protocol_required"] = "type150 (firmware/joystick_manual); current board reported as ASCII hill hold -> not usable"
    if vcfg is not None and not missing:
        info["validated"] = {k: vcfg[k] for k in ("usable_adc_min", "usable_adc_max", "steer_sign_for_adc_increase",
                                                  "command_stale_ms", "sensor_stale_ms", "reversal_neutral_ms", "limit_policy", "synthetic")}
        info["validated"]["points"] = vcfg["points"]
    return {"status": "READY" if not missing else "BLOCKED", "live_capable": not missing, "missing": missing,
            "checked_kst": kst_now(), **info}


# ----------------------------------------------------------------------------- transport
class PosixSerialTransport:
    """Nonblocking POSIX serial for Client. Same line settings as step_manual_control.MegaTransport
    (raw, 115200, 8N1, CLOCAL, no HUPCL) but: (1) the path is a parameter so a PTY can stand in for
    tests, (2) close() never restores the previous termios, so kernel echo/HUPCL are not re-enabled
    while the board is still talking (receiver review finding), (3) with check_identity the USB
    VID/PID/serial must match the recorded Mega before and after opening."""

    def __init__(self, path, check_identity=True):
        self.path = Path(path); self.fd = None
        if check_identity:
            if not self.path.is_symlink():
                raise RuntimeError(f"Mega by-id path missing: {self.path}")
            rdev, ident = smc._device_identity(self.path)
            if ident != smc.MEGA_ID:
                raise RuntimeError("Mega VID/PID/serial mismatch")
        try:
            self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if check_identity:
                rdev2, ident2 = smc._device_identity(self.path)
                if os.fstat(self.fd).st_rdev != rdev or rdev2 != rdev or ident2 != smc.MEGA_ID:
                    raise RuntimeError("Mega identity changed while opening")
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = attrs[1] = attrs[3] = 0                      # raw input/output, no ECHO/ICANON/ISIG
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8  # HUPCL deliberately off
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except BaseException:
            self.close(); raise

    def _queue(self, code):
        return struct.unpack("I", fcntl.ioctl(self.fd, code, struct.pack("I", 0)))[0]

    @property
    def in_waiting(self): return self._queue(termios.FIONREAD)

    @property
    def out_waiting(self): return self._queue(termios.TIOCOUTQ)

    def read(self, size):
        try:
            return os.read(self.fd, size)
        except BlockingIOError:
            return b""

    def write(self, data): return os.write(self.fd, data)

    def close(self):
        if self.fd is not None:
            os.close(self.fd); self.fd = None    # no termios restore, on purpose


# ----------------------------------------------------------------------------- journal
class TrialJournal:
    """events.jsonl + rx.bin + tx.bin + metadata.json. Duck-typed for Client (event/bytes).
    Also watches raw RX for ASCII firmware markers (the hill-hold board)."""

    def __init__(self, out_dir, data_kind, metadata):
        self.path = Path(out_dir)
        if self.path.exists():
            raise FileExistsError(f"output dir exists; choose a NEW dir: {self.path}")
        self.path.mkdir(parents=True, exist_ok=False)
        self.events = (self.path / "events.jsonl").open("x", encoding="utf-8")
        self.rx = (self.path / "rx.bin").open("xb"); self.tx = (self.path / "tx.bin").open("xb")
        self.rx_bytes = self.tx_bytes = 0; self.tx_frames = 0
        self.ascii_tail = b""; self.ascii_marker = None
        self.metadata = {"data_kind": data_kind, "started_kst": kst_now(), **metadata}
        self.write_metadata()

    def write_metadata(self):
        tmp = self.path / "metadata.json.tmp"
        tmp.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        os.replace(tmp, self.path / "metadata.json")

    def event(self, name, **fields):
        row = {"host_kst": kst_now(), "host_monotonic": time.monotonic(), "event": name, **fields}
        self.events.write(json.dumps(row, ensure_ascii=False, default=str) + "\n"); self.events.flush()

    def bytes(self, direction, data):
        if direction == "rx":
            self.rx.write(data); self.rx.flush(); self.rx_bytes += len(data)
            self.ascii_tail = (self.ascii_tail + data)[-4096:]
            for m in ASCII_MARKERS:
                if m in self.ascii_tail and self.ascii_marker is None:
                    self.ascii_marker = m.decode(); self.event("ascii_firmware_marker", marker=self.ascii_marker)
        else:
            self.tx.write(data); self.tx.flush(); self.tx_bytes += len(data); self.tx_frames += 1
        self.event(direction, count=len(data), hex=data.hex())

    def close(self, **final):
        self.metadata.update(ended_kst=kst_now(), rx_bytes=self.rx_bytes, tx_bytes=self.tx_bytes, tx_frames=self.tx_frames, **final)
        self.write_metadata()
        for s in (self.events, self.rx, self.tx):
            try:
                s.flush(); os.fsync(s.fileno())
            finally:
                s.close()


# ----------------------------------------------------------------------------- executor
class LiveTrial:
    """One assembled executor: Parser(type150) -> HostSteerAdapter(TargetCore) -> Client(zero-PWM encoder).
    The same class runs on the real transport, on a PTY and on the board model; only transport/clock differ."""

    PHASES_PREPARED = ("ready", "active")

    # Stall guard defaults, from stored records (see docs/handoff/CLAUDE1_STALL_GUARD_10DEG_2026-09-18.md):
    #   * normal travel with output active: 105..346 ADC/s (2026-09-18 08:44 target trials)
    #   * longest run with output active and NO motion: 1.00 s, ADC spread 1 (08:11 left/right trial,
    #     static friction before the wheel broke away) -> the window must be clearly longer than that
    #   * stationary ADC noise: 0..3 ADC over a 10 s window (2026-09-18 recenter session, 17 windows)
    # 1.5 s is 1.5x the longest observed start delay; 8 ADC is ~2.7x the largest stationary spread and is
    # reached in <0.08 s even at the slowest observed travel speed, so normal motion cannot trip it.
    STALL_WINDOW_S = 1.5
    STALL_MIN_PROGRESS_ADC = 8

    def __init__(self, lib, cfg, validated, transport, journal, angle_cdeg, max_hold_s=3.0,
                 identify_timeout_s=3.0, ack_wait_s=0.4, refresh_s=None,
                 stall_window_s=STALL_WINDOW_S, stall_min_progress_adc=STALL_MIN_PROGRESS_ADC):
        self.core = h.TargetCore(lib, cfg)
        self.adapter = h.HostSteerAdapter(self.core)
        self.client = Client(transport, self.adapter, journal, encoder=zero_drive_encoder)
        from joystick_protocol import Parser   # type150 parser, never the type144 default
        self.client.parser = Parser()
        self.transport, self.journal, self.validated = transport, journal, validated
        self.angle_cdeg, self.max_hold_s, self.identify_timeout_s, self.ack_wait_s = angle_cdeg, max_hold_s, identify_timeout_s, ack_wait_s
        stale = validated["command_stale_ms"] / 1000.0 if validated else 0.3
        self.refresh_s = refresh_s if refresh_s is not None else min(0.2, stale / 2)
        self.seq = 0; self.target_cdeg = None; self.hold_until = None; self.last_issue = None
        self.started_at = None; self.last_tick_log = None
        self.stop_attempts = 0; self.current_stop = None; self.stop_history = []; self.quit_since = None; self.last_now = 0.0
        self.last_stop_reason = None; self._announced = None
        self.finished = False; self.exit_code = None; self.exit_reason = None
        self.stall_window_s, self.stall_min_progress_adc = stall_window_s, stall_min_progress_adc
        self._stall = None            # {target, since, ref_adc, ref_req} while output is active
        self.stall_blocked = None     # set once when the guard fires; never auto-cleared
        self.events = []   # (now, text) for the terminal line / tests

    # ---- helpers
    def prepared(self):
        return self.adapter.phase in self.PHASES_PREPARED and not self.adapter.locked_by_fault

    def note(self, now, text):
        self.events.append((now, text)); self.journal.event("operator", now_s=now, text=text)

    def finish(self, code, reason, now):
        if self.finished: return
        self.finished, self.exit_code, self.exit_reason = True, code, reason
        self.note(now, f"finish exit={code}: {reason}")

    # ---- operator / planner inputs (a path follower may call command_target with its own angle)
    def key(self, k, now):
        if self.finished or self.quit_since is not None: return
        if k == "p":
            ok = self.adapter.request_prepare(now)
            if ok: self._retire_current_stop("re-prepared: earlier stop ack does not cover new motion")
            self.note(now, f"prepare -> {ok} ({self.adapter.message})")
        elif k in ("l", "r", "0"):
            angle = {"l": self.angle_cdeg, "r": -self.angle_cdeg, "0": 0}[k]
            self.command_target(angle, now, self.max_hold_s)
        elif k in ("s", " "):
            self.stop_motion("operator stop", now)
        elif k == "q":
            self.quit(now)

    @staticmethod
    def check_target_request(angle_cdeg, now, hold_s):
        """'' when the public target request is well-formed, else the reason. Checked before any core/adapter call:
        angle int (not bool) within int16; now finite non-negative; hold_s finite and > 0."""
        if isinstance(angle_cdeg, bool) or not isinstance(angle_cdeg, int) or not (-32768 <= angle_cdeg <= 32767):
            return "angle_cdeg must be int16 (no bool/float/wrap)"
        for name, v, positive in (("now", now, False), ("hold_s", hold_s, True)):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                return f"{name} must be a finite number"
            if v < 0 or (positive and v <= 0):
                return f"{name} must be {'> 0' if positive else '>= 0'}"
        return ""

    def command_target(self, angle_cdeg, now, hold_s):
        """Public target input (keys and a path follower share it). Set/extend a finite target hold.

        - well-formed, in-table target while prepared  -> accepted, one sequence issued now
        - well-formed target while NOT prepared          -> refused, nothing moves, no lock (no motion to cancel)
        - malformed request or angle outside table/usable -> C2.1 policy: the previous target and hold window are
          cancelled, the executor stops (DISARM) and locks; only 'p' + a new target resumes.
        """
        bad = self.check_target_request(angle_cdeg, now, hold_s)
        if bad:
            safe_now = now if isinstance(now, (int, float)) and not isinstance(now, bool) and math.isfinite(now) and now >= 0 else self.last_now
            self.note(safe_now, f"target request invalid ({bad}): cancel target, stop, lock")
            self.stop_motion(f"invalid target request: {bad}", safe_now); return False
        if not self.prepared():
            self.note(now, f"target {angle_cdeg} cdeg refused: not prepared ({self.adapter.phase}); nothing moves"); return False
        bad = angle_in_table(self.validated, angle_cdeg) if self.validated else "no validated calibration"
        if bad:
            self.note(now, f"target out of range ({bad}): cancel target, stop, lock")
            self.stop_motion(f"target out of range: {bad}", now); return False
        self.target_cdeg = angle_cdeg; self.hold_until = now + hold_s
        return self._issue(now)

    def _issue(self, now):
        self.seq += 1
        ok = self.adapter.set_target(self.seq, self.target_cdeg, int(now * 1000) & 0xFFFFFFFF, now)
        self.last_issue = now
        if not ok:
            self.note(now, f"seq {self.seq} angle {self.target_cdeg} not accepted: {self.adapter.first_fault or self.adapter.message}")
            self.target_cdeg = self.hold_until = None
        elif self._announced != (self.target_cdeg, self.hold_until):
            self._announced = (self.target_cdeg, self.hold_until)
            self.note(now, f"target seq {self.seq} angle {self.target_cdeg} cdeg accepted; hold until {self.hold_until:.2f}s")
        return ok

    @property
    def stop_ack_observed(self):
        """Ack of the CURRENT stop attempt only (None attempt -> False). History lives in stop_history."""
        return bool(self.current_stop and self.current_stop.get("ack_req") is not None)

    def _retire_current_stop(self, why):
        if self.current_stop is not None:
            self.current_stop["retired_by"] = why
            self.stop_history.append(self.current_stop); self.current_stop = None

    def stop_motion(self, reason, now):
        """Explicit stop: adapter lock + DISARM attempt for the latest identified request. Starts a NEW stop
        attempt record; any earlier ack is history, never the confirmation of this one."""
        self.target_cdeg = self.hold_until = None
        self.adapter.stop(reason)
        cmd = self.adapter.emergency_command()
        self._retire_current_stop("new stop attempt")
        self.last_stop_reason = reason
        if cmd is not None and not self.client.failed:
            self.stop_attempts += 1
            self.client.send(cmd)
            self.current_stop = {"reason": reason, "attempted_at_s": round(now, 3), "boot": cmd.boot_id, "req": cmd.request_id,
                                 "sent": not self.client.failed, "ack_req": None, "ack_at_s": None}
            self.note(now, f"stop '{reason}': DISARM attempted for req {cmd.request_id}" + (" (send failed)" if self.client.failed else ""))
        else:
            self.current_stop = {"reason": reason, "attempted_at_s": round(now, 3), "boot": None, "req": None, "sent": False,
                                 "ack_req": None, "ack_at_s": None, "note": "no identified board request; nothing sent"}
            self.note(now, f"stop '{reason}': no identified board request, nothing sent")

    def _check_stall_guard(self, now):
        """Cut the output when steering is commanded but the measured ADC does not move toward the target.

        Watched ONLY while a target is live and the core is actually driving the steering. Output 0 is not
        watched, which separates the three benign cases the operator asked to keep: already AT_TARGET, the
        neutral gap during a sign reversal, and not-yet-prepared. Re-issuing the SAME target with a new
        sequence does not restart the window; only real progress of at least stall_min_progress_adc toward
        the target moves the reference point. Progress away from the target counts as no progress, so a
        wrong-direction run cannot keep the guard alive. When it fires it goes through the normal
        stop_motion path (output off, DISARM attempt, target cancelled, adapter locked) and never retries,
        never widens a band and never records the stalled position as a limit.
        """
        ad = self.adapter
        t = ad.trace[-1] if ad.trace else None
        s = ad.status
        if self.target_cdeg is None or t is None or s is None or not t.core_steer:
            self._stall = None
            return
        adc, tgt = s.steering_adc, t.target_adc
        toward = 1 if tgt >= adc else -1
        st = self._stall
        if st is None or st.get("direction") != toward:
            self._stall = {"target": self.target_cdeg, "direction": toward, "since": now, "ref_adc": adc, "ref_req": s.request_id}
            return
        waited = now - st["since"]
        if s.request_id == st["ref_req"] and waited < self.stall_window_s:
            return                     # no fresh status frame yet; the clock keeps running
        progress = (adc - st["ref_adc"]) * toward
        if progress >= self.stall_min_progress_adc:
            st["since"], st["ref_adc"], st["ref_req"] = now, adc, s.request_id
            return
        if waited >= self.stall_window_s:
            detail = (f"steering output {t.core_steer:+d} held {waited:.2f}s but the measured adc moved "
                      f"{progress:+d} toward the target (needs >= {self.stall_min_progress_adc})")
            self.stall_blocked = {"at_s": round(now, 3), "target_cdeg": self.target_cdeg, "target_adc": tgt,
                                  "adc": adc, "ref_adc": st["ref_adc"], "progress_adc": progress,
                                  "waited_s": round(waited, 2), "core_steer": t.core_steer}
            self.note(now, "stall guard: " + detail)
            self._stall = None
            self.stop_motion("stall: " + detail, now)

    def quit(self, now):
        self.stop_motion("operator quit", now); self.quit_since = now

    # ---- one non-blocking step; returns False when finished
    def tick(self, now):
        if self.finished: return False
        if self.started_at is None: self.started_at = now
        self.last_now = now
        ad = self.adapter
        self.client.tick(now)
        if self.client.failed:
            self.finish(EXIT_CONN, f"connection failed: {ad.message} (no stop ack for this session end; no reconnect)", now); return False
        if not ad.identified:
            if self.journal.ascii_marker:
                self.finish(EXIT_UNIDENTIFIED, f"ASCII firmware on the port ({self.journal.ascii_marker}); type150 required; TX 0", now); return False
            if now - self.started_at >= self.identify_timeout_s:
                self.finish(EXIT_UNIDENTIFIED, "no CRC-valid type150 status; board unidentified; TX 0", now); return False
        # finite hold: refresh with new sequences while prepared, stop explicitly when the window ends
        if self.target_cdeg is not None:
            if ad.locked_by_fault or not self.prepared():
                self.note(now, f"target dropped: adapter {ad.phase} ({ad.first_fault or ad.message})")
                self.target_cdeg = self.hold_until = None
            elif now >= self.hold_until:
                self.stop_motion("hold window expired", now)
            elif now - self.last_issue >= self.refresh_s:
                self._issue(now)
        self._check_stall_guard(now)
        # stop acknowledgement for the CURRENT attempt only: a newer DISARMED status of the same boot after our DISARM
        cs = self.current_stop
        if cs and cs["sent"] and cs["ack_req"] is None and ad.status is not None:
            if ad.status.boot_id == cs["boot"] and ad.status.request_id > cs["req"] and ad.status.state == DISARMED:
                cs["ack_req"], cs["ack_at_s"] = ad.status.request_id, round(now, 3)
                self.note(now, f"stop ack observed for '{cs['reason']}': DISARMED at req {ad.status.request_id}")
            elif ad.status.boot_id != cs["boot"]:
                cs["note"] = "board rebooted before ack; ack impossible for this attempt"
        if self.quit_since is not None and (self.stop_ack_observed or now - self.quit_since >= self.ack_wait_s
                                            or ad.status is None):
            if self.stop_ack_observed: self.finish(EXIT_OK, "quit; final DISARMED status observed for this stop", now)
            elif cs and cs["sent"]: self.finish(EXIT_NO_ACK, "quit; DISARM attempted but no DISARMED status observed for this stop", now)
            else: self.finish(EXIT_OK if not ad.identified else EXIT_NO_ACK, "quit; nothing to stop (no identified request)", now)
            return False
        if self.last_tick_log is None or now - self.last_tick_log >= 0.1:
            self.last_tick_log = now; self.journal.event("tick", **self.snapshot(now))
        return True

    def snapshot(self, now):
        ad, s = self.adapter, self.adapter.status
        t = ad.trace[-1] if ad.trace else None
        return {"now_s": round(now, 3), "phase": ad.phase, "locked": ad.locked_by_fault, "identified": ad.identified,
                "target_cdeg": self.target_cdeg, "seq": self.seq, "hold_left_s": None if self.hold_until is None else round(self.hold_until - now, 2),
                "core_reason": t.core_reason if t else None, "core_steer": t.core_steer if t else None, "target_adc": t.target_adc if t else None,
                "board": None if s is None else {"boot": s.boot_id, "req": s.request_id, "state": s.state, "reason": s.reason,
                                                 "adc": s.steering_adc, "req_steer": s.requested_steer, "act_steer": s.actual_steer,
                                                 "req_drive": s.requested_drive, "act_drive": s.actual_drive},
                "first_fault": ad.first_fault, "message": ad.message, "tx_frames": self.journal.tx_frames, "drive_pwm": 0,
                "current_stop": self.current_stop, "stop_history_n": len(self.stop_history),
                "stall_watch_s": None if self._stall is None else round(now - self._stall["since"], 2),
                "stall_blocked": self.stall_blocked}

    def status_line(self, now):
        sn = self.snapshot(now); b = sn["board"]
        bd = "board -" if not b else f"boot{b['boot']} req{b['req']} {'ARMED' if b['state'] else 'DISARMED'}/{h.BOARD_REASONS.get(b['reason'], b['reason'])} adc={b['adc']} steer={b['act_steer']:+d}"
        tgt = "-" if sn["target_cdeg"] is None else f"{sn['target_cdeg'] / 100:+.2f}deg hold {sn['hold_left_s']}s"
        return (f"[{now - (self.started_at or now):6.1f}s] {sn['phase']:<15} {bd} | target {tgt} core={sn['core_reason']}/{sn['core_steer']} "
                f"tx={sn['tx_frames']} | {(sn['first_fault'] or sn['message'])[:48]}")


# ----------------------------------------------------------------------------- assembly
def build_metadata(mode, cal_path, pre, angle_cdeg, extra, argv=None, stall=None):
    return {"tool": "tools/live_steering_trial.py", "mode": mode, "steering_only": True, "drive_pwm": 0,
            "stall_guard": stall,
            "argv": list(sys.argv[1:] if argv is None else argv), "program": sys.argv[0], "cwd": os.getcwd(), "python": sys.version.split()[0],
            "calibration_path": str(cal_path), "calibration_sha256": sha256_file(cal_path) if Path(cal_path).is_file() else None,
            "calibration_validated": pre.get("validated"), "angle_cdeg": angle_cdeg,
            "code_sha256": {p: (sha256_file(ROOT / p) if (ROOT / p).is_file() else None) for p in CODE_FILES},
            "termios_restore_on_close": False, "auto_reconnect": False, "physical_motion_verified": False, **extra}


def assemble(lib, cal_path, angle_cdeg, transport, journal, allow_synthetic=False, **kw):
    """The one assembly used by --live, --model and the tests. Refuses before touching the core when
    the calibration is not READY (or is synthetic without allow_synthetic)."""
    cfg, reasons, syn = h.load_calibration(Path(cal_path), allow_synthetic=allow_synthetic)
    if cfg is None or (syn and not allow_synthetic):
        raise ValueError("calibration not usable: " + "; ".join(reasons or ["synthetic refused"]))
    validated = sc.validate(sc.load(cal_path), allow_synthetic=allow_synthetic)[0]
    return LiveTrial(lib, cfg, validated, transport, journal, angle_cdeg, **kw)


def identify(transport, journal, seconds, clock=time.monotonic, sleep=time.sleep):
    """Receive-only board identification (post-upload check). Never constructs a Client, so no byte
    can be written. Reports CRC-valid type150 statuses (boot/state/outputs) or ASCII markers."""
    from joystick_protocol import Parser
    parser = Parser(); t0 = clock(); statuses = []; errors = 0
    while clock() - t0 < seconds:
        n = transport.in_waiting
        raw = transport.read(n) if n else b""
        if raw:
            journal.bytes("rx", raw)
            values, errs = parser.feed(raw, clock()); statuses += values; errors += len(errs)
        else:
            sleep(0.01)
    boots = sorted({v.boot_id for v in statuses}); states = sorted({v.state for v in statuses})
    outputs_zero = all(v.actual_drive == 0 and v.actual_steer == 0 and v.requested_drive == 0 and v.requested_steer == 0 for v in statuses)
    rep = {"seconds": seconds, "rx_bytes": journal.rx_bytes, "tx_bytes": journal.tx_bytes, "type150_frames": len(statuses),
           "parse_errors": errors, "boot_ids": boots, "states": states, "all_outputs_and_requests_zero": outputs_zero if statuses else None,
           "adc_values": sorted({v.steering_adc for v in statuses})[:20], "ascii_marker": journal.ascii_marker,
           "verdict": ("type150_identified" if statuses and not journal.ascii_marker else
                       "ascii_firmware" if journal.ascii_marker else "unidentified"),
           "note": "receive-only; TX impossible (no Client); board flash identity not read; physical output not measured"}
    journal.event("identify", **rep)
    return rep


class StdinKeys:
    """Non-blocking single-key reader; cbreak only when stdin is a tty."""

    def __init__(self):
        self.fd = sys.stdin.fileno() if sys.stdin and sys.stdin.isatty() else None
        self.saved = None
        if self.fd is not None:
            import tty
            self.saved = termios.tcgetattr(self.fd); tty.setcbreak(self.fd)

    def poll(self):
        if self.fd is None: return []
        r, _, _ = select.select([self.fd], [], [], 0)
        return list(os.read(self.fd, 16).decode("ascii", "ignore").lower()) if r else []

    def restore(self):
        if self.fd is not None and self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)


def run_realtime(trial, keys, script=None, sleep_s=0.02, echo=True):
    """Real-clock loop: tick every <=20 ms, keys from stdin and/or a timed script ('0.5:p,1.5:l,6:q')."""
    scripted = []
    if script:
        for item in script.split(","):
            t, k = item.split(":"); scripted.append((float(t), k.strip().lower() or " "))
        scripted.sort()
    t0 = time.monotonic(); last_line = 0.0
    try:
        while True:
            now = time.monotonic()
            for k in keys.poll(): trial.key(k, now)
            while scripted and now - t0 >= scripted[0][0]:
                _, k = scripted.pop(0); trial.key(k, now)
            if not trial.tick(now): break
            if echo and now - last_line >= 0.1:
                last_line = now; print("\r" + trial.status_line(now)[:160].ljust(160), end="", flush=True)
            fds = [trial.transport.fd] if getattr(trial.transport, "fd", None) is not None else []
            if keys.fd is not None: fds.append(keys.fd)
            if fds: select.select(fds, [], [], sleep_s)
            else: time.sleep(sleep_s)
    except KeyboardInterrupt:
        now = time.monotonic(); trial.quit(now)
        deadline = now + trial.ack_wait_s + 0.2
        while time.monotonic() < deadline and trial.tick(time.monotonic()): time.sleep(sleep_s)
        if not trial.finished: trial.finish(EXIT_NO_ACK, "Ctrl-C; stop attempted, ack not observed", time.monotonic())
    finally:
        if echo: print()
    return trial.exit_code


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--preflight", action="store_true", help="device-free readiness check (default)")
    mode.add_argument("--model", action="store_true", help="board MODEL + synthetic calibration; terminal rehearsal only")
    mode.add_argument("--live", action="store_true", help="real type150 board; requires READY preflight")
    mode.add_argument("--identify", action="store_true", help="receive-only board identification for --seconds; TX impossible")
    ap.add_argument("--seconds", type=float, default=3.0, help="--identify listen time")
    ap.add_argument("--port", default=str(smc.MEGA_PATH), help="serial by-id path (live)")
    ap.add_argument("--calibration", default=str(DEFAULT_CAL))
    ap.add_argument("--angle-deg", type=float, default=5.0, help="l/r target magnitude, left front wheel degrees")
    ap.add_argument("--max-hold-s", type=float, default=3.0, help="finite hold per key press")
    ap.add_argument("--stall-window-s", type=float, default=LiveTrial.STALL_WINDOW_S,
                    help="cut the output when steering is active this long without ADC progress toward the target")
    ap.add_argument("--stall-min-progress-adc", type=int, default=LiveTrial.STALL_MIN_PROGRESS_ADC,
                    help="ADC progress toward the target that counts as moving")
    ap.add_argument("--identify-timeout-s", type=float, default=3.0)
    ap.add_argument("--output-dir", help="NEW folder (live: under data/, model: under logs/)")
    ap.add_argument("--out", help="preflight report json path")
    ap.add_argument("--script", help="timed keys for --model/--live, e.g. '0.5:p,1.5:l,6:q'")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args(argv)
    if not math.isfinite(a.stall_window_s) or a.stall_window_s <= 0: ap.error("--stall-window-s must be finite and > 0")
    if a.stall_min_progress_adc <= 0: ap.error("--stall-min-progress-adc must be > 0")
    for name in ("angle_deg", "max_hold_s", "identify_timeout_s"):
        v = getattr(a, name)
        if not math.isfinite(v) or (name != "angle_deg" and v <= 0): ap.error(f"--{name.replace('_', '-')} must be finite" + ("" if name == "angle_deg" else " and > 0"))
    if a.identify:
        if not a.output_dir: ap.error("--identify requires --output-dir (new folder under data/)")
        if not math.isfinite(a.seconds) or a.seconds <= 0: ap.error("--seconds must be finite and > 0")
        out = Path(a.output_dir)
        journal = TrialJournal(out, "live_vehicle", {"tool": "tools/live_steering_trial.py", "mode": "identify", "port": a.port,
                                                     "argv": list(sys.argv[1:] if argv is None else argv),
                                                     "os_serial_open_may_reset_board": True, "termios_restore_on_close": False,
                                                     "code_sha256": {p: sha256_file(ROOT / p) for p in CODE_FILES if (ROOT / p).is_file()}})
        try:
            transport = PosixSerialTransport(a.port, check_identity=True)
        except (OSError, RuntimeError) as exc:
            journal.close(status="error", error=str(exc)); print(f"cannot open port: {exc}", file=sys.stderr); return EXIT_CONN
        rep = None
        try:
            rep = identify(transport, journal, a.seconds)
        finally:
            transport.close()
            journal.close(status="completed" if rep else "error", **({k: rep[k] for k in ("verdict", "type150_frames", "boot_ids", "ascii_marker")} if rep else {}))
        print(json.dumps(rep, ensure_ascii=False, indent=1)); print(f"IDENTIFY: {rep['verdict']} (tx_bytes={rep['tx_bytes']}) -> {out}")
        return EXIT_OK if rep["verdict"] == "type150_identified" else EXIT_UNIDENTIFIED
    if a.model:
        cal_path, allow_syn = SYNTH_CAL, True
    else:
        cal_path, allow_syn = Path(a.calibration), False
    pre = preflight(cal_path, a.angle_deg, a.port if a.live else None, allow_synthetic=allow_syn)
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps(pre, ensure_ascii=False, indent=1) + "\n")
    if not (a.model or a.live):
        print(f"PREFLIGHT: {pre['status']} (exit {0 if pre['live_capable'] else EXIT_BLOCKED}; no device accessed)")
        for m in pre["missing"]: print("  - " + m)
        if pre["live_capable"]: print("  live command may be used ONLY after the board runs type150 and physical conditions are met")
        return EXIT_OK if pre["live_capable"] else EXIT_BLOCKED
    if not pre["live_capable"]:
        print(f"REFUSED before opening any port: preflight BLOCKED", file=sys.stderr)
        for m in pre["missing"]: print("  - " + m, file=sys.stderr)
        return EXIT_BLOCKED
    lib = h.load_library()
    angle_cdeg = pre["angle_cdeg"]
    stamp = datetime.now(KST).strftime("%Y%m%d_%H%M%S")
    if a.model:
        import steer_target_sim as sim
        out = Path(a.output_dir) if a.output_dir else ROOT / "logs" / "live_steering_model" / stamp
        journal = TrialJournal(out, "model_synthetic", build_metadata("model", cal_path, pre, angle_cdeg,
                               {"note": "board MODEL, synthetic calibration/plant; not vehicle data"}, argv,
                               stall={"window_s": a.stall_window_s, "min_progress_adc": a.stall_min_progress_adc}))
        transport = ModelTransport(lib, pre["validated"]["steer_sign_for_adc_increase"])
        trial = assemble(lib, cal_path, angle_cdeg, transport, journal, allow_synthetic=True,
                         max_hold_s=a.max_hold_s, identify_timeout_s=a.identify_timeout_s,
                         stall_window_s=a.stall_window_s, stall_min_progress_adc=a.stall_min_progress_adc)
        print(f"*** MODEL run (synthetic; no device) -> {out} *** keys: p prepare, l/r/0 target, s stop, q quit", flush=True)
    else:
        if not a.output_dir: ap.error("--live requires --output-dir (new folder under data/)")
        out = Path(a.output_dir)
        journal = TrialJournal(out, "live_vehicle", build_metadata("live", cal_path, pre, angle_cdeg,
                               {"port": a.port, "os_serial_open_may_reset_board": True,
                                "note": "real board bytes; physical wheel motion and stop are NOT verified by this file"}, argv,
                               stall={"window_s": a.stall_window_s, "min_progress_adc": a.stall_min_progress_adc}))
        try:
            transport = PosixSerialTransport(a.port, check_identity=True)
        except (OSError, RuntimeError) as exc:
            journal.close(status="error", error=str(exc)); print(f"cannot open port: {exc}", file=sys.stderr); return EXIT_CONN
        trial = assemble(lib, cal_path, angle_cdeg, transport, journal, allow_synthetic=False,
                         max_hold_s=a.max_hold_s, identify_timeout_s=a.identify_timeout_s,
                         stall_window_s=a.stall_window_s, stall_min_progress_adc=a.stall_min_progress_adc)
        print(f"LIVE steering-only trial -> {out} (drive PWM 0; nothing sent until type150 identified)", flush=True)
        print("keys: p prepare, l/r target +/-angle, 0 table-straight, s stop, q quit (Ctrl-C = quit)", flush=True)
    keys = StdinKeys()
    try:
        code = run_realtime(trial, keys, a.script, echo=not a.quiet)
    finally:
        keys.restore()
        try:
            transport.close()
        finally:
            journal.close(status="completed" if trial.exit_code == EXIT_OK else "ended",
                          exit_code=trial.exit_code, exit_reason=trial.exit_reason,
                          stop_attempts=trial.stop_attempts, stop_ack_observed=trial.stop_ack_observed,
                          final_stop_attempt=trial.current_stop, stop_history=trial.stop_history,
                          last_stop_reason=trial.last_stop_reason, stall_blocked=trial.stall_blocked,
                          adapter_first_fault_since_prepare=trial.adapter.first_fault,
                          identified=trial.adapter.identified)
    print(f"exit {code}: {trial.exit_reason} | tx_frames={journal.tx_frames} stop_ack={trial.stop_ack_observed} -> {out}")
    return code


class ModelTransport:
    """steer_target_sim MockTransport + BoardModel driven by real time (rehearsal / logged model runs).
    Board millis are deliberately offset from the host clock."""

    def __init__(self, lib, sign, boot_id=71, adc0=600):
        import steer_target_sim as sim
        self.mock = sim.MockTransport(); self.board = sim.BoardModel(lib, boot_id, adc0, sign)
        self.t0 = time.monotonic(); self.last = self.t0; self.next_status = 0.0; self.fd = None; self.closed = False

    def _board_ms(self): return (int((time.monotonic() - self.t0) * 1000) + 987654) & 0xFFFFFFFF

    def _pump(self):
        now = time.monotonic()
        for raw in self.mock.pending_tx: self.board.feed(raw, self._board_ms())
        self.mock.pending_tx.clear()
        self.board.plant(now - self.last); self.last = now
        self.board.tick(self._board_ms())
        if now >= self.next_status:
            f = self.board.status(self._board_ms())
            if f: self.mock.rx += f
            self.next_status = now + 0.05

    @property
    def in_waiting(self): self._pump(); return self.mock.in_waiting

    @property
    def out_waiting(self): return 0

    def read(self, n): return self.mock.read(n)

    def write(self, raw): return self.mock.write(raw)

    def close(self):
        if not self.closed: self.closed = True; self.board.destroy(); self.mock.close()


if __name__ == "__main__":
    sys.exit(main())
