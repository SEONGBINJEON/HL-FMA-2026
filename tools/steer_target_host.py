#!/usr/bin/env python3
"""C2: host-side adapter that turns a target steering angle into type150 manual
commands, reusing the C1.1 C++ core through ctypes (firmware/steer_target/host).

Device-free. Inputs: decoded type150 statuses (joystick_protocol.Parser), local
monotonic time, target commands (sequence, angle cdeg left +, host time).
Output per board request: a step_manual_protocol.Command (drive always 0,
encoded with drive PWM 0) or None (nothing valid to answer).

Preparation is explicit: request_prepare() -> board neutral confirmed -> ARM
confirmed -> ready (all handled by the inherited manual Session handshake).
Reversal needs the board to have ACCEPTED a neutral we actually sent: a later
status of the same boot with request_id beyond the one we answered, ARMED,
requested/actual steer 0 and reason READY/NEUTRAL, plus reversal_neutral_ms.
Faults (stop, protocol error, board restart, stale) lock the adapter; only a
new request_prepare() AND a new target sequence resume. The C1.1 core's own
latch rules stay in force underneath.
"""
import ctypes, json, math, sys, time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import List, Optional

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from step_manual_protocol import Command, DISARM, ARM, CONTROL, DISARMED, ARMED, STEP_UNUSED_FLAGS  # noqa: E402
from step_manual_control import Session  # noqa: E402
import steering_calibration as sc  # noqa: E402

LIB_PATH = ROOT / "firmware/steer_target/host/build/libsteer_host.so"
BUILD_CMD = "bash firmware/steer_target/host/build.sh"
REASON_NAMES = ["NOT_READY", "NO_CALIBRATION", "NO_TARGET", "STOPPED", "COMMAND_STALE", "SENSOR_STALE",
                "SENSOR_INVALID", "TARGET_REJECTED", "AT_TARGET", "REVERSAL_NEUTRAL", "AT_LIMIT", "MOVING"]
BOARD_REASONS = {0: "BOOT", 1: "STOP", 2: "TIMEOUT", 3: "BAD_FRAME", 4: "STALE", 5: "INTERLOCK", 6: "ARM_REJECTED",
                 7: "REVERSAL", 8: "STEER_LIMIT", 9: "READY", 10: "ACTIVE", 11: "NEUTRAL"}
POLICY = {"LIMIT_REJECT": 1, "LIMIT_CLAMP": 2}


class StcConfig(ctypes.Structure):
    _fields_ = [("calibrated", ctypes.c_uint8), ("point_count", ctypes.c_uint8),
                ("angle_cdeg", ctypes.c_int16 * 8), ("adc", ctypes.c_uint16 * 8),
                ("usable_adc_min", ctypes.c_uint16), ("usable_adc_max", ctypes.c_uint16),
                ("steer_sign_for_adc_increase", ctypes.c_int8),
                ("stop_band_adc", ctypes.c_uint16), ("restart_band_adc", ctypes.c_uint16), ("sensor_margin_adc", ctypes.c_uint16),
                ("command_stale_ms", ctypes.c_uint32), ("sensor_stale_ms", ctypes.c_uint32), ("reversal_neutral_ms", ctypes.c_uint32),
                ("limit_policy", ctypes.c_uint8)]


class StcOutput(ctypes.Structure):
    _fields_ = [("steer", ctypes.c_int8), ("reason", ctypes.c_uint8), ("fault_latched", ctypes.c_uint8),
                ("target_clamped", ctypes.c_uint8), ("target_adc", ctypes.c_uint16)]


def load_library(path: Path = LIB_PATH):
    if not path.is_file():
        raise FileNotFoundError(f"{path} missing; build with: {BUILD_CMD}")
    lib = ctypes.CDLL(str(path))
    P = ctypes.c_void_p
    lib.stc_config_valid.argtypes = [ctypes.POINTER(StcConfig)]; lib.stc_config_valid.restype = ctypes.c_int
    lib.stc_angle_to_adc.argtypes = [ctypes.POINTER(StcConfig), ctypes.c_int16, ctypes.POINTER(ctypes.c_uint16)]; lib.stc_angle_to_adc.restype = ctypes.c_int
    lib.stc_adc_to_angle.argtypes = [ctypes.POINTER(StcConfig), ctypes.c_uint16, ctypes.POINTER(ctypes.c_int16)]; lib.stc_adc_to_angle.restype = ctypes.c_int
    lib.stc_create.argtypes = [ctypes.POINTER(StcConfig)]; lib.stc_create.restype = P
    lib.stc_destroy.argtypes = [P]
    lib.stc_set_ready.argtypes = [P, ctypes.c_int, ctypes.c_uint32]
    lib.stc_stop.argtypes = [P, ctypes.c_uint32]
    lib.stc_set_target.argtypes = [P, ctypes.c_uint32, ctypes.c_int16, ctypes.c_uint32, ctypes.c_uint32]; lib.stc_set_target.restype = ctypes.c_int
    lib.stc_set_measurement.argtypes = [P, ctypes.c_uint16, ctypes.c_uint32]
    lib.stc_update.argtypes = [P, ctypes.c_uint32, ctypes.POINTER(StcOutput)]
    lib.stc_last_sequence.argtypes = [P]; lib.stc_last_sequence.restype = ctypes.c_uint32
    lib.brd_create.argtypes = [ctypes.c_uint32]; lib.brd_create.restype = P
    lib.brd_destroy.argtypes = [P]
    lib.brd_set_inputs.argtypes = [P, ctypes.c_uint16, ctypes.c_uint32]
    lib.brd_tick.argtypes = [P, ctypes.c_uint32]
    lib.brd_feed.argtypes = [P, ctypes.c_uint8, ctypes.c_uint32]
    lib.brd_status_due.argtypes = [P, ctypes.c_uint32]; lib.brd_status_due.restype = ctypes.c_int
    lib.brd_make_status.argtypes = [P, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_int32]; lib.brd_make_status.restype = ctypes.c_int
    lib.brd_snapshot.argtypes = [P, ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_int8), ctypes.POINTER(ctypes.c_uint8),
                                 ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8)]
    return lib


def config_from_validated(cfg: dict) -> StcConfig:
    """Build the C struct from tools.steering_calibration.validate()[0]. Never from a raw record."""
    c = StcConfig()
    c.calibrated = 1
    pts = cfg["points"]; c.point_count = len(pts)
    for i, (a, v) in enumerate(pts):
        c.angle_cdeg[i] = a; c.adc[i] = v
    c.usable_adc_min = cfg["usable_adc_min"]; c.usable_adc_max = cfg["usable_adc_max"]
    c.steer_sign_for_adc_increase = cfg["steer_sign_for_adc_increase"]
    for k in ("stop_band_adc", "restart_band_adc", "sensor_margin_adc", "command_stale_ms", "sensor_stale_ms", "reversal_neutral_ms"):
        setattr(c, k, cfg[k])
    c.limit_policy = POLICY[cfg["limit_policy"]]
    return c


def load_calibration(path: Path, allow_synthetic: bool = False):
    """(StcConfig or None, reasons, synthetic flag). Operational null record => (None, reasons)."""
    cfg, reasons = sc.validate(sc.load(path), allow_synthetic=allow_synthetic)
    if cfg is None:
        return None, reasons, False
    return config_from_validated(cfg), [], bool(cfg.get("synthetic"))


INT16_MIN, INT16_MAX, UINT32_MAX = -32768, 32767, 0xFFFFFFFF
LATCHING_REASONS = {"COMMAND_STALE", "SENSOR_STALE", "SENSOR_INVALID", "TARGET_REJECTED", "STOPPED"}


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def check_target_input(sequence, angle_cdeg, host_time_ms, now_s) -> str:
    """Empty string when the target may be handed to the C ABI; otherwise the reason.
    Values are checked BEFORE ctypes conversion so nothing wraps silently. (C2.1)"""
    if not _is_int(sequence) or not (1 <= sequence <= UINT32_MAX):
        return "sequence must be int in 1..2^32-1 (no wrap)"
    if not _is_int(angle_cdeg) or not (INT16_MIN <= angle_cdeg <= INT16_MAX):
        return "angle_cdeg must be int16 (no wrap)"
    if not _is_int(host_time_ms) or not (0 <= host_time_ms <= UINT32_MAX):
        return "host_time_ms must be uint32"
    if isinstance(now_s, bool) or not isinstance(now_s, (int, float)) or not math.isfinite(now_s) or now_s < 0:
        return "now must be a finite non-negative monotonic time"
    return ""


class TargetCore:
    """Thin owner of one C++ hl_steer_target::Core instance."""

    def __init__(self, lib, config: Optional[StcConfig]):
        self.lib = lib
        self.config = config if config is not None else StcConfig()  # default => NO_CALIBRATION
        self.valid = bool(lib.stc_config_valid(ctypes.byref(self.config)))
        self.handle = lib.stc_create(ctypes.byref(self.config))

    def __del__(self):
        try:
            if getattr(self, "handle", None): self.lib.stc_destroy(self.handle)
        except Exception:
            pass

    def set_ready(self, ready: bool, now_ms: int): self.lib.stc_set_ready(self.handle, int(ready), now_ms & 0xFFFFFFFF)
    def stop(self, now_ms: int): self.lib.stc_stop(self.handle, now_ms & 0xFFFFFFFF)
    def set_target(self, seq: int, angle_cdeg: int, host_time_ms: int, now_ms: int) -> bool:
        """External values are range-checked; only the LOCAL millis counter is allowed to wrap."""
        bad = check_target_input(seq, angle_cdeg, host_time_ms, 0.0)
        if bad:
            raise ValueError(bad)
        return bool(self.lib.stc_set_target(self.handle, seq, angle_cdeg, host_time_ms, now_ms & 0xFFFFFFFF))
    def last_sequence(self) -> int: return int(self.lib.stc_last_sequence(self.handle))
    def set_measurement(self, adc: int, now_ms: int):
        if not _is_int(adc) or not (0 <= adc <= 1023):
            raise ValueError("adc must be int 0..1023")
        self.lib.stc_set_measurement(self.handle, adc, now_ms & 0xFFFFFFFF)
    def update(self, now_ms: int) -> StcOutput:
        out = StcOutput(); self.lib.stc_update(self.handle, now_ms & 0xFFFFFFFF, ctypes.byref(out)); return out


@dataclass
class Decision:
    now_s: float
    phase: str
    board: Optional[dict]
    core_reason: str
    core_steer: int
    target_adc: int
    sent: Optional[dict]
    note: str = ""


class HostSteerAdapter(Session):
    """Manual-protocol Session whose steering decision comes from the C1.1 core.

    No keys are ever pressed; drive is always 0. Status ordering, board
    restart, request regression, duplicate suppression, UI/RX stall and the
    neutral->ARM handshake are the inherited Session behaviour for type150.
    """

    def __init__(self, core: TargetCore, clock_ms=None):
        super().__init__()
        self.core = core
        self.focused = True
        self.message = "목표 조향 어댑터 · 준비 전"
        self._clock_ms = clock_ms or (lambda now_s: int(now_s * 1000) & 0xFFFFFFFF)
        self.locked_by_fault = False       # requires request_prepare() again
        self.have_target_since_prepare = False
        self.last_sent_sign = 0
        self.neutral_pending_req: Optional[tuple] = None  # (boot, request_id) we answered with steer 0 while ARMED
        self.neutral_accepted_at_ms: Optional[int] = None
        self.last_now_ms = 0
        self.first_fault: Optional[str] = None   # first fault since the last successful prepare
        self.trace: List[Decision] = []

    # ---- explicit operator/planner inputs
    def request_prepare(self, now: float) -> bool:
        if not self.core.valid:
            # No validated calibration: never start the neutral->ARM handshake, never show "ready".
            self.phase = "locked"; self.message = "보정 없음 · 준비 거절 (ARM 미송신)"
            return False
        ok = self.request_arm(now)
        if ok:
            self.locked_by_fault = False
            self.have_target_since_prepare = False
            self.first_fault = None
            self.last_sent_sign = 0; self.neutral_pending_req = None; self.neutral_accepted_at_ms = None
        return ok

    def set_target(self, sequence: int, angle_cdeg: int, host_time_ms: int, now: float) -> bool:
        """Accepted only when prepared (ready/active) and not fault-locked; same sequence never renews."""
        if self.locked_by_fault or self.phase not in ("ready", "active"):
            return False
        bad = check_target_input(sequence, angle_cdeg, host_time_ms, now)
        if bad:
            # Policy: a malformed target is a planner fault. Ongoing motion must not continue on the
            # previous target, so lock now; the next board request is answered with DISARM.
            self._fault(f"invalid target input: {bad}")
            return False
        duplicate = sequence <= self.core.last_sequence()
        now_ms = self._clock_ms(now)
        self.core.set_ready(True, now_ms)   # adapter is the source of truth for readiness; a refusal below is then a rejection
        accepted = self.core.set_target(sequence, angle_cdeg, host_time_ms, now_ms)
        if accepted:
            self.have_target_since_prepare = True
        elif not duplicate:
            # A new sequence the core refused = TARGET_REJECTED (outside table/usable range under REJECT).
            # The core has latched; propagate so nothing resumes on a later sequence alone.
            self._fault("target rejected by core (outside table/usable range): lock")
        return accepted

    def _fault(self, reason: str):
        if self.first_fault is None:
            self.first_fault = reason
        self._fault_traced = False
        self.stop(reason)

    def stop(self, message="정지 · 다시 조작 준비 필요"):
        super().stop(message)
        if getattr(self, "first_fault", None) is None and getattr(self, "core", None) is not None:
            self.first_fault = message      # also covers Session-detected faults (ordering, restart, stale RX)
            self._fault_traced = False
        self.locked_by_fault = True
        self.have_target_since_prepare = False
        self.last_sent_sign = 0; self.neutral_pending_req = None; self.neutral_accepted_at_ms = None
        if hasattr(self, "core"):
            self.core.stop(self.last_now_ms)

    # ---- per RX batch
    def step(self, now, statuses=(), error=None):
        now_ms = self._clock_ms(now); self.last_now_ms = now_ms
        prev_reply = self.last_reply
        cmd = super().step(now, statuses, error)     # handshake, ordering, faults, DISARM on fault
        latest = self.status
        board = asdict(latest) if latest is not None else None
        prepared = self.phase in ("ready", "active") and not self.locked_by_fault
        self.core.set_ready(prepared, now_ms)
        core_reason, core_steer, target_adc, note = "NOT_READY", 0, 0, ""
        if prepared and latest is not None and cmd is not None and cmd.action == CONTROL:
            # Neutral acceptance: a newer status of the same boot after a neutral we sent.
            # Steer-axis neutral proof = requested and actual steer both 0 on that newer ARMED status.
            # Reason is READY(9)/NEUTRAL(11) when no axis is active, and ACTIVE(10) when the drive axis is
            # active while steer is already 0. The firmware resets steer_sign_seen on any accepted steer=0
            # command regardless of drive, so ACTIVE with steer 0 is the same proof. 2026-09-19 field run:
            # with drive on, the wait never ended and the stall guard stopped a correct reversal.
            if self.neutral_pending_req is not None:
                b, r = self.neutral_pending_req
                if latest.boot_id == b and latest.request_id > r and latest.state == ARMED and \
                        latest.requested_steer == 0 and latest.actual_steer == 0 and latest.reason in (9, 10, 11):
                    if self.neutral_accepted_at_ms is None:
                        self.neutral_accepted_at_ms = now_ms
                    self.neutral_pending_req = None
                elif latest.boot_id != b:
                    self.neutral_pending_req = None; self.neutral_accepted_at_ms = None
            self.core.set_measurement(latest.steering_adc, now_ms)
            out = self.core.update(now_ms)
            core_reason, core_steer, target_adc = REASON_NAMES[out.reason], int(out.steer), int(out.target_adc)
            if (out.fault_latched or core_reason in LATCHING_REASONS) and self.have_target_since_prepare:
                # C2.1: a latched core fault that interrupts an accepted target is an adapter fault. Drop
                # target and preparation, answer DISARM, require request_prepare() + a new sequence.
                # Right after (re)prepare the core may still hold its previous latch (NOT_READY/STOPPED)
                # until the first new sequence arrives: that is the core's documented resume rule, it
                # outputs 0 and is not a fault. AT_TARGET/REVERSAL_NEUTRAL/NO_TARGET are normal.
                self._fault(f"core fault {core_reason}: lock, DISARM")
                cmd = Command(cmd.boot_id, cmd.request_id)
                self.trace.append(Decision(now, self.phase, board, core_reason, core_steer, target_adc,
                                           {"action": DISARM, "drive": 0, "steer": 0, "deadman": 0,
                                            "boot_id": cmd.boot_id, "request_id": cmd.request_id},
                                           f"first fault: {self.first_fault}"))
                self._fault_traced = True
                return cmd
            steer = core_steer
            if steer != 0 and self.last_sent_sign != 0 and steer == -self.last_sent_sign:
                gate_ok = (self.neutral_accepted_at_ms is not None and
                           (now_ms - self.neutral_accepted_at_ms) & 0xFFFFFFFF >= self.core.config.reversal_neutral_ms)
                if not gate_ok:
                    steer = 0; note = "reversal held: board neutral not yet confirmed"
                else:
                    note = f"reversal allowed: board neutral confirmed at {self.neutral_accepted_at_ms} ms"
            deadman = 1 if steer else 0
            cmd = Command(cmd.boot_id, cmd.request_id, CONTROL, 0, steer, deadman)
            if steer == 0:
                if self.neutral_accepted_at_ms is None and self.neutral_pending_req is None:
                    self.neutral_pending_req = (latest.boot_id, latest.request_id)
            else:
                self.last_sent_sign = steer; self.neutral_pending_req = None; self.neutral_accepted_at_ms = None
            if steer != 0 and self.last_sent_sign == steer and self.neutral_accepted_at_ms is not None:
                self.neutral_accepted_at_ms = None
            self.phase = "active" if steer else "ready"
            self.message = f"목표각 제어 {core_reason} · 출력 {steer:+d}" if steer else f"목표각 제어 {core_reason} · 출력 0"
        elif cmd is not None and cmd.action != DISARM and not prepared:
            pass  # handshake commands (neutral CONTROL / ARM) from the inherited Session
        sent = None
        if cmd is not None and (self.last_reply != prev_reply or cmd.action == DISARM):
            sent = {"action": cmd.action, "drive": cmd.drive, "steer": cmd.steer, "deadman": cmd.deadman,
                    "boot_id": cmd.boot_id, "request_id": cmd.request_id}
        if cmd is not None:
            if self.locked_by_fault and self.first_fault and not getattr(self, "_fault_traced", True) and not note:
                note = f"first fault: {self.first_fault}"; self._fault_traced = True
            self.trace.append(Decision(now, self.phase, board, core_reason, core_steer, target_adc, sent, note))
        return cmd


def encode_zero_pwm(command):
    """Encoder for Client: drive PWM is always 0 in C2."""
    from joystick_protocol import encode_command
    return encode_command(command, 0)
