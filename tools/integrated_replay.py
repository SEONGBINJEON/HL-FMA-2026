#!/usr/bin/env python3
"""C5: command-flow replay of predetermined GNSS antenna poses through the real host chain.

  input record -> C4 pose_reference.rear_axle_pose -> C3 PathFollower (Q1.1 RunState)
    -> steering_angle_contract.convert_message (SYNTHETIC ideal Ackermann, bicycle -> left front wheel)
    -> FollowerC2Bridge -> HostSteerAdapter -> actual C2 host C++ core -> MockTransport -> BoardModel

This is NOT closed-loop driving: positions are read from the file and never react to steering output.
Drive/PWM are always 0. BoardModel.plant is steering-feedback only (no vehicle plant). No device, no
network, no config change. "ok" means the expected replay invariants held, never a course pass.

Preparation boundary: request_prepare is issued once as a synthetic operator action before the first
sample. After any hold/lock the remaining samples are IGNORED (counted); a reprepare record event is
NOT supported. Restarting a replay is the only explicit re-preparation boundary.

Terminal causes and exit semantics (cmd_replay):
  path_end   normal: the follower's explicit end criterion was met by the INPUT positions (reached_by_input).
             The bridge then locks the adapter on purpose; later records are counted as ignored_after_stop
             and are NOT re-interpreted as faults. Board ticks and the EOF drain still run.
  eof        normal: input ended before any stop/hold (EOF is not a path end; reached_by_input False).
  hold_lock / adapter_fault / limit_stop   abnormal: exit 4 even when every replay invariant held (ok True).
  Invariant failure (ok False) is exit 1. Input/argument rejection exit 2, engine configuration exit 3.
  cmd_demo is different: it checks that each EXPECTED failure was blocked exactly as expected, so its
  aggregate exit 0 does not mean the individual replays terminated normally.

Stop-response evidence: the board MODEL's actual steer is sampled every host tick. After a hold/fault
(or a genuine adapter fault detected in a host tick, e.g. an input gap) the first zero-output time and
any later non-zero output are recorded (stop_response). The 50 ms window is a check on the synthetic
model chain only and says nothing about real vehicle stopping performance.

Input schema (schema_version 1, strict; unknown keys refused):
  input_kind 'synthetic'|'recorded', source (label), synthetic_control true, notes (optional),
  route [[x,y],...] map metres (>=2), route_source (label),
  pose {frame_id, frame_alignment_verified, reference_point, heading_convention, map_yaw_from_enu_rad,
        offset_x_m, offset_y_m, max_age_s, planar_assumption_verified, source},
  controller {pure_pursuit: 'path_steering.SYNTHETIC_CONFIG', selection: 'qualifier_route.SYNTHETIC_PARAMS', source},
  angle_contract {mode 'synthetic_ideal_ackermann'|'unconfigured', source_kind 'bicycle_equivalent' (explicit),
        target_kind 'left_front_wheel' (explicit), wheelbase_m, track_m, source},
  columns == COLUMNS, records: list of rows [t_s, x_m, y_m, heading, position_age_s, heading_age_s,
        position_valid, heading_valid]; t strictly increasing, EVERY absolute t within 1.0..120.0 s
        (not just the span), <= 5000 rows.
Frame/reference/verification values are passed to C4 unchanged; C4 refuses what it does not accept.
"""
import argparse, copy, hashlib, json, math, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import qualifier_route as qr  # noqa: E402
import path_steering as ps  # noqa: E402
import path_steering_bridge as pb  # noqa: E402
import pose_reference as pr  # noqa: E402
import steering_angle_contract as sac  # noqa: E402
import steering_calibration as sc  # noqa: E402
import steer_target_host as h  # noqa: E402
import steer_target_sim as sim  # noqa: E402
import step_manual_control as smc  # noqa: E402
from joystick_protocol import Parser  # noqa: E402
from step_manual_control import Client  # noqa: E402

CALIB = ROOT / "tests/fixtures/steering_calibration_synthetic.json"
COLUMNS = ["t_s", "x_m", "y_m", "heading", "position_age_s", "heading_age_s", "position_valid", "heading_valid"]
TOP_KEYS = {"schema_version", "input_kind", "source", "synthetic_control", "notes", "route", "route_source", "pose",
            "controller", "angle_contract", "columns", "records"}
POSE_LABELS = ("frame_id", "reference_point", "heading_convention", "source")
POSE_BOOLS = ("frame_alignment_verified", "planar_assumption_verified")
POSE_NUMS = ("map_yaw_from_enu_rad", "offset_x_m", "offset_y_m", "max_age_s")
CONTROLLER = {"pure_pursuit": "path_steering.SYNTHETIC_CONFIG", "selection": "qualifier_route.SYNTHETIC_PARAMS"}
MIN_T, MAX_T, MAX_RECORDS, DT_MS, DRAIN_S = 1.0, 120.0, 5000, 5, 0.5
STOP_WINDOW_MS = 50   # synthetic model chain only (host tick 5 ms); not a vehicle performance figure
CONTRACT_KEYS = {"mode", "source_kind", "target_kind", "wheelbase_m", "track_m", "source"}
EXIT_OK, EXIT_INVARIANT_FAILED, EXIT_INPUT, EXIT_CONFIG, EXIT_ABNORMAL_TERMINAL = 0, 1, 2, 3, 4
NORMAL_TERMINALS = ("path_end", "eof")
SCENARIOS = ("nominal", "bad_frame", "stale_heading", "input_gap", "unconfigured_contract", "eof_midroute")
RELATED_SOURCES = ["tools/integrated_replay.py", "tools/steering_angle_contract.py", "tools/pose_reference.py",
                   "tools/path_steering.py", "tools/path_steering_bridge.py", "tools/steer_target_host.py",
                   "tests/fixtures/steering_calibration_synthetic.json", "tests/fixtures/c5_replay_synthetic.json",
                   "tools/qualifier_route.py", "tools/steering_calibration.py", "tools/steer_target_sim.py",
                   "tools/step_manual_control.py", "tools/step_manual_protocol.py", "tools/joystick_protocol.py",
                   "firmware/steer_target/host/build/libsteer_host.so"]


class ReplayConfigError(RuntimeError):
    """Engine could not be brought to readiness (not an input data error)."""


# ------------------------------------------------------------------ input validation
def _num(name, v):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"{name}: finite int/float required (bool/str refused)")
    try:
        value = float(v)
    except (OverflowError, ValueError):
        raise ValueError(f"{name}: cannot be represented as a finite float") from None
    if not math.isfinite(value):
        raise ValueError(f"{name}: finite number required")
    return value


def _bool(name, v):
    if not isinstance(v, bool):
        raise ValueError(f"{name}: real bool required")
    return v


def _label(name, v):
    if not isinstance(v, str) or not v.strip():
        raise ValueError(f"{name}: non-empty string label required")
    return v


def validate_input(d):
    if not isinstance(d, dict):
        raise ValueError("top level must be an object")
    extra = set(d) - TOP_KEYS
    if extra:
        raise ValueError(f"unknown top-level keys {sorted(extra)}")
    for k in TOP_KEYS - {"notes"}:
        if k not in d:
            raise ValueError(f"missing key {k!r}")
    if isinstance(d["schema_version"], bool) or d["schema_version"] != 1:
        raise ValueError("schema_version must be 1")
    if d["input_kind"] not in ("synthetic", "recorded"):
        raise ValueError("input_kind must be 'synthetic' or 'recorded'")
    _label("source", d["source"]); _label("route_source", d["route_source"])
    if "notes" in d: _label("notes", d["notes"])
    if _bool("synthetic_control", d["synthetic_control"]) is not True:
        raise ValueError("synthetic_control must be true (controller/angle contract are synthetic only)")
    route = d["route"]
    if not isinstance(route, list) or len(route) < 2:
        raise ValueError("route must be a list of >= 2 [x, y] points")
    for i, p in enumerate(route):
        if not isinstance(p, list) or len(p) != 2:
            raise ValueError(f"route[{i}] must be [x, y]")
        _num(f"route[{i}].x", p[0]); _num(f"route[{i}].y", p[1])
    pose = d["pose"]
    if not isinstance(pose, dict) or set(pose) != set(POSE_LABELS + POSE_BOOLS + POSE_NUMS):
        raise ValueError(f"pose header must have exactly {sorted(POSE_LABELS + POSE_BOOLS + POSE_NUMS)}")
    for k in POSE_LABELS: _label(f"pose.{k}", pose[k])
    for k in POSE_BOOLS: _bool(f"pose.{k}", pose[k])
    for k in POSE_NUMS: _num(f"pose.{k}", pose[k])
    ctl = d["controller"]
    if not isinstance(ctl, dict) or set(ctl) != {"pure_pursuit", "selection", "source"}:
        raise ValueError("controller header must have pure_pursuit, selection, source")
    for k, v in CONTROLLER.items():
        if ctl[k] != v:
            raise ValueError(f"controller.{k} must be {v!r} (only synthetic parameters are available)")
    _label("controller.source", ctl["source"])
    ac = d["angle_contract"]
    if not isinstance(ac, dict) or set(ac) != CONTRACT_KEYS:
        raise ValueError(f"angle_contract header must have exactly {sorted(CONTRACT_KEYS)}")
    if ac["mode"] not in (sac.SYNTHETIC_MODE, sac.UNCONFIGURED_MODE):
        raise ValueError(f"angle_contract.mode must be {sac.SYNTHETIC_MODE!r} or {sac.UNCONFIGURED_MODE!r}")
    if ac["source_kind"] != sac.SOURCE_KIND or ac["target_kind"] != sac.TARGET_KIND:
        raise ValueError(f"angle_contract must state source_kind {sac.SOURCE_KIND!r} and target_kind {sac.TARGET_KIND!r} "
                         "explicitly (the only conversion this tool implements)")
    _label("angle_contract.source", ac["source"])
    for k in ("wheelbase_m", "track_m"):
        if ac[k] is not None: _num(f"angle_contract.{k}", ac[k])
    if d["columns"] != COLUMNS:
        raise ValueError(f"columns must be exactly {COLUMNS}")
    recs = d["records"]
    if not isinstance(recs, list) or not 1 <= len(recs) <= MAX_RECORDS:
        raise ValueError(f"records must hold 1..{MAX_RECORDS} rows")
    prev = None
    for i, r in enumerate(recs):
        if not isinstance(r, list) or len(r) != len(COLUMNS):
            raise ValueError(f"records[{i}] must have {len(COLUMNS)} columns")
        t = _num(f"records[{i}].t_s", r[0])
        for j in range(1, 6): _num(f"records[{i}].{COLUMNS[j]}", r[j])
        _bool(f"records[{i}].position_valid", r[6]); _bool(f"records[{i}].heading_valid", r[7])
        if not (MIN_T <= t <= MAX_T):   # absolute bound: a span-only check would let the pump run for ages
            raise ValueError(f"records[{i}].t_s {t} outside the absolute replay window {MIN_T}..{MAX_T} s")
        if prev is not None and t <= prev:
            raise ValueError(f"records[{i}].t_s {t} not strictly increasing (reversal/duplicate)")
        prev = t
    return d


def load_input(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f, parse_constant=lambda c: (_ for _ in ()).throw(ValueError(f"non-finite literal {c}")))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read JSON input: {exc}") from None
    return validate_input(d)


# ------------------------------------------------------------------ engine helpers
def _route(points):
    pts = []
    for i, (x, y) in enumerate(points):
        nx, ny = points[min(i + 1, len(points) - 1)]; px, py = points[max(i - 1, 0)]
        pts.append(qr.Point(i, float(x), float(y), math.atan2(ny - py, nx - px), "replay_input"))
    return qr.ModeRoute("replay_input", [qr.Segment("replay_input", 0, len(pts) - 1, "replay input")], pts)


def _angle_limits():
    """Left-wheel cdeg interval from the SYNTHETIC calibration's usable ADC range (asymmetric, table-derived)."""
    cfg, reasons = sc.validate(sc.load(CALIB), allow_synthetic=True)
    if cfg is None:
        raise ReplayConfigError(f"calibration fixture invalid: {reasons}")
    a_lo = sc.adc_to_angle(cfg["points"], cfg["usable_adc_min"]); a_hi = sc.adc_to_angle(cfg["points"], cfg["usable_adc_max"])
    lo, hi = sorted((a_lo, a_hi))
    return int(math.ceil(lo)), int(math.floor(hi))


def _contract(ac):
    """Kinds are taken from the input as written (validate_input already required the correct labels)."""
    return sac.AngleContract(ac["mode"], ac["source_kind"], ac["target_kind"], ac["wheelbase_m"], ac["track_m"])


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run_replay(data, input_path=None):
    validate_input(data)
    lib = h.load_library()
    cfg_c2, reasons, synth = h.load_calibration(CALIB, allow_synthetic=True)
    if cfg_c2 is None or not synth:
        raise ReplayConfigError(f"synthetic calibration fixture unusable: {reasons}")
    fake = sim.FakeTime(); real_time = smc.time; smc.time = fake
    try:
        return _run(data, lib, cfg_c2, fake, input_path)
    finally:
        smc.time = real_time


def _run(data, lib, cfg_c2, fake, input_path):
    clock_ms = lambda s: int(round(s * 1000)) & 0xFFFFFFFF  # noqa: E731
    route = _route(data["route"]); state = qr.RunState(qr.SYNTHETIC_PARAMS)
    follower = ps.PathFollower(state, ps.SYNTHETIC_CONFIG, clock_ms)
    contract = _contract(data["angle_contract"]); min_cdeg, max_cdeg = _angle_limits()
    core = h.TargetCore(lib, cfg_c2); adapter = h.HostSteerAdapter(core)
    transport = sim.MockTransport(); client = Client(transport, adapter, None, encoder=h.encode_zero_pwm); client.parser = Parser()
    bridge = pb.FollowerC2Bridge(adapter, client)
    board = sim.BoardModel(lib, boot_id=71, adc0=600, sign_for_adc_increase=cfg_c2.steer_sign_for_adc_increase, rate_counts_per_s=120.0)
    st = {"host_ms": 0, "max_abs_steer": 0, "steer_nonzero": False, "last_nonzero_s": None,
          "first_zero_after_hold_s": None, "nonzero_after_zero": False}
    events = []; pose_hdr = data["pose"]
    S = {"ready": False, "started": False, "locked": False, "hold_cause": None, "hold_at": None, "stop": None, "path_end": None,
         "ignored_locked": 0, "ignored_after_stop": 0, "processed": 0, "emitted": 0, "accepted": 0, "refused": 0,
         "conversion_holds": 0, "equiv": [], "left": [], "adapter_fault": None, "fault_tick_s": None,
         "intentional_lock_label": None, "eof_at": None}

    def ev(event, t, **kw):   # 'event' is the event name; a message's own kind travels as msg_kind
        e = {"t_s": round(t, 3), "event": event}; e.update(kw); events.append(e); return e

    def hold_time():
        if S["hold_at"] is not None: return S["hold_at"]
        if S["stop"]: return S["stop"]["t_s"]
        return S["eof_at"]

    def pump_to(t_s):
        target_ms = int(round(t_s * 1000))
        while st["host_ms"] < target_ms:
            now = st["host_ms"] / 1000; fake.now = now; board_ms = (st["host_ms"] + 99999) & 0xFFFFFFFF
            board.tick(board_ms); frame = board.status(board_ms)
            if frame: transport.rx += frame
            client.tick(now)
            # A genuine adapter fault surfaces inside a host tick (e.g. COMMAND_STALE during an input gap). The
            # intentional bridge locks (hold/stop/path_end/EOF) happen outside pump_to and set S first, so they
            # are never mistaken for a fault here; the pre-readiness window is excluded as well.
            if S["ready"] and adapter.locked_by_fault and S["fault_tick_s"] is None and not S["locked"] and S["stop"] is None \
                    and S["eof_at"] is None:
                S["fault_tick_s"] = now; S["adapter_fault"] = adapter.first_fault
                ev("adapter_fault_tick", now, first_fault=adapter.first_fault, adapter_phase=adapter.phase, board=board.snapshot())
            for raw in transport.pending_tx: board.feed(raw, board_ms)
            transport.pending_tx.clear()
            board.plant(DT_MS / 1000)
            s = board.snapshot(); st["max_abs_steer"] = max(st["max_abs_steer"], abs(s["actual_steer"]))
            h_at = hold_time() if S["fault_tick_s"] is None else S["fault_tick_s"]
            if s["actual_steer"]:
                st["steer_nonzero"] = True; st["last_nonzero_s"] = now
                if st["first_zero_after_hold_s"] is not None: st["nonzero_after_zero"] = True
            elif h_at is not None and st["first_zero_after_hold_s"] is None and now >= h_at:
                st["first_zero_after_hold_s"] = now
            st["host_ms"] += DT_MS
        fake.now = st["host_ms"] / 1000

    def apply_stop(msg, t):
        """bridge.apply for a StopMessage; remembers when the adapter's first_fault is merely the intentional lock label."""
        pre = adapter.first_fault; r = bridge.apply(msg, t)
        if pre is None and adapter.first_fault is not None and S["intentional_lock_label"] is None:
            S["intentional_lock_label"] = adapter.first_fault
        return r

    def lock(t, msg, cause_label, at=None):
        r = apply_stop(msg, t)
        if not S["locked"]:
            S["locked"] = True; S["hold_cause"] = cause_label; S["hold_at"] = t if at is None else at
        ev("hold_lock", t, msg_kind=msg.kind, cause=msg.cause, reason=msg.reason, bridge=r.action, disarm_sent=r.disarm_sent,
           hold_at_s=round(S["hold_at"], 3), adapter_phase=adapter.phase, board=board.snapshot())
        return r

    # ---- warm-up, explicit synthetic operator preparation, readiness gate (no real ARM anywhere: MockTransport)
    pump_to(0.3); ok = adapter.request_prepare(fake.now); ev("request_prepare", fake.now, accepted=ok, operator="synthetic")
    pump_to(0.8)
    if adapter.phase not in pb.PREPARED or adapter.locked_by_fault:
        board.destroy(); raise ReplayConfigError(f"C2 adapter not prepared by 0.8 s: phase {adapter.phase}, fault {adapter.first_fault}")
    S["ready"] = True; ev("prepared", fake.now, adapter_phase=adapter.phase)

    # ---- samples
    for row in data["records"]:
        t = float(row[0]); pump_to(t)
        if S["stop"] is not None:                             # normal/limit terminal already reached: never a fault
            S["ignored_after_stop"] += 1; ev("ignored_after_stop", t, board=board.snapshot()); continue
        if S["fault_tick_s"] is not None and not S["locked"]:  # genuine fault detected in a host tick while waiting
            lock(t, ps.StopMessage("hold", f"adapter fault at host tick {S['fault_tick_s']:.3f} s: {S['adapter_fault']}",
                                   clock_ms(t), cause="error"), f"adapter fault: {S['adapter_fault']}", at=S["fault_tick_s"])
        if S["locked"]:
            S["ignored_locked"] += 1; ev("ignored_locked", t, board=board.snapshot()); continue
        S["processed"] += 1
        try:
            pose = pr.rear_axle_pose(x=row[1], y=row[2], heading=row[3], heading_convention=pose_hdr["heading_convention"],
                                     map_yaw_from_enu_rad=pose_hdr["map_yaw_from_enu_rad"], frame_id=pose_hdr["frame_id"],
                                     frame_alignment_verified=pose_hdr["frame_alignment_verified"],
                                     reference_point=pose_hdr["reference_point"], position_valid=row[6], heading_valid=row[7],
                                     position_age_s=row[4], heading_age_s=row[5], max_age_s=pose_hdr["max_age_s"],
                                     planar_assumption_verified=pose_hdr["planar_assumption_verified"],
                                     offset_x_m=pose_hdr["offset_x_m"], offset_y_m=pose_hdr["offset_y_m"])
        except ValueError as exc:
            lock(t, ps.StopMessage("hold", f"C4 pose rejected: {exc}", clock_ms(t), cause="error"), f"C4 pose rejected: {exc}"); continue
        if not S["started"]:
            sel = state.select_mode(route, vehicle_stopped=True); armed = sel and state.arm(pose.as_route_pose())
            if not (armed and state.start()):
                lock(t, ps.StopMessage("hold", f"Q1 arm/start refused: {state.last_reason}", clock_ms(t), cause="error"),
                     f"Q1 arm refused: {state.last_reason}"); continue
            S["started"] = True; ev("q1_armed_started", t, progress=state.progress, reason=state.last_reason)
        msg = follower.step(pose, t)
        if isinstance(msg, ps.TargetMessage):
            S["emitted"] += 1; S["equiv"].append(msg.angle_cdeg)
            conv = sac.convert_message(msg, contract, synthetic=True, min_cdeg=min_cdeg, max_cdeg=max_cdeg)
            if isinstance(conv, ps.StopMessage):
                S["conversion_holds"] += 1; lock(t, conv, f"angle conversion refused: {conv.reason}"); continue
            S["left"].append(conv.angle_cdeg)
            r = bridge.apply(conv, t)
            if r.action == "target_accepted": S["accepted"] += 1
            else: S["refused"] += 1
            ev("target", t, seq=conv.sequence, equivalent_cdeg=msg.angle_cdeg, left_front_wheel_cdeg=conv.angle_cdeg,
               bridge=r.action, reason=r.reason, rear_axle=[round(pose.x, 3), round(pose.y, 3)], progress=state.progress,
               board=board.snapshot())
            if r.action != "target_accepted" and adapter.locked_by_fault:
                S["locked"] = True; S["hold_cause"] = S["hold_cause"] or f"target refused: {r.reason}"; S["hold_at"] = t
                S["adapter_fault"] = S["adapter_fault"] or adapter.first_fault
        elif msg.kind == "stop":
            S["stop"] = {"t_s": t, "cause": msg.cause, "reached": msg.reached, "reason": msg.reason}
            if msg.cause == "path_end" and msg.reached: S["path_end"] = dict(S["stop"])
            r = apply_stop(msg, t)
            ev("stop", t, cause=msg.cause, reached=msg.reached, reason=msg.reason, bridge=r.action, disarm_sent=r.disarm_sent,
               q1_state=state.state, board=board.snapshot())
        else:
            lock(t, msg, f"follower hold: {msg.reason}")

    # ---- EOF: always an explicit StopMessage (never angle 0), then drain (EOF is not a path end)
    t_eof = fake.now; S["eof_at"] = t_eof
    eof_msg = ps.StopMessage("stop", "replay input EOF: no more samples (not a path end)", clock_ms(t_eof), cause="limit")
    r = apply_stop(eof_msg, t_eof); ev("eof_stop", t_eof, bridge=r.action, disarm_sent=r.disarm_sent, adapter_phase=adapter.phase)
    pump_to(t_eof + DRAIN_S)
    final = board.snapshot(); tx = [sim.decode_tx(f) for f in transport.tx_frames]; board.destroy()
    if S["stop"] and S["stop"]["cause"] == "path_end" and S["stop"]["reached"]: terminal = "path_end"
    elif S["stop"]: terminal = "limit_stop"
    elif S["adapter_fault"]: terminal = "adapter_fault"
    elif S["locked"]: terminal = "hold_lock"
    else: terminal = "eof"
    # Genuine adapter fault vs the intentional bridge lock label (path_end/hold/EOF stop): report only the former.
    genuine_fault = adapter.first_fault if adapter.first_fault and adapter.first_fault != S["intentional_lock_label"] else None
    # Re-acceptance evidence comes from the bridge's own result list, not from a counter that nothing updates.
    hold_at = S["hold_at"]
    accepted_after_lock = sum(1 for b in bridge.results if b.action == "target_accepted" and hold_at is not None and b.now_s > hold_at)
    accepted_times = [b.now_s for b in bridge.results if b.action == "target_accepted"]
    path_end_after_lock = bool(S["path_end"] and hold_at is not None and S["path_end"]["t_s"] > hold_at)
    # Reference instant for the stop-response record: genuine fault tick > hold > normal stop (path_end/limit) > EOF stop.
    stop_ref = S["fault_tick_s"] if S["fault_tick_s"] is not None else hold_time()
    stop_kind = ("adapter_fault" if S["fault_tick_s"] is not None else "hold" if hold_at is not None
                 else "stop" if S["stop"] else "eof")
    first_zero = st["first_zero_after_hold_s"] if stop_ref is not None else None
    stop_response = {
        "model_only": True, "window_ms": STOP_WINDOW_MS, "reference": stop_kind,
        "note": "BoardModel steer output sampled every host tick; synthetic chain response only, NOT vehicle stopping performance. "
                "latency 0 means the model output was already 0 at the reference tick; the durable evidence is "
                "nonzero_after_first_zero == false over the rest of the replay and the drain",
        "hold_at_s": None if stop_ref is None else round(stop_ref, 3),
        "steer_nonzero_before_hold": bool(stop_ref is not None and st["last_nonzero_s"] is not None and st["last_nonzero_s"] < stop_ref),
        "last_nonzero_s": None if st["last_nonzero_s"] is None else round(st["last_nonzero_s"], 3),
        "first_zero_after_hold_s": None if first_zero is None else round(first_zero, 3),
        "latency_ms": None if first_zero is None else int(round((first_zero - stop_ref) * 1000)),
        "nonzero_after_first_zero": bool(st["nonzero_after_zero"]),
    }
    hold_stop_ok = stop_ref is None or (first_zero is not None and stop_response["latency_ms"] <= STOP_WINDOW_MS
                                        and not st["nonzero_after_zero"])
    checks = {
        "drive_and_pwm_always_zero": all(d.get("drive", 0) == 0 and d.get("pwm", 0) == 0 for d in tx),
        "no_malformed_command_frames": not any(d.get("malformed") for d in tx),
        "eof_stop_explicit_and_drained_to_zero": final["actual_steer"] == 0 and final["actual_drive"] == 0 and final["drive_pwm"] == 0,
        "adapter_locked_at_end": adapter.locked_by_fault and adapter.phase == "locked",
        "no_target_accepted_after_lock": accepted_after_lock == 0 and (hold_at is None or all(a <= hold_at for a in accepted_times)),
        "no_path_end_after_lock": not path_end_after_lock,
        "lock_implies_disarm_sent": all(e.disarm_sent for e in bridge.lock_events),
        "board_never_reversal": 7 not in board.reasons_seen,
        "locked_inputs_never_reached_follower": not (S["locked"] and S["stop"] and hold_at is not None and S["stop"]["t_s"] > hold_at),
        "hold_stops_board_output_within_window_model_only": hold_stop_ok,
        "normal_terminal_not_reported_as_fault": not (terminal in NORMAL_TERMINALS and genuine_fault),
    }
    result = {
        "schema_version": 1, "input_kind": data["input_kind"], "source": data["source"], "input_path": str(input_path) if input_path else None,
        "synthetic_control": True, "hardware_access": False, "closed_loop": False,
        "note": "command-flow replay of predetermined positions; board is a MODEL; success = replay invariants, not driving completion",
        "records": len(data["records"]), "samples": {"processed": S["processed"], "ignored_locked": S["ignored_locked"],
                                                     "ignored_after_stop": S["ignored_after_stop"]},
        "targets": {"emitted": S["emitted"], "accepted": S["accepted"], "refused": S["refused"], "conversion_holds": S["conversion_holds"],
                    "bridge_refused_total": bridge.refused_targets, "accepted_after_lock": accepted_after_lock,
                    "last_accepted_s": None if not accepted_times else round(max(accepted_times), 3)},
        "angles_cdeg": {"equivalent": S["equiv"], "left_front_wheel": S["left"], "contract_mode": contract.mode,
                        "left_wheel_limits": [min_cdeg, max_cdeg]},
        "hold": {"cause": S["hold_cause"], "at_s": None if hold_at is None else round(hold_at, 3),
                 "fault_tick_s": None if S["fault_tick_s"] is None else round(S["fault_tick_s"], 3),
                 "adapter_first_fault": genuine_fault, "intentional_lock_label": S["intentional_lock_label"],
                 "lock_events": len(bridge.lock_events)},
        "stop_response": stop_response,
        "locked": bool(S["locked"] or adapter.locked_by_fault), "adapter_phase": adapter.phase,
        "path_end": S["path_end"], "reached_by_input": bool(S["path_end"]), "terminal_cause": terminal, "stop": S["stop"],
        "eof": {"t_s": round(t_eof, 3), "bridge": r.action, "drain_s": DRAIN_S},
        "wire": {"command_frames": len(tx), "malformed": sum(1 for d in tx if d.get("malformed")),
                 "nonzero_drive_or_pwm": sum(1 for d in tx if d.get("drive", 0) or d.get("pwm", 0)),
                 "nonzero_steer_frames": sum(1 for d in tx if d.get("steer", 0))},
        "board": {"nonzero_steer_observed": st["steer_nonzero"], "max_abs_actual_steer": st["max_abs_steer"], "final": final,
                  "final_adc": round(board.adc, 1), "reasons_seen": sorted(board.reasons_seen)},
        "reprepare_events": "unsupported: restarting the replay is the explicit preparation boundary",
        "checks": checks, "ok": all(checks.values()), "events": events,
    }
    return result


# ------------------------------------------------------------------ demo scenarios
def _mutate(fix, name):
    d = copy.deepcopy(fix); recs = d["records"]
    if name == "bad_frame": d["pose"]["frame_id"] = "odom"
    elif name == "stale_heading": recs[10][5] = 2.0            # t=2.0 heading age > max_age_s; later rows fresh
    elif name == "input_gap": d["records"] = [r for r in recs if not (1.7 < r[0] < 2.8)]   # 1.0 s hole > command_stale_ms
    elif name == "unconfigured_contract": d["angle_contract"] = {"mode": sac.UNCONFIGURED_MODE, "source_kind": sac.SOURCE_KIND,
                                                                 "target_kind": sac.TARGET_KIND, "wheelbase_m": None, "track_m": None,
                                                                 "source": "deliberately unconfigured (demo)"}
    elif name == "eof_midroute": d["records"] = [r for r in recs if r[0] <= 2.0]
    elif name != "nominal": raise ValueError(f"unknown scenario {name}")
    return d


def scenario_checks(name, r):
    """Each scenario checks that the EXPECTED outcome happened, from engine evidence. Passing here does not mean the
    individual replay terminated normally (only nominal/eof_midroute do); see the exit-semantics note in the module doc."""
    acc, hold, sr = r["targets"]["accepted"], (r["hold"]["cause"] or ""), r["stop_response"]
    c = {"replay_invariants_ok": r["ok"], "not_reached_unless_nominal": r["reached_by_input"] == (name == "nominal"),
         "no_reaccept_after_lock": r["targets"]["accepted_after_lock"] == 0}
    if name == "nominal":
        c.update(accepted_targets=acc > 0, nonzero_left_angle=any(a != 0 for a in r["angles_cdeg"]["left_front_wheel"]),
                 board_nonzero_steer=r["board"]["nonzero_steer_observed"], path_end_by_input=r["terminal_cause"] == "path_end", no_hold=hold == "",
                 records_after_path_end_ignored_not_faulted=r["samples"]["ignored_after_stop"] > 0 and r["hold"]["adapter_first_fault"] is None)
    elif name == "bad_frame":
        c.update(no_target=acc == 0 and r["targets"]["emitted"] == 0, hold_is_frame="frame_id" in hold, locked=r["locked"] and r["hold"]["lock_events"] >= 1)
    elif name == "stale_heading":
        c.update(accepted_before=acc > 0, hold_is_stale_heading="heading stale" in hold, fresh_after_ignored=r["samples"]["ignored_locked"] > 0,
                 locked=r["locked"], board_final_zero=r["board"]["final"]["actual_steer"] == 0, at_s=r["hold"]["at_s"] == 2.0,
                 last_accept_before_hold=(r["targets"]["last_accepted_s"] or 9e9) < 2.0, no_path_end=r["path_end"] is None,
                 steer_nonzero_before_hold=sr["steer_nonzero_before_hold"],
                 board_zero_after_hold_model_only=sr["first_zero_after_hold_s"] is not None and not sr["nonzero_after_first_zero"])
    elif name == "input_gap":
        ft = r["hold"]["fault_tick_s"]
        c.update(accepted_before=acc > 0, adapter_stale_fault="STALE" in (r["hold"]["adapter_first_fault"] or ""),
                 terminal=r["terminal_cause"] == "adapter_fault", later_ignored=r["samples"]["ignored_locked"] > 0, locked=r["locked"],
                 fault_time_from_host_tick_inside_gap=ft is not None and 1.7 < ft < 2.8 and r["hold"]["at_s"] == ft,
                 steer_nonzero_before_fault=sr["steer_nonzero_before_hold"],
                 board_zero_after_fault_model_only=sr["first_zero_after_hold_s"] is not None and not sr["nonzero_after_first_zero"])
    elif name == "unconfigured_contract":
        c.update(no_accept=acc == 0, conversion_hold=r["targets"]["conversion_holds"] == 1 and "conversion failed" in hold, locked=r["locked"])
    elif name == "eof_midroute":
        c.update(accepted=acc > 0, terminal_eof=r["terminal_cause"] == "eof", no_stop_by_input=r["stop"] is None,
                 locked_by_eof_stop=r["locked"] and r["eof"]["bridge"] == "lock_disarm")
    return c


# ------------------------------------------------------------------ output / CLI
def _new_dir(p):
    p = Path(p)
    if p.exists(): raise ValueError(f"--out {p} already exists (refusing to overwrite)")
    p.mkdir(parents=True); return p


def _hashes(input_path=None):
    hs = {s: sha256_file(ROOT / s) for s in RELATED_SOURCES if (ROOT / s).is_file()}
    if input_path: hs["input:" + str(input_path)] = sha256_file(input_path)
    return hs


def _save(out, r, name="summary"):
    ev = r.pop("events"); (out / f"{name}.json").write_text(json.dumps(r, indent=1, ensure_ascii=False) + "\n")
    (out / (name.replace("summary", "events") + ".jsonl")).write_text("".join(json.dumps(e, ensure_ascii=False) + "\n" for e in ev))
    r["events"] = ev


def replay_exit_code(r):
    """0 only for a normal terminal (path_end confirmed by input, or plain EOF) with all invariants held.
    1: an invariant failed. 4: invariants held but the replay ended in hold/adapter_fault/limit_stop (abnormal)."""
    if not r["ok"]: return EXIT_INVARIANT_FAILED
    return EXIT_OK if r["terminal_cause"] in NORMAL_TERMINALS else EXIT_ABNORMAL_TERMINAL


def cmd_replay(a):
    data = load_input(a.input); out = _new_dir(a.out); r = run_replay(data, a.input); r["source_hashes"] = _hashes(a.input)
    code = replay_exit_code(r); r["exit_code"] = code
    r["exit_meaning"] = {0: "normal terminal (path_end by input or EOF), invariants held", 1: "replay invariant failed",
                         4: "abnormal terminal (hold/adapter_fault/limit_stop) although invariants held"}[code]
    _save(out, r)
    verdict = "OK" if code == EXIT_OK else ("INVARIANT-FAIL" if code == EXIT_INVARIANT_FAILED else "ABNORMAL-TERMINAL")
    print(f"replay {verdict} (exit {code}): terminal={r['terminal_cause']} reached_by_input={r['reached_by_input']} "
          f"accepted={r['targets']['accepted']} hold={r['hold']['cause']!r} (synthetic control, board MODEL, no hardware) -> {out}")
    return code


def cmd_demo(a):
    fix = load_input(ROOT / "tests/fixtures/c5_replay_synthetic.json")
    names = SCENARIOS if a.scenario == "all" else tuple(a.scenario.split(","))
    for n in names:
        if n not in SCENARIOS: raise ValueError(f"unknown scenario {n}")
    out = _new_dir(a.out)
    agg = {"synthetic": True, "hardware_access": False, "scenarios": {}, "source_hashes": _hashes()}
    for n in names:
        r = run_replay(_mutate(fix, n)); c = scenario_checks(n, r); r["scenario"] = n; r["scenario_checks"] = c
        _save(out, r, f"summary_{n}"); agg["scenarios"][n] = {"ok": all(c.values()), "checks": c, "terminal": r["terminal_cause"],
                                                            "hold": r["hold"]["cause"], "accepted": r["targets"]["accepted"]}
        print(f"=== {n}: {'OK' if all(c.values()) else 'FAIL'} terminal={r['terminal_cause']} hold={r['hold']['cause']!r} checks={json.dumps(c)}")
    agg["ok"] = all(s["ok"] for s in agg["scenarios"].values())
    agg["meaning"] = ("aggregate exit 0 = every scenario ended the way it was EXPECTED to (including deliberate holds/faults); "
                      "it does not mean the individual replays terminated normally")
    (out / "summary.json").write_text(json.dumps(agg, indent=1, ensure_ascii=False) + "\n")
    print("DEMO RESULT:", "ALL OK" if agg["ok"] else "FAILURES PRESENT", "(synthetic; no device; no driving success claimed; "
          "expected-failure scenarios count as OK when blocked as expected)")
    return 0 if agg["ok"] else 1


PREFLIGHT_MISSING = [
    "physical steering feedback (ADC) and a measured steering calibration: no operational calibration was provided to this replay "
    "tool (only the synthetic fixture was used); the vehicle's stored configuration was NOT read or observed",
    "actual bicycle-equivalent -> left front wheel angle relationship (A1 measurement; only the synthetic ideal Ackermann contract exists)",
    "map frame alignment: GNSS lat/lon -> map projection, map_yaw_from_enu_rad and antenna offset survey (reported sensor positions exist; installation and map alignment still need verification)",
    "initial heading and GNSS quality check at the start point (heading source, convention, age/validity) on the actual receiver",
    "current board firmware/protocol identification on the vehicle (host core tested only against the BoardModel)",
    "low-speed drive integration: drive is always 0 in this tool; no drive command path was exercised",
    "actual stop behaviour of the returned code on the vehicle: the stop switch exists (confirmed by the user); what remains is "
    "verifying that hold/DISARM from this code path really stops the steering and drive on the hardware",
]


def cmd_preflight(a):
    out = _new_dir(a.out)
    rep = {"status": "BLOCKED", "live_capable": False, "hardware_access": False, "missing": PREFLIGHT_MISSING,
           "available_synthetic_only": ["C2 host core + BoardModel", "C3 PathFollower synthetic config", "C4 pose conversion (pure)",
                                        "synthetic angle contract", "this replay tool"],
           "not_observed": "vehicle configuration files, sensors, board and receiver were not read, probed, configured or sent anything",
           "exit_code_meaning": "2 = BLOCKED: no live capability; not an error of the tool",
           "note": "read-only preparation report; the only side effect is this output folder"}
    (out / "preflight.json").write_text(json.dumps(rep, indent=1, ensure_ascii=False) + "\n")
    print("PREFLIGHT: BLOCKED (exit 2, no live capability; nothing observed on the vehicle). Missing:")
    for m in PREFLIGHT_MISSING: print("  - " + m)
    return EXIT_INPUT


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    p = sp.add_parser("replay"); p.add_argument("--input", required=True); p.add_argument("--out", required=True); p.set_defaults(fn=cmd_replay)
    p = sp.add_parser("demo"); p.add_argument("--synthetic", action="store_true", required=True); p.add_argument("--scenario", default="all")
    p.add_argument("--out", required=True); p.set_defaults(fn=cmd_demo)
    p = sp.add_parser("preflight"); p.add_argument("--out", required=True); p.set_defaults(fn=cmd_preflight)
    a = ap.parse_args(argv)
    try:
        return a.fn(a)
    except ValueError as exc:
        print(f"error: input/argument rejected: {exc}", file=sys.stderr); return 2
    except ReplayConfigError as exc:
        print(f"error: replay configuration: {exc}", file=sys.stderr); return 3


if __name__ == "__main__":
    sys.exit(main())
