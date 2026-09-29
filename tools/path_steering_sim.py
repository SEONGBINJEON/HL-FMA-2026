#!/usr/bin/env python3
"""C3 demo: synthetic route -> Q1.1 RunState -> pure pursuit -> C2 adapter -> type150 board MODEL.

Everything is SYNTHETIC (route, bicycle vehicle model, calibration fixture, clocks, transport).
The vehicle model moves at a constant modelled speed only while C2 is prepared AND holds an
accepted target; the board is never asked to drive (C2 sends drive 0). Nothing here is a real
steering response, a real stop or a course pass.

C3.1 scenarios:
  nominal            full run; completion only inside the explicit end criterion (not "lookahead failed")
  hold_transient     one invalid pose mid-run -> hold -> C2 lock + DISARM, board output 0, vehicle model
                     stops; valid poses alone do not resume; explicit reprepare + new target resumes
  hold_stale         stale poses for 1 s (Q1 hold) -> same lock/resume rules
  hold_no_reprepare  hold without reprepare: never moves again, never "completed"

  python3 tools/path_steering_sim.py --synthetic [--scenario all|nominal|...] [--verbose]
Exit 0 only when every scenario's invariants hold.
"""
import argparse, json, math, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import qualifier_route as qr  # noqa: E402
import path_steering as ps  # noqa: E402
import path_steering_bridge as pb  # noqa: E402
import steer_target_host as h  # noqa: E402
import steer_target_sim as sim  # noqa: E402
import step_manual_control as smc  # noqa: E402
from joystick_protocol import Parser  # noqa: E402
from step_manual_control import Client  # noqa: E402
from step_manual_protocol import DISARM  # noqa: E402

SCENARIOS = ("nominal", "hold_transient", "hold_stale", "hold_no_reprepare")
# (start_s, end_s inclusive at 100 ms follower steps, pose fault kind)
FAULTS = {"nominal": [], "hold_transient": [(20.0, 20.0, "position_invalid")], "hold_stale": [(20.0, 21.0, "stale")],
          "hold_no_reprepare": [(20.0, 20.0, "position_invalid")]}
REPREPARE_AT = {"nominal": None, "hold_transient": 21.0, "hold_stale": 22.0, "hold_no_reprepare": None}


def synthetic_route():
    """10 m straight (east), 90 deg left arc R=6 m, 8 m straight (north). ~1 m spacing."""
    pts = []
    for i in range(11): pts.append((float(i), 0.0))
    R = 6.0; cx, cy = 10.0, R
    for k in range(1, 10):
        t = -math.pi / 2 + k * (math.pi / 2) / 9
        pts.append((cx + R * math.cos(t), cy + R * math.sin(t)))
    for i in range(1, 9): pts.append((16.0, R + float(i)))
    out = []
    for i, (x, y) in enumerate(pts):
        nx, ny = pts[min(i + 1, len(pts) - 1)]; px, py = pts[max(i - 1, 0)]
        yaw = math.atan2(ny - py, nx - px)
        out.append(qr.Point(i, x, y, yaw, "synthetic_c3"))
    return qr.ModeRoute("synthetic_c3", [qr.Segment("synthetic_c3", 0, len(out) - 1, "synthetic")], out)


def end_geometry(route, veh, cfg):
    """Independent arrival check from the simulator's own vehicle position (not from message text)."""
    es = ps.evaluate_end(route.points, veh["x"], veh["y"], cfg)
    return {"distance_m": round(es.distance_m, 3), "along_m": round(es.along_m, 3), "lateral_m": round(es.lateral_m, 3),
            "status": es.status, "reached": es.reached}


def run(verbose=False, scenario="nominal", speed_mps=0.35, seconds=150.0, start_offset_m=0.3):
    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario {scenario}")
    lib = h.load_library()
    cfg_c2, _, _ = h.load_calibration(ROOT / "tests/fixtures/steering_calibration_synthetic.json", allow_synthetic=True)
    fake = sim.FakeTime(); real_time = smc.time; smc.time = fake
    try:
        return _run(lib, cfg_c2, fake, verbose, scenario, speed_mps, seconds, start_offset_m)
    finally:
        smc.time = real_time


def _faulted_pose(pose, kind):
    if kind == "position_invalid":
        return ps.PoseInput(pose.x, pose.y, pose.yaw, position_valid=False)
    if kind == "stale":
        return ps.PoseInput(pose.x, pose.y, pose.yaw, age_s=2.0)
    raise ValueError(kind)


def _run(lib, cfg_c2, fake, verbose, scenario, speed, seconds, start_offset):
    cfg = ps.SYNTHETIC_CONFIG
    route = synthetic_route()
    state = qr.RunState(qr.SYNTHETIC_PARAMS)
    follower = ps.PathFollower(state, cfg, lambda s: int(s * 1000) & 0xFFFFFFFF)
    core = h.TargetCore(lib, cfg_c2); adapter = h.HostSteerAdapter(core)
    transport = sim.MockTransport(); client = Client(transport, adapter, None, encoder=h.encode_zero_pwm); client.parser = Parser()
    bridge = pb.FollowerC2Bridge(adapter, client)
    board = sim.BoardModel(lib, boot_id=71, adc0=600, sign_for_adc_increase=cfg_c2.steer_sign_for_adc_increase, rate_counts_per_s=120.0)
    veh = {"x": 0.0, "y": start_offset, "yaw": 0.0}
    L = cfg.wheelbase_m
    host_ms = 0; dt = 5; lines = []; events = []; lat_err = []; msgs = {"steer": 0, "hold": 0, "stop": 0}
    started = False; stop_msg = None; end_reason = None
    faults = FAULTS[scenario]; reprepare_at = REPREPARE_AT[scenario]
    hold = {"first_at": None, "veh_at_lock": None, "steer_after_ms": [], "moved_during_lock": False, "reprepared": None,
            "prepared_again": None, "target_accepted_after": None, "veh_at_reprepare": None, "refused_before_reprepare": 0}
    reached_claims = []   # (t, geometry) each time a message claims reached=True
    stop_geometry = None
    while host_ms < seconds * 1000:
        now = host_ms / 1000; fake.now = now; board_ms = (host_ms + 99999) & 0xFFFFFFFF
        pose = ps.PoseInput(veh["x"], veh["y"], veh["yaw"], age_s=0.0)
        for a, b, kind in faults:
            if a - 1e-9 <= now <= b + 1e-9: pose = _faulted_pose(pose, kind)
        if host_ms == 300:
            ok = adapter.request_prepare(now); events.append((now, f"C2 request_prepare -> {ok}"))
        if adapter.phase in pb.PREPARED and not started:
            assert state.select_mode(route, vehicle_stopped=True)
            armed = state.arm(pose.as_route_pose()); events.append((now, f"Q1.1 arm at entry -> {armed} ({state.last_reason})"))
            assert armed and state.start(); started = True
        if reprepare_at is not None and abs(now - reprepare_at) < 1e-9 and hold["first_at"] is not None:
            ok = bridge.reprepare(now); hold["reprepared"] = (now, ok); hold["veh_at_reprepare"] = dict(veh)
            events.append((now, f"EXPLICIT reprepare -> {ok} (operator decision; a new target sequence is still required)"))
        if hold["first_at"] is not None and hold["prepared_again"] is None and hold["reprepared"] and adapter.phase in pb.PREPARED:
            hold["prepared_again"] = now; events.append((now, "C2 prepared again (neutral + ARM confirmed); no target yet"))
        if started and host_ms % 100 == 0 and stop_msg is None:
            msg = follower.step(pose, now); msgs[msg.kind] += 1
            r = bridge.apply(msg, now)
            if msg.kind == "steer":
                lat_err.append(msg.decision["lateral_error_m"])
                if r.action == "target_accepted" and hold["first_at"] is not None and hold["prepared_again"] is not None and hold["target_accepted_after"] is None:
                    hold["target_accepted_after"] = now; events.append((now, f"new target seq {msg.sequence} accepted after reprepare -> resume"))
                if r.action != "target_accepted" and hold["first_at"] is not None and hold["reprepared"] is None:
                    hold["refused_before_reprepare"] += 1
            else:
                if msg.reached:
                    reached_claims.append((now, end_geometry(route, veh, cfg)))
                if msg.kind == "hold":
                    if hold["first_at"] is None:
                        hold["first_at"] = now; hold["veh_at_lock"] = dict(veh)
                        events.append((now, f"HOLD ({msg.reason}) -> bridge {r.action}, DISARM sent={r.disarm_sent}, C2 phase={adapter.phase}"))
                else:
                    stop_msg = msg; end_reason = msg.reason; stop_geometry = end_geometry(route, veh, cfg)
                    events.append((now, f"STOP cause={msg.cause} reached={msg.reached} -> bridge {r.action}, DISARM sent={r.disarm_sent}: {msg.reason}"))
        # board + wire
        board.tick(board_ms); frame = board.status(board_ms)
        if frame: transport.rx += frame
        client.tick(now)
        for raw in transport.pending_tx: board.feed(raw, board_ms)
        transport.pending_tx.clear()
        board.plant(dt / 1000)
        if hold["first_at"] is not None and now - hold["first_at"] < 0.050 + 1e-9:
            hold["steer_after_ms"].append(board.snapshot()["actual_steer"])
        # synthetic bicycle model: wheel angle taken from the SYNTHETIC calibration table of the board's ADC
        import ctypes
        ang = ctypes.c_int16(); ok = lib.stc_adc_to_angle(ctypes.byref(cfg_c2), int(round(board.adc)), ctypes.byref(ang))
        delta = math.radians(ang.value / 100.0) if ok else 0.0
        moving = started and stop_msg is None and adapter.phase in pb.PREPARED and adapter.have_target_since_prepare
        if moving:
            if hold["first_at"] is not None and hold["target_accepted_after"] is None:
                hold["moved_during_lock"] = True
            veh["x"] += speed * math.cos(veh["yaw"]) * dt / 1000; veh["y"] += speed * math.sin(veh["yaw"]) * dt / 1000
            veh["yaw"] = qr.wrap(veh["yaw"] + speed / L * math.tan(delta) * dt / 1000)
        if verbose and host_ms % 500 == 0 and started:
            lines.append(f"t={now:5.1f}s veh=({veh['x']:.2f},{veh['y']:.2f},{math.degrees(veh['yaw']):+.0f}deg) progress={state.progress} q1={state.state} "
                         f"adc={board.adc:.0f} wheel_model={math.degrees(delta):+.1f}deg last={follower.last.kind}:{getattr(follower.last,'angle_cdeg',None)} c2={adapter.phase}")
        host_ms += dt
    tx = [sim.decode_tx(r) for r in transport.tx_frames]
    final_geom = end_geometry(route, veh, cfg)
    checks = {
        "stop_is_explicit_message_not_angle_zero": stop_msg is None or (isinstance(stop_msg, ps.StopMessage) and not hasattr(stop_msg, "angle_cdeg")),
        "no_premature_completion": all(g["reached"] for _, g in reached_claims),   # every reached=True claim was geometrically true
        "drive_and_pwm_always_zero": all(d.get("drive", 0) == 0 and d.get("pwm", 0) == 0 for d in tx),
        "board_never_reversal": 7 not in board.reasons_seen,
        "c2_disarm_sent_on_every_lock": all(e.disarm_sent for e in bridge.lock_events) and bool(bridge.lock_events),
    }
    expects_completion = scenario != "hold_no_reprepare"
    if expects_completion:
        checks.update({
            "completed_inside_end_criterion": stop_msg is not None and stop_msg.kind == "stop" and stop_msg.cause == "path_end"
                                              and stop_msg.reached and stop_geometry["reached"] and state.state == state.END,
            "tracking_error_bounded": max(abs(e) for e in lat_err) <= 0.6 if lat_err else False,
            "converged_from_offset": abs(lat_err[-1]) < 0.15 if lat_err else False,
            "c2_lock_after_stop": adapter.locked_by_fault and adapter.phase == "locked",
            "disarm_sent_at_stop": any(d.get("action") == DISARM and d.get("req", 0) > 20 for d in tx),
        })
    else:
        checks.update({
            "not_completed_without_reprepare": stop_msg is None and state.state == state.RUNNING,
            "never_moves_again_after_hold": hold["first_at"] is not None and not hold["moved_during_lock"]
                                            and veh["x"] == hold["veh_at_lock"]["x"] and veh["y"] == hold["veh_at_lock"]["y"],
            "still_locked_at_end": adapter.locked_by_fault and adapter.phase == "locked",
            "valid_poses_alone_refused": hold["refused_before_reprepare"] > 0,
        })
    if faults:
        checks.update({
            "hold_emitted": msgs["hold"] >= 1 and hold["first_at"] is not None,
            "hold_locked_c2_immediately": bool(bridge.lock_events) and bridge.lock_events[0].msg_kind == "hold" and bridge.lock_events[0].disarm_sent,
            "board_output_zero_within_50ms_after_hold": bool(hold["steer_after_ms"]) and not any(hold["steer_after_ms"]),
            "no_vehicle_motion_during_lock": not hold["moved_during_lock"],
        })
        if reprepare_at is not None:
            checks.update({
                "valid_pose_alone_does_not_resume": hold["refused_before_reprepare"] > 0 and hold["veh_at_reprepare"] == hold["veh_at_lock"],
                "explicit_reprepare_then_new_target_resumes": bool(hold["reprepared"] and hold["reprepared"][1]) and hold["prepared_again"] is not None
                                                             and hold["target_accepted_after"] is not None,
            })
    summary = {"scenario": scenario, "messages": msgs, "bridge": {"locks": len(bridge.lock_events), "refused_targets": bridge.refused_targets},
               "final_vehicle": {k: round(v, 2) for k, v in veh.items()}, "final_progress": state.progress, "q1_state": state.state,
               "route_points": len(route.points), "max_abs_lateral_error_m": round(max(abs(e) for e in lat_err), 3) if lat_err else None,
               "final_lateral_error_m": round(lat_err[-1], 3) if lat_err else None,
               "stop": None if stop_msg is None else {"cause": stop_msg.cause, "reached": stop_msg.reached, "reason": end_reason},
               "end_geometry_at_stop": stop_geometry, "end_geometry_final": final_geom,
               "end_criterion": {"reach_tolerance_m": cfg.end_reach_tolerance_m, "pass_max_overshoot_m": cfg.end_pass_max_overshoot_m, "reference": cfg.end_reference},
               "hold": {"first_at_s": hold["first_at"], "board_actual_steer_first_50ms_after_hold": hold["steer_after_ms"],
                        "reprepared": hold["reprepared"], "prepared_again_s": hold["prepared_again"], "resumed_s": hold["target_accepted_after"],
                        "targets_refused_before_reprepare": hold["refused_before_reprepare"]},
               "note": "synthetic route/vehicle model/calibration/clock/transport; board is a MODEL; no real stop or course pass"}
    board.destroy()
    return {"ok": all(checks.values()), "checks": checks, "summary": summary, "events": events, "lines": lines}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--synthetic", action="store_true", required=True, help="acknowledge synthetic-only demo")
    ap.add_argument("--scenario", default="all", help="|".join(SCENARIOS) + " or all")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args(argv)
    print("*** SYNTHETIC route, vehicle model, calibration, clocks, transport; board is a MODEL ***")
    names = SCENARIOS if a.scenario == "all" else tuple(a.scenario.split(","))
    all_ok = True
    for n in names:
        if n not in SCENARIOS: ap.error(f"unknown scenario {n}")
        r = run(verbose=a.verbose, scenario=n); all_ok &= r["ok"]
        print(f"\n=== scenario {n}: {'OK' if r['ok'] else 'FAIL'} ===")
        for t, m in r["events"]: print(f"  [{t:6.2f}s] {m}")
        for ln in r["lines"]: print("  " + ln)
        print("  checks:", json.dumps(r["checks"])); print("  summary:", json.dumps(r["summary"], ensure_ascii=False))
    print("\nRESULT:", "ALL OK" if all_ok else "FAILURES PRESENT", "(synthetic; no device, no real stop verified)")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
