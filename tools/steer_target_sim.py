#!/usr/bin/env python3
"""C2 offline simulation: host adapter <-> type150 board MODEL over real protocol bytes.

Everything here is SYNTHETIC: fake host/board clocks (deliberately offset), a
mock transport, the manual C++ core compiled as a board model, and an ADC plant
that moves with the board's steer output. No serial port is opened.

  python3 tools/steer_target_sim.py --synthetic --scenario all
  python3 tools/steer_target_sim.py --synthetic --scenario reverse --verbose
  python3 tools/steer_target_sim.py --calibration config/steering_calibration.json --scenario reach
     (operational record is null => shows the no-calibration behaviour)

Exit code 0 only when every scenario's expectations and the global invariants
(board never reports REVERSAL, drive/PWM bytes always 0, no drive output) hold.
"""
import argparse, ctypes, json, struct, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import steer_target_host as h  # noqa: E402
from joystick_protocol import Parser  # noqa: E402
import step_manual_control as smc  # noqa: E402
from step_manual_control import Client  # noqa: E402


class FakeTime:
    """Replaces step_manual_control.time inside the simulation so Client's real-time
    stall checks (dispatch deadline) see the same fake clock as the session. The
    production Client is unchanged; only the module clock is swapped for the run."""
    def __init__(self): self.now = 0.0
    def monotonic(self): return self.now
from step_manual_protocol import DISARM, ARM, CONTROL, ARMED, DISARMED, crc16  # noqa: E402

SCENARIOS = ("reach", "reverse", "lost_neutral", "stale_target", "board_restart", "crc_error", "duplicate_status",
             "no_calibration", "explicit_stop", "decreasing_table", "unidentified_protocol",
             # C2.1
             "stale_then_recover", "sensor_invalid_then_recover", "bad_target_input", "wrong_status_no_reverse")


class MockTransport:
    """Host side of the wire. Bytes written by the host are handed to the board by the engine."""

    def __init__(self):
        self.rx = bytearray(); self.tx_frames = []; self.pending_tx = []
        self.closed = False
    @property
    def in_waiting(self): return len(self.rx)
    @property
    def out_waiting(self): return 0
    def read(self, size):
        data = bytes(self.rx[:size]); del self.rx[:size]; return data
    def write(self, raw):
        self.tx_frames.append(bytes(raw)); self.pending_tx.append(bytes(raw)); return len(raw)
    def close(self): self.closed = True


class BoardModel:
    def __init__(self, lib, boot_id, adc0, sign_for_adc_increase, rate_counts_per_s=60.0):
        self.lib = lib; self.boot_id = boot_id; self.adc = float(adc0)
        self.sign = sign_for_adc_increase; self.rate = rate_counts_per_s
        self.h = lib.brd_create(boot_id); self.reasons_seen = set(); self.max_actual_drive = 0
    def destroy(self):
        if self.h: self.lib.brd_destroy(self.h); self.h = None
    def snapshot(self):
        d, s, st, r, p = ctypes.c_int8(), ctypes.c_int8(), ctypes.c_uint8(), ctypes.c_uint8(), ctypes.c_uint8()
        self.lib.brd_snapshot(self.h, d, s, st, r, p)
        self.reasons_seen.add(r.value); self.max_actual_drive = max(self.max_actual_drive, abs(d.value))
        return {"actual_drive": d.value, "actual_steer": s.value, "state": st.value, "reason": r.value, "drive_pwm": p.value}
    def plant(self, dt_s):
        s = self.snapshot()
        if s["actual_steer"]:
            self.adc += (1 if s["actual_steer"] == self.sign else -1) * self.rate * dt_s
            self.adc = min(1023.0, max(0.0, self.adc))
    def tick(self, board_ms):
        self.lib.brd_set_inputs(self.h, int(round(self.adc)), board_ms)
    def status(self, board_ms):
        buf = ctypes.create_string_buffer(47)
        if self.lib.brd_make_status(self.h, buf, board_ms, 0):
            return bytes(buf.raw)
        return None
    def feed(self, raw, board_ms):
        for b in raw: self.lib.brd_feed(self.h, b, board_ms)


def decode_tx(raw):
    """type2 command bytes -> dict (for invariants)."""
    if len(raw) != 19 or raw[:4] != bytes([0xF1, 0xA5, 2, 13]): return {"malformed": True}
    boot, req, action, drive, steer, deadman, pwm = struct.unpack_from("<IIBbbBB", raw, 4)
    return {"boot": boot, "req": req, "action": action, "drive": drive, "steer": steer, "deadman": deadman, "pwm": pwm}


def run_scenario(name, lib, cfg, verbose=False, seconds=6.0):
    """Returns dict(name, ok, checks, trace_lines, summary). cfg may be None (no calibration)."""
    dec = name == "decreasing_table"
    if dec and cfg is not None:
        cfg2 = h.StcConfig(); ctypes.memmove(ctypes.byref(cfg2), ctypes.byref(cfg), ctypes.sizeof(h.StcConfig))
        for i in range(cfg2.point_count): cfg2.angle_cdeg[i] = -cfg2.angle_cdeg[i]
        cfg2.steer_sign_for_adc_increase = -cfg2.steer_sign_for_adc_increase; cfg = cfg2
    fake = FakeTime(); real_time = smc.time; smc.time = fake
    try:
        return _run_scenario(name, lib, cfg, verbose, seconds, fake)
    finally:
        smc.time = real_time


def _run_scenario(name, lib, cfg, verbose, seconds, fake):
    core = h.TargetCore(lib, None if name == "no_calibration" else cfg)
    adapter = h.HostSteerAdapter(core)
    transport = MockTransport()
    client = Client(transport, adapter, None, encoder=h.encode_zero_pwm); client.parser = Parser()
    sign = cfg.steer_sign_for_adc_increase if cfg is not None else -1
    board = BoardModel(lib, boot_id=57, adc0=600, sign_for_adc_increase=sign)
    BOARD_OFFSET_MS = 123456  # board millis deliberately unrelated to host time
    host_ms = 0; dt_ms = 5; lines = []; checks = {}; events = []
    adc_history = []
    seq = 0; state = {"prepared_at": None, "targets": [], "phase_seen": set(), "restarted": False, "stopped_at": None,
                      "neutral_drops": 0, "dups": 0, "crc_corrupt": 0, "reprepare_after_stop": False}
    def log(msg): events.append((host_ms / 1000, msg))
    def prepared(): return adapter.phase in ("ready", "active")
    def send_target(angle):
        nonlocal seq
        seq += 1; ok = adapter.set_target(seq, angle, host_ms, host_ms / 1000)
        changed = not state["targets"] or state["targets"][-1][2] != angle or state["targets"][-1][3] != ok
        state["targets"].append((host_ms, seq, angle, ok))
        if changed: log(f"set_target seq={seq} angle={angle} accepted={ok} (kept fresh with new sequences every 200 ms)")
        return ok
    last_status_line = None
    while host_ms < seconds * 1000:
        now_s = host_ms / 1000; fake.now = now_s; board_ms = (host_ms + BOARD_OFFSET_MS) & 0xFFFFFFFF
        # ---- scenario script (host side)
        if host_ms == 300:
            ok = adapter.request_prepare(now_s); log(f"request_prepare -> {ok} (needs fresh DISARMED status)"); state["prepare_result"] = ok
        if name == "unidentified_protocol" and host_ms == 200:
            transport.rx += b"HENES F8 DRIVE TEST\r\n"
        if prepared() and state["prepared_at"] is None:
            state["prepared_at"] = host_ms; log("prepared (board neutral + ARM confirmed)")
        if state["prepared_at"] is not None and host_ms == state["prepared_at"] + 100:
            if name in ("reach", "reverse", "lost_neutral", "board_restart", "crc_error", "duplicate_status", "explicit_stop", "decreasing_table",
                        "stale_then_recover", "sensor_invalid_then_recover", "bad_target_input", "wrong_status_no_reverse"):
                send_target(400)      # +4.00 deg left => ADC 660 (increasing table) / 540 (decreasing)
            elif name == "stale_target":
                send_target(400)
        # ---- C2.1 scenarios
        if name in ("sensor_invalid_then_recover", "bad_target_input", "wrong_status_no_reverse") and state["targets"] \
                and host_ms % 200 == 0 and host_ms > state["targets"][-1][0] and not state.get("fault_at"):
            angle = state["targets"][-1][2]
            if name == "wrong_status_no_reverse" and host_ms >= state["prepared_at"] + 2600: angle = -450
            send_target(angle)
        if name in ("stale_then_recover", "sensor_invalid_then_recover", "bad_target_input", "wrong_status_no_reverse", "stale_target") \
                and state["prepared_at"] and adapter.locked_by_fault and not state.get("fault_at"):
            state["fault_at"] = host_ms; state["fault_reason"] = adapter.first_fault; log(f"adapter locked: {adapter.first_fault}")
        if name == "sensor_invalid_then_recover" and state["targets"] and host_ms == state["targets"][0][0] + 400:
            state["spike"] = True; log("sensor spike: ADC 1023 injected into status")
        if name == "bad_target_input" and state["targets"] and host_ms == state["targets"][0][0] + 400:
            for bad in ((65536 + 400, "int16 wrap"), (True, "bool"), (2 ** 32 + 5, "uint32 wrap seq")):
                pass
            ok = adapter.set_target(seq + 1, 65536 + 400, host_ms, now_s); log(f"bad target angle 65936 -> accepted={ok}")
            state["bad_results"] = [ok]
        if name == "bad_target_input" and state.get("fault_at") and host_ms == state["fault_at"] + 200:
            r2 = adapter.set_target(seq + 2, True, host_ms, now_s); r3 = adapter.set_target(2 ** 32 + 5, 400, host_ms, now_s)
            state["bad_results"] += [r2, r3]; log(f"bool angle -> {r2}, wrapped sequence -> {r3} (locked, all False)")
        if name in ("stale_then_recover", "sensor_invalid_then_recover", "bad_target_input") and state.get("fault_at"):
            if host_ms == state["fault_at"] + 300:
                ok = send_target(400); log(f"new sequence WITHOUT prepare -> accepted={ok} (must be False)"); state["resume_without_prepare"] = ok
            if host_ms == state["fault_at"] + 600:
                ok = adapter.request_prepare(now_s); log(f"explicit request_prepare -> {ok}"); state["reprepare"] = ok
            if state.get("reprepare") and prepared() and not state.get("resumed"):
                state["resumed"] = host_ms; log("prepared again; sending new target sequence")
            if state.get("resumed") and host_ms >= state["resumed"] and host_ms % 200 == 0:
                send_target(400 if name != "sensor_invalid_then_recover" else 400)
        if name == "wrong_status_no_reverse" and adapter.neutral_pending_req is not None and not state.get("forged") and state.get("ready_frame"):
            # While the adapter waits for the board to confirm the neutral, replay an OLD READY frame
            # (lower request_id, requested_steer 0). Must not count as acceptance; Session flags regression.
            transport.rx += state["ready_frame"]; state["forged"] = True; log("replayed an older READY status while neutral confirmation pending")
        if name == "stale_target" and state["targets"] and host_ms % 100 == 0 and host_ms > state["targets"][0][0]:
            # keep re-sending the SAME sequence: must not renew freshness
            _, s0, a0, _ = state["targets"][0]
            adapter.set_target(s0, a0, host_ms, now_s)
        if name in ("reach", "reverse", "lost_neutral", "board_restart", "crc_error", "duplicate_status", "decreasing_table") and state["targets"]:
            # planner keeps the target fresh with new sequences every 200 ms
            if host_ms % 200 == 0 and host_ms > state["targets"][-1][0]:
                angle = state["targets"][-1][2]
                if name in ("reverse", "lost_neutral") and host_ms >= state["prepared_at"] + 2600: angle = -450  # reverse to ADC 535
                if name == "board_restart" and host_ms >= state["prepared_at"] + 3000 and not state["reprepare_after_stop"]:
                    pass
                send_target(angle)
        if name == "explicit_stop" and state["targets"] and host_ms == state["targets"][0][0] + 800:
            adapter.stop("explicit stop request"); client.send(adapter.emergency_command()); state["stopped_at"] = host_ms; log("explicit stop -> DISARM sent")
        if name == "explicit_stop" and state["stopped_at"] and host_ms == state["stopped_at"] + 600:
            ok = send_target(400); log(f"new target after stop without prepare -> accepted={ok} (must be False)")
        if name == "board_restart" and state["prepared_at"] is not None and host_ms == state["prepared_at"] + 1500 and not state["restarted"]:
            board.destroy(); board = BoardModel(lib, boot_id=58, adc0=board.adc, sign_for_adc_increase=sign); state["restarted"] = True; log("BOARD RESTART (boot 57 -> 58)")
        # ---- board produces status
        board.tick(board_ms)
        frame = board.status(board_ms)
        if name == "unidentified_protocol":
            frame = None  # the wrong (ASCII) firmware never produces type150 frames
            if host_ms % 100 == 0: transport.rx += b"[MONITOR] Drive=STOP | Steering=1023 | POT=512\r\n"
        if frame and name == "wrong_status_no_reverse" and state["prepared_at"] and not state.get("ready_frame") and host_ms > state["prepared_at"] + 60:
            state["ready_frame"] = frame  # ARMED/READY, requested 0, early request_id
        if frame and name == "sensor_invalid_then_recover" and state.get("spike") and not state.get("spiked"):
            # One status frame carries ADC 1023 (electrical rail) with a valid CRC: a sensor spike.
            bad = bytearray(frame); bad[20], bad[21] = 1023 & 0xFF, 1023 >> 8
            crc = crc16(bytes(bad[2:45])); bad[45], bad[46] = crc & 0xFF, crc >> 8
            frame = bytes(bad); state["spiked"] = True
        if frame:
            deliver = [frame]
            if name == "duplicate_status" and state["prepared_at"] and host_ms > state["prepared_at"] + 1000 and host_ms % 400 == 0:
                deliver.append(frame); state["dups"] += 1
            if name == "crc_error" and state["prepared_at"] and host_ms >= state["prepared_at"] + 1500 and state["crc_corrupt"] == 0:
                bad = bytearray(frame); bad[20] ^= 0x01; deliver = [bytes(bad)]; state["crc_corrupt"] += 1; log("CRC-corrupted status delivered")
            for f in deliver: transport.rx += f
        # ---- host tick (parser -> adapter.step -> send)
        client.tick(now_s)
        # ---- deliver host tx to board, with scenario faults
        for raw in transport.pending_tx:
            d = decode_tx(raw)
            drop = False
            if name == "lost_neutral" and d.get("action") == CONTROL and d.get("steer") == 0 and d.get("deadman") == 0 \
                    and adapter.last_sent_sign != 0 and state["neutral_drops"] < 3:
                drop = True; state["neutral_drops"] += 1; log(f"neutral command to req {d['req']} LOST on the wire")
            if not drop: board.feed(raw, board_ms)
        transport.pending_tx.clear()
        # ---- plant + trace
        board.plant(dt_ms / 1000)
        adc_history.append((host_ms, board.adc))
        snap = board.snapshot()
        if adapter.trace and adapter.trace[-1].now_s == now_s:
            t = adapter.trace[-1]
            line = (f"t={now_s:5.2f}s phase={adapter.phase:15s} board(boot={t.board['boot_id'] if t.board else '-'},req={t.board['request_id'] if t.board else '-'},"
                    f"state={t.board['state'] if t.board else '-'},reason={h.BOARD_REASONS.get(t.board['reason'],'?') if t.board else '-'},adc={t.board['steering_adc'] if t.board else '-'}) "
                    f"target_adc={t.target_adc} core={t.core_reason}/{t.core_steer:+d} sent={'-' if not t.sent else ('DISARM' if t.sent['action']==DISARM else 'ARM' if t.sent['action']==ARM else f'CTRL steer={t.sent['steer']:+d} dm={t.sent['deadman']}')} "
                    f"model(actual_steer={snap['actual_steer']:+d},plant_adc={board.adc:.0f}) {t.note}")
            if verbose or line[:60] != (last_status_line or "")[:60]: lines.append(line)
            last_status_line = line
        for r in adapter.trace[-1:]:
            state["phase_seen"].add(adapter.phase)
        host_ms += dt_ms
    # ---- invariants
    tx = [decode_tx(r) for r in transport.tx_frames]
    checks["all_tx_wellformed"] = all(not d.get("malformed") for d in tx)
    checks["drive_and_pwm_always_zero"] = all(d.get("drive", 0) == 0 and d.get("pwm", 0) == 0 for d in tx)
    checks["board_never_reversal_error"] = 7 not in (board.reasons_seen | {t.board["reason"] for t in adapter.trace if t.board})
    checks["board_never_drive_output"] = board.max_actual_drive == 0
    prepared_at_s = None if state["prepared_at"] is None else state["prepared_at"] / 1000
    after = [t.board["reason"] for t in adapter.trace if t.board and prepared_at_s is not None and t.now_s >= prepared_at_s]
    # After a successful prepare the board must never report ARM_REJECTED(6), STALE(4), BAD_FRAME(3) or REVERSAL(7)
    # in any status the host saw. (C2.1: replaces a constant-True placeholder.)
    checks["board_no_protocol_faults_after_prepare"] = not ({3, 4, 6, 7} & set(after))
    core_reasons = [t.core_reason for t in adapter.trace]
    steers_sent = [t.sent["steer"] for t in adapter.trace if t.sent and t.sent["action"] == CONTROL]
    final_adc = board.adc
    summary = {"prepared_at_s": None if state["prepared_at"] is None else state["prepared_at"] / 1000,
               "targets": len(state["targets"]), "tx_frames": len(tx), "final_plant_adc": round(final_adc),
               "board_reasons_seen": sorted(h.BOARD_REASONS.get(r, str(r)) for r in board.reasons_seen),
               "core_reasons_seen": sorted(set(core_reasons)), "final_phase": adapter.phase,
               "neutral_drops": state["neutral_drops"], "dups": state["dups"]}
    def no_direct_reversal(seq_):
        prev = 0
        for s in seq_:
            if s and prev and s == -prev: return False
            prev = s  # a sent 0 clears the previous sign
        return True
    checks["no_direct_sign_flip_in_tx"] = no_direct_reversal(steers_sent)
    # scenario expectations. Without a validated calibration every scenario must reduce to "never moves".
    if cfg is None and name != "no_calibration":
        checks["never_moves_without_calibration"] = board.adc == 600 and all(s == 0 for s in steers_sent)
        checks["prepare_refused_no_arm_sent"] = state.get("prepare_result") is False and not any(d.get("action") == ARM for d in tx)
        summary_extra = {"note": "no validated calibration: expectations replaced by never-moves"}
        board.destroy()
        summary = {"final_plant_adc": round(final_adc), "tx_frames": len(tx), "core_reasons_seen": sorted(set(core_reasons)),
                   "final_phase": adapter.phase, **summary_extra}
        return {"name": name, "ok": all(checks.values()), "checks": checks, "summary": summary, "events": events, "lines": lines}
    if name in ("reach", "duplicate_status", "crc_error", "decreasing_table"):
        target = 660 if name != "decreasing_table" else 540
        checks["reached_target"] = ("AT_TARGET" in core_reasons) and abs(final_adc - target) <= (cfg.stop_band_adc + 3 if cfg is not None else 99)
        if name == "duplicate_status":
            # a duplicated status must not earn a second reply: replies <= distinct requests
            reqs = [t.board["request_id"] for t in adapter.trace if t.board]
            checks["one_reply_per_request"] = len([d for d in tx if d.get("action") != DISARM]) <= len(set(reqs)) + 2
        if name == "crc_error":
            checks["crc_error_locked_and_disarmed"] = any(d.get("action") == DISARM for d in tx) and adapter.locked_by_fault
            checks["reached_target"] = "AT_TARGET" in core_reasons  # reached before the corruption
    if name == "reverse":
        checks["moved_both_ways"] = (-1 in steers_sent) and (1 in steers_sent)
        checks["reversal_waited_for_board_neutral"] = any(t.note.startswith("reversal allowed: board neutral confirmed") for t in adapter.trace)
        checks["final_near_reverse_target"] = abs(final_adc - 535) <= (cfg.stop_band_adc + 3)
    if name == "lost_neutral":
        # A lost reply means the board lease (150 ms) expires -> TIMEOUT -> DISARMED. The host must
        # lock, never send the opposite sign, and require a new explicit prepare.
        checks["neutral_actually_lost"] = state["neutral_drops"] >= 1
        observed = {t.board["reason"] for t in adapter.trace if t.board} | board.reasons_seen
        checks["board_timed_out_not_reversal"] = (2 in observed) and (7 not in observed)
        checks["never_sent_reverse_sign"] = 1 not in steers_sent
        checks["adapter_locked_needs_prepare"] = adapter.locked_by_fault and adapter.phase == "locked"
        checks["targets_rejected_after_lock"] = any(not ok for _, _, _, ok in state["targets"])
    if name == "stale_target":
        checks["same_sequence_expires"] = "COMMAND_STALE" in core_reasons
        checks["adapter_locked_and_disarmed_after_stale"] = "COMMAND_STALE" in (state.get("fault_reason") or "") and adapter.locked_by_fault \
            and any(d.get("action") == DISARM and d.get("req", 0) > 12 for d in tx)
        tail = [a for t, a in adc_history if t >= host_ms - 1000]
        checks["no_motion_last_second"] = max(tail) - min(tail) < 0.5
        checks["did_move_before_stale"] = abs(final_adc - 600) > 5 and abs(final_adc - 660) > 5
    if name == "board_restart":
        checks["restart_detected_and_locked"] = state["restarted"] and adapter.locked_by_fault and adapter.phase == "locked"
        checks["no_control_to_new_boot_without_prepare"] = not any(d.get("boot") == 58 and d.get("action") in (ARM, CONTROL) for d in tx)
    if name == "no_calibration":
        checks["never_moves_without_calibration"] = board.adc == 600 and all(s == 0 for s in steers_sent)
        checks["prepare_refused_no_arm_sent"] = state.get("prepare_result") is False and not any(d.get("action") == ARM for d in tx) \
            and adapter.phase == "locked" and 1 not in {t.board["state"] for t in adapter.trace if t.board}
    if name == "explicit_stop":
        after = [d for d in tx if d.get("req", 0) and d.get("action") != DISARM and d.get("deadman") == 1]
        checks["disarm_sent_on_stop"] = any(d.get("action") == DISARM for d in tx)
        checks["no_new_target_without_prepare"] = state["targets"] and state["targets"][-1][3] is False
        checks["no_motion_after_stop"] = not any(t.sent and t.sent["action"] == CONTROL and t.sent["steer"] != 0 for t in adapter.trace if t.now_s * 1000 > state["stopped_at"])
    if name in ("stale_then_recover", "sensor_invalid_then_recover", "bad_target_input"):
        fault = state.get("fault_reason") or ""   # recorded at lock time (request_prepare clears first_fault)
        expect = {"stale_then_recover": "COMMAND_STALE", "sensor_invalid_then_recover": "SENSOR_INVALID", "bad_target_input": "invalid target input"}[name]
        checks["fault_detected_and_locked"] = state.get("fault_at") is not None and expect in fault
        checks["disarm_sent_on_fault"] = any(d.get("action") == DISARM and d.get("req", 0) > 12 for d in tx)
        checks["new_sequence_alone_does_not_resume"] = state.get("resume_without_prepare") is False
        checks["explicit_prepare_then_new_target_recovers"] = bool(state.get("reprepare")) and state.get("resumed") is not None
        checks["moves_again_after_recovery"] = any(t.sent and t.sent["action"] == CONTROL and t.sent["steer"] != 0 and t.now_s * 1000 > state.get("resumed", 10 ** 9) for t in adapter.trace)
        checks["first_fault_preserved_in_trace"] = any(t.note.startswith("first fault:") for t in adapter.trace)
        checks["same_core_object_kept"] = core.last_sequence() == seq  # sequences kept increasing on one core
        if name == "bad_target_input":
            checks["all_bad_inputs_rejected"] = state.get("bad_results") == [False, False, False]
            fa = state["fault_at"]
            checks["motion_stopped_after_bad_input"] = not any(t.sent and t.sent["action"] == CONTROL and t.sent["steer"] != 0
                                                             for t in adapter.trace if fa < t.now_s * 1000 < state.get("resumed", 10 ** 9))
    if name == "wrong_status_no_reverse":
        checks["forged_status_delivered"] = bool(state.get("forged"))
        checks["never_sent_reverse_sign"] = 1 not in steers_sent
        checks["locked_by_ordering_fault"] = adapter.locked_by_fault and "역행" in (state.get("fault_reason") or "")
    if name == "unidentified_protocol":
        checks["no_command_to_unidentified_board"] = len(tx) == 0 and adapter.phase == "locked" and client.parser.unsupported_banner
        checks["prepare_refused"] = state.get("prepare_result") is False
    board.destroy()
    return {"name": name, "ok": all(checks.values()), "checks": checks, "summary": summary, "events": events, "lines": lines}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="all", help=",".join(SCENARIOS) + " or all")
    ap.add_argument("--synthetic", action="store_true", help="use tests/fixtures/steering_calibration_synthetic.json (tests/demo only)")
    ap.add_argument("--calibration", default=None, help="calibration record path (validated; null record => no calibration)")
    ap.add_argument("--verbose", action="store_true"); ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    lib = h.load_library()
    if a.synthetic:
        cfg, reasons, syn = h.load_calibration(ROOT / "tests/fixtures/steering_calibration_synthetic.json", allow_synthetic=True)
        print("*** SYNTHETIC calibration, SYNTHETIC plant/clock/transport, board MODEL — not vehicle data ***")
    else:
        path = Path(a.calibration) if a.calibration else ROOT / "config/steering_calibration.json"
        cfg, reasons, syn = h.load_calibration(path)
        print(f"calibration {path}: {'READY' if cfg is not None else 'NOT READY -> adapter will never move'}")
        for r in reasons[:6]: print("  -", r)
        print("*** plant/clock/transport are SYNTHETIC, board is a MODEL ***")
    names = SCENARIOS if a.scenario == "all" else tuple(a.scenario.split(","))
    all_ok = True; results = []
    for n in names:
        if n not in SCENARIOS: ap.error(f"unknown scenario {n}")
        r = run_scenario(n, lib, cfg, verbose=a.verbose)
        results.append(r); all_ok &= r["ok"]
        print(f"\n=== scenario {n}: {'OK' if r['ok'] else 'FAIL'} ===")
        for t, m in r["events"]: print(f"  [{t:5.2f}s] {m}")
        for ln in r["lines"][: (400 if a.verbose else 40)]: print("  " + ln)
        print("  checks:", json.dumps(r["checks"]))
        print("  summary:", json.dumps(r["summary"]))
    if a.json:
        print(json.dumps([{k: v for k, v in r.items() if k != "lines"} for r in results], indent=1))
    print("\nRESULT:", "ALL OK" if all_ok else "FAILURES PRESENT", "(synthetic; no device, no vehicle stop verified)")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
