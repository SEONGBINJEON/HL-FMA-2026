#!/usr/bin/env python3
"""Finite type144 directional-rail board checks, with motor power explicitly OFF.

This firmware can produce real output signals. The two CLI assertions do not
sense power isolation. Root/operator must establish it before execution. This
checks reported firmware states, not physical movement, PWM waveforms, stopping
distance, E-stop wiring, or completion of keyboard/continuous driving tests.
"""

import argparse
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
import select
import signal
import time
import uuid
from zoneinfo import ZoneInfo

from manual_control import MegaTransport
from step_manual_protocol import (
    ARM, ARMED, CONTROL, DISARM, DISARMED, STEP_UNUSED_FLAGS, STEP_USED_FLAGS,
    Command, FrameParser, Status, encode_command,
)

ROOT = Path(__file__).resolve().parents[1]
SCOPE = ("Motor power OFF: real-board reported output-command logic only. "
         "No physical motion/PWM measurement/E-stop or keyboard/continuous "
         "driving validation. Motor isolation is an operator assertion.")


class VerificationError(RuntimeError):
    pass


class Verifier:
    """Nonblocking transport is injected; importing this module opens nothing."""

    def __init__(self, transport, folder, report, started):
        self.transport, self.folder, self.report = transport, folder, report
        self.started = started
        self.parser = FrameParser()
        self.current = None
        self.fresh_at = None
        self.last_rx_at = None
        self.consumed = None
        self.tx_broken = False
        self.phase = "identify"
        self.events = (folder / "events.jsonl").open("x", buffering=1)
        self.rx = (folder / "raw_rx.bin").open("xb", buffering=0)
        self.tx = (folder / "raw_tx.bin").open("xb", buffering=0)

    def event(self, kind, **details):
        self.events.write(json.dumps({"event": kind, "phase": self.phase,
                                     "elapsed_s": time.monotonic() - self.started,
                                     **details}, ensure_ascii=False) + "\n")

    def check(self, name, condition, **details):
        value = {"name": name, "passed": bool(condition), **details}
        self.report["checks"].append(value)
        self.event("check", **value)
        if not condition:
            raise VerificationError(name)

    def _accept(self, values, errors, now):
        if errors:
            self.event("pre_identity_noise" if self.current is None else "rx_error",
                       errors=errors)
            if self.current is not None:
                raise VerificationError("Malformed RX after identity: " + str(errors))
        for value in values:
            if not isinstance(value, Status):
                raise VerificationError("Unexpected command frame received")
            value.validate()
            if not value.boot_id or not value.request_id:
                raise VerificationError("Zero boot/request identity")
            previous = self.current
            if previous is None:
                # A CRC/identity-validated unexpected active board still earns
                # only an emergency DISARM in cleanup, never ARM/CONTROL.
                self.current = value
                if value.state != DISARMED or value.actual_drive or value.actual_steer:
                    raise VerificationError("Initial type137 status is not DISARMED/zero")
                self.report["first_status"] = asdict(value)
            else:
                if value.boot_id != previous.boot_id:
                    raise VerificationError("Board rebooted during verification")
                if value.request_id < previous.request_id:
                    raise VerificationError("Request ID regressed")
                if ((value.millis - previous.millis) & 0xffffffff) >= 0x80000000:
                    raise VerificationError("Board clock regressed")
            if previous is None or value.request_id != previous.request_id:
                self.fresh_at = now  # Duplicates never extend request freshness.
            self.current = value
            self.last_rx_at = now
            self.event("status", **asdict(value))

    def receive(self, *, after=None, seconds=.350):
        """Read all buffered bytes before returning only the newest full status."""
        end = min(time.monotonic() + seconds, self.started + 15)
        while time.monotonic() < end:
            began = time.monotonic()
            count = self.transport.in_waiting
            if count > 4096:
                raise VerificationError("RX flood")
            raw = self.transport.read(count) if count else b""
            if raw:
                self.rx.write(raw)
            # Byte-wise processing preserves whether an error preceded or
            # followed the first accepted type137 frame in the same OS read.
            self._accept(*self.parser.feed(b"", began), began)
            for byte in raw:
                self._accept(*self.parser.feed(bytes((byte,)), began), began)
            if time.monotonic() - began >= .100:
                raise VerificationError("Receive/logging stalled for 100ms")
            if raw and self.current is not None and not self.parser.buffer:
                if after is None or self.current.request_id > after:
                    return self.current
            select.select([], [], [], .002)
        raise VerificationError("No eligible type144 response before bounded deadline")

    def send(self, action, drive=0, steer=0, deadman=0, *, cleanup=False):
        if self.current is None or self.tx_broken:
            raise VerificationError("No identified board or TX already broken")
        began = time.monotonic()
        value = self.current
        token = (value.boot_id, value.request_id)
        if self.transport.out_waiting:
            self.tx_broken = True
            raise VerificationError("Pending TX: refusing to append commands")
        command = Command(*token, action, drive, steer, deadman)
        raw = encode_command(command)
        self.event("tx_attempt", cleanup=cleanup, **asdict(command))
        now = time.monotonic()
        if action != DISARM:
            if (token == self.consumed or self.parser.buffer or
                    now - self.fresh_at >= .100 or now - began >= .100 or
                    now - self.started >= 15):
                raise VerificationError("Stale/consumed/partial request; no ARM/CONTROL sent")
        try:
            written = self.transport.write(raw)
        except BaseException:
            self.tx_broken = True
            raise
        if written != len(raw):
            self.tx_broken = True
        self.tx.write(raw[:max(0, written)])
        self.event("tx_written", bytes=written, cleanup=cleanup, **asdict(command))
        if self.tx_broken:
            raise VerificationError("Partial TX; no retry or frame appended")
        self.consumed = token
        return now, value

    def exchange(self, action, drive=0, steer=0, deadman=0):
        sent_at, issued = self.send(action, drive, steer, deadman)
        return self.receive(after=issued.request_id), sent_at, issued

    def neutral_arm(self):
        value, _, _ = self.exchange(DISARM)
        self.check(self.phase + ": explicit DISARM", value.state == DISARMED and
                   value.reason == 1 and not value.actual_drive and not value.actual_steer)
        value, _, _ = self.exchange(CONTROL)
        self.check(self.phase + ": fresh neutral clears STEP_USED",
                   value.state == DISARMED and value.reason == 11 and
                   value.flags == STEP_UNUSED_FLAGS)
        value, _, _ = self.exchange(ARM)
        self.check(self.phase + ": new ARM remains zero",
                   value.state == ARMED and value.reason == 9 and
                   value.flags == STEP_UNUSED_FLAGS and
                   not value.actual_drive and not value.actual_steer)

    def scenario(self, name, drive, steer, mode, nominal_ms):
        self.phase = name
        self.neutral_arm()
        value, activation_at, issued = self.exchange(CONTROL, drive, steer, 1)
        self.check(name + ": output became active", value.flags == STEP_USED_FLAGS and
                   value.state == ARMED and value.reason == 10 and
                   (value.actual_drive, value.actual_steer) == (drive, steer))
        first_active = asdict(value)
        renewals = 0
        stop_sent_at = None
        if mode == "disarm":
            value, stop_sent_at, _ = self.exchange(DISARM)
        else:
            until = activation_at + (nominal_ms / 1000) + .180
            while value.actual_drive or value.actual_steer:
                if time.monotonic() >= until:
                    raise VerificationError(name + ": output did not stop within observation bound")
                if ((value.actual_drive, value.actual_steer) != (drive, steer) or
                        value.flags != STEP_USED_FLAGS or value.state != ARMED or value.reason != 10):
                    raise VerificationError(name + ": unexpected active output/state")
                # Withholding mode deliberately leaves the next token unused.
                if mode == "gate" and (value.boot_id, value.request_id) != self.consumed:
                    if renewals >= 52:
                        raise VerificationError("Renewal count exceeded bound")
                    self.send(CONTROL, drive, steer, 1)
                    renewals += 1
                value = self.receive(seconds=max(.001, until - time.monotonic()))
        observed_at = time.monotonic()
        host_elapsed_ms = (observed_at - activation_at) * 1000
        board_from_request_ms = (value.millis - issued.millis) & 0xffffffff
        stopped = value.flags == STEP_USED_FLAGS and not value.actual_drive and not value.actual_steer
        if mode == "gate":
            expected_state = ((value.state == ARMED and value.reason == 10 and
                               (value.requested_drive, value.requested_steer) == (drive, steer))
                              if drive else (value.state == DISARMED and value.reason == 8))
            # 50ms telemetry sampling means this is an observation window,
            # not a measured electrical pulse width. Premature ADC stops fail.
            stopped = stopped and expected_state and nominal_ms - 20 <= host_elapsed_ms <= nominal_ms + 160
        elif mode == "lease":
            stopped = stopped and value.state == DISARMED and value.reason == 2 and board_from_request_ms <= 220
        else:
            stopped = stopped and value.state == DISARMED and value.reason == 1 and (observed_at - stop_sent_at) <= .150
        self.check(name + ": bounded stop observed", stopped,
                   requested_limit_ms=nominal_ms, host_activation_to_stop_status_ms=host_elapsed_ms,
                   mcu_request_status_to_stop_status_ms=board_from_request_ms,
                   explicit_stop_to_status_ms=(observed_at - stop_sent_at) * 1000 if stop_sent_at else None,
                   renewal_commands=renewals, first_active=first_active, stopped_status=asdict(value),
                   timing_scope="Firmware telemetry sampled about every 50ms; not physical pulse/stop timing")
        if mode != "disarm":
            value, _, _ = self.exchange(DISARM)
            self.check(name + ": final DISARM keeps STEP_USED",
                       value.state == DISARMED and value.reason == 1 and
                       value.flags == STEP_USED_FLAGS and not value.actual_drive and not value.actual_steer)

    def run(self):
        self.receive(seconds=4)
        self.phase = "persistent_sequence"
        self.neutral_arm()

        value, drive_started, _ = self.exchange(CONTROL, 1, 0, 1)
        self.check("held forward starts", value.actual_drive == 1 and value.actual_steer == 0)
        renewals = 0
        while time.monotonic() - drive_started < 2.25:
            self.check("held forward remains active", value.state == ARMED and
                       value.actual_drive == 1 and value.actual_steer == 0)
            if (value.boot_id, value.request_id) != self.consumed:
                self.send(CONTROL, 1, 0, 1)
                renewals += 1
            value = self.receive(seconds=.20)
        self.check("forward exceeds former two-second cap", value.actual_drive == 1,
                   held_seconds=time.monotonic() - drive_started, renewal_commands=renewals)

        value, steer_started, _ = self.exchange(CONTROL, 1, 1, 1)
        self.check("steering joins held drive", value.actual_drive == 1 and value.actual_steer == 1)
        renewals = 0
        while value.actual_steer:
            if time.monotonic() - steer_started > 1.20:
                raise VerificationError("steering protection did not stop within observation bound")
            if (value.boot_id, value.request_id) != self.consumed:
                self.send(CONTROL, 1, 1, 1)
                renewals += 1
            value = self.receive(seconds=.20)
        self.check("steering protection stops steering while drive continues",
                   value.state == ARMED and value.requested_drive == 1 and
                   value.requested_steer == 1 and value.actual_drive == 1 and not value.actual_steer,
                   elapsed_ms=(time.monotonic() - steer_started) * 1000,
                   renewal_commands=renewals)

        value, _, _ = self.exchange(CONTROL, 1, 0, 1)
        self.check("steering release keeps straight drive active",
                   value.state == ARMED and value.actual_drive == 1 and not value.actual_steer)
        value, _, _ = self.exchange(CONTROL)
        self.check("all keys released restores prepared zero",
                   value.state == ARMED and value.reason == 9 and not value.actual_drive and not value.actual_steer)
        value, _, _ = self.exchange(CONTROL, -1, 0, 1)
        self.check("later reverse starts in same ARM", value.actual_drive == -1)
        value, _, _ = self.exchange(DISARM)
        self.check("explicit stop disarms and zeros output", value.state == DISARMED and
                   value.reason == 1 and not value.actual_drive and not value.actual_steer)

        self.phase = "lease_timeout"
        self.neutral_arm()
        value, _, issued = self.exchange(CONTROL, 1, 0, 1)
        while value.state == ARMED:
            value = self.receive(seconds=.25)
        elapsed = (value.millis - issued.millis) & 0xffffffff
        self.check("withheld communication expires and stops",
                   value.state == DISARMED and value.reason == 2 and
                   not value.actual_drive and not value.actual_steer and elapsed <= 220,
                   board_elapsed_ms=elapsed)

    def close_logs(self):
        for stream in (self.rx, self.tx, self.events):
            stream.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--motor-power-off", action="store_true")
    ap.add_argument("--output-dir", type=Path, help="New directory only")
    args = ap.parse_args(argv)
    if not args.verify or not args.motor_power_off:
        ap.error("Both --verify and --motor-power-off are required; no device opened")
    stamp = datetime.now(ZoneInfo("Asia/Seoul"))
    folder = args.output_dir or ROOT / "data" / (stamp.strftime("step_board_check_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
    folder = folder.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    report = {"status": "starting", "scope": SCOPE, "checks": [],
              "started_kst": stamp.isoformat(), "motor_power_off_asserted": True,
              "source_sha256": {}, "maximum_test_seconds": 15}
    for relative in ("tools/verify_step_board.py", "tools/manual_control.py", "tools/manual_protocol.py",
                     "tools/step_manual_protocol.py", "firmware/step_manual/step_manual.ino",
                     "firmware/step_manual/step_gate.h", "firmware/step_manual/control_core.h",
                     "firmware/step_manual/control_core.cpp"):
        raw = (ROOT / relative).read_bytes()
        target = folder / "sources" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(raw)
        report["source_sha256"][relative] = hashlib.sha256(raw).hexdigest()
    print("OUTPUT=" + str(folder), flush=True)
    started = time.monotonic()
    transport = verifier = None
    def expired(_signum, _frame):
        raise VerificationError("15-second wall-clock test deadline")
    previous_handler = signal.signal(signal.SIGALRM, expired)
    previous_term = signal.signal(signal.SIGTERM, expired)
    signal.setitimer(signal.ITIMER_REAL, 15)
    try:
        transport = MegaTransport()
        verifier = Verifier(transport, folder, report, started)
        verifier.run()
        report["status"] = "passed"
    except (Exception, KeyboardInterrupt) as exc:
        report["status"] = "failed"
        report["error"] = type(exc).__name__ + ": " + str(exc)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        if verifier is not None:
            try:
                verifier.phase = "cleanup"
                verifier.send(DISARM, cleanup=True)
                report["cleanup_disarm_written"] = True
            except (Exception, KeyboardInterrupt) as exc:
                report["cleanup_disarm_written"] = False
                report["cleanup_error"] = str(exc)
                if report["status"] == "passed":
                    report["status"] = "failed"
        if transport is not None:
            try:
                transport.close()
                report["port_closed"] = True
            except Exception as exc:
                report["port_closed"] = False
                report["close_error"] = str(exc)
                report["status"] = "failed"
        if verifier is not None:
            verifier.close_logs()
        signal.signal(signal.SIGALRM, previous_handler)
        signal.signal(signal.SIGTERM, previous_term)
        report["elapsed_seconds"] = time.monotonic() - started
        (folder / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "checks": len(report["checks"]),
                      "report": str(folder / "report.json"), "error": report.get("error")}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
