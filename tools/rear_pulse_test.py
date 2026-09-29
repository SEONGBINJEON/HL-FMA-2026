#!/usr/bin/env python3
"""Prepare one rear-motor pulse; explicit stdin 'fire' is required once.

Type 132 firmware only. Keep the already-open port across preparation/fire;
never reconnect automatically. Physical preparation must be confirmed outside
this program. No motion is inferred from firmware-reported output values.
"""

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import struct
import sys
import time
import uuid

from manual_control import MegaTransport
from manual_protocol import Command, STATUS_STRUCT, ARM, CONTROL, DISARM, crc16, encode_command

ROOT = Path(__file__).resolve().parents[1]
UNUSED, USED = 34, 98


@dataclass(frozen=True)
class PulseStatus:
    boot_id: int
    request_id: int
    millis: int
    encoder_count: int
    steering_adc: int
    state: int
    reason: int
    requested_drive: int
    requested_steer: int
    actual_drive: int
    actual_steer: int
    flags: int

    def validate(self):
        if not self.boot_id or not self.request_id or self.flags not in (UNUSED, USED):
            raise ValueError("Unexpected boot/request/pulse flags")
        if self.state not in (0, 1) or self.reason not in range(12) or not 0 <= self.steering_adc <= 1023:
            raise ValueError("Invalid state/reason/ADC")
        if self.requested_steer != 0 or self.actual_steer != 0:
            raise ValueError("Steering is forbidden in rear-only test")
        if self.requested_drive not in (0, 1) or self.actual_drive not in (0, 1):
            raise ValueError("Unexpected drive direction")
        if self.state == 0 and (self.requested_drive or self.actual_drive):
            raise ValueError("DISARMED state reports drive output")
        if self.actual_drive and (self.flags != USED or self.state != 1 or self.requested_drive != 1):
            raise ValueError("Actual output disagrees with request/used/state")


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.started = None

    def feed(self, raw, now):
        result, errors = [], []
        if len(raw) > 4096:
            self.buffer.clear()
            self.started = None
            return [], ["RX flood"]
        if self.buffer and now - self.started >= .030:
            self.buffer.clear()
            self.started = None
            errors.append("partial frame expired")
        for byte in raw:
            if not self.buffer:
                if byte == 0xf1:
                    self.buffer.append(byte)
                    self.started = now
                elif not errors or errors[-1] != "unexpected byte":
                    errors.append("unexpected byte")
                continue
            self.buffer.append(byte)
            if len(self.buffer) == 2 and byte != 0xa5:
                errors.append("sync")
                self.buffer.clear()
                self.started = None
                if byte == 0xf1:
                    self.buffer.append(byte)
                    self.started = now
            elif len(self.buffer) == 4 and self.buffer[2:4] != b"\x84\x1a":
                errors.append("not type132/length26")
                self.buffer.clear()
                self.started = None
            elif len(self.buffer) == 32:
                frame = bytes(self.buffer)
                if crc16(frame[2:-2]) != struct.unpack("<H", frame[-2:])[0]:
                    errors.append("CRC")
                else:
                    result.append(PulseStatus(*STATUS_STRUCT.unpack(frame[4:-2])))
                self.buffer.clear()
                self.started = None
        return result, errors


class Sequence:
    """One-shot pure state machine. Duplicates never earn CONTROL/ARM."""

    def __init__(self, started):
        self.started = started
        self.status = None
        self.fresh_at = None
        self.phase = "WAIT"
        self.ready = False
        self.fire_at = None
        self.fire_consumed = False
        self.pulse_commands = 0
        self.sent_pulses = 0
        self.observed_active = False
        self.observed_used_stopped = False
        self.observed_disarmed = False
        self.done = False
        self.error = None

    def abort(self, reason):
        self.error = reason
        self.phase = "FAILED"
        self.done = True

    def stop_command(self):
        return Command(self.status.boot_id, self.status.request_id) if self.status else None

    def input(self, line, now):
        if line != "fire":
            self.abort("cancel/EOF" if line in ("cancel", "") else "Unknown input; cancelling")
            return
        if self.fire_consumed or self.phase != "WAIT" or not self.ready:
            self.abort("fire rejected: not prepared or already consumed")
            return
        self.fire_consumed = True
        if self.fresh_at is None or now - self.fresh_at >= .150 or self.status.flags != UNUSED:
            self.abort("fire rejected: stale or already-used board")
            return
        self.fire_at = now
        self.phase = "NEUTRAL"

    def timeout(self, now):
        if now - self.started >= 180:
            self.abort("180-second preparation deadline")
        elif self.fire_at is not None and now - self.fire_at >= 5:
            self.abort("5-second fire deadline")
        elif self.status is None and now - self.started >= 5:
            self.abort("No verified type132 status within 5 seconds")
        elif self.fresh_at is not None and now - self.fresh_at >= .150:
            self.abort("Status freshness exceeded 150ms")

    def receive(self, statuses, now, errors=()):
        newest = None
        had_new = False
        for value in statuses:
            previous = newest or self.status
            self.status = value  # CRC-verified type132 permits emergency DISARM.
            value.validate()
            if previous:
                if value.boot_id != previous.boot_id:
                    raise ValueError("Board reboot; restart preparation with motor power OFF")
                if value.request_id < previous.request_id:
                    raise ValueError("Request number regressed")
                if ((value.millis - previous.millis) & 0xffffffff) >= 0x80000000:
                    raise ValueError("Board clock regressed")
                if previous.flags == USED and value.flags != USED:
                    raise ValueError("PULSE_USED regressed")
            is_new = previous is None or value.request_id > previous.request_id
            if is_new:
                self.fresh_at = now
                had_new = True
            newest = value
            if self.phase in ("WAIT", "NEUTRAL", "NEUTRAL_ACK", "ARM_ACK") and value.flags == USED:
                raise ValueError("Board pulse already used before this fire")
            if self.phase == "PULSE" and value.flags == USED:
                self.observed_active |= value.actual_drive == 1
        if errors and self.status is not None:
            raise ValueError("Malformed RX: " + "; ".join(errors))
        if newest is None:
            return None
        base = (newest.boot_id, newest.request_id)
        if self.phase == "PULSE":
            if newest.flags == USED:
                self.observed_active |= newest.actual_drive == 1
                if newest.actual_drive == 0:
                    self.observed_used_stopped = True
                    self.phase = "STOP_ACK"
                    return Command(*base)  # Emergency DISARM may reuse a token.
            return None  # Never repeat the pulse or extend it with CONTROL.
        if self.phase == "STOP_ACK":
            if newest.flags == USED and newest.state == 0 and newest.actual_drive == 0:
                self.observed_disarmed = True
                self.phase = "DONE"
                self.done = True
            return None
        if not had_new:
            return None
        if self.phase == "WAIT":
            if newest.state != 0 or newest.requested_drive or newest.actual_drive:
                raise ValueError("Preparation requires DISARMED zero output")
            self.ready = True
            return Command(*base)
        if self.phase == "NEUTRAL":
            if newest.state != 0 or newest.requested_drive or newest.actual_drive:
                raise ValueError("Neutral prerequisites lost")
            self.phase = "NEUTRAL_ACK"
            return Command(*base, CONTROL)
        if self.phase == "NEUTRAL_ACK":
            if newest.state != 0 or newest.reason != 11:
                raise ValueError("Neutral acknowledgement absent")
            self.phase = "ARM_ACK"
            return Command(*base, ARM)
        if self.phase == "ARM_ACK":
            if newest.state != 1 or newest.reason != 9 or newest.requested_drive or newest.actual_drive:
                raise ValueError("ARM acknowledgement absent")
            self.pulse_commands += 1
            if self.pulse_commands != 1:
                raise ValueError("Second pulse is forbidden")
            self.phase = "PULSE"
            return Command(*base, CONTROL, 1, 0, 1)
        return None


class Archive:
    def __init__(self, folder):
        self.path = folder
        folder.mkdir(parents=True, exist_ok=False)
        self.events = (folder / "events.jsonl").open("x", encoding="utf-8", buffering=1)
        self.rx = (folder / "rx.bin").open("xb")
        self.tx = (folder / "tx.bin").open("xb")
        self.hashes = {}
        sources = [Path(__file__), ROOT / "tools/manual_control.py", ROOT / "tools/manual_protocol.py"]
        sources += list((ROOT / "firmware/rear_pulse").glob("*.ino"))
        sources += list((ROOT / "firmware/rear_pulse").glob("*.h"))
        sources += list((ROOT / "firmware/rear_pulse").glob("*.cpp"))
        for source in sources:
            raw = source.read_bytes()
            name = source.name if source.parent.name == "tools" else "firmware_" + source.name
            (folder / name).write_bytes(raw)
            self.hashes[str(source.relative_to(ROOT))] = hashlib.sha256(raw).hexdigest()

    def event(self, name, **data):
        self.events.write(json.dumps({"host_utc": datetime.now(timezone.utc).isoformat(),
            "host_monotonic_s": time.monotonic(), "event": name, **data}, ensure_ascii=False) + "\n")

    def raw(self, direction, value):
        stream = self.rx if direction == "rx" else self.tx
        stream.write(value)
        stream.flush()
        self.event(direction, count=len(value), hex=value.hex())

    def close(self):
        self.events.close()
        self.rx.close()
        self.tx.close()


def send(transport, archive, command, sequence, deadline=None):
    if command is None:
        return
    if transport.out_waiting:
        raise OSError("Pending TX; no additional writes")
    raw = encode_command(command)
    archive.event("tx_attempt", command=asdict(command))
    if command.action != DISARM and deadline is not None and time.monotonic() >= deadline:
        sequence.abort("100ms loop deadline before write")
        command = sequence.stop_command()
        raw = encode_command(command)
        archive.event("tx_replaced_by_disarm", command=asdict(command))
    count = transport.write(raw)
    if count:
        archive.raw("tx", raw[:count])
    if count != len(raw):
        raise OSError("Partial TX; no retry or suffix write")
    if command.action == CONTROL and command.drive == 1:
        sequence.sent_pulses += 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--motor-power-off", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)
    if not (args.prepare and args.motor_power_off):
        parser.error("Both --prepare and --motor-power-off are required before opening the port")
    folder = args.output_dir or ROOT / "data" / (datetime.now().astimezone().strftime("rear_pulse_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
    archive = Archive(folder)
    transport = None
    sequence = Sequence(time.monotonic())
    stream = Parser()
    pending_input = bytearray()
    last_tick = None
    ready_printed = False
    tx_broken = False
    try:
        transport = MegaTransport()
        archive.event("opened_with_motor_power_off_assertion")
        print("OUTPUT=" + str(folder), flush=True)
        while not sequence.done:
            now = time.monotonic()
            deadline = min(now + .100, last_tick + .100 if last_tick is not None else now + .100)
            if last_tick is not None and now - last_tick >= .100:
                raise ValueError("Main loop gap exceeded 100ms")
            last_tick = now
            sequence.timeout(now)
            if sequence.done:
                break
            size = transport.in_waiting
            if size > 4096:
                raise ValueError("RX flood")
            raw = transport.read(size) if size else b""
            if raw:
                archive.raw("rx", raw)
            statuses, errors = stream.feed(raw, now)
            for value in statuses:
                archive.event("status", **asdict(value))
            command = sequence.receive(statuses, now, errors)
            try:
                send(transport, archive, command, sequence, deadline)
            except OSError:
                tx_broken = True
                raise
            if sequence.done:
                break
            if sequence.ready and not ready_printed and not sequence.done:
                print("READY: type132 / output0 / pulse unused. Port stays open. Awaiting explicit fire or cancel.", flush=True)
                archive.event("ready")
                ready_printed = True
            ready, _, _ = select.select([sys.stdin.fileno()], [], [], .005)
            if ready:
                data = os.read(sys.stdin.fileno(), 256)
                if not data:
                    sequence.input("", time.monotonic())
                else:
                    pending_input.extend(data)
                    if len(pending_input) > 256:
                        raise ValueError("stdin command too long")
                    while b"\n" in pending_input:
                        line, _, remainder = pending_input.partition(b"\n")
                        pending_input[:] = remainder
                        text = line.decode("ascii", "strict").strip()
                        archive.event("stdin", value=text)
                        sequence.input(text, time.monotonic())
            if sequence.done:
                break
    except KeyboardInterrupt:
        sequence.abort("Interrupted; cancelling")
    except (OSError, ValueError, RuntimeError) as exc:
        sequence.abort(str(exc))
    finally:
        if transport is not None:
            try:
                if not tx_broken:
                    send(transport, archive, sequence.stop_command(), sequence)
            except (OSError, ValueError) as exc:
                sequence.abort("Final stop not fully sent: " + str(exc))
            finally:
                transport.close()
        report = {"phase": sequence.phase, "error": sequence.error,
            "completed": sequence.phase == "DONE", "pulse_command_count": sequence.sent_pulses,
            "pulse_command_attempts": sequence.pulse_commands,
            "observed_active": sequence.observed_active, "observed_used_stopped": sequence.observed_used_stopped,
            "observed_disarmed": sequence.observed_disarmed, "sources_sha256": archive.hashes,
            "intended_output": {"motor": "rear only", "pwm": 30, "pwm_scale": 255, "maximum_ms": 100,
                                "direction_pin": "LOW per original source; physical direction unmeasured"},
            "limitations": "Firmware telemetry is not electrical measurement or physical wheel-motion confirmation; source hashes do not read back board flash",
            "ended_utc": datetime.now(timezone.utc).isoformat()}
        (folder / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        archive.close()
        print(json.dumps(report, ensure_ascii=False), flush=True)
    return 0 if sequence.phase == "DONE" else 1


if __name__ == "__main__":
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    raise SystemExit(main())
