#!/usr/bin/env python3
"""Codec for simultaneous persistent-session type144/PWM110; no hardware access."""

from dataclasses import dataclass
import struct

SYNC = b"\xf1\xa5"
COMMAND = 1
STATUS = 144
DISARM, ARM, CONTROL = 0, 1, 2
DISARMED, ARMED = 0, 1
BENCH, DRIVE_ENABLED, STEER_ENABLED = 1, 2, 4
INTERLOCK_CONFIGURED, INTERLOCK_OK, BOOT_ID_VALID = 8, 16, 32
STEP_USED, STEP_UNUSED_FLAGS, STEP_USED_FLAGS = 64, 38, 102
COMMAND_STRUCT = struct.Struct("<IIBbbB")
STATUS_STRUCT = struct.Struct("<IIIiHBBbbbbH")
LENGTHS = {COMMAND: COMMAND_STRUCT.size, STATUS: STATUS_STRUCT.size}
MAX_FRAME = 32
PARTIAL_TIMEOUT_S = .030


class ProtocolError(ValueError):
    pass


def crc16(data):
    """CRC16-CCITT-FALSE, including TYPE and LENGTH, excluding sync."""
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def _uint32(value, name):
    if type(value) is not int or not 0 <= value <= 0xffffffff:
        raise ProtocolError(name)


def _axes(drive, steer):
    if drive not in (-1, 0, 1) or steer not in (-1, 0, 1):
        raise ProtocolError("axis outside -1/0/+1")


@dataclass(frozen=True)
class Command:
    boot_id: int
    request_id: int
    action: int = DISARM
    drive: int = 0
    steer: int = 0
    deadman: int = 0

    def validate(self):
        _uint32(self.boot_id, "boot_id")
        _uint32(self.request_id, "request_id")
        _axes(self.drive, self.steer)
        if self.action not in (DISARM, ARM, CONTROL) or self.deadman not in (0, 1):
            raise ProtocolError("action/deadman")
        if self.action in (DISARM, ARM) and (self.drive or self.steer or self.deadman):
            raise ProtocolError("ARM/DISARM must be neutral")
        if not self.deadman and (self.drive or self.steer):
            raise ProtocolError("motion without deadman")


@dataclass(frozen=True)
class Status:
    boot_id: int
    request_id: int
    millis: int = 0
    encoder_count: int = 0
    steering_adc: int = 0
    state: int = DISARMED
    reason: int = 0
    requested_drive: int = 0
    requested_steer: int = 0
    actual_drive: int = 0
    actual_steer: int = 0
    flags: int = STEP_UNUSED_FLAGS

    def validate(self):
        for name in ("boot_id", "request_id", "millis"):
            _uint32(getattr(self, name), name)
        if not -0x80000000 <= self.encoder_count <= 0x7fffffff:
            raise ProtocolError("encoder_count")
        if not 0 <= self.steering_adc <= 1023:
            raise ProtocolError("steering_adc")
        if self.state not in (DISARMED, ARMED) or self.reason not in range(12):
            raise ProtocolError("state/reason")
        if self.flags not in (STEP_UNUSED_FLAGS, STEP_USED_FLAGS):
            raise ProtocolError("unknown flags")
        _axes(self.requested_drive, self.requested_steer)
        _axes(self.actual_drive, self.actual_steer)
        if self.state == DISARMED and any((self.requested_drive, self.requested_steer,
                                          self.actual_drive, self.actual_steer)):
            raise ProtocolError("DISARMED output/request must be zero")
        if self.actual_drive and (self.actual_drive != self.requested_drive or self.flags != STEP_USED_FLAGS):
            raise ProtocolError("Actual drive/request mismatch")
        if self.actual_steer and (self.actual_steer != self.requested_steer or self.flags != STEP_USED_FLAGS):
            raise ProtocolError("Actual steer/request mismatch")


def _frame(kind, payload):
    body = bytes((kind, len(payload))) + payload
    return SYNC + body + struct.pack("<H", crc16(body))


def encode_command(value):
    value.validate()
    return _frame(COMMAND, COMMAND_STRUCT.pack(
        value.boot_id, value.request_id, value.action,
        value.drive, value.steer, value.deadman))


def encode_status(value):
    value.validate()
    return _frame(STATUS, STATUS_STRUCT.pack(
        value.boot_id, value.request_id, value.millis, value.encoder_count,
        value.steering_adc, value.state, value.reason, value.requested_drive,
        value.requested_steer, value.actual_drive, value.actual_steer, value.flags))


def decode_frame(frame):
    if len(frame) < 6 or frame[:2] != SYNC:
        raise ProtocolError("sync/length")
    kind, size = frame[2:4]
    if kind not in LENGTHS or size != LENGTHS[kind] or len(frame) != size + 6:
        raise ProtocolError("type/length")
    if crc16(frame[2:-2]) != struct.unpack("<H", frame[-2:])[0]:
        raise ProtocolError("CRC")
    if kind == COMMAND:
        value = Command(*COMMAND_STRUCT.unpack(frame[4:-2]))
    else:
        value = Status(*STATUS_STRUCT.unpack(frame[4:-2]))
    value.validate()
    return value


class FrameParser:
    """Incremental parsing, at most 32 retained bytes, bounded caller input.

    Returns (decoded values, errors). Never hides malformed bytes preceding a
    good frame; an already identified session treats any error as a stop.
    """

    def __init__(self):
        self.buffer = bytearray()
        self.started = None

    def feed(self, data, now):
        values, errors = [], []
        if len(data) > 4096:
            self.buffer.clear()
            self.started = None
            return values, ["RX flood"]
        if self.buffer and now - self.started >= PARTIAL_TIMEOUT_S:
            self.buffer.clear()
            self.started = None
            errors.append("partial frame expired")
        for byte in data:
            if not self.buffer:
                if byte == SYNC[0]:
                    self.buffer.append(byte)
                    self.started = now
                else:
                    if not errors or errors[-1] != "unexpected byte":
                        errors.append("unexpected byte")
                continue
            self.buffer.append(byte)
            if len(self.buffer) == 2 and self.buffer[1] != SYNC[1]:
                errors.append("sync")
                self.buffer.clear()
                self.started = None
                if byte == SYNC[0]:
                    self.buffer.append(byte)
                    self.started = now
            elif len(self.buffer) == 4:
                kind, size = self.buffer[2:4]
                if kind not in LENGTHS or size != LENGTHS[kind]:
                    errors.append("type/length")
                    self.buffer.clear()
                    self.started = None
            elif len(self.buffer) >= 6 and len(self.buffer) == self.buffer[3] + 6:
                try:
                    values.append(decode_frame(bytes(self.buffer)))
                except ProtocolError as exc:
                    errors.append(str(exc))
                self.buffer.clear()
                self.started = None
        return values, errors
