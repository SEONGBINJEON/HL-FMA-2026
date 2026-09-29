"""Convert a PathFollower bicycle-equivalent target into a left-front-wheel target.

SYNTHETIC MODEL ONLY. The conversion below is an ideal Ackermann linkage
(rigid geometry, no compliance, no slip, no toe/camber, no lash). It is NOT an
A1 measurement and must never be confused with a calibrated wheel angle.
Every physical constant must be supplied explicitly by the caller; nothing is
inferred or defaulted. No filesystem, serial, or network access at runtime.
"""
from __future__ import annotations

import dataclasses
import math
import os
import sys
from typing import Optional

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

import path_steering  # noqa: E402

SYNTHETIC_MODE = "synthetic_ideal_ackermann"
UNCONFIGURED_MODE = "unconfigured"
SOURCE_KIND = "bicycle_equivalent"
TARGET_KIND = "left_front_wheel"
_INT16_MIN, _INT16_MAX = -32768, 32767


@dataclasses.dataclass(frozen=True)
class AngleContract:
    """Explicit conversion contract. No operational approval, no identity fallback."""
    mode: str
    source_kind: str
    target_kind: str
    wheelbase_m: Optional[float]
    track_m: Optional[float]


def _check_int16(name: str, value) -> int:
    # bool is an int subclass; reject it explicitly. Never echo the value (it may be huge).
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be int (not bool), got {type(value).__name__}")
    if not (_INT16_MIN <= value <= _INT16_MAX):
        raise ValueError(f"{name} outside int16 range")
    return value


def _check_length(name: str, value) -> float:
    """Finite positive length. Every numeric failure (type, overflow, NaN/inf, sign) is a ValueError;
    the offending value is never repeated in the message (an int such as 10**400 has a 400-digit repr)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be int/float (not bool), got {type(value).__name__}")
    try:
        f = float(value)
    except (OverflowError, ValueError):
        raise ValueError(f"{name} cannot be represented as a finite float") from None
    if not math.isfinite(f) or f <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return f


def _check_contract(contract, synthetic) -> tuple[float, float]:
    if synthetic is not True:  # literal True only; truthy values are not accepted
        raise ValueError("synthetic model must be explicitly enabled with synthetic=True")
    if contract is None or not isinstance(contract, AngleContract):
        raise ValueError("contract is None or not an AngleContract")
    if contract.mode != SYNTHETIC_MODE:
        raise ValueError(f"contract mode {contract.mode!r} is not {SYNTHETIC_MODE!r}")
    if contract.source_kind != SOURCE_KIND or contract.target_kind != TARGET_KIND:
        raise ValueError("contract source/target kinds do not match bicycle->left_front_wheel")
    wheelbase = _check_length("wheelbase_m", contract.wheelbase_m)
    track = _check_length("track_m", contract.track_m)
    return wheelbase, track


def convert_angle(angle_cdeg, contract, *, synthetic, min_cdeg, max_cdeg) -> int:
    """Return left-front-wheel angle in centidegrees under the ideal Ackermann model."""
    wheelbase, track = _check_contract(contract, synthetic)
    angle_cdeg = _check_int16("angle_cdeg", angle_cdeg)
    min_cdeg = _check_int16("min_cdeg", min_cdeg)
    max_cdeg = _check_int16("max_cdeg", max_cdeg)
    if not (min_cdeg <= 0 <= max_cdeg) or min_cdeg >= max_cdeg:
        raise ValueError(f"invalid output interval [{min_cdeg}, {max_cdeg}]")
    try:
        delta = math.radians(angle_cdeg / 100.0)  # bicycle-equivalent steer angle
        if abs(delta) >= math.pi / 2:
            raise ValueError("bicycle-equivalent angle magnitude must be below 90 deg")
        if delta == 0.0:
            left = 0.0
        else:
            # Rear-axle turning radius must exceed half the track, else the
            # inner wheel geometry is undefined for a rigid Ackermann linkage.
            radius = wheelbase / abs(math.tan(delta))
            if not radius > track / 2.0:
                raise ValueError("turn radius does not exceed half track; geometry undefined")
            k = math.tan(delta) / wheelbase  # path curvature
            # Left wheel: left turn (k>0) shortens the denominator -> larger angle;
            # right turn (k<0) lengthens it -> smaller magnitude. Ideal linkage only.
            left = math.atan2(wheelbase * k, 1.0 - track * k / 2.0)
        out = round(math.degrees(left) * 100.0)
    except OverflowError as exc:
        raise ValueError(f"numeric overflow in synthetic conversion: {exc}") from exc
    if not math.isfinite(out):
        raise ValueError("non-finite conversion result")
    out = int(out)
    if out < min_cdeg or out > max_cdeg:  # reject, never clamp
        raise ValueError(f"converted angle {out} outside [{min_cdeg}, {max_cdeg}]")
    return out


def convert_message(msg, contract, *, synthetic, min_cdeg, max_cdeg):
    """Map a TargetMessage to a left-front-wheel TargetMessage; StopMessage passes through."""
    if isinstance(msg, path_steering.StopMessage):
        return msg
    if not isinstance(msg, path_steering.TargetMessage):
        raise TypeError(f"unsupported message type {type(msg).__name__}")
    try:
        wheel_cdeg = convert_angle(
            msg.angle_cdeg, contract, synthetic=synthetic, min_cdeg=min_cdeg, max_cdeg=max_cdeg
        )
    except ValueError as exc:
        # A failed conversion is a hold, never a zero-angle target.
        return path_steering.StopMessage(
            kind="hold",
            reason=f"steering angle conversion failed: {exc}",
            host_time_ms=msg.host_time_ms,
            decision=dict(msg.decision) if isinstance(msg.decision, dict) else None,
            cause="error",
            reached=False,
        )
    decision = dict(msg.decision) if isinstance(msg.decision, dict) else {}
    decision["bicycle_equivalent_cdeg"] = msg.angle_cdeg  # original PathFollower angle
    decision["left_front_wheel_cdeg"] = wheel_cdeg
    decision["angle_model"] = contract.mode
    return dataclasses.replace(msg, angle_cdeg=wheel_cdeg, decision=decision)
