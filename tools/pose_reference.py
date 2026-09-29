#!/usr/bin/env python3
"""C4: GNSS antenna pose -> rear-axle PoseInput for path_steering (pure, device-free).

Scope (2026-09-17, qualifier first; no operational config/calibration/existing code touched):
  * heading_to_yaw:        explicit heading convention -> ENU yaw in [-pi, pi) (rad, east = 0,
                           left/CCW +). This is an EARTH (ENU) angle, not yet a map angle.
  * antenna_to_rear_axle:  planar 2D: subtract the yaw-rotated FLU (rear-axle -> antenna) offset
                           from the antenna position. yaw must be expressed in the same frame as
                           x/y (the map frame); the result is refused if it is not finite.
  * rear_axle_pose:        keyword-only entry point that validates the caller's declarations,
                           converts the ENU yaw into the map yaw with the caller-supplied
                           `map_yaw_from_enu_rad`, and returns an existing path_steering.PoseInput
                           (reference 'rear_axle_center').

Map axis convention (revision 1, after review):
  The HYPER map axes are NOT assumed to coincide with ENU. The caller must pass
  `map_yaw_from_enu_rad` (no default): the angle to add to an ENU yaw to obtain a map yaw,
      yaw_map = wrap_pi(yaw_enu + map_yaw_from_enu_rad).
  0.0 means "map axes are ENU", but that value is never assumed here; it must be written down by
  the caller. x/y must ALREADY be map-frame positions; this module does no origin translation,
  projection or rotation of x/y. `frame_alignment_verified=True` is the caller's declaration that
  upstream has checked the x/y map position and this angle conversion against each other.
  The offset rotation from antenna to rear axle is done with the map yaw, so a +/-pi/2 map
  rotation rotates the position offset accordingly.

What this module does NOT do (unsolved, handled elsewhere or not at all yet):
  * GNSS latitude/longitude -> map projection, or determining `map_yaw_from_enu_rad` itself.
  * Yaw from a single antenna while stationary (no course-over-ground), or judging heading
    accuracy. `heading_valid=True` is the upstream receiver/validator's declaration.
  * Roll/pitch compensation. The user-reported antenna offset is (x=0.315, y=0, z=1.170) m in FLU
    from the rear axle (user report 2026-09-17, not surveyed). At z ~ 1.17 m a 5 deg tilt already
    displaces the antenna's ground projection by ~0.10 m, so the planar assumption must be
    verified before use; `planar_assumption_verified=True` is again a caller declaration.
    Those numbers are provenance only; they are NOT defaults anywhere in this module.
  * Any transport: a ValueError here must be turned into stop/hold by the vehicle-side executor.
    This module sends nothing.
  * equivalent steering -> measured left front wheel angle (A1) is a separate open item.

Input contract (all keyword-only, no defaults, nothing is substituted for bad input):
  frame_id == 'map', frame_alignment_verified is True (bool),
  reference_point == 'gnss_antenna' (an input already at 'rear_axle_center' is refused: no double
  conversion), heading_convention in HEADING_CONVENTIONS, position_valid/heading_valid/
  planar_assumption_verified are real bools and True, all numbers int/float that convert to a
  finite float (bool refused; NaN/inf refused; ints too large for a float refused),
  0 <= position_age_s <= max_age_s and 0 <= heading_age_s <= max_age_s with max_age_s > 0,
  offset_x_m/offset_y_m given explicitly by the caller from a measured setup,
  map_yaw_from_enu_rad given explicitly by the caller (rad).
Error contract: every unfit or corrupted numeric input raises ValueError (never OverflowError,
  never a silent inf/NaN result). Missing keyword arguments raise TypeError.
Units: metres, radians (after conversion), FLU (x forward, y left).
"""
import math
import sys
from pathlib import Path
from typing import Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import path_steering as ps  # noqa: E402

REQUIRED_FRAME = ps.REQUIRED_FRAME                  # 'map'
REQUIRED_INPUT_REFERENCE = "gnss_antenna"
OUTPUT_REFERENCE = ps.REQUIRED_REFERENCE            # 'rear_axle_center'

# Explicitly supported heading conventions. No implicit unit/convention guessing.
HEADING_ENU_CCW_RAD = "enu_ccw_rad"    # 0 = east (+x), counter-clockwise positive, radians
HEADING_NORTH_CW_DEG = "north_cw_deg"  # 0 = north (+y), clockwise positive, degrees (compass)
HEADING_CONVENTIONS = (HEADING_ENU_CCW_RAD, HEADING_NORTH_CW_DEG)

_TWO_PI = 2.0 * math.pi


def _require_number(name: str, v) -> float:
    """Return v as a finite float or raise ValueError.

    Accepts int/float only (bool refused even though it subclasses int). The float conversion and
    the finiteness check are done together so that an int too large for a float (OverflowError
    from float()/math.isfinite()) also ends up as the declared ValueError. The message reports the
    type, not the value: a corrupted input may be huge and its repr must not become a second
    failure inside the error path.
    """
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"{name} must be an int/float (not bool/str/None), got {type(v).__name__}")
    try:
        f = float(v)
    except OverflowError:
        raise ValueError(f"{name} ({type(v).__name__}) is too large to represent as a finite float") from None
    if not math.isfinite(f):
        raise ValueError(f"{name} must be finite (NaN/inf refused)")
    return f


def _require_bool(name: str, v) -> bool:
    if not isinstance(v, bool):
        raise ValueError(f"{name} must be a real bool (True/False), got {v!r}")
    return v


def wrap_pi(angle_rad: float) -> float:
    """Normalise to [-pi, pi). pi maps to -pi."""
    r = (angle_rad + math.pi) % _TWO_PI - math.pi
    if r >= math.pi:      # guard against rounding pushing the modulo result to exactly 2*pi
        r -= _TWO_PI
    if r < -math.pi:
        r += _TWO_PI
    return r


def heading_to_yaw(value, convention) -> float:
    """Convert an explicitly declared heading into ENU yaw (rad, east = 0, CCW +) in [-pi, pi).

    convention: 'enu_ccw_rad' (value in radians, already ENU) or 'north_cw_deg' (compass degrees,
    north = 0, clockwise +). Nothing else is accepted; the unit is never inferred from the value.
    The result is an EARTH-frame (ENU) angle. rear_axle_pose converts it to the map frame with
    the caller's map_yaw_from_enu_rad; do not feed it to a map-frame consumer unconverted.
    Corrupted values (bool, NaN, inf, ints too large for a float, non-numbers) raise ValueError.
    """
    if not isinstance(convention, str) or convention not in HEADING_CONVENTIONS:
        raise ValueError(f"unsupported heading convention {convention!r}; supported: {HEADING_CONVENTIONS}")
    v = _require_number("heading value", value)
    if convention == HEADING_ENU_CCW_RAD:
        return wrap_pi(v)
    # north_cw_deg: compass bearing b -> ENU angle (90 - b) degrees
    return wrap_pi(math.radians(90.0 - v))


def antenna_to_rear_axle(x, y, yaw, offset_x_m, offset_y_m) -> Tuple[float, float]:
    """Planar 2D: rear-axle centre from the antenna position, both in the same planar frame.

    (offset_x_m, offset_y_m) is the antenna position in the vehicle FLU frame relative to the
    rear-axle centre (x forward, y left), in metres. It is rotated by yaw into the frame of x/y
    and subtracted from the antenna position. yaw must be the vehicle yaw expressed in that same
    frame (for rear_axle_pose: the MAP yaw, not the ENU yaw). No default offset exists; the
    caller must supply the measured values. Height (z) and roll/pitch are ignored here on
    purpose (see module docstring).
    The computed result is checked for finiteness: finite inputs whose difference overflows a
    float (e.g. 1e308 - (-1e308)) are refused with ValueError instead of returning inf/NaN.
    """
    xa = _require_number("x", x)
    ya = _require_number("y", y)
    th = _require_number("yaw", yaw)
    ox = _require_number("offset_x_m", offset_x_m)
    oy = _require_number("offset_y_m", offset_y_m)
    c, s = math.cos(th), math.sin(th)
    dx = c * ox - s * oy
    dy = s * ox + c * oy
    rx = xa - dx
    ry = ya - dy
    if not (math.isfinite(rx) and math.isfinite(ry)):
        raise ValueError("rear-axle position is not finite (x/y or offset magnitude overflowed a float); input refused")
    return rx, ry


def rear_axle_pose(*, x, y, heading, heading_convention, map_yaw_from_enu_rad, frame_id,
                   frame_alignment_verified, reference_point, position_valid, heading_valid,
                   position_age_s, heading_age_s, max_age_s, planar_assumption_verified,
                   offset_x_m, offset_y_m) -> ps.PoseInput:
    """Validate the caller's declarations, convert antenna pose -> rear-axle PoseInput (map frame).

    x, y:                 antenna position ALREADY in the map frame (metres). Not projected,
                          translated or rotated here.
    heading/convention:   heading in an explicitly declared convention -> ENU yaw (heading_to_yaw).
    map_yaw_from_enu_rad: angle (rad) added to the ENU yaw to obtain the map yaw:
                              yaw_map = wrap_pi(yaw_enu + map_yaw_from_enu_rad).
                          Mandatory, no default; 0.0 is never assumed. The antenna -> rear-axle
                          offset is rotated with yaw_map, so a rotated map rotates the offset too.
    frame_alignment_verified: caller's declaration that upstream has checked that the x/y map
                          position and this ENU->map angle conversion are consistent with each
                          other and with the HYPER map. Not verifiable here.

    Raises ValueError for every unfit input (wrong/unverified frame, non-antenna reference,
    unsupported heading convention, invalid position/heading, stale or negative (future) age,
    non-positive max_age_s, unverified planar assumption, non-bool flags, non-finite or
    unrepresentable numbers, non-finite conversion result).
    Missing keyword arguments raise TypeError (there are no defaults by design).
    Never substitutes zero/normal values for bad input. Sends nothing; the executor must map a
    ValueError to stop/hold.
    """
    # ---- frame
    if not isinstance(frame_id, str) or frame_id != REQUIRED_FRAME:
        raise ValueError(f"frame_id {frame_id!r} != required route frame {REQUIRED_FRAME!r}")
    if _require_bool("frame_alignment_verified", frame_alignment_verified) is not True:
        raise ValueError("GNSS->map frame alignment not verified (map x/y position vs map_yaw_from_enu_rad vs HYPER map axes)")
    # ---- reference point
    if not isinstance(reference_point, str):
        raise ValueError(f"reference_point must be a str, got {reference_point!r}")
    if reference_point == OUTPUT_REFERENCE:
        raise ValueError(f"input reference_point is already {OUTPUT_REFERENCE!r}: refusing a double conversion")
    if reference_point != REQUIRED_INPUT_REFERENCE:
        raise ValueError(f"reference_point {reference_point!r} != {REQUIRED_INPUT_REFERENCE!r}")
    # ---- validity declarations (upstream's statements, not physically proven here)
    if _require_bool("position_valid", position_valid) is not True:
        raise ValueError("position declared invalid by upstream")
    if _require_bool("heading_valid", heading_valid) is not True:
        raise ValueError("heading declared invalid by upstream (offset rotation needs a valid yaw)")
    if _require_bool("planar_assumption_verified", planar_assumption_verified) is not True:
        raise ValueError("planar assumption not verified: high antenna on a tilted vehicle has roll/pitch error")
    # ---- ages
    max_age = _require_number("max_age_s", max_age_s)
    if max_age <= 0:
        raise ValueError(f"max_age_s must be > 0, got {max_age!r}")
    p_age = _require_number("position_age_s", position_age_s)
    h_age = _require_number("heading_age_s", heading_age_s)
    if p_age < 0:
        raise ValueError(f"position_age_s {p_age!r} is negative (timestamp in the future)")
    if h_age < 0:
        raise ValueError(f"heading_age_s {h_age!r} is negative (timestamp in the future)")
    if p_age > max_age:
        raise ValueError(f"position stale: {p_age:.3f}s > max_age_s {max_age:.3f}s")
    if h_age > max_age:
        raise ValueError(f"heading stale: {h_age:.3f}s > max_age_s {max_age:.3f}s")
    # ---- numbers and conversion (ENU yaw -> map yaw -> offset rotation in the map frame)
    yaw_enu = heading_to_yaw(heading, heading_convention)
    map_rot = _require_number("map_yaw_from_enu_rad", map_yaw_from_enu_rad)
    yaw_map = wrap_pi(yaw_enu + map_rot)
    rx, ry = antenna_to_rear_axle(x, y, yaw_map, offset_x_m, offset_y_m)
    return ps.PoseInput(x=rx, y=ry, yaw=yaw_map, frame_id=REQUIRED_FRAME, reference_point=OUTPUT_REFERENCE,
                        position_valid=True, heading_valid=True, age_s=max(p_age, h_age))
