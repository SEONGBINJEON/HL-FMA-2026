#!/usr/bin/env python3
"""C3: route + rear-axle pose -> target steering angle (pure pursuit), device-free.

Units: metres, radians, FLU (left positive). Reference point: rear-axle centre.
Output for C2 (steer_target_host): angle in centidegrees (int16), monotonic target
sequence, or an explicit STOP decision. angle 0 is never used to mean "stop".

Pure pursuit (rear-axle bicycle model):
  * take the path point at lookahead distance Ld ahead of the vehicle (interpolated),
  * alpha = bearing of that point in the vehicle frame,
  * curvature k = 2 sin(alpha) / Ld  (circle through vehicle and goal, tangent to heading),
  * equivalent steering angle delta = atan(L * k), L = wheelbase.
delta is the single "bicycle" steering angle, NOT the measured left front wheel angle
(A1). An ideal-Ackermann split is offered separately as a model, not as a measurement.

Progress comes from qualifier_route.RunState (forward window, STOPPED on violations),
never from a fresh nearest-point search over the whole route.

End handling (C3.1): "the lookahead circle no longer meets the path" is only the END
APPROACH, not arrival. While the last point is still ahead it becomes the temporary goal
and the curvature is computed with the real distance to it. Arrival is a separate,
explicit criterion (end_reach_tolerance_m around the last point, or passing the last point
by at most end_pass_max_overshoot_m within the lateral envelope; reference point named
in end_reference). A stop that is not arrival carries reached=False. Nothing steers to a
goal behind the vehicle or turns back after the end.

Operational config (wheelbase source, lookahead, limits, end criteria) is None until
measured/decided; tests use SYNTHETIC values only. The official finish line is not
defined here (unconfirmed).
"""
import math
from dataclasses import dataclass, asdict
from typing import List, Optional, Tuple
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import qualifier_route as qr  # noqa: E402

REQUIRED_FRAME = "map"
REQUIRED_REFERENCE = "rear_axle_center"
INT16_MIN, INT16_MAX = -32768, 32767


def _finite(*v) -> bool:
    return all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v)


@dataclass(frozen=True)
class PurePursuitConfig:
    wheelbase_m: float
    wheelbase_source: str          # e.g. "user report 2026-09-17 (750 mm), not surveyed"
    lookahead_m: float
    max_lateral_error_m: float     # beyond this: STOP (off path)
    max_heading_error_rad: float   # beyond this: STOP (wrong direction)
    max_equiv_steer_rad: float     # usable equivalent steering; beyond: STOP (no clamp until measured)
    pose_max_age_s: float
    end_reach_tolerance_m: float   # arrival: reference point within this distance of the route's last point
    end_pass_max_overshoot_m: float  # arrival also when the last point is behind by at most this (inside lateral envelope)
    end_reference: str             # which point is compared with the last route point, e.g. "rear_axle_center"
    source: str                    # "synthetic" for tests; operational values unset


OPERATIONAL_CONFIG: Optional[PurePursuitConfig] = None  # unset on purpose
SYNTHETIC_CONFIG = PurePursuitConfig(wheelbase_m=0.75, wheelbase_source="user report 2026-09-17 (750 mm), not surveyed",
                                     lookahead_m=1.5, max_lateral_error_m=1.0, max_heading_error_rad=math.radians(60),
                                     max_equiv_steer_rad=math.radians(30), pose_max_age_s=0.5,
                                     end_reach_tolerance_m=0.5, end_pass_max_overshoot_m=1.0,
                                     end_reference="rear_axle_center vs route last point (synthetic; official finish line unconfirmed)",
                                     source="synthetic")


def config_problem(cfg: Optional[PurePursuitConfig]) -> str:
    if cfg is None:
        return "pure-pursuit config unset (operational values not measured)"
    vals = (cfg.wheelbase_m, cfg.lookahead_m, cfg.max_lateral_error_m, cfg.max_heading_error_rad,
            cfg.max_equiv_steer_rad, cfg.pose_max_age_s, cfg.end_reach_tolerance_m, cfg.end_pass_max_overshoot_m)
    if not _finite(*vals):
        return "config non-finite"
    if not (cfg.wheelbase_m > 0 and cfg.lookahead_m > 0 and cfg.max_lateral_error_m > 0 and 0 < cfg.max_heading_error_rad <= math.pi
            and 0 < cfg.max_equiv_steer_rad < math.pi / 2 and cfg.pose_max_age_s > 0
            and cfg.end_reach_tolerance_m > 0 and cfg.end_pass_max_overshoot_m >= 0):
        return "config out of range"
    if not cfg.wheelbase_source or not cfg.source or not cfg.end_reference:
        return "config without source labels"
    return ""


@dataclass(frozen=True)
class PoseInput:
    """Rear-axle pose in the route frame. Anything else is rejected (no antenna->axle conversion here)."""
    x: float
    y: float
    yaw: float
    frame_id: str = REQUIRED_FRAME
    reference_point: str = REQUIRED_REFERENCE
    position_valid: bool = True
    heading_valid: bool = True
    age_s: float = 0.0

    def as_route_pose(self) -> qr.Pose:
        return qr.Pose(self.x, self.y, self.yaw, self.position_valid, self.heading_valid, self.age_s)


def pose_input_problem(p: Optional[PoseInput]) -> str:
    if p is None:
        return "pose missing"
    if p.frame_id != REQUIRED_FRAME:
        return f"pose frame {p.frame_id!r} != route frame {REQUIRED_FRAME!r}"
    if p.reference_point != REQUIRED_REFERENCE:
        return f"pose reference {p.reference_point!r} is not the rear-axle centre (antenna/IMU positions are not the vehicle reference)"
    return qr.pose_problem(p.as_route_pose())


STOP_CAUSES = ("path_end", "limit", "error")


@dataclass(frozen=True)
class SteeringDecision:
    kind: str                       # "steer" | "stop" | "reject"
    reason: str
    angle_rad: Optional[float] = None
    angle_cdeg: Optional[int] = None
    curvature_1pm: Optional[float] = None
    goal_xy: Optional[Tuple[float, float]] = None
    goal_idx: Optional[int] = None
    lateral_error_m: Optional[float] = None
    heading_error_rad: Optional[float] = None
    remaining_path_m: Optional[float] = None
    # C3.1: why we stop and whether the end was actually reached are separate facts.
    stop_cause: Optional[str] = None    # "path_end" | "limit" | "error" (only for kind == "stop")
    end_reached: Optional[bool] = None  # True only inside the configured end criterion; never inferred from reason text
    end_distance_m: Optional[float] = None   # straight distance reference point -> route last point
    end_along_m: Optional[float] = None      # + = last point still ahead along the final path direction
    end_approach: bool = False          # kind == "steer" using the last point as temporary goal (inside lookahead)


def rad_to_cdeg(angle_rad: float) -> int:
    v = int(round(math.degrees(angle_rad) * 100))
    if not (INT16_MIN <= v <= INT16_MAX):
        raise ValueError("angle outside int16 centidegrees")
    return v


def lookahead_point(points: List[qr.Point], start_idx: int, x: float, y: float, ld: float):
    """First crossing of the circle radius ld around (x,y) along the path from start_idx forward.
    Returns (gx, gy, idx_of_segment_end) or None when the path ends before reaching ld."""
    for j in range(start_idx + 1, len(points)):
        a, b = points[j - 1], points[j]
        da = math.hypot(a.x - x, a.y - y); db = math.hypot(b.x - x, b.y - y)
        if db >= ld:
            if da >= ld and j - 1 > start_idx:
                return a.x, a.y, j - 1
            # interpolate along segment a->b to distance ld
            dx, dy = b.x - a.x, b.y - a.y
            fx, fy = a.x - x, a.y - y
            A = dx * dx + dy * dy; B = 2 * (fx * dx + fy * dy); C = fx * fx + fy * fy - ld * ld
            disc = B * B - 4 * A * C
            if A <= 0 or disc < 0:
                return b.x, b.y, j
            t = (-B + math.sqrt(disc)) / (2 * A)
            t = min(1.0, max(0.0, t))
            return a.x + t * dx, a.y + t * dy, j
    return None


def remaining_path_length(points: List[qr.Point], idx: int, x: float, y: float) -> float:
    if idx >= len(points) - 1:
        return math.hypot(points[-1].x - x, points[-1].y - y) if idx == len(points) - 1 else 0.0
    total = math.hypot(points[idx + 1].x - x, points[idx + 1].y - y)
    for k in range(idx + 1, len(points) - 1):
        total += qr.dist(points[k], points[k + 1])
    return total


@dataclass(frozen=True)
class EndStatus:
    """Geometry of the reference point relative to the route's last point (C3.1)."""
    distance_m: float           # straight distance to the last point
    along_m: float              # projection on the final segment direction; + = last point still ahead
    lateral_m: float            # signed offset from the final segment line (+ left)
    status: str                 # "reached" | "passed" | "overshoot" | "ahead"
    reached: bool               # True only for "reached"/"passed"


def evaluate_end(points: List[qr.Point], x: float, y: float, cfg: PurePursuitConfig) -> EndStatus:
    """Explicit arrival criterion. Reference point = the pose given (rear-axle centre, see cfg.end_reference).
    reached: within end_reach_tolerance_m of the last point.
    passed:  last point behind by <= end_pass_max_overshoot_m and inside the lateral envelope (drove through).
    overshoot: behind by more than that, or beside the line: stop, NOT reached (never turn back).
    ahead:   still to be approached."""
    last = points[-1]
    h = qr.path_heading_at(points, len(points) - 1)
    dx, dy = last.x - x, last.y - y
    d = math.hypot(dx, dy)
    along = math.cos(h) * dx + math.sin(h) * dy
    lateral = -(-math.sin(h) * dx + math.cos(h) * dy)   # vehicle offset left of the final line
    if d <= cfg.end_reach_tolerance_m:
        return EndStatus(d, along, lateral, "reached", True)
    if along < 0:
        if -along <= cfg.end_pass_max_overshoot_m and abs(lateral) <= cfg.max_lateral_error_m:
            return EndStatus(d, along, lateral, "passed", True)
        return EndStatus(d, along, lateral, "overshoot", False)
    return EndStatus(d, along, lateral, "ahead", False)


def compute_target(points: List[qr.Point], progress_idx: int, pose: PoseInput,
                   cfg: Optional[PurePursuitConfig]) -> SteeringDecision:
    """Pure decision. Never returns kind='steer' with an unverified input."""
    bad = config_problem(cfg)
    if bad:
        return SteeringDecision("reject", bad)
    bad = pose_input_problem(pose)
    if bad:
        return SteeringDecision("reject", bad)
    if pose.age_s > cfg.pose_max_age_s:
        return SteeringDecision("reject", f"pose stale ({pose.age_s:.2f}s > {cfg.pose_max_age_s}s)")
    if not points or len(points) < 2:
        return SteeringDecision("reject", "route has fewer than 2 points")
    if not isinstance(progress_idx, int) or isinstance(progress_idx, bool) or not (0 <= progress_idx < len(points)):
        return SteeringDecision("reject", "progress index invalid")
    i = progress_idx
    path_h = qr.path_heading_at(points, i)
    # signed lateral error: +left of the path direction at progress
    px, py = pose.x - points[i].x, pose.y - points[i].y
    lateral = -math.sin(path_h) * px + math.cos(path_h) * py
    heading_err = qr.wrap(pose.yaw - path_h)
    remaining = remaining_path_length(points, i, pose.x, pose.y)
    es = evaluate_end(points, pose.x, pose.y, cfg)
    common = dict(lateral_error_m=lateral, heading_error_rad=heading_err, remaining_path_m=remaining,
                  end_distance_m=es.distance_m, end_along_m=es.along_m)
    if abs(lateral) > cfg.max_lateral_error_m:
        return SteeringDecision("stop", f"lateral error {lateral:+.2f}m beyond {cfg.max_lateral_error_m}m",
                                stop_cause="limit", end_reached=False, **common)
    if abs(heading_err) > cfg.max_heading_error_rad:
        return SteeringDecision("stop", f"heading error {math.degrees(heading_err):+.0f}deg beyond limit",
                                stop_cause="limit", end_reached=False, **common)
    # Spatial proximity to the endpoint is not route completion: on a loop or
    # adjacent start/finish lanes the vehicle can begin beside the endpoint.
    # RunState's monotonic progress must first reach the final segment.
    on_final_segment = i >= len(points) - 2
    # ---- explicit arrival / overshoot (C3.1), independent of the lookahead geometry
    if on_final_segment and es.status == "reached":
        return SteeringDecision("stop", f"path end reached: {es.distance_m:.2f}m to last point <= {cfg.end_reach_tolerance_m}m",
                                stop_cause="path_end", end_reached=True, **common)
    if on_final_segment and es.status == "passed":
        return SteeringDecision("stop", f"path end passed: last point {-es.along_m:.2f}m behind (<= {cfg.end_pass_max_overshoot_m}m)",
                                stop_cause="path_end", end_reached=True, **common)
    if on_final_segment and es.status == "overshoot":
        return SteeringDecision("stop", f"last point {-es.along_m:.2f}m behind / {es.lateral_m:+.2f}m beside: beyond end criterion, "
                                        f"not reached, no turning back", stop_cause="limit", end_reached=False, **common)
    goal = lookahead_point(points, i, pose.x, pose.y, cfg.lookahead_m)
    approach = False
    if goal is None:
        # END APPROACH: the path ends inside the lookahead circle but the last point is still ahead.
        # Use it as the temporary goal with its real distance (no premature "stop"/"reached").
        goal = (points[-1].x, points[-1].y, len(points) - 1); approach = True
    gx, gy, gidx = goal
    ld = math.hypot(gx - pose.x, gy - pose.y)
    if ld <= 1e-6:
        return SteeringDecision("reject", "degenerate lookahead (goal at vehicle)")
    alpha = qr.wrap(math.atan2(gy - pose.y, gx - pose.x) - pose.yaw)
    if approach and abs(alpha) >= math.pi / 2:
        # never chase a goal beside/behind the vehicle
        return SteeringDecision("stop", f"last point not ahead of the vehicle ({math.degrees(alpha):+.0f}deg): stop, not reached",
                                stop_cause="limit", end_reached=False, goal_xy=(gx, gy), goal_idx=gidx, **common)
    curvature = 2.0 * math.sin(alpha) / ld
    delta = math.atan(cfg.wheelbase_m * curvature)
    if abs(delta) > cfg.max_equiv_steer_rad:
        return SteeringDecision("stop", f"required equivalent steer {math.degrees(delta):+.1f}deg beyond usable "
                                        f"{math.degrees(cfg.max_equiv_steer_rad):.1f}deg (no clamp until measured)",
                                angle_rad=delta, curvature_1pm=curvature, goal_xy=(gx, gy), goal_idx=gidx,
                                stop_cause="limit", end_reached=False, end_approach=approach, **common)
    try:
        cdeg = rad_to_cdeg(delta)
    except ValueError as exc:
        return SteeringDecision("reject", str(exc))
    return SteeringDecision("steer", "pure pursuit (end approach: last point as goal)" if approach else "pure pursuit",
                            delta, cdeg, curvature, (gx, gy), gidx, end_approach=approach, **common)


def ideal_ackermann_wheel_angles(delta_equiv_rad: float, wheelbase_m: float, track_m: float) -> dict:
    """MODEL ONLY: split the bicycle angle into inner/outer wheel angles under ideal Ackermann
    geometry. The vehicle's linkage has NOT been measured to follow this; do not write these
    into the A1 calibration table."""
    if not _finite(delta_equiv_rad, wheelbase_m, track_m) or wheelbase_m <= 0 or track_m <= 0:
        raise ValueError("non-finite or non-positive inputs")
    if abs(delta_equiv_rad) < 1e-9:
        return {"left_rad": 0.0, "right_rad": 0.0, "model": "ideal Ackermann (assumption, unverified)"}
    R = wheelbase_m / math.tan(abs(delta_equiv_rad))
    inner = math.atan(wheelbase_m / (R - track_m / 2)) if R - track_m / 2 > 0 else math.pi / 2
    outer = math.atan(wheelbase_m / (R + track_m / 2))
    left, right = (inner, outer) if delta_equiv_rad > 0 else (outer, inner)
    sign = 1 if delta_equiv_rad > 0 else -1
    return {"left_rad": sign * left, "right_rad": sign * right, "turn_radius_rear_axle_m": R,
            "model": "ideal Ackermann (assumption, unverified); not the A1 measured wheel angle"}


# ---------------------------------------------------------------- interface to C2
@dataclass(frozen=True)
class TargetMessage:
    kind: str            # "steer"
    sequence: int
    angle_cdeg: int
    host_time_ms: int
    decision: dict


@dataclass(frozen=True)
class StopMessage:
    """kind "stop": the run is over (END/STOPPED); kind "hold": no valid target right now (RunState may stay
    RUNNING). Both mean the consumer must NOT keep executing the previous steering target (C3.1).
    cause: "path_end" | "limit" | "error"; reached: True only when the explicit end criterion was met."""
    kind: str            # "stop" | "hold"
    reason: str
    host_time_ms: int
    decision: Optional[dict] = None
    cause: str = "error"
    reached: bool = False


class PathFollower:
    """RunState (Q1.1) progress -> pure pursuit -> C2 messages. Sequence is monotonic per emitted
    target; a stop/hold never reuses angle 0 as a stop signal.

    Q1's own hold (invalid/stale pose while RUNNING) recovers internally on the next valid pose; whether
    C2 may move again is decided by the consumer (path_steering_bridge: explicit re-prepare + new target)."""

    def __init__(self, run_state: qr.RunState, cfg: Optional[PurePursuitConfig], clock_ms):
        self.state = run_state; self.cfg = cfg; self.clock_ms = clock_ms
        self.sequence = 0; self.last = None

    def _end_status(self, pose: PoseInput):
        if config_problem(self.cfg) or pose_input_problem(pose) or not self.state.route or len(self.state.route.points) < 2:
            return None
        return evaluate_end(self.state.route.points, pose.x, pose.y, self.cfg)

    def step(self, pose: PoseInput, now_s: float):
        host_ms = self.clock_ms(now_s)
        bad = pose_input_problem(pose)
        if bad:
            self.last = StopMessage("hold", f"pose rejected: {bad}", host_ms, cause="error"); return self.last
        adv = self.state.advance(pose.as_route_pose())
        if not adv["drive"]:
            if self.state.state == self.state.END:
                es = self._end_status(pose)
                reached = bool(es and es.reached)
                self.last = StopMessage("stop", adv["reason"] + ("" if reached else " (end criterion not met: not reached)"),
                                        host_ms, asdict(es) if es else None, cause="path_end" if reached else "limit", reached=reached)
            elif self.state.state == self.state.STOPPED:
                self.last = StopMessage("stop", adv["reason"], host_ms, cause="limit")
            else:
                self.last = StopMessage("hold", adv["reason"], host_ms, cause="error")
            return self.last
        dec = compute_target(self.state.route.points, self.state.progress, pose, self.cfg)
        if dec.kind == "steer":
            self.sequence += 1
            self.last = TargetMessage("steer", self.sequence, dec.angle_cdeg, host_ms, asdict(dec))
        elif dec.kind == "stop":
            if dec.stop_cause == "path_end" and dec.end_reached:
                self.state.finish(f"path steering: {dec.reason}")
            else:
                self.state._stop(f"path steering: {dec.reason}")
            self.last = StopMessage("stop", dec.reason, host_ms, asdict(dec), cause=dec.stop_cause or "limit", reached=bool(dec.end_reached))
        else:
            self.last = StopMessage("hold", dec.reason, host_ms, asdict(dec), cause="error")
        return self.last
