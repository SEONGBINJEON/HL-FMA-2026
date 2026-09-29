#!/usr/bin/env python3
"""Q1 offline route preparation for the four qualifier start modes.

Device-free: reads HYPER track CSVs listed in
docs/analysis/qualifier_start_modes_2026-09-17/route_candidates.json, verifies
their SHA256, checks quality/seams, builds the four mode routes
(bend_only, s_only, start_left, start_right) as CANDIDATES, tests start-point
selection with a synthetic pose, and renders a 2D preview.

Nothing here approves a route for driving: every mode carries
runtime_approved=False and unresolved entry/exit stays None. Candidate
segments derived from upstream labels or evidence CSVs are marked as such.
Source CSVs are never modified. No averaging or merging of routes.

CLI (all read-only except outputs under docs/analysis/...):
  python3 tools/qualifier_route.py report   [--out DIR]
  python3 tools/qualifier_route.py preview  [--out DIR]
  python3 tools/qualifier_route.py select MODE X Y YAW_RAD --synthetic-params
"""
import argparse, csv, hashlib, json, math, sys
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
CANDIDATES = ROOT / "docs/analysis/qualifier_start_modes_2026-09-17/route_candidates.json"
MODES = ("bend_only", "s_only", "start_left", "start_right")
REQUIRED_FRAME = "map"


class RouteError(ValueError):
    pass


# ----------------------------------------------------------------------------- loading
@dataclass(frozen=True)
class Point:
    idx: int
    x: float
    y: float
    yaw: float
    source: str  # track name
    frame_id: str = REQUIRED_FRAME


@dataclass
class Track:
    name: str
    path: str
    sha256: str
    columns: List[str]
    points: List[Point]
    coordinate_frame_note: str = ""

    def __len__(self):
        return len(self.points)


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def load_candidates(path: Path = CANDIDATES) -> dict:
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    if data.get("schema_version") != 1 or "sources" not in data or "profiles" not in data:
        raise RouteError("route_candidates.json: unexpected schema")
    return data


def load_track_csv(name: str, path: Path, expected_sha256: Optional[str] = None,
                   frame_note: str = "") -> Track:
    """Load a HYPER track CSV (5-column or 23-column). Rejects hash mismatch,
    NaN/inf, non-sequential idx, unknown frame_id, missing columns."""
    if not path.is_file():
        raise RouteError(f"{name}: file missing {path}")
    digest = sha256_of(path)
    if expected_sha256 is not None and digest != expected_sha256:
        raise RouteError(f"{name}: sha256 mismatch (recorded {expected_sha256[:12]}…, file {digest[:12]}…)")
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        cols = reader.fieldnames or []
        for c in ("idx", "x", "y", "yaw", "frame_id"):
            if c not in cols:
                raise RouteError(f"{name}: column {c} missing")
        points = []
        for n, row in enumerate(reader):
            try:
                idx = int(row["idx"]); x = float(row["x"]); y = float(row["y"]); yaw = float(row["yaw"])
            except (TypeError, ValueError) as exc:
                raise RouteError(f"{name}: row {n} unparsable: {exc}")
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise RouteError(f"{name}: row {n} has NaN/inf")
            if idx != n:
                raise RouteError(f"{name}: idx {idx} at row {n} not sequential")
            frame = (row.get("frame_id") or "").strip()
            if frame != REQUIRED_FRAME:
                raise RouteError(f"{name}: row {n} frame_id {frame!r} != {REQUIRED_FRAME!r}")
            points.append(Point(idx, x, y, yaw, name, frame))
    if len(points) < 2:
        raise RouteError(f"{name}: fewer than 2 points")
    return Track(name, str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
                 digest, list(cols), points, frame_note)


def load_all_tracks(cands: dict) -> dict:
    tracks = {}
    for name, src in cands["sources"].items():
        tracks[name] = load_track_csv(name, ROOT / src["path"], src.get("sha256"),
                                      src.get("coordinate_frame", ""))
    return tracks


# ----------------------------------------------------------------------------- geometry
def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def dist(p: Point, q: Point) -> float:
    return math.hypot(q.x - p.x, q.y - p.y)


def heading(p: Point, q: Point) -> float:
    return math.atan2(q.y - p.y, q.x - p.x)


def quality_report(track: Track, gap_warn_m: float = 1.5, yaw_warn_deg: float = 45.0,
                   dup_eps_m: float = 0.01) -> dict:
    """Per-track integrity: step distances, duplicates, yaw vs path heading."""
    pts = track.points
    steps = [dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    dups = [i for i, s in enumerate(steps) if s < dup_eps_m]
    gaps = [{"from_idx": i, "to_idx": i + 1, "m": round(s, 3)} for i, s in enumerate(steps) if s > gap_warn_m]
    yaw_mismatch = []
    for i in range(len(pts) - 1):
        d = math.degrees(abs(wrap(pts[i].yaw - heading(pts[i], pts[i + 1]))))
        if d > yaw_warn_deg:
            yaw_mismatch.append({"idx": i, "yaw_vs_heading_deg": round(d, 1)})
    # last point: compare with heading of the last segment
    last = len(pts) - 1
    d_last = math.degrees(abs(wrap(pts[last].yaw - heading(pts[last - 1], pts[last]))))
    if d_last > yaw_warn_deg:
        yaw_mismatch.append({"idx": last, "yaw_vs_heading_deg": round(d_last, 1), "note": "last point vs last segment"})
    return {
        "track": track.name, "path": track.path, "sha256": track.sha256, "rows": len(pts),
        "columns": len(track.columns), "frame_id": REQUIRED_FRAME,
        "coordinate_frame_note": track.coordinate_frame_note,
        "length_m": round(sum(steps), 2),
        "step_m": {"min": round(min(steps), 3), "max": round(max(steps), 3), "mean": round(sum(steps) / len(steps), 3)},
        "duplicates_lt_1cm": dups, "gaps_gt_warn": gaps, "gap_warn_m": gap_warn_m,
        "yaw_mismatch_gt_deg": yaw_mismatch, "yaw_warn_deg": yaw_warn_deg,
        "self_crossings": self_crossings(pts),
        "first": [round(pts[0].x, 3), round(pts[0].y, 3), round(pts[0].yaw, 3)],
        "last": [round(pts[-1].x, 3), round(pts[-1].y, 3), round(pts[-1].yaw, 3)],
    }


def _segments_intersect(p1, p2, p3, p4) -> bool:
    def orient(a, b, c):
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    d1, d2, d3, d4 = orient(p3, p4, p1), orient(p3, p4, p2), orient(p1, p2, p3), orient(p1, p2, p4)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def self_crossings(points: List[Point]) -> List[dict]:
    """Pairs of non-adjacent segments that intersect (route crosses itself)."""
    out = []
    n = len(points) - 1
    for i in range(n):
        for j in range(i + 2, n):
            if _segments_intersect(points[i], points[i + 1], points[j], points[j + 1]):
                out.append({"seg_a": i, "seg_b": j, "near": [round((points[i].x + points[j].x) / 2, 2),
                                                             round((points[i].y + points[j].y) / 2, 2)]})
    return out


def seam_report(a: Track, a_end_idx: int, b: Track, b_start_idx: int = 0) -> dict:
    """Geometry at a seam. No interpolation, no fix: numbers only."""
    pa = a.points[a_end_idx]; pa_prev = a.points[a_end_idx - 1]
    pb = b.points[b_start_idx]; pb_next = b.points[b_start_idx + 1]
    h_a = heading(pa_prev, pa); h_b = heading(pb, pb_next); h_seam = heading(pa, pb)
    return {
        "from": {"track": a.name, "idx": a_end_idx, "x": round(pa.x, 3), "y": round(pa.y, 3), "yaw": round(pa.yaw, 3)},
        "to": {"track": b.name, "idx": b_start_idx, "x": round(pb.x, 3), "y": round(pb.y, 3), "yaw": round(pb.yaw, 3)},
        "gap_m": round(dist(pa, pb), 3),
        "heading_last_segment_deg": round(math.degrees(h_a), 1),
        "heading_first_segment_deg": round(math.degrees(h_b), 1),
        "heading_across_seam_deg": round(math.degrees(h_seam), 1),
        "turn_across_seam_deg": round(math.degrees(wrap(h_b - h_a)), 1),
        "seam_vs_outgoing_deg": round(math.degrees(wrap(h_seam - h_a)), 1),
        "yaw_from_vs_to_deg": round(math.degrees(wrap(pb.yaw - pa.yaw)), 1),
        "note": "sampled endpoint geometry; not positioning error, not vehicle clearance; never bridged by interpolation",
    }


def nearest_index(track: Track, x: float, y: float) -> Tuple[int, float]:
    best = min(range(len(track.points)), key=lambda i: math.hypot(track.points[i].x - x, track.points[i].y - y))
    return best, math.hypot(track.points[best].x - x, track.points[best].y - y)


# ----------------------------------------------------------------------------- modes
@dataclass
class Segment:
    track: str
    start_idx: int
    end_idx: int  # inclusive
    basis: str
    verified: bool = False


@dataclass
class ModeRoute:
    mode: str
    segments: List[Segment]
    points: List[Point]
    runtime_approved: bool = False
    entry_exit_verified: bool = False
    complete: bool = False           # all indices resolved AND finish known
    unresolved: List[str] = field(default_factory=list)
    seams: List[dict] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)

    def summary(self) -> dict:
        d = asdict(self); d["points"] = len(self.points)
        d["first"] = [round(self.points[0].x, 3), round(self.points[0].y, 3)] if self.points else None
        d["last"] = [round(self.points[-1].x, 3), round(self.points[-1].y, 3)] if self.points else None
        return d


def _slice(track: Track, s: int, e: int) -> List[Point]:
    if not (0 <= s <= e < len(track)):
        raise RouteError(f"{track.name}: slice {s}..{e} outside 0..{len(track) - 1}")
    return track.points[s:e + 1]


def evidence_candidate_indices(evidence: Track, on: Track) -> dict:
    """Nearest indices on `on` to the evidence CSV's first/last point. Candidate only."""
    s_idx, s_d = nearest_index(on, evidence.points[0].x, evidence.points[0].y)
    e_idx, e_d = nearest_index(on, evidence.points[-1].x, evidence.points[-1].y)
    return {"evidence": evidence.name, "on": on.name, "start_idx": s_idx, "start_dist_m": round(s_d, 3),
            "end_idx": e_idx, "end_dist_m": round(e_d, 3), "ordered": s_idx < e_idx}


def build_mode(mode: str, cands: dict, tracks: dict) -> ModeRoute:
    if mode not in MODES:
        raise RouteError(f"unknown mode {mode!r}; choose one of {MODES}")
    prof = cands["profiles"][mode]
    r = ModeRoute(mode=mode, segments=[], points=[])
    if mode in ("bend_only", "s_only"):
        src = tracks[prof["source"]]
        s, e = prof.get("start_index"), prof.get("end_index")
        basis = "route_candidates.json explicit indices"
        if s is None or e is None:
            if mode == "s_only":
                hints = prof.get("upstream_label_hints") or {}
                s, e = hints.get("s_curve_start"), hints.get("s_curve_end")
                basis = "CANDIDATE from upstream mission labels s_curve_start/end (not qualifier placement)"
            else:
                ev = tracks.get(prof.get("evidence_candidate", ""))
                if ev is not None:
                    cand = evidence_candidate_indices(ev, src)
                    r.notes.append(json.dumps(cand))
                    if cand["ordered"] and cand["start_dist_m"] < 5 and cand["end_dist_m"] < 5:
                        s, e = cand["start_idx"], cand["end_idx"]
                        basis = f"CANDIDATE from nearest points to evidence CSV {ev.name} (geometry only)"
                    else:
                        r.unresolved.append(f"bend evidence {ev.name} does not map onto {src.name} "
                                            f"(start {cand['start_dist_m']}m/idx{cand['start_idx']}, end {cand['end_dist_m']}m/idx{cand['end_idx']})")
            r.unresolved.append("entry/exit indices not confirmed by video/map/official placement")
        if s is not None and e is not None:
            r.segments.append(Segment(src.name, s, e, basis, False))
            r.points = _slice(src, s, e)
        r.entry_exit_verified = bool(prof.get("entry_exit_verified"))
        r.complete = bool(prof.get("start_index") is not None and prof.get("end_index") is not None
                          and r.entry_exit_verified)
        return r
    # full starts
    seq = prof["candidate_sequence"]
    start = tracks[seq[0]]; common = tracks[seq[1]]
    end_hint = prof.get("upstream_start_segment_end_index_hint")
    if end_hint is None:
        r.unresolved.append("start segment end index unknown"); end_hint = len(start) - 1
    finish = prof.get("qualifier_finish_index")
    if finish is None:
        r.unresolved.append("qualifier finish index on common_1 unknown (null): route ends at candidate S end label 183 for preview only")
        s_prof = cands["profiles"]["s_only"]
        finish = (s_prof.get("upstream_label_hints") or {}).get("s_curve_end", len(common) - 1)
        finish_basis = "PREVIEW ONLY: S end label hint; official finish unknown"
    else:
        finish_basis = "route_candidates.json qualifier_finish_index"
    r.segments.append(Segment(start.name, 0, end_hint, "upstream mission cut hint", False))
    r.segments.append(Segment(common.name, 0, finish, finish_basis, False))
    r.points = _slice(start, 0, end_hint) + _slice(common, 0, finish)
    r.seams.append(seam_report(start, end_hint, common, 0))
    if not prof.get("event_policy_confirmed"):
        r.unresolved.append("official full-start policy (signals/obstacles/finish/between-course) unconfirmed")
    if not prof.get("seams_verified"):
        r.unresolved.append("seam not verified for vehicle clearance/curvature")
    r.complete = False
    return r


# ----------------------------------------------------------------------------- start selection
@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    position_valid: bool = True
    heading_valid: bool = True
    age_s: float = 0.0


@dataclass(frozen=True)
class SelectionParams:
    max_distance_m: float
    max_heading_deg: float
    entry_window_m: float        # start may only be selected within this path distance from segment start
    ambiguity_ratio: float       # a non-adjacent point closer than best*ratio => ambiguous
    pose_max_age_s: float
    source: str                  # "synthetic" for tests; operational values are unset (None) until measured


OPERATIONAL_PARAMS: Optional[SelectionParams] = None  # unset on purpose: derive from geometry + tests


def _finite(*values) -> bool:
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


def pose_problem(pose: Optional["Pose"]) -> str:
    """Empty string when the pose is usable; otherwise the rejection reason. (Q1.1)"""
    if pose is None or not pose.position_valid:
        return "position invalid"
    if not _finite(pose.x, pose.y):
        return "position non-finite"
    if not pose.heading_valid:
        return "heading invalid"
    if not _finite(pose.yaw):
        return "heading non-finite"
    if not _finite(pose.age_s) or pose.age_s < 0:
        return "pose age non-finite or negative"
    return ""


def params_problem(params: Optional["SelectionParams"]) -> str:
    """Empty string when params are usable. Rejects None, non-finite and out-of-range values. (Q1.1)"""
    if params is None:
        return "selection parameters unset (operational values not measured)"
    fields = (params.max_distance_m, params.max_heading_deg, params.entry_window_m,
              params.ambiguity_ratio, params.pose_max_age_s)
    if not _finite(*fields):
        return "selection parameters non-finite"
    if not (params.max_distance_m > 0 and 0 < params.max_heading_deg <= 180 and params.entry_window_m >= 0
            and params.ambiguity_ratio >= 1 and params.pose_max_age_s > 0):
        return "selection parameters out of range"
    if not isinstance(params.source, str) or not params.source:
        return "selection parameters without source label"
    return ""

SYNTHETIC_PARAMS = SelectionParams(max_distance_m=1.0, max_heading_deg=30.0, entry_window_m=6.0,
                                   ambiguity_ratio=1.5, pose_max_age_s=1.0, source="synthetic")


@dataclass(frozen=True)
class Selection:
    accepted: bool
    reason: str
    index: Optional[int] = None
    distance_m: Optional[float] = None
    heading_error_deg: Optional[float] = None


def cumulative_s(points: List[Point]) -> List[float]:
    s = [0.0]
    for i in range(1, len(points)):
        s.append(s[-1] + dist(points[i - 1], points[i]))
    return s


def path_heading_at(points: List[Point], i: int) -> float:
    if i + 1 < len(points):
        return heading(points[i], points[i + 1])
    return heading(points[i - 1], points[i])


def select_start(route: Optional[ModeRoute], pose: Optional[Pose], params: Optional[SelectionParams]) -> Selection:
    """Pick the start index inside the mode's entry window, or reject with a reason."""
    if route is None or not route.segments:
        return Selection(False, "mode not selected")
    bad = params_problem(params)
    if bad:
        return Selection(False, bad)
    if not route.points:
        return Selection(False, "mode has no resolved points")
    bad = pose_problem(pose)
    if bad:
        return Selection(False, bad)
    if pose.age_s > params.pose_max_age_s:
        return Selection(False, f"pose stale ({pose.age_s:.2f}s > {params.pose_max_age_s}s)")
    pts = route.points
    d = [math.hypot(p.x - pose.x, p.y - pose.y) for p in pts]
    best = min(range(len(pts)), key=d.__getitem__)
    if d[best] > params.max_distance_m:
        return Selection(False, f"nearest route point {d[best]:.2f}m away > {params.max_distance_m}m", best, d[best])
    s_along = cumulative_s(pts)
    if s_along[best] > params.entry_window_m:
        return Selection(False, f"nearest point idx {best} is {s_along[best]:.1f}m into the segment (> entry window "
                                f"{params.entry_window_m}m): no jump into the middle", best, d[best])
    # ambiguity: another point, not adjacent along the path, almost as close (crossing/parallel lane)
    for i, di in enumerate(d):
        if abs(i - best) > 3 and di <= d[best] * params.ambiguity_ratio and di <= params.max_distance_m * params.ambiguity_ratio:
            return Selection(False, f"ambiguous: idx {i} ({di:.2f}m) nearly as close as idx {best} ({d[best]:.2f}m)", best, d[best])
    herr = math.degrees(abs(wrap(pose.yaw - path_heading_at(pts, best))))
    if herr > params.max_heading_deg:
        why = "reverse direction" if herr > 135 else "heading mismatch"
        return Selection(False, f"{why}: {herr:.0f}deg > {params.max_heading_deg}deg", best, d[best], herr)
    return Selection(True, "start accepted inside entry window", best, d[best], herr)


# ----------------------------------------------------------------------------- run state (pure)
class RunState:
    """Explicit mode → stopped/prepared → valid pose → run → segment end (stop, no wrap).

    Q1.1: while RUNNING every pose is checked against the route (distance, heading,
    forward continuity). Any violation moves to STOPPED; only reset → select_mode → arm
    recovers. A pose alone cannot detect every relocation, so operators must also reset
    after moving the vehicle.
    """
    IDLE, MODE_SELECTED, READY, RUNNING, END, STOPPED = "IDLE", "MODE_SELECTED", "READY", "RUNNING", "END", "STOPPED"
    BACKTRACK_TOLERANCE = 1  # indices; progress may never move further back than this

    def __init__(self, params: Optional[SelectionParams]):
        self.params = params
        self.reset()

    def reset(self):
        self.state = self.IDLE; self.route = None; self.progress = None; self.last_reason = "reset"

    def select_mode(self, route: ModeRoute, vehicle_stopped: bool):
        if not vehicle_stopped:
            self.last_reason = "mode change requires stopped vehicle"; return False
        bad = params_problem(self.params)
        if bad:
            self.last_reason = bad; return False
        self.reset(); self.route = route; self.state = self.MODE_SELECTED; self.last_reason = "mode selected"
        return True

    def _stop(self, reason: str) -> dict:
        self.state = self.STOPPED; self.last_reason = reason
        return {"drive": False, "target_idx": self.progress, "reason": reason}

    def arm(self, pose: Pose):
        if self.state != self.MODE_SELECTED:
            self.last_reason = f"arm rejected in state {self.state}"; return False
        sel = select_start(self.route, pose, self.params)
        self.last_reason = sel.reason
        if not sel.accepted:
            return False
        self.progress = sel.index; self.state = self.READY
        return True

    def start(self):
        if self.state != self.READY:
            self.last_reason = f"start rejected in state {self.state}"; return False
        self.state = self.RUNNING; self.last_reason = "running"; return True

    def advance(self, pose: Pose, window: int = 5) -> dict:
        """Forward-only progress with route checks. Output stop at end; STOPPED on violations."""
        if self.state != self.RUNNING:
            return {"drive": False, "target_idx": self.progress, "reason": f"not running ({self.state})"}
        bad = pose_problem(pose)
        if bad:
            return {"drive": False, "target_idx": self.progress, "reason": f"{bad}: hold"}
        if pose.age_s > self.params.pose_max_age_s:
            return {"drive": False, "target_idx": self.progress, "reason": "pose stale: hold"}
        pts = self.route.points
        d_all = [math.hypot(p.x - pose.x, p.y - pose.y) for p in pts]
        hi = min(len(pts) - 1, self.progress + window)
        forward = min(range(self.progress, hi + 1), key=d_all.__getitem__)
        nearest = min(range(len(pts)), key=d_all.__getitem__)
        # (1) continuity: the vehicle is clearly closer to a point behind the tolerated backtrack
        if nearest < self.progress - self.BACKTRACK_TOLERANCE and d_all[nearest] < d_all[forward]:
            return self._stop(f"progress discontinuity: nearest idx {nearest} behind progress {self.progress} "
                              f"(relocation?): stop, reset→mode→arm required")
        # (2) distance: the forward candidate must be within the tracking envelope
        if d_all[forward] > self.params.max_distance_m:
            return self._stop(f"off route: {d_all[forward]:.2f}m from idx {forward} > {self.params.max_distance_m}m: stop")
        # (3) heading: must follow the path direction at the candidate
        herr = math.degrees(abs(wrap(pose.yaw - path_heading_at(pts, forward))))
        if herr > self.params.max_heading_deg:
            why = "reverse direction" if herr > 135 else "heading mismatch"
            return self._stop(f"{why} during run: {herr:.0f}deg > {self.params.max_heading_deg}deg: stop")
        self.progress = forward
        last = len(pts) - 1
        if self.progress >= last:
            # C3.1: the nearest index becoming the last one is only the END APPROACH. Arrival (END) needs the
            # pose at or beyond the last point along the final path direction; the follower may also call
            # finish() earlier inside its explicit end tolerance. Distance/heading/continuity checks above
            # still apply on the final approach; there is still no wrap.
            h = path_heading_at(pts, last)
            along = math.cos(h) * (pts[last].x - pose.x) + math.sin(h) * (pts[last].y - pose.y)
            if along > 0:
                return {"drive": True, "target_idx": last, "reason": f"final approach: last point {along:.2f}m ahead"}
            self.state = self.END; self.last_reason = "segment end: stop"
            return {"drive": False, "target_idx": last, "reason": "segment end: stop, no wrap"}
        return {"drive": True, "target_idx": self.progress + 1, "reason": "following"}

    def finish(self, reason: str) -> dict:
        """C3.1: arrival confirmed by the follower's explicit end criterion (tolerance/pass). Only from
        RUNNING; END is terminal like advance()'s own END (no wrap, no restart without reset)."""
        if self.state != self.RUNNING:
            return {"drive": False, "target_idx": self.progress, "reason": f"finish ignored in state {self.state}"}
        self.state = self.END; self.last_reason = reason
        return {"drive": False, "target_idx": self.progress, "reason": reason}


# ----------------------------------------------------------------------------- outputs
def full_report(cands: dict, tracks: dict) -> dict:
    report = {"generated_from": str(CANDIDATES.relative_to(ROOT)), "upstream_commit": cands.get("upstream_commit"),
              "runtime_approved": False, "tracks": {}, "seams": {}, "modes": {}, "observations": []}
    for name, t in tracks.items():
        report["tracks"][name] = quality_report(t)
    sl, sr, c1 = tracks["start_left"], tracks["start_right"], tracks["common_1"]
    report["seams"]["start_left_wp120_to_common_1"] = seam_report(sl, 120, c1)
    report["seams"]["start_left_wp121_to_common_1"] = seam_report(sl, 121, c1)
    report["seams"]["start_right_wp121_to_common_1"] = seam_report(sr, 121, c1)
    p120, p121 = sl.points[120], sl.points[121]
    report["observations"].append({
        "topic": "start_left wp121 yaw",
        "mission_yaml_claim": "wp121 yaw 2.564, ~173deg off neighbours; mission cuts at wp120",
        "csv_at_pinned_commit": {"wp120_yaw": round(p120.yaw, 4), "wp121_yaw": round(p121.yaw, 4),
                                  "heading_120_to_121": round(heading(p120, p121), 4)},
        "conclusion": "CSV wp121 yaw matches its heading at this commit; the yaml comment is stale or refers to another revision. "
                      "The wp120 cut hint is preserved unchanged; both seams are reported.",
    })
    report["observations"].append({
        "topic": "18.8m gap", "value_m": round(dist(tracks["s_curve"].points[-1], tracks["s_curve_right_angle"].points[0]), 3),
        "meaning": "distance between separate evidence CSVs s_curve end and s_curve_right_angle start; NOT a gap in start_*→common_1",
    })
    for ev in ("s_curve", "s_curve_right_angle"):
        for on in ("common_1", "start_left", "start_right"):
            report["observations"].append({"topic": f"evidence {ev} on {on}", **evidence_candidate_indices(tracks[ev], tracks[on])})
    for m in MODES:
        report["modes"][m] = build_mode(m, cands, tracks).summary()
    return report


def render_preview(cands: dict, tracks: dict, out_png: Path) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 10.5))
    c1 = tracks["common_1"]
    ax.plot([p.x for p in c1.points], [p.y for p in c1.points], color="#555", lw=1.2, label="common_1.csv (recorded path)")
    for name, col in (("start_left", "#1f77b4"), ("start_right", "#d62728")):
        t = tracks[name]
        ax.plot([p.x for p in t.points], [p.y for p in t.points], color=col, lw=1.2, label=f"{name}.csv (recorded path)")
        ax.plot(t.points[0].x, t.points[0].y, "o", color=col, ms=7)
        ax.annotate(f"{name} start", (t.points[0].x, t.points[0].y), textcoords="offset points", xytext=(6, 6), fontsize=8, color=col)
    for name, col in (("s_curve", "#2ca02c"), ("s_curve_right_angle", "#ff7f0e")):
        t = tracks[name]
        ax.plot([p.x for p in t.points], [p.y for p in t.points], "--", color=col, lw=1.0, alpha=0.8,
                label=f"{name}.csv (separate evidence recording, NOT boundary)")
    s_mode = build_mode("s_only", cands, tracks)
    if s_mode.points:
        ax.plot([p.x for p in s_mode.points], [p.y for p in s_mode.points], color="#2ca02c", lw=4, alpha=0.35,
                label=f"s_only CANDIDATE common_1[{s_mode.segments[0].start_idx}..{s_mode.segments[0].end_idx}] (label hint, unverified)")
    b_mode = build_mode("bend_only", cands, tracks)
    if b_mode.points:
        ax.plot([p.x for p in b_mode.points], [p.y for p in b_mode.points], color="#ff7f0e", lw=4, alpha=0.35,
                label=f"bend_only CANDIDATE {b_mode.segments[0].track}[{b_mode.segments[0].start_idx}..{b_mode.segments[0].end_idx}] (geometry hint, unverified)")
    else:
        ax.text(0.01, 0.97, "bend_only: NO candidate segment resolved on common_1 alone —\nevidence starts ~1.9m from start_right[111], ends near common_1[50] (see report)",
                transform=ax.transAxes, fontsize=8, color="#ff7f0e", va="top")
    for seam_name, (t, i), dy in (("L wp120", ("start_left", 120), -14), ("R wp121", ("start_right", 121), 10)):
        p = tracks[t].points[i]
        ax.plot(p.x, p.y, "x", color="k", ms=8)
        ax.annotate(f"seam {seam_name}→common_1[0]", (p.x, p.y), textcoords="offset points", xytext=(-90, dy), fontsize=8)
    for c in self_crossings(c1.points):
        ax.plot(c["near"][0], c["near"][1], "*", color="m", ms=11)
        ax.annotate(f"common_1 self-crossing seg{c['seg_a']}/seg{c['seg_b']}", c["near"], textcoords="offset points", xytext=(8, 4), fontsize=7, color="m")
    ax.plot(c1.points[0].x, c1.points[0].y, "s", color="k", ms=6)
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]  (HYPER 'map' frame, origin/axes unverified vs our vehicle frame)")
    ax.set_ylabel("y [m]")
    ax.set_title("Q1 candidate routes — HYPER track CSVs @ " + cands.get("upstream_commit", "")[:12] +
                 "\nLines are recorded vehicle paths, NOT surveyed road boundaries. All modes runtime_approved=false.", fontsize=10)
    ax.legend(fontsize=7, loc="upper left", bbox_to_anchor=(0.0, -0.08), ncol=2)
    srcs = "\n".join(f"{k}: {v['path'].split('/')[-1]} sha256 {v['sha256'][:12]}…" for k, v in cands["sources"].items())
    ax.text(0.99, 0.01, srcs, transform=ax.transAxes, fontsize=6.5, ha="right", va="bottom", family="monospace",
            bbox=dict(boxstyle="round", fc="white", alpha=0.8))
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(); fig.savefig(out_png, dpi=130); plt.close(fig)
    return out_png


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["report", "preview", "select"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--candidates", default=str(CANDIDATES))
    ap.add_argument("--out", default=str(CANDIDATES.parent))
    ap.add_argument("--synthetic-params", action="store_true", help="use SYNTHETIC selection thresholds (tests only)")
    a = ap.parse_args(argv)
    cands = load_candidates(Path(a.candidates))
    tracks = load_all_tracks(cands)
    out = Path(a.out)
    if a.command == "report":
        rep = full_report(cands, tracks)
        out.mkdir(parents=True, exist_ok=True)
        (out / "quality_report.json").write_text(json.dumps(rep, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"tracks": {k: {"rows": v["rows"], "len_m": v["length_m"], "max_step": v["step_m"]["max"],
                                          "gaps": len(v["gaps_gt_warn"]), "yaw_mismatch": len(v["yaw_mismatch_gt_deg"]),
                                          "dups": len(v["duplicates_lt_1cm"]), "self_crossings": v["self_crossings"]} for k, v in rep["tracks"].items()},
                          "seams": {k: {"gap_m": v["gap_m"], "turn_deg": v["turn_across_seam_deg"]} for k, v in rep["seams"].items()},
                          "modes": {k: {"points": v["points"], "complete": v["complete"], "unresolved": v["unresolved"]}
                                    for k, v in rep["modes"].items()},
                          "observations": rep["observations"][:2]}, ensure_ascii=False, indent=1))
        print(f"written {out / 'quality_report.json'}")
        return 0
    if a.command == "preview":
        p = render_preview(cands, tracks, out / "preview_map.png")
        print(f"written {p}"); return 0
    if len(a.args) != 4:
        ap.error("select MODE X Y YAW_RAD")
    mode, x, y, yaw = a.args[0], float(a.args[1]), float(a.args[2]), float(a.args[3])
    params = SYNTHETIC_PARAMS if a.synthetic_params else OPERATIONAL_PARAMS
    sel = select_start(build_mode(mode, cands, tracks), Pose(x, y, yaw), params)
    print(json.dumps(asdict(sel), ensure_ascii=False))
    return 0 if sel.accepted else 1


if __name__ == "__main__":
    sys.exit(main())
