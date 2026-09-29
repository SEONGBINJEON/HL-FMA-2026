#!/usr/bin/env python3
"""C3.1: PathFollower messages -> C2 HostSteerAdapter, including hold/stop.

Device-free glue. The bridge does not decide steering; it enforces one rule:
a follower "hold" or "stop" means there is NO valid target, so the adapter must not keep
executing the previous target. On the first hold/stop while the adapter is prepared
(or preparing) the bridge:
  1. calls adapter.stop(reason)  -> C2 lock (locked_by_fault), target dropped, core stopped;
  2. builds the emergency DISARM for the latest identified board request and sends it
     through the Client when one is given (the board accepts a well-framed DISARM at any
     time); every later board request is answered with DISARM by the locked Session.
Resuming needs an EXPLICIT reprepare() (operator/planner decision) AND a new target
sequence, exactly like every other C2 fault. A valid pose or a new sequence alone never
resumes. The bridge never prepares on its own.

A "stop" is never expressed as angle 0; stop/boot/request/neutral rules of C2 are untouched.
"""
from dataclasses import dataclass, asdict
from typing import List, Optional
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import path_steering as ps  # noqa: E402

PREPARED = ("ready", "active")
PREPARING = ("neutral_pending", "neutral_sent", "arm_sent")


@dataclass(frozen=True)
class BridgeResult:
    now_s: float
    msg_kind: str            # "steer" | "hold" | "stop"
    action: str              # "target_accepted" | "target_refused" | "lock_disarm" | "already_locked" | "not_prepared"
    reason: str
    disarm_command: Optional[dict] = None   # what was built for the latest valid request (None: no identified board)
    disarm_sent: bool = False               # True only when a Client actually queued it on the transport
    adapter_phase: str = ""
    cause: Optional[str] = None
    reached: Optional[bool] = None


class FollowerC2Bridge:
    def __init__(self, adapter, client=None):
        self.adapter = adapter; self.client = client
        self.results: List[BridgeResult] = []
        self.lock_events: List[BridgeResult] = []   # one per lock transition caused by hold/stop
        self.refused_targets = 0
        self.last_stop: Optional[ps.StopMessage] = None

    # ---- explicit operator/planner action; never called from apply()
    def reprepare(self, now_s: float) -> bool:
        """Explicit re-preparation after a hold/stop lock. Returns the adapter's decision; still needs a
        new target sequence afterwards before anything moves (C2 rule)."""
        return bool(self.adapter.request_prepare(now_s))

    def apply(self, msg, now_s: float) -> BridgeResult:
        ad = self.adapter
        if isinstance(msg, ps.TargetMessage):
            if ad.locked_by_fault or ad.phase not in PREPARED:
                self.refused_targets += 1
                r = BridgeResult(now_s, "steer", "not_prepared", f"target seq {msg.sequence} not handed to C2: adapter {ad.phase}"
                                 + (" (locked; explicit reprepare needed)" if ad.locked_by_fault else ""), adapter_phase=ad.phase)
            else:
                ok = ad.set_target(msg.sequence, msg.angle_cdeg, msg.host_time_ms, now_s)
                if ok:
                    r = BridgeResult(now_s, "steer", "target_accepted", f"seq {msg.sequence} angle {msg.angle_cdeg} cdeg", adapter_phase=ad.phase)
                else:
                    self.refused_targets += 1
                    r = BridgeResult(now_s, "steer", "target_refused", f"seq {msg.sequence} refused: {ad.first_fault or ad.message}", adapter_phase=ad.phase)
            self.results.append(r); return r
        if not isinstance(msg, ps.StopMessage):
            raise TypeError(f"unknown follower message {type(msg).__name__}")
        self.last_stop = msg
        label = f"path follower {msg.kind}: {msg.reason}"
        if ad.phase in PREPARED or ad.phase in PREPARING or ad.have_target_since_prepare:
            ad.stop(label)                       # C2 lock: target dropped, core stopped, DISARM on every next request
            cmd = ad.emergency_command()         # latest identified request; None when no board was ever identified
            sent = False
            if cmd is not None and self.client is not None:
                self.client.send(cmd); sent = not self.client.failed
            r = BridgeResult(now_s, msg.kind, "lock_disarm", label, asdict(cmd) if cmd is not None else None, sent, ad.phase,
                             msg.cause, msg.reached)
            self.lock_events.append(r)
        else:
            r = BridgeResult(now_s, msg.kind, "already_locked", label, adapter_phase=ad.phase, cause=msg.cause, reached=msg.reached)
        self.results.append(r); return r
