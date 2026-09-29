"""Deterministic mission decisions; no hardware or synthetic live measurements.

The live follower calls this before issuing each drive permission. Unsupported
missions remain explicit barriers. All times here are monotonic seconds.
"""
from dataclasses import dataclass, asdict
from collections import deque, Counter
import math


def finite(x):
    return type(x) in (int, float) and math.isfinite(x)


@dataclass
class Gate:
    name: str
    kind: str
    stop_s: float
    commit_s: float
    minimum_s: float = 0.
    allowed: tuple = ()
    direction: int = 1
    stage_start: int = 0
    stage_end: int = -1
    next_section: str = ''
    approach_m: float = .5


class Stationary:
    """Fresh distinct type150 samples and zero requested drive, not a stop ACK.

Exact encoder equality is conservative. It does not prove zero wheel slip.
Hold timing starts only after settle_s of observed encoder stability.
"""
    def __init__(self, settle_s=.5):
        self.settle_s = settle_s
        self.reset()

    def reset(self):
        self.key = self.encoder = self.stable_since = self.stopped_since = None
        self.last_received = None

    def update(self, board, now):
        if (not board or not finite(board.get('received_at')) or
                not 0 <= now-board['received_at'] < .15 or
                board.get('requested_drive') != 0 or type(board.get('encoder')) is not int):
            self.reset(); return False
        key = (board['boot'], board['request'])
        if self.key is not None and (key[0] != self.key[0] or key[1] < self.key[1]):
            self.reset()
        if key == self.key:
            return self.stopped_since is not None
        self.last_received = board['received_at']
        if self.encoder != board['encoder'] or self.stable_since is None:
            self.stable_since = now
            self.stopped_since = None
        self.key, self.encoder = key, board['encoder']
        if self.stopped_since is None and now-self.stable_since >= self.settle_s:
            self.stopped_since = now
        return self.stopped_since is not None

    def duration(self, now):
        return 0. if self.stopped_since is None else now-self.stopped_since


class SignalVotes:
    def __init__(self, target, epoch, required=3, max_age=.6):
        self.target, self.epoch = target, epoch
        self.required, self.max_age = required, max_age
        self.last_id = self.last_capture = self.last_decision = None
        self.count = 0
        self.sign = None
        self.seen = set()

    def update(self, event, now, allowed):
        if event is None:
            return self.permits(now, allowed)
        cap, dec = event.get('capture_at'), event.get('decision_at')
        if (event.get('target') != self.target or event.get('epoch') != self.epoch or
                not finite(cap) or not finite(dec) or not 0 <= now-cap <= self.max_age or
                not cap <= dec <= now or event.get('valid') is not True or
                not finite(event.get('confidence')) or not .5 <= event['confidence'] <= 1.):
            self.count = 0; self.sign = None; return False
        identity = (event.get('source'), event.get('frame_id'))
        if None in identity:
            self.count = 0; return False
        if identity in self.seen:
            return self.permits(now, allowed)
        if self.last_capture is not None and cap <= self.last_capture:
            self.count = 0; self.sign = None; return False
        self.seen.add(identity)
        if len(self.seen) > 2048:
            # Identity is monotonic per source too; retain bounded memory.
            self.seen = {identity}
        sign = event.get('sign')
        gap = self.last_capture is not None and cap-self.last_capture > self.max_age
        self.count = self.count+1 if not gap and sign == self.sign and sign in allowed else (1 if sign in allowed else 0)
        self.sign, self.last_capture, self.last_decision = sign, cap, dec
        self.last_id = identity
        return self.permits(now, allowed)

    def permits(self, now, allowed):
        return bool(allowed and self.sign in allowed and self.count >= self.required and
                    self.last_capture is not None and 0 <= now-self.last_capture <= self.max_age)


class Mission:
    def __init__(self, gates, run_id, epoch=0, completed=(), skipped=()):
        self.gates = list(gates)
        if any(g.kind not in ('hill', 'signal', 'unsupported', 'finish', 's_curve', 'parking', 'gear', 'transition') for g in self.gates):
            raise ValueError('unknown mission kind')
        if any(not finite(g.stop_s) or not finite(g.commit_s) or g.stop_s > g.commit_s for g in self.gates):
            raise ValueError('invalid mission distance')
        if any(a.commit_s > b.stop_s for a, b in zip(self.gates, self.gates[1:])):
            raise ValueError('unordered/overlapping missions')
        self.run_id, self.epoch = run_id, epoch
        self.completed, self.skipped = list(completed), list(skipped)
        self.index = 0
        while self.index < len(self.gates) and self.gates[self.index].name in self.completed+self.skipped:
            self.index += 1
        self.stationary = Stationary()
        self.votes = None
        self.holding = False
        self.hold_reason = None
        self.fault = None
        self.boot = None
        self.obstacle_latched = False
        self.quality_wait = False
        self.wait_encoder = None
        self.recovery_required = False
        self.recovery_cause = None
        self.previous_encoder = None
        self.previous_position_ok = True
        self.resume_ready = True
        self.signal_pass = None
        self.signal_wait_detail = None
        self.last_signal_crossing = None

    @property
    def gate(self):
        return self.gates[self.index] if self.index < len(self.gates) else None

    def fail(self, reason):
        self.fault = reason
        return self.hold(reason)

    def hold(self, reason):
        if not self.holding:
            self.stationary.reset()
        self.holding, self.hold_reason = True, reason
        return {'drive': False, 'reason': reason, 'mission': None if self.gate is None else self.gate.name}

    def release(self):
        if self.holding and (self.stationary.stopped_since is None or not self.resume_ready):
            return self.hold('resume_wait_observed_stop_and_heading')
        self.holding, self.hold_reason = False, None
        self.stationary.reset()
        return {'drive': True, 'reason': 'mission_permits', 'mission': None if self.gate is None else self.gate.name}

    def advance(self):
        self.completed.append(self.gate.name)
        self.index += 1
        self.votes = None
        self.signal_pass = None
        self.signal_wait_detail = None

    def step(self, *, s, now, board, quality_ok, heading_ok, aligned,
             signal=None, obstacle='unknown', sensor_required=True, position_ok=None, resume_ready=True, maneuver_ok=False, finish_confirmed=None, position_observed_at=None, position_continuity_proven=False):
        if self.fault:
            return self.hold(self.fault)
        if not board or not finite(board.get('received_at')) or not 0 <= now-board['received_at'] < .15:
            return self.fail('board_stale')
        if self.boot is None:
            self.boot = board['boot']
        if board['boot'] != self.boot:
            return self.fail('board_reboot')
        if board.get('fault') or board.get('state') != 'ARMED':
            return self.fail('board_fault_or_disarmed')
        if self.holding:
            self.stationary.update(board, now)
        self.resume_ready = resume_ready
        # RTCM freshness is not position observability. Only motion across a
        # missing-position interval destroys the continuously observed pose.
        position_ok = quality_ok if position_ok is None else position_ok
        moved = self.previous_encoder is not None and board.get('encoder') != self.previous_encoder
        if moved and (not position_ok or not self.previous_position_ok):
            if not self.recovery_required:self.recovery_cause = 'gps_gap_motion'
            self.recovery_required = True
        self.previous_encoder = board.get('encoder')
        self.previous_position_ok = position_ok
        if not quality_ok:
            self.quality_wait = True
        if (self.recovery_required and self.recovery_cause=='gps_gap_motion' and
                position_continuity_proven and quality_ok and heading_ok and aligned and
                self.stationary.stopped_since is not None and resume_ready):
            self.recovery_required=False;self.recovery_cause=None
        if self.quality_wait or self.recovery_required:
            if (not quality_ok or not heading_ok or not aligned or self.recovery_required or
                    self.stationary.stopped_since is None or not resume_ready):
                return self.hold('pose_recovery_required' if self.recovery_required else
                                 ('position_quality_wait' if not position_ok else 'correction_quality_wait'))
            self.quality_wait = False
        if not heading_ok or not aligned:
            return self.fail('heading_or_route_invalid')
        # A stationary gear/section transition issues no motion. Check the NEW
        # directional corridor on the next tick before granting any drive.
        neutral_gate=self.gate
        if (neutral_gate and neutral_gate.kind in ('gear','transition') and finite(s) and
                s>=neutral_gate.stop_s-neutral_gate.approach_m and self.holding and
                self.stationary.stopped_since is not None and resume_ready):
            return {**self.hold('maneuver_transition_ready'),'action':neutral_gate.kind}
        if not sensor_required:self.obstacle_latched=False
        if sensor_required and obstacle == 'blocked':
            if not self.obstacle_latched:
                self.stationary.reset()  # prior signal/hill dwell cannot satisfy a NEW obstacle stop
            self.obstacle_latched = True
        if sensor_required and obstacle not in ('clear', 'blocked'):
            return self.hold('obstacle_observation_unknown')
        if self.obstacle_latched:
            if obstacle != 'clear' or self.stationary.duration(now) < 3.:
                return self.hold('obstacle_minimum_3s')
            self.obstacle_latched = False
        gate = self.gate
        if gate is None:
            return self.hold('route_end_not_official_finish')
        if not finite(s):
            return self.fail('invalid_progress')
        if gate.kind == 'signal' and gate.stop_s-s <= 12.:
            if self.votes is None:
                self.votes = SignalVotes(gate.name, self.epoch)
            self.votes.update(signal, now, gate.allowed)
        if s < gate.stop_s-gate.approach_m:
            return self.release()
        if gate.kind == 'unsupported':
            return self.hold('unsupported:'+gate.name)
        if gate.kind == 's_curve':
            if not maneuver_ok:return self.hold('s_curve_candidate_unavailable')
            if s>=gate.commit_s:self.advance()
            return self.release()
        if gate.kind == 'parking':
            return self.hold('parking_observed_candidate_required')
        if gate.kind in ('gear','transition'):
            if self.stationary.stopped_since is None or not resume_ready:
                return self.hold('maneuver_neutral_stop')
            return {**self.hold('maneuver_transition_ready'),'action':gate.kind}
        if gate.kind == 'finish':
            if finish_confirmed is None:return self.hold('rear_wheel_finish_line_unconfirmed')
            if finish_confirmed:
                self.advance()
                return {**self.hold('official_rear_wheels_finished'),'finished':True}
            return self.release()
        if gate.kind == 'hill':
            if self.stationary.duration(now) < gate.minimum_s:
                return self.hold('hill_observed_stop')
            self.advance()
            return self.release()
        # Permission is acquired only before the line from three real green frames.
        # A bounded token tolerates loss. A red can be ignored only AFTER an
        # independently timestamped, already-crossed position observation.
        permitted=bool(self.votes and self.votes.permits(now,gate.allowed))
        fresh_red=bool(signal and signal.get('target')==gate.name and signal.get('epoch')==self.epoch and
            signal.get('valid') is True and signal.get('sign')=='red' and
            finite(signal.get('capture_at')) and 0<=now-signal['capture_at']<=.6 and
            finite(signal.get('confidence')) and signal['confidence']>=.5)
        token=self.signal_pass
        bounded=bool(token and token['gate']==gate.name and 0<=now-token['at']<=3. and
                     token['start_s']-.1<=s<=gate.commit_s+1.)
        completed_before_red=bool(fresh_red and bounded and s>=gate.commit_s and position_ok and
            finite(position_observed_at) and 0<=now-position_observed_at<=.5 and
            token['at']<=position_observed_at<signal['capture_at'])
        if completed_before_red:
            self.last_signal_crossing={'gate':gate.name,'permission_at':token['at'],
                'position_observed_at':position_observed_at,'red_capture_at':signal['capture_at'],
                's_m':s,'commit_s':gate.commit_s,'basis':'observed_crossing_before_red'}
            self.advance(); return self.release()
        if fresh_red:self.signal_pass=None
        if permitted and s<gate.commit_s and self.signal_pass is None:
            self.signal_pass={'gate':gate.name,'at':now,'start_s':s}
        token=self.signal_pass
        crossing=bool(token and token['gate']==gate.name and 0<=now-token['at']<=3. and
                      token['start_s']-.1<=s<=gate.commit_s+1. and not fresh_red)
        self.signal_wait_detail=('red' if fresh_red else 'no_frame' if not signal else
            'target_or_epoch_mismatch' if signal.get('target')!=gate.name or signal.get('epoch')!=self.epoch else
            'stale_frame' if not finite(signal.get('capture_at')) or not 0<=now-signal['capture_at']<=.6 else
            'invalid_or_low_confidence' if not signal.get('valid') or not finite(signal.get('confidence')) or signal['confidence']<.5 else
            'not_detected' if signal.get('sign') in (None,'none') else
            'insufficient_green_frames' if signal.get('sign') in gate.allowed else 'nonpermitted_signal')
        if s >= gate.commit_s:
            # Current green alone cannot retroactively authorize an unobserved entry.
            if not crossing:
                return self.fail('signal_crossed_without_permission:'+gate.name)
            self.last_signal_crossing={'gate':gate.name,'permission_at':token['at'],
                'position_observed_at':position_observed_at,'s_m':s,'commit_s':gate.commit_s,
                'basis':'prior_green_permission_bounded_crossing'}
            self.advance(); return self.release()
        if permitted or crossing:
            return self.release()
        return self.hold('signal_wait:'+gate.name)

    def snapshot(self):
        return {'run_id': self.run_id, 'epoch': self.epoch, 'index': self.index,
                'completed': self.completed, 'skipped': self.skipped, 'boot': self.boot,
                'fault': self.fault, 'holding': self.holding, 'reason': self.hold_reason,
                'gates': [asdict(g) for g in self.gates]}


class ForkVotes:
    """Fresh distinct two-panel verdicts in a 3s window; no timeout/default lane."""
    def __init__(self, epoch):
        self.guard = SignalVotes('finish_fork', epoch, required=1)
        self.rows = deque()

    def update(self, event, now):
        previous = self.guard.last_id
        valid = self.guard.update(event, now, ('allow_left','allow_right'))
        if not valid:
            self.rows.clear(); return None
        if self.guard.last_id != previous:
            self.rows.append((self.guard.last_capture,self.guard.sign))
        while self.rows and now-self.rows[0][0]>3.:self.rows.popleft()
        if len(self.rows)<3 or self.rows[-1][0]-self.rows[0][0]<.2:return None
        counts=Counter(v for _,v in self.rows)
        winner,n=counts.most_common(1)[0]
        if n/len(self.rows)<2/3 or self.guard.sign!=winner:return None
        return winner.removeprefix('allow_')


def validate_recovery(saved, request, now, allowed_indices):
    """Validate a new explicit recovery; this never arms or supplies heading itself."""
    if saved.get('stop_ack') is not True or saved.get('process_released') is not True:
        raise ValueError('stop_unconfirmed or device ownership not released')
    if saved.get('fault') in ('stop_unconfirmed', 'board_reboot'):
        raise ValueError('board state must be independently reconciled; no automatic rearm')
    if request.get('judge_permitted') is not True:
        raise ValueError('recovery must be explicitly permitted by judge')
    if request.get('run_id') != saved.get('run_id') or request.get('epoch') != saved.get('epoch', 0)+1:
        raise ValueError('stale/duplicate/foreign recovery')
    if request.get('segment') not in allowed_indices:
        raise ValueError('rejoin outside current permitted section')
    if not finite(request.get('body_yaw')) or request.get('heading_source') != 'measured_body_yaw':
        raise ValueError('fresh body yaw required; reverse movement course is not body yaw')
    if not finite(request.get('observed_at')) or not 0 <= now-request['observed_at'] <= .5:
        raise ValueError('stale recovery pose')
    if request.get('stationary') is not True or request.get('fresh_fix') is not True:
        raise ValueError('stopped fresh FIX required')
    if request.get('completed', saved.get('completed', [])) != saved.get('completed', []):
        raise ValueError('recovery cannot fabricate completed missions')
    return {**saved, 'epoch': request['epoch'], 'segment': request['segment'],
            'body_yaw': request['body_yaw'], 'prepared': False}
