"""Finish-only fixed HYPER branch fallback; never grants traffic-light permission."""
from main_event_core import ForkVotes

class FinishChoice:
    def __init__(self,epoch,lane):
        if lane not in ('left','right'):raise ValueError('fixed finish lane required')
        self.operator_lane=None;self.default=lane;self.votes=ForkVotes(epoch);self.started=None;self.choice=None
    def update(self,event,now,s,gate):
        if self.choice is not None:return self.choice
        if gate.name!='finish_fork' or gate.stop_s-s>12.:return None
        if self.started is None:self.started=now
        if self.operator_lane is not None:
            if self.operator_lane not in ('left','right'):raise ValueError('invalid session finish lane')
            self.choice=dict(lane=self.operator_lane,source='operator_session_choice',default_lane=self.default,
                elapsed_s=now-self.started,s_m=s,reason='prestart_choice_locked')
            return self.choice
        observed=self.votes.update(event,now)
        due=now-self.started>=3. or s>=gate.stop_s-gate.approach_m-1.
        if observed is None and not due:return None
        self.choice=dict(lane=observed or self.default,source='fresh_panel_window' if observed else 'fallback_penalty',
            default_lane=self.default,elapsed_s=now-self.started,s_m=s,
            reason='valid_panel_vote' if observed else 'insufficient_fresh_panel_votes')
        return self.choice
