"""One-shot receive synchronization BEFORE identification/ARM; DISARM only.
Raw bytes are still recorded by Client. The normal parser/session is unmodified
and resumes strict error/order checks immediately after two fresh DISARM replies.
"""
from main_event_serial import SerialFrames
from step_manual_protocol import Command, DISARMED

class StartupParser:
    def __init__(self,client):
        self.client=client;self.parser=SerialFrames(client._event);self.started=None;self.done=False
        self.previous=None;self.challenge=None;self.proofs=0;self.failure=None
        self.detail='initial_receive_sync';self.quarantined=0
    def snapshot(self):
        return dict(complete=self.done,detail=self.failure or self.detail,
                    disarm_replies=self.proofs,quarantined_batches=self.quarantined,
                    late_crc_valid_frames=self.parser.late_frames)
    def feed(self,raw,now):
        if self.done:return self.parser.feed(raw,now)
        if self.failure:return [],[self.failure]
        if self.started is None:self.started=now
        if now-self.started>=2.5:
            self.failure='startup_sync_timeout: fresh DISARM replies not confirmed'
            return [],[self.failure]
        values,errors=self.parser.feed(raw,now)
        if not values:
            if errors:self.proofs=0;self.quarantined+=1
            return [],[]
        previous=self.previous;bad=bool(errors)
        newest=previous
        for value in values:
            if newest is not None:
                if value.boot_id!=newest.boot_id:
                    self.failure='startup_board_reboot';return [],[self.failure]
                if value.request_id<=newest.request_id:
                    bad=True;continue
                if ((value.millis-newest.millis)&0xffffffff)>=0x80000000:
                    bad=True;continue
            newest=value
        if newest is previous:
            self.proofs=0;self.quarantined+=1;return [],[]
        self.previous=newest
        ack=bool(not bad and self.challenge and newest.boot_id==self.challenge[0] and
                 newest.request_id>self.challenge[1] and 0<now-self.challenge[2]<.15 and
                 newest.state==DISARMED and newest.reason==1 and
                 not any((newest.requested_drive,newest.requested_steer,newest.actual_drive,newest.actual_steer)))
        self.proofs=self.proofs+1 if ack else 0
        if bad:
            self.quarantined+=1
            self.client._event('startup_rx_quarantined',frames=len(values),errors=errors,
                               boot=newest.boot_id,request=newest.request_id)
        if self.proofs>=2 and not self.parser.pending:
            self.done=True;self.detail='fresh_disarm_sync_confirmed'
            self.client._event('startup_sync_confirmed',boot=newest.boot_id,request=newest.request_id,
                               received_at=now,disarm_replies=self.proofs)
            return [newest],[]
        # Only a stopped-output command is permitted during acquisition. It is
        # bound to an actually received boot/request, never a guessed request.
        if not self.client.transport.out_waiting:
            self.client.send(Command(newest.boot_id,newest.request_id))
            if not self.client.failed:self.challenge=(newest.boot_id,newest.request_id,now)
        self.detail='waiting_fresh_disarm_replies'
        return [],[]

def enable_startup_sync(client):
    if client.session.identified or client.session.locked_by_fault:
        raise ValueError('startup sync only on a new, unidentified client')
    client.parser=StartupParser(client)

def startup_status(trial):
    parser=getattr(getattr(trial,'client',None),'parser',None)
    return parser.snapshot() if isinstance(parser,StartupParser) else None
