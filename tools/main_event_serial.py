"""Bounded serial-fragment reassembly without refreshing on late fragments alone.
The 30ms threshold marks a late fragment, not loss of a complete status stream.
Late CRC-valid frames are deferred until a fresh complete frame follows. All
frames then reach Session in wire order. 150ms age, CRC/order/boot guards remain.
"""
from joystick_protocol import Parser, PAYLOAD, SIZE, decode

class SerialFrames(Parser):
    def __init__(self,event=None):
        super().__init__();self.deferred=[];self.event=event;self.late_frames=0
    def feed(self,data,now):
        if len(data)>4096:return [],['RX flood']
        values=[];errors=[]
        if not self.identified:
            self.diagnostic_tail=(self.diagnostic_tail+data)[-8192:]
            if b'HENES F8 DRIVE TEST' in self.diagnostic_tail or b'[MONITOR] Drive=' in self.diagnostic_tail:self.unsupported_banner=True
        if self.pending and now-self.since>=.150:
            self.pending.clear();errors.append('Partial status exceeded communication deadline')
        if self.deferred and now-self.deferred[0][0]>=.150:
            self.deferred.clear();errors.append('Deferred status exceeded communication deadline')
        for byte in data:
            if not self.pending:
                if byte==0xf1:self.pending.append(byte);self.since=now
                elif self.identified:errors.append('Unexpected byte')
                continue
            self.pending.append(byte)
            if (len(self.pending)==2 and byte!=0xa5) or (len(self.pending)==4 and self.pending[2:]!=bytes([150,PAYLOAD.size])):
                if self.identified:errors.append('Wrong frame')
                self.pending.clear()
            elif len(self.pending)==SIZE:
                try:
                    value=decode(bytes(self.pending));self.identified=True
                    if now-self.since>=.03:
                        self.deferred.append((self.since,value));self.late_frames+=1
                        if self.event:self.event('serial_late_fragment_crc_valid',age_s=now-self.since,boot=value.boot_id,request=value.request_id)
                    else:
                        # Do not drop ordering checks for a deferred older frame.
                        values.extend(v for _,v in self.deferred);self.deferred.clear();values.append(value)
                except ValueError as exc:errors.append(str(exc))
                self.pending.clear()
        return values,errors
