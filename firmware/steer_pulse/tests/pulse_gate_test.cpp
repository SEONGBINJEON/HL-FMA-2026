#include "../pulse_gate.h"
#include <assert.h>
#include <stdio.h>
#include <vector>
using namespace hl_manual;
using namespace hl_steer_pulse;

struct Harness {
  Core core;
  PulseGate pulse;
  uint16_t adc;
  Harness():core(88,true),adc(400){}
  void tick(uint32_t now){core.setInputs(adc,false,now);pulse.update(core.snapshot(),adc,now);}
  void request(uint32_t now){uint8_t out[STATUS_SIZE];tick(now);assert(core.makeStatus(out,now,0));tick(now);}
  void command(uint32_t now,uint8_t action,int8_t drive=0,int8_t steer=0,uint8_t deadman=0){
    uint8_t bytes[COMMAND_SIZE]={0xF1,0xA5,1,12};
    const Snapshot s=core.snapshot();writeU32(bytes+4,s.boot_id);writeU32(bytes+8,s.request_id);
    bytes[12]=action;bytes[13]=static_cast<uint8_t>(drive);bytes[14]=static_cast<uint8_t>(steer);bytes[15]=deadman;
    const uint16_t crc=crc16(bytes+2,14);bytes[16]=static_cast<uint8_t>(crc);bytes[17]=static_cast<uint8_t>(crc>>8);
    for(uint8_t byte:bytes){tick(now);core.feed(byte,now);tick(now);}
  }
  void ready(uint32_t start=0){request(start);command(start+1,CONTROL);request(start+50);command(start+51,ARM);assert(!pulse.active());}
  void start(uint32_t start=100){request(start);command(start+1,CONTROL,0,-1,1);}
};

static void time_and_repeat(){
  Harness h;h.ready();h.start();assert(h.pulse.active()&&h.pulse.used());
  h.request(150);h.command(151,CONTROL,0,-1,1);
  h.tick(200);assert(h.pulse.active());h.tick(201);assert(!h.pulse.active());
  h.command(202,DISARM);h.ready(250);h.start(350);assert(!h.pulse.active()&&h.pulse.used());
  const uint32_t base=UINT32_MAX-180;
  Harness wrap;wrap.ready(base);wrap.start(base+100);wrap.tick(base+200);assert(wrap.pulse.active());
  wrap.tick(base+201);assert(!wrap.pulse.active());
}
static void adc_bounds(){
  for(int direction:{-1,1}){
    Harness h;h.ready();h.start();assert(h.pulse.active());
    h.adc=static_cast<uint16_t>(400+direction*11);h.tick(110);assert(h.pulse.active());
    h.adc=static_cast<uint16_t>(400+direction*12);h.tick(111);assert(!h.pulse.active()&&h.pulse.used());
    h.adc=400;h.tick(112);assert(!h.pulse.active());
  }
  for(uint16_t adc:{uint16_t(0),uint16_t(1),uint16_t(1022),uint16_t(1023),uint16_t(65535)}){
    Harness h;h.adc=adc;h.ready();h.start();assert(!h.pulse.active()&&h.pulse.used());
    h.adc=400;h.tick(110);assert(!h.pulse.active());
  }
}
static void stops(){
  for(uint8_t cause=0;cause<4;++cause){
    Harness h;h.ready();h.start();assert(h.pulse.active());
    if(cause==0)h.command(110,DISARM);
    if(cause==1){h.request(150);h.command(151,CONTROL,0,0,1);}
    if(cause==2){h.core.feed('F',110);h.tick(110);}
    if(cause==3)h.tick(250);
    assert(!h.pulse.active()&&h.pulse.used());
  }
}
static void permitted_command_only(){
  Harness h;h.request(0);h.command(1,CONTROL,0,-1,1);assert(!h.pulse.active());
  for(uint8_t kind=0;kind<3;++kind){
    Harness bad;bad.ready();bad.request(100);
    if(kind==0)bad.command(101,CONTROL,1,-1,1);
    if(kind==1)bad.command(101,CONTROL,0,1,1);
    if(kind==2)bad.command(101,CONTROL,0,-1,0);
    assert(!bad.pulse.active());
  }
}
int main(){time_and_repeat();adc_bounds();stops();permitted_command_only();puts("PASS: 4 steering-pulse groups; copied Core, time/wrap, ADC both signs/rails, repeat/rearm, stop/input gate.");}
