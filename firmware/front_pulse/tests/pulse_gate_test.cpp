#include "../pulse_gate.h"
#include <assert.h>
#include <stdio.h>
#include <vector>

using namespace hl_manual;
using namespace hl_front_pulse;

static std::vector<uint8_t> frame(Core &core, uint8_t action, int8_t drive = 0,
                                int8_t steer = 0, uint8_t deadman = 0) {
  std::vector<uint8_t> bytes(COMMAND_SIZE, 0);
  bytes[0]=0xF1; bytes[1]=0xA5; bytes[2]=1; bytes[3]=12;
  Snapshot state=core.snapshot();
  writeU32(bytes.data()+4,state.boot_id); writeU32(bytes.data()+8,state.request_id);
  bytes[12]=action; bytes[13]=static_cast<uint8_t>(drive);
  bytes[14]=static_cast<uint8_t>(steer); bytes[15]=deadman;
  const uint16_t crc=crc16(bytes.data()+2,14);
  bytes[16]=static_cast<uint8_t>(crc); bytes[17]=static_cast<uint8_t>(crc>>8);
  return bytes;
}

struct Harness {
  Core core;
  PulseGate pulse;
  explicit Harness(uint32_t boot=123,bool valid=true):core(boot,valid){}
  void tick(uint32_t now) { core.tick(now); pulse.update(core.snapshot(),now); }
  void request(uint32_t now) {
    uint8_t bytes[STATUS_SIZE]; tick(now);
    assert(core.makeStatus(bytes,now,0)); tick(now);
  }
  void command(uint32_t now,uint8_t action,int8_t drive=0,int8_t steer=0,uint8_t deadman=0) {
    const auto bytes=frame(core,action,drive,steer,deadman);
    for(uint8_t byte:bytes){tick(now);core.feed(byte,now);tick(now);}
  }
  void ready(uint32_t start=0) {
    request(start); command(start+1,CONTROL);
    assert(!pulse.active()&&!pulse.used());
    request(start+50); command(start+51,ARM);
    assert(core.snapshot().state==ARMED);
    assert(!pulse.active()&&!pulse.used());
  }
  void start(uint32_t at=100) {
    request(at);command(at+1,CONTROL,1,0,1);
    assert(pulse.active()&&pulse.used());
  }
};

static void timeout_and_repeat() {
  Harness h;h.ready();h.start();
  h.request(150);h.command(151,CONTROL,1,0,1);
  h.tick(200);assert(h.pulse.active());
  h.tick(201);assert(!h.pulse.active()&&h.pulse.used());
  h.request(250);h.command(251,CONTROL,1,0,1);assert(!h.pulse.active());
  h.command(252,DISARM);
  h.request(300);h.command(301,CONTROL);
  h.request(350);h.command(351,ARM);
  h.request(400);h.command(401,CONTROL,1,0,1);
  assert(h.core.snapshot().state==ARMED && !h.pulse.active() && h.pulse.used());
}

static void stop_neutral_error_expiry() {
  for(uint8_t mode=0;mode<5;++mode){
    Harness h;h.ready();h.start();
    if(mode==0) h.command(110,DISARM);
    if(mode==1){h.request(150);h.command(151,CONTROL,0,0,1);}
    if(mode==2){h.core.feed('F',110);h.tick(110);}
    if(mode==3){h.core.inputOverload(110);h.tick(110);}
    if(mode==4){h.tick(250);assert(h.core.snapshot().reason==TIMEOUT);}
    assert(!h.pulse.active()&&h.pulse.used());
  }
}

static void requires_fresh_handshake() {
  Harness missing;missing.request(0);missing.command(1,CONTROL,1,0,1);
  assert(!missing.pulse.active()&&!missing.pulse.used());
  Harness held;held.ready();held.request(100);held.command(101,CONTROL,1,0,0);
  assert(!held.pulse.active()&&!held.pulse.used());
  Harness reverse;reverse.ready();reverse.request(100);reverse.command(101,CONTROL,-1,0,1);
  assert(!reverse.pulse.active());
  Harness steer;steer.ready();steer.request(100);steer.command(101,CONTROL,1,1,1);
  assert(!steer.pulse.active());
  Harness invalid(0,false);invalid.request(0);invalid.command(1,CONTROL);
  assert(!invalid.pulse.active()&&!invalid.pulse.used());
}

static void wrap_bound() {
  const uint32_t start=UINT32_MAX-180;
  Harness h;h.ready(start);h.start(start+100);
  h.tick(start+200);assert(h.pulse.active());
  h.tick(start+201);assert(!h.pulse.active()&&h.pulse.used());
}

int main(){
  timeout_and_repeat();stop_neutral_error_expiry();requires_fresh_handshake();wrap_bound();
  puts("PASS: 4 front-pulse scenario groups; real copied Core plus PulseGate, ASAN/UBSAN.");
}
