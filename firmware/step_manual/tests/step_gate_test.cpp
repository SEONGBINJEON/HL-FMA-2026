#include "../step_gate.h"
#include <assert.h>
#include <stdio.h>
#include <initializer_list>
using namespace hl_manual;
using namespace hl_step_manual;

struct Harness {
  Core core; StepGate step; uint16_t adc;
  Harness():core(88,true),adc(400){}
  void tick(uint32_t now){core.setInputs(adc,false,now);step.update(core.snapshot(),adc,now);}
  void request(uint32_t now){uint8_t out[STATUS_SIZE];tick(now);assert(core.makeStatus(out,now,0));tick(now);}
  void command(uint32_t now,uint8_t action,int8_t drive=0,int8_t steer=0,uint8_t deadman=0){
    uint8_t bytes[COMMAND_SIZE]={0xF1,0xA5,1,12}; const Snapshot s=core.snapshot();
    writeU32(bytes+4,s.boot_id);writeU32(bytes+8,s.request_id);bytes[12]=action;
    bytes[13]=static_cast<uint8_t>(drive);bytes[14]=static_cast<uint8_t>(steer);bytes[15]=deadman;
    const uint16_t crc=crc16(bytes+2,14);bytes[16]=static_cast<uint8_t>(crc);bytes[17]=static_cast<uint8_t>(crc>>8);
    for(uint8_t byte:bytes){tick(now);core.feed(byte,now);tick(now);}
  }
  void ready(uint32_t start=0){request(start);command(start+1,CONTROL);assert(!step.used()&&!step.active());request(start+50);command(start+51,ARM);assert(core.snapshot().state==ARMED&&!step.active());}
  void control(uint32_t at,int8_t drive,int8_t steer,uint8_t deadman=1){request(at);command(at+1,CONTROL,drive,steer,deadman);}
};

static void held_drive_and_neutral_transitions(){
  Harness h;h.ready();h.control(100,1,0);assert(h.step.actualDrive()==1);
  for(uint32_t at=150;at<=10150;at+=50){h.control(at,1,0);assert(h.step.actualDrive()==1);}
  h.control(10200,0,0,0);assert(!h.step.active());assert(h.core.snapshot().state==ARMED&&h.core.snapshot().reason==READY);
  h.control(10250,-1,0);assert(h.step.actualDrive()==-1);h.control(10300,-1,1);assert(h.step.actualDrive()==-1&&h.step.actualSteer()==1);
  h.control(10350,-1,0);assert(h.step.actualDrive()==-1&&h.step.actualSteer()==0);h.control(10400,0,0,0);h.control(10450,0,1);assert(h.step.actualSteer()==1);
}

static void steering_press_limits_and_restarts_after_neutral(){
  Harness h;h.ready();h.control(100,0,1);assert(h.step.actualSteer()==1);
  for(uint32_t at=150;at<=1050;at+=50){h.control(at,0,1);assert(h.step.active());}
  h.tick(1100);assert(h.step.active());h.tick(1101);assert(!h.step.active()&&h.step.used());
  h.control(1150,0,1);assert(!h.step.active());h.control(1200,0,0,0);assert(h.core.snapshot().state==ARMED&&!h.step.active());
  h.control(1250,0,-1);assert(h.step.actualSteer()==-1);
}

static void adc_guard_restarts_only_after_neutral(){
  Harness h;h.ready();h.control(100,0,1);h.adc=499;h.tick(110);assert(h.step.active());
  h.adc=500;h.tick(111);assert(!h.step.active());h.control(150,0,1);assert(!h.step.active());
  h.control(200,0,0,0);h.control(250,0,-1);assert(h.step.actualSteer()==-1);
  for(uint16_t adc:{uint16_t(0),uint16_t(1),uint16_t(1022),uint16_t(1023)}){
    Harness drive;drive.adc=adc;drive.ready();drive.control(100,1,0);assert(drive.step.actualDrive()==1);
    const int8_t inward=adc<=1?-1:1; Harness recover;recover.adc=adc;recover.ready();recover.control(100,0,inward);assert(recover.step.actualSteer()==inward);
    Harness outward;outward.adc=adc;outward.ready();outward.control(100,0,-inward);assert(!outward.step.active());
  }
}

static void explicit_stop_errors_and_lease_timeout(){
  Harness stop;stop.ready();stop.control(100,1,0);stop.command(110,DISARM);assert(!stop.step.active());
  Harness malformed;malformed.ready();malformed.control(100,1,0);malformed.core.feed('F',110);malformed.tick(110);assert(!malformed.step.active());
  Harness expired;expired.ready();expired.control(100,1,0);expired.tick(250);assert(!expired.step.active()&&expired.core.snapshot().reason==TIMEOUT);
}

static void simultaneous_axes_and_sign_reversal_rules(){
  Harness mixed;mixed.ready();mixed.control(100,1,1);assert(mixed.step.actualDrive()==1&&mixed.step.actualSteer()==1);
  mixed.control(150,1,0);assert(mixed.step.actualDrive()==1&&!mixed.step.actualSteer());
  mixed.control(200,1,-1);assert(mixed.step.actualDrive()==1&&mixed.step.actualSteer()==-1);
  for(uint8_t axis=0;axis<2;++axis){Harness h;h.ready();h.control(100,axis?0:1,axis?1:0);h.control(150,axis?0:-1,axis?-1:0);assert(!h.step.active()&&h.core.snapshot().state==DISARMED&&h.core.snapshot().reason==REVERSAL);}
}

static void millis_wrap_held_drive(){
  const uint32_t base=UINT32_MAX-180; Harness h;h.ready(base);h.control(base+100,1,0);
  for(uint32_t at=150;at<=2150;at+=50){h.control(base+at,1,0);assert(h.step.active());}
  h.control(base+2200,0,0,0);assert(!h.step.active()&&h.core.snapshot().state==ARMED);
}

int main(){held_drive_and_neutral_transitions();steering_press_limits_and_restarts_after_neutral();adc_guard_restarts_only_after_neutral();explicit_stop_errors_and_lease_timeout();simultaneous_axes_and_sign_reversal_rules();millis_wrap_held_drive();puts("PASS: 6 simultaneous persistent-session groups; held drive, steering guards, errors, axis changes, wrap.");}
