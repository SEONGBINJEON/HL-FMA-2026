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
  void command(uint32_t now,uint8_t action,int8_t drive=0,int8_t steer=0,uint8_t deadman=0,uint8_t pwm=60,bool bad_crc=false,bool stale=false){
    uint8_t bytes[COMMAND_SIZE]={0xF1,0xA5,2,13}; const Snapshot s=core.snapshot();
    writeU32(bytes+4,s.boot_id);writeU32(bytes+8,s.request_id+(stale?1:0));bytes[12]=action;
    bytes[13]=static_cast<uint8_t>(drive);bytes[14]=static_cast<uint8_t>(steer);bytes[15]=deadman;
    bytes[16]=pwm; const uint16_t crc=crc16(bytes+2,15);bytes[17]=static_cast<uint8_t>(crc);bytes[18]=static_cast<uint8_t>(crc>>8);if(bad_crc)bytes[17]^=1;
    for(uint8_t byte:bytes){tick(now);core.feed(byte,now);tick(now);}
  }
  void ready(uint32_t start=0){request(start);command(start+1,CONTROL);assert(!step.used()&&!step.active());request(start+50);command(start+51,ARM);assert(core.snapshot().state==ARMED&&!step.active());}
  void control(uint32_t at,int8_t drive,int8_t steer,uint8_t deadman=1){request(at);command(at+1,CONTROL,drive,steer,deadman);}
};

static void held_drive_and_neutral_transitions(){
  Harness h;h.ready();h.control(100,1,0);assert(h.step.actualDrive()==1);
  for(uint32_t at=150;at<=1150;at+=50){h.control(at,1,0);assert(h.step.actualDrive()==1);}
  h.control(1200,0,0,0);assert(!h.step.active());assert(h.core.snapshot().state==ARMED&&h.core.snapshot().reason==READY);
  h.control(1250,-1,0);assert(h.step.actualDrive()==-1);h.control(1300,-1,1);assert(h.step.actualDrive()==-1&&h.step.actualSteer()==1);
  h.control(1350,-1,0);assert(h.step.actualDrive()==-1&&h.step.actualSteer()==0);h.control(1400,0,0,0);h.control(1450,0,1);assert(h.step.actualSteer()==1);
}

static void continuous_drive_and_steer_without_timer(){
  Harness h;h.ready();h.control(100,1,1);
  for(uint32_t at=150;at<=60100;at+=50){h.control(at,1,1);assert(h.step.actualDrive()==1);assert(h.step.actualSteer()==1);}
  h.control(60150,0,0,0);assert(!h.step.active());
}

static void continuous_travel_and_rail_guards(){
  Harness h;h.ready();h.control(100,1,1);
  for(uint16_t adc:{uint16_t(501),uint16_t(600),uint16_t(700)}){
    h.adc=adc;h.tick(110);assert(h.step.actualSteer()==1&&h.step.actualDrive()==1);
  }
  h.adc=1;h.tick(120);assert(!h.step.actualSteer()&&h.step.actualDrive()==1);
  h.adc=5;h.control(150,1,1);assert(!h.step.actualSteer());
  h.control(200,1,0);h.control(250,1,-1);assert(h.step.actualSteer()==-1);
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
  for(uint32_t at=150;at<=1150;at+=50){h.control(base+at,1,0);assert(h.step.active());}
  h.control(base+1200,0,0,0);assert(!h.step.active()&&h.core.snapshot().state==ARMED);
}

static void pwm_acceptance_and_rejected_frames(){
  Harness h;h.ready();assert(h.core.drivePwm()==60);
  for(uint8_t pwm:{uint8_t(0),uint8_t(65),uint8_t(255)}){
    static uint32_t at=100;h.request(at);h.command(at+1,CONTROL,1,1,1,pwm);assert(h.core.drivePwm()==pwm);at+=50;
  }
  h.request(250);h.command(251,CONTROL,1,1,1,10,true);assert(h.core.drivePwm()==255&&!h.step.active());
  Harness stale;stale.ready();stale.request(100);stale.command(101,CONTROL,1,0,1,200,false,true);
  assert(stale.core.drivePwm()==60&&!stale.step.active());
  Harness disarm;disarm.ready();disarm.control(100,1,0);disarm.command(110,DISARM,0,0,0,250);
  assert(!disarm.step.active()&&disarm.core.drivePwm()==60);
}
static void no_pulses_and_release(){
  Harness h;h.ready();h.control(100,0,1);
  h.tick(190);assert(h.step.actualSteer()==1);
  h.tick(191);assert(h.step.actualSteer()==1);
  h.control(200,0,1);assert(h.step.actualSteer()==1);
  h.tick(249);assert(h.step.actualSteer()==1);
  h.control(250,0,1);assert(h.step.actualSteer()==1);
  h.control(300,0,0,0);assert(h.step.actualSteer()==0);
  h.control(350,0,-1);assert(h.step.actualSteer()==-1);
  Harness wrap;const uint32_t base=UINT32_MAX-180;wrap.ready(base);wrap.control(base+100,0,1);
  for(uint32_t at=150;at<1000;at+=50){wrap.control(base+at,0,1);assert(wrap.step.actualSteer()==1);}
}
int main(){no_pulses_and_release();pwm_acceptance_and_rejected_frames();held_drive_and_neutral_transitions();continuous_drive_and_steer_without_timer();continuous_travel_and_rail_guards();explicit_stop_errors_and_lease_timeout();simultaneous_axes_and_sign_reversal_rules();millis_wrap_held_drive();puts("PASS: 8 groups; continuous steering, no relative travel cap, release and wrap; PWM acceptance/rejection, 60s continuous axes, ADC, lease, reversal, wrap.");}
