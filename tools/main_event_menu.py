#!/usr/bin/env python3
"""Single terminal operator menu. Opening the menu does not open vehicle ports."""
import json
from pathlib import Path
import subprocess
import sys
import time
from datetime import datetime
from zoneinfo import ZoneInfo
import main_event
from main_event_manual import current_mission, stopped, load_handoff
from main_event_plan import ROOT, NEXT_RECOVERY

NAMES={'s_curve':'S자','t_parking':'T주차','p_parking':'P주차','light_1':'신호1',
       'light_2':'신호2','light_3':'신호3','hill':'경사로','finish_fork':'종점 분기'}
ENTRIES={'s_curve':'실제 충돌·심판 허용 때만 S 출구 → 지도 S 화살표','t_parking':'T 통과 후 → 지도 T 화살표',
         'p_parking':'P 통과 후 → 지도 P 화살표'}
MAP=ROOT/'data/main_event_autonomous_passages_20260920.png'


def snapshot():
    if not main_event.ACTIVE.exists():return {}
    session=main_event.active()
    s=json.loads((session/'state.json').read_text());s['session']=str(session)
    age=time.monotonic()-s.get('updated_at',0)
    s['heartbeat_fresh']=0<=age<1.
    if not s['heartbeat_fresh']:s['ready']=False
    return s


def confirmed(s):
    try:stopped(s);return True
    except ValueError:return False


class Menu:
    def __init__(self,read=snapshot,execute=None,ask=input,emit=print,log=None):
        self.read,self.execute,self.ask,self.emit=read,execute or self.command,ask,emit
        self.log=log
        self.manual_unconfirmed=False

    def event(self,name,**fields):
        if self.log:
            with self.log.open('a') as out:
                out.write(json.dumps({'kst':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
                    'event':name,**fields},ensure_ascii=False)+'\n')

    def command(self,args):
        argv=[sys.executable,str(ROOT/'tools/main_event.py'),*args]
        self.emit('실행: '+' '.join(args))
        result=subprocess.run(argv,cwd=ROOT,text=True,capture_output=True)
        self.event('command',argv=argv,exit_code=result.returncode,stdout=result.stdout,stderr=result.stderr)
        if result.returncode:self.emit((result.stderr or result.stdout).strip())
        return result.returncode,result.stdout

    def show(self):
        s=self.read();mission=current_mission(s)
        self.emit('\n현재: '+NAMES.get(mission,mission or '세션 없음/종료')+' | '+str(s.get('section','미준비'))+
                  ' | '+('READY' if s.get('ready') and s.get('heartbeat_fresh') else '대기/정지')+
                  ' | '+str(s.get('phase','없음')))
        waits=[]
        b=s.get('board') or {}
        if s.get('board_locked') or b.get('fault'):waits.append('보드 제어 잠금: '+str(s.get('board_first_fault') or b.get('first_fault') or '원인 로그 확인'))
        sync=s.get('startup_sync')
        if sync and not sync.get('complete'):waits.append('초기 동기화: '+str(sync.get('detail')))
        if b:
            if not 0<=time.monotonic()-b.get('received_at',0)<.15:waits.append('보드 수신 오래됨')
            if b.get('state')!='DISARMED' and s.get('phase')=='waiting':waits.append('보드 DISARMED 미확인: '+str(b.get('state')))
            self.emit('보드: '+str(b.get('state'))+' / adapter='+str(s.get('board_adapter_phase',b.get('adapter_phase')))+' / locked='+str(s.get('board_locked',b.get('fault'))))
        if s and not s.get('heartbeat_fresh'):waits.append('상태 갱신 없음')
        if s.get('phase')=='waiting':
            if not s.get('board'):waits.append('보드 입력 없음')
            if not s.get('pose_fresh'):waits.append('신선한 RTK FIX 없음')
            if not s.get('correction_fresh'):waits.append('NTRIP 보정 대기')
            if not s.get('gnss_rate_ready'):waits.append('GNSS 주기 대기')
            if not s.get('vision_ready'):self.emit('보조 인지 저하: 카메라 영상/판정 지연 (출발 필수 READY와 별도; 신호에서는 정지 가능)')
            if s.get('lidar_stop_applied') and s.get('obstacle')!='clear':waits.append('임무 라이다 '+str(s.get('obstacle','미수신')))
        self.emit('이유: '+str(s.get('reason') or s.get('fault') or (' / '.join(waits)) or
                  ('출발 선택 대기' if s.get('ready') else '준비/상태 확인 필요')))
        if waits:self.emit('차단 상세: '+' / '.join(waits))
        self.emit('라이다 원관측: '+str(s.get('lidar_raw_observation',s.get('obstacle','unknown')))+' | 정지 적용: '+str(s.get('lidar_stop_applied',False))+' | '+str(s.get('lidar_stop_scope','출발에서는 미적용')))
        if s.get('session_choices'):self.emit('대기소 선택: '+json.dumps(s['session_choices'],ensure_ascii=False))
        if s.get('parking_execution'):
            for key,v in s['parking_execution'].items():
                mode='선택 경로 수행 시도' if v['mode']=='selected_path_attempt' else ('미진입 통과: 현재 조향 한계로 후보 없음' if v['choice'] else '선택 대기: 미지정이면 미진입 통과')
                self.emit(key.upper()+' 주차: '+str(v.get('choice') or '미정')+' / '+mode)
        if s.get('choices_error'):self.emit('선택 거부: '+s['choices_error'])
        if s.get('perception_health'):self.emit('보조 센서: '+json.dumps(s['perception_health'],ensure_ascii=False))
        if s.get('signal_wait_detail'):self.emit('신호 대기 상세: '+s['signal_wait_detail'])
        if s.get('maneuver_reason'):self.emit('경로: '+str(s['maneuver_reason']))
        if s.get('localization_error'):self.emit('재개 위치: '+str(s['localization_error']))
        if s.get('section') in NEXT_RECOVERY.values():
            self.emit('재개 후보: '+json.dumps(s.get('recovery_hint'),ensure_ascii=False)+
                      ' | 방향 취득 준비: '+str(s.get('recovery_bootstrap_ready',False)))
            if not (s.get('recovery_hint') or {}).get('candidates'):self.emit('GPS 후보 없음: FIX/입구 위치 확인. 지도의 짧은 색 선 안으로 배치 필요.')
        if s.get('nonprogress'):self.emit('실제 진행 없음: %.1f초 | 신호 대기: %s | 시간초과로 센서 조건 해제 안 함' % (s['nonprogress']['seconds_without_progress'],s['nonprogress']['signal_wait']))
        self.emit('PWM 선택/최근 wire: '+str(s.get('selected_pwm','--'))+' / '+str(s.get('wire_pwm','--'))+' | '+str(s.get('speed_reason','미준비'))+' (m/s 아님; 모터 실출력 측정 아님)')
        b=s.get('board') or {};fresh=s.get('heartbeat_fresh') and 0<=time.monotonic()-b.get('received_at',0)<.15
        sonar={k:(b.get(k) if fresh and isinstance(b.get(k),(int,float)) and 0<b[k]<999 else None)
               for k in ('left_cm','center_cm','right_cm')}
        self.emit('초음파 L/C/R(cm): '+' / '.join('--' if v is None else str(v) for v in sonar.values())+
                  ' (보조만; --=미반환/오래됨, 채널 실물 방향 미확인)')
        self.event('status',session=s.get('session'),phase=s.get('phase'),mission=mission,
                   ready=s.get('ready',False),reason=s.get('reason'),sonar=sonar,board_fresh=bool(fresh))
        self.emit('수동 후 다음 구간: '+ENTRIES[mission] if mission=='s_curve' else
                  'T/P/더미 수동 통과 금지. 출발 전 확정한 자동 정책으로 연결; 0은 비상 정지.')
        self.emit('1 준비 | N 새 경기 | 2 명시적 자율 출발/재출발 | M 실제 S충돌 허용복구만 | R 재개 준비 재시도 | 0 정지 | I 상태 | G 지도 | V 카메라 한 장 | Q 정지 후 종료')
        return s

    def stop(self):
        if self.manual_unconfirmed:
            self.emit('수동 종료 정지 미확인: 이전 자율 정지 기록으로 성공 처리하지 않음. 물리 정지·수동 창 종료/로그 확인 필요.')
            return False
        s=self.read()
        if not s:return True
        if confirmed(s):return True
        if s.get('phase')=='stopped' and s.get('process_released') is True:
            self.emit('실행기는 종료됐지만 정지 관측 실패 기록이 남았습니다. Q 반복 대신 Ctrl+C로 메뉴 종료(실패 유지). 차량 고정 후 python3 tools/main_event.py recheck-stop --vehicle-secured')
            return False
        code,_=self.execute(['stop'])
        if code or not confirmed(self.read()):
            self.emit('정지/해제 미확인: 다음 단계 중단. 스틱 중립·물리 정지 수단 확인. 다른 제어 창을 열지 마세요.');return False
        return True

    def resume(self):
        s=self.read()
        if not confirmed(s):raise ValueError('재개 준비 전 정지/해제 확인 필요')
        old=Path(s['session'])
        if not (old/'manual_handoff.json').is_file():
            raise ValueError('R은 실제 S충돌 허용 수동복구 후 재개 전용입니다. 초기 출발 실패는 정상 종료 확인 후 N → NEW → 1로 새 준비; I로 잠금 원인 확인')
        info=json.loads((old/'session.json').read_text())
        receipt=load_handoff(old,info)  # persisted manual_passage_stopped evidence
        self.manual_unconfirmed=False
        self.emit(NAMES[receipt['mission']]+' 생략 → '+ENTRIES[receipt['mission']])
        code,_=self.execute(['recover','--after-manual'])
        if code:raise ValueError('재개 준비 실패. I로 상태 확인; 새 수동 통과 없이 R로 준비만 재시도 가능')
        self.emit('재개 준비 요청 완료. I로 READY 확인 후 2를 직접 선택. 아직 자율 출발하지 않았습니다.')

    def action(self,key):
        key=key.lower().strip()
        if key=='1':
            s=self.read()
            if s:raise ValueError('기존 세션이 있습니다. 중간에 새 출발로 덮어쓰지 않습니다. 수동 후에는 M/R 사용; 새 경기만 N 사용')
            lane=self.ask('출발 차선: L 왼쪽 / R 오른쪽: ').lower().strip()
            if lane not in ('l','r'):raise ValueError('L 또는 R 선택 필요')
            code,_=self.execute(['prepare','--live','--lane',{'l':'left','r':'right'}[lane]])
            if code:raise ValueError('준비 실패: I로 상태 확인')
        elif key=='n':
            s=self.read()
            if self.manual_unconfirmed:raise ValueError('수동 종료 미확인; 새 경기 전환 불가')
            if s and not confirmed(s):raise ValueError('새 경기 전 기존 정지/해제 확인 필요. 0 사용')
            if self.ask('중간 재개에는 사용 금지. 새 경기를 처음부터 시작하려면 NEW 입력: ').strip()!='NEW':
                raise ValueError('새 경기 전환 취소')
            archive_active()
            self.emit('이전 기록 보존 완료. 1로 새 출발 준비')
        elif key=='2':
            s=self.read()
            if not (s.get('ready') and s.get('heartbeat_fresh') and s.get('phase')=='waiting'):
                raise ValueError('현재 READY가 아닙니다. I로 대기 이유 확인')
            if s.get('section') in NEXT_RECOVERY.values() and not s.get('recovery_bootstrap_ready'):
                raise ValueError('수동 후 방향 취득 준비 미완료')
            code,_=self.execute(['start'])
            if code:raise ValueError('출발 요청 거부: 상태 확인')
        elif key=='m':
            mission=current_mission(self.read())
            if mission!='s_curve':raise ValueError('규정 정정: T/P/더미/일반 구간 M 수동 통과 금지')
            if self.ask('실제 S 충돌이 발생했고 심판이 이번 복구를 허용한 경우에만 C 입력(그 외 Enter 취소): ').strip().lower()!='c':
                raise ValueError('충돌 예외 미확인: 수동 복구 취소')
            self.emit(ENTRIES[mission]+' 위치에서 진행방향 정렬·약2m 직진 여유 → 정지·스틱 중립·창 닫기')
            if not self.stop():return True
            self.manual_unconfirmed=True
            code,output=self.execute(['manual','--mission',mission,'--collision-recovery-confirmed'])
            records=[]
            for line in output.splitlines():
                try:records.append(json.loads(line))
                except ValueError:pass
            if code or not any(isinstance(r,dict) and r.get('manual_passage_stopped') is True for r in records):
                raise ValueError('manual_passage_stopped 미확인. 재개 안 함; 수동 로그/정지 상태 확인')
            self.resume()
        elif key=='r':self.resume()
        elif key in ('0','q'):
            ok=self.stop()
            if key=='q' and ok:return False
        elif key=='g':
            self.emit('자동 미진입 경로 지도(참고, 실측 아님): '+str(MAP))
            subprocess.Popen(['xdg-open',str(MAP)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        elif key=='v':
            session=Path(self.read()['session'])
            frame=json.loads((session/'frames/latest.json').read_text())
            age=time.monotonic()-frame['capture_at']
            if not 0<=age<=.6:raise ValueError('현재 OAK 프레임 없음/오래됨; 저장 사진을 현재 영상으로 표시하지 않음')
            decision=json.loads((session/'signal.json').read_text())
            self.emit('OAK 한 장 시각 기준 경과 %.3fs / 추론 %.3fs / 판정 프레임 경과 %s' %
                      (age,decision.get('inference_s',0),decision.get('frame_age_s')))
            self.emit('사진 창은 실시간 영상이 아닙니다. 닫고 V를 다시 누르면 새 한 장입니다.')
            self.event('camera_snapshot',frame=frame,decision_age_s=time.monotonic()-decision['decision_at'])
            subprocess.Popen(['xdg-open',str(session/'frames'/frame['file'])],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        elif key!='i':self.emit('표시된 키를 선택하세요.')
        return True


def archive_active():
    if not main_event.ACTIVE.exists():return
    stopped(snapshot())
    import fcntl,os
    fd=os.open(ROOT/'data/main_event.lock',os.O_RDWR|os.O_CREAT,0o600)
    try:
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        target=main_event.ACTIVE.with_name('main_event_active_archived_'+str(time.time_ns())+'.json')
        main_event.ACTIVE.rename(target)
    finally:os.close(fd)


def main(argv=None):
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.parse_args(argv)
    folder=ROOT/'data'/('me_menu_'+datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S'))
    folder.mkdir(exist_ok=False)
    menu=Menu(log=folder/'events.jsonl')
    print('메뉴만 열렸습니다. 장치 준비는 1부터. 로그: '+str(folder))
    while True:
        try:
            menu.show()
            if not menu.action(input('선택 > ')):return 0
        except (EOFError,KeyboardInterrupt):
            print('\n메뉴 입력 종료: 자동 정지/해제 요청')
            return 0 if menu.stop() else 3
        except (ValueError,OSError,KeyError) as exc:print('중단: '+str(exc))


if __name__=='__main__':raise SystemExit(main())
