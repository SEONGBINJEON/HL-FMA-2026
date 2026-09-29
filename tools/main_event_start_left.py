#!/usr/bin/env python3
"""Final left-lane prepare. No ARM/start until explicit fresh-READY key 2."""
import fcntl,os,select,sys,time,signal
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
from main_event_menu import Menu,archive_active,ROOT
from main_event_release import verify
from main_event_control import send
from main_event_degraded import normalize_choices

def prepare_left(menu):
    verify()
    archive_active()  # refuses running/unconfirmed stop; archives only, never deletes
    code,_=menu.execute(['prepare','--live','--lane','left'])
    if code:raise ValueError('L 자동 준비 실패 — 위 오류 확인; 자동 출발 없음')

def main():
    # Second launcher must not archive or stop the first launcher's session.
    fd=os.open(ROOT/'data/main_event_start_left.lock',os.O_RDWR|os.O_CREAT,0o600)
    menu=None;owns=False;interrupt=[False]
    handlers={sig:signal.signal(sig,lambda *_:interrupt.__setitem__(0,True)) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        folder=ROOT/'data'/('me_left_'+datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S'))
        folder.mkdir(exist_ok=False)
        def emit(line):
            if str(line).startswith('1 준비 |'):line='2 명시적 출발 | T Left/Right · P Up/Down · E Left/Right (출발 전) | Q/0 종료 | I 상태 | M 실제 S충돌 허용복구만 | R 허용복구 재시도'
            print(line,flush=True)
        menu=Menu(log=folder/'events.jsonl',emit=emit)
        prepare_left(menu);owns=True
        print('차선 L 고정 · 자동 준비 중 · 입력 전 ARM/출발 없음 · Q/0 종료',flush=True)
        last=None;updated=0
        while True:
            if interrupt[0]:return 0 if menu.stop() else 3
            s=menu.read();ready=bool(s.get('ready') and s.get('heartbeat_fresh') and s.get('phase')=='waiting')
            key=(ready,s.get('phase'),s.get('board_first_fault'),s.get('vision_ready'),s.get('pose_fresh'),s.get('correction_fresh'))
            if key!=last or time.monotonic()-updated>=2:
                menu.show();print('차선 L — 주행 가능 — 2 입력 후 Enter' if ready else '차선 L — 준비 대기/정지 · 위 차단 상세 확인',flush=True)
                last=key;updated=time.monotonic()
            if s.get('phase') in ('stopped','prepare_failed'):
                print('실행기 종료: '+str(s.get('reason')),flush=True);return int(s.get('exit_code',1))
            if select.select([sys.stdin],[],[],.2)[0]:
                line=sys.stdin.readline()
                if not line:return 0 if menu.stop() else 3
                action=line.strip().lower()
                try:
                    words=action.split()
                    if len(words)==2 and words[0] in ('t','p','e'):
                        current=menu.read()
                        if current.get('phase')!='waiting':raise ValueError('선택은 출발 전 대기 중에만 가능')
                        values=normalize_choices({{'t':'t','p':'p','e':'finish'}[words[0]]:words[1]})
                        response=send(current['session'],'choices',values)
                        if not response.get('accepted'):raise ValueError('선택 전달 거부: '+str(response))
                        print('선택 전달됨; 다음 상태의 선택값/주차 수행 정책 확인. T Left는 조향 한계로 미진입.',flush=True)
                        last=None;continue
                    if action in ('q','0'):return 0 if menu.stop() else 3
                    if action not in ('2','m','r','i','g','v'):
                        print('2 출발 | Q/0 종료 | I 상태 | M 실제 S충돌 허용복구만',flush=True);continue
                    if action=='2':
                        fresh=menu.read()
                        if not (fresh.get('ready') and fresh.get('heartbeat_fresh') and fresh.get('phase')=='waiting'):
                            menu.show();print('출발 거부: 최신 READY 아님',flush=True);continue
                    menu.action(action)
                except (ValueError,OSError,KeyError) as exc:print('중단: '+str(exc),flush=True)
    except (KeyboardInterrupt,EOFError):
        return 0 if owns and menu.stop() else 3
    except (ValueError,OSError,KeyError) as exc:
        print('자동 준비 중단: '+str(exc),flush=True)
        if owns:menu.stop()
        return 1
    finally:
        for sig,handler in handlers.items():signal.signal(sig,handler)
        os.close(fd)

if __name__=='__main__':raise SystemExit(main())
