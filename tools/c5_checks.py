#!/usr/bin/env python3
"""Fixed C5 check entry point. Only Codex changes this dispatcher."""
import datetime,json,re,subprocess,sys,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
KST=datetime.timezone(datetime.timedelta(hours=9))

def main():
 stamp=datetime.datetime.now(KST).strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
 out=ROOT/'logs/c5_checks'/stamp;out.mkdir(parents=True)
 sources=['tools/steering_angle_contract.py','tools/integrated_replay.py','tests/test_c5_integration.py','tests/fixtures/c5_replay_synthetic.json','tests/test_c5_contract_review.py']
 for name in sources:
  if not (ROOT/name).is_file():
   print('Missing required file: '+name);return 1
 sandbox=['bwrap','--unshare-all','--die-with-parent','--ro-bind','/usr','/usr','--symlink','usr/bin','/bin','--symlink','usr/lib','/lib','--symlink','usr/lib64','/lib64','--proc','/proc','--dev','/dev','--tmpfs','/tmp','--bind',str(ROOT),str(ROOT),'--chdir',str(ROOT),'--clearenv','--setenv','PATH','/usr/bin:/bin','--setenv','HOME','/tmp','--setenv','LANG','C.UTF-8','--']
 cases=[('unit',0,['python3','-m','unittest','discover','-s','tests','-p','test_c5*.py']),('replay',0,['python3','tools/integrated_replay.py','replay','--input','tests/fixtures/c5_replay_synthetic.json','--out',str(out/'replay')]),('demo',0,['python3','tools/integrated_replay.py','demo','--synthetic','--scenario','all','--out',str(out/'demo')]),('preflight',2,['python3','tools/integrated_replay.py','preflight','--out',str(out/'preflight')])]
 results=[]
 for name,expected,command in cases:
  cmd=['python3','tools/dev_run_log.py','--name','c5_'+name,'--operator','unspecified','--stage','C5','--hardware','none','--input-kind','synthetic','--log-root',str(out/'runs')]
  for src in sources:cmd+=['--source',src]
  proc=subprocess.run(cmd+['--']+sandbox+command,cwd=ROOT,text=True,capture_output=True)
  print(proc.stdout,end='');print(proc.stderr,end='',file=sys.stderr)
  ok=proc.returncode==expected
  if name=='unit' and ok:
   logs=list((out/'runs').glob('*_c5_unit/stderr.log'))
   ok=bool(logs and re.search(r'Ran [1-9][0-9]* tests?',logs[0].read_text()))
  results.append({'name':name,'exit_code':proc.returncode,'expected_exit':expected,'ok':ok})
 (out/'checks_summary.json').write_text(json.dumps({'cases':results,'ok':all(r['ok'] for r in results),'sandbox':'unshare-all; isolated devices; clear environment'},indent=2)+'\n')
 print('CHECK_SUMMARY '+str(out/'checks_summary.json'))
 return 0 if all(r['ok'] for r in results) else 1
if __name__=='__main__':sys.exit(main())
