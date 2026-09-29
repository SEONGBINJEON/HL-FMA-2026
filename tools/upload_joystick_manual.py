#!/usr/bin/env python3
"""Upload the built type150 steering-diagnostic image, preserving verified current-board backup."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import subprocess
import shutil
from backup_mega import ROOT,PORT,AVR_DIR,identity

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--upload',action='store_true');ap.add_argument('--backup',type=Path,required=True);ap.add_argument('--conditions-file',type=Path,required=True)
    a=ap.parse_args()
    if not a.upload:ap.error('--upload required')
    backup=json.loads((a.backup/'metadata.json').read_text())
    if backup['status']!='completed':raise RuntimeError('Backup incomplete')
    for name in ('flash.bin','eeprom.bin'):
        if sha(a.backup/name)!=backup['files'][name]['sha256']:raise RuntimeError('Backup mismatch')
    conditions=json.loads(a.conditions_file.read_text())
    if not conditions.get('ground_trial_authorized') or not conditions.get('cutoff_available'):raise RuntimeError('Current operator conditions missing')
    source=ROOT/'firmware/joystick_manual';build=json.loads((source/'build_provenance.json').read_text())
    for name,h in build['source_sha256'].items():
        if sha(source/name)!=h:raise RuntimeError('Source changed after build: '+name)
    hexfile=source/'build_output/joystick_manual.ino.hex'
    if sha(hexfile)!=build['hex_sha256']:raise RuntimeError('Build image mismatch')
    ident=identity();folder=ROOT/'data'/('joystick_upload_'+datetime.now().strftime('%Y%m%d_%H%M%S'));folder.mkdir()
    for name in build['source_sha256']:shutil.copyfile(source/name,folder/name)
    shutil.copyfile(hexfile,folder/'uploaded.hex');shutil.copyfile(__file__,folder/'executed_upload.py')
    cmd=[str(AVR_DIR/'bin/avrdude'),'-C',str(AVR_DIR/'etc/avrdude.conf'),'-N','-p','m2560','-c','wiring','-P',str(PORT),'-b','115200','-D','-q','-U','flash:w:'+str(folder/'uploaded.hex')+':i']
    report={'started_utc':datetime.now(timezone.utc).isoformat(),'status':'uploading','identity':ident,'command':cmd,'backup':str(a.backup),'conditions':conditions,'build':build,'scope':'Flash image only; verification enabled. No fuse, bootloader or EEPROM upload. Firmware initializes/increments boot identity in EEPROM bytes16..27.'}
    (folder/'metadata.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    with (folder/'avrdude.log').open('xb') as f:r=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,timeout=90)
    report.update(status='completed' if r.returncode==0 else 'failed',returncode=r.returncode,ended_utc=datetime.now(timezone.utc).isoformat())
    (folder/'metadata.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(folder);print(report['status'])
    return r.returncode

if __name__=='__main__':raise SystemExit(main())
