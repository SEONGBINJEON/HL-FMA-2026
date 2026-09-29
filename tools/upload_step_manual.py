#!/usr/bin/env python3
"""Upload only the reviewed persistent manual image to the identified Mega, with verify.

Requires current motor power isolation and an attending cutoff operator.
"""
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
from zoneinfo import ZoneInfo
from backup_mega import ROOT, PORT, AVR_DIR, identity, now

REVIEWED_HEX_SHA256 = '0faeb1ceb8a0de7f88d074a2c00540f209cfc38c53b4df25bb0ea84245842c31'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upload', action='store_true')
    parser.add_argument('--motor-power-off', action='store_true')
    parser.add_argument('--wheels-raised', action='store_true')
    parser.add_argument('--spotter-present', action='store_true')
    args = parser.parse_args()
    if not (args.upload and args.motor_power_off and args.spotter_present):
        parser.error('Explicit --upload --motor-power-off --spotter-present required; no device opened')
    source_dir = ROOT / 'firmware/step_manual'
    provenance = json.loads((source_dir / 'build_provenance.json').read_text())
    for name in ('control_core.h', 'control_core.cpp', 'step_gate.h', 'step_manual.ino'):
        relative = 'firmware/step_manual/' + name
        if hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() != provenance['source_sha256'][relative]:
            raise RuntimeError('Source changed since reviewed build: ' + name)
    hex_file = source_dir / 'build_output/step_manual.ino.hex'
    if hashlib.sha256(hex_file.read_bytes()).hexdigest() != REVIEWED_HEX_SHA256:
        raise RuntimeError('HEX does not match the reviewed reviewed step manual image')
    backup = ROOT / 'data/mega_backup_20260905_214303_3fa92b14'
    original = json.loads((backup / 'metadata.json').read_text())
    if original['status'] != 'completed':
        raise RuntimeError('Original board backup incomplete')
    for name in ('flash.bin', 'eeprom.bin'):
        if hashlib.sha256((backup / name).read_bytes()).hexdigest() != original['files'][name]['sha256']:
            raise RuntimeError('Original backup checksum mismatch')
    ident = identity()
    folder = ROOT / 'data' / (datetime.now(ZoneInfo('Asia/Seoul')).strftime('step_manual_upload_%Y%m%d_%H%M%S_') + os.urandom(4).hex())
    folder.mkdir()
    for name in ('control_core.h', 'control_core.cpp', 'step_gate.h', 'step_manual.ino', 'build_provenance.json'):
        shutil.copyfile(source_dir / name, folder / name)
    shutil.copyfile(hex_file, folder / 'uploaded.hex')
    shutil.copyfile(__file__, folder / 'executed_upload.py')
    command = [str(AVR_DIR / 'bin/avrdude'), '-C', str(AVR_DIR / 'etc/avrdude.conf'), '-N',
               '-p', 'm2560', '-c', 'wiring', '-P', str(PORT), '-b', '115200', '-D', '-q',
               '-U', 'flash:w:' + str(folder / 'uploaded.hex') + ':i']
    report = {'started_utc': now(), 'identity': ident, 'command': command, 'status': 'preparing',
              'hex_sha256': REVIEWED_HEX_SHA256, 'original_backup': str(backup),
              'scope': 'REAL OUTPUT CAPABLE type144: one prepare persists across zero-key gaps. Both drive motors PWM110 while held and may run with steering PWM60. Steering is max1000ms/relativeADC100 per press; ADC rails block only outward steering and allow recovery, while drive continues. Each axis requires an observed zero before sign reversal. Core150ms communication lease remains. No physical power feedback or hardware watchdog. Avrdude verify enabled/no-V; no bootloader/fuse/EEPROM write commands.',
              'eeprom_note': 'The sketch initializes/increments its verified boot record in EEPROM bytes16..27. The damaged bytes0..11 and full current EEPROM were preserved before this field repair.',
              'operator_condition': 'Latest user explicitly confirmed drive power OFF after ground tests, large clear indoor floor and attending cutoff operator. Maintain OFF throughout upload and initial serial connection. Wheels are on ground unless separately indicated; battery physical disconnection not claimed.',
              'wheels_raised_asserted': args.wheels_raised}
    fd = None
    code = 1
    print('OUTPUT=' + str(folder), flush=True)
    try:
        fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if identity() != ident or os.fstat(fd).st_rdev != PORT.stat().st_rdev:
            raise RuntimeError('Mega identity changed while locking')
        with (folder / 'avrdude.log').open('xb') as log:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                     timeout=90, pass_fds=(fd,), check=False)
        report['returncode'] = process.returncode
        if process.returncode:
            raise RuntimeError('Upload or verification failed; inspect local log before any further action')
        report['status'] = 'uploaded_and_verified'
        code = 0
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        report.update(status='failed', error=str(exc))
    except KeyboardInterrupt:
        report['status'] = 'interrupted'
        code = 130
    finally:
        if fd is not None:
            os.close(fd)
        report['ended_utc'] = now()
        (folder / 'metadata.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k:report[k] for k in ('status', 'error') if k in report}), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
