#!/usr/bin/env python3
"""Read flash/EEPROM of the explicitly identified Mega; never writes memory.

Opening the programmer can reset the board. Use only with motor power isolated.
The CLI requires --read and --motor-power-off; otherwise no device is opened.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
PORT = Path('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_95431353338351A01190-if00')
EXPECTED = ('2341', '0042', '95431353338351A01190')
AVR_DIR = ROOT / '.setup/arduino-cli/data/packages/arduino/tools/avrdude/8.0.0-arduino1'


def now():
    return datetime.now(timezone.utc).isoformat()


def identity():
    real = PORT.resolve(strict=True)
    if not stat.S_ISCHR(real.stat().st_mode):
        raise RuntimeError('Target is not a character device')
    device = (Path('/sys/class/tty') / real.name / 'device').resolve()
    for parent in (device, *device.parents):
        if (parent / 'idVendor').exists():
            got = tuple((parent / key).read_text().strip() for key in ('idVendor', 'idProduct', 'serial'))
            if got != EXPECTED:
                raise RuntimeError('Mega USB identity mismatch')
            return {'port': str(PORT), 'real_port': str(real), 'usb_identity': list(got), 'sysfs': str(parent)}
    raise RuntimeError('USB identity unavailable')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--read', action='store_true')
    parser.add_argument('--motor-power-off', action='store_true')
    args = parser.parse_args()
    if not (args.read and args.motor_power_off):
        parser.error('Explicit --read --motor-power-off required; no port opened')
    ident = identity()
    folder = ROOT / 'data' / (datetime.now(ZoneInfo('Asia/Seoul')).strftime('mega_backup_%Y%m%d_%H%M%S_') + os.urandom(4).hex())
    folder.mkdir()
    binary, config = AVR_DIR / 'bin/avrdude', AVR_DIR / 'etc/avrdude.conf'
    command = [str(binary), '-C', str(config), '-N', '-p', 'm2560', '-c', 'wiring',
               '-P', str(PORT), '-b', '115200', '-n', '-D', '-q',
               '-U', 'flash:r:' + str(folder / 'flash.bin') + ':r',
               '-U', 'eeprom:r:' + str(folder / 'eeprom.bin') + ':r']
    report = {'started_utc': now(), 'identity': ident, 'command': command, 'status': 'preparing',
              'operator_condition': 'Current-turn user confirmed motor power and vehicle battery OFF; keep isolated',
              'scope': 'Only read operations; -n disables memory writes, -D disables auto erase. Programmer may reset MCU.',
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'avrdude_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'avrdude_config_sha256': hashlib.sha256(config.read_bytes()).hexdigest()}
    (folder / 'executed_backup.py').write_bytes(Path(__file__).read_bytes())
    print('OUTPUT=' + str(folder), flush=True)
    fd = None
    code = 1
    try:
        fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if identity() != ident or os.fstat(fd).st_rdev != PORT.stat().st_rdev:
            raise RuntimeError('Mega identity changed while locking')
        report['status'] = 'reading'
        with (folder / 'avrdude.log').open('xb') as log:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, timeout=120, pass_fds=(fd,), check=False)
        report['returncode'] = process.returncode
        if process.returncode:
            raise RuntimeError('Memory read failed; inspect avrdude.log')
        report['files'] = {}
        for name, size in (('flash.bin', 262144), ('eeprom.bin', 4096)):
            raw = (folder / name).read_bytes()
            report['files'][name] = {'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest(),
                                    'all_ff': all(b == 255 for b in raw)}
            if len(raw) != size:
                raise RuntimeError('Unexpected memory read size: ' + name)
        report['status'] = 'completed'
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
        print(json.dumps({k: report[k] for k in ('status', 'files', 'error') if k in report}), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
