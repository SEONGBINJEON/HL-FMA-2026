#!/usr/bin/env python3
"""Package data only; materialize internal links and verify every archived byte."""
import base64
import hashlib
import json
import os
from pathlib import Path
import time
from datetime import datetime
from zoneinfo import ZoneInfo
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'data'


def credential_patterns():
    # Values stay in memory; never report them or include the configuration.
    config = ROOT / 'config/ntrip.local.json'
    if not config.exists():
        return []
    obj = json.loads(config.read_text())
    patterns = set()
    def walk(value):
        if not isinstance(value, dict):
            return
        users, passwords = [], []
        for key, item in value.items():
            if isinstance(item, dict):
                walk(item)
            if not isinstance(item, str):
                continue
            key = key.lower()
            if any(k in key for k in ('user', 'password', 'passwd', 'secret', 'token')):
                if len(item) >= 6:
                    patterns.add(item.encode())
            if 'user' in key:
                users.append(item)
            if 'password' in key or 'passwd' in key:
                passwords.append(item)
        for user in users:
            for password in passwords:
                patterns.add(base64.b64encode(f'{user}:{password}'.encode()))
    walk(obj)
    return sorted(patterns)


def enumerate_files():
    items, links = [], []
    def visit(path, ancestors):
        real = path.resolve(strict=True)
        if not real.is_relative_to(DATA.resolve()):
            raise RuntimeError('Link outside data: ' + str(path.relative_to(ROOT)))
        if path.is_symlink():
            links.append({'path': path.relative_to(ROOT).as_posix(),
                          'target': os.readlink(path), 'export': 'regular copy'})
        if path.is_dir():
            if real in ancestors:
                raise RuntimeError('Cyclic link: ' + str(path))
            for child in sorted(path.iterdir()):
                visit(child, ancestors | {real})
        elif path.is_file():
            items.append((path, path.relative_to(ROOT).as_posix()))
        else:
            raise RuntimeError('Unsupported file: ' + str(path))
    visit(DATA, set())
    return items, links


def main():
    stamp = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')
    output = ROOT / 'exports' / f'HL_FMA_data_{stamp}'
    output.mkdir(parents=True, exist_ok=False)
    archive = output / f'HL_FMA_data_{stamp}.zip'
    partial = archive.with_suffix('.zip.partial')
    items, links = enumerate_files()
    patterns = credential_patterns()
    overlap = max([len(p) for p in patterns] or [1]) - 1
    manifest = []
    total = sum(p.stat().st_size for p, _ in items)
    done = 0
    last_print = time.monotonic()
    print(f'OUTPUT={output}\nFILES={len(items)} RAW_BYTES_WITH_LINK_COPIES={total}', flush=True)
    with zipfile.ZipFile(partial, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=1, allowZip64=True) as z:
        for path, name in items:
            before = path.stat()
            h = hashlib.sha256()
            tail = b''
            info = zipfile.ZipInfo.from_file(path, name)
            info.compress_type = zipfile.ZIP_DEFLATED
            info._compresslevel = 1
            with path.open('rb') as src, z.open(info, 'w', force_zip64=True) as dst:
                while chunk := src.read(1024 * 1024):
                    probe = tail + chunk
                    if any(p in probe for p in patterns):
                        raise RuntimeError('Credential match; do not share: ' + name)
                    tail = probe[-overlap:] if overlap else b''
                    h.update(chunk)
                    dst.write(chunk)
                    done += len(chunk)
                    if time.monotonic() - last_print >= 20:
                        print(f'PACKING {done/total:.0%}', flush=True)
                        last_print = time.monotonic()
            after = path.stat()
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise RuntimeError('Source changed during export: ' + name)
            manifest.append({'path': name, 'bytes': before.st_size, 'sha256': h.hexdigest()})
        info = {'created_kst': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
                'source': 'data only; config and external paths excluded',
                'links_materialized': links, 'files': manifest}
        z.writestr('data/SHARE_MANIFEST.json', json.dumps(info, ensure_ascii=False, indent=2))
    print('VERIFYING full archive CRC and SHA256', flush=True)
    with zipfile.ZipFile(partial) as z:
        assert len(z.infolist()) == len(manifest) + 1
        for entry in manifest:
            h = hashlib.sha256()
            with z.open(entry['path']) as src:
                while chunk := src.read(1024 * 1024):
                    h.update(chunk)
            if h.hexdigest() != entry['sha256']:
                raise RuntimeError('Archive mismatch: ' + entry['path'])
        json.loads(z.read('data/SHARE_MANIFEST.json'))
    partial.rename(archive)
    h = hashlib.sha256()
    with archive.open('rb') as src:
        while chunk := src.read(4 * 1024 * 1024):
            h.update(chunk)
    digest = h.hexdigest()
    (output / 'SHA256SUMS.txt').write_text(f'{digest}  {archive.name}\n')
    report = {'archive': str(archive.relative_to(ROOT)), 'created_kst': info['created_kst'],
              'source_files_with_link_copies': len(manifest), 'links_materialized': len(links),
              'uncompressed_bytes': total, 'archive_bytes': archive.stat().st_size,
              'archive_sha256': digest, 'all_crc_and_sha256_verified': True,
              'credential_match_scan': 'no matches to configured credential patterns',
              'scope': 'all data files; internal directory links copied for Windows compatibility'}
    (output / 'verification.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
