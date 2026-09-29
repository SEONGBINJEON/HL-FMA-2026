#!/usr/bin/env python3
"""Verify a private restored workspace against its local backup manifest.

Reads regular files listed by the manifest; never follows a listed symlink.
Does not open devices, authenticate, repair, delete, or start the application.
"""
import hashlib
import json
import sys
from pathlib import Path


def main():
    manifest, root = Path(sys.argv[1]), Path(sys.argv[2]).resolve()
    records = json.loads(manifest.read_text())['files']
    failures = []
    for item in records:
        rel = Path(item['path'])
        if rel.is_absolute() or '..' in rel.parts:
            raise ValueError('Unsafe manifest path')
        p = root / rel
        if item['type'] == 'symlink':
            good = p.is_symlink() and str(p.readlink()) == item['target']
        else:
            good = (p.is_file() and not p.is_symlink() and p.resolve().is_relative_to(root)
                    and p.stat().st_size == item['bytes'])
            if good:
                h = hashlib.sha256()
                with p.open('rb') as f:
                    for b in iter(lambda: f.read(4 * 1024 * 1024), b''):
                        h.update(b)
                good = h.hexdigest() == item['sha256']
        if not good:
            failures.append(item['path'])
    print(json.dumps({'checked': len(records), 'failures': failures}, ensure_ascii=False))
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(main())
